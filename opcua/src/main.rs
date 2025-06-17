// Ship of Theseus'd from the simple server example
use log::warn;
use std::sync::{Arc, Mutex, LazyLock};
use tokio::task::LocalSet;

use anyhow::{anyhow, Result};

mod ipc;
use crate::ipc::*;
use iceoryx2::{port::{publisher, subscriber}, prelude::*};

use opcua::{server::{
    address_space::{AccessLevel, Variable, VariableBuilder}, diagnostics::NamespaceMetadata, node_manager::memory::{
        simple_node_manager, InMemoryNodeManager, SimpleNodeManager, SimpleNodeManagerImpl
    }, ServerBuilder, SubscriptionCache
}, types::TimestampsToReturn};

use opcua::types::{
    BuildInfo, DataValue, DateTime,
    NodeId, StatusCode, DataTypeId, NumericRange, Variant};

static SERVER_COPY: LazyLock<Mutex<IpcData>> = LazyLock::new(|| Mutex::new(IpcData {
    temperature: 0.0,
    humidity: 0.0,
    status: 0,
    area_1_lights: 0,
    area_2_lights: 0,
    area_1_lights_hmi_cmd: 0,
    modbus_ai_0: 0.0,
    modbus_di_0: 0,
}));

// Very important, call once before entering ctrl loop to initialize shared ipc types to default values
// shared ipc types must implement the PlacementDefault trait
fn opcua_init_ipc_to_plc(publisher: Arc<iceoryx2::port::publisher::Publisher<iceoryx2::prelude::ipc::Service, IpcDataToPlc, ()>>) -> Result<(), anyhow::Error> {
    let mut sample = publisher.loan_uninit()?;
    unsafe { IpcDataToPlc::placement_default(sample.payload_mut().as_mut_ptr()) };
    Ok(())
}

#[tokio::main]
async fn main() {
    env_logger::init();

    let ipc_rt = tokio::runtime::Builder::new_current_thread()
    .enable_all() // Enable I/O, time, etc., for the LocalSet's tasks
    .build()
    .expect("Failed to create current_thread runtime for LocalSet");

    let _ = std::thread::spawn(move || {
            ipc_rt.block_on(async move {
                let local = LocalSet::new();
                local.run_until(async move {
            
                    // Init IPC
                    let pub_node = NodeBuilder::new().create::<iceoryx2::prelude::ipc::Service>()?;
                    let pub_service = pub_node
                    .service_builder(&"ipc_to_plc".try_into()?)
                    .publish_subscribe::<IpcDataToPlc>()
                    .open_or_create()?;
            
                    let publisher = pub_service.publisher_builder().create()?;
                    let publisher: Arc<publisher::Publisher<iceoryx2::prelude::ipc::Service, IpcDataToPlc, ()>> = Arc::new(publisher);
                    opcua_init_ipc_to_plc(publisher.clone())?;
            
                    let sub_node = NodeBuilder::new().create::<iceoryx2::prelude::ipc::Service>()?;
                    let sub_service = sub_node
                    .service_builder(&"ipc_from_plc".try_into()?)
                    .publish_subscribe::<IpcDataFromPlc>()
                    .open_or_create()?;
            
                    let subscriber = sub_service.subscriber_builder().create()?;
                    let subscriber = Arc::new(subscriber);

                    let modbus_sub_node = NodeBuilder::new().create::<iceoryx2::prelude::ipc::Service>()?;
                    let modbus_sub_service = modbus_sub_node
                    .service_builder(&"modbus_ipc_tx".try_into()?)
                    .publish_subscribe::<ModbusIpcDataTx>()
                    .open_or_create()?;

                    let modbus_subscriber = modbus_sub_service.subscriber_builder().create()?;
                    let modbus_subscriber = Arc::new(modbus_subscriber);
            
                    loop {
                        {
                            let mut local = SERVER_COPY.lock().unwrap();
                            while let Some(sample) = subscriber.receive()? {
                                log::info!("[Via iceoryx2] temp: {}, humd: {}, stat: {}, ar1:{}, ar2:{}",
                                            sample.payload().temperature,
                                            sample.payload().humidity,
                                            sample.payload().status,
                                            sample.payload().area_1_lights,
                                            sample.payload().area_2_lights,
                                );
            
                                local.temperature   = sample.payload().temperature;
                                local.humidity      = sample.payload().humidity;
                                local.status        = sample.payload().status;
                                local.area_1_lights = sample.payload().area_1_lights;
                                local.area_2_lights = sample.payload().area_2_lights;
                            }

                            if subscriber.receive().unwrap().is_none() {
                                log::warn!("[ipc ecat] not getting anything!")
                            }

                            while let Some(sample) = modbus_subscriber.receive()? {
                                log::info!("[Modbus] AN0: {}, DI0: {}",
                                    sample.payload().modbus_ai_0,
                                    sample.payload().modbus_di_0,
                                );

                                local.modbus_ai_0 = sample.payload().modbus_ai_0;
                                local.modbus_di_0 = sample.payload().modbus_di_0;
                            }

                            if modbus_subscriber.receive().unwrap().is_none() {
                                log::warn!("[ipc modbus] not getting anything!")
                            }
                        }

                        {
                            let local = SERVER_COPY.lock().unwrap();

                            // ⚠️UB Warning!⚠️ Compiler cannot prove safety: Make sure opcua_init_ipc_to_plc() has been called
                            let sample = publisher.loan_uninit()?;
                            let mut sample = unsafe { sample.assume_init() };
                            let data = sample.payload_mut();

                            data.area_1_lights_hmi_cmd = local.area_1_lights_hmi_cmd;

                            // Send payload
                            sample.send()?;
                        }
                        
                        // Update done. 100ms window for read callbacks to acquire lock for reading.
                        std::thread::sleep(std::time::Duration::from_millis(100));
                    }
                    Ok::<(), anyhow::Error>(())
                }).await.expect("ipc task failed");
            })
        });

    let (server, handle) = ServerBuilder::new()
    .with_config_from("../server.conf")
    .build_info(BuildInfo {
        product_uri: "https://github.com/andergisomon/gipop".into(),
        manufacturer_name: "Pongipop Tohog Oundar Gipop".into(),
        product_name: "Gipop OPC-UA Server".into(),
        software_version: "0.1.0".into(),
        build_number: "1".into(),
        build_date: DateTime::now(),
    })
    .with_node_manager(simple_node_manager(
        NamespaceMetadata {
            namespace_uri: "urn:GipopPlcServer".to_owned(),
            ..Default::default()
        },
        "simple",
    ))
    .trust_client_certs(true)
    .diagnostics_enabled(true).build().unwrap();
    let node_manager = handle
        .node_managers()
        .get_of_type::<SimpleNodeManager>()
        .unwrap();
    let ns = handle.get_namespace_index("urn:GipopPlcServer").unwrap();

    add_plc_variables(ns, node_manager, handle.subscriptions().clone());

    let handle_c = handle.clone();
    tokio::spawn(async move {
        if let Err(e) = tokio::signal::ctrl_c().await {
            warn!("Failed to register CTRL-C handler: {e}");
            return;
        }
        handle_c.cancel();
    });
    
    log::info!("Server running");
    server.run().await.unwrap();
}

fn add_plc_variables(
    ns: u16,
    manager: Arc<InMemoryNodeManager<SimpleNodeManagerImpl>>,
    _subscriptions: Arc<SubscriptionCache>,
) {
    let temp_node = NodeId::new(ns, "temperature");
    let humd_node = NodeId::new(ns, "humidity");
    let stat_node = NodeId::new(ns, "status");
    let ar1_lights_node = NodeId::new(ns, "area 1 lights");
    let ar2_lights_node = NodeId::new(ns, "area 2 lights");
    let ar1_lights_hmi_cmd_node = NodeId::new(ns, "area 1 lights hmi cmd");
    // let plc_cycle_count = NodeId::new(ns, "plc cycle count");

    let address_space = manager.address_space();

    {
        let mut address_space = address_space.write();

        // Create a sample folder under objects folder
        let plc_folder_id = NodeId::new(ns, "plc_tags");
        address_space.add_folder(
            &plc_folder_id,
            "PlcTags", // browse_name
            "PlcTags", // display_name
            &NodeId::objects_folder_id(), // parent_node_id
        );

        // Add some variables to our folder
        let builder =
            VariableBuilder::new(&ar1_lights_hmi_cmd_node, "area 1 lights hmi cmd", "area 1 lights hmi cmd")
                .value(0_u32)
                .data_type(DataTypeId::UInt32)
                .historizing(false)
                .access_level(AccessLevel::all())
                .user_access_level(AccessLevel::all());
        let ar1_lights_hmi_cmd_node_var = builder.build();
        
        let _ = address_space.add_variables(
            vec![
                Variable::new(&temp_node, "temperature", "temperature", 0_f32),
                Variable::new(&humd_node, "humidity", "humidity", 0_f32),
                Variable::new(&stat_node, "status", "status", 0_u32),
                Variable::new(&ar1_lights_node, "area 1 lights", "area 1 lights", 0_u32),
                Variable::new(&ar2_lights_node, "area 2 lights", "area 2 lights", 0_u32),
                // Variable::new(&plc_cycle_count, "plc cycle count", "plc cycle count", 0_u64),
                ar1_lights_hmi_cmd_node_var,
            ],
            &plc_folder_id,
        );
        
    }

    {
        // Client write callback
        manager.inner().add_write_callback(
            ar1_lights_hmi_cmd_node.clone(),
            move |val: DataValue, _| {
                write_ar1_lights(val, &NumericRange::None)
            }
        );

        manager.inner().add_read_callback(
            temp_node.clone(),
            move |_, _, _| {
                Ok(DataValue::new_now(
                    fetch_temp()
                )
            )
        });
        manager.inner().add_read_callback(
            humd_node.clone(),
            move |_, _, _| {
                Ok(DataValue::new_now(
                    fetch_humd()
                )
            )
        });
        manager.inner().add_read_callback(stat_node.clone(),
        move |_, _, _| {
            Ok(DataValue::new_now(
                    fetch_status()
                )
            )
        });
        manager.inner().add_read_callback(ar1_lights_node.clone(),
        move |_, _, _| {
            Ok(DataValue::new_now(
                    fetch_ar1_lights()
                )
            )
        });
        manager.inner().add_read_callback(ar2_lights_node.clone(),
            move |_, _, _| {
                Ok(DataValue::new_now(
                    fetch_ar2_lights()
                )
            )
        });
        manager.inner().add_read_callback(ar1_lights_hmi_cmd_node.clone(),
            move |_, _, _| {
                Ok(DataValue::new_now(
                    fetch_ar1_lights_hmi_cmd()
                )
            )
        });
        // manager.inner().add_read_callback(plc_cycle_count.clone(),
        // move |_, _, _| {
        //     Ok(DataValue::new_now(
        //         fetch_cycle_count() // call fetcher function
        //     )
        // )
        // });
    }

}

fn fetch_temp() -> f32 {
    let local = SERVER_COPY.lock().unwrap();
    return local.temperature
}

fn fetch_humd() -> f32 {
    let local = SERVER_COPY.lock().unwrap();
    return local.humidity
}

fn fetch_status() -> u32 {
    let local = SERVER_COPY.lock().unwrap();
    return local.status
}

fn fetch_ar1_lights() -> u32 {
    let local = SERVER_COPY.lock().unwrap();
    return local.area_1_lights
}

fn fetch_ar2_lights() -> u32 {
    let local = SERVER_COPY.lock().unwrap();
    return local.area_2_lights
}

fn fetch_ar1_lights_hmi_cmd() -> u32 {
    let local = SERVER_COPY.lock().unwrap();
    return local.area_1_lights_hmi_cmd
}

fn write_ar1_lights(val: DataValue, _range: &NumericRange) -> StatusCode {
    let mut local = SERVER_COPY.lock().unwrap();
    match val.value {
        Some(Variant::UInt32(n)) => {
            local.area_1_lights_hmi_cmd = n;
            StatusCode::Good
        }
        other => {
            log::error!("Unexpected value type: {:?}", other);
            StatusCode::Bad
        }
    }
}

// fn fetch_cycle_count() -> u64 {

// }