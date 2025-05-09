// OPCUA for Rust
// SPDX-License-Identifier: MPL-2.0
// Copyright (C) 2017-2024 Adam Lock
// Modified 2025 Ander Jiloh

use std::sync::atomic::{AtomicBool, AtomicI32, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use std::{fs::OpenOptions, path::Path};

use log::warn;
use opcua::server::address_space::{Variable, VariableBuilder, AccessLevel, NodeType};
use opcua::server::diagnostics::NamespaceMetadata;
use opcua::server::node_manager::memory::{
    simple_node_manager, InMemoryNodeManager, SimpleNodeManager, SimpleNodeManagerImpl,
};
use opcua::server::{ServerBuilder, SubscriptionCache};
use opcua::types::{BuildInfo, DataValue, DateTime, NodeId, UAString, StatusCode, DataTypeId, NumericRange, Variant, TimestampsToReturn};
mod shared;
use crate::shared::{SharedData, SHM_PATH, map_shared_memory, read_data, write_data};

#[tokio::main]
async fn main() {
    env_logger::init();
    // Open shared memory file. NOTE: The file is created by plc/main.rs
    // PLC must be running
    let file = OpenOptions::new().read(true).write(true).open(SHM_PATH).unwrap();
    let mut mmap = map_shared_memory(&file);

    let shared_data = Arc::new(Mutex::new(SharedData {
        temperature: 0.0,
        humidity: 0.0,
        status: 0,
        area_1_lights: 0,
        area_2_lights: 0,
        area_1_lights_hmi_cmd: 0,
    }));

    // spawn polling task
    let shared_data_clone = shared_data.clone();
    tokio::spawn(async move {
        loop {
            {
                let mut local = shared_data_clone.lock().unwrap();
                let data = read_data(&mmap);
                local.temperature = data.temperature;
                local.humidity = data.humidity;
                local.status = data.status;
                local.area_1_lights = data.area_1_lights;
                local.area_2_lights = data.area_2_lights;
                local.area_1_lights_hmi_cmd = data.area_1_lights_hmi_cmd;

                log::info!(
                    "[OPC UA sync] temp: {}, humd: {}, stat: {}, area1: {}, area2: {}, area1_cmd: {}",
                    local.temperature, local.humidity, local.status, local.area_1_lights, local.area_2_lights, local.area_1_lights_hmi_cmd
                );
            }
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    });

    // Create an OPC UA server with sample configuration and default node set
    let (server, handle) = ServerBuilder::new()
        .with_config_from("../server.conf")
        .build_info(BuildInfo {
            product_uri: "https://github.com/freeopcua/async-opcua".into(),
            manufacturer_name: "Pongipop Tohog Oundar Gipop".into(),
            product_name: "Gipop OPC-UA Server".into(),
            // Here you could use something to inject the build time, version, number at compile time
            software_version: "0.1.0".into(),
            build_number: "1".into(),
            build_date: DateTime::now(),
        })
        .with_node_manager(simple_node_manager(
            // Set the namespace for the node manager. For simple node managers this decides
            // node ownership, so make sure to use a different value here than the application URI
            // in server.conf, as that is the namespace used by the diagnostic node manager.
            NamespaceMetadata {
                namespace_uri: "urn:GipopPlcServer".to_owned(),
                ..Default::default()
            },
            "simple",
        ))
        .trust_client_certs(true)
        .diagnostics_enabled(true)
        .build()
        .unwrap();
    let node_manager = handle
        .node_managers()
        .get_of_type::<SimpleNodeManager>()
        .unwrap();
    let ns = handle.get_namespace_index("urn:GipopPlcServer").unwrap();

    // Add some variables of our own
    add_plc_variables(ns, node_manager, handle.subscriptions().clone());

    // If you don't register a ctrl-c handler, the server will close without
    // informing clients.
    let handle_c = handle.clone();
    tokio::spawn(async move {
        if let Err(e) = tokio::signal::ctrl_c().await {
            warn!("Failed to register CTRL-C handler: {e}");
            return;
        }
        handle_c.cancel();
    });
    
    log::info!("Server running");
    // Run the server. This does not ordinarily exit so you must Ctrl+C to terminate
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
                write_ar1_lights_to_shmem(val, &NumericRange::None)
            }
        );

        manager.inner().add_read_callback(
            temp_node.clone(),
            move |_, _, _| {
                Ok(DataValue::new_now(
                    fetch_temp_from_shmem() // call fetcher function
                )
            )
        });
        manager.inner().add_read_callback(
            humd_node.clone(),
            move |_, _, _| {
                Ok(DataValue::new_now(
                    fetch_humd_from_shmem()// call fetcher function
                )
            )
        });
        manager.inner().add_read_callback(stat_node.clone(),
        move |_, _, _| {
            Ok(DataValue::new_now(
                    fetch_status_from_shmem()// call fetcher function
                )
            )
        });
        manager.inner().add_read_callback(ar1_lights_node.clone(),
        move |_, _, _| {
            Ok(DataValue::new_now(
                    fetch_ar1_lights_from_shmem() // call fetcher function
                )
            )
        });
        manager.inner().add_read_callback(ar2_lights_node.clone(),
            move |_, _, _| {
                Ok(DataValue::new_now(
                    fetch_ar2_lights_from_shmem() // call fetcher function
                )
            )
        });
    }

}

fn fetch_temp_from_shmem() -> f32 {
    let file = OpenOptions::new().read(true).write(true).open(SHM_PATH).unwrap();
    let mut mmap = map_shared_memory(&file);
    let data = read_data(&mmap);
    return data.temperature
}

fn fetch_humd_from_shmem() -> f32 {
    let file = OpenOptions::new().read(true).write(true).open(SHM_PATH).unwrap();
    let mut mmap = map_shared_memory(&file);
    let data = read_data(&mmap);
    return data.humidity
}

fn fetch_status_from_shmem() -> u32 {
    let file = OpenOptions::new().read(true).write(true).open(SHM_PATH).unwrap();
    let mut mmap = map_shared_memory(&file);
    let data = read_data(&mmap);
    return data.status
}

fn fetch_ar1_lights_from_shmem() -> u32 {
    let file = OpenOptions::new().read(true).write(true).open(SHM_PATH).unwrap();
    let mut mmap = map_shared_memory(&file);
    let data = read_data(&mmap);
    return data.area_1_lights
}

fn fetch_ar2_lights_from_shmem() -> u32 {
    let file = OpenOptions::new().read(true).write(true).open(SHM_PATH).unwrap();
    let mut mmap = map_shared_memory(&file);
    let data = read_data(&mmap);
    return data.area_2_lights
}

fn write_ar1_lights_to_shmem(val: DataValue, _range: &NumericRange) -> StatusCode {
    let file = match OpenOptions::new().read(true).write(true).open(SHM_PATH) {
        Ok(f) => f,
        Err(e) => {
            log::error!("Failed to open shared memory file: {}", e);
            return StatusCode::Bad;
        }
    };

    let mut mmap = map_shared_memory(&file);
    let mut data = read_data(&mmap);

    match val.value {
        Some(Variant::UInt32(n)) => {
            log::info!("SERVER RECEIVED VALUE: {}", n);
            data.area_1_lights_hmi_cmd = n;
            write_data(&mut mmap, data);
            StatusCode::Good
        }
        other => {
            log::error!("Unexpected value type: {:?}", other);
            StatusCode::Bad
        }
    }
}
