use std::sync::{Arc, Mutex, LazyLock};
mod ipc;
use crate::ipc::*;
use iceoryx2::{port::{publisher, subscriber}, prelude::*};
use anyhow::{anyhow, Result};

static SERVER_COPY: LazyLock<Mutex<IpcData>> = LazyLock::new(|| Mutex::new(IpcData {
    modbus_ai_0: 0.0,
    modbus_di_0: 0,
    modbus_do_0: 0,
}));

// Very important, call once before entering scan loop to initialize shared ipc types to default values
// shared ipc types must implement the PlacementDefault trait
fn modbus_ipc_init(publisher: Arc<iceoryx2::port::publisher::Publisher<iceoryx2::prelude::ipc::Service, ModbusIpcDataTx, ()>>) -> Result<(), anyhow::Error> {
    let mut sample = publisher.loan_uninit()?;
    unsafe { ModbusIpcDataTx::placement_default(sample.payload_mut().as_mut_ptr()) };
    Ok(())
}

async fn client_app() -> Result<(), Box<dyn std::error::Error>> {
    use tokio_modbus::prelude::*;

    let socket_addr = "172.30.40.69:502".parse().unwrap();
    let mut ctx = tcp::connect(socket_addr).await?;

    loop {        
        {
            // let iriv_io_id = ctx.read_input_registers(0x0f00, 1).await??;
            let an_0 = ctx.read_input_registers(0x0200, 1).await??;
            let di_0 = ctx.read_discrete_inputs(0x0000, 1).await??;

            let mut local = SERVER_COPY.lock().unwrap();

            local.modbus_ai_0 = f32::from(an_0[0] as u16);
            local.modbus_di_0 = u32::from(di_0[0]);
        }

        std::thread::sleep(std::time::Duration::from_millis(22));
    }
    // println!("Disconnecting");
    // ctx.disconnect().await?;
    Ok(())
}

fn main() {
    tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .unwrap()
        .block_on(async {

            let local = tokio::task::LocalSet::new();
            local.run_until(async move {
        
                // Init IPC
                let pub_node = NodeBuilder::new().create::<iceoryx2::prelude::ipc::Service>()?;
                let pub_service = pub_node
                .service_builder(&"modbus_ipc_tx".try_into()?)
                .publish_subscribe::<ModbusIpcDataTx>()
                .open_or_create()?;
        
                let publisher = pub_service.publisher_builder().create()?;
                let publisher: Arc<publisher::Publisher<iceoryx2::prelude::ipc::Service, ModbusIpcDataTx, ()>> = Arc::new(publisher);
                modbus_ipc_init(publisher.clone())?;
        
                // let sub_node = NodeBuilder::new().create::<iceoryx2::prelude::ipc::Service>()?;
                // let sub_service = sub_node
                // .service_builder(&"modbus_ipc_from_logic".try_into()?)
                // .publish_subscribe::<ModbusIpcDataFromLogic>()
                // .open_or_create()?;
        
                // let subscriber = sub_service.subscriber_builder().create()?;
                // let subscriber = Arc::new(subscriber);
        
                loop {
                    // {
                    //     let mut local = SERVER_COPY.lock().unwrap();
                    //     while let Some(sample) = subscriber.receive()? {
                    //         log::info!("[Via iceoryx2] temp: {}, humd: {}, stat: {}, ar1:{}, ar2:{}",
                    //                     sample.payload().temperature,
                    //                     sample.payload().humidity,
                    //                     sample.payload().status,
                    //                     sample.payload().area_1_lights,
                    //                     sample.payload().area_2_lights,
                    //         );
        
                    //         local.temperature   = sample.payload().temperature;
                    //         local.humidity      = sample.payload().humidity;
                    //         local.status        = sample.payload().status;
                    //         local.area_1_lights = sample.payload().area_1_lights;
                    //         local.area_2_lights = sample.payload().area_2_lights;
                    //     }
                    //     if subscriber.receive().unwrap().is_none() {
                    //         log::warn!("not getting anything!")
                    //     }
                    // }

                    {
                        let local = SERVER_COPY.lock().unwrap();

                        // ⚠️UB Warning!⚠️ Compiler cannot prove safety: Make sure modbus_ipc_init() has been called
                        let sample = publisher.loan_uninit()?;
                        let mut sample = unsafe { sample.assume_init() };
                        let data = sample.payload_mut();

                        data.modbus_ai_0 = local.modbus_ai_0;
                        data.modbus_di_0 = local.modbus_di_0;

                        // Send payload
                        sample.send()?;
                    }
                    
                    std::thread::sleep(std::time::Duration::from_millis(5));
                }
                Ok::<(), anyhow::Error>(())
            }).await.expect("ipc task failed");
        }
    );

    tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .unwrap()
        .block_on(async {
            client_app().await.expect("client app error")
        })
}