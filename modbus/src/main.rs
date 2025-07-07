use env_logger::Env;
use std::sync::{Arc, Mutex, LazyLock};
mod ipc;
use crate::ipc::*;
use iceoryx2::{port::publisher, prelude::*};
use anyhow::Result;
use libc::*;

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

async fn client_app() -> Result<(), anyhow::Error> {
    use tokio_modbus::prelude::*;

    let socket_addr = "172.30.40.69:502".parse().unwrap();
    let mut ctx = tcp::connect(socket_addr).await?;

    loop {        
        {
            // let iriv_io_id = ctx.read_input_registers(0x0f00, 1).await??;
            let an_0 = ctx.read_input_registers(0x0200, 1).await??;
            let di_0 = ctx.read_discrete_inputs(0x0000, 1).await??;
            let mut val: u32 = 2;

            {
                let state = SERVER_COPY.lock();
    
                if let Ok(mut local) = state {
                    val = local.modbus_do_0;
                    local.modbus_ai_0 = f32::from(an_0[0] as u16) / 1000.0; // Analog voltage reading is natively in mV, analog current reading is natively in μA
                    local.modbus_di_0 = u32::from(di_0[0]);
                    log::info!("AI0: {:.03}, DI0: {}", local.modbus_ai_0, local.modbus_di_0);
                }
            }

            let val_b = match val {
                0 => Some(false),
                1 => Some(true),
                _ => None,
            };

            if let Some(val) = val_b {
                // ctx.write_single_coil(0x0100, val).await??;
                ctx.write_multiple_coils(0x0100, &[val; 4]).await??;
            }
        }

        std::thread::sleep(std::time::Duration::from_millis(50));
    }
    // TODO?: Handle disconnect
    // log::info!("Disconnecting");
    // ctx.disconnect().await?;
    Ok(())
}

fn main() {
    env_logger::Builder::from_env(Env::default().default_filter_or("info")).init();

    let res = unsafe {
        mlockall(libc::MCL_CURRENT | libc::MCL_FUTURE)
    };
    match res {
        0 => {
            log::info!("mlockall() returned 0");
        }
        _ => {
            log::error!("mlockall() failed, returned {}", res);
        }
    }

    let rt = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .unwrap();

        rt.block_on(async {

            let local = tokio::task::LocalSet::new();
            let modbus_client_handle = tokio::task::spawn(client_app());
            let ipc_handle = local.spawn_local(async move {

                // Init IPC
                let pub_node = NodeBuilder::new().create::<iceoryx2::prelude::ipc::Service>()?;
                let pub_service = pub_node
                .service_builder(&"modbus_ipc_tx".try_into()?)
                .publish_subscribe::<ModbusIpcDataTx>()
                .open_or_create()?;
        
                let publisher = pub_service.publisher_builder().create()?;
                let publisher: Arc<publisher::Publisher<iceoryx2::prelude::ipc::Service, ModbusIpcDataTx, ()>> = Arc::new(publisher);
                modbus_ipc_init(publisher.clone())?;
        
                let sub_node = NodeBuilder::new().create::<iceoryx2::prelude::ipc::Service>()?;
                let sub_service = sub_node
                .service_builder(&"modbus_ipc_rx".try_into()?)
                .publish_subscribe::<ModbusIpcDataRx>()
                .open_or_create()?;
        
                let subscriber = sub_service.subscriber_builder().create()?;
                let subscriber = Arc::new(subscriber);
        
                loop {
                    {
                        let mut local = SERVER_COPY.lock().unwrap();
                        while let Some(sample) = subscriber.receive()? {
                            log::info!("[Via iceoryx2] Modbus DO0: {}",
                                        sample.payload().modbus_do_0,
                            );
        
                            local.modbus_do_0   = sample.payload().modbus_do_0;
                        }
                        // if subscriber.receive().unwrap().is_none() {
                        //     log::warn!("not getting anything!")
                        // }
                    }

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
                    
                    std::thread::sleep(std::time::Duration::from_millis(22));
                }
                Ok::<(), anyhow::Error>(())
            });

            let _ = local.run_until(ipc_handle).await.expect("ipc error");
            let _ = modbus_client_handle.await.expect("modbus client error");

        }
    );
}