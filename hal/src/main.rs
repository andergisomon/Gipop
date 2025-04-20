use ethercrab::{
    MainDevice, MainDeviceConfig, PduStorage, Timeouts, error::Error, std::ethercat_now, RetryBehaviour
};
use smol::{
    Task, // lock::RwLock
};
use async_executor::*;
use std::{
    sync::{
        Arc, atomic::{AtomicBool, Ordering}, RwLock, LazyLock
    },
    time::Duration,
};
use bitvec::prelude::*;
use anyhow::Result;
mod term_cfg;
mod io_defs;

const MAX_SUBDEVICES: usize = 16; /// Max no. of SubDevices that can be stored. This must be a power of 2 greater than 1.
const MAX_PDU_DATA: usize = PduStorage::element_size(1100); /// Max PDU data payload size - set this to the max PDI size or higher.
const MAX_FRAMES: usize = 16; /// Max no. of EtherCAT frames that can be in flight at any one time.
const PDI_LEN: usize = 64; /// Max total PDI length.
static PDU_STORAGE: PduStorage<MAX_FRAMES, MAX_PDU_DATA> = PduStorage::new();

fn main() {
    smol::block_on(entry_loop("enp3s0")).expect("Entry loop task");
    log::info!("Program terminated.");
}


pub async fn entry_loop(network_interface: &str) -> Result<(), anyhow::Error> {

    let network_interface = network_interface.to_string();
    
    let (tx, rx, pdu_loop) = PDU_STORAGE.try_split().expect("can only split once");

    let maindevice = Arc::new(MainDevice::new(
        pdu_loop,
        Timeouts { // BK coupler is a bit sluggish
            state_transition: Duration::from_millis(10_000), // Other values that seem to work: 5000, 15_000
            pdu: Duration::from_micros(30_000), // Can try 50_000
            eeprom: Duration::from_millis(10), // Can try 100
            wait_loop_delay: Duration::from_millis(2),
            mailbox_echo: Duration::from_millis(600), // Set to 100 in TwinCAT
            mailbox_response: Duration::from_millis(6000), // Set to 6000 in TwinCAT. Can try 25_000
        },
        MainDeviceConfig {retry_behaviour: RetryBehaviour::Count(10), ..Default::default()}
    ));
    
    std::thread::spawn(move || {
        let rt = tokio::runtime::Runtime::new().expect("Failed to create runtime");
        let _ = rt.block_on(async move { // tokio::runtime::Runtime block_on takes a Future, async turns the block into a Future, move all variables needed into the block for this task
            ethercrab::std::tx_rx_task(&network_interface, tx, rx)
                .expect("spawn TX/RX task")
                .await
        });
    });

    let group = maindevice
    .init_single_group::<MAX_SUBDEVICES, PDI_LEN>(ethercat_now)
    .await
    .expect("Init");

    log::info!("Discovered {} SubDevices", group.len());

    // This should probably be abstracted away by a helper function
    for subdevice in group.iter(&maindevice) {
        if matches!(subdevice.name(), "EL3004" | "EL3024") {
            log::info!("Found EL30{}4. Configuring...", subdevice.name().chars().nth(4).unwrap());

            subdevice.sdo_write(0x1c12, 0, 0u8).await?;
            subdevice
                .sdo_write_array(0x1c13, &[0x1a00u16, 0x1a02, 0x1a04, 0x1a06])
                .await?;
            subdevice.sdo_write(0x1c13, 0, 0x4u8).await?;
        }
    }

    // Move from PRE-OP -> SAFE-OP -> OP
    let group = group.into_op(&maindevice).await.expect("PRE-OP -> OP"); // Should probably handle errors better

    let shutdown = Arc::new(AtomicBool::new(false)); // Handling Ctrl+C
    signal_hook::flag::register(signal_hook::consts::SIGINT, Arc::clone(&shutdown)).expect("Register hook");

    // Enter the primary loop
    loop {
        /* Do stuff here:
        - From term inputs copy into memory
        - From memory copy into term outputs
        - other stuff
        */
        if shutdown.load(Ordering::Relaxed) {
            log::info!("Shutting down...");
            break;
        }

        group.tx_rx(&maindevice).await.expect("TX/RX");

        

    }

    let group = group.into_safe_op(&maindevice).await.expect("OP -> SAFE-OP");
    log::info!("Commence shutdown: OP -> SAFE-OP");

    let group = group.into_pre_op(&maindevice).await.expect("SAFE-OP -> PRE-OP");
    log::info!("SAFE-OP -> PRE-OP");

    let _group = group.into_init(&maindevice).await.expect("PRE-OP -> INIT");
    log::info!("PRE-OP -> INIT, shutdown complete");

    Ok(())
}

