use env_logger::Env;
use ethercrab::{
    MainDevice, MainDeviceConfig, PduStorage, Timeouts, error::Error, std::ethercat_now, RetryBehaviour
};
use std::{
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    time::Duration,
};
use tokio::time::MissedTickBehavior;
use bitvec::prelude::*;

/// Maximum number of SubDevices that can be stored. This must be a power of 2 greater than 1.
const MAX_SUBDEVICES: usize = 16;
/// Maximum PDU data payload size - set this to the max PDI size or higher.
const MAX_PDU_DATA: usize = PduStorage::element_size(1100);
/// Maximum number of EtherCAT frames that can be in flight at any one time.
const MAX_FRAMES: usize = 16;
/// Maximum total PDI length.
const PDI_LEN: usize = 64;

static PDU_STORAGE: PduStorage<MAX_FRAMES, MAX_PDU_DATA> = PduStorage::new();

#[tokio::main]
async fn main() -> Result<(), Error> {
    env_logger::Builder::from_env(Env::default().default_filter_or("info")).init();

    let interface = std::env::args()
        .nth(1)
        .expect("Provide network interface as first argument.");

    log::info!("Starting EK1100/EK1501 demo...");
    log::info!(
        "Ensure an EK1100 or EK1501 is the first SubDevice, with any number of modules connected after"
    );
    log::info!("Run with RUST_LOG=ethercrab=debug or =trace for debug information");

    let (tx, rx, pdu_loop) = PDU_STORAGE.try_split().expect("can only split once");

    let maindevice = Arc::new(MainDevice::new(
        pdu_loop,
        Timeouts { // BK is a bit sluggish
            state_transition: Duration::from_millis(10_000), // 5000 // 15_000 // 5000 seems to work just fine
            pdu: Duration::from_micros(30_000), // 30_000 // 50_000
            eeprom: Duration::from_millis(10), // 10 // 100
            wait_loop_delay: Duration::from_millis(2), // 2
            mailbox_echo: Duration::from_millis(600), // 100, 100 in TwinCAT / 5000
            mailbox_response: Duration::from_millis(6_000), // 1000, 6000 in TwinCAT // 25_000
        },
       MainDeviceConfig {dc_static_sync_iterations: 10_000, retry_behaviour: RetryBehaviour::Count(10)}
    ));

    // #[cfg(target_os = "windows")]
    // std::thread::spawn(move || {
    //     ethercrab::std::tx_rx_task_blocking(
    //         &interface,
    //         tx,
    //         rx,
    //         ethercrab::std::TxRxTaskConfig { spinloop: false },
    //     )
    //     .expect("TX/RX task")
    // });
    #[cfg(not(target_os = "windows"))]
    tokio::spawn(ethercrab::std::tx_rx_task(&interface, tx, rx).expect("spawn TX/RX task"));

    let group = maindevice
        .init_single_group::<MAX_SUBDEVICES, PDI_LEN>(ethercat_now)
        .await
        .expect("Init");

    log::info!("Discovered {} SubDevices", group.len());

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

    let group = group.into_op(&maindevice).await.expect("PRE-OP -> OP");

    for subdevice in group.iter(&maindevice) {
        let io = subdevice.io_raw();

        log::info!(
            "-> SubDevice {:#06x} {} inputs: {} bytes, outputs: {} bytes",
            subdevice.configured_address(),
            subdevice.name(),
            io.inputs().len(),
            io.outputs().len()
        );
    }

    let mut tick_interval = tokio::time::interval(Duration::from_millis(5));
    tick_interval.set_missed_tick_behavior(MissedTickBehavior::Skip);

    let shutdown = Arc::new(AtomicBool::new(false));
    signal_hook::flag::register(signal_hook::consts::SIGINT, Arc::clone(&shutdown))
        .expect("Register hook");

    loop {
        // Graceful shutdown on Ctrl + C
        if shutdown.load(Ordering::Relaxed) {
            log::info!("Shutting down...");

            break;
        }

        group.tx_rx(&maindevice).await.expect("TX/RX");

        // Increment every output byte for every SubDevice by one
        for subdevice in group.iter(&maindevice) {
            let mut o = subdevice.outputs_raw_mut();

            for byte in o.iter_mut() {
                *byte = byte.wrapping_add(1);
            }
        }

        let peek_el1889 = group.subdevice(&maindevice, 1).expect("No EL1889 found as first EK1100 terminal");
        let peek_input = peek_el1889.inputs_raw();
        let peek_bits = peek_input.view_bits::<Lsb0>();

        log::info!(
            "EL1889 Channel 13: {:?}",
            match peek_bits[12] {
                true => "Limit switch hit!",
                false => "",
            }
        );

        let peek_kl1889 = group.subdevice(&maindevice, 4).expect("No BK1120 found as final subdevice");
        let peek_input = peek_kl1889.inputs_raw()[15]; // Byte 14 is KL1889[0], Byte 15 is KL1889[1]; inputs_raw() poops out a &[u8], which bitvec and work with
        let peek_bits = peek_input.view_bits::<Lsb0>();

        log::info!(
            "KL1889 Channel 13: {:?}",
            match peek_bits[4] {
                true => "Limit switch hit!",
                false => "",
            }
        );

        tick_interval.tick().await;
    }

    let group = group
        .into_safe_op(&maindevice)
        .await
        .expect("OP -> SAFE-OP");

    log::info!("OP -> SAFE-OP");

    let group = group
        .into_pre_op(&maindevice)
        .await
        .expect("SAFE-OP -> PRE-OP");

    log::info!("SAFE-OP -> PRE-OP");

    let _group = group.into_init(&maindevice).await.expect("PRE-OP -> INIT");

    log::info!("PRE-OP -> INIT, shutdown complete");

    Ok(())
}