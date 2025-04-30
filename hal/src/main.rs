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
mod term_cfg; // Terminal configuration
mod io_defs; // IO definitions
use env_logger::Env;
use crate::io_defs::*;
use crate::term_cfg::*;

const MAX_SUBDEVICES: usize = 16; /// Max no. of SubDevices that can be stored. This must be a power of 2 greater than 1.
const MAX_PDU_DATA: usize = PduStorage::element_size(1100); /// Max PDU data payload size - set this to the max PDI size or higher.
const MAX_FRAMES: usize = 16; /// Max no. of EtherCAT frames that can be in flight at any one time.
const PDI_LEN: usize = 64; /// Max total PDI length.
static PDU_STORAGE: PduStorage<MAX_FRAMES, MAX_PDU_DATA> = PduStorage::new();

fn main() {
    env_logger::Builder::from_env(Env::default().default_filter_or("info")).init();
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

    std::thread::Builder::new()
    .name("EthercatTxRxThread".to_owned())
    .spawn(move || {
        let runtime = smol::LocalExecutor::new();
        let _ = smol::block_on(runtime.run(async {
            ethercrab::std::tx_rx_task(&network_interface, tx, rx)
                .expect("spawn TX/RX task")
                .await
        }));
    })
    .expect("build TX/RX thread");

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

        if subdevice.name() == "BK1120" {
            let coupler_ctrl = subdevice.sdo_read::<u16>(0xf100, 01);
            let coupler_ctrl = coupler_ctrl.await.unwrap();
            log::info!("BK1120 CouplerCtrl: {:?}", coupler_ctrl);
        }
    }

    // Move from PRE-OP -> SAFE-OP -> OP
    let group = group.into_op(&maindevice).await.expect("PRE-OP -> OP"); // Should probably handle errors better

    let shutdown = Arc::new(AtomicBool::new(false)); // Handling Ctrl+C
    signal_hook::flag::register(signal_hook::consts::SIGINT, Arc::clone(&shutdown)).expect("Register hook");    

    // Enter the primary loop
    loop {
        if shutdown.load(Ordering::Relaxed) {
            log::info!("Shutting down...");
            break;
        }

        group.tx_rx(&maindevice).await.expect("TX/RX");

        { // use fn write() implemented by Setter trait
            let wr_guard = &mut *TERM_EL2889.write().expect("acquire EL3024 write lock");
            wr_guard.write(true, ChannelInput::Channel(TermChannel::Ch16)).unwrap();

            
            let wr_guard = &mut *TERM_KL2889.write().expect("acquire KL2889 write lock");
            for idx in 0..16 { // All 16 bits of KL2889
                wr_guard.write(true, ChannelInput::Index(idx)).unwrap();
            }

            let wr_guard = &mut *TERM_KL6581.write().expect("acquire KL6581 write lock");
            wr_guard.write(true, ChannelInput::Index(1)).unwrap(); // CB.1

        }

        // Physical Input Terminal --> Program Code Input Terminal Object
        for subdevice in group.iter(&maindevice) {
            let input = subdevice.inputs_raw();
            let input_bits = input.view_bits::<Lsb0>();
        
            if subdevice.name() == "EL1889" {
                el1889_handler(&*TERM_EL1889, input_bits);
            }

            if subdevice.name() == "EL3024" {
                el3024_handler(&*TERM_EL3024, input_bits, TermChannel::Ch1);
                el3024_handler(&*TERM_EL3024, input_bits, TermChannel::Ch3);
            }

            if subdevice.name() == "BK1120" {
                // View only KL6581 portion of the input process image (bytes 2-13)
                // indexing is by bit in here, not by byte
                kl6581_input_handler(&*TERM_KL6581, &input_bits[16..112]);
            }
        }

        // Program Code Output Terminal Object --> Physical Output Terminal
        for subdevice in group.iter(&maindevice) {
            let mut output = subdevice.outputs_raw_mut();
            let output_bits = output.view_bits_mut::<Lsb0>();

            if subdevice.name() == "EL2889" {
                el2889_handler(output_bits, &*TERM_EL2889);
            }
            if subdevice.name() == "BK1120" {
                // View only KL6581 portion of the output process image (bytes 2-13)
                // indexing is by bit in here, not by byte.
                kl6581_output_handler(&mut output_bits[16..112], &*TERM_KL6581);
                kl2889_handler(&mut output_bits[112..128], &*TERM_KL2889);

                // let cb = &mut output_bits[16..24];
                // cb.set(1, true);

                // let kl2889 = &mut output_bits[112..128]; // this works
                // kl2889.fill(true);
            }
        }

        { // use fn read() implemented by Getter trait
            let read_guard = &*TERM_EL1889.read().expect("Acquire TERM_EL1889 read guard");
            if read_guard.read(ChannelInput::Channel(TermChannel::Ch11)).unwrap() == ElectricalObservable::Simple(1) {
                log::info!("Limit switch hit");
            }
        }

        {
            let peek_kl6581 = group.subdevice(&maindevice, 4).expect("No BK1120 found as final subdevice");
            let peek_input = peek_kl6581.inputs_raw()[8]; // DB3
            let peek_bits = peek_input.view_bits::<Lsb0>();
            let subslice = &peek_bits[0..8];
            let value: u8 = subslice.load::<u8>();
            
            if value != 0 {
                log::info!(
                    "DB3 bytes: {:08b}",
                    value
                );
            }
        }

    }

    let group = group.into_safe_op(&maindevice).await.expect("OP -> SAFE-OP");
    log::info!("Commence shutdown: OP -> SAFE-OP");

    let group = group.into_pre_op(&maindevice).await.expect("SAFE-OP -> PRE-OP");
    log::info!("SAFE-OP -> PRE-OP");

    let _group = group.into_init(&maindevice).await.expect("PRE-OP -> INIT");
    log::info!("PRE-OP -> INIT, shutdown complete");

    Ok(())
}