use ethercrab::{
    MainDevice, MainDeviceConfig, PduStorage, Timeouts, std::ethercat_now, RetryBehaviour
};
use async_io::Timer;
use std::{
    sync::{Arc, Mutex, atomic::{AtomicBool, Ordering}},
    time::Duration,
    fs::OpenOptions
};
use bitvec::prelude::*;
use anyhow::Result;
use enum_iterator::all;

// For getting read/write locks to terminal objects in PLC memory
use hal::io_defs::*;
use hal::term_cfg::*;
use crate::logic::*; // Business logic execution; Calls to methods to accomplish business logic
use crate::shared::{SharedData, SHM_PATH, map_shared_memory, read_data, write_data};

const MAX_SUBDEVICES: usize = 16; /// Max no. of SubDevices that can be stored. This must be a power of 2 greater than 1.
const MAX_PDU_DATA: usize = PduStorage::element_size(1100); /// Max PDU data payload size - set this to the max PDI size or higher.
const MAX_FRAMES: usize = 16; /// Max no. of EtherCAT frames that can be in flight at any one time.
const PDI_LEN: usize = 64; /// Max total PDI length.
static PDU_STORAGE: PduStorage<MAX_FRAMES, MAX_PDU_DATA> = PduStorage::new();

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

    }

    // Move from PRE-OP -> SAFE-OP -> OP
    let group = group.into_op(&maindevice).await.expect("PRE-OP -> OP"); // Should probably handle errors better

    let shutdown = Arc::new(AtomicBool::new(false)); // Handling Ctrl+C
    signal_hook::flag::register(signal_hook::consts::SIGINT, Arc::clone(&shutdown)).expect("Register hook");    

    // outgoing fields of plc_data should be populated by values from terminal objects in PLC memory
    let plc_data = Arc::new(Mutex::new(SharedData {
        temperature: 0.0,
        humidity: 0.0,
        status: 0,
        area_1_lights: 0,
        area_2_lights: 0,
        area_1_lights_hmi_cmd: 0, // incoming to PLC
    }));

    let plc_data_for_thread = Arc::clone(&plc_data);    

    std::thread::Builder::new()
    .name("PlcOpcUaServerShmThread".to_owned())
    .spawn(move || {
        let file = OpenOptions::new().read(true).write(true).open(SHM_PATH).unwrap();
        let mut mmap = map_shared_memory(&file);
        let runtime = smol::LocalExecutor::new();
        smol::block_on(runtime.run(async move {
            loop {
                {
                    let mut data = read_data(&mmap);
                    let plc_data = plc_data_for_thread.lock().unwrap();
                    let mut cmd = INCOMING_HMI_CMD.lock().unwrap();

                    data.temperature = plc_data.temperature;

                    let rd_guard = &*TERM_KL1889.read().expect("Acquire TERM_KL1889 read guard");
                    data.status = rd_guard.read(Some(ChannelInput::Channel(TermChannel::Ch7))).unwrap().pick_simple().unwrap() as u32;

                    let rd_guard = &*TERM_EL3024.read().expect("Acquire TERM_EL3024 read guard");
                    let ch1_reading = rd_guard.read(Some(ChannelInput::Channel(TermChannel::Ch1))).unwrap();
                    let current = ch1_reading.pick_current().unwrap();
                    let rh = ((current * 493.0)/1000.0 + 0.96) * 10.0;
                    data.humidity = rh;

                    data.area_1_lights = read_area_1_lights() as u32;
                    data.area_2_lights = read_area_2_lights() as u32;

                    cmd.area_1_lights_hmi_cmd = data.area_1_lights_hmi_cmd; // Copy HMI command from shared mem to local PLC state

                    write_data(&mut mmap, data);
                }

                Timer::after(Duration::from_millis(100)).await;
            }
        }));
    })
    .expect("build shared mem thread");
    

    // Enter the primary loop
    loop {
        if shutdown.load(Ordering::Relaxed) {
            log::info!("Shutting down...");
            break;
        }

        group.tx_rx(&maindevice).await.expect("TX/RX");

        plc_execute_logic().await;

        // Physical Input Terminal --> Program Code Input Terminal Object
        for subdevice in group.iter(&maindevice) {
            let input = subdevice.inputs_raw();
            let input_bits = input.view_bits::<Lsb0>();
        
            if subdevice.name() == "EL1889" {
                el1889_handler(&*TERM_EL1889, input_bits);
            }

            if subdevice.name() == "EL3024" {
                for channel in all::<TermChannel>() {
                    if channel as u8 > EL3024_NUM_CHANNELS { break; }
                    el3024_handler(&*TERM_EL3024, input_bits, channel);
                }
            }

            if subdevice.name() == "BK1120" {
                // View only KL6581 portion of the input process image (bytes 2-13)
                // indexing is by bit in here, not by byte
                kl6581_input_handler(&*TERM_KL6581, &input_bits[16..112]);
                kl1889_handler(&*TERM_KL1889, &input_bits[112..128]);
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
            }
        }

        { // use fn read() implemented by Getter trait
            let rd_guard = &*TERM_EL1889.read().expect("Acquire TERM_EL1889 read guard");
            if rd_guard.read(Some(ChannelInput::Channel(TermChannel::Ch11))).unwrap() == ElectricalObservable::Simple(1) {
                log::info!("(EL1889) Limit switch hit");
            }

            let rd_guard = &*TERM_KL1889.read().expect("Acquire TERM_KL1889 read guard");
            if rd_guard.read(Some(ChannelInput::Channel(TermChannel::Ch7))).unwrap() == ElectricalObservable::Simple(1) {
                log::info!("(KL1889) Limit switch hit");
            }

            let rd_guard = &*TERM_EL3024.read().expect("Acquire TERM_EL3024 read guard");
            let reading = rd_guard.read(Some(ChannelInput::Channel(TermChannel::Ch1))).unwrap();
            let current = reading.pick_current().unwrap();
            // log::info!("Current Channel 1: {}", current);
            let rh = ((current * 493.0)/1000.0 + 0.96) * 10.0; // 0.96-0.97V offset because I have no idea how else to work with this hardware setup
            // log::info!("%RH: {}", rh);
            // smol::Timer::after(Duration::from_millis(50)).await;
        }

        { // use fn check() implemented by Checker trait
            let rd_guard = &*TERM_EL3024.read().expect("Acquire TERM_EL3024 read guard");
            let channel_status = rd_guard.check(ChannelInput::Channel(TermChannel::Ch1)).unwrap();
            // log::info!("EL3024 Ch1 Status: {}", channel_status.as_bitslice());
        }

        // {
        //     let peek_kl6581 = group.subdevice(&maindevice, 4).expect("No BK1120 found as final subdevice");
        //     let peek_input = peek_kl6581.inputs_raw()[8]; // DB3
        //     let peek_bits = peek_input.view_bits::<Lsb0>();
        //     let subslice = &peek_bits[0..8];
        //     let value: u8 = subslice.load::<u8>();
            
        //     if value != 0 {
        //         log::info!(
        //             "DB3 bytes direct: {:08b}",
        //             value
        //         );
        //     }
        // }

    }

    let group = group.into_safe_op(&maindevice).await.expect("OP -> SAFE-OP");
    log::info!("Commence shutdown: OP -> SAFE-OP");

    let group = group.into_pre_op(&maindevice).await.expect("SAFE-OP -> PRE-OP");
    log::info!("SAFE-OP -> PRE-OP");

    let _group = group.into_init(&maindevice).await.expect("PRE-OP -> INIT");
    log::info!("PRE-OP -> INIT, shutdown complete");

    Ok(())
}