use ethercrab::{
    MainDevice, MainDeviceConfig, PduStorage, Timeouts, std::ethercat_now, RetryBehaviour
};
use async_io::Timer;
use memmap2::{Mmap, MmapMut};
use std::{
    sync::{Arc, Mutex, atomic::{AtomicBool, Ordering}, RwLock},
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

    // initialize terminal states
    let mut term_states = TermStates::new();
    
    for subdevice in group.iter(&maindevice) {
        if subdevice.name() == "EL2889" {
            let io = subdevice.io_raw();
            let size = 8*(io.inputs().len() + io.outputs().len());
            term_states.ebus_do_terms.
            push(
                Arc::new(
                    RwLock::new(
                        DOTerm::new(size as u8))));
        }

        if subdevice.name() == "EL1889" {
            let io = subdevice.io_raw();
            let size = 8*(io.inputs().len() + io.outputs().len());
            term_states.ebus_di_terms.
            push(
                Arc::new(
                    RwLock::new(
                        DITerm::new(size as u8))));
        }
    }

    let shutdown = Arc::new(AtomicBool::new(false)); // Handling Ctrl+C
    signal_hook::flag::register(signal_hook::consts::SIGINT, Arc::clone(&shutdown)).expect("Register hook");    

    std::thread::Builder::new()
    .name("PlcOpcUaServerShmThread".to_owned())
    .spawn(move || {
        let runtime = smol::LocalExecutor::new();
        smol::block_on(runtime.run(async move {
            loop {
                {
                    opcua_shm();
                }

                Timer::after(Duration::from_millis(100)).await;
            }
        }));
    })
    .expect("build shared mem thread");

    let peek_num_of_channels = term_states.ebus_di_terms[0].read().expect("get EL1889 from dyn heap read lock");
    log::info!("EL1889 in dyn heap: {}", peek_num_of_channels.num_of_channels);
    
    let peek_num_of_channels = term_states.ebus_do_terms[0].read().expect("get EL2889 from dyn heap read lock");
    log::info!("EL2889 in dyn heap: {}", peek_num_of_channels.num_of_channels);

    // Enter the primary loop
    loop {
        if shutdown.load(Ordering::Relaxed) {
            log::info!("Shutting down...");
            break;
        }

        group.tx_rx(&maindevice).await.expect("TX/RX");

        // PLC logic entry point. Cycle time watchdog should be here (TODO)
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

    }

    let group = group.into_safe_op(&maindevice).await.expect("OP -> SAFE-OP");
    log::info!("Commence shutdown: OP -> SAFE-OP");

    let group = group.into_pre_op(&maindevice).await.expect("SAFE-OP -> PRE-OP");
    log::info!("SAFE-OP -> PRE-OP");

    let _group = group.into_init(&maindevice).await.expect("PRE-OP -> INIT");
    log::info!("PRE-OP -> INIT, shutdown complete");

    Ok(())
}

fn opcua_shm() {
    let file = OpenOptions::new().read(true).write(true).open(SHM_PATH).unwrap();

    let mut mmap = map_shared_memory(&file);
    let mut data = read_data(&mmap);

    // the reason for making a duplicate is so that the logic loop can fetch from LOCAL_PLC_DATA
    // instead of opening the shared mem file, which is dedicated for IPC between the ctrl_loop and the OPC UA server
    let mut plc_data = LOCAL_PLC_DATA.lock().unwrap();

    let rd_guard = &*TERM_EL3024.read().expect("Acquire TERM_EL3024 read guard"); // calling read() twice in this scope will cause a freeze
    let ch2_reading = rd_guard.read(Some(ChannelInput::Channel(TermChannel::Ch2))).unwrap();
    let current = ch2_reading.pick_current().unwrap();
    let temp = ((current * 493.0)/1000.0 + 1.044) * 5.0; // offset can be calculated delta / 5.0
    plc_data.temperature = temp;
    data.temperature = temp;

    let ch1_reading = rd_guard.read(Some(ChannelInput::Channel(TermChannel::Ch1))).unwrap();
    let current = ch1_reading.pick_current().unwrap();
    let rh = ((current * 493.0)/1000.0 + 1.018) * 10.0; // offset can be calculated delta / 10.0
    plc_data.humidity = rh;
    data.humidity = rh;

    let rd_guard = &*TERM_KL1889.read().expect("Acquire TERM_KL1889 read guard");
    data.status = rd_guard.read(Some(ChannelInput::Channel(TermChannel::Ch7))).unwrap().pick_simple().unwrap() as u32;

    plc_data.area_1_lights = read_area_1_lights() as u32;
    data.area_1_lights = plc_data.area_1_lights;

    plc_data.area_2_lights = read_area_2_lights() as u32;
    data.area_2_lights = plc_data.area_2_lights;

    // Incoming to PLC: HMI command from shmem to local PLC state
    plc_data.area_1_lights_hmi_cmd = data.area_1_lights_hmi_cmd;
    write_data(&mut mmap, data);
}