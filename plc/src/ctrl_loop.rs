use ethercrab::{
    std::ethercat_now, MainDevice, MainDeviceConfig, PduStorage, RetryBehaviour, SubDeviceGroup, SubDeviceRef, Timeouts
};
use async_io::Timer;
use memmap2::{Mmap, MmapMut};
use std::{
    fs::OpenOptions, ops::Deref, sync::{atomic::{AtomicBool, Ordering}, Arc, Mutex, RwLock}, time::Duration
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

pub async fn entry_loop(network_interface: &String) -> Result<(), anyhow::Error> {

    let network_interface = network_interface.to_string();
    
    let (tx, rx, pdu_loop) = PDU_STORAGE.try_split().expect("can only split once");

    let maindevice = Arc::new(MainDevice::new(
        pdu_loop,
        Timeouts { // BK coupler is a bit sluggish
            state_transition: Duration::from_millis(20_000), // Other values that seem to work: 5000, 15_000
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

    // initialize terminal states
    let term_states = init_term_states();

    for sd in group.iter(&maindevice) {
        if matches!(sd.name(), "EL3004" | "EL3024") {
            log::info!("Found EL30{}4. Configuring...", sd.name().chars().nth(4).unwrap());

            sd.sdo_write(0x1c12, 0, 0u8).await?;
            sd
                .sdo_write_array(0x1c13, &[0x1a00u16, 0x1a02, 0x1a04, 0x1a06])
                .await?;
            sd.sdo_write(0x1c13, 0, 0x4u8).await?;
        }

        // Configure K-bus terminals
        if sd.name() == "BK1120" {
            let num_of_terms: u8 = sd.sdo_read(0x4012, 0).await?;
            log::info!("Number of K-bus terminals detected: {}", num_of_terms-1);

            for term in 1..num_of_terms+1 {
                let term_name: u16 = sd.sdo_read(0x4012, term).await?;
                let ts = term_states.clone();
                parse_term(term_name, ts);
            }
            let ts = term_states.clone();
            set_slot_idx_range(ts);
        }

    }

    // Move from PRE-OP -> SAFE-OP -> OP
    let group = group.into_op(&maindevice).await.expect("PRE-OP -> OP"); // Should probably handle errors better

    for subdevice in group.iter(&maindevice) {
        // TODO: all of these if blocks contain repetitive code, should be abstracted away in a helper function
        if subdevice.name() == "EL2889" {
            let io = subdevice.io_raw();
            let size = 8*(io.inputs().len() + io.outputs().len());
            let guard = term_states.clone();
            let mut guard = guard.write().expect("get term_states write guard");

            guard.ebus_do_terms
            .push(
                Arc::new(
                    RwLock::new(
                        DOTerm::new(size as u8))));
        }

        if subdevice.name() == "EL1889" {
            let io = subdevice.io_raw();
            let size = 8*(io.inputs().len() + io.outputs().len());
            let guard = term_states.clone();
            let mut guard = guard.write().expect("get term_states write guard");
           
            guard.ebus_di_terms
            .push(
                Arc::new(
                    RwLock::new(
                        DITerm::new(size as u8))));
        }

        if subdevice.name() == "EL3024" {
            let io = subdevice.io_raw();
            let size = (io.inputs().len() + io.outputs().len()) / 4;
            let guard = term_states.clone();
            let mut guard = guard.write().expect("get term_states write guard");
            log::warn!("size of EL3024: {}", size);
           
            guard.ebus_ai_terms
            .push(
                Arc::new(
                    RwLock::new(
                        AITerm::new(size as u8))));
        }
    }

    let shutdown = Arc::new(AtomicBool::new(false)); // Handling Ctrl+C
    signal_hook::flag::register(signal_hook::consts::SIGINT, Arc::clone(&shutdown)).expect("Register hook");    

    let shm_ts_ref = term_states.clone();

    std::thread::Builder::new()
    .name("PlcOpcUaServerShmThread".to_owned())
    .spawn(move || {
        let runtime = smol::LocalExecutor::new();
        smol::block_on(runtime.run(async move {
            loop {
                {
                    opcua_shm(shm_ts_ref.clone());
                }

                Timer::after(Duration::from_millis(100)).await;
            }
        }));
    })
    .expect("build shared mem thread");

    {
        let peek_num_of_channels 
        = term_states.read()
        .expect("get term_states read guard");
        
        let peek_num_of_channels = peek_num_of_channels.ebus_di_terms[0].read()
        .expect("get EL1889 from dyn heap read lock");

        log::info!("EL1889 in dyn heap: {}", peek_num_of_channels.num_of_channels);
    }

    {
        let peek_num_of_channels 
        = term_states.read()
        .expect("get term_states read guard");
        
        let peek_num_of_channels = peek_num_of_channels.ebus_do_terms[0].read()
        .expect("get EL2889 from dyn heap read lock");

        log::info!("EL2889 in dyn heap: {}", peek_num_of_channels.num_of_channels);
    }

    // Enter the primary loop
    loop {
        if shutdown.load(Ordering::Relaxed) {
            log::info!("Shutting down...");
            break;
        }

        group.tx_rx(&maindevice).await.expect("TX/RX");

        // PLC logic entry point. Cycle time watchdog should be here (TODO)
        plc_execute_logic(term_states.clone()).await;

        {
            let peek_num_of_channels 
            = term_states.read()
            .expect("get term_states read guard");

            let peek_num_of_channels = peek_num_of_channels.ebus_di_terms[0].read()
            .expect("get EL1889 from dyn heap read lock");

            // log::info!("EL1889 in dyn heap value: {:b}", peek_num_of_channels.values);
        }

        {
            let peek_num_of_channels = term_states.read().expect("get term_states read guard");

            let peek_num_of_channels = peek_num_of_channels.ebus_ai_terms[0].read()
            .expect("get EL1889 from dyn heap read lock");

            let ch1_reading = peek_num_of_channels.read(Some(ChannelInput::Channel(TermChannel::Ch2))).unwrap();
            let current = ch1_reading.pick_current().unwrap();
            let humd = ((current * 493.0)/1000.0 + 1.022) * 5.0; // offset can be calculated delta / 5.0

            log::info!("EL3024 in dyn heap value: {}", humd);
        }

        // Physical Input Terminal --> Program Code Input Terminal Object
        for subdevice in group.iter(&maindevice) {
            let input = subdevice.inputs_raw();
            let input_bits = input.view_bits::<Lsb0>();
        
            if subdevice.name() == "EL1889" {
                el1889_handler(&*TERM_EL1889, input_bits); // TODO purge static allocation

                {
                    let guard =
                    term_states.read().expect("get term_states read guard");

                    let mut guard = guard.ebus_di_terms[0].write()
                    .expect("get EL1889 from dyn heap read lock");

                    guard.refresh(input_bits);
                }
            }

            if subdevice.name() == "EL3024" {
                for channel in all::<TermChannel>() {
                    if channel as u8 > EL3024_NUM_CHANNELS { break; }
                    el3024_handler(&*TERM_EL3024, input_bits, channel);
                }

                {
                    let guard =
                    term_states.read().expect("get term_states read guard");

                    let mut guard = guard.ebus_ai_terms[0].write()
                    .expect("get EL1889 from dyn heap read lock");

                    guard.refresh(input_bits);
                }
            }

            if subdevice.name() == "BK1120" {
                // View only KL6581 portion of the input process image (bytes 2-13)
                // indexing is by bit in here, not by byte
                kl6581_input_handler(&*TERM_KL6581, &input_bits[16..112]);
                // kl1889_handler(&*TERM_KL1889, &input_bits[112..128]);

                {
                    let guard =
                    term_states.read().expect("get term_states read guard");

                    // kbus_terms are indexed based on physical location from BK coupler
                    let mut guard = guard.kbus_terms[0].write()
                    .expect("get BK1120/KL1889 from dyn heap read lock");

                    guard.refresh_ctrlr(Some(input_bits), None);
                }
            }
        }

        // Program Code Output Terminal Object --> Physical Output Terminal
        for subdevice in group.iter(&maindevice) {
            let mut output = subdevice.outputs_raw_mut();
            let output_bits = output.view_bits_mut::<Lsb0>();

            if subdevice.name() == "EL2889" {
                el2889_handler(output_bits, &*TERM_EL2889); // TODO purge static allocation

                {
                    let guard = 
                    term_states.read().expect("get term_states read guard");

                    // kbus_terms are indexed based on physical location from BK coupler
                    let guard = guard.ebus_do_terms[0].read()
                    .expect("get EL2889 from dyn heap read lock");

                    guard.refresh(output_bits);
                }
            }
            if subdevice.name() == "BK1120" {
                // View only KL6581 portion of the output process image (bytes 2-13)
                // indexing is by bit in here, not by byte.
                kl6581_output_handler(&mut output_bits[16..112], &*TERM_KL6581);
                // kl2889_handler(&mut output_bits[112..128], &*TERM_KL2889);

                {
                    let guard = 
                    term_states.read().expect("get term_states read guard");

                    let guard = guard.kbus_terms[1].read()
                    .expect("get BK1120/KL2889 from dyn heap read lock");

                    guard.refresh_term(output_bits);
                }
            }
        }

        {
            let peek = term_states.read().expect("get term_states read guard");
            let peek = peek.kbus_terms[0].read().expect("get KL1889 from dyn heap read lock");

            let ch6_reading = peek.read(Some(ChannelInput::Channel(TermChannel::Ch6))).unwrap();
            let res = ch6_reading.pick_simple().unwrap();
            log::info!("KL1889 Channel 6 from dyn heap: {}", res)
        }

        {
            let peek = term_states.read().expect("get term_states read guard");
            let mut peek = peek.kbus_terms[1].write().expect("get KL1889 from dyn heap read lock");
            _ = peek.write(true, ChannelInput::Channel(TermChannel::Ch12));
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

fn opcua_shm(term_states: Arc<RwLock<TermStates>>) {
    let file = OpenOptions::new().read(true).write(true).open(SHM_PATH).unwrap();

    let mut mmap = map_shared_memory(&file);
    let mut data = read_data(&mmap);

    // the reason for making a duplicate is so that the logic loop can fetch from LOCAL_PLC_DATA
    // instead of opening the shared mem file, which is dedicated for IPC between the ctrl_loop and the OPC UA server
    let mut plc_data = LOCAL_PLC_DATA.lock().unwrap();

    {   
        let rd_guard = term_states.read().expect("Acquire TERM_EL3024 read guard"); // calling read() twice in this scope will cause a freeze
        let guard = rd_guard.ebus_ai_terms[0].read().unwrap();
        let ch2_reading = guard.read(Some(ChannelInput::Channel(TermChannel::Ch2))).unwrap();
        let current = ch2_reading.pick_current().unwrap();
        let temp = ((current * 493.0)/1000.0 + 1.044) * 5.0; // offset can be calculated delta / 5.0
        plc_data.temperature = temp;
        data.temperature = temp;

        let ch1_reading = guard.read(Some(ChannelInput::Channel(TermChannel::Ch1))).unwrap();
        let current = ch1_reading.pick_current().unwrap();
        let rh = ((current * 493.0)/1000.0 + 1.018) * 10.0; // offset can be calculated delta / 10.0
        plc_data.humidity = rh;
        data.humidity = rh;
    }

    let ts_status = term_states.clone();
    let rd_guard = ts_status.read().expect("get term_states read guard");
    let rd_guard = rd_guard.kbus_terms[0].read().expect("get KL1889 read guard");
    data.status = rd_guard.read(Some(ChannelInput::Channel(TermChannel::Ch6))).unwrap().pick_simple().unwrap() as u32;

    let ts_1 = term_states.clone();
    let ts_2 = ts_1.clone();

    plc_data.area_1_lights = read_area_1_lights(ts_1) as u32;
    data.area_1_lights = plc_data.area_1_lights;

    plc_data.area_2_lights = read_area_2_lights(ts_2) as u32;
    data.area_2_lights = plc_data.area_2_lights;

    // Incoming to PLC: HMI command from shmem to local PLC state
    plc_data.area_1_lights_hmi_cmd = data.area_1_lights_hmi_cmd;
    write_data(&mut mmap, data);
}

/// Parses K-bus terminals and pushes them into the heap, but with `slot_idx_range` initialized to (0, 0)
fn parse_term(term_name: u16, term_states: Arc<RwLock<TermStates>>) {
    let guard = term_states.clone();
    let mut guard = guard.write().expect("get term_states write guard");

    log::warn!("K-bus term name: {}", term_name);

    // KL6581 is guaranteed Intelligent
    if term_name == 6581 {
        guard.kbus_terms
        .push(
            Arc::new(
                RwLock::new(
                    KBusTerm::new(
                        term_name,
                        true,
                        192,
                        KBusTerminalGender::Enby,
                        (0, 0)
                ))));
    }

    let term_name_bits: BitVec<u16, Lsb0> = BitVec::from_element(term_name as u16);

    // If Simple Terminal
    if term_name_bits[15] {
        let size_in_bits: u8 = term_name_bits[7..15].load_le();
        log::warn!("K-bus term size in bits: {}", size_in_bits);

        // If Input Terminal
        if term_name_bits[0] && !term_name_bits[1] { 
            guard.kbus_terms
            .push(
                Arc::new(
                    RwLock::new(
                        KBusTerm::new(
                            term_name,
                            false,
                            size_in_bits / 2,
                            KBusTerminalGender::Input,
                            (0, 0)
                ))));
        }

        // If Output Terminal
        if !term_name_bits[0] && term_name_bits[1] { 
            guard.kbus_terms
            .push(
                Arc::new(
                    RwLock::new(
                        KBusTerm::new(
                            term_name,
                            false,
                            size_in_bits / 2,
                            KBusTerminalGender::Output,
                            (0, 0)
                ))));
        }
    }

    log::warn!("Total K-bus terminals parsed: {}", guard.kbus_terms.len());

}

// Determine and set the correct `slot_idx_range` occupied by each K-bus terminal in the BK coupler input/output images
fn set_slot_idx_range(term_states: Arc<RwLock<TermStates>>) {
    let guard = term_states.clone();
    let guard = guard.write().expect("get term_states write guard");
    let terms = &guard.kbus_terms;

    // This implementation is incomplete. It does not cover the following cases:
    // - Multiple instances of the same terminal
    // - Non-contiguous terminal layout (from mixed Simple and Terminal physical layout -> cluster Simple/Terminal separately in memory).
    // TODO: KBusTerm (any terminal instance, really) should have a UID
    for (_pos, term) in terms.iter().enumerate() {
        let mut term_lock = term.write().expect("get K-bus term write guard");

        // setting slot index ranges should be conditioned on UID instead of non-unique attributes like name and gender
        if term_lock.name == 6581 {
            assert!(term_lock.intelligent && term_lock.name == 6581); // Panic if KL6581 is for some reason not Intelligent
            term_lock.slot_idx_range = (16, 15+(12*8));
        }

        if term_lock.gender == KBusTerminalGender::Input {
            term_lock.slot_idx_range = (112, 112+15);
        }

        if term_lock.gender == KBusTerminalGender::Output {
            term_lock.slot_idx_range = (112, 112+15);
        }

    }
}