use ethercrab::{
    std::ethercat_now, MainDevice, MainDeviceConfig, PduStorage, RetryBehaviour, SubDeviceGroup, SubDeviceRef, Timeouts
};
use std::{
    num::Wrapping,
    fs::{File, OpenOptions}, sync::{atomic::{AtomicBool, Ordering}, Arc, RwLock, LazyLock, Mutex}, time::{Duration, Instant},
    io::Write
};
use bitvec::prelude::*;
use anyhow::Result;

// For getting read/write locks to terminal objects in PLC memory
use hal::io_defs::*;
use hal::term_cfg::*;
use crate::logic::*; // Business logic execution; Calls to methods to accomplish business logic
use crate::shared::{SHM_PATH, map_shared_memory, read_data, write_data};

#[derive(Debug, Clone, Copy, serde::Serialize)]
struct JitterSample {
    cycle: u64,
    jitter: i64, // signed to show early/late
}

pub static JITTER_BUF: LazyLock<Mutex<Vec<JitterSample>>> = LazyLock::new(|| Mutex::new(Vec::with_capacity(10_000)));

pub fn start_jitter_exporter() {
    smol::spawn(async {
        // Wait N seconds before exporting
        smol::Timer::after(Duration::from_secs(120)).await;

        let filename = "jitter_export.csv";
        let file = File::create(filename).expect("Unable to create CSV file");
        let mut writer = csv::Writer::from_writer(file);

        if let Ok(buf) = JITTER_BUF.lock() {
            for sample in buf.iter() {
                writer.serialize(sample).expect("Write failed");
            }
        }

        writer.flush().expect("Flush failed");
        println!("✅ Jitter data exported to {}", filename);
    }).detach();
}


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
            pdu: Duration::from_micros(20_000), // Can try 50_000
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
           
            guard.ebus_ai_terms
            .push(
                Arc::new(
                    RwLock::new(
                        AITerm::new(size as u8))));
        }
    }

    let shutdown = Arc::new(AtomicBool::new(false)); // Handling Ctrl+C
    signal_hook::flag::register(signal_hook::consts::SIGINT, Arc::clone(&shutdown)).expect("Register hook");

    let ts = term_states.clone();

    // PLC logic entry point. Cycle time watchdog should be here
    let mut counter: u64 = 0;
    let cycle = Duration::from_millis(10);
    let mut next_time = Instant::now() + cycle;
    let start_bench= Instant::now();

    smol::spawn(async move 
        {
            start_jitter_exporter();
            loop {
                let now = Instant::now();

                if now.duration_since(start_bench) == Duration::from_secs(120) {
                    break;
                }

                let jitter = now.duration_since(next_time);
                if let Ok(mut buf) = JITTER_BUF.try_lock() {
                    buf.push(JitterSample {
                        cycle: counter,
                        jitter: jitter.as_micros() as i64,
                    });
                }

                plc_execute_logic(ts.clone(), counter.clone());
                counter = counter.wrapping_add(1);
                next_time += cycle;
    
                let now = Instant::now();
                if next_time > now {
                    smol::Timer::at(next_time).await;
                } else {
                    log::warn!("Missed cycle deadline, skipping sleep");
                }
            }
        }
    ).detach();

    let ts = term_states.clone();
    // Shared memory
    smol::spawn(async move 
        {
            loop {
                smol::Timer::after(Duration::from_millis(100)).await; // We're not controlling servos :)
                opcua_shm(ts.clone());
            }
        }
    ).detach();

    // Enter the primary loop
    loop {
        if Arc::clone(&shutdown).load(Ordering::Relaxed) {
            log::info!("Shutting down...");
            break;
        }
        smol::Timer::after(Duration::from_millis(8)).await; // We're not controlling servos :)

        group.tx_rx(&maindevice).await.expect("TX/RX");

        // Physical Input Terminal --> Program Code Input Terminal Object
        for subdevice in group.iter(&maindevice) {
            let input = subdevice.inputs_raw();
            let input_bits = input.view_bits::<Lsb0>();
        
            if subdevice.name() == "EL1889" {

                {
                    let guard =
                    term_states.read().expect("get term_states read guard");

                    let mut guard = guard.ebus_di_terms[0].write()
                    .expect("get EL1889 from dyn heap read lock");

                    guard.refresh(input_bits);
                }
            }

            if subdevice.name() == "EL3024" {

                {
                    let guard =
                    term_states.read().expect("get term_states read guard");

                    let mut guard = guard.ebus_ai_terms[0].write()
                    .expect("get EL1889 from dyn heap read lock");

                    guard.refresh(input_bits);
                }
            }

            if subdevice.name() == "BK1120" {

                {
                    let guard =
                    term_states.read().expect("get term_states read guard");

                    // kbus_terms are indexed based on physical location from BK coupler
                    let mut kl1889 = guard.kbus_terms[0].write()
                    .expect("get BK1120/KL1889 from dyn heap read lock");

                    kl1889.refresh_ctrlr(Some(input_bits), None);
                }

                {
                    let guard =
                    term_states.read().expect("get term_states read guard");

                    // kbus_terms are indexed based on physical location from BK coupler
                    let mut kl6581 = guard.kbus_terms[2].write()
                    .expect("get BK1120/KL6581 from dyn heap read lock");
                    kl6581.refresh_ctrlr(Some(input_bits), None);
                }
            }
        }

        // Program Code Output Terminal Object --> Physical Output Terminal
        for subdevice in group.iter(&maindevice) {
            let mut output = subdevice.outputs_raw_mut();
            let output_bits = output.view_bits_mut::<Lsb0>();

            if subdevice.name() == "EL2889" {

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

                {
                    let guard = 
                    term_states.read().expect("get term_states read guard");

                    let kl2889 = guard.kbus_terms[1].read()
                    .expect("get BK1120/KL2889 from dyn heap read lock");

                    kl2889.refresh_term(output_bits);
                }

                {
                    let guard =
                    term_states.read().expect("get term_states read guard");

                    // kbus_terms are indexed based on physical location from BK coupler
                    let kl6581 = guard.kbus_terms[2].write()
                    .expect("get BK1120/KL6581 from dyn heap read lock");
                    kl6581.refresh_term(output_bits);
                }
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

fn opcua_shm(term_states: Arc<RwLock<TermStates>>) {
    let file = OpenOptions::new().read(true).write(true).open(SHM_PATH).unwrap();

    let mut mmap = map_shared_memory(&file);
    let mut data = read_data(&mmap);

    {
        // the reason for making a duplicate is so that the logic loop can fetch from LOCAL_PLC_DATA
        // instead of opening the shared mem file, which is dedicated for IPC between the ctrl_loop and the OPC UA server
        let mut plc_data = LOCAL_PLC_DATA.write().unwrap();

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
        let kl1889 = rd_guard.kbus_terms[0].read().expect("get KL1889 read guard");
        data.status = kl1889.read(Some(ChannelInput::Channel(TermChannel::Ch6))).unwrap().pick_simple().unwrap() as u32;

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
}