// For getting read/write locks to terminal objects in PLC memory
use hal::io_defs::*;
use hal::term_cfg::*;
use hal::enocean_driver::*;
use std::sync::{Arc, RwLock, LazyLock};

// PLC (business logic) program is defined here via methods that read/write to/from terminal objects in PLC memory

pub struct Gvl {
    pub blinkerlamps: bool
}

impl Gvl {
    fn new() -> Self {
        Self {
            blinkerlamps: false
        }
    }
}

pub struct LocalPlcData {
    pub temperature: f32,
    pub humidity: f32,
    pub status: u32,
    pub area_1_lights: u32,
    pub area_2_lights: u32,
    pub area_1_lights_hmi_cmd: u32, // incoming to PLC
    pub modbus_ai_0: f32,
    pub modbus_di_0: u32,
}

// hmi cmd variables latch by default
struct LastCycle {
    area_1_lights_hmi_cmd: u32,
    blinkerlamps: bool,
}

impl LocalPlcData {
    pub fn new() -> Self {
        LocalPlcData {
            temperature: 0.0,
            humidity: 0.0,
            status: 0,
            area_1_lights: 0,
            area_2_lights: 0,
            area_1_lights_hmi_cmd: 0,
            modbus_ai_0: 0.0,
            modbus_di_0: 0,
        }
    }
}

pub static LOCAL_PLC_DATA: LazyLock<RwLock<LocalPlcData>> = LazyLock::new(|| RwLock::new(LocalPlcData::new()));
pub static GVL: LazyLock<RwLock<Gvl>> = LazyLock::new(|| RwLock::new(Gvl::new()));
static LAST_CYCLE: LazyLock<RwLock<LastCycle>> = LazyLock::new(|| RwLock::new(LastCycle { area_1_lights_hmi_cmd: 0, blinkerlamps: false }));

pub fn plc_execute_logic(term_states: Arc<RwLock<TermStates>>, counter: u64) {

    {
        enocean_sm(Arc::clone(&term_states));

        let cmd = LOCAL_PLC_DATA.read().unwrap();
        let mut last_cycle = LAST_CYCLE.write().unwrap();

        if cmd.area_1_lights_hmi_cmd == 2 && cmd.area_1_lights_hmi_cmd != last_cycle.area_1_lights_hmi_cmd {
            // log::info!("Area 1 Lights Command On");
            GVL.write().unwrap().blinkerlamps = true;
            // write_all_channel_kl2889(Arc::clone(&term_states), true);
            last_cycle.area_1_lights_hmi_cmd = cmd.area_1_lights_hmi_cmd; // Must be reset to avoid conflict with EnOcean
        }

        if cmd.area_1_lights_hmi_cmd == 1 && cmd.area_1_lights_hmi_cmd != last_cycle.area_1_lights_hmi_cmd {
            // log::info!("Area 1 Lights Command Off");
            GVL.write().unwrap().blinkerlamps = false;
            // write_all_channel_kl2889(Arc::clone(&term_states), false);
            last_cycle.area_1_lights_hmi_cmd = cmd.area_1_lights_hmi_cmd; // Must be reset to avoid conflict with EnOcean
        }

        if cmd.modbus_di_0 == 1 {
            write_all_channel_el2889(true, Arc::clone(&term_states));
        }

        let blink = GVL.read().unwrap().blinkerlamps;
        if blink {
            if counter % 5 == 0 {
                if read_area_1_lights(Arc::clone(&term_states)) == 1 {
                    write_all_channel_kl2889(Arc::clone(&term_states), false);
                }
                else {
                    write_all_channel_kl2889(Arc::clone(&term_states), true);   
                }
            }
        }
        else { // Avoid logical race condition: If blinkerlamps == false, always make sure output inactive
            write_all_channel_kl2889(Arc::clone(&term_states), false);
        }
    }
}

fn enocean_sm(term_states: Arc<RwLock<TermStates>>) {

    if check_sb_bit(Arc::clone(&term_states), 6) {
        log::error!(
            "{}",
            CnodeErrors::cnode_err_to_string(read_cnode(Arc::clone(&term_states)))
        );
    }
    else if check_sb_bit(Arc::clone(&term_states), 5) {
        log::error!("Config mismatch!");
    }
    else if check_sb_bit(Arc::clone(&term_states), 4) {
        log::error!("AddrConflict - Address of a KL6583 doubly assigned!");
    }
    else if check_sb_bit(Arc::clone(&term_states), 3) {
        log::error!("Communication Error - No KL6583 ready for op found. Check cabling and addresses");
    }
    else { // No errors
        let check_sb_1 = check_sb_bit(Arc::clone(&term_states), 1);
        let read_cb1 = read_cb1(Arc::clone(&term_states));
        let mut last_cycle = LAST_CYCLE.write().unwrap();

        // log::info!("CB1: {}, SB1: {}", read_cb1, check_sb_1);

        if read_cb1 != check_sb_1 {

            let db3 = read_db3(Arc::clone(&term_states));
            log::warn!("DB3: {:08b}", db3);

            if (db3 & 0b11110000) == 0b01010000 {
                log::info!("Rocker B, I pos. pressed");
                GVL.write().unwrap().blinkerlamps = true;
                last_cycle.blinkerlamps = true;
                // write_all_channel_kl2889(Arc::clone(&term_states), true);
            }

            if (db3 & 0b11110000) == 0b01110000 {
                log::info!("Rocker B, O pos. pressed");
                GVL.write().unwrap().blinkerlamps = false;
                // write_all_channel_kl2889(Arc::clone(&term_states), false);
            }

            if (db3 & 0b11110000) == 0b00010000 {
                log::info!("Rocker A, I pos. pressed");
                write_all_channel_el2889(true, Arc::clone(&term_states));
            }

            if (db3 & 0b11110000) == 0b00110000 {
                log::info!("Rocker A, 0 pos. pressed");
                write_all_channel_el2889(false, Arc::clone(&term_states));
            }
            let check_sb_1 = check_sb_bit(Arc::clone(&term_states), 1);

            write_cb1(Arc::clone(&term_states), check_sb_1); // Very important. Tells KL6581 we've fetched the packet.
        }
        else {
            let buffer_full = buffer_full(Arc::clone(&term_states));
            if buffer_full {
                log::info!("Buffer full");
                let check_sb_1 = check_sb_bit(Arc::clone(&term_states), 1);
                write_cb1(Arc::clone(&term_states), check_sb_1); // Very important. Tells KL6581 we've fetched the packet.
            }
        }
    }
}

pub fn read_area_1_lights(term_states: Arc<RwLock<TermStates>>) -> u8 {
    let rd_guard = term_states.read().expect("get term_states read guard");
    let kl2889 = rd_guard.kbus_terms[1].write().expect("acquire KL2889 dyn heap write lock");

    let reading = kl2889.read(Some(ChannelInput::Channel(TermChannel::Ch1))).unwrap();
    return reading.pick_simple().unwrap()
}

pub fn read_area_2_lights(term_states: Arc<RwLock<TermStates>>) -> u8 {
    let rd_guard = term_states.read().expect("get term_states read guard");
    let el2889 = rd_guard.ebus_do_terms[0].write().expect("acquire EL2889 dyn heap write lock");

    let reading = el2889.read(Some(ChannelInput::Channel(TermChannel::Ch1))).unwrap();
    return reading.pick_simple().unwrap()
}

fn write_all_channel_kl2889(term_states: Arc<RwLock<TermStates>>, val: bool) {
    let wr_guard = term_states.write().expect("get term_states write guard");
    let mut kl2889 = wr_guard.kbus_terms[1].write().expect("get KL2889 write guard");

    for idx in 0..kl2889.size_in_bits { // All 16 bits of KL2889
        kl2889.write(val, ChannelInput::Index(idx)).unwrap();
    }
}

fn write_all_channel_el2889(val: bool, term_states: Arc<RwLock<TermStates>>) {
    let wr_guard = term_states.read().expect("get term_states read guard");
    let mut el2889 = wr_guard.ebus_do_terms[0].write().expect("acquire EL2889 dyn heap write lock");

    for idx in 0..el2889.num_of_channels {
        el2889.write(val, ChannelInput::Index(idx)).unwrap();
    }
}