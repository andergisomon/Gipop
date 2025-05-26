use bitvec::prelude::*;
// For getting read/write locks to terminal objects in PLC memory
use hal::io_defs::*;
use hal::term_cfg::*;
use std::sync::{Arc, RwLock, LazyLock, Mutex};
use std::fs::OpenOptions;
use std::time::Duration;
use crate::shared::{SharedData, SHM_PATH, map_shared_memory, read_data, write_data};

// PLC (business logic) program is defined here via methods that read/write to/from terminal objects in PLC memory

pub struct LocalPlcData {
    pub temperature: f32,
    pub humidity: f32,
    pub status: u32,
    pub area_1_lights: u32,
    pub area_2_lights: u32,
    pub area_1_lights_hmi_cmd: u32, // incoming to PLC
}

impl LocalPlcData {
    pub fn new() -> Self {
        LocalPlcData {
            temperature: 0.0,
            humidity: 0.0,
            status: 0,
            area_1_lights: 0,
            area_2_lights: 0,
            area_1_lights_hmi_cmd: 0
        }
    }
}

pub static LOCAL_PLC_DATA: LazyLock<RwLock<LocalPlcData>> = LazyLock::new(|| RwLock::new(LocalPlcData::new()));

pub fn plc_execute_logic(term_states: Arc<RwLock<TermStates>>) {

    {
        enocean_sm(Arc::clone(&term_states));

        let cmd = LOCAL_PLC_DATA.read().unwrap();

        if cmd.area_1_lights_hmi_cmd == 2 {
            // log::info!("Area 1 Lights Command On");
            write_all_channel_kl2889(Arc::clone(&term_states), true);
            reset_hmi_cmd(); // Must be reset to avoid conflict with EnOcean
        }

        if cmd.area_1_lights_hmi_cmd == 1 {
            // log::info!("Area 1 Lights Command Off");
            write_all_channel_kl2889(Arc::clone(&term_states), false);
            reset_hmi_cmd(); // Must be reset to avoid conflict with EnOcean
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

        // log::info!("CB1: {}, SB1: {}", read_cb1, check_sb_1);

        if read_cb1 != check_sb_1 {

            let db3 = read_db3(Arc::clone(&term_states));
            log::warn!("DB3: {:08b}", db3);

            if (db3 & 0b11110000) == 0b01010000 {
                log::info!("Rocker B, I pos. pressed");
                write_all_channel_kl2889(Arc::clone(&term_states), true);
            }

            if (db3 & 0b11110000) == 0b01110000 {
                log::info!("Rocker B, O pos. pressed");
                write_all_channel_kl2889(Arc::clone(&term_states), false);
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
            // log::info!("CB.1 == SB.1");
            let buffer_full = buffer_full(Arc::clone(&term_states));
            if buffer_full {
                log::info!("Buffer full");
                let check_sb_1 = check_sb_bit(Arc::clone(&term_states), 1);
                write_cb1(Arc::clone(&term_states), check_sb_1); // Very important. Tells KL6581 we've fetched the packet.
            }
        }
    }
}

fn read_cnode(term_states: Arc<RwLock<TermStates>>) -> BitVec<u8, Lsb0> {
    let rd_guard = term_states.write().expect("get term_states write guard");
    let rd_guard = rd_guard.kbus_terms[2].write().expect("get KL6581 write guard");
    let reading = rd_guard.read(None).unwrap();
    let value: BitVec<u8, Lsb0> = reading.pick_smart().unwrap(); // 192 bits = 24 bytes
    let bits: &BitSlice<u8, Lsb0> = value.as_bitslice();
    return BitVec::from_bitslice(&bits[8..16]);
}

#[repr(u8)]
enum CnodeErrors { // variant names follow the KL6581 manual from Beckhoff, with the exception of the obvious 'KL6853` typo
    WatchdogError     = 0x10,
    NoComWithKL6581   = 0x11,
    idx_number_not_OK = 0x12,
    Switch_to_Stopp   = 0x13,
    not_ready         = 0x14,
    No_KL6583_Found   = 0x15,
    TransmissionError = 0x16,
}

impl CnodeErrors {
    fn cnode_err_from_u8(value: u8) -> Result<Self, String> {
        match value {
            0x10 => Ok(CnodeErrors::WatchdogError),
            0x11 => Ok(CnodeErrors::NoComWithKL6581),
            0x12 => Ok(CnodeErrors::idx_number_not_OK),
            0x13 => Ok(CnodeErrors::Switch_to_Stopp),
            0x14 => Ok(CnodeErrors::not_ready),
            0x15 => Ok(CnodeErrors::No_KL6583_Found),
            0x16 => Ok(CnodeErrors::TransmissionError),
            _ => Err("Invalid CNODE byte value".into()),
        }
    }

    // To be used with read_cnode()
    fn cnode_err_to_string(cnode: BitVec<u8, Lsb0>) -> String {
        let cnode: u8 = cnode.load_le();
    
        let err_message = match CnodeErrors::cnode_err_from_u8(cnode) {
            Ok(CnodeErrors::WatchdogError)     => "The KL6581 does not answer anymore. Check the mapping and communication.",
            Ok(CnodeErrors::NoComWithKL6581)   => "The KL6581 does not answer. Check the mapping and communication.",
            Ok(CnodeErrors::idx_number_not_OK) => "nIdx is not correct. nIdx may have a value from 0 to 64.",
            Ok(CnodeErrors::Switch_to_Stopp)   => "bInit is FALSE. Set bInit back to TRUE.",
            Ok(CnodeErrors::not_ready)         => "The terminal is not in data exchange. Check the mapping and communication.",
            Ok(CnodeErrors::No_KL6583_Found)   => "There is no KL6583 connected. Check the wiring to the KL6583.",
            Ok(CnodeErrors::TransmissionError) => "The KL6581 does not answer anymore. Check the mapping and communication.",
            _ => "Invalid CNODE byte value",
        };
        return err_message.to_string()
    }
}

fn read_cb1(term_states: Arc<RwLock<TermStates>>) -> bool {
    let rd_guard = term_states.write().expect("get term_states write guard");
    let rd_guard = rd_guard.kbus_terms[2].write().expect("get KL6581 write guard");
    let reading = rd_guard.read(None).unwrap();
    let value: BitVec<u8, Lsb0> = reading.pick_smart().unwrap(); // 192 bits = 24 bytes
    let bits: &BitSlice<u8, Lsb0> = value.as_bitslice();
    return bits[1];
}

pub fn read_db3(term_states: Arc<RwLock<TermStates>>) -> u8 {
    let rd_guard = term_states.write().expect("get term_states write guard");
    let rd_guard = rd_guard.kbus_terms[2].write().expect("get KL6581 write guard");
    let reading = rd_guard.read(None).unwrap();
    let value: BitVec<u8, Lsb0> = reading.pick_smart().unwrap(); // 192 bits = 24 bytes
    let bits: &BitSlice<u8, Lsb0> = value.as_bitslice();
    return bits[6*8+96..56+96].load::<u8>();
}

fn buffer_full(term_states: Arc<RwLock<TermStates>>) -> bool {
    let ts = term_states.clone();
    check_sb_bit(ts, 2)
}

fn write_cb1(term_states: Arc<RwLock<TermStates>>, val: bool) {
    let wr_guard = term_states.write().expect("get term_states write guard");
    let mut wr_guard = wr_guard.kbus_terms[2].write().expect("get KL6581 write guard");
    wr_guard.write(val, ChannelInput::Index(1)).unwrap(); // CB.1
}

fn check_sb_bit(term_states: Arc<RwLock<TermStates>>, bit: usize) -> bool {
    let rd_guard = term_states.write().expect("get term_states write guard");
    let rd_guard = rd_guard.kbus_terms[2].write().expect("get KL6581 write guard");
    let reading = rd_guard.read(None).unwrap().pick_smart().unwrap();
    return reading[bit+96];
}

pub fn read_area_1_lights(term_states: Arc<RwLock<TermStates>>) -> u8 {
    let rd_guard = term_states.read().expect("get term_states read guard");
    let rd_guard = rd_guard.kbus_terms[1].write().expect("acquire KL2889 dyn heap write lock");

    let reading = rd_guard.read(Some(ChannelInput::Channel(TermChannel::Ch1))).unwrap();
    return reading.pick_simple().unwrap()
}

pub fn read_area_2_lights(term_states: Arc<RwLock<TermStates>>) -> u8 {
    let rd_guard =
    term_states.read()
    .expect("get term_states read guard");

    let rd_guard =
    rd_guard.ebus_do_terms[0]
    .write()
    .expect("acquire EL2889 dyn heap write lock");

    let reading = rd_guard.read(Some(ChannelInput::Channel(TermChannel::Ch1))).unwrap();
    return reading.pick_simple().unwrap()
}

fn write_all_channel_kl2889(term_states: Arc<RwLock<TermStates>>, val: bool) {
    let wr_guard = term_states.write().expect("get term_states write guard");
    let mut wr_guard = wr_guard.kbus_terms[1].write().expect("get KL2889 write guard");

    for idx in 0..wr_guard.size_in_bits { // All 16 bits of KL2889
        wr_guard.write(val, ChannelInput::Index(idx)).unwrap();
    }
}

fn write_all_channel_el2889(val: bool, term_states: Arc<RwLock<TermStates>>) {
    let wr_guard =
    term_states.read()
    .expect("get term_states read guard");

    let mut wr_guard =
    wr_guard.ebus_do_terms[0]
    .write()
    .expect("acquire EL2889 dyn heap write lock");

    for idx in 0..wr_guard.num_of_channels {
        wr_guard.write(val, ChannelInput::Index(idx)).unwrap();
    }
}

// Very important. Resets hmi cmd in shared mem so that the old value doesn't create conflict with
// later EnOcean commands
fn reset_hmi_cmd() {
    let file = OpenOptions::new().read(true).write(true).open(SHM_PATH).unwrap();
    let mut mmap = map_shared_memory(&file);
    let mut data = read_data(&mmap);
    data.area_1_lights_hmi_cmd = 0;
    write_data(&mut mmap, data);
}