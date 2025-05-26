use bitvec::prelude::*;
use std::sync::{Arc, RwLock};
use crate::io_defs::*;
use crate::term_cfg::*;

#[repr(u8)]
pub enum CnodeErrors { // variant names follow the KL6581 manual from Beckhoff, with the exception of the obvious 'KL6853` typo
    WatchdogError     = 0x10,
    NoComWithKL6581   = 0x11,
    idx_number_not_OK = 0x12,
    Switch_to_Stopp   = 0x13,
    not_ready         = 0x14,
    No_KL6583_Found   = 0x15,
    TransmissionError = 0x16,
}

impl CnodeErrors {
    pub fn cnode_err_from_u8(value: u8) -> Result<Self, String> {
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
    pub fn cnode_err_to_string(cnode: BitVec<u8, Lsb0>) -> String {
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

pub fn read_cnode(term_states: Arc<RwLock<TermStates>>) -> BitVec<u8, Lsb0> {
    let rd_guard = term_states.write().expect("get term_states write guard");
    let rd_guard = rd_guard.kbus_terms[2].write().expect("get KL6581 write guard");
    let reading = rd_guard.read(None).unwrap();
    let value: BitVec<u8, Lsb0> = reading.pick_smart().unwrap(); // 192 bits = 24 bytes
    let bits: &BitSlice<u8, Lsb0> = value.as_bitslice();
    return BitVec::from_bitslice(&bits[8..16]);
}

pub fn read_cb1(term_states: Arc<RwLock<TermStates>>) -> bool {
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

pub fn check_sb_bit(term_states: Arc<RwLock<TermStates>>, bit: usize) -> bool {
    let rd_guard = term_states.write().expect("get term_states write guard");
    let rd_guard = rd_guard.kbus_terms[2].write().expect("get KL6581 write guard");
    let reading = rd_guard.read(None).unwrap().pick_smart().unwrap();
    return reading[bit+96];
}

pub fn buffer_full(term_states: Arc<RwLock<TermStates>>) -> bool {
    let ts = term_states.clone();
    check_sb_bit(ts, 2)
}

pub fn write_cb1(term_states: Arc<RwLock<TermStates>>, val: bool) {
    let wr_guard = term_states.write().expect("get term_states write guard");
    let mut wr_guard = wr_guard.kbus_terms[2].write().expect("get KL6581 write guard");
    wr_guard.write(val, ChannelInput::Index(1)).unwrap(); // CB.1
}
