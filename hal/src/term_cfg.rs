use bitvec::prelude::*;
use enum_iterator::Sequence;
use std::ops::Deref;
use std::sync::{Arc, RwLock};

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Sequence)]
pub enum TermChannel { // Channels are always physically labeled starting from 1
    Ch1 = 1, Ch2,  Ch3,  Ch4,
    Ch5,     Ch6,  Ch7,  Ch8,
    Ch9,     Ch10, Ch11, Ch12,
    Ch13,    Ch14, Ch15, Ch16
}

pub enum ChannelInput {
    Channel(TermChannel), // Simple DI/O terminals
    Index(u8) // For EnOcean/intelligent digital terminals
}

#[derive(PartialEq)]
pub enum ElectricalObservable {
    Voltage(f32),
    Current(f32),
    Simple(u8), // Boolean values
    Smart(BitVec<u8, Lsb0>), // For intelligent digital terminals
}

impl ElectricalObservable { // there has to be a better way, will refactor later
    pub fn pick_voltage(&self) -> Option<f32> {
        match self {
            ElectricalObservable::Voltage(v) => Some(*v),
            _ => None
        }
    }
    pub fn pick_current(&self) -> Option<f32> {
        match self {
            ElectricalObservable::Current(i) => Some(*i),
            _ => None
        }
    }
    pub fn pick_simple(&self) -> Option<u8> {
        match self {
            ElectricalObservable::Simple(val) => Some(*val),
            _ => None
        }
    }
    pub fn pick_smart(&self) -> Option<BitVec<u8, Lsb0>> {
        match self {
            ElectricalObservable::Smart(val) => Some(val.clone()),
            _ => None
        }
    }
}

pub enum InputRange {
    Current_0_20mA,
    Current_4_20mA,
    Voltage_0_10V,
    Voltage_2_10V,
}

#[derive(PartialEq)]
pub enum VoltageOrCurrent {
    Voltage,
    Current
}

pub const EL1889_IMG_LEN_BITS: u8 = 2*8;
pub const KL1889_IMG_LEN_BITS: u8 = 2*8;
pub const EL2889_IMG_LEN_BITS: u8 = 2*8;
pub const KL2889_IMG_LEN_BITS: u8 = 2*8;
pub const KL6581_IMG_LEN_BITS: u8 = 12*2*8; // 24 bytes total, 12 each for Input/Output
pub const EL3024_IMG_LEN_BITS: u8 = 16*8; // 16 bytes total, for each channel value is 2 bytes and status is 2 bytes
pub const EL3024_NUM_CHANNELS: u8 = 4;

pub trait Getter { // channel should be passed as None for Enby terms
    fn read(&self, channel: Option<ChannelInput>) -> Result<ElectricalObservable, String>;
}

pub trait Setter {
    fn write(&mut self, data_to_write: bool, channel: ChannelInput) -> Result<(), String>;
}

pub trait Checker { // this is a trait not shared by simple terminals w/o status bits
    fn check(&self, channel: Option<ChannelInput>) -> Option<Result<BitVec::<u8, Lsb0>, String>>; // Returns all non-value bits
}

#[derive(PartialEq)]
pub enum KBusTerminalGender {
    Enby, // 0b00
    Output, // 0b01
    Input, // 0b10
}

// this struct shouldn't actually be populated manually, as all fields except tx_data and rx_data are stored in the
// bk1120 coupler table (starting index 4000); TODO: automatically define E and K bus subdevices
pub struct KBusSubDevice {
    pub hr_name: u32, // human-readable: the 4-digit decimal in 'KLXXXX'; we're not gonna use the coding specified for simple terminals in https://download.beckhoff.com/download/document/io/bus-terminals/bk11x0_bk1250en.pdf
    pub intelligent: bool, // intelligent or simple terminal? 0 -> intelligent, 1 -> simple
    pub size_in_bits: u8, // terminal size in bits
    pub is_kl1212: bool, // is the terminal KL1212?
    pub gender: KBusTerminalGender, // 00 -> KL1202 or KL2212 (digital terminals with both input and output), 01 -> output terminal, 10 -> input terminal
    pub tx_data: Option<BitVec<u8, Lsb0>>, // Output data for Simple Terminals
    pub rx_data: Option<BitVec<u8, Lsb0>>, // Input data for Simple Terminals
}

impl Getter for KBusSubDevice {
    // For Enby terminals the inputs and outputs are concatenated in this order (Lsb) as a single bitvec: [rx_data, tx_data]
    // for reading Enby terminals, channel should be passed as None
    fn read(&self, channel: Option<ChannelInput>) -> Result<ElectricalObservable, String> {
        let channel: usize = match channel {
            Some(ChannelInput::Channel(tc)) => tc as usize - 1, // TermChannel starts at 1
            Some(ChannelInput::Index(idx)) => idx as usize, // Index starts at 0
            None => 0,
        };
    
        let mut values: BitVec<u8> = match self.gender {
            KBusTerminalGender::Input | KBusTerminalGender::Output => BitVec::<u8, Lsb0>::repeat(false, 16),
            KBusTerminalGender::Enby if channel == 0 => BitVec::<u8, Lsb0>::repeat(false, 32*8),
            _ => return Err(format!("Must pass channel input param as None for Enby terms"))
        };

        if self.gender == KBusTerminalGender::Input {
            values = self.rx_data.clone().unwrap();
        }
        if self.gender == KBusTerminalGender::Output {
            values = self.tx_data.clone().unwrap();
        }
        if self.gender == KBusTerminalGender::Enby {
            values = self.rx_data.clone().unwrap();
            values.extend(self.tx_data.clone().unwrap());
        }

        if self.gender == KBusTerminalGender::Input || self.gender == KBusTerminalGender::Output {
            let readout = match values.get(channel) {
                Some(bit) => bit,
                None => return Err(format!("Error reading channel {}: Index out of bounds", channel)),
            };
            let readout_cast = readout.deref().clone() as u8;
            Ok(ElectricalObservable::Simple(readout_cast))
        }
        else {
            if self.gender == KBusTerminalGender::Enby {
                let readout = values;
                Ok(ElectricalObservable::Smart(readout))
            }
            else {unreachable!()} // there are only three genders
        }
    }
}

impl Setter for KBusSubDevice {
    fn write(&mut self, data_to_write: bool, channel: ChannelInput) -> Result<(), String> {
        let channel: usize = match channel {
            ChannelInput::Channel(tc) => tc as usize - 1, // TermChannel starts at 1
            ChannelInput::Index(idx) => idx as usize, // Index starts at 0
        };
    
        if channel > (self.tx_data.as_ref().unwrap().len() as usize) {
            return Err("Specified channel doesn't exist. Index out of bounds".into())
        }
        self.tx_data.as_mut().unwrap().set(channel, data_to_write);
        Ok(())
    }
}

pub struct BK1120_Coupler { // Should probably abstract this away but we're fine with this for now
    k_bus_subdevices: Vec<KBusSubDevice>,
    len: u8, // We'll only support up to 127 K-bus terminals for now
}

pub struct DITerm {
    pub values: BitVec<u8, Lsb0>, // Length should match num_of_channels
    pub num_of_channels: u8,
}

impl DITerm {
    pub fn new(num_of_channels: u8) -> Self {
        Self {
            values: BitVec::<u8, Lsb0>::repeat(false, num_of_channels as usize),
            num_of_channels: num_of_channels
        }
    }

    pub fn refresh(&mut self, bits: &BitSlice<u8, Lsb0>) {
        let num_of_channels = self.values.len();
    
        if bits.len() != num_of_channels {
            panic!(
                "Actual DITerm Values len {} does not match defined number of channels {}",
                bits.len(),
                num_of_channels
            );
        }
    
        for i in 0..num_of_channels {
            self.values.set(i, bits[i]);
        }
    }
}

// how to use:
// let mut read_guard = &*TERM_EL1889.read().expect("Acquire TERM_EL1889 read guard");
// if read_guard.read(TermChannel::Ch11).unwrap() == ElectricalObservable::Simple(1) {
//     log::info!("Limit switch hit");
// }
impl Getter for DITerm {
    fn read(&self, channel: Option<ChannelInput>) -> Result<ElectricalObservable, String> {
        let channel: usize = match channel {
            Some(ChannelInput::Channel(tc)) => (tc as usize) - 1,
            Some(ChannelInput::Index(idx)) => idx as usize,
            None => return Err(format!("Can only pass None for Enby terms"))
        };

        let values = self.values.clone();

        let readout = match values.get(channel) {
            Some(bit) => bit,
            None => return Err(format!("Error reading channel {}: Index out of bounds", channel)),
        };

        let readout_cast = readout.deref().clone() as u8;

        Ok(ElectricalObservable::Simple(readout_cast))
    }
}


pub struct DOTerm {
    pub values: BitVec<u8, Lsb0>,
    pub num_of_channels: u8,
}

impl DOTerm {
    pub fn new(num_of_channels: u8) -> Self {
        Self {
            values: BitVec::<u8, Lsb0>::repeat(false, num_of_channels as usize),
            num_of_channels: num_of_channels
        }
    }

    pub fn refresh(&self, dst: &mut BitSlice<u8, Lsb0>) {    
        let num_of_channels = self.values.len();
    
        if dst.len() != num_of_channels {
            panic!(
                "Actual DOTerm Values len {} does not match defined number of channels {}",
                dst.len(),
                num_of_channels
            );
        }
    
        for i in 0..num_of_channels {
            dst.set(i, self.values[i]);
        }
    }
}

// need to acquire write lock to DO terminal's static instance of LazyLock<Arc<RwLock<DOTerm>>>
// e.g. &mut *TERM_EL3024.write().expect("Acquire TERM_EL2889 write guard").write(...)
// how to use:
// let mut wr_guard = &mut *TERM_EL2889.write().expect("acquire EL3024 write lock");
// wr_guard.write(true, TermChannel::Ch16).unwrap();
impl Setter for DOTerm {
    fn write(&mut self, data_to_write: bool, channel: ChannelInput) -> Result<(), String> {
        let channel: usize = match channel {
            ChannelInput::Channel(tc) => (tc as usize) - 1,
            ChannelInput::Index(idx) => idx as usize,
        };

        if channel > (self.num_of_channels as usize) {
            return Err("Specified channel doesn't exist. Index out of bounds".into())
        }
        self.values.set(channel, data_to_write);
        Ok(())
    }
}

impl Getter for DOTerm {
    fn read(&self, channel: Option<ChannelInput>) -> Result<ElectricalObservable, String> {
        let channel: usize = match channel {
            Some(ChannelInput::Channel(tc)) => (tc as usize) - 1,
            Some(ChannelInput::Index(idx)) => idx as usize,
            None => return Err(format!("Can only pass None for Enby terms"))
        };

        let values = self.values.clone();

        let readout = match values.get(channel) {
            Some(bit) => bit,
            None => return Err(format!("Error reading channel {}: Index out of bounds", channel)),
        };

        let readout_cast = readout.deref().clone() as u8;

        Ok(ElectricalObservable::Simple(readout_cast))
    }
}

// TODO this should be a Vec<> instead
pub struct Analog4ChValues {
    pub ch1: BitVec<u8, Lsb0>,
    pub ch2: BitVec<u8, Lsb0>,
    pub ch3: BitVec<u8, Lsb0>,
    pub ch4: BitVec<u8, Lsb0>,
}

impl Analog4ChValues {
    pub fn new() -> Self {
        Self { // values u16 each
            ch1: BitVec::<u8, Lsb0>::repeat(false, 16),
            ch2: BitVec::<u8, Lsb0>::repeat(false, 16),
            ch3: BitVec::<u8, Lsb0>::repeat(false, 16),
            ch4: BitVec::<u8, Lsb0>::repeat(false, 16)
        }
    }
}

// TOOD this should be a Vec<> instead
pub struct Analog4ChStatuses {
    pub ch1: El30xxStatuses,
    pub ch2: El30xxStatuses,
    pub ch3: El30xxStatuses,
    pub ch4: El30xxStatuses,
}

impl Analog4ChStatuses {
    pub fn new() -> Self {
        Self {
            ch1: El30xxStatuses::new(),
            ch2: El30xxStatuses::new(),
            ch3: El30xxStatuses::new(),
            ch4: El30xxStatuses::new(),
        }
    }
}

#[derive(Clone)]
pub struct El30xxStatuses {
    pub txpdo_toggle: bool,
    pub txpdo_state: bool,
    pub err: bool,
    pub limit1: u8,
    pub limit2: u8,
    pub underrange: bool,
    pub overrange: bool
}

impl El30xxStatuses {
    pub fn new() -> Self {
        Self {
            txpdo_toggle: false,
            txpdo_state: false,
            err: false,
            limit1: 0b00,
            limit2: 0b00,
            underrange: false,
            overrange: false
        }
    }
}

pub struct AITerm4Ch {
    pub v_or_i: VoltageOrCurrent,
    pub input_range: InputRange,
    pub num_of_channels: u8,
    pub ch_values: Analog4ChValues,
    pub ch_statuses: Analog4ChStatuses
}

// TODO the type AITerm4Ch needs to be completely refactored to be number-of-channels-agnostic
// the data contained (values and statuses) should really be Vec<> instead of structs
impl AITerm4Ch {
    pub fn new() -> Self {
        Self {
            v_or_i: VoltageOrCurrent::Current,
            input_range: InputRange::Current_4_20mA,
            num_of_channels: 4,
            ch_values: Analog4ChValues::new(), // this should really be a Vec<>
            ch_statuses: Analog4ChStatuses::new() // this should really be a Vec<>
        }
    }
}

impl Getter for AITerm4Ch {
    fn read(&self, channel: Option<ChannelInput>) -> Result<ElectricalObservable, String> {
        let channel: usize = match channel {
            Some(ChannelInput::Channel(tc)) => tc as usize,
            Some(ChannelInput::Index(idx)) => idx as usize + 1,
            None => return Err(format!("Can only pass None for Enby terms"))
        };

        let raw_int: BitVec::<u8, Lsb0> =
            match channel {
                1 => self.ch_values.ch1.clone(),
                2 => self.ch_values.ch2.clone(),
                3 => self.ch_values.ch3.clone(),
                4 => self.ch_values.ch4.clone(),
                _ => return Err("Invalid channel. Can only specify Channels 1-4.".into())
            };

        if self.v_or_i == VoltageOrCurrent::Current {
            let t = raw_int.load::<u16>() as f32 / 30518.0;
            let i = 4.0*(1.0-t) + 20.0*t;
            return Ok(ElectricalObservable::Current(i))
        }
        else {
            unreachable!("Voltage signal AITerm detected. This is not yet implemented")
        }
        // Don't have access to any EL AI terminal that takes in voltage right now
    }
}

impl Checker for AITerm4Ch {
    fn check(&self, channel: Option<ChannelInput>) -> Option<Result<BitVec::<u8, Lsb0>, String>> {
        let channel: usize = match channel {
            Some(ChannelInput::Channel(tc)) => tc as usize,
            Some(ChannelInput::Index(idx)) => idx as usize + 1,
            None => return Some(Err("Cannot return None channel. Can only specify Channels 1-4.".into()))
        };
        
        let ch_status = match channel {
            1 => self.ch_statuses.ch1.clone(),
            2 => self.ch_statuses.ch2.clone(),
            3 => self.ch_statuses.ch3.clone(),
            4 => self.ch_statuses.ch4.clone(),
            _ => return Some(Err("Invalid channel. Can only specify Channels 1-4.".into()))
        };

        let mut bits = BitVec::<u8, Lsb0>::new();

        // these are bools
        bits.push(ch_status.txpdo_toggle);
        bits.push(ch_status.txpdo_state);
        bits.push(ch_status.err);

        // push first Lsb 2 bits from limit2
        bits.push((ch_status.limit2 & 0b01) != 0);
        bits.push((ch_status.limit2 & 0b10) != 0);

        // push first Lsb 2 bits from limit1
        bits.push((ch_status.limit1 & 0b01) != 0);
        bits.push((ch_status.limit1 & 0b10) != 0);

        // remaining bools
        bits.push(ch_status.overrange);
        bits.push(ch_status.underrange);

        Some(Ok(bits))
    }
}

impl Checker for KBusSubDevice {
    fn check(&self, _channel: Option<ChannelInput>) -> Option<Result<BitVec::<u8, Lsb0>, String>> {
        if self.intelligent && self.hr_name == 6581 {
            let value: BitVec::<u8, Lsb0> = self.tx_data.clone().unwrap(); // Input image, transmitted from terminal to controller
            let bits: &BitSlice<u8, Lsb0> = value.as_bitslice();
            return Some(Ok(BitVec::from_bitslice(&bits[0..8]))) // SB - Status Byte
        }

        if self.gender != KBusTerminalGender::Enby {
            return None
        }
        else {
            unimplemented!("We don't have access to simple enby terminals")
        }

    }
}