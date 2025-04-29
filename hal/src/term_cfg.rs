// This file probably needs to be used as a module by another file to handle the cyclic tasks in the primary loop
use bitvec::prelude::*;
use std::ops::Deref;

#[repr(u8)]
pub enum TermChannel { // Channels are always physically labeled starting from 1
    Ch1 = 1, Ch2,  Ch3,  Ch4,
    Ch5,     Ch6,  Ch7,  Ch8,
    Ch9,     Ch10, Ch11, Ch12,
    Ch13,    Ch14, Ch15, Ch16
}

#[derive(PartialEq)]
pub enum ElectricalObservable {
    Voltage(f32),
    Current(f32),
    Simple(u8), // Boolean values
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

pub const EL1889_IMG_LEN: u8 = 2;
pub const KL1889_IMG_LEN: u8 = 2;
pub const EL2889_IMG_LEN: u8 = 2;
pub const KL2889_IMG_LEN: u8 = 2;
pub const KL6581_IMG_LEN: u8 = 12;
pub const EL3024_IMG_LEN: u8 = 16; // 16 bytes total, for each channel value is 2 bytes and status is 2 bytes

// InputTerm and OutputTerm are meant for lower level access to inputs_raw(), outputs_raw() respectively, where the distinction between terminal types aren't important
pub struct InputTerm<'maindevice> {
    in_img_slice: &'maindevice BitSlice<u8, Lsb0>,
    img_len: u8, // No. of bytes
}

pub struct OutputTerm<'maindevice> {
    out_img_slice: &'maindevice BitSlice<u8, Lsb0>,
    img_len: u8, // No. of bytes
}

pub trait Getter {
    fn read(&self, channel: TermChannel) -> Result<ElectricalObservable, String>;
}

pub trait Setter {
    fn write(&mut self, data_to_write: bool, channel: TermChannel) -> Result<(), String>;
}

pub enum KBusTerminalGender {
    Enby, // 0b00
    Output, // 0b01
    Input, // 0b10
}

pub struct KBusSubDevice {
    // name: u8, // for intelligent terminals, name is the 4-digit decimal in 'KLXXXX'
    pub intelligent: bool, // intelligent or simple terminal? 0 -> intelligent, 1 -> simple
    pub size_in_bits: u8, // terminal size in bits
    pub is_kl1212: bool, // is the terminal KL1212?
    pub gender: KBusTerminalGender, // 00 -> KL1202 or KL2212 (digital terminals with both input and output), 01 -> output terminal, 10 -> input terminal
}

pub struct BK1120_Coupler { // Should probably abstract this away but we're fine with this for now
    k_bus_subdevices: Vec<KBusSubDevice>,
    len: u8, // We'll only support up to 127 K-bus terminals for now
}

pub struct DITerm {
    pub values: BitVec<u8, Lsb0>, // Length should match num_of_channels
    pub num_of_channels: u8,
}

impl Getter for DITerm {
    fn read(&self, channel: TermChannel) -> Result<ElectricalObservable, String> {
        let channel: usize = channel as usize;
        let values = self.values.clone();

        let readout = match values.get(channel - 1) {
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

// need to acquire write lock to DO terminal's static instance of LazyLock<Arc<RwLock<DOTerm>>>
// e.g. &mut *TERM_EL3024.write().expect("Acquire TERM_EL2889 write guard").write(...)
impl Setter for DOTerm {
    fn write(&mut self, data_to_write: bool, channel: TermChannel) -> Result<(), String> {
        let channel: usize = channel as usize;
        if channel > (self.num_of_channels as usize) {
            return Err("Specified channel doesn't exist. Index out of bounds".into())
        }
        self.values.set(channel - 1, data_to_write);
        Ok(())
    }
}

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

impl AITerm4Ch {
    pub fn new() -> Self {
        Self {
            v_or_i: VoltageOrCurrent::Current,
            input_range: InputRange::Current_4_20mA,
            num_of_channels: 4,
            ch_values: Analog4ChValues::new(),
            ch_statuses: Analog4ChStatuses::new()
        }
    }
}

impl Getter for AITerm4Ch {
    fn read(&self, channel: TermChannel) -> Result<ElectricalObservable, String> {
        let channel: u8 = channel as u8;

        let raw_int: BitVec::<u8, Lsb0> =
            match channel {
                1 => self.ch_values.ch1.clone(),
                2 => self.ch_values.ch2.clone(),
                3 => self.ch_values.ch3.clone(),
                4 => self.ch_values.ch4.clone(),
                _ => return Err("Invalid channel. Can only specify Channels 1-4.".into())
            };

        if self.v_or_i == VoltageOrCurrent::Current {
            return Ok(ElectricalObservable::Current((raw_int.load::<u16>() as f32 / 32767.0) * 10.0))
        }
        else {
            unreachable!("Voltage signal AITerm detected. This is not yet implemented")
        }
        // Don't have access to any EL AI terminal that takes in voltage right now
    }
}
