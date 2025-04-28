// This file probably needs to be used as a module by another file to handle the cyclic tasks in the primary loop
use bitvec::prelude::*;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Volts(pub f32);

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Milliamps(pub f32);

pub enum ElectricalObservable {
    Voltage(Volts),
    Current(Milliamps),
}

pub enum InputRange {
    Current_0_20mA,
    Current_4_20mA,
    Voltage_0_10V,
    Voltage_2_10V,
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
    fn read(&self) -> &BitVec<u8, Lsb0>;
}

pub trait Setter {
    fn write(&self, data_to_write: &BitSlice<u8, Lsb0>) -> Result<(), anyhow::Error>;
}

pub enum KBusTerminalGender {
    Enby, // 00
    Output, // 01
    Input, // 10
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
    pub values: BitVec<u8, Lsb0>,
    pub num_of_channels: u8,
}

impl Getter for DITerm {
    fn read(&self) -> &BitVec<u8, Lsb0> {
        &self.values
    }
}

pub struct DOTerm {
    pub values: BitVec<u8, Lsb0>,
    pub num_of_channels: u8,
}

// impl Setter for DOTerm {
//     fn write(&self, data_to_write) -> > Result<(), anyhow::Error> {
//         // TODO
//         // get address from &self, write data_to_write at address
//         ;
//     }
// }

pub struct AITerm {
    pub v_or_i: ElectricalObservable,
    pub input_range: InputRange,
    pub value: BitVec<u8, Lsb0>,
    pub num_of_channels: u8,
    pub underrange: bool,
    pub overrange: bool,
    pub limit1: u8,
    pub limit2: u8,
    pub err: bool,
    pub txpdo_state: bool,
    pub txpdo_toggle: bool,
}

impl AITerm {
    fn new(v_or_i: ElectricalObservable,
           input_range: InputRange,
           value: BitVec<u8, Lsb0>,
           num_of_channels: u8,
           underrange: bool,
           overrange: bool,
           limit1: u8,
           limit2: u8,
           err: bool,
           txpdo_state: bool,
           txpdo_toggle: bool
        ) -> Self {
        Self {
            v_or_i,
            input_range,
            value,
            num_of_channels,
            underrange,
            overrange,
            limit1,
            limit2,
            err,
            txpdo_state,
            txpdo_toggle
        }
    }
}

impl Getter for AITerm {
    fn read(&self) -> &BitVec<u8, Lsb0> {
        &self.value
    }
}
