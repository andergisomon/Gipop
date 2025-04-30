use crate::term_cfg::*;
use bitvec::prelude::*;
use std::sync::{Arc, RwLock, LazyLock};

pub static TERM_KL1889: LazyLock<Arc<RwLock<KBusSubDevice>>> = LazyLock::new(|| {
    Arc::new(
        RwLock::new(
            KBusSubDevice {
                intelligent: false,
                size_in_bits: 16,
                is_kl1212: false,
                gender: KBusTerminalGender::Input,
                tx_data: None,
                rx_data: Some(BitVec::<u8, Lsb0>::repeat(false, 16)), // Capacity must match input process image size
            }
        )
    )
});

pub static TERM_KL2889: LazyLock<Arc<RwLock<KBusSubDevice>>> = LazyLock::new(|| {
    Arc::new(
        RwLock::new(
            KBusSubDevice {
                intelligent: false,
                size_in_bits: 16,
                is_kl1212: false,
                gender: KBusTerminalGender::Output,
                tx_data: Some(BitVec::<u8, Lsb0>::repeat(false, 16)), // Capacity must match output process image size
                rx_data: None
            }
        )
    )
});

pub static TERM_EL3024: LazyLock<Arc<RwLock<AITerm4Ch>>> = LazyLock::new(|| {
    Arc::new(
        RwLock::new(
            AITerm4Ch::new()
        )
    )
});

pub fn el3024_handler(dst: &Arc<RwLock<AITerm4Ch>>, bits: &BitSlice<u8, Lsb0>, channel: TermChannel) {
    let channel: u8 = channel as u8;
    let bits: &BitSlice<u8, Lsb0> = &bits[32*(channel as usize -1)..(32*channel as usize)];
    let mut rw_guard = dst.write().expect("Acquire TERM_EL3024 read/write guard");

    match channel { // will reimplement using bitmasking later; should be way neater
        1 => {
            rw_guard.ch_statuses.ch1.txpdo_toggle = *bits.get(15).unwrap() as bool;
            if !rw_guard.ch_statuses.ch1.txpdo_toggle { // The TxPDO toggle is toggled by the slave when the data of the associated TxPDO is updated.
                return; }
        },
        2 => {
            rw_guard.ch_statuses.ch2.txpdo_toggle = *bits.get(15).unwrap() as bool;
            if !rw_guard.ch_statuses.ch2.txpdo_toggle { // The TxPDO toggle is toggled by the slave when the data of the associated TxPDO is updated.
                return;}
        },
        3 => {
            rw_guard.ch_statuses.ch3.txpdo_toggle = *bits.get(15).unwrap() as bool;
            if !rw_guard.ch_statuses.ch3.txpdo_toggle { // The TxPDO toggle is toggled by the slave when the data of the associated TxPDO is updated.
                return;}
        },
        4 => {
            rw_guard.ch_statuses.ch4.txpdo_toggle = *bits.get(15).unwrap() as bool;
            if !rw_guard.ch_statuses.ch4.txpdo_toggle { // The TxPDO toggle is toggled by the slave when the data of the associated TxPDO is updated.
                return;}
        },
        _ => {unreachable!();}
    }

    match channel { // this is really ugly, but i don't want to add more abstractions and having to deal with more borrow checking gymnastics
        1 => {
            rw_guard.ch_values.ch1.copy_from_bitslice(bits.get(16..32).unwrap());
            rw_guard.ch_statuses.ch1.txpdo_state = *bits.get(14).unwrap() as bool;
            rw_guard.ch_statuses.ch1.err         = *bits.get(6).unwrap() as bool;
            rw_guard.ch_statuses.ch1.limit2      =  bits.get(4..6).unwrap().load_le::<u8>();
            rw_guard.ch_statuses.ch1.limit1      =  bits.get(2..4).unwrap().load_le::<u8>();
            rw_guard.ch_statuses.ch1.overrange   = *bits.get(1).unwrap() as bool;
            rw_guard.ch_statuses.ch1.underrange  = *bits.get(0).unwrap() as bool;
        },
        2 => {
            rw_guard.ch_values.ch2.copy_from_bitslice(bits.get(16..32).unwrap());
            rw_guard.ch_statuses.ch2.txpdo_state = *bits.get(14).unwrap() as bool;
            rw_guard.ch_statuses.ch2.err         = *bits.get(6).unwrap() as bool;
            rw_guard.ch_statuses.ch2.limit2      =  bits.get(4..6).unwrap().load_le::<u8>();
            rw_guard.ch_statuses.ch2.limit1      =  bits.get(2..4).unwrap().load_le::<u8>();
            rw_guard.ch_statuses.ch2.overrange   = *bits.get(1).unwrap() as bool;
            rw_guard.ch_statuses.ch2.underrange  = *bits.get(0).unwrap() as bool;
        },
        3 => {
            rw_guard.ch_values.ch3.copy_from_bitslice(bits.get(16..32).unwrap());
            rw_guard.ch_statuses.ch3.txpdo_state = *bits.get(14).unwrap() as bool;
            rw_guard.ch_statuses.ch3.err         = *bits.get(6).unwrap() as bool;
            rw_guard.ch_statuses.ch3.limit2      =  bits.get(4..6).unwrap().load_le::<u8>();
            rw_guard.ch_statuses.ch3.limit1      =  bits.get(2..4).unwrap().load_le::<u8>();
            rw_guard.ch_statuses.ch3.overrange   = *bits.get(1).unwrap() as bool;
            rw_guard.ch_statuses.ch3.underrange  = *bits.get(0).unwrap() as bool;
        },
        4 => {
            rw_guard.ch_values.ch4.copy_from_bitslice(bits.get(16..32).unwrap());
            rw_guard.ch_statuses.ch4.txpdo_state = *bits.get(14).unwrap() as bool;
            rw_guard.ch_statuses.ch4.err         = *bits.get(6).unwrap() as bool;
            rw_guard.ch_statuses.ch4.limit2      =  bits.get(4..6).unwrap().load_le::<u8>();
            rw_guard.ch_statuses.ch4.limit1      =  bits.get(2..4).unwrap().load_le::<u8>();
            rw_guard.ch_statuses.ch4.overrange   = *bits.get(1).unwrap() as bool;
            rw_guard.ch_statuses.ch4.underrange  = *bits.get(0).unwrap() as bool;
        },
        _ => {unreachable!();}
    }

}

pub static TERM_EL1889: LazyLock<Arc<RwLock<DITerm>>> = LazyLock::new(|| {
    Arc::new(
        RwLock::new(
            DITerm {
                values: BitVec::<u8, Lsb0>::repeat(false, 16), // Capacity must match num_of_channels (yes ik i couldve used dynamic dispatch here, zig's comptime would be great here)
                num_of_channels: 16,
            }
        )
    )
});

pub fn el1889_handler(dst: &Arc<RwLock<DITerm>>, bits: &BitSlice<u8, Lsb0>) {
    let mut rw_guard = dst.write().expect("Acquire TERM_EL1889 read/write guard");

    let num_of_channels = rw_guard.values.len();

    if bits.len() != num_of_channels as usize {
        panic!(
            "Actual DITerm Values len {} does not match defined number of channels {}",
            bits.len(),
            num_of_channels
        );
    }

    for i in 0..num_of_channels as usize {
        rw_guard.values.set(i, bits[i]);
    }
}

pub static TERM_EL2889: LazyLock<Arc<RwLock<DOTerm>>> = LazyLock::new(|| {
    Arc::new(
        RwLock::new(
            DOTerm {
                values: BitVec::<u8, Lsb0>::repeat(false, 16), // Capacity must match num_of_channels (yes ik i couldve used dynamic dispatch here, zig's comptime would be great here)
                num_of_channels: 16,
            }
        )
    )
});

pub fn el2889_handler(dst: &mut BitSlice<u8, Lsb0>, bits: &Arc<RwLock<DOTerm>>) {
    let rd_guard = bits.read().expect("Acquire TERM_EL2889 read guard"); // RO access

    let num_of_channels = rd_guard.values.len();

    if dst.len() != num_of_channels as usize {
        panic!(
            "Actual DOTerm Values len {} does not match defined number of channels {}",
            dst.len(),
            num_of_channels
        );
    }

    for i in 0..num_of_channels as usize {
        dst.set(i, rd_guard.values[i]);
    }
}

pub static TERM_KL6581: LazyLock<Arc<RwLock<KBusSubDevice>>> = LazyLock::new(|| {
    Arc::new(
        RwLock::new(
            KBusSubDevice {
                intelligent: true,
                size_in_bits: 24*8, // 12 bytes input, 12 bytes output
                is_kl1212: false,
                gender: KBusTerminalGender::Enby,
                tx_data: Some(BitVec::<u8, Lsb0>::repeat(false, 12*8)), // Capacity must match output process image size
                rx_data: Some(BitVec::<u8, Lsb0>::repeat(false, 12*8)), // Capacity must match input process image size
            }
        )
    )
});

pub fn kl6581_output_handler(dst: &mut BitSlice<u8, Lsb0>, bits: &Arc<RwLock<KBusSubDevice>>) {
    let rd_guard = bits.read().expect("Acquire TERM_KL6581 read guard"); // RO access

    let num_of_channels = rd_guard.tx_data.as_ref().unwrap().len();

    if dst.len() != num_of_channels as usize {
        panic!(
            "Actual DOTerm Values len {} does not match defined number of channels {}",
            dst.len(),
            num_of_channels
        );
    }

    for i in 0..num_of_channels as usize {
        dst.set(i, rd_guard.tx_data.as_ref().unwrap()[i]);
    }
}

pub fn kl6581_input_handler(dst: &Arc<RwLock<KBusSubDevice>>, bits: &BitSlice<u8, Lsb0>) {
    let mut rw_guard = dst.write().expect("Acquire TERM_KL6581 read/write guard");

    let num_of_channels = rw_guard.rx_data.as_ref().unwrap().len();

    if bits.len() != num_of_channels as usize {
        panic!(
            "Actual DITerm Values len {} does not match defined number of channels {}",
            bits.len(),
            num_of_channels
        );
    }

    for i in 0..num_of_channels as usize {
        rw_guard.rx_data.as_mut().unwrap().set(i, bits[i]);
    }
}