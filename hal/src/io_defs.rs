use crate::term_cfg::*;
use bitvec::prelude::*;
use std::sync::{Arc, RwLock, LazyLock};

pub trait TxPDO {
    fn parse(&self, bits: &BitSlice<u8, Lsb0>);
}

pub trait RxPDO {
    fn parse(&self, bits: &BitSlice<u8, Lsb0>);
}

pub static TERM_KL1889: LazyLock<Arc<RwLock<KBusSubDevice>>> = LazyLock::new(|| {
    Arc::new(
        RwLock::new(
            KBusSubDevice {
                intelligent: false,
                size_in_bits: 16,
                is_kl1212: false,
                gender: KBusTerminalGender::Input,
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
            }
        )
    )
});

pub static TERM_EL3024: LazyLock<Arc<RwLock<AITerm>>> = LazyLock::new(|| {
    Arc::new(
        RwLock::new(
            AITerm {
                v_or_i: VoltageOrCurrent::Current,
                input_range: InputRange::Current_4_20mA,
                value: BitVec::<u8, Lsb0>::repeat(false, 16), // value is u16
                num_of_channels: 4,
                underrange:   false,
                overrange:    false,
                limit1:       00,
                limit2:       00,
                err:          false,
                txpdo_state:  false,
                txpdo_toggle: false,
            }
        )
    )
});

pub fn el3024_handler(dst: &Arc<RwLock<AITerm>>, bits: &BitSlice<u8, Lsb0>) {
    let bits: &BitSlice<u8, Lsb0> = &bits[0..33]; // test channel 1 for now

    let mut rw_guard = dst.write().expect("Acquire TERM_EL3024 read/write guard");

    rw_guard.txpdo_toggle = *bits.get(15).unwrap() as bool;
    if !rw_guard.txpdo_toggle { // The TxPDO toggle is toggled by the slave when the data of the associated TxPDO is updated.
        return;
    }
    rw_guard.value.copy_from_bitslice(bits.get(17..33).unwrap());
    rw_guard.txpdo_state = *bits.get(14).unwrap() as bool;
    rw_guard.err         = *bits.get(6).unwrap() as bool;
    rw_guard.limit2      =  bits.get(4..6).unwrap().load_le::<u8>();
    rw_guard.limit1      =  bits.get(2..4).unwrap().load_le::<u8>();
    rw_guard.overrange   = *bits.get(1).unwrap() as bool;
    rw_guard.underrange  = *bits.get(0).unwrap() as bool;
}

pub static TERM_EL1889: LazyLock<Arc<RwLock<DITerm>>> = LazyLock::new(|| {
    Arc::new(
        RwLock::new(
            DITerm {
                values: BitVec::<u8, Lsb0>::repeat(false, 16), // Capacity must match num_of_channels
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
                values: BitVec::<u8, Lsb0>::repeat(false, 16), // Capacity must match num_of_channels
                num_of_channels: 16,
            }
        )
    )
});

pub fn el2889_handler(dst: &mut BitSlice<u8, Lsb0>, bits: &Arc<RwLock<DOTerm>>) {
    let mut rw_guard = bits.write().expect("Acquire TERM_EL2889 read/write guard");

    let num_of_channels = rw_guard.values.len();

    if dst.len() != num_of_channels as usize {
        panic!(
            "Actual DOTerm Values len {} does not match defined number of channels {}",
            dst.len(),
            num_of_channels
        );
    }

    for i in 0..num_of_channels as usize {
        dst.set(i, rw_guard.values[i]);
    }
}