use crate::term_cfg::*;
use bitvec::prelude::*;
use std::sync::{Arc, RwLock, LazyLock};

pub trait TxPDO {
    fn parse_offset(&self, bits: &BitSlice<u8, Lsb0>);
}

pub trait RxPDO {
    fn parse_offset(&self, bits: &BitSlice<u8, Lsb0>);
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
                v_or_i: ElectricalObservable::Current(Milliamps(0.0)),
                input_range: InputRange::Current_4_20mA,
                raw_values: BitVec::<u8, Lsb0>::new(),
                num_of_channels: 4,
            }
        )
    )
});

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
                values: BitVec::<u8, Lsb0>::new(),
                num_of_channels: 16,
            }
        )
    )
});