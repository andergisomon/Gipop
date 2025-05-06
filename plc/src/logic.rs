use bitvec::prelude::*;
// For getting read/write locks to terminal objects in PLC memory
use hal::io_defs::*;
use hal::term_cfg::*;

// PLC (business logic) program is defined here via methods that read/write to/from terminal objects in PLC memory

pub async fn plc_execute_logic() {
    // smol::Timer::after(std::time::Duration::from_millis(30)).await;
    if read_cb1() != read_sb1() {
        // log::info!("CB.1 <> SB.1");
        // log::info!("CB.1 : {}, SB.1 : {}", read_cb1(), read_sb1());
        // if read_db3() != 0 {
        //     log::info!("DB3 contents: {}", read_db3());
        // }
        if (read_db3() & 0b11110000) == 0b01010000 {
            log::info!("Rocker B, I pos. pressed");
            write_all_channel_kl2889(true);
        }

        if (read_db3() & 0b11110000) == 0b01110000 {
            log::info!("Rocker B, O pos. pressed");
            write_all_channel_kl2889(false);
        }

        if (read_db3() & 0b11110000) == 0b00010000 {
            log::info!("Rocker A, I pos. pressed");
            write_all_channel_el2889(true);
        }

        if (read_db3() & 0b11110000) == 0b00110000 {
            log::info!("Rocker A, 0 pos. pressed");
            write_all_channel_el2889(false);
        }

        write_cb1(!read_sb1()); // Very important. Tells KL6581 we've fetched the packet.
    }
    else {
        // log::info!("CB.1 == SB.1");
        if buffer_full() {
            log::info!("Buffer full");
            write_cb1(!read_sb1());
        }
    }

}

// use fn write() implemented by Setter trait
fn write_cb1(val: bool) {
    let wr_guard = &mut *TERM_KL6581.write().expect("acquire KL6581 write lock");
    wr_guard.write(val, ChannelInput::Index(1)).unwrap(); // CB.1
}

fn read_sb1() -> bool {
    let rd_guard = &*TERM_KL6581.read().expect("Acquire TERM_KL6581 read guard");
    let reading = rd_guard.read(None).unwrap();
    let value: BitVec<u8, Lsb0> = reading.pick_smart().unwrap(); // 192 bits = 24 bytes
    let bits: &BitSlice<u8, Lsb0> = value.as_bitslice();
    return bits[(12*8)+1];
}

fn read_cb1() -> bool {
    let rd_guard = &*TERM_KL6581.read().expect("Acquire TERM_KL6581 read guard");
    let reading = rd_guard.read(None).unwrap();
    let value: BitVec<u8, Lsb0> = reading.pick_smart().unwrap(); // 192 bits = 24 bytes
    let bits: &BitSlice<u8, Lsb0> = value.as_bitslice();
    return bits[1];
}

pub fn read_db3() -> u8 {
    let rd_guard = &*TERM_KL6581.read().expect("Acquire TERM_KL6581 read guard");
    let reading = rd_guard.read(None).unwrap();
    let value: BitVec<u8, Lsb0> = reading.pick_smart().unwrap(); // 192 bits = 24 bytes
    let bits: &BitSlice<u8, Lsb0> = value.as_bitslice();
    return bits[6*8..56].load::<u8>();
}

pub fn read_area_1_lights() -> u8 {
    let rd_guard = &*TERM_KL2889.read().expect("Acquire TERM_KL2889 read guard");
    let reading = rd_guard.read(Some(ChannelInput::Channel(TermChannel::Ch1))).unwrap();
    return reading.pick_simple().unwrap();
}

pub fn read_area_2_lights() -> u8 {
    let rd_guard = &*TERM_EL2889.read().expect("Acquire TERM_EL2889 read guard");
    let reading = rd_guard.read(Some(ChannelInput::Channel(TermChannel::Ch1))).unwrap();
    return reading.pick_simple().unwrap();
}

fn set_ch16_el2889() {
    let wr_guard = &mut *TERM_EL2889.write().expect("acquire EL2889 write lock");
    wr_guard.write(true, ChannelInput::Channel(TermChannel::Ch16)).unwrap();
}

fn write_all_channel_kl2889(val: bool) {
    let wr_guard = &mut *TERM_KL2889.write().expect("acquire KL2889 write lock");
    for idx in 0..KL2889_IMG_LEN_BITS { // All 16 bits of KL2889
        wr_guard.write(val, ChannelInput::Index(idx)).unwrap();
    }
}

fn write_all_channel_el2889(val: bool) {
    let wr_guard = &mut *TERM_EL2889.write().expect("acquire EL2889 write lock");
    for idx in 0..EL2889_IMG_LEN_BITS { // All 16 bits of EL2889
        wr_guard.write(val, ChannelInput::Index(idx)).unwrap();
    }
}

fn buffer_full() -> bool {
    let rd_guard = &*TERM_KL6581.read().expect("Acquire TERM_KL6581 read guard");
    let reading = rd_guard.read(None).unwrap();
    let value: BitVec<u8, Lsb0> = reading.pick_smart().unwrap(); // 192 bits = 24 bytes
    let bits: &BitSlice<u8, Lsb0> = value.as_bitslice();
    return bits[(12*8)+2]; // SB.2
}
