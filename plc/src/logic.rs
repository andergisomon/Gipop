use bitvec::prelude::*;
// For getting read/write locks to terminal objects in PLC memory
use hal::io_defs::*;
use hal::term_cfg::*;

// PLC (business logic) program is defined here via methods that read/write to/from terminal objects in PLC memory

pub async fn plc_execute_logic() {
    smol::Timer::after(std::time::Duration::from_millis(30)).await;

    if read_cb1() != read_sb1() {
        // log::info!("CB.1 <> SB.1");
        log::info!("CB.1 : {}, SB.1 : {}", read_cb1(), read_sb1());
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
        // if read_cb1() {
        //     reset_cb1();
        // }
        // set_cb1();
        write_cb1(!read_sb1());
    }
    else {
        log::info!("CB.1 == SB.1");
    }
    // else {
    //     if buffer_full() {
    //         log::info!("Buffer full");
    //         if read_cb1() {
    //             reset_cb1();
    //         }
    //     }
    // }
    set_ch16_el2889();
}

// use fn write() implemented by Setter trait
fn set_cb1() {
    let wr_guard = &mut *TERM_KL6581.write().expect("acquire KL6581 write lock");
    wr_guard.write(true, ChannelInput::Index(1)).unwrap(); // CB.1
}

fn reset_cb1() {
    let wr_guard = &mut *TERM_KL6581.write().expect("acquire KL6581 write lock");
    wr_guard.write(false, ChannelInput::Index(1)).unwrap(); // CB.1
}

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

fn read_db3() -> u8 {
    let rd_guard = &*TERM_KL6581.read().expect("Acquire TERM_KL6581 read guard");
    let reading = rd_guard.read(None).unwrap();
    let value: BitVec<u8, Lsb0> = reading.pick_smart().unwrap(); // 192 bits = 24 bytes
    let bits: &BitSlice<u8, Lsb0> = value.as_bitslice();
    return bits[6*8..56].load::<u8>();
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

fn buffer_full() -> bool {
    let rd_guard = &*TERM_KL6581.read().expect("Acquire TERM_KL6581 read guard");
    let reading = rd_guard.read(None).unwrap();
    let value: BitVec<u8, Lsb0> = reading.pick_smart().unwrap(); // 192 bits = 24 bytes
    let bits: &BitSlice<u8, Lsb0> = value.as_bitslice();
    return bits[(12*8)+2]; // SB.2
}

fn set_cb0() {
    let wr_guard = &mut *TERM_KL6581.write().expect("acquire KL6581 write lock");
    wr_guard.write(true, ChannelInput::Index(0)).unwrap(); // CB.0
}

fn reset_cb0() {
    let wr_guard = &mut *TERM_KL6581.write().expect("acquire KL6581 write lock");
    wr_guard.write(false, ChannelInput::Index(0)).unwrap(); // CB.0
}

fn reset_db3() {
    let wr_guard = &mut *TERM_KL6581.write().expect("acquire KL6581 write lock");
    wr_guard.write(false, ChannelInput::Index(8*8)).unwrap(); // DB3 output image
}