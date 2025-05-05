// For getting read/write locks to terminal objects in PLC memory
use hal::io_defs::*;
use hal::term_cfg::*;

// PLC (business logic) program is defined here via methods that read/write to/from terminal objects in PLC memory

pub async fn plc_execute_logic() {
    toggle_cb3();
    set_ch16_el2889();
    set_all_channel_kl2889();
}

// use fn write() implemented by Setter trait
fn toggle_cb3() {
    let wr_guard = &mut *TERM_KL6581.write().expect("acquire KL6581 write lock");
    wr_guard.write(true, ChannelInput::Index(1)).unwrap(); // CB.1
}

fn set_ch16_el2889() {
    let wr_guard = &mut *TERM_EL2889.write().expect("acquire EL2889 write lock");
    wr_guard.write(true, ChannelInput::Channel(TermChannel::Ch16)).unwrap();
    
}

fn set_all_channel_kl2889() {
    let wr_guard = &mut *TERM_KL2889.write().expect("acquire KL2889 write lock");
    for idx in 0..KL2889_IMG_LEN_BITS { // All 16 bits of KL2889
        wr_guard.write(true, ChannelInput::Index(idx)).unwrap();
    }
}

