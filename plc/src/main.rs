use env_logger::Env;
pub mod ctrl_loop;
use std::{fs::{OpenOptions}, path::Path,};
mod shared;
use shared::{SharedData, SHM_PATH};

fn main() { // opcua setup + config + shutdown should be done here
    let init = init_shared_memory(); // shared memory between PLC and OPC UA server
    match init {
        Ok(_file) => {
        }
        Err(error) => {
            log::error!("Error opening the file: {}", error);
        }
    }
    
    env_logger::Builder::from_env(Env::default().default_filter_or("info")).init();
    smol::block_on(ctrl_loop::entry_loop("enp3s0")).expect("Entry loop task");
    log::info!("Program terminated.");
}

fn init_shared_memory() -> std::io::Result<std::fs::File> {
    let path = Path::new(SHM_PATH);

    let file = OpenOptions::new()
        .read(true)
        .write(true)
        .create(true)    // create if it doesn't exist
        .truncate(true)  // resize to correct length
        .open(path)?;

    file.set_len(std::mem::size_of::<SharedData>() as u64)?;
    Ok(file)
}
