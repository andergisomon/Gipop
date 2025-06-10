use env_logger::Env;
pub mod ctrl_loop;
mod ipc;
pub mod logic;
use std::env;
use thread_priority::*;

fn main() { // opcua setup + config + shutdown should be done here
    env_logger::Builder::from_env(Env::default().default_filter_or("info")).init();

    let args: Vec<String> = env::args().collect();

    if args.len() != 2 {
        log::error!("Provide only 1 argument: The network interface name!");
    }

    let network_interface = args[1].clone();
    let handle = std::thread::spawn(move || {

        let set_priority_result = set_current_thread_priority(
            ThreadPriority::Max,
        );  

        match set_priority_result {
            Ok(_) => log::info!("set thread prio to MAX"),
            Err(e) => log::error!("failed to set thread prio to MAX: {:?}.", e),
        }

        smol::block_on(ctrl_loop::entry_loop(&network_interface))
    });

    match handle.join() {
        Ok(result) => result.expect("Entry loop task"),
        Err(e) => panic!("Thread panicked: {:?}", e),
    }
    
    log::info!("Program terminated.");
}