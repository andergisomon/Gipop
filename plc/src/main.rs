use env_logger::Env;
pub mod ctrl_loop;
mod ipc;
pub mod logic;
use std::env;
use thread_priority::*;

fn main() { // opcua setup + config + shutdown should be done here
    env_logger::Builder::from_env(Env::default().default_filter_or("info")).init();

    let args: Vec<_> = env::args().collect();

    if args.len() != 3 {
        log::error!("Provide only 2 arguments: [1] The network interface name, and [2] Measure jitter? (YES/NO)");
    }

    let network_interface = args[1].clone();
    let measure_jitter_opt = args[2].clone();

    let cmp = measure_jitter_opt.clone();
    if cmp.as_str() != "YES" && cmp.as_str() != "NO" {
        log::error!("Provide only YES or NO to specify Measure jitter arg")
    }

    let measure_jitter: bool = match measure_jitter_opt.clone().as_str() {
        "YES" => true,
        "NO" => false,
        _ => unreachable!()
    };

    let handle = std::thread::spawn(move || {

        let set_priority_result = set_current_thread_priority(
            ThreadPriority::Max,
        );  

        match set_priority_result {
            Ok(_) => log::info!("set thread prio to MAX"),
            Err(e) => log::error!("failed to set thread prio to MAX: {:?}.", e),
        }

        smol::block_on(ctrl_loop::entry_loop(&network_interface, measure_jitter))
    });

    match handle.join() {
        Ok(result) => result.expect("Entry loop task"),
        Err(e) => panic!("Thread panicked: {:?}", e),
    }
    
    log::info!("Program terminated.");
}