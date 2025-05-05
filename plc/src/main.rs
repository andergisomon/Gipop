use env_logger::Env;
use hal::ctrl_loop;

fn main() {
    env_logger::Builder::from_env(Env::default().default_filter_or("info")).init();
    smol::block_on(ctrl_loop::entry_loop("enp3s0")).expect("Entry loop task");
    log::info!("Program terminated.");
}