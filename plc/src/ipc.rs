use iceoryx2::prelude::*;

pub fn init_ipc() {


    while node.wait(CYCLE_TIME).is_ok() {
        counter += 1;
        let sample = publisher.loan_uninit()?;

        let sample = sample.write_payload(TransmissionData {
            x: counter as i32,
            y: counter as i32 * 3,
            funky: counter as f64 * 812.12,
        });

        sample.send()?;

        println!("Send sample {} ...", counter);
    }
}