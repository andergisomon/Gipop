#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    use tokio_modbus::prelude::*;

    let socket_addr = "172.30.40.69:502".parse().unwrap();

    let mut ctx = tcp::connect(socket_addr).await?;

    let data = ctx.read_input_registers(0x3840, 1).await??;

    println!("Model name 1: {:04x}", data[0]);

    println!("Disconnecting");
    ctx.disconnect().await?;

    Ok(())
}