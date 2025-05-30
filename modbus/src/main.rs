async fn client_app() -> Result<(), Box<dyn std::error::Error>> {
    use tokio_modbus::prelude::*;

    let socket_addr = "172.30.40.69:502".parse().unwrap();

    let mut ctx = tcp::connect(socket_addr).await?;

    let data = ctx.read_input_registers(0x0f00, 1).await??;

    println!("Model name 1: {:04x}", data[0]);

    println!("Disconnecting");
    ctx.disconnect().await?;

    Ok(())
}

fn main() {
    tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .unwrap()
        .block_on(async {
            client_app().await.expect("client app error")
        })
}