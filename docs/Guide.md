Root guide describes installing ARM cross-compiler from source
Root guide: https://chacin.dev/blog/cross-compiling-rust-for-the-raspberry-pi/

## Setup
### Install std:

```
rustup target add armv7-unknown-linux-gnueabihf
```

Install a cross-compile toolchain and add it to your path.

For Raspberry Pi 5, download the correct cross compiler from the arm website here:
https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
The architecture you want is aarch64-none-linux-gnu. Then use the linker as per this blog.
Thank you so much. This post was a godsend. I'm now compiling for the Pi 5 using Windows / WSL2 / Debian just fine.

arm-gnu-toolchain-14.2.rel1-aarch64-aarch64-none-linux-gnu.tar.xz

Download toolchain from here, you are looking for "AArch64 GNU/Linux target" most likely its going to be "arm-gnu-toolchain-[VERSION]-x86_64-aarch64-none-linux-gnu.tar.xz".
I compiled with "arm-gnu-toolchain-14.2.rel1-x86_64-aarch64-none-linux-gnu.tar.xz" on my x86 64bit laptop.

Unarchive the files somewhere and add them to your path like this

```
export PATH="[PATH TO YOUR FOLDER]/arm-gnu-toolchain-[VERSION]-x86_64-aarch64-none-linux-gnu/bin:$PATH"
```

### Set PATH env variable:

```
export PATH="/home/ander/SIIP_project/cross-compile/arm-gnu-toolchain-14.2.rel1-x86_64-aarch64-none-linux-gnu/bin:$PATH"
```
export BINDGEN_EXTRA_CLANG_ARGS=--sysroot=~/SIIP_project/cross-compile/arm-gnu-toolchain-14.2.rel1-x86_64-aarch64-none-linux-gnu/aarch64-none-linux-gnu

Run

```
rustup target add aarch64-unknown-linux-gnu
```

Specify linker for our target

```
export CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=aarch64-none-linux-gnu-gcc
```

### Build

```
cargo build --target aarch64-unknown-linux-gnu
```
or
```
cross build --target aarch64-unknown-linux-gnu --release
```

In theory you can specify this in cargo.toml but in my testing it gets ignored for some reason.
Transfer to your Raspberry Pi 5 and execute.

IF LINKER NOT FOUND, set the PATH env variable again

## Deployment

### Upload binaries over SSH

### Sunsuyon binary
```
scp -i ~/gipop_plc /home/ander/SIIP_project/iiot_gateway/sunsuyon/app.py pi@172.30.40.32:/home/pi/Gipop/sunsuyon/sunsuyon/ && scp -i ~/gipop_plc /home/ander/SIIP_project/iiot_gateway/sunsuyon/opcua_client.py pi@172.30.40.32:/home/pi/Gipop/sunsuyon/sunsuyon/
```

### Sunsuyon .env
```
scp -i ~/gipop_plc /home/ander/SIIP_project/iiot_gateway/sunsuyon/.env pi@172.30.40.32:/home/pi/Gipop/sunsuyon/sunsuyon/
```

## Run Sunsuyon
```
/home/pi/Gipop/sunsuyon/sunsuyon/bin/python /home/pi/Gipop/sunsuyon/sunsuyon/app.py
```

### OPCUA binary
Debug:
```
scp -i ~/gipop_plc /home/ander/SIIP_project/ponuridangan/Gipop_proj_repo/Gipop/opcua/target/aarch64-unknown-linux-gnu/debug/opcua pi@172.30.40.32:/home/pi/Gipop/opcua/bin
```
Release:
```
scp -i ~/gipop_plc /home/ander/SIIP_project/ponuridangan/Gipop_proj_repo/Gipop/opcua/target/aarch64-unknown-linux-gnu/release/opcua pi@172.30.40.32:/home/pi/Gipop/opcua/bin
```
### OPCUA server.conf

```
scp -i ~/gipop_plc /home/ander/SIIP_project/ponuridangan/Gipop_proj_repo/Gipop/opcua/target/server.conf pi@172.30.40.32:/home/pi/Gipop/opcua
```

### PLC binary
Debug:
```
scp -i ~/gipop_plc /home/ander/SIIP_project/ponuridangan/Gipop_proj_repo/Gipop/target/aarch64-unknown-linux-gnu/debug/gipop_plc pi@172.30.40.32:/home/pi/Gipop/plc
```
Release:
```
scp -i ~/gipop_plc /home/ander/SIIP_project/ponuridangan/Gipop_proj_repo/Gipop/target/aarch64-unknown-linux-gnu/release/gipop_plc pi@172.30.40.32:/home/pi/Gipop/plc
```

### Run PLC

```
cd /home/pi/Gipop/plc/ && sudo taskset -c 3 ./gipop_plc end0
```

### Run OPCUA server

```
cd /home/pi/Gipop/opcua/bin && sudo RUST_LOG=debug ./opcua
```

### Modbus driver
```
scp -i ~/gipop_plc /home/ander/SIIP_project/ponuridangan/Gipop_proj_repo/Gipop/modbus/target/aarch64-unknown-linux-gnu/debug/modbus pi@172.30.40.32:/home/pi/Gipop/modbus/
```

### Kill GUI

```
sudo systemctl isolate multi-user.target
```

### Restart GUI

```
sudo systemctl isolate graphical.target
```

### Check temp

```
vcgencmd measure_temp
```

### System monitoring

```
htop
```

Click `Filter`, type in `gipop|opcua` to view PLC-related processes

Kill GUI and start PLC
```
sudo systemctl isolate multi-user.target && cd /home/pi/Gipop/plc/ && sudo taskset -c 3 ./gipop_plc end0
```

## Tip on using `cross`
If you get a cryptic error trying to build with `cross` try running `cargo clean`.

