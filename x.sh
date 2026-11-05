   cargo build --release
    rust-objcopy -O binary target/thumbv7em-none-eabihf/release/lm4f120-hel
    lo firmware.bin
    lm4flash firmware.bin
