#/bin/bash 
# A lancer à la main...
#st-util > st-link.log 2>&1 &
arm-none-eabi-gdb target/thumbv7em-none-eabihf/debug/stm32-blue-pill-rust
#rust-gdb target/thumbv7em-none-eabihf/debug/nucleo_rust
