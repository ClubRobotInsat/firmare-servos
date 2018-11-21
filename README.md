# Create a new firmware

```bash
git clone https://github.com/ClubRobotInsat/firmware-template
mv firmware-template firmware-<my-board>
cd firmware-<myboard>
rustup target add thumbv7m-none-eabi
cargo build --release # everything *should* work
```


