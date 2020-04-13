use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    cc::Build::new()
        .file("src/delay_spi.s")
        .archiver("arm-none-eabi-ar")
        .compile("delay_spi");

    // Put the linker script somewhere the linker can find it
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());

    println!("cargo:rerun-if-changed=src/delay_spi.s");
    println!("cargo:rerun-if-changed=memory.x");
}
