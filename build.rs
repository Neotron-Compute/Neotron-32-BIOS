fn main() {
    println!("cargo:rerun-if-changed=src/delay_spi.s");
    println!("cargo:rerun-if-changed=neotron-32-bios.ld");
    cc::Build::new()
        .file("src/delay_spi.s")
        .compile("delay_spi");
}
