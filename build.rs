fn main() {
    println!("cargo:rerun-if-changed=src/delay_spi.s");
    cc::Build::new()
        .file("src/delay_spi.s")
        .compile("delay_spi");
}
