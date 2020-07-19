/// This is the build-script for the Neotron 32 BIOS
///
/// It basically tells Cargo which files are important and should trigger a
/// rebuild. It also compiles any C or ASM components, using the `cc` crate.

fn main() {
    println!("cargo:rerun-if-changed=src/delay_spi.s");
    println!("cargo:rerun-if-changed=neotron-32-bios.ld");
    cc::Build::new()
        .file("src/delay_spi.s")
        .compile("delay_spi");
}
