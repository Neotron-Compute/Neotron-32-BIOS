//! # Neotron 32 BIOS
//!
//! This is the BIOS for the Neotron 32. The Neotron 32 is a retro-style home
//! computer based upon the Texas Instruments Tiva-C Launchpad, which is
//! powered by the TM4C123 SoC and its 80 MHz Cortex-M4F processor core. It
//! has 32 KiB of SRAM (hence the name) and 256 KiB of Flash.
#![no_main]
#![no_std]

use core::fmt::Write;
use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};
use cortex_m_rt::entry;
use hal::gpio::GpioExt;
use hal::sysctl::SysctlExt;
use hal::time::U32Ext;
use tm4c123x_hal as hal;

static BIOS_VERSION: &str = env!("CARGO_PKG_VERSION");

/// Entry point to the BIOS. This is called from the reset vector by
/// `cortex-m-rt`.
#[entry]
fn main() -> ! {
    // Grab the singletons
    let p = hal::Peripherals::take().unwrap();

    // Set the system up for 80 MHz
    let mut sc = p.SYSCTL.constrain();
    sc.clock_setup.oscillator = hal::sysctl::Oscillator::Main(
        hal::sysctl::CrystalFrequency::_16mhz,
        hal::sysctl::SystemClock::UsePll(hal::sysctl::PllOutputFrequency::_80_00mhz),
    );
    let clocks = sc.clock_setup.freeze();

    let mut porta = p.GPIO_PORTA.split(&sc.power_control);

    // Activate UART
    let mut uart = hal::serial::Serial::uart0(
        p.UART0,
        porta
            .pa1
            .into_af_push_pull::<hal::gpio::AF1>(&mut porta.control),
        porta
            .pa0
            .into_af_push_pull::<hal::gpio::AF1>(&mut porta.control),
        (),
        (),
        115_200_u32.bps(),
        hal::serial::NewlineMode::SwapLFtoCRLF,
        &clocks,
        &sc.power_control,
    );

    writeln!(uart, "Neotron 32 BIOS, version {}", BIOS_VERSION).unwrap();
    loop {
        cortex_m::asm::nop();
    }
}

#[inline(never)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // TODO: Print the crash info to the console
    loop {
        atomic::compiler_fence(Ordering::SeqCst);
    }
}

// End of file
