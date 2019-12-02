//! # Neotron 32 BIOS
//!
//! This is the BIOS for the Neotron 32. The Neotron 32 is a retro-style home
//! computer based upon the Texas Instruments Tiva-C Launchpad, which is
//! powered by the TM4C123 SoC and its 80 MHz Cortex-M4F processor core. It
//! has 32 KiB of SRAM (hence the name) and 256 KiB of Flash.
//!
//! ## Basic Operation
//!
//! We initialise the bare-minimum of hardware required to provide a console,
//! and then jump to the Operating System. We currently assume the Operating
//! System is located at address 0x0002_0000 (giving 128 KiB for the BIOS and
//! 128 KiB for the OS).
//!
//! ## Hardware
//!
//! * TM4C123GH6PM System-on-Chip
//!     * Cortex-M4F @ 80 MHz
//!     * 32 KiB SRAM
//!     * 256 KiB Flash ROM
//!     * 256 KiB Flash ROM
//! * PS/2 Keyboard and Mouse controller (connected to UART7)
//! * Mono audio output
//! * SD Card connector (connected to SSI3)
//! * 800 x 600 resolution 8-colour VGA video output (connected to SSI0, SSI1 and SSI2)
//! * MCP7940N I2C battery-backed Real Time Clock
//! * USB Serial interface (connected to UART0)
//! * RS-232 serial interface (connected to UART1)
//! * MIDI In and Out (connected to UART3)
//! * Atari compatible joystick interface
//!
//! ## License
//!
//!     Copyright (C) 2019 Jonathan 'theJPster' Pallant <github@thejpster.org.uk>
//!
//!     This program is free software: you can redistribute it and/or modify
//!     it under the terms of the GNU General Public License as published by
//!     the Free Software Foundation, either version 3 of the License, or
//!     (at your option) any later version.
//!
//!     This program is distributed in the hope that it will be useful,
//!     but WITHOUT ANY WARRANTY; without even the implied warranty of
//!     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//!     GNU General Public License for more details.
//!
//!     You should have received a copy of the GNU General Public License
//!     along with this program.  If not, see <https://www.gnu.org/licenses/>.

#![no_main]
#![no_std]
#![deny(missing_docs)]

// ===========================================================================
// Sub-Modules
// ===========================================================================

// None

// ===========================================================================
// Imports
// ===========================================================================

use core::fmt::Write;
use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};
use cortex_m_rt::entry;
use hal::gpio::GpioExt;
use hal::sysctl::SysctlExt;
use hal::time::U32Ext;
use tm4c123x_hal as hal;

use neotron_common_bios as common;

// ===========================================================================
// Types
// ===========================================================================

/// This holds our system state - all our HAL drivers, etc.
pub struct BoardInner {
    usb_uart: hal::serial::Serial<
        hal::serial::UART0,
        hal::gpio::gpioa::PA1<hal::gpio::AlternateFunction<hal::gpio::AF1, hal::gpio::PushPull>>,
        hal::gpio::gpioa::PA0<hal::gpio::AlternateFunction<hal::gpio::AF1, hal::gpio::PushPull>>,
        (),
        (),
    >,
    avr_uart: hal::serial::Serial<
        hal::serial::UART7,
        hal::gpio::gpioe::PE1<hal::gpio::AlternateFunction<hal::gpio::AF1, hal::gpio::PushPull>>,
        hal::gpio::gpioe::PE0<hal::gpio::AlternateFunction<hal::gpio::AF1, hal::gpio::PushPull>>,
        (),
        (),
    >,
    midi_uart: hal::serial::Serial<
        hal::serial::UART3,
        hal::gpio::gpioc::PC7<hal::gpio::AlternateFunction<hal::gpio::AF1, hal::gpio::PushPull>>,
        hal::gpio::gpioc::PC6<hal::gpio::AlternateFunction<hal::gpio::AF1, hal::gpio::PushPull>>,
        (),
        (),
    >,
    rs232_uart: hal::serial::Serial<
        hal::serial::UART1,
        hal::gpio::gpiob::PB1<hal::gpio::AlternateFunction<hal::gpio::AF1, hal::gpio::PushPull>>,
        hal::gpio::gpiob::PB0<hal::gpio::AlternateFunction<hal::gpio::AF1, hal::gpio::PushPull>>,
        hal::gpio::gpioc::PC4<hal::gpio::AlternateFunction<hal::gpio::AF8, hal::gpio::PushPull>>,
        hal::gpio::gpioc::PC5<hal::gpio::AlternateFunction<hal::gpio::AF8, hal::gpio::PushPull>>,
    >,
}

// ===========================================================================
// Static Variables and Constants
// ===========================================================================

static BIOS_VERSION: &str = concat!("Neotron 32 BIOS, version ", env!("CARGO_PKG_VERSION"), "\0");

static API_CALLS: common::Api = common::Api {
    api_version_get,
    bios_version_get,
    serial_configure,
    serial_get_info,
    serial_write,
    time_get,
    time_set,
};

static GLOBAL_BOARD: spin::Mutex<Option<BoardInner>> = spin::Mutex::new(None);

// ===========================================================================
// Public Functions
// ===========================================================================

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
    let mut portb = p.GPIO_PORTB.split(&sc.power_control);
    let mut portc = p.GPIO_PORTC.split(&sc.power_control);
    // let mut portd = p.GPIO_PORTD.split(&sc.power_control);
    let mut porte = p.GPIO_PORTE.split(&sc.power_control);
    // let mut portf = p.GPIO_PORTF.split(&sc.power_control);

    let mut board = BoardInner {
        // USB Serial UART
        usb_uart: self::hal::serial::Serial::uart0(
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
            self::hal::serial::NewlineMode::SwapLFtoCRLF,
            &clocks,
            &sc.power_control,
        ),

        // MIDI UART
        midi_uart: self::hal::serial::Serial::uart3(
            p.UART3,
            portc
                .pc7
                .into_af_push_pull::<hal::gpio::AF1>(&mut portc.control),
            portc
                .pc6
                .into_af_push_pull::<hal::gpio::AF1>(&mut portc.control),
            (),
            (),
            31250_u32.bps(),
            self::hal::serial::NewlineMode::Binary,
            &clocks,
            &sc.power_control,
        ),

        // AVR UART
        avr_uart: self::hal::serial::Serial::uart7(
            p.UART7,
            porte
                .pe1
                .into_af_push_pull::<hal::gpio::AF1>(&mut porte.control),
            porte
                .pe0
                .into_af_push_pull::<hal::gpio::AF1>(&mut porte.control),
            (),
            (),
            19200_u32.bps(),
            self::hal::serial::NewlineMode::Binary,
            &clocks,
            &sc.power_control,
        ),

        // RS-232 UART
        rs232_uart: self::hal::serial::Serial::uart1(
            p.UART1,
            portb
                .pb1
                .into_af_push_pull::<hal::gpio::AF1>(&mut portb.control),
            portb
                .pb0
                .into_af_push_pull::<hal::gpio::AF1>(&mut portb.control),
            portc
                .pc4
                .into_af_push_pull::<hal::gpio::AF8>(&mut portc.control),
            portc
                .pc5
                .into_af_push_pull::<hal::gpio::AF8>(&mut portc.control),
            115_200_u32.bps(),
            self::hal::serial::NewlineMode::Binary,
            &clocks,
            &sc.power_control,
        ),
    };

    writeln!(
        board.usb_uart,
        "{}",
        &BIOS_VERSION[..BIOS_VERSION.len() - 1]
    )
    .unwrap();

    *GLOBAL_BOARD.lock() = Some(board);

    let code: common::OsStartFn = unsafe { ::core::mem::transmute(0x0002_0000) };

    code(&API_CALLS);
}

/// Get the API version this crate implements
pub extern "C" fn api_version_get() -> u32 {
    common::API_VERSION
}

/// Get this BIOS version as a string.
pub extern "C" fn bios_version_get() -> common::ApiString<'static> {
    BIOS_VERSION.into()
}

/// Re-configure the UART. We default to 115200/8N1 on UART1, and the other
/// UARTs default to disabled.
pub extern "C" fn serial_configure(
    device: u8,
    _serial_config: common::serial::Config,
) -> common::Result<()> {
    match device {
        0 => {
            // Configure the USB JTAG/Debug interface
            unimplemented!();
        }
        1 => {
            // Configure the RS232 port
            unimplemented!();
        }
        2 => {
            // Configure the MIDI port
            unimplemented!();
        }
        _ => common::Result::Err(common::Error::InvalidDevice),
    }
}

/// Get infomation about the UARTs available in ths system.
///
/// We have four UARTs, but we only expose three of them. The keyboard/mouse
/// interface UART is kept internal to the BIOS.
pub extern "C" fn serial_get_info(device: u8) -> common::Option<common::serial::DeviceInfo> {
    match device {
        0 => {
            // Device 0 is UART 0, which goes to the USB JTAG/Debug interface.
            // No hardware handshaking.
            common::Option::Some(common::serial::DeviceInfo {
                device_type: common::serial::DeviceType::UsbCdc,
                name: "UART0".into(),
            })
        }
        1 => {
            // Device 1 is UART 1, which is an RS232 interface with RTS/CTS
            common::Option::Some(common::serial::DeviceInfo {
                device_type: common::serial::DeviceType::Rs232,
                name: "UART1".into(),
            })
        }
        2 => {
            // Device 2 is UART 3, which is the MIDI interface. Fixed baud, no
            // hardware handshaking.
            common::Option::Some(common::serial::DeviceInfo {
                device_type: common::serial::DeviceType::Midi,
                name: "UART2".into(),
            })
        }
        _ => common::Option::None,
    }
}

/// Write some text to a UART.
pub extern "C" fn serial_write(
    device: u8,
    data: common::ApiByteSlice,
    _timeout: common::Option<common::Timeout>,
) -> common::Result<usize> {
    if let Some(ref mut board) = *crate::GLOBAL_BOARD.lock() {
        // TODO: Add a timer to the board and use it to handle the timeout.
        // Match on the result of write:
        // * if we get an error, return it.
        // * if we get a WouldBlock, spin (or WFI?).
        // * if we get Ok, carry on.
        let data = data.as_slice();
        match device {
            0 => {
                board.usb_uart.write_all(data);
            }
            1 => {
                board.rs232_uart.write_all(data);
            }
            2 => {
                board.midi_uart.write_all(data);
            }
            _ => {
                return common::Result::Err(common::Error::InvalidDevice);
            }
        }
        common::Result::Ok(data.len())
    } else {
        panic!("HW Lock fail");
    }
}

/// Get the current wall time.
pub extern "C" fn time_get() -> common::Time {
    unimplemented!();
}

/// Set the current wall time.
pub extern "C" fn time_set(new_time: common::Time) {
    unimplemented!();
}

// ===========================================================================
// Private Functions
// ===========================================================================

/// This function is called whenever the BIOS crashes.
#[inline(never)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // TODO: Print the crash info to the console
    loop {
        atomic::compiler_fence(Ordering::SeqCst);
    }
}

// ===========================================================================
// End Of File
// ===========================================================================
