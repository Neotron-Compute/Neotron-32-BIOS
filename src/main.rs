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
//! 128 KiB for the OS). The BIOS takes the top 8 KiB of RAM for stack and
//! video ram. The OS is given the bottom 24 KiB of RAM.
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
//!
//! ## Pinout
//!
//! As of PCB revision 1.2.0, the pinout for the Launchpad is:
//!
//! | Pin  |GPIO Name | PCB Net Name | Function                         | Field in BoardInner |
//! |------|----------|--------------|----------------------------------|-|
//! | J1.1 | N/A      |              | 3.3V output from on-board LDO    | |
//! | J1.2 | PB5      |PB5_VGA_SYNC  | VGA Vertical Sync                | `vga.vsync_pin` |
//! | J1.3 | PB0      |PB0_U1RX      | UART RX from KB/MS/JS Controller | `hid_uart` |
//! | J1.4 | PB1      |PB1_U1TX      | UART TX to KB/MS/JS Controller   | `hid_uart` |
//! | J1.5 | PE4      |PE4_AUDIO_L   | Audio Left Channel               | `audio.left` |
//! | J1.6 | PE5      |PE5_AUDIO_R   | Audio Right Channel              | `audio.right` |
//! | J1.7 | PB4      |PB4_VGA_HSYNC | VGA Horizontal Sync              | `vga.hsync_pin` |
//! | J1.8 | PA5      |PA5_SPI_MOSI  | SPI MOSI                         | `spi_bus` |
//! | J1.9 | PA6      |PA6_I2C_SCL   | I2C Bus Clock                    | `i2c_bus` |
//! | J1.10| PA7      |PA7_I2C_SDA   | I2C Bus Data                     | `i2c_bus` |
//! | J2.1 | N/A      |GND           |                                  | |
//! | J2.2 | PB2      |/PB2_STROBE   | Parallel Port Strobe Line        | |
//! | J2.3 | PE0      |PE0_U7RX      | UART RX from WiFi Modem          | |
//! | J2.4 | PF0      |/PF0_IRQ1     | IRQ 1 and Launchpad Button 1     | |
//! | J2.5 | N/A      |/RESET        | Resets the CPU                   | |
//! | J2.6 | PB7      |PB7_VGA_GREEN | VGA Green Channel                | `vga.green_pin` |
//! | J2.7 | PB6      |/PB6_SPI_CS2  | SPI Chip Select 2                | `spi_cs2` |
//! | J2.8 | PA4      |PA4_SPI_MISO  | SPI MISO                         | `spi_bus` |
//! | J2.9 | PA3      |/PA3_SPI_CS0  | SPI Chip Select 0                | `spi_cs0` |
//! | J2.10| PA2      |PA2_SPI_CLK   | SPI Clock                        | `spi_bus` |
//! | J3.1 | N/A      |5V            |                                  | |
//! | J3.2 | N/A      |GND           |                                  | |
//! | J3.3 | PD0      |              |                                  | |
//! | J3.4 | PD1      |              |                                  | |
//! | J3.5 | PD2      |/PD2_IRQ3     | IRQ 3                            | |
//! | J3.6 | PD3      |PD3_VGA_BLUE  | VGA Blue Channel                 | `vga.blue_pin` |
//! | J3.7 | PE1      |PE1_U7TX      | UART TX to WiFi Modem            | |
//! | J3.8 | PE2      |/PE2_SPI_CS3  | SPI Chip Select 3                | `spi_cs3` |
//! | J3.9 | PE3      |/PE3_IRQ2     | IRQ 2                            | |
//! | J3.10| PF1      |PF1_VGA_RED   | VGA Red Channel                  | `vga.red_pin` |
//! | J4.1 | PF2      |              |                                  | |
//! | J4.2 | PF3      |              |                                  | |
//! | J4.3 | PB3      |/PB3_SPI_CS1  | SPI Chip Select 1                | `spi_cs1` |
//! | J4.4 | PC4      |PC4_U1RTS     | UART RTS to RS-232               | |
//! | J4.5 | PC5      |PC5_U1CTS     | UART CTS from RS-232             | |
//! | J4.6 | PC6      |PC6_MIDI_IN   | UART RX from MIDI In             | |
//! | J4.7 | PC7      |PC7_MIDI_OUT  | UART TX to MIDI Out              | |
//! | J4.8 | PD6      |PD6_U2RX      | UART RX from RS-232              | |
//! | J4.9 | PD7      |PD7_U2TX      | UART TX to RS-232                | |
//! | J4.10| PF4      |              | Launchpad Button 2               | |
//!
//! ## License
//!
//!     Neotron-32-BIOS Copyright (c) The Neotron Developers, 2020
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

/// Most of our pins are in Alternate Function Mode 1, so this saves some typing later.
type AltFunc1 = hal::gpio::AlternateFunction<hal::gpio::AF1, hal::gpio::PushPull>;

/// Some of our pins are in Alternate Function Mode 2, so this saves some typing later.
type AltFunc2 = hal::gpio::AlternateFunction<hal::gpio::AF2, hal::gpio::PushPull>;

/// Our I2C are in Alternate Function Mode 3 in Open Drain mode, so this saves some typing later.
type AltFunc3 =
    hal::gpio::AlternateFunction<hal::gpio::AF3, hal::gpio::OpenDrain<hal::gpio::Floating>>;

/// We have two pins in Alternate Function Mode 8, so this saves some typing later.
type AltFunc8 = hal::gpio::AlternateFunction<hal::gpio::AF8, hal::gpio::PushPull>;

/// Soft VGA controller. Bit-bashes 3 bit (8 colour) VGA at 10, 20 or 40 MHz.
#[allow(dead_code)]
pub struct Vga {
    // Timer for generating horizontal sync pulses
    h_timer: hal::tm4c123x::TIMER1,
    // SSI peripheral for generating red pixels
    red: hal::tm4c123x::SSI1,
    // SSI peripheral for generating green pixels
    green: hal::tm4c123x::SSI2,
    // SSI peripheral for generating blue pixels
    blue: hal::tm4c123x::SSI3,
    // VGA Vertical Sync
    vsync_pin: hal::gpio::gpiob::PB5<hal::gpio::Output<hal::gpio::PushPull>>,
    // VGA Horizontal Sync
    hsync_pin: hal::gpio::gpiob::PB4<hal::gpio::Output<hal::gpio::PushPull>>,
    // VGA Red Channel (SSI1 MOSI)
    red_pin: hal::gpio::gpiof::PF1<AltFunc2>,
    // VGA Green Channel (SSI2 MOSI)
    green_pin: hal::gpio::gpiob::PB7<AltFunc2>,
    // VGA Blue Channel (SSI3 MOSI)
    blue_pin: hal::gpio::gpiod::PD3<AltFunc1>,
}

/// Soft Audio Controller
#[allow(dead_code)]
pub struct Audio {
    left: hal::gpio::gpioe::PE4<hal::gpio::Output<hal::gpio::PushPull>>,
    right: hal::gpio::gpioe::PE5<hal::gpio::Output<hal::gpio::PushPull>>,
}

/// This holds our system state - all our HAL drivers, etc.
#[allow(dead_code)]
pub struct BoardInner {
    /// The VGA controller
    vga: Vga,
    /// The UART connected to the on-board USB debug chip
    usb_uart: hal::serial::Serial<
        hal::serial::UART0,
        hal::gpio::gpioa::PA1<AltFunc1>,
        hal::gpio::gpioa::PA0<AltFunc1>,
        (),
        (),
    >,
    /// The UART connected to the Human-Input Device Controller chip (keyboard, mouse and joystick)
    hid_uart: hal::serial::Serial<
        hal::serial::UART7,
        hal::gpio::gpioe::PE1<AltFunc1>,
        hal::gpio::gpioe::PE0<AltFunc1>,
        (),
        (),
    >,
    /// The UART connected to the MIDI interface chip (MIDI Out and MIDI In)
    midi_uart: hal::serial::Serial<
        hal::serial::UART3,
        hal::gpio::gpioc::PC7<AltFunc1>,
        hal::gpio::gpioc::PC6<AltFunc1>,
        (),
        (),
    >,
    /// The UART connected to the RS232 level shifter (with RTS and CTS)
    rs232_uart: hal::serial::Serial<
        hal::serial::UART1,
        hal::gpio::gpiob::PB1<AltFunc1>,
        hal::gpio::gpiob::PB0<AltFunc1>,
        hal::gpio::gpioc::PC4<AltFunc8>,
        hal::gpio::gpioc::PC5<AltFunc8>,
    >,
    /// The Inter-Integrated Circuit Bus (aka the Two Wire Interface). Used to talk to the RTC.
    i2c_bus: hal::i2c::I2c<
        hal::tm4c123x::I2C1,
        (
            hal::gpio::gpioa::PA6<AltFunc3>,
            hal::gpio::gpioa::PA7<AltFunc3>,
        ),
    >,
    /// The Serial Peripheral Interface Bus. Has four chip selects, where CS0
    /// is the SD Card and CS1 is the Parallel Port controller, while CS2, and
    /// CS3 are on the expansion slots.
    spi_bus: hal::spi::Spi<
        hal::tm4c123x::SSI0,
        (
            hal::gpio::gpioa::PA2<AltFunc2>,
            hal::gpio::gpioa::PA4<AltFunc2>,
            hal::gpio::gpioa::PA5<AltFunc2>,
        ),
    >,
    /// Chip-select for the SD card
    spi_cs0: hal::gpio::gpioa::PA3<hal::gpio::Output<hal::gpio::PushPull>>,
    /// Chip-select for the Parallel Port controller
    spi_cs1: hal::gpio::gpiob::PB3<hal::gpio::Output<hal::gpio::PushPull>>,
    /// Chip-select for expansion slot A
    spi_cs2: hal::gpio::gpiob::PB6<hal::gpio::Output<hal::gpio::PushPull>>,
    /// Chip-select for expansion slot B
    spi_cs3: hal::gpio::gpioe::PE2<hal::gpio::Output<hal::gpio::PushPull>>,
    /// IRQ for the Parallel Port controller
    irq1: hal::gpio::gpiof::PF0<hal::gpio::Input<hal::gpio::PullUp>>,
    /// IRQ for expansion slot A
    irq2: hal::gpio::gpioe::PE3<hal::gpio::Input<hal::gpio::PullUp>>,
    /// IRQ for expansion slot B
    irq3: hal::gpio::gpiod::PD2<hal::gpio::Input<hal::gpio::PullUp>>,
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
    let mut portd = p.GPIO_PORTD.split(&sc.power_control);
    let mut porte = p.GPIO_PORTE.split(&sc.power_control);
    let mut portf = p.GPIO_PORTF.split(&sc.power_control);

    let mut board = BoardInner {
        // Soft-VGA output
        vga: Vga {
            h_timer: p.TIMER1,
            red: p.SSI1,
            green: p.SSI2,
            blue: p.SSI3,
            vsync_pin: portb.pb5.into_push_pull_output(),
            hsync_pin: portb.pb4.into_push_pull_output(),
            red_pin: portf
                .pf1
                .into_af_push_pull::<hal::gpio::AF2>(&mut portf.control),
            green_pin: portb
                .pb7
                .into_af_push_pull::<hal::gpio::AF2>(&mut portb.control),
            blue_pin: portd
                .pd3
                .into_af_push_pull::<hal::gpio::AF1>(&mut portd.control),
        },

        // USB Serial UART
        usb_uart: hal::serial::Serial::uart0(
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
        ),

        // MIDI UART
        midi_uart: hal::serial::Serial::uart3(
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
            hal::serial::NewlineMode::Binary,
            &clocks,
            &sc.power_control,
        ),

        // AVR UART
        hid_uart: hal::serial::Serial::uart7(
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
            hal::serial::NewlineMode::Binary,
            &clocks,
            &sc.power_control,
        ),

        // RS-232 UART
        rs232_uart: hal::serial::Serial::uart1(
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
            hal::serial::NewlineMode::Binary,
            &clocks,
            &sc.power_control,
        ),

        // I2C Bus for RTC and Expansion Slots
        i2c_bus: hal::i2c::I2c::i2c1(
            p.I2C1,
            (
                porta
                    .pa6
                    .into_af_open_drain::<hal::gpio::AF3, hal::gpio::Floating>(&mut porta.control),
                porta
                    .pa7
                    .into_af_open_drain::<hal::gpio::AF3, hal::gpio::Floating>(&mut porta.control),
            ),
            tm4c123x_hal::time::Hertz(100_000),
            &clocks,
            &sc.power_control,
        ),

        // SPI Bus for SD Card, I/O Expander, and Expansion Slots. Note this
        // SPI peripheral doesn't understand chip-selects. We need to create a
        // wrapper which does, and which can hand out objects which implement
        // embedded_hal::spi::FullDuplex and automatically enable the
        // appropriate chip-select.
        //
        // In fact, we probably want to change embedded-sdmmc to use the
        // blocking SPI traits so the chip select only toggles once.
        spi_bus: hal::spi::Spi::spi0(
            p.SSI0,
            (
                porta
                    .pa2
                    .into_af_push_pull::<hal::gpio::AF2>(&mut porta.control),
                porta
                    .pa4
                    .into_af_push_pull::<hal::gpio::AF2>(&mut porta.control),
                porta
                    .pa5
                    .into_af_push_pull::<hal::gpio::AF2>(&mut porta.control),
            ),
            embedded_hal::spi::MODE_0,
            250_000.hz(),
            &clocks,
            &sc.power_control,
        ),

        spi_cs0: porta.pa3.into_push_pull_output(),
        spi_cs1: portb.pb3.into_push_pull_output(),
        spi_cs2: portb.pb6.into_push_pull_output(),
        spi_cs3: porte.pe2.into_push_pull_output(),
        irq1: portf.pf0.unlock(&mut portf.control).into_pull_up_input(),
        irq2: porte.pe3.into_pull_up_input(),
        irq3: portd.pd2.into_pull_up_input(),
    };

    writeln!(
        board.usb_uart,
        "{}",
        &BIOS_VERSION[..BIOS_VERSION.len() - 1]
    )
    .unwrap();

    *GLOBAL_BOARD.lock() = Some(board);

    let code: &common::OsStartFn = unsafe { ::core::mem::transmute(0x0002_0000) };

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
            // UART Device 0 is TM4C U0, which goes to the USB JTAG/Debug interface.
            // No hardware handshaking.
            common::Option::Some(common::serial::DeviceInfo {
                device_type: common::serial::DeviceType::UsbCdc,
                name: "UART0".into(),
            })
        }
        1 => {
            // UART Device 1 is TM4C U1, which is an RS232 interface with RTS/CTS
            common::Option::Some(common::serial::DeviceInfo {
                device_type: common::serial::DeviceType::Rs232,
                name: "UART1".into(),
            })
        }
        2 => {
            // UART Device 2 is TM4C U3, which is the MIDI interface. Fixed baud, no
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
pub extern "C" fn time_set(_new_time: common::Time) {
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
