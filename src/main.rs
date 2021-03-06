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
//! 128 KiB for the OS). The BIOS takes the top 6 KiB of RAM for stack and
//! video ram. The OS is given the bottom 26 KiB of RAM.
//!
//! ## Hardware
//!
//! * TM4C123GH6PM System-on-Chip
//!     * Cortex-M4F @ 80 MHz
//!     * 32 KiB SRAM
//!     * 256 KiB Flash ROM
//! * A separate Human-interface Device controller
//!    * This is an AtMega 328 connected to UART7
//!    * It has two PS/2 ports - one for the keyboard, one for the mouse
//!    * It also has two Atari/SEGA compatible joystick ports
//! * Mono/Stereo audio output
//! * SD Card connector (connected to SSI0)
//! * 800 x 600 resolution 8-colour VGA video output (connected to SSI1, SSI2 and SSI3)
//! * MCP7940N I²C battery-backed Real Time Clock
//! * USB Serial interface (connected to UART0 - use the "Debug" USB micro-B on the Launchpad)
//! * RS-232 serial interface
//!    * connected to UART1
//!    * has RTS/CTS hardware handshaking, but not RI, DSR, DTR or DCD
//! * MIDI In and Out (connected to UART3)
//! * PC Printer Port (via an MCP23S17 connected to SSI0)
//!
//! ## Pinout
//!
//! As of PCB revision 1.3.0, the pinout for the Launchpad is:
//!
//! | Pin  |GPIO Name | PCB Net Name  | Dir | Function                         | Field in Board struct |
//! |------|----------|---------------|-----| ---------------------------------|-----------------------|
//! | J1.1 | N/A      | N/A           | Out | 3.3V output from on-board LDO    |                       |
//! | J1.2 | PB5      | PB5_VGA_SYNC  | Out | VGA Vertical Sync                | `vga._vsync_pin`      |
//! | J1.3 | PB0      | PB0_U1RX      | In  | UART RX from KB/MS/JS Controller | `hid_uart`            |
//! | J1.4 | PB1      | PB1_U1TX      | Out | UART TX to KB/MS/JS Controller   | `hid_uart`            |
//! | J1.5 | PE4      | PE4_AUDIO_L   | Out | Audio Left Channel               | `audio.left`          |
//! | J1.6 | PE5      | PE5_AUDIO_R   | Out | Audio Right Channel              | `audio.right`         |
//! | J1.7 | PB4      | PB4_VGA_HSYNC | Out | VGA Horizontal Sync              | `vga._hsync_pin`      |
//! | J1.8 | PA5      | PA5_SPI_COPI  | Out | SPI COPI                         | `sd_interface`        |
//! | J1.9 | PA6      | PA6_I2C_SCL   | Out | I²C Bus Clock                    | `i2c_bus`             |
//! | J1.10| PA7      | PA7_I2C_SDA   | Bi  | I²C Bus Data                     | `i2c_bus`             |
//! | J2.1 | N/A      | GND           | In  |                                  |                       |
//! | J2.2 | PB2      | /PB2_STROBE   | Out | Parallel Port Strobe Line        | `lpt_strobe`          |
//! | J2.3 | PE0      | PE0_U7RX      | In  | UART RX from WiFi Modem          | `wifi_uart`           |
//! | J2.4 | PF0      | /PF0_IRQ1     | In  | IRQ 1 and Launchpad Button 1     | `irq1`                |
//! | J2.5 | N/A      | /RESET        | In  | Resets the CPU                   |                       |
//! | J2.6 | PB7      | PB7_VGA_GREEN | Out | VGA Green Channel                | `vga._green_pin`      |
//! | J2.7 | PB6      | /PB6_SPI_CS2  | Out | SPI Chip Select 2                | `spi_cs2`             |
//! | J2.8 | PA4      | PA4_SPI_MISO  | Out | SPI CIPO                         | `sd_interface`        |
//! | J2.9 | PA3      | /PA3_SPI_CS0  | Out | SPI Chip Select 0                | `sd_interface`        |
//! | J2.10| PA2      | PA2_SPI_CLK   | Out | SPI Clock                        | `sd_interface`        |
//! | J3.1 | N/A      | 5V            | In  |                                  |                       |
//! | J3.2 | N/A      | GND           | In  |                                  |                       |
//! | J3.3 | PD0      | N/A           | --- | [Wired to PB6]                   |                       |
//! | J3.4 | PD1      | N/A           | --- | [Wired to PB7]                   |                       |
//! | J3.5 | PD2      | /PD2_IRQ3     | In  | IRQ 3                            | `irq3`                |
//! | J3.6 | PD3      | PD3_VGA_BLUE  | Out | VGA Blue Channel                 | `vga._blue_pin`       |
//! | J3.7 | PE1      | PE1_U7TX      | Out | UART TX to WiFi Modem            | `wifi_uart`           |
//! | J3.8 | PE2      | /PE2_SPI_CS3  | Out | SPI Chip Select 3                | `spi_cs3`             |
//! | J3.9 | PE3      | /PE3_IRQ2     | In  | IRQ 2                            | `irq2`                |
//! | J3.10| PF1      | PF1_VGA_RED   | Out | VGA Red Channel                  | `vga._red_pin`        |
//! | J4.1 | PF2      | N/A           | Out | Launchpad Green LED              |                       |
//! | J4.2 | PF3      | N/A           | Out | Launchpad Blue LED               |                       |
//! | J4.3 | PB3      | /PB3_SPI_CS1  | Out | SPI Chip Select 1                | `spi_cs1`             |
//! | J4.4 | PC4      | PC4_U1RTS     | Out | UART RTS to RS-232               | `ext_uart`            |
//! | J4.5 | PC5      | PC5_U1CTS     | In  | UART CTS from RS-232             | `ext_uart`            |
//! | J4.6 | PC6      | PC6_MIDI_IN   | In  | UART RX from MIDI In             | `midi_uart`           |
//! | J4.7 | PC7      | PC7_MIDI_OUT  | Out | UART TX to MIDI Out              | `midi_uart`           |
//! | J4.8 | PD6      | PD6_U2RX      | In  | UART RX from RS-232              | `ext_uart`            |
//! | J4.9 | PD7      | PD7_U2TX      | Out | UART TX to RS-232                | `ext_uart`            |
//! | J4.10| PF4      | /PF4_AT_RESET | Out | HID Reset                        | `hid_reset`           |
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

use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};
use cortex_m_rt::entry;
use embedded_hal::{blocking::delay::DelayMs, digital::v2::OutputPin};
use embedded_sdmmc as sdmmc;
use tm4c123x_hal::{
    self as hal, bb,
    gpio::{self, GpioExt},
    sysctl::SysctlExt,
    time::U32Ext,
    tm4c123x::{self as cpu, interrupt},
};
use vga_framebuffer as fb;

use neotron_common_bios as common;

// ===========================================================================
// Types
// ===========================================================================

/// Most of our pins are in Alternate Function Mode 1.
type AltFunc1 = gpio::AlternateFunction<gpio::AF1, gpio::PushPull>;

/// Some of our pins are in Alternate Function Mode 2.
type AltFunc2 = gpio::AlternateFunction<gpio::AF2, gpio::PushPull>;

/// Our I²C Data pin must be in Open Drain mode.
type AltFunc3OD = gpio::AlternateFunction<gpio::AF3, gpio::OpenDrain<gpio::PullUp>>;

/// Our I²C Clock pin must be in Push-Pull mode.
type AltFunc3PP = gpio::AlternateFunction<gpio::AF3, gpio::PushPull>;

/// We have some pins in Alternate Function Mode 7.
type AltFunc7 = gpio::AlternateFunction<gpio::AF7, gpio::PushPull>;

/// We have two pins in Alternate Function Mode 8.
type AltFunc8 = gpio::AlternateFunction<gpio::AF8, gpio::PushPull>;

/// A push-pull output pin
type PushPullOut = gpio::Output<gpio::PushPull>;

/// An input pin with a pull-up
type PullUpInput = gpio::Input<gpio::PullUp>;

/// The type of our general-purpose SPI bus
type SpiDevice = hal::spi::Spi<
    cpu::SSI0,
    (
        gpio::gpioa::PA2<AltFunc2>,
        gpio::gpioa::PA4<AltFunc2>,
        gpio::gpioa::PA5<AltFunc2>,
    ),
>;

/// Soft VGA controller. Bit-bashes 3 bit (8 colour) VGA at 10, 20 or 40 MHz.
pub struct VgaHardware {
    // Timer for generating horizontal sync pulses
    h_timer: cpu::TIMER1,
    /// Used to put the CPU in an idle state just before we draw the video
    /// pixels, to reduce interrupt jitter
    h_timer2: cpu::TIMER2,
    // SSI peripheral for generating red pixels
    red: cpu::SSI1,
    // SSI peripheral for generating green pixels
    green: cpu::SSI2,
    // SSI peripheral for generating blue pixels
    blue: cpu::SSI3,
    // VGA Vertical Sync
    _vsync_pin: gpio::gpiob::PB5<PushPullOut>,
    // VGA Horizontal Sync
    _hsync_pin: gpio::gpiob::PB4<AltFunc7>,
    // VGA Red Channel (SSI1 COPI)
    _red_pin: gpio::gpiof::PF1<AltFunc2>,
    // VGA Green Channel (SSI2 COPI)
    _green_pin: gpio::gpiob::PB7<AltFunc2>,
    // VGA Blue Channel (SSI3 COPI)
    _blue_pin: gpio::gpiod::PD3<AltFunc1>,
}

/// Soft Audio Controller
#[allow(dead_code)]
pub struct Audio {
    left: gpio::gpioe::PE4<PushPullOut>,
    right: gpio::gpioe::PE5<PushPullOut>,
}

/// This holds our system state - all our HAL drivers, etc.
#[allow(dead_code)]
pub struct Board {
    /// The UART connected to the on-board USB debug chip
    usb_uart: hal::serial::Serial<
        hal::serial::UART0,
        gpio::gpioa::PA1<AltFunc1>,
        gpio::gpioa::PA0<AltFunc1>,
        (),
        (),
    >,
    /// The UART connected to the Human-Input Device Controller chip (keyboard, mouse and joystick)
    hid_uart: hal::serial::Serial<
        hal::serial::UART7,
        gpio::gpioe::PE1<AltFunc1>,
        gpio::gpioe::PE0<AltFunc1>,
        (),
        (),
    >,
    /// The UART connected to the MIDI interface chip (MIDI Out and MIDI In)
    midi_uart: hal::serial::Serial<
        hal::serial::UART3,
        gpio::gpioc::PC7<AltFunc1>,
        gpio::gpioc::PC6<AltFunc1>,
        (),
        (),
    >,
    /// The UART connected to the RS232 level shifter (with RTS and CTS)
    ext_uart: hal::serial::Serial<
        hal::serial::UART1,
        gpio::gpiob::PB1<AltFunc1>,
        gpio::gpiob::PB0<AltFunc1>,
        gpio::gpioc::PC4<AltFunc8>,
        gpio::gpioc::PC5<AltFunc8>,
    >,
    /// The UART connected to the ESP01 Wi-Fi Modem
    wifi_uart: hal::serial::Serial<
        hal::serial::UART2,
        gpio::gpiod::PD7<AltFunc1>,
        gpio::gpiod::PD6<AltFunc1>,
        (),
        (),
    >,
    /// The Inter-Integrated Circuit Bus (aka the Two Wire Interface). Used to talk to the RTC.
    i2c_bus: hal::i2c::I2c<cpu::I2C1, (gpio::gpioa::PA6<AltFunc3PP>, gpio::gpioa::PA7<AltFunc3OD>)>,
    /// Currently, the SD/MMC controller device 'owns' our SPI bus. This is OK
    /// though, as we can 'borrow' the SPI device whenever we want. It'll only
    /// be a problem if we get another driver that wants to own the bus. We
    /// could pass in a proxy object, but the SD/MMC controller only uses the
    /// single-byte `FullDuplex` trait, rather than the blocking trait (to
    /// avoid needing to allocate big buffers) and we wouldn't want to either
    /// toggle the CSx pin for each word, nor grab the spin lock for each
    /// word.
    sd_interface: sdmmc::SdMmcSpi<SpiDevice, gpio::gpioa::PA3<PushPullOut>>,
    /// Chip-select for the PC Printer port interface controller
    spi_cs1: gpio::gpiob::PB3<PushPullOut>,
    /// Chip-select for expansion slot A
    spi_cs2: gpio::gpiob::PB6<PushPullOut>,
    /// Chip-select for expansion slot B
    spi_cs3: gpio::gpioe::PE2<PushPullOut>,
    /// IRQ for the Parallel Port controller
    irq1: gpio::gpiof::PF0<PullUpInput>,
    /// IRQ for expansion slot A
    irq2: gpio::gpioe::PE3<PullUpInput>,
    /// IRQ for expansion slot B
    irq3: gpio::gpiod::PD2<PullUpInput>,
    /// Our soft audio controller
    audio: Audio,
    /// The strobe pin for the PC Printer interface.
    /// We need 17 pins for this interface and the MCP23S17 only has
    /// 16 pins, so wired the strobe pin straight to the TM4C.
    lpt_strobe: gpio::gpiob::PB2<PushPullOut>,
    /// We take this low to reset the HID controller. It can also be pulled
    /// hard low by Button 2 on the Launchpad.
    hid_reset: gpio::gpiof::PF4<gpio::Output<gpio::OpenDrain<gpio::Floating>>>,
    /// Our Cortex-M SysTick peripheral wrapped up in a utility that can do busy-wait delays
    delay: hal::delay::Delay,
}

/// Makes it possible to share an I²C bus with a driver that wants to own the
/// bus.
struct I2cBus<'a, T>(&'a mut T)
where
    T: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead;

// ===========================================================================
// Static Variables and Constants
// ===========================================================================

/// The BIOS version string
static BIOS_VERSION: &str = concat!("Neotron 32 BIOS, version ", env!("CARGO_PKG_VERSION"), "\0");

/// The table of API calls we provide the OS
static API_CALLS: common::Api = common::Api {
    api_version_get,
    bios_version_get,
    serial_configure,
    serial_get_info,
    serial_write,
    time_get,
    time_set,
    video_memory_info_get,
};

/// Holds the global state for the motherboard
static GLOBAL_BOARD: spin::Mutex<Option<Board>> = spin::Mutex::new(None);

/// This is both the video renderer state, and the buffer into which text
/// characters are drawn. These should probably be two separate things.
static mut FRAMEBUFFER: fb::FrameBuffer = fb::FrameBuffer::new();

/// Hardware we need for driving the VGA output
static mut VGA_HW: Option<VgaHardware> = None;

/// This is a magic value to make the video timing work.
const ISR_LATENCY: u32 = 24;

/// This is a magic value for a pre-ISR which puts the CPU into a known state
/// before our pixel start ISR.
const ISR_LATENCY_WARMUP: u32 = 3;

/// This is how long we hold the HID controller in reset
const HID_RESET_TIME_MS: u32 = 100;

// ===========================================================================
// Macros
// ===========================================================================

/// Prints to the screen
#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => {
        {
            use core::fmt::Write as _;
            write!(unsafe { &mut FRAMEBUFFER }, $($arg)*).unwrap();
        }
    };
}

/// Prints to the screen and puts a new-line on the end
#[macro_export]
macro_rules! println {
    () => (print!("\n"));
    ($($arg:tt)*) => {
        {
            use core::fmt::Write as _;
            writeln!(unsafe { &mut FRAMEBUFFER }, $($arg)*).unwrap();
        }
    };
}

// ===========================================================================
// Public Functions
// ===========================================================================

/// Entry point to the BIOS. This is called from the reset vector by
/// `cortex-m-rt`.
#[entry]
fn main() -> ! {
    // Grab the peripheral singleton from the TM4C123 HAL.
    let p = hal::Peripherals::take().unwrap();
    // Grab the common Cortex-M peripherals (like NVIC)
    let mut cp = hal::CorePeripherals::take().unwrap();

    // Set the system up for 80 MHz
    let mut sc = p.SYSCTL.constrain();
    sc.clock_setup.oscillator = hal::sysctl::Oscillator::Main(
        hal::sysctl::CrystalFrequency::_16mhz,
        hal::sysctl::SystemClock::UsePll(hal::sysctl::PllOutputFrequency::_80_00mhz),
    );

    // This object records what speed we set the chip to. Things like the UART
    // setup need it to calculate baud rate registers correctly.
    let clocks = sc.clock_setup.freeze();

    // Make Timer1A (start of line) lower priority than Timer1B (clocking out
    // data) so that it can be interrupted. Timer2A is between the two.
    // Priorities go from 0*16 (most urgent) to 15*16 (least urgent).
    unsafe {
        cortex_m::peripheral::NVIC::unmask(cpu::Interrupt::TIMER1A);
        cortex_m::peripheral::NVIC::unmask(cpu::Interrupt::TIMER1B);
        cortex_m::peripheral::NVIC::unmask(cpu::Interrupt::TIMER2A);
        cp.NVIC.set_priority(cpu::Interrupt::TIMER1A, 8 * 16);
        cp.NVIC.set_priority(cpu::Interrupt::TIMER2A, 6 * 16);
        cp.NVIC.set_priority(cpu::Interrupt::TIMER1B, 4 * 16);
    }

    enable(hal::sysctl::Domain::Timer1, &mut sc.power_control);
    enable(hal::sysctl::Domain::Timer2, &mut sc.power_control);
    enable(hal::sysctl::Domain::Ssi0, &mut sc.power_control);
    enable(hal::sysctl::Domain::Ssi1, &mut sc.power_control);
    enable(hal::sysctl::Domain::Ssi2, &mut sc.power_control);
    enable(hal::sysctl::Domain::Ssi3, &mut sc.power_control);
    enable(hal::sysctl::Domain::Pwm0, &mut sc.power_control);

    // Pins. So many pins.
    let mut porta = p.GPIO_PORTA.split(&sc.power_control);
    let mut portb = p.GPIO_PORTB.split(&sc.power_control);
    let mut portc = p.GPIO_PORTC.split(&sc.power_control);
    let mut portd = p.GPIO_PORTD.split(&sc.power_control);
    let mut porte = p.GPIO_PORTE.split(&sc.power_control);
    let mut portf = p.GPIO_PORTF.split(&sc.power_control);

    // Construct the bumper object holding all the drivers.
    let mut board = Board {
        // USB Serial UART
        usb_uart: hal::serial::Serial::uart0(
            p.UART0,
            porta.pa1.into_af_push_pull::<gpio::AF1>(&mut porta.control),
            porta.pa0.into_af_push_pull::<gpio::AF1>(&mut porta.control),
            (),
            (),
            115_200_u32.bps(),
            hal::serial::NewlineMode::SwapLFtoCRLF,
            &clocks,
            &sc.power_control,
        ),

        // RS-232 UART
        ext_uart: hal::serial::Serial::uart1(
            p.UART1,
            portb.pb1.into_af_push_pull::<gpio::AF1>(&mut portb.control),
            portb.pb0.into_af_push_pull::<gpio::AF1>(&mut portb.control),
            portc.pc4.into_af_push_pull::<gpio::AF8>(&mut portc.control),
            portc.pc5.into_af_push_pull::<gpio::AF8>(&mut portc.control),
            115_200_u32.bps(),
            hal::serial::NewlineMode::Binary,
            &clocks,
            &sc.power_control,
        ),

        // ESP UART
        wifi_uart: hal::serial::Serial::uart2(
            p.UART2,
            portd
                .pd7
                .unlock(&mut portd.control)
                .into_af_push_pull::<gpio::AF1>(&mut portd.control),
            portd.pd6.into_af_push_pull::<gpio::AF1>(&mut portd.control),
            (),
            (),
            115200_u32.bps(),
            hal::serial::NewlineMode::Binary,
            &clocks,
            &sc.power_control,
        ),

        // MIDI UART
        midi_uart: hal::serial::Serial::uart3(
            p.UART3,
            portc.pc7.into_af_push_pull::<gpio::AF1>(&mut portc.control),
            portc.pc6.into_af_push_pull::<gpio::AF1>(&mut portc.control),
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
            porte.pe1.into_af_push_pull::<gpio::AF1>(&mut porte.control),
            porte.pe0.into_af_push_pull::<gpio::AF1>(&mut porte.control),
            (),
            (),
            19200_u32.bps(),
            hal::serial::NewlineMode::Binary,
            &clocks,
            &sc.power_control,
        ),

        // I²C bus for RTC and Expansion Slots. SDA is open-drain but SCL isn't (see the TM4C123 TRM page 657)
        i2c_bus: hal::i2c::I2c::i2c1(
            p.I2C1,
            (
                porta.pa6.into_af_push_pull::<gpio::AF3>(&mut porta.control),
                porta
                    .pa7
                    .into_af_open_drain::<gpio::AF3, gpio::PullUp>(&mut porta.control),
            ),
            hal::time::Hertz(100_000),
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
        sd_interface: sdmmc::SdMmcSpi::new(
            hal::spi::Spi::spi0(
                p.SSI0,
                (
                    porta.pa2.into_af_push_pull::<gpio::AF2>(&mut porta.control),
                    porta.pa4.into_af_push_pull::<gpio::AF2>(&mut porta.control),
                    porta.pa5.into_af_push_pull::<gpio::AF2>(&mut porta.control),
                ),
                embedded_hal::spi::MODE_0,
                // Defaut to a really low speed for SD card initialisation
                250_000.hz(),
                &clocks,
                &sc.power_control,
            ),
            porta.pa3.into_push_pull_output(),
        ),
        spi_cs1: portb.pb3.into_push_pull_output(),
        spi_cs2: portb.pb6.into_push_pull_output(),
        spi_cs3: porte.pe2.into_push_pull_output(),
        irq1: portf.pf0.unlock(&mut portf.control).into_pull_up_input(),
        irq2: porte.pe3.into_pull_up_input(),
        irq3: portd.pd2.into_pull_up_input(),
        audio: Audio {
            left: porte.pe4.into_push_pull_output(),
            right: porte.pe5.into_push_pull_output(),
        },
        lpt_strobe: portb.pb2.into_push_pull_output(),
        hid_reset: portf.pf4.into_open_drain_output(),
        delay: hal::delay::Delay::new(cp.SYST, &clocks),
    };

    unsafe {
        // Soft-VGA output
        let mut vga_hw = VgaHardware {
            h_timer: p.TIMER1,
            h_timer2: p.TIMER2,
            red: p.SSI1,
            green: p.SSI2,
            blue: p.SSI3,
            _vsync_pin: portb.pb5.into_push_pull_output(),
            _hsync_pin: portb.pb4.into_af_push_pull::<gpio::AF7>(&mut portb.control),
            _red_pin: portf.pf1.into_af_push_pull::<gpio::AF2>(&mut portf.control),
            _green_pin: portb.pb7.into_af_push_pull::<gpio::AF2>(&mut portb.control),
            _blue_pin: portd.pd3.into_af_push_pull::<gpio::AF1>(&mut portd.control),
        };
        FRAMEBUFFER.init(|m| vga_hw.configure(m));
        FRAMEBUFFER.set_cursor_visible(false);
        VGA_HW = Some(vga_hw);
    }

    // Say hello to the nice users.
    println!("{} booting...", &BIOS_VERSION[..BIOS_VERSION.len() - 1]);

    extern "C" {
        static _start_osram_sym: u32;
        static _end_osram_sym: u32;
        static _start_os_flash_sym: u32;
        static _end_os_flash_sym: u32;
    }

    // Note:(unsafe) - only taking address of external static
    let start_addr = unsafe { &_start_osram_sym as &u32 as *const u32 as usize };
    let end_addr = unsafe { &_end_osram_sym as &u32 as *const u32 as usize };
    let length = end_addr - start_addr;
    println!(
        "OSRAM 0x{:08x}..0x{:08x} = {} bytes",
        start_addr, end_addr, length
    );

    // Fetch the time from the RTC. It might be wrong, but our internal time
    // is *definitely* wrong, so this never makes things worse.
    load_time(&mut board);

    // Reset the HID controller
    let _ = board.hid_reset.set_low();
    board.delay_ms(HID_RESET_TIME_MS);
    let _ = board.hid_reset.set_high();

    // Stash the big object with all the driver state somewhere we can access
    // it from the BIOS callback functions.
    *GLOBAL_BOARD.lock() = Some(board);

    // We assume the OS can initialise its own memory, as we have no idea how
    // much it's using so we can't initialise the memory for it.

    // The first four bytes of OS flash must be the address of the start
    // function. If it looks OK, we call it.

    let start_os_flash_address = unsafe { &_start_os_flash_sym as *const u32 as usize };
    let os_entry_address = unsafe { _start_os_flash_sym } as usize;
    let end_os_flash_address = unsafe { &_end_os_flash_sym as *const u32 as usize };

    if (os_entry_address >= start_os_flash_address) && (os_entry_address <= end_os_flash_address) {
        // On this BIOS, the flash split between BIOS and OS is fixed. This value
        // must match the BIOS linker script and the OS linker script.
        let code: &common::OsStartFn = unsafe {
            &*(&_start_os_flash_sym as *const u32
                as *const for<'r> extern "C" fn(&'r neotron_common_bios::Api) -> !)
        };
        code(&API_CALLS);
    } else {
        println!(
            "No OS found at 0x{:08x}..0x{:08x} (0x{:08x} is bad)",
            start_os_flash_address, end_os_flash_address, os_entry_address
        );
        loop {
            cortex_m::asm::wfi();
        }
    }
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
            // TODO!
            common::Result::Err(common::Error::Unimplemented)
        }
        1 => {
            // Configure the RS232 port
            // TODO!
            common::Result::Err(common::Error::Unimplemented)
        }
        2 => {
            // Configure the MIDI port
            // TODO!
            common::Result::Err(common::Error::Unimplemented)
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
        let data = data.as_slice();
        match device {
            0 => {
                board.usb_uart.write_all(data);
            }
            1 => {
                board.ext_uart.write_all(data);
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
    // TODO - read from hib::RTCC module
    common::Time {
        frames_since_second: 0,
        seconds_since_epoch: 0,
    }
}

/// Set the current wall time.
pub extern "C" fn time_set(_new_time: common::Time) {
    // TODO - write to hib::RTCC module
    // TODO: Write the new time to the RTC (which is only accurate to the second)
}

/// Gets information about the memory-mapped text buffer.
pub extern "C" fn video_memory_info_get(
    address: &mut *mut u8,
    width_cols: &mut u8,
    height_rows: &mut u8,
) {
    *address = unsafe { FRAMEBUFFER.get_address() };
    *width_cols = 48;
    *height_rows = 36;
}

// ===========================================================================
// Private Functions
// ===========================================================================

/// This function is called whenever the BIOS crashes.
#[inline(never)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // TODO: Print the crash info to the console
    // TODO: Flash an LED?
    // TODO:
    loop {
        atomic::compiler_fence(Ordering::SeqCst);
    }
}

/// Grab the current time from the MCP7940N Real-Time Clock (RTC).
///
/// We create a new instance of the mcp794xx driver, bind it to the I²C bus
/// and then get the time from the chip. Because the driver wants to 'own' the
/// I²C bus object it is given, but we can't give it our I²C bus because it
/// belongs to a larger structure we're only (mutably) borrowing, we create an
/// `I2cBus` object as a proxy. It's ugly, but it works.
///
/// This function is going to be fairly slow, and is only accurate to the
/// nearest second, so don't do it very often. Once, at start-up, is probably
/// fine as we can get the CPU to keep track of time.
///
/// Before anyone notes that there is already a battery-backed RTC on the
/// TM4C123 chip itself - yes there is, but the TM4C Launchpad doesn't bring
/// out the battery backup power pins anywhere useful, making the RTC useless.
fn load_time(board: &mut Board) {
    use chrono::prelude::*;
    println!("Checking for time...");
    use mcp794xx::Rtcc;
    let bus = I2cBus(&mut board.i2c_bus);
    let mut rtc = mcp794xx::Mcp794xx::new_mcp7940n(bus);
    let mut dt = match rtc.get_datetime() {
        Ok(dt) => dt,
        Err(e) => {
            println!("Error reading RTC: {:?}", e);
            return;
        }
    };
    if dt.year() == 2001 || rtc.has_power_failed().unwrap_or(true) {
        // Clock has been reset - setting the time starts the clock running
        // again
        println!("RTC Battery Failed! Please set time/date.");
        dt = NaiveDate::from_ymd(2020, 3, 1).and_hms(0, 0, 0);
        if let Err(e) = rtc.clear_power_failed() {
            println!("RTC Error: {:?}", e);
        }
        if let Err(e) = rtc.enable_backup_battery_power() {
            println!("RTC Error {:?}", e);
        }
        if let Err(e) = rtc.set_datetime(&dt) {
            println!("RTC Error {:?}", e);
        }
        if let Err(e) = rtc.enable() {
            println!("RTC Error {:?}", e);
        }
    }

    let posix_time = dt.timestamp();
    println!("Time is {}", dt);
    let our_epoch = Utc.ymd(2001, 1, 1).and_hms(0, 0, 0).timestamp();
    let seconds_since_epoch = (posix_time - our_epoch) as u32;
    time_set(common::Time {
        seconds_since_epoch,
        frames_since_second: 0,
    });
}

impl<'a, T> embedded_hal::blocking::i2c::Write for I2cBus<'a, T>
where
    T: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
{
    type Error = <T as embedded_hal::blocking::i2c::Write>::Error;
    fn write(&mut self, register: u8, buffer: &[u8]) -> Result<(), Self::Error> {
        self.0.write(register, buffer)
    }
}

impl<'a, T> embedded_hal::blocking::i2c::WriteRead for I2cBus<'a, T>
where
    T: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
{
    type Error = <T as embedded_hal::blocking::i2c::WriteRead>::Error;

    fn write_read(
        &mut self,
        register: u8,
        out_buffer: &[u8],
        in_buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.0.write_read(register, out_buffer, in_buffer)
    }
}

impl Board {
    /// Delay (busy-wait) for a specified number of milliseconds
    fn delay_ms(&mut self, period_ms: u32) {
        self.delay.delay_ms(period_ms);
    }
}

impl VgaHardware {
    /// Set up the SPI peripherals to clock out RGB video with the given timings.
    fn configure(&mut self, mode_info: &fb::ModeInfo) {
        // Need to configure SSI1, SSI2 and SSI3 at `clock_rate` Hz.
        // First up, we disable all three.
        self.red.cr1.modify(|_, w| w.sse().clear_bit());
        self.blue.cr1.modify(|_, w| w.sse().clear_bit());
        self.green.cr1.modify(|_, w| w.sse().clear_bit());
        // SSIClk = SysClk / (CPSDVSR * (1 + SCR))
        // e.g. 20 MHz = 80 MHz / (4 * (1 + 0))
        // CPSDVSR = 4 ------------^
        // SCR = 0 -------------------------^
        let ratio = 80_000_000 / mode_info.clock_rate;
        // For all sensible divisors of 80 MHz, we want SCR = 0.
        self.red
            .cpsr
            .write(|w| unsafe { w.cpsdvsr().bits(ratio as u8) });
        self.blue
            .cpsr
            .write(|w| unsafe { w.cpsdvsr().bits(ratio as u8) });
        self.green
            .cpsr
            .write(|w| unsafe { w.cpsdvsr().bits(ratio as u8) });
        // Each channel needs to clock out 8 bit words, with the correct
        // phase/polarity.
        self.red.cr0.write(|w| {
            w.dss()._8();
            w.frf().moto();
            w.spo().clear_bit();
            w.sph().set_bit();
            w
        });
        self.blue.cr0.write(|w| {
            w.dss()._8();
            w.frf().moto();
            w.spo().clear_bit();
            w.sph().set_bit();
            w
        });
        self.green.cr0.write(|w| {
            w.dss()._8();
            w.frf().moto();
            w.spo().clear_bit();
            w.sph().set_bit();
            w
        });
        // Set clock source to sysclk
        self.red.cc.modify(|_, w| w.cs().syspll());
        self.blue.cc.modify(|_, w| w.cs().syspll());
        self.green.cc.modify(|_, w| w.cs().syspll());

        // Configure Timer1A for h-sync and Timer1B for line trigger
        self.h_timer.ctl.modify(|_, w| {
            w.taen().clear_bit();
            w.tben().clear_bit();
            w
        });
        // Timer runs in dual-16-bit mode.
        // Timer A is periodic with PWM enabled.
        self.h_timer.cfg.modify(|_, w| w.cfg()._16_bit());
        self.h_timer.tamr.modify(|_, w| {
            w.taams().set_bit();
            w.tacmr().clear_bit();
            w.tapwmie().set_bit();
            w.tamr().period();
            w
        });
        // Timer B is periodic with PWM enabled.
        self.h_timer.tbmr.modify(|_, w| {
            w.tbams().set_bit();
            w.tbcmr().clear_bit();
            w.tbmr().period();
            w.tbpwmie().set_bit();
            w
        });
        self.h_timer.ctl.modify(|_, w| {
            // Trigger Timer A capture on rising edge (i.e. line start)
            w.tapwml().clear_bit();
            // Trigger Timer B capture on falling edge (i.e. data start)
            w.tbpwml().set_bit();
            w
        });
        // We're counting down in PWM mode, so start at the end
        // We start 16 pixels early
        let convert_to_clockset = |i: u32| -> u32 { (ratio * i) - 1 };
        self.h_timer
            .tailr
            .modify(|_, w| unsafe { w.bits(convert_to_clockset(mode_info.width)) });
        self.h_timer
            .tbilr
            .modify(|_, w| unsafe { w.bits(convert_to_clockset(mode_info.width)) });
        self.h_timer.tamatchr.modify(|_, w| unsafe {
            w.bits(convert_to_clockset(mode_info.width - mode_info.sync_end))
        });
        // Counting down, so adding here makes it earlier
        self.h_timer.tbmatchr.modify(|_, w| unsafe {
            w.bits(convert_to_clockset(
                ISR_LATENCY + mode_info.width - mode_info.line_start,
            ))
        });
        self.h_timer.imr.modify(|_, w| {
            w.caeim().set_bit(); // Timer1A fires at start of line
            w.cbeim().set_bit(); // Timer1B fires at start of data
            w
        });

        // Configure Timer2A to run just before Timer 1B
        self.h_timer2.ctl.modify(|_, w| {
            w.taen().clear_bit();
            w.tben().clear_bit();
            w
        });
        self.h_timer2.cfg.modify(|_, w| w.cfg()._16_bit());
        self.h_timer2.tamr.modify(|_, w| {
            w.taams().set_bit();
            w.tacmr().clear_bit();
            w.tapwmie().set_bit();
            w.tamr().period();
            w
        });
        self.h_timer2.ctl.modify(|_, w| {
            // Trigger Timer A capture on falling edge (i.e. just before data start)
            w.tapwml().set_bit();
            w
        });
        // We're counting down in PWM mode, so start at the end
        // We start a few pixels before Timer1B
        self.h_timer2
            .tailr
            .modify(|_, w| unsafe { w.bits(convert_to_clockset(mode_info.width)) });
        // Counting down, so adding here makes it earlier
        self.h_timer2.tamatchr.modify(|_, w| unsafe {
            w.bits(convert_to_clockset(
                ISR_LATENCY + ISR_LATENCY_WARMUP + mode_info.width - mode_info.line_start,
            ))
        });
        self.h_timer2.imr.modify(|_, w| {
            w.caeim().set_bit(); // Timer1A fires just before at start of data
            w
        });

        // Clear interrupts
        self.h_timer.icr.write(|w| {
            w.tbmcint().set_bit();
            w.tbtocint().set_bit();
            w
        });

        // Clear interrupts
        self.h_timer2.icr.write(|w| {
            w.tbmcint().set_bit();
            w.tbtocint().set_bit();
            w
        });

        self.h_timer2.ctl.modify(|_, w| {
            w.taen().set_bit();
            w
        });

        self.h_timer.ctl.modify(|_, w| {
            w.taen().set_bit();
            w.tben().set_bit();
            w
        });
    }
}

impl fb::Hardware for VgaHardware {
    /// Called when V-Sync needs to be high.
    fn vsync_on(&mut self) {
        let gpio = unsafe { &*cpu::GPIO_PORTB::ptr() };
        unsafe { bb::change_bit(&gpio.data, 5, true) };
    }

    /// Called when V-Sync needs to be low.
    fn vsync_off(&mut self) {
        let gpio = unsafe { &*cpu::GPIO_PORTB::ptr() };
        unsafe { bb::change_bit(&gpio.data, 5, false) };
    }

    /// Write pixels straight to FIFOs
    fn write_pixels(&mut self, xrgb: vga_framebuffer::XRGBColour) {
        let ssi_r = unsafe { &*cpu::SSI1::ptr() };
        let ssi_g = unsafe { &*cpu::SSI2::ptr() };
        let ssi_b = unsafe { &*cpu::SSI3::ptr() };
        while (ssi_r.sr.read().bits() & 0x02) == 0 {}
        ssi_r.dr.write(|w| unsafe { w.bits(xrgb.red()) });
        ssi_g.dr.write(|w| unsafe { w.bits(xrgb.green()) });
        ssi_b.dr.write(|w| unsafe { w.bits(xrgb.blue()) });
    }
}

/// Power on a peripheral and then reset it.
fn enable(p: hal::sysctl::Domain, sc: &mut hal::sysctl::PowerControl) {
    hal::sysctl::control_power(
        sc,
        p,
        hal::sysctl::RunMode::Run,
        hal::sysctl::PowerState::On,
    );
    hal::sysctl::control_power(
        sc,
        p,
        hal::sysctl::RunMode::Sleep,
        hal::sysctl::PowerState::On,
    );
    hal::sysctl::reset(sc, p);
}

// ===========================================================================
// Interrupts
// ===========================================================================

/// Called just before Timer1B, which gives Timer1B lower interrupt jitter.
#[interrupt]
fn TIMER2A() {
    unsafe {
        cortex_m::asm::wfi();
        let timer = &*cpu::TIMER2::ptr();
        timer.icr.write(|w| w.caecint().set_bit());
    }
}

/// Called on start of sync pulse (end of front porch). This is unsafe because
/// we mutate statics while the main thread might be using them at the same
/// time (technically this is undefined behaviour).
#[interrupt]
fn TIMER1A() {
    unsafe {
        // let pwm = &*cpu::PWM0::ptr();
        let ssi_r = &*cpu::SSI1::ptr();
        let ssi_g = &*cpu::SSI2::ptr();
        let ssi_b = &*cpu::SSI3::ptr();
        // static mut NEXT_SAMPLE: u8 = 128;
        // Play the previously calculated and buffered audio sample. We play
        // it here as the video generation is variable-length, and we don't
        // want audio jitter.
        // pwm._2_cmpa.write(|w| w.compa().bits(NEXT_SAMPLE as u16));
        // Disable the SPIs as we don't want pixels yet
        ssi_r.cr1.modify(|_, w| w.sse().clear_bit());
        ssi_g.cr1.modify(|_, w| w.sse().clear_bit());
        ssi_b.cr1.modify(|_, w| w.sse().clear_bit());
        // Pre-load red with 2 bytes and green 1 with (they start early so we can line them up)
        ssi_r.dr.write(|w| w.data().bits(0));
        ssi_r.dr.write(|w| w.data().bits(0));
        ssi_g.dr.write(|w| w.data().bits(0));
        if let Some(ref mut hw) = VGA_HW {
            // Run the draw routine
            FRAMEBUFFER.isr_sol(hw);
            // Run the audio routine
            // NEXT_SAMPLE = G_SYNTH.next().into();
        }
        // Clear timer A interrupt
        let timer = &*cpu::TIMER1::ptr();
        timer.icr.write(|w| w.caecint().set_bit());
    }
}

/// Called on start of pixel data (end of back porch)
#[interrupt]
fn TIMER1B() {
    unsafe {
        extern "C" {
            fn __delay_spi();
        }
        // Activate the three SPI FIFOs exactly 32 clock cycles (or 8 pixels)
        // apart. This gets the colour video lined up, as we preload the red
        // channel with 0x00 0x00 and the green channel with 0x00.
        __delay_spi();
        // Clear timer B interrupt
        let timer = &*cpu::TIMER1::ptr();
        timer.icr.write(|w| w.cbecint().set_bit());
    }
}

// ===========================================================================
// End Of File
// ===========================================================================
