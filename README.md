# Neotron 32 BIOS

![Build Status](https://github.com/neotron-compute/Neotron-32-BIOS/workflows/Build/badge.svg "Github Action Build Status")

![Format Status](https://github.com/neotron-compute/Neotron-32-BIOS/workflows/Format/badge.svg "Github Action Format Check Status")

This is the BIOS for the Neotron 32. It implements the Neotron BIOS API (a
hardware-abstraction layer used by the Neotron OS).

## Hardware

The Neotron 32 is a revised version of the
[Monotron](https://github.com/thejpster/monotron). It uses the same Texas
Instruments Tiva-C TM4C123 microcontroller, with just 32 KiB of RAM.

## Status

This BIOS is a work in progress. Bits of the Monotron firmware will be ported
over one at a time. The todo list is:

* [x] Get it booting
* [x] USB Serial UART (blocking)
* [x] Text Mode (48 x 36)
* [ ] Time Keeping
* [ ] USB Serial UART (with timeouts)
* [ ] RS-232 UART
* [ ] MIDI UART
* [ ] SD Card
* [ ] Audio Synthesiser
* [ ] PS/2 Keyboard Interface
* [ ] PS/2 Mouse Interface
* [ ] Text Mode (80 x 36)

## Memory Map

The Neotron 32 has 256 KiB of Flash and 32 KiB of RAM. The Flash layout is:

* First 128 KiB of Flash for the BIOS (including the 64 KiB CLUT)
* Next 64 KiB of Flash for the OS
* Next 64 KiB of Flash for the Shell

The RAM layout is flexible - the BIOS takes as much as it needs, then passes
the OS the definitions of how much RAM is available and where it is located.
The OS then dynamically allocates almost everything it needs from that. The
BIOS is also responsible for configuring the stack, and moving the interrupt
vector table to RAM.

## Compilation and Flashing

```console
$ rustup target add thumbv7em-none-eabihf
$ opencd # run this in another terminal
$ cargo run --release # will compile and flash using OpenOCD
```

You will then need to install the OS. Be sure not to erase the BIOS when you
install the OS!

## Changelog

### Unreleased Changes ([Source](https://github.com/neotron-compute/Neotron-32-BIOS/tree/master))

* Initialises text mode video
* Attempts to talk to RTC
* Tracks wall-time during operation
* Jumps to OS

## Licence

    Neotron-32-BIOS Copyright (c) The Neotron Developers, 2020

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

See the full text in [LICENSE.md](./LICENSE.md). Broadly, we interpret this to
mean (and note that we are not lawyers and this is not legal advice) that if
you give someone a Neotron 32, you must also give them one of:

* Complete and corresponding source code (e.g. as a link to your own on-line
  Git repo) for any GPL components (e.g. the BIOS and the OS), as supplied on
  the Neotron 32.
* A written offer to provide complete and corresponding source code on
  request.

If you are not offering Neotron 32 commercially, and you are using an
unmodified upstream version of the source code, then the third option is to
give them:

* A link to the tag/hash on the official Neotron 32 Github repositories -
  https://github.com/Neotron-Compute/Neotron-32-BIOS and
  https://github.com/Neotron-Compute/Neotron-OS.

This is to ensure everyone always has the freedom to access the source code in
their Neotron 32.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in the work by you shall be licensed as above, without
any additional terms or conditions.

## Code of Conduct

Contribution to this crate is organized under the terms of the [Rust Code of
Conduct][CoC], the maintainer of this crate, the [Neotron team][team], promises
to intervene to uphold that code of conduct.

[CoC]: CODE_OF_CONDUCT.md
[team]: https://github.com/Neotron-Compute/meta/blob/master/README.md#developers
