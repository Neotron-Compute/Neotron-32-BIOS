# Neotron 32 BIOS

![Build Status](https://github.com/neotron-compute/Neotron-32-BIOS/workflows/Build/badge.svg "Github Action Build Status")

![Format Status](https://github.com/neotron-compute/Neotron-32-BIOS/workflows/Format/badge.svg "Github Action Format Check Status")

This is the BIOS for the Neotron 32. It implements the Neotron BIOS API (a hardware-abstraction layer used by the Neotron OS).

## Hardware

The Neotron 32 is a revised version of the [Monotron](https://github.com/thejpster/monotron). It uses the same Texas Instruments Tiva-C TM4C123 microcontroller, with just 32 KiB of RAM.

## Status

This BIOS is a work in progress. Bits of the Monotron firmware will be ported over one at a time. The todo list is:

* [x] Get it booting
* [ ] UART
* [ ] SD Card
* [ ] Text Mode (48 x 36)
* [ ] Audio
* [ ] Text Mode (80 x 36)

## Memory Map

The Neotron 32 has 256 KiB of Flash and 32 KiB of RAM. The Flash layout is:

* First 128 KiB of Flash for the BIOS (including the 64 KiB CLUT)
* Next 64 KiB of Flash for the OS
* Next 64 KiB of Flash for the Shell

The RAM layout is flexible - the BIOS takes as much as it needs, then passes the OS the definitions of how much RAM is available and where it is located. The OS then dynamically allocates almost everything it needs from that. The BIOS is also responsible for configuring the stack, and moving the interrupt vector table to RAM.

## Changelog

### Unreleased Changes ([Source](https://github.com/neotron-compute/Neotron-32-BIOS/tree/master))

* Basic UART hello on start-up

## Licence

    Neotron-32-BIOS Copyright (c) The Neotron Developers, 2019

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

## Contribution

Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in the work by you shall be licensed as above, without
any additional terms or conditions.
