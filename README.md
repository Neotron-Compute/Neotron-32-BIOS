# Neotron 32 BIOS

This is the BIOS for the Neotron 32. It implements the Neotron BIOS API (a hardware-abstraction layer used by the Neotron OS).

## Hardware

The Neotron 32 is a revised version of the [Monotron](https://github.com/thejpster/monotron). It uses the same Texas Instruments Tiva-C TM4C123 microcontroller, with just 32 KiB of RAM.

## Status

This BIOS is a work in progress. Bits of the Monotron firmware will be ported over one at a time. The todo list is:

* [ ] Get it booting
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
