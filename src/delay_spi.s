// Line up three SPI channels
// Copyright (c) Jonathan 'theJPster' Pallant
// Licensed under the GNU GPL, version 3 or later
	.section .text.__delay_spi
	.global __delay_spi
	.thumb_func
__delay_spi:
	movs    r0, #0x0084
	movs    r1, #1
	movt    r0, #0x4212
	movs    r2, #0x0000
    movt    r2, #0x0002
	movs    r3, #0x0000
    movt    r3, #0x0004
	str r1, [r0, #0]
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	str r1, [r0, r2]
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	str r1, [r0, r3]
    bx lr
