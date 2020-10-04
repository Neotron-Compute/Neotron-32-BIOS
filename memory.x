/* Provides information about the memory layout of the device */
MEMORY
{
    /* The BIOS gets the first 128 KiB of Flash, leaving 128 KiB for the OS */
    FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 128K
    /* The BIOS gets the top 6 KiB of SRAM (including the Stack), leaving 26 KiB for the OS */
    RAM   (rwx) : ORIGIN = 0x20006800, LENGTH = 6K
    /* This is where the OS and any applications are loaded */
    OSRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 26K
}

/* This is where the call stack will be allocated. */
/* The stack is of the full descending type. */
/* You may want to use this variable to locate the call stack and static
   variables in different memory regions. Below is shown the default value */
/* _stack_start = ORIGIN(RAM) + LENGTH(RAM); */

/* You can use this symbol to customize the location of the .text section */
/* If omitted the .text section will be placed right after the .vector_table
   section */
/* This is required only on microcontrollers that store some configuration right
   after the vector table */
/* _stext = ORIGIN(FLASH) + 0x400; */

_start_os_flash_sym = ORIGIN(FLASH) + LENGTH(FLASH);
_end_os_flash_sym = ORIGIN(FLASH) + 256K;

_start_osram_sym = ORIGIN(OSRAM);
_end_osram_sym = ORIGIN(OSRAM) + LENGTH(OSRAM);

/* Example of putting non-initialized variables into custom RAM locations. */
/* This assumes you have defined a region RAM2 above, and in the Rust
   sources added the attribute `#[link_section = ".ram2bss"]` to the data
   you want to place there. */
/* Note that the section will not be zero-initialized by the runtime! */
/* SECTIONS {
     .ram2bss (NOLOAD) : ALIGN(4) {
       *(.ram2bss);
       . = ALIGN(4);
     } > RAM2
   } INSERT AFTER .bss;
*/
