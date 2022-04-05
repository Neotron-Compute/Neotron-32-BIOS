/* Provides information about the memory layout of the device */
MEMORY
{
    /*
     * The TM4C123 has 256 KiB of external Flash Memory. We allow ourselves 128
     * KiB for the BIOS, leaving the rest
     * for the OS and any user applications.
     */
    FLASH : ORIGIN = 0x00000000, LENGTH = 128K
    /*
     * This is the remainder of the 256 KiB flash chip.
     */
    FLASH_OS : ORIGIN = 0x00020000, LENGTH = 256K - 128K
    /*
     * This is the bottom of the SRAM. It's for the OS and Apps.
     */
    RAM_OS : ORIGIN = 0x20000000, LENGTH = 26K
    /*
     * This is the top of the RAM. It's for the BIOS and stack.
     */
    RAM : ORIGIN = 0x20006800, LENGTH = 6K
}

/*
 * Export some symbols to tell the BIOS where it might find the OS.
 */
_flash_os_start = ORIGIN(FLASH_OS);
_flash_os_len = LENGTH(FLASH_OS);
_ram_os_start = ORIGIN(RAM_OS);
_ram_os_len = LENGTH(RAM_OS);



SECTIONS {
    /* ### Neotron OS */
    .flash_os ORIGIN(FLASH_OS) :
    {
        KEEP(*(.flash_os));
    } > FLASH_OS
} INSERT BEFORE .text;
