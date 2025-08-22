/* short summary of the memory map in the block diagram */
MEMORY
{
    program_rom (RX)     : ORIGIN = 0x00000000, LENGTH = 128K
    internal_ram (WA)     : ORIGIN = 0x00020000, LENGTH = 32K
    external_ram (WA)     : ORIGIN = 0x80000000, LENGTH = 512M
    ransac_unit_ctrl (W) : ORIGIN = 0xA0000000, LENGTH = 512M
    internal_mmio (W)    : ORIGIN = 0x40000000, LENGTH = 1024M
}

ENTRY(start);
PROVIDE(start = 0x00000000);
SECTIONS
{
    .text : {
        *(.init);
        *(.text);
    } >program_rom
    .internal_data : {
        PROVIDE(global_pointer = .);
        PROVIDE(bss_start = .);
        *(.bss);
        PROVIDE(stack_top = .);
        . += 8k;
        PROVIDE(bss_end = .);
        *(.data);
    } >internal_ram
    .external_data : {
        PROVIDE(external_data_start = .);
        *(.data);
        PROVIDE(external_data_end = .);
    } >external_ram
}