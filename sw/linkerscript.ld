ENTRY(_start);
MEMORY
{
    /* 
       the BRAM connected to the microblaze's ILMB port is only connected to the
       ILMB port. As such, a load to or from that region of memory will always
       fail. For the same reasons, the processor cannot ever find itself
       executing code outside the iram region. This might change in later
       iterations of the processor, but it should work for now.
    */
    iram (X) : ORIGIN = 0, LENGTH = 32k
    dram (RW) : ORIGIN = 0x8000, LENGTH = 32k
    dutreg (RW) : ORIGIN = 0x40000000, LENGTH = 4k
    uartreg (RW) : ORIGIN = 0x40600000, LENGTH = 64k
    ddr3mem (RW) : ORIGIN = 0x80000000, LENGTH = 512M
    bram (RW) : ORIGIN = 0xC0000000, LENGTH = 128k
}

SECTIONS
{
    .text : {
        *(.init);
        *(.text);
    } > iram

    /* This section could probably be set up better. */
    .bss : {
        . = ALIGN(16);  /* RISC-V ABI requirement. */
        PROVIDE(stack_top = .);
        . += 16k; /* 16k of stack */
        PROVIDE(global_pointer = .);
        PROVIDE(bss_end = .);
    } > dram

    .rodata : {
        PROVIDE(bss_start = .);
        *(.bss);
        *(.rodata);
    } > bram

    .data : {
        *(.data);
    } > bram

    /* PROVIDE(uartlite_rxdata = 0x40600000); */
    /* PROVIDE(uartlite_txdata = 0x40600004); */
    /* PROVIDE(uartlite_stat_reg = 0x40600008); */
    /* PROVIDE(uartlite_ctrl_reg = 0x4060000C); */

    /* PROVIDE(ransac_unit_calculation_start_strobe    = 0x40000000); */
    /* PROVIDE(ransac_unit_calculation_abort_strobe    = 0x40000004); */
    /* PROVIDE(ransac_unit_calculation_state_reg       = 0x40000008); */
    /* PROVIDE(ransac_unit_cloud_base_reg              = 0x4000000C); */
    /* PROVIDE(ransac_unit_cloud_size_reg              = 0x40000010); */
    /* PROVIDE(ransac_unit_iteration_to_process_reg    = 0x40000014); */
    /* PROVIDE(ransac_unit_iterations_processed_reg    = 0x40000018); */
    /* PROVIDE(ransac_unit_inlier_threshold_reg        = 0x4000001C); */
    /* PROVIDE(ransac_unit_ground_plane_inliers_reg    = 0x40000020); */
    /* PROVIDE(ransac_unit_ground_plane_normal_x_reg   = 0x40000024); */
    /* PROVIDE(ransac_unit_ground_plane_normal_y_reg   = 0x40000028); */
    /* PROVIDE(ransac_unit_ground_plane_normal_z_reg   = 0x4000002C); */
    /* PROVIDE(ransac_unit_ground_plane_scaled_d_reg   = 0x40000030); */

}