.section .init
.type start, @function
.type startup_memclear, @function
.type crash_handler, @function
.global start

.equ uartlite_tx_fifo_addr, 0x40600004

start:
    .cfi_startproc
.option push
.option norelax
    la gp, global_pointer
.option pop
    la sp, stack_top
    
    la a0, bss_start
    la a1, bss_end
    li a2, 1
    la a3, memclear_clobbered_stack
    call startup_memclear
memclear_clobbered_stack:

    li a2, 0
    li a3, 0
    la a0, external_data_start
    la a1, external_data_end
    call startup_memclear

    la t0, crash_handler
    csrw mtvec, t0

    call main
    .cfi_endproc

startup_memclear:
    .cfi_startproc
    bgeu a0, a1, startup_memclear_nothing_to_do
    sw zero, 0(a0)
    addi a0, a0, 4
    bltu a0, a1, startup_memclear
startup_memclear_nothing_to_do:
    beqz a2, return_normally
    la sp, stack_top
    addi sp, sp, -4
    sw a3, 0(sp)
return_normally:
    ret
    .cfi_endproc



.p2align 2
/* simple crash handler which just sends mcause over uart.
 */
crash_handler:
    .cfi_startproc
    li t0, uartlite_tx_fifo_addr
    li t1, 3
    csrr t2, mcause
send_mcause_loop:
    sb t2, 0(t0)
    srli t0, t0, 8
    addi t1, t1, -1
    bgez t1, send_mcause_loop
    j crash_handler
    .cfi_endproc