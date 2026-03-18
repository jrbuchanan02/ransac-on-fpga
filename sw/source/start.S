// startup code.

.section .init
.align 2
.globl _start

.option push
.option nopic
.type _start, @function
_start:
.cfi_startproc
.option push
.option norelax
    la gp, global_pointer
.option pop
    // set up some basic error handling for when a trap occurs.
    la t0, _basic_error_handler
    csrw mtvec, t0

    // specifically, clear out mtval and mcause so that we know these values aren't
    // stale.
    csrw mtval, zero
    csrw mcause, zero

    la sp, stack_top
    la a0, bss_start
    la a1, bss_end
    call _startup_memclr

    // clear and disable the hardware performance counters.
    // re-enable them later.
    csrw mcounteren, zero
    csrw mcycle, zero
    csrw minstret, zero

    j main
    // if main returns, loop indefinitely.
loop:
    j loop
.cfi_endproc

_startup_memclr:
    sw zero, (a0)
    addi a0, a0, 4
    bltu a0, a1, _startup_memclr
    ret

.align 2
_basic_error_handler:
    j _basic_error_handler

.option pop // pop nopic option
.end
