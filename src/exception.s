.macro CODE_BEGIN name
    .section .text.\name, "ax", %progbits
    .global \name
    .type \name, %function
    .align 2
    .cfi_startproc
\name:
.endm

.macro CODE_END
    .cfi_endproc
.endm

// Called by crt0 when the args at the time of entry indicate an exception occured.
CODE_BEGIN __libnx_exception_entry

    cmp x1, #0
    beq __libnx_exception_entry_abort__

    adrp x2, GDBStub_exceptiondump
    add  x2, x2, #:lo12:GDBStub_exceptiondump

    // Save x1, x2
    mov x5, x1
    mov x6, x2

    // Load from ThreadExceptionFrameA64 to ThreadExceptionDump_t

    // error_desc
    str w0, [x2], #4
    // pad
    str wzr, [x2], #4
    str wzr, [x2], #4
    str wzr, [x2], #4

    // GPRs 0..8
    ldp x3, x4, [x1], #16
    stp x3, x4, [x2], #16
    ldp x3, x4, [x1], #16
    stp x3, x4, [x2], #16
    ldp x3, x4, [x1], #16
    stp x3, x4, [x2], #16
    ldp x3, x4, [x1], #16
    stp x3, x4, [x2], #16
    ldr x3, [x1], #8
    str x3, [x2], #8

    // GPRs 9..28
    str x9, [x2], #8
    stp x10, x11, [x2], #16
    stp x12, x13, [x2], #16
    stp x14, x15, [x2], #16
    stp x16, x17, [x2], #16
    stp x18, x19, [x2], #16
    stp x20, x21, [x2], #16
    stp x22, x23, [x2], #16
    stp x24, x25, [x2], #16
    stp x26, x27, [x2], #16
    str x28, [x2], #8

    // fp
    str x29, [x2], #8

    // lr
    ldr x3, [x1], #8
    str x3, [x2], #8

    // sp
    ldr x3, [x1], #8
    str x3, [x2], #8

    // elr_el1 (pc)
    ldr x3, [x1], #8
    str x3, [x2], #8

    // fpsr
    mrs x3, FPSR
    str w3, [x2], #4

    // fpcr
    mrs x3, FPCR
    str w3, [x2], #4

    // fpu_gprs
    stp q0, q1, [x2], #32
    stp q2, q3, [x2], #32
    stp q4, q5, [x2], #32
    stp q6, q7, [x2], #32
    stp q8, q9, [x2], #32
    stp q10, q11, [x2], #32
    stp q12, q13, [x2], #32
    stp q14, q15, [x2], #32
    stp q16, q17, [x2], #32
    stp q18, q19, [x2], #32
    stp q20, q21, [x2], #32
    stp q22, q23, [x2], #32
    stp q24, q25, [x2], #32
    stp q26, q27, [x2], #32
    stp q28, q29, [x2], #32
    stp q30, q31, [x2], #32

    // 4 u32s: pstate, afsr0, afsr1, and esr.
    ldr w3, [x1], #4
    str w3, [x2], #4
    ldr w3, [x1], #4
    str w3, [x2], #4
    ldr w3, [x1], #4
    str w3, [x2], #4
    ldr w3, [x1], #4
    str w3, [x2], #4

    //far
    ldr x3, [x1], #8
    str x3, [x2], #8

    adrp x3, GDBStub_exception_stack
    add  x3, x3, #:lo12:GDBStub_exception_stack

    adrp x4, GDBStub_exception_stack_size
    ldr  x4, [x4, #:lo12:GDBStub_exception_stack_size]
    add x3, x3, x4
    mov sp, x3

    mov x0, x6 // ThreadExceptionDump_t
    stp x5, x6, [sp, #-16]!
    bl GDBStub_exception_handler
    ldp x5, x6, [sp], #16

    // Restore x1, x2
    mov x1, x5
    mov x2, x6

    // Store from ThreadExceptionDump_t to ThreadExceptionFrameA64

    // error_desc
    add x2, x2, #16

    // GPRs 0..8
    ldp x3, x4, [x2], #16
    stp x3, x4, [x1], #16
    ldp x3, x4, [x2], #16
    stp x3, x4, [x1], #16
    ldp x3, x4, [x2], #16
    stp x3, x4, [x1], #16
    ldp x3, x4, [x2], #16
    stp x3, x4, [x1], #16
    ldr x3, [x2], #8
    str x3, [x1], #8

    // GPRs 9..28
    ldr x9, [x2], #8
    ldp x10, x11, [x2], #16
    ldp x12, x13, [x2], #16
    ldp x14, x15, [x2], #16
    ldp x16, x17, [x2], #16
    ldp x18, x19, [x2], #16
    ldp x20, x21, [x2], #16
    ldp x22, x23, [x2], #16
    ldp x24, x25, [x2], #16
    ldp x26, x27, [x2], #16
    ldr x28, [x2], #8

    // fp
    ldr x29, [x2], #8

    // lr
    ldr x3, [x2], #8
    str x3, [x1], #8

    // sp
    ldr x3, [x2], #8
    str x3, [x1], #8

    // elr_el1 (pc)
    ldr x3, [x2], #8
    str x3, [x1], #8

    // fpsr
    ldr w3, [x2], #4
    msr FPSR, x3

    // fpcr
    ldr w3, [x2], #4
    msr FPCR, x3

    // fpu_gprs
    ldp q0, q1, [x2], #32
    ldp q2, q3, [x2], #32
    ldp q4, q5, [x2], #32
    ldp q6, q7, [x2], #32
    ldp q8, q9, [x2], #32
    ldp q10, q11, [x2], #32
    ldp q12, q13, [x2], #32
    ldp q14, q15, [x2], #32
    ldp q16, q17, [x2], #32
    ldp q18, q19, [x2], #32
    ldp q20, q21, [x2], #32
    ldp q22, q23, [x2], #32
    ldp q24, q25, [x2], #32
    ldp q26, q27, [x2], #32
    ldp q28, q29, [x2], #32
    ldp q30, q31, [x2], #32

    // 4 u32s: pstate, afsr0, afsr1, and esr.
    ldr w3, [x2], #4
    str w3, [x1], #4
    ldr w3, [x2], #4
    str w3, [x1], #4
    ldr w3, [x2], #4
    str w3, [x1], #4
    ldr w3, [x2], #4
    str w3, [x1], #4

    //far
    ldr x3, [x2], #8
    str x3, [x1], #8

    mov w0, wzr
    b __libnx_exception_entry_end__

__libnx_exception_entry_abort__:
    mov w0, #0xf801
__libnx_exception_entry_end__:
    bl svcReturnFromException
    b .
CODE_END

