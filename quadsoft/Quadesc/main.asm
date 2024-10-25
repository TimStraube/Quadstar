.section .text
.global _start

_start:

loop:

delay_loop:

.section .bss
.global _sbss, _ebss
_sbss:
    .space 0x200
_ebss:
