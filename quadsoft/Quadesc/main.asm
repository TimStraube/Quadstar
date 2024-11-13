.section .text
.global _start

_start: 
    // Enable GPIOA clock
    LDR R0, = 0x40021000  // Base address of RCC
    LDR R1, [R0, #0x30]
    ORR R1, R1, R3   // Enable GPIOA and GPIOC clocks (bits 0 and 3)
    STR R1, [R0, #0x30]  // Write to RCC_AHB1ENR (offset 0x30)

    // Configure GPIOA pin 0 as output
    LDR R0, = 0x40020000  // Base address of GPIOA
    LDR R1, [R0, #0x00]  // Read GPIOA_MODER
    LDR R2, =0x00000001  // Load immediate value 0x00000001 into R2
    ORR R1, R1, R2       // Set pin 0 as output (bits 1:0 = 01)
    STR R1, [R0, #0x00]  // Write back to GPIOA_MODER

    // Configure GPIOD pin 12 as output
    LDR R0, =0x40020C00  // Base address of GPIOD
    LDR R1, [R0, #0x00]  // Read GPIOD_MODER
    LDR R2, =0x01000000  // Load immediate value 0x01000000 into R2
    ORR R1, R1, R2  // Set pin 12 as output (bits 25:24 = 01)
    STR R1, [R0, #0x00]  // Write back to GPIOD_MODER

    // Set GPIOA pin 0 high
    LDR R1, = 0x00000001  // Pin 0
    STR R1, [R0, #0x18]  // Write to GPIOA_BSRR (offset 0x18)

loop:
    // Toggle GPIOD pin 12
    LDR R1, [R0, #0x14]  // Read GPIOD_ODR
    LDR R2, =0x00001000  // Load immediate value 0x00001000 into R2
    EOR R1, R1, R2  // Toggle pin 12 (bit 12)
    STR R1, [R0, #0x14]  // Write back to GPIOD_ODR

    // Delay loop
    LDR R2, =0x0003FFFF  // Load delay count (split into two instructions)
    MOV R3, #0x0003      // Load upper half of the delay count
    LSL R3, R3, #16      // Shift left to form 0x00030000
    ORR R2, R2, R3       // Combine with lower half

delay:
    SUB R2, R2, #1      // Decrement delay count
    CMP R2, #0            // Repeat until delay count is zero
    BNE delay

    B loop

delay_loop:

.section .bss
.global _sbss, _ebss
_sbss:
    .space 0x200
_ebss:
