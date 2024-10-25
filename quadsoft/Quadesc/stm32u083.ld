/* Linker script for STM32U083 */

/*
- SRAM1: 32 Kbytes with hardware parity check, mapped at address 0x2000 0000
- SRAM2: 8 Kbytes with hardware parity check, located at address 0x1000 0000
SRAM2 is also mapped at address 0x2000 8000, offering a contiguous address space with SRAM1
(8 Kbytes aliased by bit band).
*/

/* _estack = ORIGIN(RAM) + LENGTH(RAM); */
_estack = 0x2000A000;

_Min_Heap_Size = 0x200; 
_Min_Stack_Size = 0x400; 

MEMORY
{
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 256K
  RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 40K
}
