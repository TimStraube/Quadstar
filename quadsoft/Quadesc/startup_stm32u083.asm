; Stack initailization

Stack_Size      EQU     0x400;
                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size

__initial_sp

AREA STACK, NOINIT, READWRITE, ALIGN=3

; Vector table initalization

; Vector Table Mapped to Address 0 at Reset
                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors
                DCD     __initial_sp
                DCD     Reset_Handler
                DCD     NMI_Handler
                DCD     HardFault_Handler       

; Reset handler

; Exception and interrupt handlers

; Memory section initalization