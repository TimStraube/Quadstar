# Assemble the program
arm-none-eabi-as -mcpu=cortex-m0 -mthumb main.asm -o main.o

# Link the object file
arm-none-eabi-ld -Tstm32u083.ld -o firmware.elf main.o

# Convert the ELF file to a binary file
arm-none-eabi-objcopy -O binary firmware.elf firmware.bin

# Flash the binary file to the MCU
STM32_Programmer_CLI -c port=SWD -d firmware.elf 0x08000000 -rst

# Green LED LD4