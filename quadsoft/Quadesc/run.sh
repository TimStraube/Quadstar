#!/bin/bash

# Define the toolchain
CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
SIZE=arm-none-eabi-size

# Define the target
TARGET=Quadesc

# Define the source files
SRCS="main.c syscall.c sysmem.c startup_stm32g431cbtx.s \
      Drivers/STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c \
      Drivers/STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c \
      Drivers/STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c \
      Drivers/STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c \
      Drivers/STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c \
      Drivers/STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c \
      Drivers/STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c \
      Drivers/STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c \
      Drivers/STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c \
      Drivers/STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c \
      Drivers/STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c \
      Drivers/STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c \
      Drivers/STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c \
      Drivers/STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/system_stm32g4xx.c"

# Define the include directories
INCLUDES="-IDrivers/STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Inc \
          -IDrivers/STM32CubeG4/Drivers/CMSIS/Device/ST/STM32G4xx/Include \
          -IDrivers/STM32CubeG4/Drivers/CMSIS/Include \
          -IInc"

# Define the compiler flags
CFLAGS="-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -Wall -Wextra -Wpedantic -Og -g3 -ggdb $INCLUDES -DSTM32G431xx"

# Define the linker flags without a linker script
LDFLAGS="-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -specs=nosys.specs -Wl,--gc-sections -Wl,-Map=build/$TARGET.map -Wl,--print-memory-usage"

# Create the build directory if it doesn't exist
mkdir -p build

# Compile the source files
$CC $CFLAGS -c $SRCS

# Move object files to the build directory
mv *.o build/

# Link the object files into an ELF file
$CC $CFLAGS build/*.o $LDFLAGS -o build/$TARGET.elf

# Convert the ELF file to HEX and BIN formats
$OBJCOPY -O ihex build/$TARGET.elf build/$TARGET.hex
$OBJCOPY -O binary build/$TARGET.elf build/$TARGET.bin

# Display the size of the binary
$SIZE build/$TARGET.elf

# Flash the binary to the STM32 microcontroller
st-flash write build/$TARGET.bin 0x8000000