################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/sht40ad1b/sht40ad1b.c \
../Drivers/BSP/Components/sht40ad1b/sht40ad1b_reg.c 

OBJS += \
./Drivers/BSP/Components/sht40ad1b/sht40ad1b.o \
./Drivers/BSP/Components/sht40ad1b/sht40ad1b_reg.o 

C_DEPS += \
./Drivers/BSP/Components/sht40ad1b/sht40ad1b.d \
./Drivers/BSP/Components/sht40ad1b/sht40ad1b_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/sht40ad1b/%.o Drivers/BSP/Components/sht40ad1b/%.su Drivers/BSP/Components/sht40ad1b/%.cyclo: ../Drivers/BSP/Components/sht40ad1b/%.c Drivers/BSP/Components/sht40ad1b/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../MEMS/App -I../MEMS/Target -I../Drivers/BSP/STM32F4xx_Nucleo -I../Drivers/BSP/Components/lsm6dsv16x -I../Drivers/BSP/Components/lis2duxs12 -I../Drivers/BSP/Components/lis2mdl -I../Drivers/BSP/Components/lsm6dso16is -I../Drivers/BSP/Components/sht40ad1b -I../Drivers/BSP/Components/lps22df -I../Drivers/BSP/Components/stts22h -I../Drivers/BSP/IKS4A1 -I../Drivers/BSP/Components/Common -I../Middlewares/ST/STM32_MotionFX_Library/Inc -I../X-CUBE-ISPU/Target -I../Drivers/BSP/Components/ism330is -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-sht40ad1b

clean-Drivers-2f-BSP-2f-Components-2f-sht40ad1b:
	-$(RM) ./Drivers/BSP/Components/sht40ad1b/sht40ad1b.cyclo ./Drivers/BSP/Components/sht40ad1b/sht40ad1b.d ./Drivers/BSP/Components/sht40ad1b/sht40ad1b.o ./Drivers/BSP/Components/sht40ad1b/sht40ad1b.su ./Drivers/BSP/Components/sht40ad1b/sht40ad1b_reg.cyclo ./Drivers/BSP/Components/sht40ad1b/sht40ad1b_reg.d ./Drivers/BSP/Components/sht40ad1b/sht40ad1b_reg.o ./Drivers/BSP/Components/sht40ad1b/sht40ad1b_reg.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-sht40ad1b

