################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/lsm6dsv16x/lsm6dsv16x.c \
../Drivers/BSP/Components/lsm6dsv16x/lsm6dsv16x_reg.c 

OBJS += \
./Drivers/BSP/Components/lsm6dsv16x/lsm6dsv16x.o \
./Drivers/BSP/Components/lsm6dsv16x/lsm6dsv16x_reg.o 

C_DEPS += \
./Drivers/BSP/Components/lsm6dsv16x/lsm6dsv16x.d \
./Drivers/BSP/Components/lsm6dsv16x/lsm6dsv16x_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/lsm6dsv16x/%.o Drivers/BSP/Components/lsm6dsv16x/%.su Drivers/BSP/Components/lsm6dsv16x/%.cyclo: ../Drivers/BSP/Components/lsm6dsv16x/%.c Drivers/BSP/Components/lsm6dsv16x/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../X-CUBE-ISPU/Target -I../MEMS/App -I../MEMS/Target -I../Core/Inc -I../Drivers/BSP/STM32F4xx_Nucleo -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/STM32_MotionID_Library/Inc -I../Drivers/BSP/Components/lsm6dsv16x -I../Drivers/BSP/Components/lis2duxs12 -I../Drivers/BSP/Components/lis2mdl -I../Drivers/BSP/Components/lsm6dso16is -I../Drivers/BSP/Components/sht40ad1b -I../Drivers/BSP/Components/lps22df -I../Drivers/BSP/Components/stts22h -I../Drivers/BSP/IKS4A1 -I../Drivers/BSP/Components/Common -I../Middlewares/ST/STM32_MotionFX_Library/Inc -I../Drivers/BSP/Components/ism330is -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-lsm6dsv16x

clean-Drivers-2f-BSP-2f-Components-2f-lsm6dsv16x:
	-$(RM) ./Drivers/BSP/Components/lsm6dsv16x/lsm6dsv16x.cyclo ./Drivers/BSP/Components/lsm6dsv16x/lsm6dsv16x.d ./Drivers/BSP/Components/lsm6dsv16x/lsm6dsv16x.o ./Drivers/BSP/Components/lsm6dsv16x/lsm6dsv16x.su ./Drivers/BSP/Components/lsm6dsv16x/lsm6dsv16x_reg.cyclo ./Drivers/BSP/Components/lsm6dsv16x/lsm6dsv16x_reg.d ./Drivers/BSP/Components/lsm6dsv16x/lsm6dsv16x_reg.o ./Drivers/BSP/Components/lsm6dsv16x/lsm6dsv16x_reg.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-lsm6dsv16x

