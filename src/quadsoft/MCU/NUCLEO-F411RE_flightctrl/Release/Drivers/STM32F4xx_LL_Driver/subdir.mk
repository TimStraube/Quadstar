################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/tim/Programme/Quadstar/Quadsoft/TIM_PWMOutput/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_utils.c 

OBJS += \
./Drivers/STM32F4xx_LL_Driver/stm32f4xx_ll_utils.o 

C_DEPS += \
./Drivers/STM32F4xx_LL_Driver/stm32f4xx_ll_utils.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STM32F4xx_LL_Driver/stm32f4xx_ll_utils.o: /home/tim/Programme/Quadstar/Quadsoft/TIM_PWMOutput/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_utils.c Drivers/STM32F4xx_LL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../MEMS/App -I../MEMS/Target -I../Drivers/BSP/STM32F4xx_Nucleo -I../Drivers/BSP/Components/lsm6dsv16x -I../Drivers/BSP/Components/lis2duxs12 -I../Drivers/BSP/Components/lis2mdl -I../Drivers/BSP/Components/lsm6dso16is -I../Drivers/BSP/Components/sht40ad1b -I../Drivers/BSP/Components/lps22df -I../Drivers/BSP/Components/stts22h -I../Drivers/BSP/IKS4A1 -I../Drivers/BSP/Components/Common -I../Middlewares/ST/STM32_MotionFX_Library/Inc -I../X-CUBE-ISPU/Target -I../Drivers/BSP/Components/ism330is -I../Middlewares/ST/STM32_MotionID_Library/Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-STM32F4xx_LL_Driver

clean-Drivers-2f-STM32F4xx_LL_Driver:
	-$(RM) ./Drivers/STM32F4xx_LL_Driver/stm32f4xx_ll_utils.cyclo ./Drivers/STM32F4xx_LL_Driver/stm32f4xx_ll_utils.d ./Drivers/STM32F4xx_LL_Driver/stm32f4xx_ll_utils.o ./Drivers/STM32F4xx_LL_Driver/stm32f4xx_ll_utils.su

.PHONY: clean-Drivers-2f-STM32F4xx_LL_Driver

