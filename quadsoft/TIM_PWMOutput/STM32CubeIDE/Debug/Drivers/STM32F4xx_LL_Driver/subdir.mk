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
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DSTM32F411xE -DHSE_VALUE=8000000U -c -I../../Inc -I../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-STM32F4xx_LL_Driver

clean-Drivers-2f-STM32F4xx_LL_Driver:
	-$(RM) ./Drivers/STM32F4xx_LL_Driver/stm32f4xx_ll_utils.cyclo ./Drivers/STM32F4xx_LL_Driver/stm32f4xx_ll_utils.d ./Drivers/STM32F4xx_LL_Driver/stm32f4xx_ll_utils.o ./Drivers/STM32F4xx_LL_Driver/stm32f4xx_ll_utils.su

.PHONY: clean-Drivers-2f-STM32F4xx_LL_Driver

