################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../X-CUBE-ISPU/Target/custom_ispu_motion_sensors.c \
../X-CUBE-ISPU/Target/custom_ispu_motion_sensors_ex.c 

OBJS += \
./X-CUBE-ISPU/Target/custom_ispu_motion_sensors.o \
./X-CUBE-ISPU/Target/custom_ispu_motion_sensors_ex.o 

C_DEPS += \
./X-CUBE-ISPU/Target/custom_ispu_motion_sensors.d \
./X-CUBE-ISPU/Target/custom_ispu_motion_sensors_ex.d 


# Each subdirectory must supply rules for building sources it contributes
X-CUBE-ISPU/Target/%.o X-CUBE-ISPU/Target/%.su X-CUBE-ISPU/Target/%.cyclo: ../X-CUBE-ISPU/Target/%.c X-CUBE-ISPU/Target/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../MEMS/App -I../MEMS/Target -I../Drivers/BSP/STM32F4xx_Nucleo -I../Drivers/BSP/Components/lsm6dsv16x -I../Drivers/BSP/Components/lis2duxs12 -I../Drivers/BSP/Components/lis2mdl -I../Drivers/BSP/Components/lsm6dso16is -I../Drivers/BSP/Components/sht40ad1b -I../Drivers/BSP/Components/lps22df -I../Drivers/BSP/Components/stts22h -I../Drivers/BSP/IKS4A1 -I../Drivers/BSP/Components/Common -I../Middlewares/ST/STM32_MotionFX_Library/Inc -I../X-CUBE-ISPU/Target -I../Drivers/BSP/Components/ism330is -I../Middlewares/ST/STM32_MotionID_Library/Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-X-2d-CUBE-2d-ISPU-2f-Target

clean-X-2d-CUBE-2d-ISPU-2f-Target:
	-$(RM) ./X-CUBE-ISPU/Target/custom_ispu_motion_sensors.cyclo ./X-CUBE-ISPU/Target/custom_ispu_motion_sensors.d ./X-CUBE-ISPU/Target/custom_ispu_motion_sensors.o ./X-CUBE-ISPU/Target/custom_ispu_motion_sensors.su ./X-CUBE-ISPU/Target/custom_ispu_motion_sensors_ex.cyclo ./X-CUBE-ISPU/Target/custom_ispu_motion_sensors_ex.d ./X-CUBE-ISPU/Target/custom_ispu_motion_sensors_ex.o ./X-CUBE-ISPU/Target/custom_ispu_motion_sensors_ex.su

.PHONY: clean-X-2d-CUBE-2d-ISPU-2f-Target

