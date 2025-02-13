################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/IKS4A1/iks4a1_env_sensors.c \
../Drivers/BSP/IKS4A1/iks4a1_env_sensors_ex.c \
../Drivers/BSP/IKS4A1/iks4a1_motion_sensors.c \
../Drivers/BSP/IKS4A1/iks4a1_motion_sensors_ex.c 

OBJS += \
./Drivers/BSP/IKS4A1/iks4a1_env_sensors.o \
./Drivers/BSP/IKS4A1/iks4a1_env_sensors_ex.o \
./Drivers/BSP/IKS4A1/iks4a1_motion_sensors.o \
./Drivers/BSP/IKS4A1/iks4a1_motion_sensors_ex.o 

C_DEPS += \
./Drivers/BSP/IKS4A1/iks4a1_env_sensors.d \
./Drivers/BSP/IKS4A1/iks4a1_env_sensors_ex.d \
./Drivers/BSP/IKS4A1/iks4a1_motion_sensors.d \
./Drivers/BSP/IKS4A1/iks4a1_motion_sensors_ex.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/IKS4A1/%.o Drivers/BSP/IKS4A1/%.su Drivers/BSP/IKS4A1/%.cyclo: ../Drivers/BSP/IKS4A1/%.c Drivers/BSP/IKS4A1/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../MEMS/App -I../MEMS/Target -I../Drivers/BSP/STM32F4xx_Nucleo -I../Drivers/BSP/Components/lsm6dsv16x -I../Drivers/BSP/Components/lis2duxs12 -I../Drivers/BSP/Components/lis2mdl -I../Drivers/BSP/Components/lsm6dso16is -I../Drivers/BSP/Components/sht40ad1b -I../Drivers/BSP/Components/lps22df -I../Drivers/BSP/Components/stts22h -I../Drivers/BSP/IKS4A1 -I../Drivers/BSP/Components/Common -I../Middlewares/ST/STM32_MotionID_Library/Inc -I../Middlewares/ST/STM32_MotionFX_Library/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-IKS4A1

clean-Drivers-2f-BSP-2f-IKS4A1:
	-$(RM) ./Drivers/BSP/IKS4A1/iks4a1_env_sensors.cyclo ./Drivers/BSP/IKS4A1/iks4a1_env_sensors.d ./Drivers/BSP/IKS4A1/iks4a1_env_sensors.o ./Drivers/BSP/IKS4A1/iks4a1_env_sensors.su ./Drivers/BSP/IKS4A1/iks4a1_env_sensors_ex.cyclo ./Drivers/BSP/IKS4A1/iks4a1_env_sensors_ex.d ./Drivers/BSP/IKS4A1/iks4a1_env_sensors_ex.o ./Drivers/BSP/IKS4A1/iks4a1_env_sensors_ex.su ./Drivers/BSP/IKS4A1/iks4a1_motion_sensors.cyclo ./Drivers/BSP/IKS4A1/iks4a1_motion_sensors.d ./Drivers/BSP/IKS4A1/iks4a1_motion_sensors.o ./Drivers/BSP/IKS4A1/iks4a1_motion_sensors.su ./Drivers/BSP/IKS4A1/iks4a1_motion_sensors_ex.cyclo ./Drivers/BSP/IKS4A1/iks4a1_motion_sensors_ex.d ./Drivers/BSP/IKS4A1/iks4a1_motion_sensors_ex.o ./Drivers/BSP/IKS4A1/iks4a1_motion_sensors_ex.su

.PHONY: clean-Drivers-2f-BSP-2f-IKS4A1

