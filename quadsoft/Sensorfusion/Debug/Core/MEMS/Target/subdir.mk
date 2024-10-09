################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/MEMS/Target/com.c \
../Core/MEMS/Target/demo_serial.c \
../Core/MEMS/Target/iks4a1_mems_control.c \
../Core/MEMS/Target/iks4a1_mems_control_ex.c \
../Core/MEMS/Target/motion_fx_manager.c \
../Core/MEMS/Target/serial_protocol.c 

OBJS += \
./Core/MEMS/Target/com.o \
./Core/MEMS/Target/demo_serial.o \
./Core/MEMS/Target/iks4a1_mems_control.o \
./Core/MEMS/Target/iks4a1_mems_control_ex.o \
./Core/MEMS/Target/motion_fx_manager.o \
./Core/MEMS/Target/serial_protocol.o 

C_DEPS += \
./Core/MEMS/Target/com.d \
./Core/MEMS/Target/demo_serial.d \
./Core/MEMS/Target/iks4a1_mems_control.d \
./Core/MEMS/Target/iks4a1_mems_control_ex.d \
./Core/MEMS/Target/motion_fx_manager.d \
./Core/MEMS/Target/serial_protocol.d 


# Each subdirectory must supply rules for building sources it contributes
Core/MEMS/Target/%.o Core/MEMS/Target/%.su Core/MEMS/Target/%.cyclo: ../Core/MEMS/Target/%.c Core/MEMS/Target/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../MEMS/App -I../MEMS/Target -I../Drivers/BSP/STM32F4xx_Nucleo -I../Drivers/BSP/Components/lsm6dsv16x -I../Drivers/BSP/Components/lis2duxs12 -I../Drivers/BSP/Components/lis2mdl -I../Drivers/BSP/Components/lsm6dso16is -I../Drivers/BSP/Components/sht40ad1b -I../Drivers/BSP/Components/lps22df -I../Drivers/BSP/Components/stts22h -I../Drivers/BSP/IKS4A1 -I../Drivers/BSP/Components/Common -I../Middlewares/ST/STM32_MotionFX_Library/Inc -I../X-CUBE-ISPU/Target -I../Drivers/BSP/Components/ism330is -I../Middlewares/ST/STM32_MotionID_Library/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-MEMS-2f-Target

clean-Core-2f-MEMS-2f-Target:
	-$(RM) ./Core/MEMS/Target/com.cyclo ./Core/MEMS/Target/com.d ./Core/MEMS/Target/com.o ./Core/MEMS/Target/com.su ./Core/MEMS/Target/demo_serial.cyclo ./Core/MEMS/Target/demo_serial.d ./Core/MEMS/Target/demo_serial.o ./Core/MEMS/Target/demo_serial.su ./Core/MEMS/Target/iks4a1_mems_control.cyclo ./Core/MEMS/Target/iks4a1_mems_control.d ./Core/MEMS/Target/iks4a1_mems_control.o ./Core/MEMS/Target/iks4a1_mems_control.su ./Core/MEMS/Target/iks4a1_mems_control_ex.cyclo ./Core/MEMS/Target/iks4a1_mems_control_ex.d ./Core/MEMS/Target/iks4a1_mems_control_ex.o ./Core/MEMS/Target/iks4a1_mems_control_ex.su ./Core/MEMS/Target/motion_fx_manager.cyclo ./Core/MEMS/Target/motion_fx_manager.d ./Core/MEMS/Target/motion_fx_manager.o ./Core/MEMS/Target/motion_fx_manager.su ./Core/MEMS/Target/serial_protocol.cyclo ./Core/MEMS/Target/serial_protocol.d ./Core/MEMS/Target/serial_protocol.o ./Core/MEMS/Target/serial_protocol.su

.PHONY: clean-Core-2f-MEMS-2f-Target

