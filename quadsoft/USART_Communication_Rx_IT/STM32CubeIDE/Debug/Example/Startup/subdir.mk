################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Example/Startup/startup_stm32f411retx.s 

OBJS += \
./Example/Startup/startup_stm32f411retx.o 

S_DEPS += \
./Example/Startup/startup_stm32f411retx.d 


# Each subdirectory must supply rules for building sources it contributes
Example/Startup/%.o: ../Example/Startup/%.s Example/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Example-2f-Startup

clean-Example-2f-Startup:
	-$(RM) ./Example/Startup/startup_stm32f411retx.d ./Example/Startup/startup_stm32f411retx.o

.PHONY: clean-Example-2f-Startup

