################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/MotorControl/subdir.mk
-include Drivers/STM32G4xx_HAL_Driver/subdir.mk
-include Drivers/CMSIS/subdir.mk
-include Application/User/subdir.mk
-include Application/Startup/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := BG431B_ESC1_CubeIDE_motor_1
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
BG431B_ESC1_CubeIDE_motor_1.elf \

MAP_FILES += \
BG431B_ESC1_CubeIDE_motor_1.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
BG431B_ESC1_CubeIDE_motor_1.list \


# All Target
all: main-build

# Main-build Target
main-build: BG431B_ESC1_CubeIDE_motor_1.elf secondary-outputs

# Tool invocations
BG431B_ESC1_CubeIDE_motor_1.elf BG431B_ESC1_CubeIDE_motor_1.map: $(OBJS) $(USER_OBJS) /home/timstraube/Programme/Quadstar/src/quadsoft/ESC/BG431B_ESC1_CubeIDE_motor_1/STM32CubeIDE/STM32G431CBUX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "BG431B_ESC1_CubeIDE_motor_1.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/timstraube/Programme/Quadstar/src/quadsoft/ESC/BG431B_ESC1_CubeIDE_motor_1/STM32CubeIDE/STM32G431CBUX_FLASH.ld" --specs=nosys.specs -Wl,-Map="BG431B_ESC1_CubeIDE_motor_1.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

BG431B_ESC1_CubeIDE_motor_1.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "BG431B_ESC1_CubeIDE_motor_1.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) BG431B_ESC1_CubeIDE_motor_1.elf BG431B_ESC1_CubeIDE_motor_1.list BG431B_ESC1_CubeIDE_motor_1.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
