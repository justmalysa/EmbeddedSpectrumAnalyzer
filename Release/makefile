################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include TouchGFX/touchgfx/os/subdir.mk
-include TouchGFX/touchgfx/framework/source/touchgfx/widgets/canvas/subdir.mk
-include TouchGFX/touchgfx/framework/source/touchgfx/widgets/subdir.mk
-include TouchGFX/touchgfx/framework/source/touchgfx/containers/scrollers/subdir.mk
-include TouchGFX/touchgfx/framework/source/touchgfx/containers/progress_indicators/subdir.mk
-include TouchGFX/touchgfx/framework/source/touchgfx/containers/clock/subdir.mk
-include TouchGFX/touchgfx/framework/source/touchgfx/containers/subdir.mk
-include TouchGFX/touchgfx/framework/source/platform/hal/simulator/sdl2/subdir.mk
-include TouchGFX/touchgfx/framework/source/platform/driver/touch/subdir.mk
-include TouchGFX/gui/src/screen1_screen/subdir.mk
-include TouchGFX/gui/src/model/subdir.mk
-include TouchGFX/gui/src/common/subdir.mk
-include TouchGFX/generated/texts/src/subdir.mk
-include TouchGFX/generated/images/src/__designer/subdir.mk
-include TouchGFX/generated/images/src/subdir.mk
-include TouchGFX/generated/gui_generated/src/screen1_screen/subdir.mk
-include TouchGFX/generated/gui_generated/src/common/subdir.mk
-include TouchGFX/generated/fonts/src/subdir.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include Middlewares/Third_Party/FatFs/src/option/subdir.mk
-include Middlewares/Third_Party/FatFs/src/subdir.mk
-include Middlewares/ST/TouchGFX/touchgfx/os/subdir.mk
-include Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/widgets/canvas/subdir.mk
-include Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/widgets/subdir.mk
-include Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/scrollers/subdir.mk
-include Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/progress_indicators/subdir.mk
-include Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/clock/subdir.mk
-include Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/subdir.mk
-include Middlewares/ST/TouchGFX/touchgfx/framework/source/platform/hal/simulator/sdl2/subdir.mk
-include Middlewares/ST/TouchGFX/touchgfx/framework/source/platform/driver/touch/subdir.mk
-include Middlewares/ST/STM32_USB_Host_Library/Core/Src/subdir.mk
-include Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Src/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Drivers/BSP/Components/ts3510/subdir.mk
-include Drivers/BSP/Components/stmpe811/subdir.mk
-include Drivers/BSP/Components/otm8009a/subdir.mk
-include Drivers/BSP/Components/ft6x06/subdir.mk
-include Drivers/BSP/Components/exc7200/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

BUILD_ARTIFACT_NAME := SpectrumAnalyzer
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX := 
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME).$(BUILD_ARTIFACT_EXTENSION)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
SpectrumAnalyzer.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
SpectrumAnalyzer.list \

OBJCOPY_BIN += \
SpectrumAnalyzer.bin \


# All Target
all: main-build

# Main-build Target
main-build: SpectrumAnalyzer.elf secondary-outputs

# Tool invocations
SpectrumAnalyzer.elf: $(OBJS) $(USER_OBJS) D:\Projekty\embedded_spectrum_analyzer_git\EmbeddedSpectrumAnalyzer\STM32F469NIHX_FLASH.ld
	arm-none-eabi-g++ -o "SpectrumAnalyzer.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\Projekty\embedded_spectrum_analyzer_git\EmbeddedSpectrumAnalyzer\STM32F469NIHX_FLASH.ld" --specs=nosys.specs -Wl,-Map="SpectrumAnalyzer.map" -Wl,--gc-sections -static -LMiddlewares/ST/TouchGFX/touchgfx/lib/core/cortex_m4f/gcc --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

SpectrumAnalyzer.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "SpectrumAnalyzer.list"
	@echo 'Finished building: $@'
	@echo ' '

SpectrumAnalyzer.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "SpectrumAnalyzer.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents fail-specified-linker-script-missing warn-no-linker-script-specified
.SECONDARY:

-include ../makefile.targets
