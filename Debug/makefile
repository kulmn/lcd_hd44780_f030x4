################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include startup/subdir.mk
-include protocols/subdir.mk
-include library/freertos/source/subdir.mk
-include indicators/subdir.mk
-include all/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
lcd_hc595_f030x4.bin \

SECONDARY_SIZE += \
lcd_hc595_f030x4.siz \


# All Target
all: lcd_hc595_f030x4.elf secondary-outputs

# Tool invocations
lcd_hc595_f030x4.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross ARM C Linker'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -T "D:\old_comp\kulish\stm32\workspace\lcd_hc595_f030x4\startup\stm32f030x4_flash.ld" -nostartfiles -nodefaultlibs -nostdlib -Xlinker --gc-sections -L"D:\old_comp\kulish\stm32\workspace\lcd_hc595_f030x4\library\libopencm3" -Wl,-Map,"lcd_hc595_f030x4.map" -o "lcd_hc595_f030x4.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

lcd_hc595_f030x4.bin: lcd_hc595_f030x4.elf
	@echo 'Invoking: Cross ARM GNU Create Flash Image'
	arm-none-eabi-objcopy -O binary "lcd_hc595_f030x4.elf"  "lcd_hc595_f030x4.bin"
	@echo 'Finished building: $@'
	@echo ' '

lcd_hc595_f030x4.siz: lcd_hc595_f030x4.elf
	@echo 'Invoking: Cross ARM GNU Print Size'
	arm-none-eabi-size --format=berkeley "lcd_hc595_f030x4.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(SECONDARY_FLASH)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_UPPER_DEPS)$(C_DEPS) lcd_hc595_f030x4.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_SIZE)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
