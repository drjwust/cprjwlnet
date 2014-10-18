################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := D:/ProgramFiles/CCS/ccsv5/tools/compiler/c2000_6.1.3

ORDERED_OBJS += \
$(GEN_CMDS__FLAG) \
"./sources/F28335A_main.obj" \
"./sources/DSP2833x_PieVect.obj" \
"./sources/DSP2833x_PieCtrl.obj" \
"./sources/DSP2833x_GlobalVariableDefs.obj" \
"./sources/DSP2833x_DefaultIsr.obj" \
"./sources/DSP2833x_CodeStartBranch.obj" \
"./sources/DSP2833x_ADC_cal.obj" \
-l"libc.a" \
"../28335_RAM_lnk.cmd" \
"../DSP2833x_Headers_nonBIOS.cmd" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include sources/subdir_vars.mk
-include subdir_rules.mk
-include sources/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: FFT141018A.out

# Tool invocations
FFT141018A.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: C2000 Linker'
	"D:/ProgramFiles/CCS/ccsv5/tools/compiler/c2000_6.1.3/bin/cl2000" -v28 -ml -mt --float_support=fpu32 -g --diag_warning=225 --display_error_number --diag_wrap=off -z --stack_size=0x300 -m"FFT141018A.map" --warn_sections -i"D:/ProgramFiles/CCS/ccsv5/tools/compiler/c2000_6.1.3/lib" -i"D:/ProgramFiles/CCS/ccsv5/tools/compiler/c2000_6.1.3/include" --reread_libs --display_error_number --diag_wrap=off --xml_link_info="FFT141018A_linkInfo.xml" --rom_model -o "FFT141018A.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(C2000_EXECUTABLE_OUTPUTS__QUOTED) "FFT141018A.out"
	-$(RM) "sources\DSP2833x_DefaultIsr.pp" "sources\DSP2833x_GlobalVariableDefs.pp" "sources\DSP2833x_PieCtrl.pp" "sources\DSP2833x_PieVect.pp" "sources\F28335A_main.pp" 
	-$(RM) "sources\DSP2833x_ADC_cal.obj" "sources\DSP2833x_CodeStartBranch.obj" "sources\DSP2833x_DefaultIsr.obj" "sources\DSP2833x_GlobalVariableDefs.obj" "sources\DSP2833x_PieCtrl.obj" "sources\DSP2833x_PieVect.obj" "sources\F28335A_main.obj" 
	-$(RM) "sources\DSP2833x_ADC_cal.pp" "sources\DSP2833x_CodeStartBranch.pp" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
