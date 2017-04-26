################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Source/Sensor/Lidar/Driver/subdir.mk
-include Source/Sensor/Lidar/subdir.mk
-include Source/Navigation/Planner/Implements/GlobalPlanner/Algorithm/subdir.mk
-include Source/Navigation/Planner/Implements/GlobalPlanner/subdir.mk
-include Source/Navigation/CostMap/Utils/subdir.mk
-include Source/Navigation/CostMap/Layers/subdir.mk
-include Source/Navigation/CostMap/CostMap2D/subdir.mk
-include Source/Navigation/CostMap/subdir.mk
-include Source/Navigation/subdir.mk
-include Source/Mapping/GMapping/Utils/subdir.mk
-include Source/Mapping/GMapping/Sensor/SensorRange/subdir.mk
-include Source/Mapping/GMapping/Sensor/SensorOdometry/subdir.mk
-include Source/Mapping/GMapping/Sensor/SensorBase/subdir.mk
-include Source/Mapping/GMapping/ScanMatcher/subdir.mk
-include Source/Mapping/GMapping/Log/subdir.mk
-include Source/Mapping/GMapping/GridFastSlam/subdir.mk
-include Source/Mapping/GMapping/subdir.mk
-include Source/Controller/Communication/subdir.mk
-include Source/Controller/subdir.mk
-include Source/Communication/subdir.mk
-include Source/Application/subdir.mk
-include Source/subdir.mk
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
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: SeNavigation

# Tool invocations
SeNavigation: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross G++ Linker'
	arm-openwrt-linux-muslgnueabi-g++ -L$(SENAVICOMMON_PATH)/Build -L$(BOOST_LIB_DIR) -o "SeNavigation" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(CC_DEPS)$(C++_DEPS)$(EXECUTABLES)$(OBJS)$(C_UPPER_DEPS)$(CXX_DEPS)$(C_DEPS)$(CPP_DEPS) SeNavigation
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
