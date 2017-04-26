################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/Navigation/Planner/Implements/GlobalPlanner/GlobalPlanner.cpp 

OBJS += \
./Source/Navigation/Planner/Implements/GlobalPlanner/GlobalPlanner.o 

CPP_DEPS += \
./Source/Navigation/Planner/Implements/GlobalPlanner/GlobalPlanner.d 


# Each subdirectory must supply rules for building sources it contributes
Source/Navigation/Planner/Implements/GlobalPlanner/%.o: ../Source/Navigation/Planner/Implements/GlobalPlanner/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-openwrt-linux-muslgnueabi-g++ -I$(SENAVICOMMON_PATH)/Source -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


