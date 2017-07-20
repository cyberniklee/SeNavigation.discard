################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/Controller/Simulator/SimulateController.cpp 

OBJS += \
./Source/Controller/Simulator/SimulateController.o 

CPP_DEPS += \
./Source/Controller/Simulator/SimulateController.d 


# Each subdirectory must supply rules for building sources it contributes
Source/Controller/Simulator/%.o: ../Source/Controller/Simulator/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-openwrt-linux-muslgnueabi-g++ -I$(SENAVICOMMON_PATH)/Source -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


