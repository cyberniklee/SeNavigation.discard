################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/Navigation/CostMap/Layers/StaticLayer.cpp 

OBJS += \
./Source/Navigation/CostMap/Layers/StaticLayer.o 

CPP_DEPS += \
./Source/Navigation/CostMap/Layers/StaticLayer.d 


# Each subdirectory must supply rules for building sources it contributes
Source/Navigation/CostMap/Layers/%.o: ../Source/Navigation/CostMap/Layers/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -D__GLIBC__ -I$(SENAVICOMMON_PATH)/Source -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


