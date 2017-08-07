################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/Navigation/CostMap/Utils/ArrayParser.cpp \
../Source/Navigation/CostMap/Utils/Footprint.cpp \
../Source/Navigation/CostMap/Utils/Math.cpp 

OBJS += \
./Source/Navigation/CostMap/Utils/ArrayParser.o \
./Source/Navigation/CostMap/Utils/Footprint.o \
./Source/Navigation/CostMap/Utils/Math.o 

CPP_DEPS += \
./Source/Navigation/CostMap/Utils/ArrayParser.d \
./Source/Navigation/CostMap/Utils/Footprint.d \
./Source/Navigation/CostMap/Utils/Math.d 


# Each subdirectory must supply rules for building sources it contributes
Source/Navigation/CostMap/Utils/%.o: ../Source/Navigation/CostMap/Utils/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -I$(SENAVICOMMON_PATH)/Source -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


