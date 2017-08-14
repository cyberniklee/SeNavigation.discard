################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/Mapping/Hector/Utils/PoseInfoContainer.cpp 

OBJS += \
./Source/Mapping/Hector/Utils/PoseInfoContainer.o 

CPP_DEPS += \
./Source/Mapping/Hector/Utils/PoseInfoContainer.d 


# Each subdirectory must supply rules for building sources it contributes
Source/Mapping/Hector/Utils/%.o: ../Source/Mapping/Hector/Utils/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-openwrt-linux-muslgnueabi-g++ -I$(SENAVICOMMON_PATH)/Source -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


