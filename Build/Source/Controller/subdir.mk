################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/Controller/ControllerApplication.cpp 

OBJS += \
./Source/Controller/ControllerApplication.o 

CPP_DEPS += \
./Source/Controller/ControllerApplication.d 


# Each subdirectory must supply rules for building sources it contributes
Source/Controller/%.o: ../Source/Controller/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-openwrt-linux-muslgnueabi-g++ -I"/home/seeing/SeeingNavi/SeNaviCommon/Source" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


