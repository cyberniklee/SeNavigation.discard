################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/Sensor/Lidar/SelidarApplication.cpp 

OBJS += \
./Source/Sensor/Lidar/SelidarApplication.o 

CPP_DEPS += \
./Source/Sensor/Lidar/SelidarApplication.d 


# Each subdirectory must supply rules for building sources it contributes
Source/Sensor/Lidar/%.o: ../Source/Sensor/Lidar/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -I"/home/seeing/SeeingNavi/SeNaviCommon/Source" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


