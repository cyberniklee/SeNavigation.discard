################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/Sensor/Lidar/Driver/SelidarDriver.cpp \
../Source/Sensor/Lidar/Driver/Serial.cpp 

OBJS += \
./Source/Sensor/Lidar/Driver/SelidarDriver.o \
./Source/Sensor/Lidar/Driver/Serial.o 

CPP_DEPS += \
./Source/Sensor/Lidar/Driver/SelidarDriver.d \
./Source/Sensor/Lidar/Driver/Serial.d 


# Each subdirectory must supply rules for building sources it contributes
Source/Sensor/Lidar/Driver/%.o: ../Source/Sensor/Lidar/Driver/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-openwrt-linux-muslgnueabi-g++ -I"/home/seeing/SeeingNavi/SeNaviCommon/Source" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


