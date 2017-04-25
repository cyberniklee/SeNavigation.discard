################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/Mapping/GMapping/Sensor/SensorRange/RangeReading.cpp \
../Source/Mapping/GMapping/Sensor/SensorRange/RangeSensor.cpp 

OBJS += \
./Source/Mapping/GMapping/Sensor/SensorRange/RangeReading.o \
./Source/Mapping/GMapping/Sensor/SensorRange/RangeSensor.o 

CPP_DEPS += \
./Source/Mapping/GMapping/Sensor/SensorRange/RangeReading.d \
./Source/Mapping/GMapping/Sensor/SensorRange/RangeSensor.d 


# Each subdirectory must supply rules for building sources it contributes
Source/Mapping/GMapping/Sensor/SensorRange/%.o: ../Source/Mapping/GMapping/Sensor/SensorRange/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -I"/home/seeing/SeeingNavi/SeNaviCommon/Source" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


