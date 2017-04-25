################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/Mapping/GMapping/Sensor/SensorBase/Sensor.cpp \
../Source/Mapping/GMapping/Sensor/SensorBase/SensorReading.cpp 

OBJS += \
./Source/Mapping/GMapping/Sensor/SensorBase/Sensor.o \
./Source/Mapping/GMapping/Sensor/SensorBase/SensorReading.o 

CPP_DEPS += \
./Source/Mapping/GMapping/Sensor/SensorBase/Sensor.d \
./Source/Mapping/GMapping/Sensor/SensorBase/SensorReading.d 


# Each subdirectory must supply rules for building sources it contributes
Source/Mapping/GMapping/Sensor/SensorBase/%.o: ../Source/Mapping/GMapping/Sensor/SensorBase/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -I"/home/seeing/SeeingNavi/SeNaviCommon/Source" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


