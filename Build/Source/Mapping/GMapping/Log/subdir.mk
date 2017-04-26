################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/Mapping/GMapping/Log/CarmenConfiguration.cpp \
../Source/Mapping/GMapping/Log/Configuration.cpp \
../Source/Mapping/GMapping/Log/SensorLog.cpp \
../Source/Mapping/GMapping/Log/SensorStream.cpp 

OBJS += \
./Source/Mapping/GMapping/Log/CarmenConfiguration.o \
./Source/Mapping/GMapping/Log/Configuration.o \
./Source/Mapping/GMapping/Log/SensorLog.o \
./Source/Mapping/GMapping/Log/SensorStream.o 

CPP_DEPS += \
./Source/Mapping/GMapping/Log/CarmenConfiguration.d \
./Source/Mapping/GMapping/Log/Configuration.d \
./Source/Mapping/GMapping/Log/SensorLog.d \
./Source/Mapping/GMapping/Log/SensorStream.d 


# Each subdirectory must supply rules for building sources it contributes
Source/Mapping/GMapping/Log/%.o: ../Source/Mapping/GMapping/Log/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-openwrt-linux-muslgnueabi-g++ -I$(SENAVICOMMON_PATH)/Source -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


