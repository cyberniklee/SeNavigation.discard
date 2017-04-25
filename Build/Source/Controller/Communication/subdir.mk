################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/Controller/Communication/Protocol.cpp \
../Source/Controller/Communication/SpiComm.cpp 

OBJS += \
./Source/Controller/Communication/Protocol.o \
./Source/Controller/Communication/SpiComm.o 

CPP_DEPS += \
./Source/Controller/Communication/Protocol.d \
./Source/Controller/Communication/SpiComm.d 


# Each subdirectory must supply rules for building sources it contributes
Source/Controller/Communication/%.o: ../Source/Controller/Communication/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	$(TARGET_CXX) -I"/home/seeing/SeeingNavi/SeNaviCommon/Source" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


