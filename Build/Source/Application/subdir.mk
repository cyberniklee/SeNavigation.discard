################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/Application/Application.cpp \
../Source/Application/ApplicationManager.cpp 

OBJS += \
./Source/Application/Application.o \
./Source/Application/ApplicationManager.o 

CPP_DEPS += \
./Source/Application/Application.d \
./Source/Application/ApplicationManager.d 


# Each subdirectory must supply rules for building sources it contributes
Source/Application/%.o: ../Source/Application/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -I"/home/seeing/SeeingNavi/SeNaviCommon/Source" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


