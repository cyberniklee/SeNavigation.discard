################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/Mapping/GMapping/GMappingApplication.cpp 

OBJS += \
./Source/Mapping/GMapping/GMappingApplication.o 

CPP_DEPS += \
./Source/Mapping/GMapping/GMappingApplication.d 


# Each subdirectory must supply rules for building sources it contributes
Source/Mapping/GMapping/%.o: ../Source/Mapping/GMapping/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	$(TARGET_CXX) -I"/home/seeing/SeeingNavi/SeNaviCommon/Source" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


