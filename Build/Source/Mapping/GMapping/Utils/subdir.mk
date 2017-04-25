################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/Mapping/GMapping/Utils/Movement.cpp \
../Source/Mapping/GMapping/Utils/PrintMemUsage.cpp \
../Source/Mapping/GMapping/Utils/Stat.cpp 

OBJS += \
./Source/Mapping/GMapping/Utils/Movement.o \
./Source/Mapping/GMapping/Utils/PrintMemUsage.o \
./Source/Mapping/GMapping/Utils/Stat.o 

CPP_DEPS += \
./Source/Mapping/GMapping/Utils/Movement.d \
./Source/Mapping/GMapping/Utils/PrintMemUsage.d \
./Source/Mapping/GMapping/Utils/Stat.d 


# Each subdirectory must supply rules for building sources it contributes
Source/Mapping/GMapping/Utils/%.o: ../Source/Mapping/GMapping/Utils/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	$(TARGET_CXX) -I"/home/seeing/SeeingNavi/SeNaviCommon/Source" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


