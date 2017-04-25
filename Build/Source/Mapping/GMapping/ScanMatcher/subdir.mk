################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/Mapping/GMapping/ScanMatcher/Eig3.cpp \
../Source/Mapping/GMapping/ScanMatcher/ScanMatcher.cpp \
../Source/Mapping/GMapping/ScanMatcher/ScanMatcherMap.cpp \
../Source/Mapping/GMapping/ScanMatcher/ScanMatcherProcessor.cpp 

OBJS += \
./Source/Mapping/GMapping/ScanMatcher/Eig3.o \
./Source/Mapping/GMapping/ScanMatcher/ScanMatcher.o \
./Source/Mapping/GMapping/ScanMatcher/ScanMatcherMap.o \
./Source/Mapping/GMapping/ScanMatcher/ScanMatcherProcessor.o 

CPP_DEPS += \
./Source/Mapping/GMapping/ScanMatcher/Eig3.d \
./Source/Mapping/GMapping/ScanMatcher/ScanMatcher.d \
./Source/Mapping/GMapping/ScanMatcher/ScanMatcherMap.d \
./Source/Mapping/GMapping/ScanMatcher/ScanMatcherProcessor.d 


# Each subdirectory must supply rules for building sources it contributes
Source/Mapping/GMapping/ScanMatcher/%.o: ../Source/Mapping/GMapping/ScanMatcher/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -I"/home/seeing/SeeingNavi/SeNaviCommon/Source" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


