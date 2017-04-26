################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/Mapping/GMapping/GridFastSlam/GFSReader.cpp \
../Source/Mapping/GMapping/GridFastSlam/GridSlamProcessor.cpp \
../Source/Mapping/GMapping/GridFastSlam/GridSlamProcessorTree.cpp \
../Source/Mapping/GMapping/GridFastSlam/MotionModel.cpp 

OBJS += \
./Source/Mapping/GMapping/GridFastSlam/GFSReader.o \
./Source/Mapping/GMapping/GridFastSlam/GridSlamProcessor.o \
./Source/Mapping/GMapping/GridFastSlam/GridSlamProcessorTree.o \
./Source/Mapping/GMapping/GridFastSlam/MotionModel.o 

CPP_DEPS += \
./Source/Mapping/GMapping/GridFastSlam/GFSReader.d \
./Source/Mapping/GMapping/GridFastSlam/GridSlamProcessor.d \
./Source/Mapping/GMapping/GridFastSlam/GridSlamProcessorTree.d \
./Source/Mapping/GMapping/GridFastSlam/MotionModel.d 


# Each subdirectory must supply rules for building sources it contributes
Source/Mapping/GMapping/GridFastSlam/%.o: ../Source/Mapping/GMapping/GridFastSlam/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-openwrt-linux-muslgnueabi-g++ -I$(SENAVICOMMON_PATH)/Source -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


