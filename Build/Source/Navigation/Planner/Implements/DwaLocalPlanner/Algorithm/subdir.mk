################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/Navigation/Planner/Implements/DwaLocalPlanner/Algorithm/DwaPlanner.cpp \
../Source/Navigation/Planner/Implements/DwaLocalPlanner/Algorithm/LatchedStopRotateController.cpp \
../Source/Navigation/Planner/Implements/DwaLocalPlanner/Algorithm/LocalPlannerUtil.cpp \
../Source/Navigation/Planner/Implements/DwaLocalPlanner/Algorithm/MapGridCostFunction.cpp \
../Source/Navigation/Planner/Implements/DwaLocalPlanner/Algorithm/ObstacleCostFunction.cpp \
../Source/Navigation/Planner/Implements/DwaLocalPlanner/Algorithm/OscillationCostFunction.cpp \
../Source/Navigation/Planner/Implements/DwaLocalPlanner/Algorithm/PreferForwardCostFunction.cpp \
../Source/Navigation/Planner/Implements/DwaLocalPlanner/Algorithm/SimpleScoredSamplingPlanner.cpp \
../Source/Navigation/Planner/Implements/DwaLocalPlanner/Algorithm/SimpleTrajectoryGenerator.cpp 

OBJS += \
./Source/Navigation/Planner/Implements/DwaLocalPlanner/Algorithm/DwaPlanner.o \
./Source/Navigation/Planner/Implements/DwaLocalPlanner/Algorithm/LatchedStopRotateController.o \
./Source/Navigation/Planner/Implements/DwaLocalPlanner/Algorithm/LocalPlannerUtil.o \
./Source/Navigation/Planner/Implements/DwaLocalPlanner/Algorithm/MapGridCostFunction.o \
./Source/Navigation/Planner/Implements/DwaLocalPlanner/Algorithm/ObstacleCostFunction.o \
./Source/Navigation/Planner/Implements/DwaLocalPlanner/Algorithm/OscillationCostFunction.o \
./Source/Navigation/Planner/Implements/DwaLocalPlanner/Algorithm/PreferForwardCostFunction.o \
./Source/Navigation/Planner/Implements/DwaLocalPlanner/Algorithm/SimpleScoredSamplingPlanner.o \
./Source/Navigation/Planner/Implements/DwaLocalPlanner/Algorithm/SimpleTrajectoryGenerator.o 

CPP_DEPS += \
./Source/Navigation/Planner/Implements/DwaLocalPlanner/Algorithm/DwaPlanner.d \
./Source/Navigation/Planner/Implements/DwaLocalPlanner/Algorithm/LatchedStopRotateController.d \
./Source/Navigation/Planner/Implements/DwaLocalPlanner/Algorithm/LocalPlannerUtil.d \
./Source/Navigation/Planner/Implements/DwaLocalPlanner/Algorithm/MapGridCostFunction.d \
./Source/Navigation/Planner/Implements/DwaLocalPlanner/Algorithm/ObstacleCostFunction.d \
./Source/Navigation/Planner/Implements/DwaLocalPlanner/Algorithm/OscillationCostFunction.d \
./Source/Navigation/Planner/Implements/DwaLocalPlanner/Algorithm/PreferForwardCostFunction.d \
./Source/Navigation/Planner/Implements/DwaLocalPlanner/Algorithm/SimpleScoredSamplingPlanner.d \
./Source/Navigation/Planner/Implements/DwaLocalPlanner/Algorithm/SimpleTrajectoryGenerator.d 


# Each subdirectory must supply rules for building sources it contributes
Source/Navigation/Planner/Implements/DwaLocalPlanner/Algorithm/%.o: ../Source/Navigation/Planner/Implements/DwaLocalPlanner/Algorithm/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-openwrt-linux-muslgnueabi-g++ -I$(SENAVICOMMON_PATH)/Source -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


