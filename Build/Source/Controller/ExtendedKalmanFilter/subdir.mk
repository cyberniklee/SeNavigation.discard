################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/Controller/ExtendedKalmanFilter/Estimation.cpp \
../Source/Controller/ExtendedKalmanFilter/NonLinearAnalyticConditionalGaussianOdo.cpp 

OBJS += \
./Source/Controller/ExtendedKalmanFilter/Estimation.o \
./Source/Controller/ExtendedKalmanFilter/NonLinearAnalyticConditionalGaussianOdo.o 

CPP_DEPS += \
./Source/Controller/ExtendedKalmanFilter/Estimation.d \
./Source/Controller/ExtendedKalmanFilter/NonLinearAnalyticConditionalGaussianOdo.d 


# Each subdirectory must supply rules for building sources it contributes
Source/Controller/ExtendedKalmanFilter/%.o: ../Source/Controller/ExtendedKalmanFilter/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-openwrt-linux-muslgnueabi-g++ -I$(SENAVICOMMON_PATH)/Source -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


