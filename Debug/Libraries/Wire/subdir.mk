################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Libraries/Wire/Wire.cpp 

OBJS += \
./Libraries/Wire/Wire.o 

CPP_DEPS += \
./Libraries/Wire/Wire.d 


# Each subdirectory must supply rules for building sources it contributes
Libraries/Wire/Wire.o: ../Libraries/Wire/Wire.cpp Libraries/Wire/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Libraries/Wire/Wire.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

