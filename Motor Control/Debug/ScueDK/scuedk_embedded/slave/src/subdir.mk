################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ScueDK/scuedk_embedded/slave/src/serial_line.c 

OBJS += \
./ScueDK/scuedk_embedded/slave/src/serial_line.o 

C_DEPS += \
./ScueDK/scuedk_embedded/slave/src/serial_line.d 


# Each subdirectory must supply rules for building sources it contributes
ScueDK/scuedk_embedded/slave/src/%.o: ../ScueDK/scuedk_embedded/slave/src/%.c ScueDK/scuedk_embedded/slave/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Jin Kim/Desktop/Projects/Robit/KIRC2021/Embedded/Motor Control/ScueDK/scuedk_embedded/slave/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

