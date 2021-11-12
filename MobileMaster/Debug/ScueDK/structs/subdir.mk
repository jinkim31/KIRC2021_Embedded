################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ScueDK/structs/structs.c 

C_DEPS += \
./ScueDK/structs/structs.d 

OBJS += \
./ScueDK/structs/structs.o 


# Each subdirectory must supply rules for building sources it contributes
ScueDK/structs/%.o: ../ScueDK/structs/%.c ScueDK/structs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Jin Kim/Desktop/Projects/Robit/KIRC2021/Embedded/MobileMaster/Sequence/inc" -I"C:/Users/Jin Kim/Desktop/Projects/Robit/KIRC2021/Embedded/MobileMaster/ScueDK/structs" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

