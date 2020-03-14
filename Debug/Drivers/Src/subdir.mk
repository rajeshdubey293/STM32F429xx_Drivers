################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/GPIOx_Drivers.c 

OBJS += \
./Drivers/Src/GPIOx_Drivers.o 

C_DEPS += \
./Drivers/Src/GPIOx_Drivers.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/GPIOx_Drivers.o: ../Drivers/Src/GPIOx_Drivers.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F429I_DISC1 -DSTM32 -DSTM32F429ZITx -DSTM32F4 -DDEBUG -c -I"/home/vicky/STM32CubeIDE/STM32F429xx_Drivers/Drivers" -I"/home/vicky/STM32CubeIDE/STM32F429xx_Drivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/GPIOx_Drivers.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

