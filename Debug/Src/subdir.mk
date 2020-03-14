################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/002LEDWithBtn.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/002LEDWithBtn.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/002LEDWithBtn.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/002LEDWithBtn.o: ../Src/002LEDWithBtn.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F429I_DISC1 -DSTM32 -DSTM32F429ZITx -DSTM32F4 -DDEBUG -c -I"/home/vicky/STM32CubeIDE/STM32F429xx_Drivers/Drivers" -I"/home/vicky/STM32CubeIDE/STM32F429xx_Drivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/002LEDWithBtn.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/syscalls.o: ../Src/syscalls.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F429I_DISC1 -DSTM32 -DSTM32F429ZITx -DSTM32F4 -DDEBUG -c -I"/home/vicky/STM32CubeIDE/STM32F429xx_Drivers/Drivers" -I"/home/vicky/STM32CubeIDE/STM32F429xx_Drivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/syscalls.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/sysmem.o: ../Src/sysmem.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F429I_DISC1 -DSTM32 -DSTM32F429ZITx -DSTM32F4 -DDEBUG -c -I"/home/vicky/STM32CubeIDE/STM32F429xx_Drivers/Drivers" -I"/home/vicky/STM32CubeIDE/STM32F429xx_Drivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/sysmem.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

