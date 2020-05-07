################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/CANx_Drivers.c \
../Drivers/Src/GPIOx_Drivers.c \
../Drivers/Src/I2Cx_Drivers.c \
../Drivers/Src/RCC_Drivers.c \
../Drivers/Src/SPIx_Drivers.c 

OBJS += \
./Drivers/Src/CANx_Drivers.o \
./Drivers/Src/GPIOx_Drivers.o \
./Drivers/Src/I2Cx_Drivers.o \
./Drivers/Src/RCC_Drivers.o \
./Drivers/Src/SPIx_Drivers.o 

C_DEPS += \
./Drivers/Src/CANx_Drivers.d \
./Drivers/Src/GPIOx_Drivers.d \
./Drivers/Src/I2Cx_Drivers.d \
./Drivers/Src/RCC_Drivers.d \
./Drivers/Src/SPIx_Drivers.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/CANx_Drivers.o: ../Drivers/Src/CANx_Drivers.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F429I_DISC1 -DSTM32 -DSTM32F429ZITx -DSTM32F4 -DDEBUG -c -I"D:/STM32CubeIDE/workspace_1.0.2/STM32F429xx_Drivers/Drivers/Inc" -I"D:/STM32CubeIDE/workspace_1.0.2/STM32F429xx_Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/CANx_Drivers.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/Src/GPIOx_Drivers.o: ../Drivers/Src/GPIOx_Drivers.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F429I_DISC1 -DSTM32 -DSTM32F429ZITx -DSTM32F4 -DDEBUG -c -I"D:/STM32CubeIDE/workspace_1.0.2/STM32F429xx_Drivers/Drivers/Inc" -I"D:/STM32CubeIDE/workspace_1.0.2/STM32F429xx_Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/GPIOx_Drivers.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/Src/I2Cx_Drivers.o: ../Drivers/Src/I2Cx_Drivers.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F429I_DISC1 -DSTM32 -DSTM32F429ZITx -DSTM32F4 -DDEBUG -c -I"D:/STM32CubeIDE/workspace_1.0.2/STM32F429xx_Drivers/Drivers/Inc" -I"D:/STM32CubeIDE/workspace_1.0.2/STM32F429xx_Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/I2Cx_Drivers.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/Src/RCC_Drivers.o: ../Drivers/Src/RCC_Drivers.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F429I_DISC1 -DSTM32 -DSTM32F429ZITx -DSTM32F4 -DDEBUG -c -I"D:/STM32CubeIDE/workspace_1.0.2/STM32F429xx_Drivers/Drivers/Inc" -I"D:/STM32CubeIDE/workspace_1.0.2/STM32F429xx_Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/RCC_Drivers.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/Src/SPIx_Drivers.o: ../Drivers/Src/SPIx_Drivers.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F429I_DISC1 -DSTM32 -DSTM32F429ZITx -DSTM32F4 -DDEBUG -c -I"D:/STM32CubeIDE/workspace_1.0.2/STM32F429xx_Drivers/Drivers/Inc" -I"D:/STM32CubeIDE/workspace_1.0.2/STM32F429xx_Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/SPIx_Drivers.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

