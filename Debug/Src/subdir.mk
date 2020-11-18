################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/009I2CSlaveRxString.c 

OBJS += \
./Src/009I2CSlaveRxString.o 

C_DEPS += \
./Src/009I2CSlaveRxString.d 


# Each subdirectory must supply rules for building sources it contributes
Src/009I2CSlaveRxString.o: ../Src/009I2CSlaveRxString.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DDEBUG -c -I../Inc -I"/home/imtiaz/Desktop/Learning/FastBit/STM32F4xx_Drivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/009I2CSlaveRxString.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

