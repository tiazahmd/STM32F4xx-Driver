################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/stm32f4xx.c \
../Drivers/Src/stm32f4xx_GPIO_Driver.c \
../Drivers/Src/stm32f4xx_I2C_Driver.c \
../Drivers/Src/stm32f4xx_SPI_Driver.c 

OBJS += \
./Drivers/Src/stm32f4xx.o \
./Drivers/Src/stm32f4xx_GPIO_Driver.o \
./Drivers/Src/stm32f4xx_I2C_Driver.o \
./Drivers/Src/stm32f4xx_SPI_Driver.o 

C_DEPS += \
./Drivers/Src/stm32f4xx.d \
./Drivers/Src/stm32f4xx_GPIO_Driver.d \
./Drivers/Src/stm32f4xx_I2C_Driver.d \
./Drivers/Src/stm32f4xx_SPI_Driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/stm32f4xx.o: ../Drivers/Src/stm32f4xx.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DDEBUG -c -I../Inc -I"/home/imtiaz/Desktop/Learning/FastBit/STM32F4xx_Drivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/stm32f4xx.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/Src/stm32f4xx_GPIO_Driver.o: ../Drivers/Src/stm32f4xx_GPIO_Driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DDEBUG -c -I../Inc -I"/home/imtiaz/Desktop/Learning/FastBit/STM32F4xx_Drivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/stm32f4xx_GPIO_Driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/Src/stm32f4xx_I2C_Driver.o: ../Drivers/Src/stm32f4xx_I2C_Driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DDEBUG -c -I../Inc -I"/home/imtiaz/Desktop/Learning/FastBit/STM32F4xx_Drivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/stm32f4xx_I2C_Driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/Src/stm32f4xx_SPI_Driver.o: ../Drivers/Src/stm32f4xx_SPI_Driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DDEBUG -c -I../Inc -I"/home/imtiaz/Desktop/Learning/FastBit/STM32F4xx_Drivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/stm32f4xx_SPI_Driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

