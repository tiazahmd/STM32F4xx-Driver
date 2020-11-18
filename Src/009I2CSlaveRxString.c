/*
 * 007SPITxToArduinoSlaveTest.c
 *
 *  Created on: Oct 14, 2020
 *      Author: Imtiaz Ahmed
 *
 *   Using the following pins as I2C:
 *  	PB6 	== 	SCL
 *  	PB7  	== 	SDA
 *
 *   Button: PC13
 */

#include <stdio.h>
#include <string.h>
#include <stm32f4xx.h>

I2C_Handle_t I2CHandle;

// Arduino analog pins
#define ANALOG_PIN_0						0
#define ANALOG_PIN_1						1
#define ANALOG_PIN_2						2
#define ANALOG_PIN_3						3
#define ANALOG_PIN_4						4
#define ANALOG_PIN_5						5

// Address
#define MY_STM_DEVICE_ADDRESS				0x68
#define SLAVE_ADDRESS						0x68

void I2C1_GPIOInits(void);
void GPIO_Button_Init(void);
void I2C1_Inits(void);

int main(void)
{
	uint8_t data[] = "We are testing some I2C Master Tx.\n";

	I2C1_GPIOInits();
	I2C1_Inits();
	GPIO_Button_Init();
	I2C_PeripheralControl(&I2CHandle, ENABLE);

	// Wait for button press
	while (1) {
		while (!(GPIO_ReadFromInputPin(DRV_GPIOC, GPIO_PIN_NO_13))) {
			delay(3);
			// Send some data to the slave
			I2C_MasterSendData(&I2CHandle, data, strlen((char *) data), SLAVE_ADDRESS);
		}
	}
}

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = DRV_GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF4;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTTYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_PULLU;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	// SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);
}

// Button GPIO Init
void GPIO_Button_Init(void)
{
	GPIO_Handle_t gpioButton;

	gpioButton.pGPIOx = DRV_GPIOC;
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOPUPD;

	GPIO_Init(&gpioButton);
}

// Peripheral configurations for I2C
void I2C1_Inits(void)
{
	I2CHandle.pI2Cx = DRV_I2C1;
	I2CHandle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2CHandle.I2C_Config.I2C_DeviceAddress = MY_STM_DEVICE_ADDRESS; 			// Since it's a master device, address doesn't matter
	I2CHandle.I2C_Config.I2C_FMDutyCycle = I2C_FMDUTY_2;						// This doesn't matter either since duty cycle is necessary only for fast mode
	I2CHandle.I2C_Config.I2C_SCLSpeed = I2C_SCLSPEED_SM;						// 100khz

	I2C_Init(&I2CHandle);
}





// End of file
