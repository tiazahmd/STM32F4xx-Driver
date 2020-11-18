/*
 * 007SPITxToArduinoSlaveTest.c
 *
 *  Created on: Sep 14, 2020
 *      Author: Imtiaz Ahmed
 */

#include <string.h>
#include <stm32f4xx.h>

// The following function is used to initialize GPIO pins as SPI pins
void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = DRV_GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTTYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_PULLU;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	// MISO		<-- Disabling these ones as we don't need them
	//				and they are free for other functions
	// SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	// GPIO_Init(&SPIPins);

	// MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	// NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

// Button GPIO Init
void GPIOButton_Init(void)
{
	GPIO_Handle_t gpioButton;

	gpioButton.pGPIOx = DRV_GPIOC;
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOPUPD;

	GPIO_Init(&gpioButton);

}

// Peripheral configurations for SPI2
void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = DRV_SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_FULLDUPLEX;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SMM = SPI_SSM_DI;						// Hardware slave management enabled
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BIT;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;		// Generates serial clock of 2MHz

	SPI_Init(&SPI2Handle);
}

int main(void)
{
	char user_date[] = "Hi, this is Master(STM32) sending Slave(Arduino) some data";

	// Initiate GPIO as AF to be used for SPI2
	SPI2_GPIOInits();

	// Initiate GPIO Button
	GPIOButton_Init();

	// Initialize SPI peripherals
	SPI2_Inits();

	// Enable SSOE
	SPI_ConfigureSSOE(DRV_SPI2, ENABLE);

	while (1) {
		while (!(GPIO_ReadFromInputPin(DRV_GPIOC, GPIO_PIN_NO_13))) {
			delay(3);
			// Enable the SPI peripheral
			SPI_PeripheralControl(DRV_SPI2, ENABLE);

			// Send length to slave, so that it knows how much data to capture
			uint8_t dataLen = strlen(user_date);
			SPI_SendData(DRV_SPI2, &dataLen, 1);

			// Send actual data
			SPI_SendData(DRV_SPI2, (uint8_t *) user_date, strlen(user_date));

			// Wait while SPI is busy
			while (!(SPI_GetFlagStatus(DRV_SPI2, SPI_SR_BSY_FLAG)));
			SPI_PeripheralControl(DRV_SPI2, DISABLE);

		}
	}

	return 0;
}
