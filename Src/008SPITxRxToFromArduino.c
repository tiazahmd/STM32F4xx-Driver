/*
 * 007SPITxToArduinoSlaveTest.c
 *
 *  Created on: Sep 15, 2020
 *      Author: Imtiaz Ahmed
 *
 *   Using the following pins as SPI2:
 *  	PB12 	== 	NSS
 *  	PB13  	== 	SCK
 *  	PB14	==	MISO
 *  	PB15	==	MOSI
 *  	AF	    ==	5
 */


#include <string.h>
#include <stm32f4xx.h>

extern void initialise_monitor_handles();

// Command Codes
#define COMMAND_LED_CTRL					0x50
#define COMMAND_SENSOR_READ					0x51
#define COMMAND_LED_READ					0x52
#define COMMAND_PRINT						0x53
#define COMMAND_ID_READ						0x54

#define LED_ON								1
#define LED_OFF								0

// Arduino analog pins
#define ANALOG_PIN_0						0
#define ANALOG_PIN_1						1
#define ANALOG_PIN_2						2
#define ANALOG_PIN_3						3
#define ANALOG_PIN_4						4
#define ANALOG_PIN_5						5

// Arduino LED Pin
#define LED_PIN								9

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

	 // MISO
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    GPIO_Init(&SPIPins);

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

uint8_t SPI_VerifyResponse(uint8_t ack_byte)
{
	if (ack_byte == 0xF5) {
		// Slave returned ack
		return TRUE;
	}

	// Else Slave return nack
	return FALSE;
}

int main(void)
{
	uint8_t command_code;
	uint8_t dummy_write_byte = 0xff;
	uint8_t dummy_read_byte;
	uint8_t ack_byte;
	uint8_t analog_read;
	uint8_t led_read;
	uint8_t args[2];
	char user_data[] = "Hello, world!";
	uint8_t data_inp[11];

	initialise_monitor_handles();

	// Initiate GPIO as AF to be used for SPI2
	SPI2_GPIOInits();

	// Initiate GPIO Button
	GPIOButton_Init();

	// Initialize SPI peripherals
	SPI2_Inits();

	// Enable SSOE
	SPI_ConfigureSSOE(DRV_SPI2, ENABLE);

	printf("Configurations initialized and enabled.\n");

	while (1) {
		while (!(GPIO_ReadFromInputPin(DRV_GPIOC, GPIO_PIN_NO_13))) {
			// Enable the SPI peripheral
			SPI_PeripheralControl(DRV_SPI2, ENABLE);

			// 1. Send first command: COMMAND_LED_CONTROL	<pin no (1)>	<value (1)>
			command_code = COMMAND_LED_CTRL;
			SPI_SendData(DRV_SPI2, &command_code, 1);

			// Do a dummy read
			SPI_ReceiveData(DRV_SPI2, &dummy_read_byte, 1);

			// Send 1 byte of dummy data to fetch response from slave
			SPI_SendData(DRV_SPI2, &dummy_write_byte, 1);

			// Read the ack byte sent by slave
			SPI_ReceiveData(DRV_SPI2, &ack_byte, 1);

			// Verify if the sent data is ack or nack byte
			if (SPI_VerifyResponse(ack_byte)) {
				// Send arguments
				args[0] = LED_PIN;
				args[1] = LED_ON;
				SPI_SendData(DRV_SPI2, args, 2);
			}

			printf("LED turned on at slave.\n");

			delay(5);
			// 2. COMMAND_SENSOR_READ	<analog pin no (1)>
			// Wait till button is pressed again
			while (GPIO_ReadFromInputPin(DRV_GPIOC, GPIO_PIN_NO_13));

			command_code = COMMAND_SENSOR_READ;
			SPI_SendData(DRV_SPI2, &command_code, 1);

			// Do a dummy read
			SPI_ReceiveData(DRV_SPI2, &dummy_read_byte, 1);

			// Send 1 byte of dummy data to fetch response from slave
			SPI_SendData(DRV_SPI2, &dummy_write_byte, 1);

			// Read the ack byte send by slave
			SPI_ReceiveData(DRV_SPI2, &ack_byte, 1);

			// Verify if the sent datta is ack or nack byte
			if (SPI_VerifyResponse(ack_byte)) {
				// Send argument
				args[0] = ANALOG_PIN_0;
				SPI_SendData(DRV_SPI2, args, 1);
			}

			// Do a dummy read
			SPI_ReceiveData(DRV_SPI2, &dummy_read_byte, 1);

			// Add some delay so that Slave can process its shit
			delay(2);

			// Send 1 byte of dummy data to fetch response from slave
			SPI_SendData(DRV_SPI2, &dummy_write_byte, 1);

			// Receive analog pin value from Slave
			SPI_ReceiveData(DRV_SPI2, &analog_read, 1);

			printf("Analog value read from slave: %d\n", analog_read);

			delay(3);

			// 3. COMMAND_LED_READ		<pin no (1)>
			// Wait till button is pressed again
			while (GPIO_ReadFromInputPin(DRV_GPIOC, GPIO_PIN_NO_13));

			command_code = COMMAND_LED_READ;
			SPI_SendData(DRV_SPI2, &command_code, 1);

			// Do a dummy read
			SPI_ReceiveData(DRV_SPI2, &dummy_read_byte, 1);

			// Send 1 byte of dummy data to fetch response from slave
			SPI_SendData(DRV_SPI2, &dummy_write_byte, 1);

			// Read the ack byte send by slave
			SPI_ReceiveData(DRV_SPI2, &ack_byte, 1);

			// Verify if the sent data is ack or nack byte
			if (SPI_VerifyResponse(ack_byte)) {
				// Send arguments
				args[0] = LED_PIN;printf("Slave successfully printed message.\n");
				SPI_SendData(DRV_SPI2, args, 1);
			}

			// Do a dummy read
			SPI_ReceiveData(DRV_SPI2, &dummy_read_byte, 1);

			// Add some delay so that Slave can process its shit
			delay(2);

			// Send 1 byte of dummy data to fetch response from slave
			SPI_SendData(DRV_SPI2, &dummy_write_byte, 1);

			// Receive analog pin value from Slave
			SPI_ReceiveData(DRV_SPI2, &led_read, 1);

			printf("LED Value read from slave: %d\n", led_read);

			delay(3);

			// 4. COMMAND_PRINT 	<len (2)>		<message (len)>

			// Wait till button is pressed again
			while (GPIO_ReadFromInputPin(DRV_GPIOC, GPIO_PIN_NO_13));

			command_code = COMMAND_PRINT;
			SPI_SendData(DRV_SPI2, &command_code, 1);

			// Do a dummy read
			SPI_ReceiveData(DRV_SPI2, &dummy_read_byte, 1);

			// Send 1 byte of dummy data to fetch response from slave
			SPI_SendData(DRV_SPI2, &dummy_write_byte, 1);

			// Read the ack byte send by slave
			SPI_ReceiveData(DRV_SPI2, &ack_byte, 1);

			// Verify if the sent data is ack or nack byte
			if (SPI_VerifyResponse(ack_byte)) {
				// Send arguments
				args[0] = strlen(user_data);
				SPI_SendData(DRV_SPI2, args, 1);
				SPI_SendData(DRV_SPI2, (uint8_t *) user_data, strlen(user_data));
				printf("Slave successfully printed message.\n");
			}

			delay(2);

			// 5. COMMAND_ID_READ
			// Wait till button is pressed again
			while (GPIO_ReadFromInputPin(DRV_GPIOC, GPIO_PIN_NO_13));

			delay(2);

			command_code = COMMAND_ID_READ;
			SPI_SendData(DRV_SPI2, &command_code, 1);

			// Do a dummy read
			SPI_ReceiveData(DRV_SPI2, &dummy_read_byte, 1);

			// Send 1 byte of dummy data to fetch response from slave
			SPI_SendData(DRV_SPI2, &dummy_write_byte, 1);

			// Read the ack byte send by slave
			SPI_ReceiveData(DRV_SPI2, &ack_byte, 1);

			if (SPI_VerifyResponse(ack_byte)) {
				// Read 11 bytes from slave
				for (uint32_t i = 0; i < 10; i++) {
					SPI_SendData(DRV_SPI2, &dummy_write_byte, 1);
					SPI_ReceiveData(DRV_SPI2, &data_inp[i], 1);
				}
			}

			data_inp[10] = '\0';

			printf("Arduino board: %s\n", data_inp);

			// Wait while SPI is busy
			while (!(SPI_GetFlagStatus(DRV_SPI2, SPI_SR_BSY_FLAG)));
			SPI_PeripheralControl(DRV_SPI2, DISABLE);

			printf("SPI communication complete.\n");
		}
	}
}
