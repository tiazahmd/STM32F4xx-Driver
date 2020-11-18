/*
 * 001LEDToggle.c
 *
 *  Created on: Aug 27, 2020
 *      Author		: 	Imtiaz Ahmed
 *      Description	: 	Toggles the on oard LED on/off
 */

#include <stm32f4xx.h>

int main(void)
{
	GPIO_Handle_t gpioLED;
	GPIO_Handle_t gpioButton;

	gpioLED.pGPIOx = DRV_GPIOA;
	gpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	gpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTTYPE_PP;
	gpioLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_PULLU;

	gpioButton.pGPIOx = DRV_GPIOC;
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOPUPD;


	GPIO_PCLKControl(DRV_GPIOA, ENABLE);
	GPIO_PCLKControl(DRV_GPIOC, ENABLE);

	GPIO_Init(&gpioLED);
	GPIO_Init(&gpioButton);

	while (1) {
		if (!(GPIO_ReadFromInputPin(DRV_GPIOC, GPIO_PIN_NO_13))) {
			GPIO_ToggleOutputPin(DRV_GPIOA, GPIO_PIN_NO_5);
			delay(2);
		}
	}

	return 0;
}

