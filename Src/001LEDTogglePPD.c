/*
 * 001LEDToggle.c
 *
 *  Created on: Aug 27, 2020
 *      Author		: 	Imtiaz Ahmed
 *      Description	: 	Toggles the on oard LED on/off
 */

#include <stm32f4xx.h>

int notmain1(void)
{
	GPIO_Handle_t gpioLED;

	gpioLED.pGPIOx = DRV_GPIOA;
	gpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	gpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTTYPE_PP;
	gpioLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_PULLU;

	GPIO_PCLKControl(DRV_GPIOA, ENABLE);
	GPIO_Init(&gpioLED);

	while (1) {
		GPIO_ToggleOutputPin(DRV_GPIOA, GPIO_PIN_NO_5);
		delay(5);
	}

	return 0;
}

