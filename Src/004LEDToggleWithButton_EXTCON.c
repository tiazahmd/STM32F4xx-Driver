/*
 * 001LEDToggle.c
 *
 *  Created on: Aug 27, 2020
 *      Author			: 	Imtiaz Ahmed
 *      Description		: 	Toggles the external LED on/off
 *      					based on external switch
 *      External LED	:	PB9 [CN5 -> 10]
 *      External Button	:   PB8 [CN5 -> 9]
 */

#include <stm32f4xx.h>

int notmain4(void)
{
	GPIO_Handle_t extLED, extBTN;

	extLED.pGPIOx = DRV_GPIOB;
	extLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	extLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	extLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	extLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTTYPE_PP;
	extLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOPUPD;

	extBTN.pGPIOx = DRV_GPIOB;
	extBTN.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	extBTN.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	extBTN.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	extBTN.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_PULLU;

	GPIO_PCLKControl(DRV_GPIOB, ENABLE);

	GPIO_Init(&extLED);
	GPIO_Init(&extBTN);

	while (1) {
		if (!(GPIO_ReadFromInputPin(DRV_GPIOB, GPIO_PIN_NO_8))) {
			GPIO_ToggleOutputPin(DRV_GPIOB, GPIO_PIN_NO_9);
			delay(10);
		}
	}

	return 0;
}
