/*
 * 005LEDToggleWithInterrupt.c
 *
 *  Created on: Aug 28, 2020
 *      Author			: 	Imtiaz Ahmed
 *      Description		: 	Toggles the internal LED on/off
 *      					based on interrupt
 *      LED				:	D14 (PB9)
 *      BUTTON			:	D15 (PB8)
 */

#include <stm32f4xx.h>
#include <string.h>

int notmain(void)
{
	GPIO_Handle_t extLED, extBTN;

	// Clear the extLED and extBTN bits just in case
	// they have garbage value

	memset(&extLED, 0, sizeof(extLED));
	memset(&extBTN, 0, sizeof(extBTN));

	// LED pin config
	extLED.pGPIOx = DRV_GPIOB;
	extLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	extLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	extLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	extLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTTYPE_PP;
	extLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOPUPD;

	// BUTTON pin config
	extBTN.pGPIOx = DRV_GPIOB;
	extBTN.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	extBTN.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	extBTN.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	extBTN.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_PULLU;

	GPIO_PCLKControl(DRV_GPIOB, ENABLE);

	GPIO_Init(&extBTN);
	GPIO_Init(&extLED);

	// IRQ Configurations
	GPIO_IRQPriorityConfig(EXTI9_5_IRQ, NVIC_IRQ_PRIORITY_15);
	GPIO_IRQInterruptConfig(EXTI9_5_IRQ, ENABLE);

	// Infinite Loop
	while (1) {

	}

}

void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_8);
	GPIO_ToggleOutputPin(DRV_GPIOB, GPIO_PIN_NO_9);
	delay(10);
}
