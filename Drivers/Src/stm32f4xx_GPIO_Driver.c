/*
 * stm32f4xx_GPIO_Driver.c
 *
 *  Created on: Aug 26, 2020
 *      Author: Imtiaz Ahmed
 */

#include <stm32f4xx_GPIO_Driver.h>

/*
 * API Functions
 */

// Peripheral Clock Enable

/********************************************************************
 * FUNCTION NAME	:	GPIO_PCLKControl
 *
 * BRIEF			:	This function enables or disables peripheral
 * 						clock for the given GPIO port
 *
 * PARAM[0]			:	Base address of GPIO port
 * PARAM[1]			:	ENABLE or DISABLE bit (macros)
 *
 * RETURN			:	NONE
 *
 * NOTE				:	N/A
 ********************************************************************/
void GPIO_PCLKControl(GPIO_RegDef_t * pGPIOx, uint8_t ENorDI)
{
	if (ENorDI == ENABLE) {
		if (pGPIOx == DRV_GPIOA) { DRV_GPIOA_PCLK_EN(); }
		else if (pGPIOx == DRV_GPIOB) { DRV_GPIOB_PCLK_EN(); }
		else if (pGPIOx == DRV_GPIOC) { DRV_GPIOC_PCLK_EN(); }
		else if (pGPIOx == DRV_GPIOD) { DRV_GPIOD_PCLK_EN(); }
		else if (pGPIOx == DRV_GPIOE) { DRV_GPIOE_PCLK_EN(); }
		else if (pGPIOx == DRV_GPIOH) { DRV_GPIOH_PCLK_EN(); }
	} else {
		if (pGPIOx == DRV_GPIOA) { DRV_GPIOA_PCLK_DI(); }
		else if (pGPIOx == DRV_GPIOB) { DRV_GPIOB_PCLK_DI(); }
		else if (pGPIOx == DRV_GPIOC) { DRV_GPIOC_PCLK_DI(); }
		else if (pGPIOx == DRV_GPIOD) { DRV_GPIOD_PCLK_DI(); }
		else if (pGPIOx == DRV_GPIOE) { DRV_GPIOE_PCLK_DI(); }
		else if (pGPIOx == DRV_GPIOH) { DRV_GPIOH_PCLK_DI(); }
	}
}

// Init & Deinit
/********************************************************************
 * FUNCTION NAME	:	GPIO_Init
 *
 * BRIEF			:	This function initiates the GPIOx port
 *
 * PARAM[0]			:	A structure pointer that holds the base address
 * 						of the GPIO that needs to be initiated as well as
 * 						a pointer to a structure that specifies the
 * 						configuration for that port.
 *
 * RETURN			:	NONE
 *
 * NOTE				:	N/A
 ********************************************************************/
void GPIO_Init(GPIO_Handle_t *  pGPIOHandle)
{
	// 0. Initiate the GPIO BUS
	GPIO_PCLKControl(pGPIOHandle->pGPIOx, ENABLE);

	uint32_t temp = 0;
	// 1. Configure the GPIO Mode
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		// Non-interrupt mode setup
		temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0;
	} else {
		// Interrupt mode setp
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
			// Configure the falling trigger register
			DRV_EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			// Clear the corresponding RTSR bit just in case it was enabled
			DRV_EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {
			// Configure the rising trigger register
			DRV_EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			// Clear the corresponding FTSR bit just in case it was enabled
			DRV_EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {
			// Configure the rising and falling trigger register
			DRV_EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			DRV_EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// Configure the GPIO port selection on SYSCFG_EXTICR register
		uint8_t t1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / EXTI_REG_BITS);
		uint8_t t2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % EXTI_REG_BITS);

		DRV_SYSCFG_PCLK_EN();

		// Get port with macro function:
		uint8_t portcode = GPIO_PORT_CODE(pGPIOHandle->pGPIOx);
		DRV_SYSCFG->EXTICR[t1] &= ~(0xF << (t2 * EXTI_REG_BITS));
		DRV_SYSCFG->EXTICR[t1] |= (portcode << (t2 * EXTI_REG_BITS));

		// Get port without macro function:
//		if (pGPIOHandle->pGPIOx = DRV_GPIOA) {
//			DRV_SYSCFG->EXTICR[t1] &= ~(0xF << (t2 * EXTI_REG_BITS));
//			DRV_SYSCFG->EXTICR[t1] |= (EXTI_PA << (t2 * EXTI_REG_BITS));
//		} else if (pGPIOHandle->pGPIOx = DRV_GPIOB) {
//			DRV_SYSCFG->EXTICR[t1] &= ~(0xF << (t2 * EXTI_REG_BITS));
//			DRV_SYSCFG->EXTICR[t1] |= (EXTI_PB << (t2 * EXTI_REG_BITS));
//		} else if (pGPIOHandle->pGPIOx = DRV_GPIOC) {
//			DRV_SYSCFG->EXTICR[t1] &= ~(0xF << (t2 * EXTI_REG_BITS));
//			DRV_SYSCFG->EXTICR[t1] |= (EXTI_PC << (t2 * EXTI_REG_BITS));
//		} else if (pGPIOHandle->pGPIOx = DRV_GPIOD) {
//			DRV_SYSCFG->EXTICR[t1] &= ~(0xF << (t2 * EXTI_REG_BITS));
//			DRV_SYSCFG->EXTICR[t1] |= (EXTI_PD << (t2 * EXTI_REG_BITS));
//		} else if (pGPIOHandle->pGPIOx = DRV_GPIOE) {
//			DRV_SYSCFG->EXTICR[t1] &= ~(0xF << (t2 * EXTI_REG_BITS));
//			DRV_SYSCFG->EXTICR[t1] |= (EXTI_PE << (t2 * EXTI_REG_BITS));
//		} else if (pGPIOHandle->pGPIOx = DRV_GPIOH) {
//			DRV_SYSCFG->EXTICR[t1] &= ~(0xF << (t2 * EXTI_REG_BITS));
//			DRV_SYSCFG->EXTICR[t1] |= (EXTI_PH << (t2 * EXTI_REG_BITS));
//		}

		// Enable the EXTI interrupt delivery using IMR
		DRV_EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	// 2. Configure GPIO Speed
	temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed) << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	// 3. Configure PUPDR
	temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl) << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	// 4. Configure output type
	temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType) << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	// 5. Configure alternate functionality, if mode == alternate function
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_AF) {
		uint8_t t1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / AF_REG_NO;
		uint8_t t2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % AF_REG_NO;
		temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode) << (AF_REG_BITS * t2));
		pGPIOHandle->pGPIOx->AFR[t1] &= ~(0xF << (AF_REG_BITS * t2));
		pGPIOHandle->pGPIOx->AFR[t1] |= temp;
		temp = 0;
	}
}

/********************************************************************
 * FUNCTION NAME	:	GPIO_DeInit
 *
 * BRIEF			:	This function de-initiates the GPIOx port
 *
 * PARAM[0]			:	Since only resetting a certain bit of the GPIO
 * 						port's register resets everything for that
 * 						port, this function takes only the GPIO's base
 * 						address and uses that to access the register
 * 						to reset the GPIO.
 *
 * RETURN			:	NONE
 *
 * NOTE				:	N/A
 ********************************************************************/
void GPIO_DeInit(GPIO_RegDef_t * pGPIOx)
{
	if (pGPIOx == DRV_GPIOA) { GPIOA_REG_RESET(); }
	else if (pGPIOx == DRV_GPIOB) { GPIOB_REG_RESET(); }
	else if (pGPIOx == DRV_GPIOC) { GPIOC_REG_RESET(); }
	else if (pGPIOx == DRV_GPIOD) {	GPIOD_REG_RESET(); }
	else if (pGPIOx == DRV_GPIOE) {	GPIOE_REG_RESET(); }
	else if (pGPIOx == DRV_GPIOH) {	GPIOH_REG_RESET(); }
}

// Data Read/Write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x01);
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t) (pGPIOx->IDR);
	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	if (value == GPIO_PIN_SET) {
		pGPIOx->ODR |= (1 << PinNumber);
	} else {
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}

/********************************************************************
 * FUNCTION NAME	:	GPIO_ToggleOutputPin
 *
 * BRIEF			:	Toggles the output data register
 *
 * PARAM[0]			:	The register pointer to GPIO
 * PARAM[1]			:	The pin number for that GPIO
 *
 * RETURN			:	NONE
 *
 * NOTE				:	N/A
 ********************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/********************************************************************
 * FUNCTION NAME	:	GPIO_IRQInterruptConfig
 *
 * BRIEF			:	Configure the NVIC of the ARM Cortex. This is
 * 						processor specific - everything that happens
 * 						in this function affects processor
 * 						regissters.
 *
 * PARAM[0]			:	The IRQ number or position number
 * PARAM[1]			:	To set the priority for the IRQ
 * PARAM[2]			:	ENABLE or DISABLE
 *
 * RETURN			:	NONE
 *
 * NOTE				:	N/A
 ********************************************************************/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI)
{
	NVIC_RegDef_t *pNVIC;
	if (ENorDI == ENABLE) {
		if (IRQNumber <= 31) {
			// Program the ISER0 register
			pNVIC = DRV_NVIC_ISER0_BASEADDR;
			pNVIC->SETENA |= (ENABLE << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber <= 63) {
			// Program the ISER1 register
			pNVIC = DRV_NVIC_ISER1_BASEADDR;
			pNVIC->SETENA |= (ENABLE << IRQNumber % 32);
		} else if (IRQNumber > 63 && IRQNumber <= 95) {
			// Program the ISER2 register
			pNVIC = DRV_NVIC_ISER2_BASEADDR;
			pNVIC->SETENA |= (ENABLE << IRQNumber % 64);
		}
	} else {
		if (IRQNumber <= 31) {
			// Program the ICER0 register
			pNVIC = DRV_NVIC_ICER0_BASEADDR;
			pNVIC->CLRENA |= (ENABLE << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber <= 63) {
			// Program the ICER1 register
			pNVIC = DRV_NVIC_ICER1_BASEADDR;
			pNVIC->CLRENA |= (ENABLE << IRQNumber % 32);
		} else if (IRQNumber > 63 && IRQNumber <= 95) {
			// Program the ICER2 register
			pNVIC = DRV_NVIC_ICER2_BASEADDR;
			pNVIC->CLRENA |= (ENABLE << IRQNumber % 64);
		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	// Setup Priority
	NVIC_RegDef_t *pNVIC;
	pNVIC = DRV_NVIC_IPR_BASEADDR;
	uint8_t total_shift = ((IRQNumber % 4) * 8) + (8 - NON_IMPLEMENTED_PR_BITS);
	pNVIC->PRIORITY[IRQNumber / 4] |= (IRQPriority << total_shift);
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	// Clear the pending bit register corresponding to the Pin Number
	if (DRV_EXTI->PR & (1 << PinNumber)) {
		DRV_EXTI->PR |= (1 << PinNumber);
	}
}






// End of File
