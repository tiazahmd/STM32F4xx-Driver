/*
 * stm32f4xx_GPIO_Driver.h
 *
 *  Created on: Aug 26, 2020
 *      Author: Imtiaz Ahmed
 */

#ifndef INC_STM32F4XX_GPIO_DRIVER_H_
#define INC_STM32F4XX_GPIO_DRIVER_H_

#include <stm32f4xx.h>

typedef struct {
	uint8_t GPIO_PinNumber;																// Possible values from @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode;																// Possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;																// Possible values from @GPIO_SPEED
	uint8_t GPIO_PinPuPdControl;														// Possible values from @GPIO_PUPDR
	uint8_t GPIO_PinOPType;																// Possible values from @GPIO_OUTPUT_TYPES
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

/*
 * Create GPIOx handle & config structures
 */

// Handle
typedef struct {
	GPIO_RegDef_t *pGPIOx;																// holds the base address of GPIO
	GPIO_PinConfig_t GPIO_PinConfig;													// Holds GPIO Pin configurations
} GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * All possible GPIO pin numbers
 */
#define GPIO_PIN_NO_0						0
#define GPIO_PIN_NO_1						1
#define GPIO_PIN_NO_2						2
#define GPIO_PIN_NO_3						3
#define GPIO_PIN_NO_4						4
#define GPIO_PIN_NO_5						5
#define GPIO_PIN_NO_6						6
#define GPIO_PIN_NO_7						7
#define GPIO_PIN_NO_8						8
#define GPIO_PIN_NO_9						9
#define GPIO_PIN_NO_10						10
#define GPIO_PIN_NO_11						11
#define GPIO_PIN_NO_12						12
#define GPIO_PIN_NO_13						13
#define GPIO_PIN_NO_14						14
#define GPIO_PIN_NO_15						15

/*
 * @GPIO_PIN_MODES
 * GPIO Possible Pin Modes
 */
#define GPIO_MODE_IN 						0
#define GPIO_MODE_OUT 						1
#define GPIO_MODE_AF 						2
#define GPIO_MODE_ANALOG 					3
#define GPIO_MODE_IT_FT						4											// GPIO Interrupt: Falling Edge Trigger
#define GPIO_MODE_IT_RT						5											// GPIO Interrupt: Rising Edge Trigger
#define GPIO_MODE_IT_RFT					6											// GPIO Interrupt: Rising and Falling Edge Trigger

/*
 * @GPIO_OUTPUT_TYPES
 * GPIO Possible Output Types
 */
#define GPIO_OUTTYPE_PP						0
#define GPIO_OUTTYPE_OD						1

/*
 * @GPIO_SPEED
 * GPIO Possible Speed Modes
 */
#define GPIO_SPEED_LOW					0
#define GPIO_SPEED_MEDIUM				1
#define GPIO_SPEED_FAST					2
#define GPIO_SPEED_HIGH					3

/*
 * GPIO_PUPDR
 * GPIO Possible Pullup/Pulldown Registers
 */
#define GPIO_PUPDR_NOPUPD					0
#define GPIO_PUPDR_PULLU					1
#define GPIO_PUPDR_PULLD					2

/*
 * GPIO Alternate Function numbers
 */
#define GPIO_AF0							0
#define GPIO_AF1							1
#define GPIO_AF2							2
#define GPIO_AF3							3
#define GPIO_AF4							4
#define GPIO_AF5							5
#define GPIO_AF6							6
#define GPIO_AF7							7
#define GPIO_AF8							8
#define GPIO_AF9							9
#define GPIO_AF10							10
#define GPIO_AF11							11
#define GPIO_AF12							12
#define GPIO_AF13							13
#define GPIO_AF14							14
#define GPIO_AF15							15

/*
 * API Function Prototypes
 */

// Peripheral Clock Enable
void GPIO_PCLKControl(GPIO_RegDef_t * pGPIOx, uint8_t ENorDI);							// Enable GPIOx Peripheral Control Clock

// Init & Deinit
void GPIO_Init(GPIO_Handle_t *  pGPIOHandle);											// Initiate GPIOx
void GPIO_DeInit(GPIO_RegDef_t * pGPIOx);												// De-Initiate GPIOx

// Data Read/Write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);				// Read from one pin
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);									// Read from entire port
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);	// Write to one pin
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);						// Write to entire port
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);					// Turn the output pin on/off

// IRQ and ISR Handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI);						// Configure IRQ for GPIO; Interrupt Configuration
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);					// Configure IRQ Priority
void GPIO_IRQHandling(uint8_t PinNumber);												// What to do when an interrupt occurs; Something to
																						//		process the interrupt

#endif /* INC_STM32F4XX_GPIO_DRIVER_H_ */



// End fo file
