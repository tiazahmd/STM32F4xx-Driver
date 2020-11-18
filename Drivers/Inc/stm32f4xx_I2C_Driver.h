/*
 * stm32f4xx_SPI_Driver.h
 *
 *  Created on: Oct 5, 2020
 *      Author: Imtiaz Ahmed
 */

#ifndef INC_STM32F4XX_I2C_DRIVER_H_
#define INC_STM32F4XX_I2C_DRIVER_H_

#include <stm32f4xx.h>

// I2C Configuration & Handle Structure
typedef struct {
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_ACKControl;
	uint8_t I2C_FMDutyCycle;
} I2C_Config_t;

typedef struct {
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
} I2C_Handle_t;

// Register Definition Macros: CR1
#define I2C_CR1_PE					0
#define I2C_CR1_SMBUS				1
#define I2C_CR1_SMBTYPE				3
#define I2C_CR1_ENARP				4
#define I2C_CR1_ENPEC				5
#define I2C_CR1_ENGC				6
#define I2C_CR1_NOSTRETCH			7
#define I2C_CR1_START				8
#define I2C_CR1_STOP				9
#define I2C_CR1_ACK					10
#define I2C_CR1_POS					11
#define I2C_CR1_PEC					12
#define I2C_CR1_ALERT				13
#define I2C_CR1_SWRST				15

// Register Definition Macros: CR2
#define I2C_CR2_FREQ				0
#define I2C_CR2_ITERREN				8
#define I2C_CR2_ITEVTEN				9
#define I2C_CR2_ITBUFEN				10
#define I2C_CR2_DMAEN				11
#define I2C_CR2_LAST				12

// Register Definition Macros: CR2
#define I2C_SR1_SB					0
#define I2C_SR1_ADDR				1
#define I2C_SR1_BTF					2
#define I2C_SR1_ADD10				3
#define I2C_SR1_STOPF				4
#define I2C_SR1_RXNE				6
#define I2C_SR1_TXE					7
#define I2C_SR1_BERR				8
#define I2C_SR1_ARLO				9
#define I2C_SR1_AF					10
#define I2C_SR1_OVR					11
#define I2C_SR1_PECERR				12
#define I2C_SR1_TIMEOUT				14
#define I2C_SR1_SMBALERT			15

// Register Definition Macros: CR2
#define I2C_SR2_MSL					0
#define I2C_SR2_BUSY				1
#define I2C_SR2_TRA					2
#define I2C_SR2_GENCALL				4
#define I2C_SR2_SMBDEFAULT			5
#define I2C_SR2_SMBHOST				6
#define I2C_SR2_DUALF				7
#define I2C_SR2_PEC					8

// Register Definition Macros: CCR
#define I2C_CCR_CCR					0
#define I2C_CCR_DUTY				14
#define I2C_SR2_FS					15

// I2C User Configurable Macro
#define I2C_SCLSPEED_SM				100000			// SM = Standard Mode (100khz)
#define I2C_SCLSPEED_FM2K			200000			// FM = Fast Mode (anything above 100khz)
#define I2C_SCLSPEED_FM4K			400000			// FM = Fast Mode (400khz)
#define I2C_ACK_ENABLE				1
#define I2C_ACK_DISABLE				0
#define I2C_FMDUTY_2				0
#define I2C_FMDUTY_16_9				1

// I2C Related Status Flags
#define I2C_FLAG_SB					(1 << I2C_SR1_SB)
#define I2C_FLAG_TXE				(1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE				(1 << I2C_SR1_RXNE)
#define I2C_FLAG_ADDR				(1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF				(1 << I2C_SR1_BTF)
#define I2C_FLAG_STOPF				(1 << I2C_SR1_STOPF)
#define I2C_FLAG_BERR				(1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO				(1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF					(1 << I2C_SR1_AF)
#define I2C_FLAG_OVR				(1 << I2C_SR1_OVR)
#define I2C_FLAG_TIMEOUT			(1 << I2C_SR1_TIMEOUT)

// Other macros

/*
 * I2C PE Enable/Disable
 */
#define ENABLE_PE								(1 << I2C_CR1_PE)
#define DISABLE_PE								~(1 << I2C_CR1_PE)

/*
 * API Function Prototypes for I2C
 */

// Peripheral Clock Control
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

// Init and Deinit
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_Deinit(I2C_RegDef_t *pI2Cx);

// Data send and receive
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint8_t len, uint8_t SlaveAddr);

// Interrupt config and handle
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t priority);

// Other peripheral controls
void I2C_PeripheralControl(I2C_Handle_t *pI2CHandle, uint8_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_ApplicationEventCallBack(I2C_Handle_t *pI2CHandle, uint8_t event);
uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPLLOutputClock(void);
void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);

#endif








// End of file
