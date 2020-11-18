/*
 * stm32f4xx_SPI_Driver.h
 *
 *  Created on: Sep 11, 2020
 *      Author: Imtiaz Ahmed
 */

#ifndef INC_STM32F4XX_SPI_DRIVER_H_
#define INC_STM32F4XX_SPI_DRIVER_H_

#include <stm32f4xx.h>

// SPI Configuration & Handle Structure
typedef struct {
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SMM;
} SPI_Config_t;

typedef struct {
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
	uint8_t *pTXBuffer;
	uint8_t *pRXBuffer;
	uint32_t TXLen;
	uint32_t RXLen;
	uint8_t TXState;
	uint8_t RXState;
} SPI_Handle_t;

/*
 * @SPI_DeviceMode
 * Possible Modes for SPI
 */

#define SPI_MODE_SLAVE							0
#define SPI_MODE_MASTER							1

/*
 * @SPI_BusConfig
 * Possible BUS Configurations for SPI
 */
#define SPI_BUS_FULLDUPLEX						0
#define SPI_BUS_HALFDUPLEX						1
#define SPI_BUS_SIMPLEX_RXONLY					2

/*
 * @SPI_SclkSpeed
 * Possible Baud Rate configuration for SPI
 */
#define SPI_SCLK_SPEED_DIV2						0				// If this is used, then fCLK / 2 will be implemented
#define SPI_SCLK_SPEED_DIV4						1				// If this is used, then fCLK / 4 will be implemented
#define SPI_SCLK_SPEED_DIV8						2				// If this is used, then fCLK / 8 will be implemented
#define SPI_SCLK_SPEED_DIV16					3				// If this is used, then fCLK / 16 will be implemented
#define SPI_SCLK_SPEED_DIV32					4				// If this is used, then fCLK / 32 will be implemented
#define SPI_SCLK_SPEED_DIV64					5				// If this is used, then fCLK / 64 will be implemented
#define SPI_SCLK_SPEED_DIV128					6				// If this is used, then fCLK / 128 will be implemented
#define SPI_SCLK_SPEED_DIV256					7				// If this is used, then fCLK / 128 will be implemented

/*
 * SPI Control Register 1 Macros
 */
#define SPI_CR1_CPHA							0
#define SPI_CR1_CPOL							1
#define SPI_CR1_MSTR							2
#define SPI_CR1_BDR								3
#define SPI_CR1_SPE								6
#define SPI_CR1_LSBFST							7
#define SPI_CR1_SSI								8
#define SPI_CR1_SSM								9
#define SPI_CR1_RXONLY							10
#define SPI_CR1_DFF								11
#define SPI_CR1_CRCNEXT							12
#define SPI_CR1_CRCEN							13
#define SPI_CR1_BIDIOE							14
#define SPI_CR1_BIDIMODE						15

/*
 * SPI Control Register 2 Macros
 */
#define SPI_CR2_RXDMAEN							0
#define SPI_CR2_TXDMAEN							1
#define SPI_CR2_SSOE							2
#define SPI_CR2_FRF								4
#define SPI_CR2_ERRIE							5
#define SPI_CR2_RXNEIE							6
#define SPI_CR2_TXEIE							7

/*
 * SPI Status Register Macros
 */
#define SPI_SR_RXNE								0
#define SPI_SR_TXE								1				// TX Buffer empty/not empty. If TXE == 1, TX Buffer is empty
#define SPI_SR_CHSIDE							2
#define SPI_SR_UDR								3
#define SPI_SR_CRCERR							4
#define SPI_SR_MODF								5
#define SPI_SR_OVR								6
#define SPI_SR_BSY								7
#define SPI_SR_FRE								8

/*
 * @SPI_DFF
 * Possible DFF configuration for SPI
 */
#define SPI_DFF_8BIT							0				// 8 Bit data frame is selected
#define SPI_DFF_16BIT							1				// 16 Bit data frame is selected

/*
 * @SPI_CPOL
 * Possible CPOL configuration for SPI
 */
#define SPI_CPOL_LOW							0
#define SPI_CPOL_HIGH							1

/*
 * @SPI_CPHA
 * Possible CPHA configuration for SPI
 */
#define SPI_CPHA_LOW							0
#define SPI_SPHA_HIGH							1

/*
 * @SPI_SMM
 * Possible SMM configuration for SPI
 */
#define SPI_SSM_DI								0
#define SPI_SSM_EN								1

/*
 * Other macros
 */
#define FLAG_RESET								RESET
#define FLAG_SET								SET

/*
 * SPI Flags
 */
#define SPI_SR_RXNE_FLAG						(0 << SPI_SR_RXNE)
#define SPI_SR_TXE_FLAG							(1 << SPI_SR_TXE)
#define SPI_SR_CHSIDE_FLAG						(2 << SPI_SR_TXE)
#define SPI_SR_UDR_FLAG							(3 << SPI_SR_TXE)
#define SPI_SR_CRCERR_FLAG						(4 << SPI_SR_TXE)
#define SPI_SR_MODF_FLAG						(5 << SPI_SR_TXE)
#define SPI_SR_OVR_FLAG							(6 << SPI_SR_TXE)
#define SPI_SR_BSY_FLAG							(7 << SPI_SR_TXE)
#define SPI_SR_FRE_FLAG							(8 << SPI_SR_TXE)

/*
 * SPI CR1 SPE Enable/Disable
 */
#define ENABLE_SPE								(1 << SPI_CR1_SPE)
#define DISABLE_SPE								~(1 << SPI_CR1_SPE)

/*
 * SPI SSOE Enable/Disable
 */
#define ENABLE_SSOE								(1 << SPI_CR2_SSOE)
#define DISABLE_SSOE							~(1 << SPI_CR2_SSOE)

/*
 * SPI Application states
 */
#define SPI_READY								0
#define SPI_BUSY_IN_RX							1
#define SPI_BUSY_IN_TX							2

/*
 * SPI Event callback macros
 */
#define SPI_EVENT_TX_COMPLETE					1
#define SPI_EVENT_RX_COMPLETE					2
#define SPI_EVENT_OVR_ERR						3
#define SPI_EVENT_CRC_ERR						4
#define SPI_EVENT_MODF_ERR						5
#define SPI_EVENT_FRE_ERR						6

/*
 * API Function Prototypes for SPI
 */

// Peripheral Clock setup
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

// Init & Deinit
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

// Data send and receive
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len);

// IRQ Config and ISR handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t priority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

// Other peripheral control functions
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_ConfigureSSOE(SPI_RegDef_t *pSPIx, uint8_t EnorDI);
void SPI_TXEInterruptHandle(SPI_Handle_t *pSPIHandle);
void SPI_RXNEInterruptHandle(SPI_Handle_t *pSPIHandle);
void SPI_OVRInterruptHandle(SPI_Handle_t *pSPIHandle);
void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHandle, uint8_t event);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);		// In case interrupt handle doesn't/can't clear
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);		// In case interrupt doesn't/can't close
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);		// In case interrupt doesn't/can't close

#endif /* INC_STM32F4XX_SPI_DRIVER_H_ */
