/*
 * stm32f4xx_SPI_Driver.c
 *
 *  Created on: Sep 11, 2020
 *      Author: Imtiaz Ahmed
 */

#include <stm32f4xx_SPI_Driver.h>

/*
 * API Function Prototypes for SPI
 */

// Peripheral Clock Setup
/********************************************************************
 * FUNCTION NAME	:	SPI_PeriClockControl
 *
 * BRIEF			:	This function enables or disables peripheral
 * 						clock for the given SPI port
 *
 * PARAM[0]			:	Base address of SPI port
 * PARAM[1]			:	ENABLE or DISABLE bit (macros)
 *
 * RETURN			:	NONE
 *
 * NOTE				:	N/A
 ********************************************************************/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE) {
		if (pSPIx == DRV_SPI1) { DRV_SPI1_PCLK_EN(); }
		else if (pSPIx == DRV_SPI2) { DRV_SPI2_PCLK_EN(); }
		else if (pSPIx == DRV_SPI3) { DRV_SPI3_PCLK_EN(); }
		else if (pSPIx == DRV_SPI4) { DRV_SPI4_PCLK_EN(); }
		else if (pSPIx == DRV_SPI5) { DRV_SPI5_PCLK_EN(); }
	} else {
		if (pSPIx == DRV_SPI1) { DRV_SPI1_PCLK_DI(); }
		else if (pSPIx == DRV_SPI2) { DRV_SPI2_PCLK_DI(); }
		else if (pSPIx == DRV_SPI3) { DRV_SPI3_PCLK_DI(); }
		else if (pSPIx == DRV_SPI4) { DRV_SPI4_PCLK_DI(); }
		else if (pSPIx == DRV_SPI5) { DRV_SPI5_PCLK_DI(); }
	}
}

// Init & Deinit
// Init & Deinit
/********************************************************************
 * FUNCTION NAME	:	SPI_Init
 *
 * BRIEF			:	This function initiates the SPIx port
 *
 * PARAM[0]			:	A structure pointer that holds the base address
 * 						of the SPI that needs to be initiated as well as
 * 						a pointer to a structure that specifies the
 * 						configuration for that port.
 *
 * RETURN			:	NONE
 *
 * NOTE				:	N/A
 ********************************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	// Initiate SPI peripheral clock control
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	uint32_t temp = 0;

	// Configure device mode
	temp |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	// Configure BUS Config
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_FULLDUPLEX) {
		// BIDI bit should be reset
		temp &= ~(SET << SPI_CR1_BIDIMODE);
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_HALFDUPLEX) {
		// BIDI bit should be set
		temp |= (SET << SPI_CR1_BIDIMODE);
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_SIMPLEX_RXONLY) {
		// BIDI bit should be cleared
		// RXONLY bit must be set
		temp &= ~(SET << SPI_CR1_BIDIMODE);
		temp |= (SET << SPI_CR1_RXONLY);
	}

	// Configure SSI
	if (pSPIHandle->SPIConfig.SPI_SMM == SPI_SSM_EN) {
		temp |= (SET << SPI_CR1_SSI);
	} else {
		temp &= ~(SET << SPI_CR1_SSI);
	}

	// Configure SPI serial clock speed (Baud Rate)
	temp |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BDR);

	// Configure DFF
	temp |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	// Configure SPI CPOL
	temp |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	// Configure SPI CPHA
	temp |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	// Configutre SPI SMM
	temp |= (pSPIHandle->SPIConfig.SPI_SMM << SPI_CR1_SSM);

	// Initialize the entire CR1 register
	pSPIHandle->pSPIx->CR1 = temp;
}

// Init & Deinit
/********************************************************************
 * FUNCTION NAME	:	SPI_DeInit
 *
 * BRIEF			:	This function deinitiate the SPIx port
 *
 * PARAM[0]			:	A structure pointer that holds the base address
 * 						of the SPI that needs to be initiated as well as
 * 						a pointer to a structure that specifies the
 * 						configuration for that port.
 *
 * RETURN			:	NONE
 *
 * NOTE				:	N/A
 ********************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if (pSPIx == DRV_SPI1) { SPI1_REG_RESET(); }
	else if (pSPIx == DRV_SPI2) { SPI2_REG_RESET(); }
	else if (pSPIx == DRV_SPI3) { SPI3_REG_RESET(); }
	else if (pSPIx == DRV_SPI4) { SPI4_REG_RESET(); }
	else if (pSPIx == DRV_SPI5) { SPI5_REG_RESET(); }
}

// Data send and receive

// Send and receive data not using interrupt, which is a blocking method
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
	/*
	 * Note: This is a blocking function. Meaning, the function will not return
	 * until all the operations are complete. So if len == 1000, the function will
	 * block the application until all 1000 bytes are transferred.
	 *
	 * This is also called Polling type as the Tx buffer loop may run forever due to
	 * some error. We'll have to set up watchdogs for that.
	 */
	while (len > 0) {
		// Wait until TX Buffer is empty
		while (SPI_GetFlagStatus(pSPIx, SPI_SR_TXE_FLAG) == FLAG_RESET);

		// Check DFF bit in CR1
		if (pSPIx->CR1 & SPI_CR1_DFF) {
			// 16 bit DFF
			// Load data to DR
			pSPIx->DR = *((uint16_t *) pTxBuffer);								// Convert the pTxBuffer to 16 bit and dereference it
			len = len - 2;
			(uint16_t *) pTxBuffer++;
		} else {
			// 8 bit DFF
			pSPIx->DR = *(pTxBuffer);
			len--;
			pTxBuffer++;
		}
	}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{
	while (len > 0) {
		// Wait until RX Buffer is not empty
		while (SPI_GetFlagStatus(pSPIx, SPI_SR_RXNE_FLAG == FLAG_SET));

		// Check DFF bit in CR1
		if (pSPIx->CR1 & SPI_CR1_DFF) {
			*((uint16_t *) pRxBuffer) = pSPIx->DR;
			len = len - 2;
			(uint16_t *) pRxBuffer++;
		} else {
			*pRxBuffer = pSPIx->DR;
			len--;
			pRxBuffer++;
		}
	}
}

// Send and receive data using interrupt, which is a non-blocking method
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len)
{
	// 0. Do everything as long as SPI is not busy
	uint8_t SPI_State = pSPIHandle->TXState;

	if (SPI_State != SPI_BUSY_IN_TX) {
		// 1. Save the TX buffer address and len information in some global variable
		pSPIHandle->pTXBuffer = pTxBuffer;
		pSPIHandle->TXLen = len;

		// 2. Mark the SPI state as busy in transmission so that no other code
		//	  can take over the SPI peripheral until transmission is over
		pSPIHandle->TXState = SPI_BUSY_IN_TX;

		// 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set
		//	  in the status register (SR)
		pSPIHandle->pSPIx->CR2 |= (ENABLE << SPI_CR2_TXEIE);

		// 4. Transmit data using the ISR (SPI_Handle())
	}

	return SPI_State;
}


uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len)
{
	// 0. Do everything as long as SPI is not busy
	uint8_t SPI_State = pSPIHandle->RXState;

	if (SPI_State != SPI_BUSY_IN_RX) {
		// 1. Save the RX buffer address and len information in some global variable
		pSPIHandle->pRXBuffer = pRxBuffer;
		pSPIHandle->RXLen = len;

		// 2. Mark the SPI state as busy in reception so that no other code
		//	  can take over the SPI peripheral until reception is over
		pSPIHandle->RXState = SPI_BUSY_IN_RX;

		// 3. Enable the RXNEIE control bit to get interrupt whenever RXE flag is set
		//	  in the status register (SR)
		pSPIHandle->pSPIx->CR2 |= (ENABLE << SPI_CR2_RXNEIE);

		// 4. Receive data using the ISR (SPI_Handle())
	}

	return SPI_State;
}

// IRQ Config and ISR handling
/********************************************************************
 * FUNCTION NAME	:	SPI_IRQInterruptConfig
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
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI)
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// Setup Priority
	NVIC_RegDef_t *pNVIC;
	pNVIC = DRV_NVIC_IPR_BASEADDR;
	uint8_t total_shift = ((IRQNumber % 4) * 8) + (8 - NON_IMPLEMENTED_PR_BITS);
	pNVIC->PRIORITY[IRQNumber / 4] |= (IRQPriority << total_shift);
}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	// 0. Check why the interrupt happened
	uint8_t temp1, temp2, temp3, temp4, temp5;

	// For TXE
	temp1 = pSPIHandle->pSPIx->SR & (SET << SPI_SR_TXE_FLAG);
	temp2 = pSPIHandle->pSPIx->CR2 & (SET << SPI_CR2_TXEIE);

	if (temp1 && temp2) {
		// Since both are true, that means TXE interrupt has occured
		// and we'll have to handle TXE interrupt
		SPI_TXEInterruptHandle(pSPIHandle);
	}

	// For RXE
	temp1 = pSPIHandle->pSPIx->SR & (SET << SPI_SR_RXNE_FLAG);
	temp2 = pSPIHandle->pSPIx->CR2 & (SET << SPI_CR2_RXNEIE);

	if (temp1 && temp2) {
		// Since both are true, that means RXNE interrupt has occured
		// and we'll have to handle RXNE interrupt
		SPI_RXNEInterruptHandle(pSPIHandle);
	}

	// For ERRIE
	temp1 = pSPIHandle->pSPIx->SR & (SET << SPI_SR_MODF);
	temp2 = pSPIHandle->pSPIx->SR & (SET << SPI_SR_OVR);
	temp3 = pSPIHandle->pSPIx->SR & (SET << SPI_SR_CRCERR);
	temp4 = pSPIHandle->pSPIx->SR & (SET << SPI_SR_FRE);
	temp5 = pSPIHandle->pSPIx->CR2 & (SET << SPI_CR2_ERRIE);

	if (temp1 && temp5) {
		// Handle MODF
	} else if (temp2 && temp5) {
		// Handle OVR
		SPI_OVRInterruptHandle(pSPIHandle);
	} else if (temp3 && temp5) {
		// Handle CRCERR
	} else if (temp4 && temp5) {
		// Handle FRE
	}
}

// Other peripheral control functions

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName) {
	if (pSPIx->SR & FlagName) {
		return FLAG_SET;
	}

	return FLAG_RESET;
}


void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE) {
		pSPIx->CR1 |= ENABLE_SPE;
	} else {
		pSPIx->CR1 &= DISABLE_SPE;
	}
}

void SPI_ConfigureSSOE(SPI_RegDef_t *pSPIx, uint8_t EnorDI)
{
	if (EnorDI == ENABLE) {
		pSPIx->CR2 |= ENABLE_SSOE;
	} else {
		pSPIx->CR2 &= DISABLE_SSOE;
	}
}

void SPI_TXEInterruptHandle(SPI_Handle_t *pSPIHandle)
{
	// Check DFF bit in CR1
	if (pSPIHandle->pSPIx->CR1 & SPI_CR1_DFF) {
		// 16 bit DFF
		// Load data to DR
		pSPIHandle->pSPIx->DR = *((uint16_t *) pSPIHandle->pTXBuffer);								// Convert the pTxBuffer to 16 bit and dereference it
		pSPIHandle->TXLen = pSPIHandle->TXLen - 2;
		(uint16_t *) (pSPIHandle->pTXBuffer)++;
	} else {
		// 8 bit DFF
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTXBuffer);
		(pSPIHandle->TXLen)--;
		(pSPIHandle->pTXBuffer)++;
	}

	if (!(pSPIHandle->TXLen)) {
		// Close SPI transmission and inform application tx is over
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_TX_COMPLETE);
	}
}

void SPI_RXNEInterruptHandle(SPI_Handle_t *pSPIHandle)
{
	// Check DFF bit in CR1
	if (pSPIHandle->pSPIx->CR1 & SPI_CR1_DFF) {
		*((uint16_t *) pSPIHandle->pRXBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RXLen = pSPIHandle->RXLen - 2;
		(uint16_t *) pSPIHandle->pRXBuffer++;
	} else {
		*(pSPIHandle->pRXBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RXLen--;
		pSPIHandle->pRXBuffer++;
	}

	if (!(pSPIHandle->RXLen)) {
		// Close SPI transmission and inform application tx is over
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_RX_COMPLETE);
	}
}

void SPI_OVRInterruptHandle(SPI_Handle_t *pSPIHandle)
{
	// Clear OVR flag and then inform application
	// Clear by reading the DR and SR registers if SPI is not busy
	if (pSPIHandle->TXState != SPI_BUSY_IN_TX) {
		SPI_ClearOVRFlag(pSPIHandle->pSPIx);
	}

	SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_OVR_ERR);
}

__weak__ void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHandle, uint8_t event)
{
	//
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(ENABLE << SPI_CR2_TXEIE);										// Prevents interrupt from TXE flag
	pSPIHandle->pTXBuffer = NULL;
	pSPIHandle->TXLen = 0;
	pSPIHandle->TXState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(ENABLE << SPI_CR2_RXNEIE);										// Prevents interrupt from TXE flag
	pSPIHandle->pRXBuffer = NULL;
	pSPIHandle->RXLen = 0;
	pSPIHandle->RXState = SPI_READY;
}

// End of File
