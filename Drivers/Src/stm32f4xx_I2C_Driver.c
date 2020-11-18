/*
 * stm32f4xx_SPI_Driver.c
 *
 *  Created on: Oct 5, 2020
 *      Author: Imtiaz Ahmed
 */

#include <stm32f4xx_I2C_Driver.h>

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE) {
		if (pI2Cx == DRV_I2C1) { DRV_I2C1_PCLK_EN(); }
		else if (pI2Cx == DRV_I2C2) { DRV_I2C2_PCLK_EN(); }
		else if (pI2Cx == DRV_I2C3) { DRV_I2C3_PCLK_EN(); }
	} else {
		if (pI2Cx == DRV_I2C1) { DRV_I2C1_PCLK_DI(); }
		else if (pI2Cx == DRV_I2C2) { DRV_I2C2_PCLK_DI(); }
		else if (pI2Cx == DRV_I2C3) { DRV_I2C3_PCLK_DI(); }
	}
}


// Init and Deinit

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;
	uint16_t CCRValue = 0;

	// Enable peripheral clock
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	// Configure FREQ field of CR2
	tempreg = 0;
	tempreg |= (RCC_GetPCLK1Value() / 1000000U);
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	// Program device's own address
	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1);
	tempreg |= 1 << 14;			// To comply with keeping the 14th bit 1 always
	pI2CHandle->pI2Cx->OAR1 = (tempreg);

	// CCR Calculations
	tempreg = 0;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCLSPEED_SM) {
		// Standard mode selected
		CCRValue = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (CCRValue & 0xFFF);
	} else {
		// Fast mode is selected
		tempreg |= (1 << 15);		// Enable Fast Mode
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);		// Configure duty cycle
		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FMDUTY_2) {
			CCRValue = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		} else {
			CCRValue = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (CCRValue & 0xFFF);
	}

	pI2CHandle->pI2Cx->CCR |= tempreg;

	tempreg = 0;

	// Configure TRISE
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCLSPEED_SM) {
		// Standard Mode
		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	} else {
		tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
	}

	pI2CHandle->pI2Cx->TRISE = 0;
	pI2CHandle->pI2Cx->TRISE |= (tempreg & 0x3F);

}

void I2C_Deinit(I2C_RegDef_t *pI2Cx)
{
	if (pI2Cx == DRV_I2C1) { I2C1_REG_RESET(); }
	else if (pI2Cx == DRV_I2C2) { I2C2_REG_RESET(); }
	else if (pI2Cx == DRV_I2C3) { I2C3_REG_RESET(); }
}


// Send & Receive Data
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint8_t len, uint8_t SlaveAddr)
{
	// 1. Generate start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. Confirm that start generation is complete by checking the SB flag in SR1
	while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)));

	// 3. Send address of the slave with r/rw bit set to 0
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr);

//	// 4. Confirm the address phase is complete by checking the ADDR flag in the SR1
//	while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)));
//
//	// 5. Clear the ADDR flag
//	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	// 6. Send the data until len == 0 (after checking if DR is empty by checking TXE flag
	while (len > 0) {
		while (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));	// Wait till TXE is set
		pI2CHandle->pI2Cx->DR = *pTXBuffer;
		pTXBuffer++;
		len--;
	}

	// 7. Wait until TXE == 1 and BTF == 1 before generating the stop condition
	while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)));
	while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF)));

	// 8. Generate the stop condtion
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

// Other peripheral control
void I2C_PeripheralControl(I2C_Handle_t *pI2CHandle, uint8_t EnorDi)
{
	uint32_t tempreg = 0;

	if (EnorDi == ENABLE) { pI2CHandle->pI2Cx->CR1 |= ENABLE_PE; }
	else { pI2CHandle->pI2Cx->CR1 &= DISABLE_PE; }

	// Enable ACKing
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;

	pI2CHandle->pI2Cx->CR1 |= tempreg;
}


// Interrupts
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI)
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


void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// Setup Priority
	NVIC_RegDef_t *pNVIC;
	pNVIC = DRV_NVIC_IPR_BASEADDR;
	uint8_t total_shift = ((IRQNumber % 4) * 8) + (8 - NON_IMPLEMENTED_PR_BITS);
	pNVIC->PRIORITY[IRQNumber / 4] |= (IRQPriority << total_shift);
}

/*
 * In order for us to configure the FREQ field in CR2, we need to know the current
 * value of APB bus speed. We can start off with something, but during multiple
 * other applications, the value of APB Bus speed may change. This function gives
 * the most recent value of APB Bus. Configure it using the clock tree.
 */
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t PClk1;

	uint8_t ClckSrc;
	uint32_t SystemClc;

	uint8_t AHBPrescalerVal;
	uint32_t AHBPrescalerFactor;
	uint16_t AHBPreArr[8] = {2, 4, 8, 16, 64, 128, 256, 512};

	uint8_t APB1PrescalerVal;
	uint32_t APB1PrescalerFactor;
	uint16_t APBPreArr[4] = {2, 4, 8, 16};

	// We are only concerned about bit 2 and 3. We need to bring bit 2 and 3 to
	// position 0 and 1. In order to do that, we have to shift the value in CFGR
	// by 2. Then if we mask it by 0b11, we'll get the first 2 bits.
	ClckSrc = ((DRV_RCC->CFGR >> 2) & 0x3);

	if (ClckSrc == 0) {
		// System clock is HSI
		SystemClc = 16000000;
	} else if (ClckSrc == 1) {
		// System clock is HSE
		SystemClc = 8000000;
	} else if (ClckSrc == 2) {
		// System clocks is PL
		SystemClc = RCC_GetPLLOutputClock();
	}

	// Get the value of AHB prescaler
	AHBPrescalerVal = ((DRV_RCC->CFGR >> 4) & 0xF);
	if (AHBPrescalerVal < 8) {
		// System clock is not divided
		AHBPrescalerFactor = 1;
	} else {
		AHBPrescalerFactor = AHBPreArr[AHBPrescalerVal - 8];
	}

	// Get the value of APB prescaler
	APB1PrescalerVal = ((DRV_RCC->CFGR >> 10) & 0x7);
	if (APB1PrescalerVal < 4) {
		// APB is not divided
		APB1PrescalerFactor = 1;
	} else {
		APB1PrescalerFactor = APBPreArr[APB1PrescalerVal - 4];
	}

	PClk1 = ((SystemClc / AHBPrescalerFactor) / APB1PrescalerFactor);

	return PClk1;
}

uint32_t RCC_GetPLLOutputClock(void)
{
	uint32_t PLLClock = 0;

	return PLLClock;
}

void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if (pI2Cx->SR1 & FlagName) {
		return FLAG_SET;
	}

	return FLAG_RESET;
}

void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;		// Shift by 1 bit for the last bit to hold
									// R/W bit
	SlaveAddr &= ~(1);				// Clear the 0th bit
	pI2Cx->DR = SlaveAddr;
}

void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void) dummyRead;
}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}


// End of file
