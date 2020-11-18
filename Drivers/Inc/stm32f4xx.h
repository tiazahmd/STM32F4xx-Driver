/*
 * stm32f4xx.h
 *
 *  Created on: August 26, 2020
 *      Author: Imtiaz Ahmed
 *
 *  Description: This file contains all MCU
 *  specific details such as base address for
 *  various memories, base address of peripherals,
 *  clock management macro etc. all specific
 *  for STM32F4RE (Nucleo F411RE in particular).
 *
 *  All macros start with DRV suffix to ensure
 *  that the final reader is aware all macro starting
 *  with DRV can be found in this driver file.
 */

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#ifndef INC_STM32F4XX_H_
#define INC_STM32F4XX_H_

/*
 * Create structures to hold all registers for RCC and Peripherals
 */

// RCC register struct
typedef struct {
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t RESERVED_1[2];
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t RESERVED_2[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t RESERVED_3[2];
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t RESERVED_4[2];
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t RESERVED_5[2];
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	volatile uint32_t RESERVED_6[2];
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	volatile uint32_t RESERVED_7[2];
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	volatile uint32_t DCKCFGR;
} RCC_RegDef_t;

typedef struct {
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
} EXTI_RegDef_t;

typedef struct {
	volatile uint32_t SETENA;
	volatile uint32_t CLRENA;
	volatile uint32_t SETPND;
	volatile uint32_t CRLPND;
	volatile uint32_t ACTIVE;
	volatile uint32_t PRIORITY[60];
	volatile uint32_t STIR;
} NVIC_RegDef_t;

typedef struct {
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	volatile uint32_t CMPCR;
} SYSCFG_RegDef_t;

// Peripheral register struct
typedef struct {
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];
} GPIO_RegDef_t;

typedef struct {
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;

} SPI_RegDef_t;

typedef struct {
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t BRR;
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t GTPR;
} USART_RegDef_t;

typedef struct {
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t OAR1;
	volatile uint32_t OAR2;
	volatile uint32_t DR;
	volatile uint32_t SR1;
	volatile uint32_t SR2;
	volatile uint32_t CCR;
	volatile uint32_t TRISE;
	volatile uint32_t FLTR;
} I2C_RegDef_t;

/*
 * Define: Base addresses of Flash and SRAM
 * ROM_ADDR = System Memory
 */
#define DRV_FLASH_BASEADDR					0x08000000UL
#define DRV_SRAM_BASEADDR					0x20000000UL
#define DRV_ROM_BASEADDR					0x1FFF0000UL
#define DRV_SRAM 							DRV_SRAM_BASEADDR

/*
 * Define: BUS domain base addresses
 */

#define DRV_PERIPHERAL_BASEADDR				0x40000000UL
#define DRV_AHB1_BASEADDR					0x40020000UL
#define DRV_AHB2_BASEADDR					0x50000000UL
#define DRV_APB1_BASEADDR					DRV_PERIPHERAL_BASEADDR
#define DRV_APB2_BASEADDR					0x40010000UL

/*
 * Define: Cortex M4 specific base addresses
 */

#define DRV_CORTEX_INTERNAL_PERI_BASEADDR	0xE0000000UL
#define DRV_NVIC_BASE_OFFSET				0xE100UL
#define DRV_NVIC_BASEADDR					(DRV_CORTEX_INTERNAL_PERI_BASEADDR + DRV_NVIC_BASE_OFFSET)
#define DRV_NVIC_ISER_OFFSET				0x000UL
#define DRV_NVIC_ICER_OFFSET				0x080UL
#define DRV_NVIC_ISPR_OFFSET				0x100UL
#define DRV_NVIC_ICPR_OFFSET				0x180UL
#define DRV_NVIC_IABR_OFFSET				0x200UL
#define DRV_NVIC_IPR_OFFSET					0x300UL
#define DRV_NVIC_STIR_OFFSET				0xE00UL

#define DRV_NVIC_ISER_BASEADDR				((NVIC_RegDef_t *) (DRV_NVIC_BASEADDR + DRV_NVIC_ISER_OFFSET))
#define DRV_NVIC_ICER_BASEADDR				((NVIC_RegDef_t *) (DRV_NVIC_BASEADDR + DRV_NVIC_ICER_OFFSET))
#define DRV_NVIC_ISPR_BASEADDR				((NVIC_RegDef_t *) (DRV_NVIC_BASEADDR + DRV_NVIC_ISPR_OFFSET))
#define DRV_NVIC_ICPR_BASEADDR				((NVIC_RegDef_t *) (DRV_NVIC_BASEADDR + DRV_NVIC_ICPR_OFFSET))
#define DRV_NVIC_IABR_BASEADDR				((NVIC_RegDef_t *) (DRV_NVIC_BASEADDR + DRV_NVIC_IABR_OFFSET))
#define DRV_NVIC_IPR_BASEADDR				((NVIC_RegDef_t *) (DRV_NVIC_BASEADDR + DRV_NVIC_IPR_OFFSET))
#define DRV_NVIC_STIR_BASEADDR				((NVIC_RegDef_t *) (DRV_NVIC_BASEADDR + DRV_NVIC_STIR_OFFSET))

#define DRV_NVIC_ISER0_OFFSET				0x00UL
#define DRV_NVIC_ISER1_OFFSET				0x04UL
#define DRV_NVIC_ISER2_OFFSET				0x08UL
#define DRV_NVIC_ISER0_BASEADDR				((NVIC_RegDef_t *) (DRV_NVIC_ISER_BASEADDR + DRV_NVIC_ISER0_OFFSET))
#define DRV_NVIC_ISER1_BASEADDR				((NVIC_RegDef_t *) (DRV_NVIC_ISER_BASEADDR + DRV_NVIC_ISER1_OFFSET))
#define DRV_NVIC_ISER2_BASEADDR				((NVIC_RegDef_t *) (DRV_NVIC_ISER_BASEADDR + DRV_NVIC_ISER2_OFFSET))

#define DRV_NVIC_ICER0_OFFSET				0x00UL
#define DRV_NVIC_ICER1_OFFSET				0x04UL
#define DRV_NVIC_ICER2_OFFSET				0x08UL
#define DRV_NVIC_ICER0_BASEADDR				((NVIC_RegDef_t *) (DRV_NVIC_ICER_BASEADDR + DRV_NVIC_ICER0_OFFSET))
#define DRV_NVIC_ICER1_BASEADDR				((NVIC_RegDef_t *) (DRV_NVIC_ICER_BASEADDR + DRV_NVIC_ICER1_OFFSET))
#define DRV_NVIC_ICER2_BASEADDR				((NVIC_RegDef_t *) (DRV_NVIC_ICER_BASEADDR + DRV_NVIC_ICER2_OFFSET))

#define DRV_NVIC_ISPR0_OFFSET				0x00UL
#define DRV_NVIC_ISPR1_OFFSET				0x04UL
#define DRV_NVIC_ISPR2_OFFSET				0x08UL
#define DRV_NVIC_ISPR0_BASEADDR				((NVIC_RegDef_t *) (DRV_NVIC_ISPR_BASEADDR + DRV_NVIC_ISPR0_OFFSET))
#define DRV_NVIC_ISPR1_BASEADDR				((NVIC_RegDef_t *) (DRV_NVIC_ISPR_BASEADDR + DRV_NVIC_ISPR1_OFFSET))
#define DRV_NVIC_ISPR2_BASEADDR				((NVIC_RegDef_t *) (DRV_NVIC_ISPR_BASEADDR + DRV_NVIC_ISPR2_OFFSET))

#define DRV_NVIC_ICPR0_OFFSET				0x00UL
#define DRV_NVIC_ICPR1_OFFSET				0x04UL
#define DRV_NVIC_ICPR2_OFFSET				0x08UL
#define DRV_NVIC_ICPR0_BASEADDR				((NVIC_RegDef_t *) (DRV_NVIC_ICPR_BASEADDR + DRV_NVIC_ICPR0_OFFSET))
#define DRV_NVIC_ICPR1_BASEADDR				((NVIC_RegDef_t *) (DRV_NVIC_ICPR_BASEADDR + DRV_NVIC_ICPR1_OFFSET))
#define DRV_NVIC_ICPR2_BASEADDR				((NVIC_RegDef_t *) (DRV_NVIC_ICPR_BASEADDR + DRV_NVIC_ICPR2_OFFSET))

#define DRV_NVIC_IABR0_OFFSET				0x00UL
#define DRV_NVIC_IABR1_OFFSET				0x04UL
#define DRV_NVIC_IABR2_OFFSET				0x08UL
#define DRV_NVIC_IABR0_BASEADDR				((NVIC_RegDef_t *) (DRV_NVIC_IABR_BASEADDR + DRV_NVIC_IABR0_OFFSET))
#define DRV_NVIC_IABR1_BASEADDR				((NVIC_RegDef_t *) (DRV_NVIC_IABR_BASEADDR + DRV_NVIC_IABR1_OFFSET))
#define DRV_NVIC_IABR2_BASEADDR				((NVIC_RegDef_t *) (DRV_NVIC_IABR_BASEADDR + DRV_NVIC_IABR2_OFFSET))

// NVIC Priorities
#define NVIC_IRQ_PRIORITY_0					0
#define NVIC_IRQ_PRIORITY_1					1
#define NVIC_IRQ_PRIORITY_2					2
#define NVIC_IRQ_PRIORITY_3					3
#define NVIC_IRQ_PRIORITY_4					4
#define NVIC_IRQ_PRIORITY_5					5
#define NVIC_IRQ_PRIORITY_6					6
#define NVIC_IRQ_PRIORITY_7					7
#define NVIC_IRQ_PRIORITY_8					8
#define NVIC_IRQ_PRIORITY_9					9
#define NVIC_IRQ_PRIORITY_10				10
#define NVIC_IRQ_PRIORITY_11				11
#define NVIC_IRQ_PRIORITY_12				12
#define NVIC_IRQ_PRIORITY_13				13
#define NVIC_IRQ_PRIORITY_14				14
#define NVIC_IRQ_PRIORITY_15				15
#define NVIC_IRQ_PRIORITY_16				16
#define NVIC_IRQ_PRIORITY_17				17
#define NVIC_IRQ_PRIORITY_18				18
#define NVIC_IRQ_PRIORITY_19				19
#define NVIC_IRQ_PRIORITY_20				20
#define NVIC_IRQ_PRIORITY_21				21
#define NVIC_IRQ_PRIORITY_22				22
#define NVIC_IRQ_PRIORITY_23				23
#define NVIC_IRQ_PRIORITY_24				24
#define NVIC_IRQ_PRIORITY_25				25

/*
 * Define: RCC, Base address of AHB1, APB1
 * and APB2 peripherals.
 */

// RCC
#define DRV_RCC_OFFSET						0x3800UL
#define DRV_RCC_BASEADDR					(DRV_AHB1_BASEADDR + DRV_RCC_OFFSET)

// SYSCFG
#define DRV_SYSCFG_OFFSET					0x3800UL
#define DRV_SYSCFG_BASEADDR					(DRV_APB2_BASEADDR + DRV_SYSCFG_OFFSET)

// GPIO
#define DRV_GPIOA_OFFSET					0x0000UL
#define DRV_GPIOB_OFFSET					0x0400UL
#define DRV_GPIOC_OFFSET					0x0800UL
#define DRV_GPIOD_OFFSET					0x0C00UL
#define DRV_GPIOE_OFFSET					0x1000UL
#define DRV_GPIOH_OFFSET					0x1C00UL

#define DRV_GPIOA_BASEADDR					(DRV_AHB1_BASEADDR + DRV_GPIOA_OFFSET)
#define DRV_GPIOB_BASEADDR					(DRV_AHB1_BASEADDR + DRV_GPIOB_OFFSET)
#define DRV_GPIOC_BASEADDR					(DRV_AHB1_BASEADDR + DRV_GPIOC_OFFSET)
#define DRV_GPIOD_BASEADDR					(DRV_AHB1_BASEADDR + DRV_GPIOD_OFFSET)
#define DRV_GPIOE_BASEADDR					(DRV_AHB1_BASEADDR + DRV_GPIOE_OFFSET)
#define DRV_GPIOH_BASEADDR					(DRV_AHB1_BASEADDR + DRV_GPIOH_OFFSET)

// I2C
#define DRV_I2C1_OFFSET						0x5400UL
#define DRV_I2C2_OFFSET						0x5800UL
#define DRV_I2C3_OFFSET						0x5C00UL

#define DRV_I2C1_BASEADDR					(DRV_APB1_BASEADDR + DRV_I2C1_OFFSET)
#define DRV_I2C2_BASEADDR					(DRV_APB1_BASEADDR + DRV_I2C2_OFFSET)
#define DRV_I2C3_BASEADDR					(DRV_APB1_BASEADDR + DRV_I2C3_OFFSET)

// SPI
#define DRV_SPI1_OFFSET						0x3000UL
#define DRV_SPI2_OFFSET						0x3800UL
#define DRV_SPI3_OFFSET						0x3C00UL
#define DRV_SPI4_OFFSET						0x3400UL
#define DRV_SPI5_OFFSET						0x5000UL

#define DRV_SPI1_BASEADDR					(DRV_APB2_BASEADDR + DRV_SPI1_OFFSET)
#define DRV_SPI2_BASEADDR					(DRV_APB1_BASEADDR + DRV_SPI2_OFFSET)
#define DRV_SPI3_BASEADDR					(DRV_APB1_BASEADDR + DRV_SPI3_OFFSET)
#define DRV_SPI4_BASEADDR					(DRV_APB2_BASEADDR + DRV_SPI4_OFFSET)
#define DRV_SPI5_BASEADDR					(DRV_APB2_BASEADDR + DRV_SPI5_OFFSET)

// USART
#define DRV_USART1_OFFSET					0x1000UL
#define DRV_USART2_OFFSET					0x4400UL
#define DRV_USART6_OFFSET					0x1400UL

#define DRV_USART1_BASEADDR					(DRV_APB2_BASEADDR + DRV_USART1_OFFSET)
#define DRV_USART2_BASEADDR					(DRV_APB1_BASEADDR + DRV_USART2_OFFSET)
#define DRV_USART6_BASEADDR					(DRV_APB2_BASEADDR + DRV_USART6_OFFSET)

// EXTI
#define DRV_EXTI_OFFSET						0x3C00UL
#define DRV_EXTI_BASEADDR					(DRV_APB2_BASEADDR + DRV_EXTI_OFFSET)

// SYSCFG
#define DRV_SYSCFG_OFFSET					0x3800UL
#define DRV_SYSCFG_BASEADDR					(DRV_APB2_BASEADDR + DRV_SYSCFG_OFFSET)

/*
 * Connect RCC and Peripherals to their corresponding registers
 */

// RCC
#define DRV_RCC								((RCC_RegDef_t *) DRV_RCC_BASEADDR)

// SYSCFG
#define DRV_SYSCFG							((SYSCFG_RegDef_t *) DRV_SYSCFG_BASEADDR)

// GPIO
#define DRV_GPIOA							((GPIO_RegDef_t *) DRV_GPIOA_BASEADDR)
#define DRV_GPIOB							((GPIO_RegDef_t *) DRV_GPIOB_BASEADDR)
#define DRV_GPIOC							((GPIO_RegDef_t *) DRV_GPIOC_BASEADDR)
#define DRV_GPIOD							((GPIO_RegDef_t *) DRV_GPIOD_BASEADDR)
#define DRV_GPIOE							((GPIO_RegDef_t *) DRV_GPIOE_BASEADDR)
#define DRV_GPIOH							((GPIO_RegDef_t *) DRV_GPIOH_BASEADDR)

// I2C
#define DRV_I2C1							((I2C_RegDef_t *) DRV_I2C1_BASEADDR)
#define DRV_I2C2							((I2C_RegDef_t *) DRV_I2C2_BASEADDR)
#define DRV_I2C3							((I2C_RegDef_t *) DRV_I2C3_BASEADDR)

// SPI
#define DRV_SPI1							((SPI_RegDef_t *) DRV_SPI1_BASEADDR)
#define DRV_SPI2							((SPI_RegDef_t *) DRV_SPI2_BASEADDR)
#define DRV_SPI3							((SPI_RegDef_t *) DRV_SPI3_BASEADDR)
#define DRV_SPI4							((SPI_RegDef_t *) DRV_SPI4_BASEADDR)
#define DRV_SPI5							((SPI_RegDef_t *) DRV_SPI5_BASEADDR)

// USART
#define DRV_USART1							((USART_RegDef_t) * DRV_USART1_BASEADDR)
#define DRV_USART2							((USART_RegDef_t) * DRV_USART2_BASEADDR)
#define DRV_USART6							((USART_RegDef_t) * DRV_USART6_BASEADDR)

// EXTI
#define DRV_EXTI							((EXTI_RegDef_t *) DRV_EXTI_BASEADDR)

// NVIC
#define DRV_NVIC							((NVIC_RegDef_t) *) DRV_NVIC_BASEADDR)

/*
 * Clock Enable Macros for peripherals and others
 */

// GPIOx
#define DRV_GPIOA_PCLK_EN()				(DRV_RCC->AHB1ENR |= (1 << 0))
#define DRV_GPIOB_PCLK_EN()				(DRV_RCC->AHB1ENR |= (1 << 1))
#define DRV_GPIOC_PCLK_EN()				(DRV_RCC->AHB1ENR |= (1 << 2))
#define DRV_GPIOD_PCLK_EN()				(DRV_RCC->AHB1ENR |= (1 << 3))
#define DRV_GPIOE_PCLK_EN()				(DRV_RCC->AHB1ENR |= (1 << 4))
#define DRV_GPIOH_PCLK_EN()				(DRV_RCC->AHB1ENR |= (1 << 7))

// I2Cx
#define DRV_I2C1_PCLK_EN()				(DRV_RCC->APB1ENR |= (1 << 21))
#define DRV_I2C2_PCLK_EN()				(DRV_RCC->APB1ENR |= (1 << 22))
#define DRV_I2C3_PCLK_EN()				(DRV_RCC->APB1ENR |= (1 << 23))

// SPIx
#define DRV_SPI1_PCLK_EN()				(DRV_RCC->APB2ENR |= (1 << 12))
#define DRV_SPI2_PCLK_EN()				(DRV_RCC->APB1ENR |= (1 << 14))
#define DRV_SPI3_PCLK_EN()				(DRV_RCC->APB1ENR |= (1 << 15))
#define DRV_SPI4_PCLK_EN()				(DRV_RCC->APB2ENR |= (1 << 13))
#define DRV_SPI5_PCLK_EN()				(DRV_RCC->APB2ENR |= (1 << 20))

// USARTx
#define DRV_USART1_PCLK_EN()			(DRV_RCC->APB2ENR |= (1 << 4))
#define DRV_USART2_PCLK_EN()			(DRV_RCC->APB1ENR |= (1 << 17))
#define DRV_USART6_PCLK_EN()			(DRV_RCC->APB2ENR |= (1 << 5))

// SYSCFGx
#define DRV_SYSCFG_PCLK_EN()			(DRV_RCC->APB2ENR |= (1 << 14))

/*
 * Clock Disable Macros for peripherals and others
 */

// GPIOx
#define DRV_GPIOA_PCLK_DI()				(DRV_RCC->AHB1ENR &= ~(1 << 0))
#define DRV_GPIOB_PCLK_DI()				(DRV_RCC->AHB1ENR &= ~(1 << 1))
#define DRV_GPIOC_PCLK_DI()				(DRV_RCC->AHB1ENR &= ~(1 << 2))
#define DRV_GPIOD_PCLK_DI()				(DRV_RCC->AHB1ENR &= ~(1 << 3))
#define DRV_GPIOE_PCLK_DI()				(DRV_RCC->AHB1ENR &= ~(1 << 4))
#define DRV_GPIOH_PCLK_DI()				(DRV_RCC->AHB1ENR &= ~(1 << 7))

// I2Cx
#define DRV_I2C1_PCLK_DI()				(DRV_RCC->APB1ENR &= ~(1 << 21))
#define DRV_I2C2_PCLK_DI()				(DRV_RCC->APB1ENR &= ~(1 << 22))
#define DRV_I2C3_PCLK_DI()				(DRV_RCC->APB1ENR &= ~(1 << 23))

// SPIx
#define DRV_SPI1_PCLK_DI()				(DRV_RCC->APB2ENR &= ~(1 << 12))
#define DRV_SPI2_PCLK_DI()				(DRV_RCC->APB1ENR &= ~(1 << 14))
#define DRV_SPI3_PCLK_DI()				(DRV_RCC->APB1ENR &= ~(1 << 15))
#define DRV_SPI4_PCLK_DI()				(DRV_RCC->APB2ENR &= ~(1 << 13))
#define DRV_SPI5_PCLK_DI()				(DRV_RCC->APB2ENR &= ~(1 << 20))

// USARTx
#define DRV_USART1_PCLK_DI()			(DRV_RCC->APB2ENR &= ~(1 << 4))
#define DRV_USART2_PCLK_DI()			(DRV_RCC->APB1ENR &= ~(1 << 17))
#define DRV_USART6_PCLK_DI()			(DRV_RCC->APB2ENR &= ~(1 << 5))

// SYSCFGx
#define DRV_SYSCFG_PCLK_DI()			(DRV_RCC->APB2ENR &= ~(1 << 14))

// Macros to reset the GPIO peripherals
// REG_RESET: Turn the reset bits on first, then turn them back to 0
#define GPIOA_REG_RESET()				do { (DRV_RCC->AHB1RSTR |= (1 << 0)); (DRV_RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET()				do { (DRV_RCC->AHB1RSTR |= (1 << 1)); (DRV_RCC->AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET()				do { (DRV_RCC->AHB1RSTR |= (1 << 2)); (DRV_RCC->AHB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET()				do { (DRV_RCC->AHB1RSTR |= (1 << 3)); (DRV_RCC->AHB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET()				do { (DRV_RCC->AHB1RSTR |= (1 << 4)); (DRV_RCC->AHB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOH_REG_RESET()				do { (DRV_RCC->AHB1RSTR |= (1 << 7)); (DRV_RCC->AHB1RSTR &= ~(1 << 7)); } while(0)

// Macros to reset the SPI peripherals
// REG_RESET: Turn the reset bits on first, then turn them back to 0
#define SPI1_REG_RESET()			    do { (DRV_RCC->APB2RSTR |= (1 << 12)); (DRV_RCC->APB2RSTR &= ~(1 << 12)); } while (0)
#define SPI2_REG_RESET()			    do { (DRV_RCC->APB1RSTR |= (1 << 14)); (DRV_RCC->APB1RSTR &= ~(1 << 14)); } while (0)
#define SPI3_REG_RESET()			    do { (DRV_RCC->APB1RSTR |= (1 << 15)); (DRV_RCC->APB1RSTR &= ~(1 << 15)); } while (0)
#define SPI4_REG_RESET()			    do { (DRV_RCC->APB2RSTR |= (1 << 13)); (DRV_RCC->APB2RSTR &= ~(1 << 13)); } while (0)
#define SPI5_REG_RESET()			    do { (DRV_RCC->APB2RSTR |= (1 << 20)); (DRV_RCC->APB2RSTR &= ~(1 << 20)); } while (0)

// Macros to reset the I2C peripherals
// REG_RESET: Turn the reset bits on first, then turn them back to 0
#define I2C1_REG_RESET()				do { (DRV_RCC->APB1RSTR |= (1 << 21)); (DRV_RCC->APB1RSTR &= ~(1 << 21)); } while (0)
#define I2C2_REG_RESET()				do { (DRV_RCC->APB1RSTR |= (1 << 22)); (DRV_RCC->APB1RSTR &= ~(1 << 22)); } while (0)
#define I2C3_REG_RESET()				do { (DRV_RCC->APB1RSTR |= (1 << 23)); (DRV_RCC->APB1RSTR &= ~(1 << 23)); } while (0)

// EXTI Macros
#define EXTI_PA							((uint8_t) 0)
#define EXTI_PB							((uint8_t) 1)
#define EXTI_PC							((uint8_t) 2)
#define EXTI_PD							((uint8_t) 3)
#define EXTI_PE							((uint8_t) 4)
#define EXTI_PH							((uint8_t) 7)

// IRQ Numbers for EXTI Line
#define EXTI0_IRQ						6
#define EXTI1_IRQ						7
#define EXTI2_IRQ						8
#define EXTI3_IRQ						9
#define EXTI4_IRQ						10
#define EXTI9_5_IRQ						23
#define EXTI15_10_IRQ					40

// Macro function to return port code from port (for EXTI)
#define GPIO_PORT_CODE(x)				( (x == DRV_GPIOA) ? EXTI_PA :\
										  (x == DRV_GPIOB) ? EXTI_PB :\
										  (x == DRV_GPIOC) ? EXTI_PC :\
										  (x == DRV_GPIOD) ? EXTI_PD :\
										  (x == DRV_GPIOE) ? EXTI_PE :\
										  (x == DRV_GPIOH) ? EXTI_PH : 0 )


// IRQ Numbers for SPI
#define SPI1_IRQ						35
#define SPI2_IRQ						36
#define SPI3_IRQ						51
#define SPI4_IRQ						84
#define SPI5_IRQ						85


/*
 * Generic Macros
 */

#define TRUE							1
#define FALSE							0
#define ENABLE 							1
#define DISABLE 						0
#define SET								ENABLE
#define RESET							DISABLE
#define GPIO_PIN_SET					SET
#define GPIO_PIN_RESET					RESET
#define AF_REG_NO						8
#define AF_REG_BITS						4
#define EXTI_REG_BITS					4
#define NON_IMPLEMENTED_PR_BITS			4
#define __weak__						__attribute__((weak))

/*
 * Generic functions
 */
void delay(uint32_t del);

/*
 * Other includes
 */
#include <stm32f4xx_GPIO_Driver.h>
#include <stm32f4xx_SPI_Driver.h>
#include <stm32f4xx_I2C_Driver.h>

#endif /* INC_STM32F4XX_H_ */







/*
 * End of file
 */
