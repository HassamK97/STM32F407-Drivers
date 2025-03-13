/*
 * stm32f407xx.h
 *
 *  Created on: Feb 28, 2024
 *      Author: Hassam
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */
#define NVIC_ISER0          ((volatile uint32_t*)0xE000E100 )
#define NVIC_ISER1          ((volatile uint32_t*)0xE000E104 )
#define NVIC_ISER2          ((volatile uint32_t*)0xE000E108 )
#define NVIC_ISER3          ((volatile uint32_t*)0xE000E10c )

/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			((volatile uint32_t*)0XE000E180)
#define NVIC_ICER1			((volatile uint32_t*)0XE000E184)
#define NVIC_ICER2  		((volatile uint32_t*)0XE000E188)
#define NVIC_ICER3			((volatile uint32_t*)0XE000E18C)

/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((volatile uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4

/*
 * Common Macros for controlling and configuring drivers
 */
#define ENABLE						1
#define DISABLE						0
#define SET							ENABLE
#define RESET						DISABLE
#define GPIO_PIN_SET				SET
#define GPIO_PIN_RESET				RESET

/*
 * Base Addresses of FLASH and SRAM Memories
 */
#define FLASH_BASEADDR				0x08000000U			/* Base address of FlASH memory known as main memory observed in reference manual */
#define SRAM1_BASEADDR				0x20000000U			/* SRAM1 is of 112 KB and it is accessible by all AHB masters */
#define SRAM2_BASEADDR				0x2001C000U			/* SRAM2 is of 16 KB and it is accessible by all AHB masters */
#define ROM							0x1FFF0000U			/* Base address of ROM known as system memory observed in reference manual */
#define SRAM						SRAM1_BASEADDR		/* SRAM starting address has been set to the SRAM1 base address */

/*
 * AHBx and APBx Bus Peripheral Base Addresses
 * AHBx bus is used for high speed data communication processes
 * APBx bus is used for low speed communication processes
 */
#define PERIPHERAL_BASEADDR			0x40000000U
#define APB1PERIPH_BASEADDR			PERIPHERAL_BASE
#define APB2PERIPH_BASEADDR			0x40010000U
#define AHB1PERIPH_BASEADDR			0x40020000U
#define AHB2PERIPH_BASEADDR			0x50000000U

/*
 * Base Addresses of Peripherals Hanging on AHB1 Bus
 */
#define GPIOA_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR				(AHB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR                (AHB1PERIPH_BASEADDR + 0x3800)

/*
 *  Base Addresses of Peripherals Hanging on APB1 Bus
 */
#define I2C1_BASEADDR				(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR				(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR				(APB1PERIPH_BASEADDR + 0x5C00)
#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR + 0x3C00)
#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR				(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR				(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR				(APB1PERIPH_BASEADDR + 0x5000)

/*
 *  Base Addresses of Peripherals Hanging on APB2 Bus
 */
#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR				(APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR				(APB2PERIPH_BASEADDR + 0x1400)

/*
 *	Peripheral Definitions (Peripheral Base Addresses Type Casted to Relevant Register Structure Definitions)
 */
#define GPIOA		((GPIO_Reg_Def*) GPIOA_BASEADDR)
#define GPIOB		((GPIO_Reg_Def*) GPIOB_BASEADDR)
#define GPIOC		((GPIO_Reg_Def*) GPIOC_BASEADDR)
#define GPIOD		((GPIO_Reg_Def*) GPIOD_BASEADDR)
#define GPIOE		((GPIO_Reg_Def*) GPIOE_BASEADDR)
#define GPIOF		((GPIO_Reg_Def*) GPIOF_BASEADDR)
#define GPIOG		((GPIO_Reg_Def*) GPIOG_BASEADDR)
#define GPIOH		((GPIO_Reg_Def*) GPIOH_BASEADDR)
#define GPIOI		((GPIO_Reg_Def*) GPIOI_BASEADDR)

#define RCC			((RCC_Reg_Def*)	RCC_BASEADDR)
#define EXTI		((EXTI_Reg_Def*) EXTI_BASEADDR)
#define SYSCFG		((SYSCFG_Reg_Def*) SYSCFG_BASEADDR)

/*
 * Macro to return port code for given base address in the range of 0 to 7
 */
#define GPIO_BASEADDR_TO_PORT_CODE(x)   ((x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
								        (x == GPIOE)?4:\
								        (x == GPIOF)?5:\
								        (x == GPIOG)?6:\
								        (x == GPIOH)?7: \
								        (x == GPIOI)?8:0)

/*
 * Clock Enable Macros for GPIOx Peripherals
 */
#define GPIOA_PCLK_EN	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN	(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN	(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN	(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN	(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN	(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN	(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN	(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN	(RCC->AHB1ENR |= (1 << 8))


/*
 * Clock Enable Macros for I2Cx Peripherals
 */
#define I2C1_PCLK_EN	(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN	(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN	(RCC->APB1ENR |= (1 << 23))


/*
 * Clock Enable Macros for SPIx Peripherals
 */
#define SPI1_PCLK_EN	(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN	(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN	(RCC->APB1ENR |= (1 << 15))


/*
 * Clock Enable Macros for USARTx/UARTx Peripherals
 */
#define USART1_PCLK_EN	(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN	(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN	(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN	(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN	(RCC->APB2ENR |= (1 << 5))


/*
 * Clock Enable Macros for SYSCFG Peripheral
 */
#define SYSCFG_PCLK_EN	(RCC->APB2ENR |= (1 << 14))


/*
 * Clock Disable Macros for GPIOx Peripherals
 */
#define GPIOA_PCLK_DI	(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI	(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI	(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI	(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI	(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI	(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI	(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI	(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI	(RCC->AHB1ENR &= ~(1 << 8))


/*
 * Clock Disable Macros for I2Cx Peripherals
 */
#define I2C1_PCLK_DI	(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI	(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI	(RCC->APB1ENR &= ~(1 << 23))


/*
 * Clock Disable Macros for SPIx Peripherals
 */
#define SPI1_PCLK_DI	(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI	(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI	(RCC->APB1ENR &= ~(1 << 15))


/*
 * Clock Disable Macros for USARTx/UARTx Peripherals
 */
#define USART1_PCLK_DI	(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI	(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI	(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI	(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI	(RCC->APB2ENR &= ~(1 << 5))


/*
 * Clock Disable Macros for SYSCFG Peripherals
 */
#define SYSCFG_PCLK_DI	(RCC->APB2ENR &= ~(1 << 14))


/*
 * Clock Disable Macros for GPIOx Peripherals
 */
#define GPIOA_REG_RST	do {(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));} while(0)
#define GPIOB_REG_RST	do {(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));} while(0)
#define GPIOC_REG_RST	do {(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));} while(0)
#define GPIOD_REG_RST	do {(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));} while(0)
#define GPIOE_REG_RST	do {(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));} while(0)
#define GPIOF_REG_RST	do {(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));} while(0)
#define GPIOG_REG_RST	do {(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));} while(0)
#define GPIOH_REG_RST	do {(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));} while(0)
#define GPIOI_REG_RST	do {(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8));} while(0)


/**********************************Peripheral Register Definition Structures **********************************/

/*
 * Note : Registers of a peripheral are specific to MCu. Review the reference manual for information
 * related to registers. For Example: Number of Registers of SPI peripheral of STM32F4x family of MCUs
 * might be different compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 */

typedef struct
{
	volatile uint32_t MODER;		/*!< GPIO port mode register  							Address Offset: 0x00		*/
	volatile uint32_t OTYPER;		/*!< GPIO port output type register 					Address Offset: 0x04		*/
	volatile uint32_t OSPEEDR;		/*!< GPIO port output speed register 					Address Offset: 0x08		*/
	volatile uint32_t PUPDR;		/*!< GPIO port pull-up/pull-down register 				Address Offset: 0x0C		*/
	volatile uint32_t IDR;			/*!< GPIO port input data register 						Address Offset: 0x10		*/
	volatile uint32_t ODR;			/*!< GPIO port output data register 					Address Offset: 0x14		*/
	volatile uint32_t BSRR;			/*!< GPIO port bit set/reset register 					Address Offset: 0x18		*/
	volatile uint32_t LCKR;			/*!< GPIO port configuration lock register 				Address Offset: 0x1C		*/
	volatile uint32_t AFR[2];		/*!< GPIO alternate function low and high registers 	Address Offset: 0x20-0x24	*/
}GPIO_Reg_Def;


typedef struct
{
	volatile uint32_t CR;			/*!< RCC clock control register										Address offset: 0x00 		*/
	volatile uint32_t PLLCFGR;		/*!< RCC PLL configuration register									Address offset: 0x04 		*/
	volatile uint32_t CFGR;			/*!< RCC clock configuration register								Address offset: 0x08 		*/
	volatile uint32_t CIR;			/*!< RCC clock interrupt register									Address offset: 0x0C 		*/
	volatile uint32_t AHB1RSTR;		/*!< RCC AHB1 peripheral reset register								Address offset: 0x10 		*/
	volatile uint32_t AHB2RSTR;		/*!< RCC AHB2 peripheral reset register								Address offset: 0x14 		*/
	volatile uint32_t AHB3RSTR;		/*!< RCC AHB3 peripheral reset register								Address offset: 0x18 		*/
	uint32_t RESERVED1;				/*!< Reserved														Address offset: 0x1C 		*/
	volatile uint32_t APB1RSTR;		/*!< RCC APB1 peripheral reset register								Address offset: 0x20 		*/
	volatile uint32_t APB2RSTR;		/*!< RCC APB2 peripheral reset register								Address offset: 0x24 		*/
	uint32_t RESERVED2[2];			/*!< Reserved														Address offset: 0x28-0x2C 	*/
	volatile uint32_t AHB1ENR;		/*!< RCC AHB1 peripheral clock register								Address offset: 0x30 		*/
	volatile uint32_t AHB2ENR;		/*!< RCC AHB2 peripheral clock register								Address offset: 0x34 		*/
	volatile uint32_t AHB3ENR;		/*!< RCC AHB3 peripheral clock register								Address offset: 0x38 		*/
	uint32_t RESERVED3;				/*!< Reserved														Address offset: 0x3C 		*/
	volatile uint32_t APB1ENR;		/*!< RCC APB1 peripheral clock enable register						Address offset: 0x40 		*/
	volatile uint32_t APB2ENR;		/*!< RCC APB2 peripheral clock enable register						Address offset: 0x44 		*/
	uint32_t RESERVED4[2];			/*!< Reserved														Address offset: 0x48-0x4C 	*/
	volatile uint32_t AHB1LPENR;	/*!< RCC AHB1 peripheral clock enable in low power mode register	Address offset: 0x50 		*/
	volatile uint32_t AHB2LPENR;	/*!< RCC AHB2 peripheral clock enable in low power mode register	Address offset: 0x54 		*/
	volatile uint32_t AHB3LPENR;	/*!< RCC AHB3 peripheral clock enable in low power mode register	Address offset: 0x58 		*/
	uint32_t RESERVED5;				/*!< Reserved														Address offset: 0x5C 		*/
	volatile uint32_t APB1LPENR;	/*!< RCC APB1 peripheral clock enable in low power mode register	Address offset: 0x60 		*/
	volatile uint32_t APB2LPENR;	/*!< RCC APB2 peripheral clock enable in low power mode register	Address offset: 0x64 		*/
	uint32_t RESERVED6[2];			/*!< Reserved														Address offset: 0x68-0x6C 	*/
	volatile uint32_t BDCR;			/*!< RCC Backup domain control register								Address offset: 0x70 		*/
	volatile uint32_t CSR;			/*!< RCC clock control & status register							Address offset: 0x74 		*/
	uint32_t RESERVED7[2];			/*!< Reserved														Address offset: 0x78-0x7C 	*/
	volatile uint32_t SSCGR;		/*!< RCC spread spectrum clock generation register					Address offset: 0x80 		*/
	volatile uint32_t PLLI2SCFGR;	/*!< RCC PLLI2S configuration register								Address offset: 0x84 		*/
	volatile uint32_t PLLSAICFGR;	/*!< RCC PLL configuration register									Address offset: 0x88 		*/
	volatile uint32_t DCKCFGR;		/*!< RCC Dedicated Clock Configuration Register						Address offset: 0x8C 		*/
}RCC_Reg_Def;


typedef struct
{
	volatile uint32_t IMR;		/*!< Interrupt Mask Register,          	  	Address offset: 0x00 */
	volatile uint32_t EMR;		/*!< Event Mask Register,          	  	    Address offset: 0x04 */
	volatile uint32_t RTSR;		/*!< Rising Trigger Selection Register,     Address offset: 0x08 */
	volatile uint32_t FTSR;		/*!< Falling Trigger Selection Register,    Address offset: 0x0C */
	volatile uint32_t SWIER;	/*!< Software Interrupt Event Register,     Address offset: 0x10 */
	volatile uint32_t PR;		/*!< Pending Register,          	  	    Address offset: 0x14 */
}EXTI_Reg_Def;


typedef struct
{
	volatile uint32_t MEMRMP;	 /*!< Memory Remap Register,                      Address offset: 0x00      */
	volatile uint32_t PMC;		 /*!< Peripheral Mode Configuration Register,     Address offset: 0x04      */
	volatile uint32_t EXTICR[4]; /*!< External Interrupt Configuration Register,  Address offset: 0x08-0x14 */
	uint32_t RESERVED1[2];		 /*!< Reserved									  Address offset: 0x18-0x1C */
	volatile uint32_t CMPCR; 	 /*!< Compensation Cell Control Register,         Address offset: 0x20      */
}SYSCFG_Reg_Def;


#endif /* INC_STM32F407XX_H_ */
