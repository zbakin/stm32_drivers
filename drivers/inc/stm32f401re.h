/*
 * stm32f401re.h
 *
 *  Created on: 18 Oct 2019
 *      Author: zhanibeksmac
 */

#ifndef INC_STM32F401RE_H_
#define INC_STM32F401RE_H_

#include <stdint.h>
#include <stddef.h>

#define __vo volatile

/***************************************START:Processor specific details*********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register addresses
 */
#define NVIC_ISER0				((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1				((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2				((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3				((__vo uint32_t*)0xE000E10C)

/*
 * ARM Cortex Mx Processor NVIC ICERx register addresses
 */
#define NVIC_ICER0				((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1				((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2				((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3				((__vo uint32_t*)0xE000E18C)

/*
 * ARM Contex mX Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASEADDR		((__vo uint32_t*)0xE000E400)


#define NO_PR_BITS_IMPLEMENTED				4


/*
 * base addresses of Flash and SRAM memories
 */
// all this information can be found in reference manual
#define FLASH_BASEADDR			0x08000000U 		// Flash memory - main memory; U - unsigned number
#define SRAM1_BASEADDR			0x20000000U			// SRAM - static random access memory
#define ROM						0x1FFF0000U			// ROM - read-only memory - system memory
#define SRAM 					SRAM1_BASEADDR 		// main SRAM is actually SRAM1


 /*
  * AHBx and APBx Bus Peripheral base addresses
  */

#define PERIPH_BASEADDR				0x40000000U
#define APB1PERIPH_BASEADDR			PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR			0x40010000U
#define AHB1PERIPH_BASEADDR 		0x40020000U
#define AHB2PERIPH_BASEADDR			0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOH_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1C00)
#define RCC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3800)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR + 0x3C00)
#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0x4400)
#define I2C1_BASEADDR				(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR				(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR				(APB1PERIPH_BASEADDR + 0x5C00)


/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR				(APB2PERIPH_BASEADDR + 0x1400)
#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR				(APB2PERIPH_BASEADDR + 0x3400)
#define SYSCFG_BASEADDR				(APB2PERIPH_BASEADDR + 0x3800)
#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0x3C00)


/***********************************Peripheral register definition structures***********************/

/*
 * Peripheral register definition structure for GPIO
 */

typedef struct{
	__vo uint32_t MODER;			//port mode register  					     // address offset 0x00
	__vo uint32_t OTYPER;			//port output type register			    	 // address offset 0x04
	__vo uint32_t OSPEEDR;			//port output speed register
	__vo uint32_t PUPDR;			//port pull-up/pull-down register
	__vo uint32_t IDR;				//port input data register
	__vo uint32_t ODR;				//port output data register
	__vo uint32_t BSRR;				//port bit set/reset register
	__vo uint32_t LCKR;				//port configuration lock register
	__vo uint32_t AFR[2];			//AFR[0] : alternate function low register 	// address offset 0x20 - 0x24
									//AFR[1] : GPIO alternate function high register
}GPIO_RegDef_t;

/*
 * Peripheral register definition structure for RCC
 */

typedef struct{
	__vo uint32_t CR;				//clock control register							//offset 0x00
	__vo uint32_t PLLCFGR;			//PLL configuration register						//offset 0x04
	__vo uint32_t CFGR;				//clock configuration register					//0x08
	__vo uint32_t CIR;				//clock interrupt register						//0x0C
	__vo uint32_t AHB1RSTR;			//AHB1 peripheral reset register					//0x10
	__vo uint32_t AHB2RSTR;			//AHB2 peripheral reset register					//0x14
	uint32_t RESERVED[2];															//0x18-0x1C
	__vo uint32_t APB1RSTR;			//APB1 peripheral reset register					//0x20
	__vo uint32_t APB2RSTR;			//APB2 peripheral reset register					//0x24
	uint32_t RESERVED2[2];															//0x28-0x2C
	__vo uint32_t AHB1ENR;			//AHB1 peripheral clock enable register			//0x30
	__vo uint32_t AHB2ENR;			//AHB2 peripheral clock enable register			//0x34
	uint32_t RESERVED3[2];															//0x38-0x3C
	__vo uint32_t APB1ENR;															//0x40
	__vo uint32_t APB2ENR;															//0x44
	uint32_t RESERVED4[2];															//0x48-0x4C
	__vo uint32_t AHB1LPENR;															//0x50
	__vo uint32_t AHB2LPENR;															//0x54
	uint32_t RESERVED5[2];															//0x58-0x5C
	__vo uint32_t APB1LPENR;															//0x60
	__vo uint32_t APB2LPENR;															//0x64
	uint32_t RESERVED6[2];															//0x68-0x6C
	__vo uint32_t BDCR;																//0x70
	__vo uint32_t CSR;																//0x74
	uint32_t RESERVED7[2];															//0x78-0x7C
	__vo uint32_t SSCGR;																//0x80
	__vo uint32_t PLLI2SCFGR;														//0x84
	uint32_t RESERVED8;																//0x88
	__vo uint32_t DCKCFGR;															//0x8C

}RCC_RegDef_t;

/*
 * Peripheral register definition structure for EXTI
 */


typedef struct{
	__vo uint32_t IMR;			// Interrupt Mask Register		    		// address offset 0x00
	__vo uint32_t EMR;			// Event Mask Register						// offset 0x04
	__vo uint32_t RTSR;			// Rising Trigger Selection Register		// 0x08
	__vo uint32_t FTSR;			// Falling Trigger Selection Register		// 0x0C
	__vo uint32_t SWIER;		// Software interrupt event register		// 0x10
	__vo uint32_t PR;			// Pending register							// 0x14
}EXTI_RegDef_t;

/*
 * Peripheral register definition structure for SYSCFG
 */
typedef struct{
	__vo uint32_t MEMRMP;		// memory remap register	    					// 0x00
	__vo uint32_t PMC;			// peripheral mode configuration register			// 0x04
	__vo uint32_t EXTICR[4];	// external interrupt configuration register		// 0x08-0x14
	__vo uint32_t RESERVED[2];	// reserved, not used								// 0x18-0x1C
	__vo uint32_t CMPCR;		// compensation cell control register				// 0x20
}SYSCFG_RegDef_t;

/*
 * Peripheral register definition structure for SPI
 */

typedef struct{
	__vo uint32_t CR1;		//control register 1	    	// 0x00
	__vo uint32_t CR2;		//control register 2			// 0x04
	__vo uint32_t SR;		//status register				// 0x08
	__vo uint32_t DR;		//data register					// 0x0C
	__vo uint32_t CRCPR;	//CRC polynomial register		// 0x10
	__vo uint32_t RXCRCR;	//RX CRC register				// 0x14
	__vo uint32_t TXCRCR;	//TX CRC register				// 0x18
	__vo uint32_t I2SCFGR;  //SPI_I2S configuration register	//0x1C
	__vo uint32_t I2SPR;	//SPI_I2S prescaler register		//0x20
}SPI_RegDef_t;


/*
 * peripheral definitions - peripheral base addresses typecasted to xxx_RegDef_t
 */


#define GPIOA		((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOH		((GPIO_RegDef_t*) GPIOH_BASEADDR)

#define RCC			((RCC_RegDef_t*) RCC_BASEADDR)

#define EXTI		((EXTI_RegDef_t*) EXTI_BASEADDR)

#define SYSCFG		((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

#define SPI1		((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3		((SPI_RegDef_t*) SPI3_BASEADDR)
#define SPI4		((SPI_RegDef_t*) SPI4_BASEADDR)

/*
 * Clock Enable Macros for GPIOx Peripherals
 */

#define GPIOA_PCLK_EN()	RCC->AHB1ENR |= (1 << 0)        //GPIOA enable register is at bit 0 of AHB1ENR
#define GPIOB_PCLK_EN()	RCC->AHB1ENR |= (1 << 1)
#define GPIOC_PCLK_EN()	RCC->AHB1ENR |= (1 << 2)
#define GPIOD_PCLK_EN()	RCC->AHB1ENR |= (1 << 3)
#define GPIOE_PCLK_EN()	RCC->AHB1ENR |= (1 << 4)
#define GPIOH_PCLK_EN()	RCC->AHB1ENR |= (1 << 7)


/*
 * Clock Enable Macros for I2Cx Peripherals
 */

#define I2C1_PCLK_EN() RCC->APB1ENR |= (1 << 21)
#define I2C2_PCLK_EN() RCC->APB1ENR |= (1 << 22)
#define I2C3_PCLK_EN() RCC->APB1ENR |= (1 << 23)

/*
 * Clock Enable Macros for SPIx Peripherals
 */

#define SPI1_PCLK_EN() RCC->APB2ENR |= (1 << 12)
#define SPI2_PCLK_EN() RCC->APB1ENR |= (1 << 14)
#define SPI3_PCLK_EN() RCC->APB1ENR |= (1 << 15)
#define SPI4_PCLK_EN() RCC->APB2ENR |= (1 << 13)

/*
 * Clock Enable Macros for USARTx Peripherals
 */
#define USART1_PCLK_EN() RCC->APB2ENR |= (1 << 4)
#define USART2_PCLK_EN() RCC->APB1ENR |= (1 << 17)
#define USART6_PCLK_EN() RCC->APB2ENR |= (1 << 5)

/*
 * Clock Enable Macros for SYSCFG Peripheral
 */

#define SYSCFG_PCLK_EN() RCC->APB2ENR |= (1 << 14)

/*
 * Clock Disable Macros for GPIOx Peripherals
 */

#define GPIOA_PCLK_DI()	RCC->AHB1ENR &= ~(1 << 0)        //GPIOA enable register is at bit 0 of AHB1ENR
#define GPIOB_PCLK_DI()	RCC->AHB1ENR &= ~(1 << 1)
#define GPIOC_PCLK_DI()	RCC->AHB1ENR &= ~(1 << 2)
#define GPIOD_PCLK_DI()	RCC->AHB1ENR &= ~(1 << 3)
#define GPIOE_PCLK_DI()	RCC->AHB1ENR &= ~(1 << 4)
#define GPIOH_PCLK_DI()	RCC->AHB1ENR &= ~(1 << 7)


/*
 * Clock Disable Macros for I2Cx Peripherals
 */

#define I2C1_PCLK_DI() RCC->APB1ENR &= ~(1 << 21)
#define I2C2_PCLK_DI() RCC->APB1ENR &= ~(1 << 22)
#define I2C3_PCLK_DI() RCC->APB1ENR &= ~(1 << 23)

/*
 * Clock Disable Macros for SPIx Peripherals
 */

#define SPI1_PCLK_DI() RCC->APB2ENR &= ~(1 << 12)
#define SPI2_PCLK_DI() RCC->APB1ENR &= ~(1 << 14)
#define SPI3_PCLK_DI() RCC->APB1ENR &= ~(1 << 15)
#define SPI4_PCLK_DI() RCC->APB2ENR &= ~(1 << 13)

/*
 * Clock Disable Macros for USARTx Peripherals
 */

#define USART1_PCLK_DI() RCC->APB2ENR &= ~(1 << 4)
#define USART2_PCLK_DI() RCC->APB1ENR &= ~(1 << 17)
#define USART6_PCLK_DI() RCC->APB2ENR &= ~(1 << 5)

/*
 * Clock Disable Macros for SYSCFG Peripheral
 */

#define SYSCFG_PCLK_DI() RCC->APB2ENR &= ~(1 << 14)


/*
 *	Macros to reset the GPIOx peripheral
 */
#define GPIOA_REG_RESET() 	do{RCC->AHB1RSTR |= (1 << 0);/*first set to 1*/ RCC->AHB1RSTR &= ~(1 << 0);/*then set to 0*/}while(0)
#define GPIOB_REG_RESET() 	do{RCC->AHB1RSTR |= (1 << 1);/*first set to 1*/ RCC->AHB1RSTR &= ~(1 << 1);/*then set to 0*/}while(0)
#define GPIOC_REG_RESET() 	do{RCC->AHB1RSTR |= (1 << 2);/*first set to 1*/ RCC->AHB1RSTR &= ~(1 << 2);/*then set to 0*/}while(0)
#define GPIOD_REG_RESET() 	do{RCC->AHB1RSTR |= (1 << 3);/*first set to 1*/ RCC->AHB1RSTR &= ~(1 << 3);/*then set to 0*/}while(0)
#define GPIOE_REG_RESET() 	do{RCC->AHB1RSTR |= (1 << 4);/*first set to 1*/ RCC->AHB1RSTR &= ~(1 << 4);/*then set to 0*/}while(0)
#define GPIOH_REG_RESET() 	do{RCC->AHB1RSTR |= (1 << 5);/*first set to 1*/ RCC->AHB1RSTR &= ~(1 << 5);/*then set to 0*/}while(0)

/*
 * Macros to reset the SPI peripheral
 */
#define SPI1_REG_RESET()	do{RCC->APB2RSTR |= (1 << 12); RCC->APB2RSTR &= ~(1 << 12);}while(0);
#define SPI2_REG_RESET()	do{RCC->APB1RSTR |= (1 << 14); RCC->APB1RSTR &= ~(1 << 14);}while(0);
#define SPI3_REG_RESET()	do{RCC->APB1RSTR |= (1 << 15); RCC->APB1RSTR &= ~(1 << 15);}while(0);
#define SPI4_REG_RESET()	do{RCC->APB2RSTR |= (1 << 13); RCC->APB2RSTR &= ~(1 << 13);}while(0);


/*
 *  returns port code for given GPIOx base address
 */

#define GPIO_BASEADDR_TO_CODE(x) 		((x == GPIOA) ? 0:\
										 (x == GPIOB) ? 1:\
										 (x == GPIOC) ? 2:\
										 (x == GPIOD) ? 3:\
										 (x == GPIOE) ? 4:\
										 (x == GPIOH) ? 5:0)

/*
 * IRQ(Interrupt Request) Number of STM32F401RE MCU
 */

#define IRQ_NO_EXTI0					6
#define IRQ_NO_EXTI1					7
#define IRQ_NO_EXTI2					8
#define IRQ_NO_EXTI3					9
#define IRQ_NO_EXTI4					10
#define IRQ_NO_EXTI9_5				 	23
#define IRQ_NO_EXTI15_10				40

/*
 * IRQ Numbers for SPI
 */
#define IRQ_SPI1						35
#define IRQ_SPI2						36
#define IRQ_SPI3						51
#define IRQ_SPI4						84

/*
 * macros of all possible priority levels
 */
#define NVIC_IRQ_PRI0				0
#define NVIC_IRQ_PRI1				1
#define NVIC_IRQ_PRI2				2
#define NVIC_IRQ_PRI3				3
#define NVIC_IRQ_PRI4				4
#define NVIC_IRQ_PRI5				5
#define NVIC_IRQ_PRI6				6
#define NVIC_IRQ_PRI7				7
#define NVIC_IRQ_PRI8				8
#define NVIC_IRQ_PRI9				9
#define NVIC_IRQ_PRI10				10
#define NVIC_IRQ_PRI11				11
#define NVIC_IRQ_PRI12				12
#define NVIC_IRQ_PRI13				13
#define NVIC_IRQ_PRI14				14
#define NVIC_IRQ_PRI15				15


/*
 * Generic Macros
 */

#define ENABLE 					1
#define DISABLE 					0
#define SET						ENABLE
#define RESET					DISABLE
#define GPIO_PIN_SET				SET
#define GPIO_PIN_RESER			RESET
#define FLAG_RESET				RESET
#define FLAG_SET					SET


/*********************************************************************************
 * Bit position definitions of SPI peripheral
 *********************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRC_NEXT		12
#define SPI_CR1_CRC_EN			13
#define SPI_CR1_BIDI_OE			14
#define SPI_CR1_BIDI_MODE		15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7

/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8



#include "stm32f401re_gpio_driver.h"
#include "stm32f401re_spi_driver.h"

#endif /* INC_STM32F401RE_H_ */
