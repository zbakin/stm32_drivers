/*
 * stm32f401re_gpio_driver.c
 *
 *  Created on: 18 Oct 2019
 *      Author: zhanibeksmac
 */


#include "stm32f401re_gpio_driver.h"
#include <stdint.h>

/*
 * Peripheral Clock Setup
 */
/******************************************************************
 * @fn						- GPIO_PeriClockControl
 *
 * @brief					- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]				- base address of the GPIO peripheral
 * @param[in]				- ENABLE or DISABLE macros
 * @param[in]				-
 *
 * @return					- none
 *
 * @Note					- none

 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else
		{
			GPIOH_PCLK_DI();
		}
	}
}

/*
 * Init and De-init
 */
/******************************************************************
 * @fn						- GPIO_Init and GPIO_DeInit
 *
 * @brief					- This functions initialis and deinitialise given GPIO port
 *
 * @param[in]				- GPIO handler / base address of the GPIO peripheral
 * @param[in]				-
 * @param[in]				-
 *
 * @return					- none
 *
 * @Note					- none

 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0; // temporary register

	//enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1. configure the mode of the gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// non-interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
		pGPIOHandle->pGPIOx->MODER |= temp; 												// setting
	}
	else
	{
		// interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// 1. Configure the Falling edge selection register(FTSR)
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// 1. Configure the Rising edge selection register(RTSR)
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear the corresponding FTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// 1. Configure both rising and falling selection registers
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// 2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN(); // enable sys clock before using it
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		// 3. Enable the EXTI interrupt delivery using Interrupt Mask Register
		EXTI->IMR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp = 0;
	//2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;
	//3. configure pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;
	//4. configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;
	//5. configure the alt. functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2)); // clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	// Need to reset all the registers of the GPIOx port
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else
	{
		GPIOH_REG_RESET();
	}
}

/*
 * Data read and write
 */
/******************************************************************
 * @fn						- GPIO_ReadFromInputPin
 *
 * @brief					- Read and Write functions
 *
 * @param[in]				- Base address of GPIO port
 * @param[in]				- Pin number
 * @param[in]				-
 *
 * @return					- 0 or 1 for read
 *
 * @Note					- none

 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		//write 1 to pin
		pGPIOx->ODR |= (0x1 << PinNumber);
	}
	else
	{
		//write 0 to pin
		pGPIOx->ODR &= ~(0x1 << PinNumber);
	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^=  (1 << PinNumber); // bitwise XOR operation toggles the pin
}

/*
 * IRQ configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
	//Manage the interrupt configuration
	if(EnOrDi == ENABLE)
	{
		if(IRQNumber < 32)
		{
			// program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );
		}
		else if(IRQNumber >= 32 && IRQNumber < 64)
		{
			// program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber <= 64 && IRQNumber < 96)
		{
			// program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 32));
		}
	}
	else
	{
		if(IRQNumber < 32)
		{
			// program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}
		else if(IRQNumber >= 32 && IRQNumber < 64)
		{
			// program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber <= 64 && IRQNumber < 96)
		{
			// program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 32));
		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// find out ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (iprx_section * 8) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	//IRQ handling function to process the interrupt
	// clear the exti pr register corresponding to the pin number

	if(EXTI->PR & (1 << PinNumber))
	{
		// clear pending register bit
		EXTI->PR |= (1 << PinNumber);
	}
}
