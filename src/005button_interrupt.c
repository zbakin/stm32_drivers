/*
 * 005button_interrupt.c
 *
 *  Created on: 2 Nov 2019
 *      Author: zhanibeksmac
 */

#include <string.h>
#include "stm32f401re.h"

#define HIGH 1
#define LOW 0
#define BTN_PRESSED LOW

void delay(void)
{
	// this will introduce ~200ms delay when system clock is 16MHz
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void)
{
	// create GPIO handler
	GPIO_Handle_t gpioled, button;
	memset(&gpioled,0,sizeof(gpioled)); // this will set every element's memory of this structure to 0
	memset(&button,0,sizeof(button));

	gpioled.pGPIOx = GPIOB;
	gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	gpioled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	button.pGPIOx = GPIOA;
	button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	//enable the clock for GPIOB
	GPIO_PeriClockControl(GPIOB,ENABLE);
	//enable the clock for GPIOA
	GPIO_PeriClockControl(GPIOA,ENABLE);
	// initialise the led pin
	GPIO_Init(&gpioled);
	// initialise the button pin
	GPIO_Init(&button);


	//IRQ Configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

	return 0;
}

void EXTI9_5_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_6);
	GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_9);
}
