/*
 * 001ledtoggle.c
 *
 *  Created on: 27 Oct 2019
 *      Author: zhanibeksmac
 */


#include "stm32f401re.h"

void delay(void)
{
	for(uint32_t i = 0; i < 500000; i++);
}

int main(void)
{
	// create GPIO handler
	GPIO_Handle_t gpioled;
	gpioled.pGPIOx = GPIOA; // port A is for LED
	gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT; // set as output
	gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5; // pin number 5 is LED pin
	gpioled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD; // push pull
	gpioled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; // pull up

	//enable the clock for GPIOA
	GPIO_PeriClockControl(GPIOA,ENABLE);
	// initialise the GPIOA
	GPIO_Init(&gpioled);

	// loop
	while(1){
		// toggle the pin
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		// delay
		delay();
	}

	return 0;
}
