/*
 * 002ledtogglebutton.c
 *
 *  Created on: 27 Oct 2019
 *      Author: zhanibeksmac
 */


#include "stm32f401re.h"

#define HIGH 1
#define LOW 0
#define BTN_PRESSED LOW

void delay(void)
{
	for(uint32_t i = 0; i < 350000; i++);
}

int main(void)
{
	// create GPIO handler
	GPIO_Handle_t gpioled, button;

	gpioled.pGPIOx = GPIOB;
	gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	gpioled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	button.pGPIOx = GPIOB;
	button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;

	//enable the clock for GPIOB
	GPIO_PeriClockControl(GPIOB,ENABLE);
	// initialise the led pin
	GPIO_Init(&gpioled);
	// initialise the button pin
	GPIO_Init(&button);

	while(1){

		if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_8) == BTN_PRESSED) // pin is always pulled up, however when the button is pressed the pin is 0(reset)
		{
			// delay for debouncing
			delay();
			// toggle the pin
			GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_9);
		}
	}
	return 0;
}
