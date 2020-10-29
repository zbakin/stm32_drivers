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

	gpioled.pGPIOx = GPIOA; // port A is for LED
	gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT; // set as output
	gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5; // pin number 5 is LED pin
	gpioled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; // push pull
	gpioled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; // pull up

	button.pGPIOx = GPIOC; // PC13 is for button
	button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;

	//enable the clock for GPIOA
	GPIO_PeriClockControl(GPIOA,ENABLE);
	//enable the clock for GPIOC
	GPIO_PeriClockControl(GPIOC,ENABLE);
	// initialise the GPIOA
	GPIO_Init(&gpioled);
	// initialise the GPIOC
	GPIO_Init(&button);

	// loop
	while(1){

		if(GPIO_ReadFromInputPin(GPIOC,13) == BTN_PRESSED) // pin is always pulled up, however when the button is pressed the pin is 0(reset)
		{
			// delay for debouncing
			delay();
			// toggle the pin
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		}
	}
	return 0;
}
