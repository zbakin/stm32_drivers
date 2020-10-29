/*
 * stm32f401re_gpio_driver.h
 *
 *  Created on: 18 Oct 2019
 *      Author: zhanibeksmac
 */

#ifndef INC_STM32F401RE_GPIO_DRIVER_H_
#define INC_STM32F401RE_GPIO_DRIVER_H_

#include "stm32f401re.h"

/*
 * This is a Configuration structure for a GPIO pin
 */

typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;				// Possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;				// Possible values from @GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdControl;			// Possible values from @GPIO_PIN_PUPD
	uint8_t GPIO_PinOPType;				// Possible values from @GPIO_PIN_OUTTYPE
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/*
 * This is a Handle structure for a GPIO pin
 */

typedef struct
{
	//pointer to hold the base address of the GPIO peripheral
	GPIO_RegDef_t *pGPIOx; 				// Holds the base address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;  	// Holds GPIO pin configuration settings
}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBDER
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15


/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 		2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT 		4			//FT - Falling edge Trigger
#define GPIO_MODE_IT_RT 		5			//RT - Rising edge Trigger
#define GPIO_MODE_IT_RFT		6			//RFT - Rising Falling edge Trigger


/*
 * @GPIO_PIN_OUTTYPE
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP		0			// output type - push pull
#define GPIO_OP_TYPE_OD		1			// output type - open drain


/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_HIGH		2
#define GPIO_SPEED_VERYHIGH	3

/*
 * @GPIO_PIN_PUPD
 * GPIO pin pull up pull down configuration options
 */
#define GPIO_NO_PUPD 		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2


/******************************APIs supported by this driver*******************************/
/**********************************Function prototypes*************************************/
/*
 * Peripheral Clock Setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);				//Initialise the GPIO port
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);					//De-initialise the GPIO port

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);		//Manage the interrupt configuration
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);											//IRQ handling function to process the interrupt


#endif /* INC_STM32F401RE_GPIO_DRIVER_H_ */
