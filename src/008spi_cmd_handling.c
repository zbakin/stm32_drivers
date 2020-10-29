/*
 * 007spi_txonly_arduino.c
 *
 *  Created on: 15 Oct 2020
 *      Author: zhanibeksmac
 */


#include <string.h>
#include <stdio.h>
#include "stm32f401re.h"

extern void initialise_monitor_handles();

#define COMMAND_LED_CTRL				0x50
#define COMMAND_SENSOR_READ			0x51
#define COMMAND_LED_READ				0x52
#define COMMAND_PRINT				0x53
#define COMMAND_ID_READ				0x54

#define LED_ON						1
#define LED_OFF						0

// Arduino analog pins
#define ANALOG_PIN0					0
#define ANALOG_PIN1					1
#define ANALOG_PIN2					2
#define ANALOG_PIN3					3
#define ANALOG_PIN4					4

#define LED_PIN						9

void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 --> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode : 5
 */

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);


	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);


}

void SPI2_Inits(void)
{

	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;//generates sclk of 2MHz
	SPI2handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPI_Config.SPI_SSM = SPI_SSM_DI; //Hardware slave management enabled for NSS pin

	SPI_Init(&SPI2handle);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);

}

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
	if(ackbyte == 0xF5)
	{
		//ack
		return 1;
	}
	return 0;
}

int main(void)
{
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

	initialise_monitor_handles();

	printf("Application is running\n");

	GPIO_ButtonInit();

	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	//This function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	printf("SPI Init. done\n");

	/*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1 , NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
	SPI_SSOEConfig(SPI2,ENABLE);

	while(1)
	{
		//wait till button is pressed
		while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		//to avoid button debouncing related issues 200ms of delay
		delay();

		//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		// 1. Send CMD_LED_CTRL
		uint8_t cmdcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		//send command code
		SPI_SendData(SPI2, &cmdcode, 1);
		// dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// send some dummy bits (1byte) to fetch response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);
		// read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if (SPI_VerifyResponse(ackbyte))
		{
			args[0] = LED_PIN;
			args[1] = LED_ON;
			//send arguments
			SPI_SendData(SPI2, args, 2);
		}
		//end of 1. COMMAND_LED_CTRL

		// 2. CMD_SENSOR_READ <analog pin number(1) >

		//wait till button is pressed
		while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		delay();

		cmdcode = COMMAND_SENSOR_READ;
		//send command code
		SPI_SendData(SPI2, &cmdcode, 1);
		// dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);
		// send some dummy bits (1byte) to fetch response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);
		// read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if (SPI_VerifyResponse(ackbyte))
		{
			args[0] = ANALOG_PIN0;
			//send arguments
			SPI_SendData(SPI2, args, 1);
			// dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);
			// insert delay so that slave is ready to send ADC value
			delay();
			// send some dummy bits (1byte) to fetch response from the slave
			SPI_SendData(SPI2, &dummy_write, 1);
			uint8_t analog_read;
			SPI_ReceiveData(SPI2, &analog_read, 1);
			printf("COMMAND_SENSOR_READ : %d\n", analog_read);
		}

		// 3. CMD_LED_READ

		//wait until button is pressed
		while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		delay();

		cmdcode = COMMAND_LED_READ;
		//send command code
		SPI_SendData(SPI2, &cmdcode, 1);
		// dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);
		// send some dummy bits (1byte) to fetch response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);
		// read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if (SPI_VerifyResponse(ackbyte))
		{
			args[0] = LED_PIN;

			SPI_SendData(SPI2, args, 1);
			SPI_ReceiveData(SPI2, &dummy_read, 1);
			delay();
			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t led_status;
			SPI_ReceiveData(SPI2, &led_status, 1);
			printf("COMMAND_READ_LED %d\n", led_status);
		}


		// 4. COMMAND_PRINT

		//wait until button is pressed
		while (GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13));
		delay();

		cmdcode = COMMAND_PRINT;
		//send command code
		SPI_SendData(SPI2, &cmdcode, 1);
		// dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);
		// send some dummy bits (1byte) to fetch response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);
		// read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);
		uint8_t message[] = "Hello ! How are you ??";
		if (SPI_VerifyResponse(ackbyte))
		{
			args[0] = strlen((char*)message);
			//send arguments
			SPI_SendData(SPI2,args,1); //sending length
			//send message
			SPI_SendData(SPI2,message,args[0]);
			printf("COMMAND_PRINT Executed \n");
		}

		// 5. COMMAND_ID_READ

		//wait until button is pressed
		while (GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13));
		delay();

		cmdcode = COMMAND_ID_READ;
		//send command code
		SPI_SendData(SPI2, &cmdcode, 1);
		// dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);
		// send some dummy bits (1byte) to fetch response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);
		// read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);
		uint8_t id[11];
		uint32_t i = 0;
		if (SPI_VerifyResponse(ackbyte))
		{
			//read 10 bytes id from the slave
			for(i = 0 ; i < 10 ; i++)
			{
				//send dummy byte to fetch data from slave
				SPI_SendData(SPI2,&dummy_write,1);
				SPI_ReceiveData(SPI2, &id[i], 1);
			}
			id[11] = '\0';
			printf("COMMAND_ID : %s\n",id);
		}

		// confirm SPI is not busy
		while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );

		// Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,DISABLE);

		printf("SPI Communication Closed\n");
	}

	return 0;

}
