/*
 * 006spi_tx_testing.c
 *
 *  Created on: 5 Dec 2019
 *      Author: zhanibeksmac
 */

#include "stm32f401re.h"
#include <string.h>

void SPI2_GPIOInits(void)
{
	// first need to configure GPIO pins to works as SPI pins
	// PB15 --> SPI2_MOSI - alternate functionality AF05
	// PB14 --> SPI2_MISO
	// PB13 --> SPI2_SCK
	// PB12 --> SPI2_NSS

	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//SCKL
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);

	//NSS
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
//	GPIO_Init(&SPIPins);

}


void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_EN; // software slave mng enabled

	SPI_Init(&SPI2Handle);

}

int main()
{
	//this function is used to initialise the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	SPI2_Inits();

//	//configurations and handling settings
//	SPI_Config_t spiconfig;
//	SPI_Handle_t spihandle;
//
//	spiconfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
//	spiconfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
//	spiconfig.SPI_DFF = SPI_DFF_8BITS;
//	//spiconfig.SPI_DFF = SPI_DFF_16BITS;
//
//	spihandle.pSPIx = SPI2;
//	spihandle.SPI_Config = spiconfig;
//
//	char message[15];	strcpy(message,"Hello world!\n");
//
//	SPI2_PCLK_EN();
//	SPI_Init(spihandle);

	char message[] = "Hello world!\n";

	SPI_SendData(SPI2,(uint8_t*)message,strlen(message));

	while(1);

	return 0;
}
