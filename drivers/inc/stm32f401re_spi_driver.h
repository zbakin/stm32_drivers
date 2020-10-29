/*
 * stm32f401re_spi_driver.h
 *
 *  Created on: 19 Nov 2019
 *      Author: zhanibeksmac
 */

#ifndef INC_STM32F401RE_SPI_DRIVER_H_
#define INC_STM32F401RE_SPI_DRIVER_H_

#include "stm32f401re.h"

/*
 * Configuration structure for SPIx peripheral
 */

typedef struct
{
	uint8_t SPI_DeviceMode;			// master or slave @SPI_DeviceMode
	uint8_t SPI_BusConfig;			//@SPI_BusConfig
	uint8_t SPI_SclkSpeed;			//@SPI_SclkSpeed
	uint8_t SPI_DFF;				// data frame format @SPI_DFF
	uint8_t SPI_CPOL;				// clock polarity	@SPI_CPOL
	uint8_t SPI_CPHA;				// clock phase		@SPI_CPHA
	uint8_t SPI_SSM;				// @SPI_SSM
}SPI_Config_t;

/*
 * Handle structure for SPIx peripheral
 */
typedef struct
{
	//pointer to hold the base address of the GPIO peripheral
	SPI_RegDef_t 	*pSPIx; 			// Holds the base address of the GPIO port to which the pin belongs
	SPI_Config_t 	SPI_Config;  		// Holds GPIO pin configuration settings
	uint8_t 		*pTxBuffer;			// Stores the application TxBuffer address
	uint8_t 		*pRxBuffer;			// Stores the application RxBuffer address
	uint32_t 		TxLen;				// Stores Tx len
	uint32_t 		RxLen;				// Stores Rx len
	uint8_t 		TxState;			// Stores Tx state
	uint8_t 		RxState;			// Stores Rx state
}SPI_Handle_t;

/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER		1
#define SPI_DEVICE_MODE_SLAVE 		0

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2					0
#define SPI_SCLK_SPEED_DIV4					1
#define SPI_SCLK_SPEED_DIV8					2
#define SPI_SCLK_SPEED_DIV16				3
#define SPI_SCLK_SPEED_DIV32				4
#define SPI_SCLK_SPEED_DIV64				5
#define SPI_SCLK_SPEED_DIV128				6
#define SPI_SCLK_SPEED_DIV256				7

/*
 * @SPI_DFF
 */

#define SPI_DFF_8BITS			0
#define SPI_DFF_16BITS       	1

/*
 * @SPI_CPOL
 */

#define SPI_CPOL_HIGH 			1
#define SPI_CPOL_LOW 			0

/*
 * @SPI_CPHA
 */

#define SPI_CPHA_HIGH 			1
#define SPI_CPHA_LOW 			0


/*
 * @SPI_SMM
 */

#define SPI_SSM_EN				1
#define SPI_SSM_DI				0

/*
 * SPI related status flag definitions
 */
#define SPI_TXE_FLAG		(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG	(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG	(1 << SPI_SR_BSY)

/*
 * SPI Application State
 */

#define SPI_READY					0
#define SPI_BUSY_IN_RX				1
#define SPI_BUSY_IN_TX				2

/*
 * Possible SPI Application events
 */

#define SPI_EVENT_TX_CMPLT			1
#define SPI_EVENT_RX_CMPLT			2
#define SPI_EVENT_OVR_ERR			3
#define SPI_EVENT_CRC_ERR			4

/*********************************************************************************
 * 								APIs supported by this driver
 * 				For more information about the APIs check the function definitions
 ********************************************************************************/

/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);				//Initialise the SPI port
void SPI_DeInit(SPI_RegDef_t *pSPIx);				//De-initialise the SPI port


/*
 * Data Send and Receive - Blocking
 */

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer, uint32_t Len);

/*
 * Data Send and Receive - Non-blocking using Interrupt
 */

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);


/*
 * IRQ Configuration and ISR handling
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName);


#endif /* INC_STM32F401RE_SPI_DRIVER_H_ */
