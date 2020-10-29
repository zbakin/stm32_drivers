/*
 * stm32f401re_spi_driver.c
 *
 *  Created on: 19 Nov 2019
 *      Author: zhanibeksmac
 */

#include "stm32f401re_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);

/*
 * Peripheral Clock setup
 */
/******************************************************************
 * @fn						- SPI_PeriClockControl
 *
 * @brief					- This function enables or disables peripheral clock for the given SPI peripheral
 *
 * @param[in]				- base address of the SPI peripheral
 * @param[in]				- ENABLE or DISABLE macros
 * @param[in]				-
 *
 * @return					- none
 *
 * @Note						- none

 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
	}
}

/*
 * Init and De-init
 */
/******************************************************************
 * @fn						- SPI_Init and SPI_DeInit
 *
 * @brief					- This functions initialises and deinitialises given SPI peripheral
 *
 * @param[in]				- SPI handler / base address of the SPI peripheral
 * @param[in]				-
 * @param[in]				-
 *
 * @return					- none
 *
 * @Note						- none

 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	// configure SPI_CR1 register
	uint32_t tempreg = 0;

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// 1. configure device mode
	tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;

	// 2. configure bus config
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDI_MODE);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= (1 << SPI_CR1_BIDI_MODE);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//bidi should be cleared and RXONLY bit must be set
		tempreg &= ~(1 << SPI_CR1_BIDI_MODE);
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	// 3. configure spi serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR;

	// 4. configure the DFF
	tempreg |= pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF;

	// 5. configure the CPOL
	tempreg |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;

	// 6. configure the CPHA
	tempreg |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempreg;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	//reset all registers of SPI peripheral
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
	else if(pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


/*
 * Data Send and Receive
 */
/******************************************************************
 * @fn						- SPI_SendData and SPI_ReceiveData
 *
 * @brief					- This functions are to send and receive data to and from SPI peripheral
 *
 * @param[in]				- Base address of the SPI peripheral
 * @param[in]				- 8 bit data to send or receive
 * @param[in]				- 32 bit length of the data
 *
 * @return					- none
 *
 * @Note						- This is a blocking call

 */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len)
{
	//check if the Length is not 0
	while(Len > 0)
	{
		// wait until status of TXE is empty - transmit empty register
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG)==FLAG_RESET);

		// check the DFF bit
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF)) // 16-bit data frame format
		{
			pSPIx->DR = *((uint16_t*)pTxBuffer); // typecasting to 16 bit
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}
		else 								// 8-bit data frame format
		{
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer, uint32_t Len)
{
	//check if the Length is not 0
	while(Len > 0)
	{
		// wait until RXNE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		// check the DFF bit
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF)) // 16-bit data frame format
		{
			// load the data from DR to Rxbuffer address
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		}
		else 								// 8-bit data frame format
		{
			*(pRxBuffer) = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}


/*
 * Data Send and Receive - Non-blocking using Interrupt
 */
/******************************************************************
 * @fn						- SPI_SendDataIT and SPI_ReceiveDataIT
 *
 * @brief					- This functions are to send and receive data to and from SPI peripheral
 *
 * @param[in]				- SPI Handle
 * @param[in]				- 8 bit data to send or receive
 * @param[in]				- 32 bit length of the data
 *
 * @return					- none
 *
 * @Note					- This is a non blocking call

 */


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX)
	{
		// 1. Save the Tx Buffer address and Len information in global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		// 2. Mark SPI state as busy in transmission
		// so that no other code can take over the same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// 3. Enable TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		// 4. Data Transmission will be handled by the ISR code
		// to be filled
	}
	return state;
}


uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX)
	{
		// 1. Save the Rx Buffer address and Len information in global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		// 2. Mark SPI state as busy in transmission
		// so that no other code can take over the same SPI peripheral until transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// 3. Enable TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		// 4. Data Transmission will be handled by the ISR code
		// to be filled
	}
	return state;
}


/*
 * Implementation of SPI IRQ Handling
 */

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1, temp2;
	//check for TXE
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}

	//check for RXNE
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		//handle TXE
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	//check for OVR - overrun error
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		//handle TXE
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}

}

// helper functions

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// check the DFF bit
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) // 16-bit data frame format
	{
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer); // typecasting to 16 bit
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}
	else 								// 8-bit data frame format
	{
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(!pSPIHandle->TxLen)
	{
		// TxLen is zero, close the spi transmission and inform the application that TX is over
		pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
		pSPIHandle->pTxBuffer = NULL;
		pSPIHandle->TxLen = 0;
		pSPIHandle->TxState = SPI_READY;
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

}



/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |=  (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &=  ~(1 << SPI_CR1_SPE);
	}
}



/*********************************************************************
 * @fn      		  - SPI_SSIConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void  SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |=  (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &=  ~(1 << SPI_CR1_SSI);
	}
}


/*********************************************************************
 * @fn      		  - SPI_SSOEConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void  SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |=  (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &=  ~(1 << SPI_CR2_SSOE);
	}
}

/*
 * IRQ configuration and ISR handling
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
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


void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// find out ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (iprx_section * 8) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}
