/*
 * stm32f4xx_spi_driver.c
 *
 *  Created on: Apr 8, 2024
 *      Author: leo
 */


#include "stm32f4xx_spi_driver.h"

static void spi_txe_int_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxe_int_handle(SPI_Handle_t *pSPIHandle);
static void spi_err_int_handle(SPI_Handle_t *pSPIHandle);

/**************************************************
 * @fn             -
 *
 * @brief          -
 *
 * @param[in]      -
 * @param[in]      -
 * @param[in]      -
 *
 * @return         - none
 *
 * @Note           - none
 */

/*
 * Peripheral Clock Setup
 */
/**************************************************
 * @fn             - SPI_PeriClock_Control
 *
 * @brief          - Enables or disable periheral clock for the given spiport
 *
 * @param[in]      - base address of the peripheral
 * @param[in]      - ENABLE or disable macros
 *
 *
 * @return         - none
 *
 * @Note           - none
 */
void SPI_PeriClock_Control(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
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
			SPI1_PCLK_DIS();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DIS();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DIS();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_DIS();
		}
	}

}
/**************************************************
 * @fn             - SPI_Init()
 *
 * @brief          - Initialize spi with config values
 *
 * @param[in]      - SPI Handle structure pointer
 * @param[in]      -
 * @param[in]      -
 *
 * @return         - none
 *
 * @Note           - none
 */

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t temp_reg = 0;

	//Configure device mode
	temp_reg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//BIDI MODE SHOULD BE CLEARED
		temp_reg &= ~( 1 << 15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//BIDI MODE SHOULD BE enabled
		temp_reg |= ( 1 << 15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMP_RXONLY)
	{
		//BIDI MODE SHOULD BE CLEARED
		temp_reg &= ~( 1 << 15);
		//RXonly bit must be set
		temp_reg |= ( 1 << 10);
	}

	//Configure spi serial clock speed (baudrate)
	temp_reg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << 3;

	//Configure DFF
	temp_reg |= pSPIHandle->SPIConfig.SPI_DFF << 11;

	//Configure CPOL
	temp_reg |= pSPIHandle->SPIConfig.SPI_CPOL << 1;

	//Configure spi serial clock speed (baudrate)
	temp_reg |= pSPIHandle->SPIConfig.SPI_CPHA << 0;

	pSPIHandle->pSPIx->SPI_CR1 = temp_reg;

}
/**************************************************
 * @fn             - SPI_DeInit()
 *
 * @brief          - Deinitialization of a SPI port
 *
 * @param[in]      - Structure spi handle with config atributes
 *
 * @return         - none
 *
 * @Note           - none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
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
/**************************************************
 * @fn             - SPI_GetFlagStatus
 *
 * @brief          - returns the value of the given flag
 *
 * @param[in]      - Structure spi handle with config atributes
 * @param[in]      - buffer with the data
 * @param[in]      - lenght of the buffer
 *
 * @return         - none
 *
 * @Note           - none
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SPI_SR & FlagName)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}
/**************************************************
 * @fn             - SPI_SendData()
 *
 * @brief          - send data over spi peripheral
 *
 * @param[in]      - Structure spi handle with config atributes
 * @param[in]      - buffer with the data
 * @param[in]      - lenght of the buffer
 *
 * @return         - none
 *
 * @Note           - this is a blocking call
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTXbuffer, uint32_t len)
{
	while(len > 0)
	{
		//wait until txe is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//Check DFF
		if(pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
		{
			//16 BIT DFF
			//Load data in to the dr
			pSPIx->SPI_DR = *((uint16_t*)pTXbuffer);
			len--;
			len--;
			(uint16_t*)pTXbuffer++;
		}
		else
		{
			//8 BIT DFF
			pSPIx->SPI_DR = *(pTXbuffer);
			len--;
			pTXbuffer++;
		}

	}
}
/**************************************************
 * @fn             - SPI_ReceiveData()
 *
 * @brief          - send data over spi peripheral
 *
 * @param[in]      - Structure spi handle with config atributes
 * @param[in]      - buffer with the data
 * @param[in]      - lenght of the buffer
 *
 * @return         - none
 *
 * @Note           - this is a blocking call
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRXbuffer, uint32_t len)
{
	while(len > 0)
	{
		//wait until txe is set
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		//Check DFF
		if(pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
		{
			//16 BIT DFF
			//Load data FROM the dr
			*((uint16_t*)pRXbuffer) = pSPIx->SPI_DR;
			len--;
			len--;
			(uint16_t*)pRXbuffer++;
		}
		else
		{
			//8 BIT DFF
			*(pRXbuffer) = pSPIx->SPI_DR;
			len--;
			pRXbuffer++;
		}

	}
}
/**************************************************
 * @fn             - SPI_PeripheralControl()
 *
 * @brief          - Enable or disable spi peripheral
 *
 * @param[in]      - Structure spi handle with config atributes
 * @param[in]      - enable or disable macro

 *
 * @return         - none
 *
 * @Note           -
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->SPI_CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SPE);
	}
}
/**************************************************
 * @fn             - SPI_PeripheralControl()
 *
 * @brief          - Enable or disable spi ssi
 *
 * @param[in]      - Structure spi handle with config atributes
 * @param[in]      - enable or disable macro

 *
 * @return         - none
 *
 * @Note           -
 */
void SSI_Control(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->SPI_CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SSI);
	}
}
/**************************************************
 * @fn             - SSOE_Control()
 *
 * @brief          - Enable or disable spi ssoe
 *
 * @param[in]      - Structure spi handle with config atributes
 * @param[in]      - enable or disable macro

 *
 * @return         - none
 *
 * @Note           -
 */
void SSOE_Control(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->SPI_CR2 |= (1 << 2);
	}
	else
	{
		pSPIx->SPI_CR2 &= ~(1 << 2);
	}
}

/**************************************************
 * @fn             - SPI_IRQInterruptConfig()
 *
 * @brief          -	CPU config for irqs in the NVIC
 *
 * @param[in]      -	IRQ number
 * @param[in]      - 	Enable or disable macro
 *
 * @return         - none
 *
 * @Note           - none
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber <= 64)
		{
			*NVIC_ISER1 |= (1 << IRQNumber % 32);
		}
		else if(IRQNumber > 64 && IRQNumber <= 96)
		{
			*NVIC_ISER2 |= (1 << IRQNumber % 64);
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber <= 64)
		{
			*NVIC_ICER1 |= (1 << IRQNumber % 32);
		}
		else if(IRQNumber > 64 && IRQNumber <= 96)
		{
			*NVIC_ICER2 |= (1 << IRQNumber % 64);
		}
	}


}

/**************************************************
 * @fn             - SPI_IRQPriorityConfig()
 *
 * @brief          -	priority assignment for the requested interrupt
 *
 * @param[in]      -	IRQ number
 * @param[in]      - 	Priority value
 *
 * @return         - none
 *
 * @Note           - none
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = ( 8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx*4)) |= (IRQPriority << shift_amount);
}

/**************************************************
 * @fn             - SPI_SendDataIT()
 *
 * @brief          - Send data through interrupt handling
 *
 * @param[in]      - Structure spi handle with config atributes
 * @param[in]      - buffer with the data
 * @param[in]      - lenght of the buffer
 *
 * @return         - state of the tx buffer
 *
 * @Note           - none
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTXbuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		pSPIHandle->pTxBuffer = pTXbuffer;
		pSPIHandle->Txlen = len;

		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//Enable tranmition interrupts
		pSPIHandle->pSPIx->SPI_CR2 |= (1 << 7);
	}


		//from now data transmition is handle by isr code
	return state;
}

/**************************************************
 * @fn             - SPI_SendDataIT()
 *
 * @brief          - Receive data through interrupt handling
 *
 * @param[in]      - Structure spi handle with config atributes
 * @param[in]      - buffer with the data
 * @param[in]      - lenght of the buffer
 *
 * @return         - state of the rx buffer
 *
 * @Note           - none
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRXbuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		pSPIHandle->pRxBuffer = pRXbuffer;
		pSPIHandle->Rxlen = len;

		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		pSPIHandle->pSPIx->SPI_CR2 |= (1 << 6);
	}

	return state;
}
/**************************************************
 * @fn             - SPI_IRQHandling()
 *
 * @brief          -
 *
 * @param[in]      -
 *
 * @return         - none
 *
 * @Note           - none
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	//Check ISR
	uint8_t temp1;
	uint8_t temp2;

	temp1 = pSPIHandle->pSPIx->SPI_SR & (1 << SPI_SR_TXE); //TX ENABLED
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1 << 7);  //TX INTERRUPT ENABLED

	if(temp1 && temp2)
	{
		spi_txe_int_handle(pSPIHandle);
	}
	temp1 = pSPIHandle->pSPIx->SPI_SR & (1 << SPI_SR_RXNE); //RX ENABLED
	temp2 = pSPIHandle->pSPIx->SPI_CR2  & (1 << 6);  //RX INTERRUPT ENABLED

	if(temp1 && temp2)
	{
		spi_rxe_int_handle(pSPIHandle);
	}

	temp1 = pSPIHandle->pSPIx->SPI_SR & (1 << SPI_SR_OVR); //ovr flag
	temp2 = pSPIHandle->pSPIx->SPI_CR2  & (1 << 5);  //erro INTERRUPT ENABLED

	if(temp1 && temp2)
	{
		spi_err_int_handle(pSPIHandle);
	}

}

//AUXILIAR FUNCTIONS

/**************************************************
 * @fn             - spi_txe_int_handle()
 *
 * @brief          -	handle the tranmition routine
 *
 * @param[in]      -    SPI handle structure
 *
 * @return         - none
 *
 * @Note           - This function should not be called by user
 */
static void spi_txe_int_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
	{
		//16 BIT DFF
		//Load data in to the dr
		pSPIHandle->pSPIx->SPI_DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->Txlen--;
		pSPIHandle->Txlen--;
		pSPIHandle->pTxBuffer++;
	}
	else
	{
		//8 BIT DFF
		pSPIHandle->pSPIx->SPI_DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->Txlen--;
		pSPIHandle->pTxBuffer++;
	}

	//if len is 0, close the communication

	if(pSPIHandle->Txlen == 0)
	{
		SPI_CloseTransmit(pSPIHandle);

		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

/**************************************************
 * @fn             - spi_rxe_int_handle()
 *
 * @brief          -	handle the reception routine
 *
 * @param[in]      -    SPI handle structure
 *
 * @return         - none
 *
 * @Note           - This function should not be called by user
 */
static void spi_rxe_int_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
	{
		//16 BIT DFF
		//Load data in to the dr
		pSPIHandle->pSPIx->SPI_DR = *((uint16_t*)pSPIHandle->pRxBuffer);
		pSPIHandle->Rxlen--;
		pSPIHandle->Rxlen--;
		pSPIHandle->pRxBuffer++;
	}
	else
	{
		//8 BIT DFF
		pSPIHandle->pSPIx->SPI_DR = *(pSPIHandle->pRxBuffer);
		pSPIHandle->Rxlen--;
		pSPIHandle->pRxBuffer++;
	}

	if(pSPIHandle->Txlen == 0)
	{
		SPI_CloseRecept(pSPIHandle);

		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}
/**************************************************
 * @fn             - spi_err_int_handle()
 *
 * @brief          -	handle the error interrupt
 *
 * @param[in]      -    SPI handle structure
 *
 * @return         - none
 *
 * @Note           - This function should not be called by user
 */
static void spi_err_int_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//clear ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->SPI_DR;
		temp = pSPIHandle->pSPIx->SPI_SR;
	}
	(void)temp;
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);

}
/**************************************************
 * @fn             - SPI_CloseTransmit()
 *
 * @brief          -	Closes interrupt handled transmition
 *
 * @param[in]      -	SPI handle structure
 *
 * @return         - none
 *
 * @Note           - none
 */
void SPI_CloseTransmit(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << 7);  // DISABLE TX INTERRUPT
	pSPIHandle->Txlen = 0;
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxState = SPI_READY;
}
/**************************************************
 * @fn             - SPI_CloseRecept()
 *
 * @brief          -	Closes interrupt handled reception
 *
 * @param[in]      -	SPI handle structure
 *
 * @return         - none
 *
 * @Note           - none
 */
void SPI_CloseRecept(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << 6);  // DISABLE TX INTERRUPT
	pSPIHandle->Rxlen = 0;
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->pSPIx->SPI_SR &= ~(1 << SPI_SR_RXNE);
}
/**************************************************
 * @fn             - SPI_ClearOVRFlag()
 *
 * @brief          -	Clear Overrun error flag
 *
 * @param[in]      -	SPI registers structure
 *
 * @return         - none
 *
 * @Note           - none
 */
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;

	temp = pSPIx->SPI_DR;
	temp = pSPIx->SPI_SR;

	(void)temp;
}
/**************************************************
 * @fn             - SPI_ApplicationEventCallback()
 *
 * @brief          -	callback for an interrupt event
 *
 * @param[in]      -
 *
 * @return         - none
 *
 * @Note           - none
 */
__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t spi_event)
{
	//weak implementation, app may overrun this function
}
