/*
 * stm32f4xx_spi_driver.c
 *
 *  Created on: Apr 8, 2024
 *      Author: leo
 */


#include "stm32f4xx_spi_driver.h"
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
 * @brief          - send data over spi peripheral
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
