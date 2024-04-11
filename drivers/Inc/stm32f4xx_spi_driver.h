/*
 * stm32f4xx_spi_driver.h
 *
 *  Created on: Apr 8, 2024
 *      Author: leo
 */

#ifndef INC_STM32F4XX_SPI_DRIVER_H_
#define INC_STM32F4XX_SPI_DRIVER_H_

#include "stm32f407xx.h"
#include "stdint.h"
#include "stddef.h"

/*
 * Configuration structure for SPIx peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

/*
 * Handle structure
 */

typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t Txlen;
	uint32_t Rxlen;
	uint8_t TxState;
	uint8_t RxState;
}SPI_Handle_t;



/*
 * @SPI_DeviceMode
 */

#define SPI_DEVICE_MODE_MASTER     1
#define SPI_DEVICE_MODE_SLAVE      0

/*
 * @SPI_BusConfig
 */

#define SPI_BUS_CONFIG_FD            1
#define SPI_BUS_CONFIG_HD            2
#define SPI_BUS_CONFIG_SIMP_RXONLY   3
//#define SPI_BUS_CONFIG_SIMP_RXONLY   4

/*
 * @SPI_CLKSpeed
 */

#define SPI_SCLK_SPEED_DIV2        0
#define SPI_SCLK_SPEED_DIV4        1
#define SPI_SCLK_SPEED_DIV8		   2
#define SPI_SCLK_SPEED_DIV16       3
#define SPI_SCLK_SPEED_DIV32       4
#define SPI_SCLK_SPEED_DIV64       5
#define SPI_SCLK_SPEED_DIV128      6
#define SPI_SCLK_SPEED_DIV256      7

/*
 * @SPI_DFF
 */

#define SPI_DFF_8BITS  0
#define SPI_DFF_16BITS 1

/*
 * @SPI_CPOL
 */

#define SPI_CPOL_HIGH  1
#define SPI_CPOL_LOW   0

/*
 * @SPI_CPHA
 */

#define SPI_CPHA_HIGH  1
#define SPI_CPHA_LOW   0

/*
 * @SPI_CSSM
 */

#define SPI_CSSM_EN   1
#define SPI_CSSM_DIS  0

/*
 * SPI related flags
 */
#define SPI_TXE_FLAG   (1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG  (1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG  (1 << SPI_SR_BSY)

/*
 * SPI app states
 */
#define SPI_READY		0
#define SPI_BUSY_IN_RX 	1
#define SPI_BUSY_IN_TX 	2

/*
 * Events
 */
#define SPI_EVENT_TX_CMPLT 1
#define SPI_EVENT_RX_CMPLT 2
#define SPI_EVENT_OVR_ERR  3
#define SPI_EVENT_CRC_ERR  4

/*
 * Peripheral clock setup
 */


void SPI_PeriClock_Control(SPI_RegDef_t *pSPIx, uint8_t EnorDi);



/*
 * Init and Deinit
 */

void SPI_Init(SPI_Handle_t *);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


/*
 * Send and Receive Data
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTXbuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRXbuffer, uint32_t len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTXbuffer, uint32_t len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRXbuffer, uint32_t len);

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SSI_Control(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SSOE_Control(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
/*
 * Get flag status
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

/*
 * IRQ
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmit(SPI_Handle_t *pSPIHandle);
void SPI_CloseRecept(SPI_Handle_t *pSPIHandle);

/*
 * Application callback
 */

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t spi_event);

#endif /* INC_STM32F4XX_SPI_DRIVER_H_ */
