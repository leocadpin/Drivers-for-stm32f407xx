/*
 * stm32f4xx_usart_drivers.h
 *
 *  Created on: Apr 19, 2024
 *      Author: leo
 */

#ifndef INC_STM32F4XX_USART_DRIVERS_H_
#define INC_STM32F4XX_USART_DRIVERS_H_

#include "stm32f407xx.h"

/*
 * Configuration structure for USART
 */
typedef struct
{
	uint8_t USART_Mode;
	uint32_t USART_Baud;
	uint8_t USART_nb_stp_bits;
	uint8_t USART_wrd_length;
	uint8_t USART_parity_control;
	uint8_t USART_hw_flow_control;
}USART_Config_t;

/*
 * Handle structure for USART peripheral
 */
typedef struct
{
	USART_RegDef_t *pUSARTx;
	USART_Config_t USART_Config;
}USART_Handle_t;


/*
 * Init and deinit
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_Deinit(USART_RegDef_t *pUSARTx);

/*
 * Peripheral clock related functions
 */

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t en_or_di);
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t en_or_di);


/*
 * Flag functions
 */

uint8_t USART_Get_Flag_status(USART_RegDef_t *pUSARTx, uint8_t status_flag_name);
void USART_Clear_Flag(USART_RegDef_t *pUSARTx, uint16_t status_flag_name);

/*
 * Interrupt related functions
 */
void USART_IRQ_interrupt_config(uint8_t IRQ_number, uint8_t en_or_di);
void USART_IRQ_priority_config(uint8_t IRQ_number, uint32_t en_or_di);

#endif /* INC_STM32F4XX_USART_DRIVERS_H_ */
