/*
 * stm32f4xx_i2c_driver.h
 *
 *  Created on: Apr 11, 2024
 *      Author: leo
 */

#ifndef INC_STM32F4XX_I2C_DRIVER_H_
#define INC_STM32F4XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

/*
 * I2C config structure
 */
typedef struct
{
	uint32_t I2C_SCL_Speed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;

/*
 * I2C Handle structure
 */
typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_config;

}I2C_Handle_t;



/*
 * Peripheral clock setup
 */


void I2C_PeriClock_Control(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

uint32_t RCC_Get_PCLK_val(void);



/*
 * Init and Deinit
 */

void I2C_Init(I2C_Handle_t *);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);


/*
 * Send and Receive Data
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTXbuffer, uint32_t len, uint8_t slave_addr);
void I2C_ReceiveData(I2C_RegDef_t *pI2Cx, uint8_t *pRXbuffer, uint32_t len);



/*
 * Get flag status
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

/*
 * IRQ
 */

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);


/*
 * Application callback
 */

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t I2C_event);


/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM 100000
#define I2C_SCL_SPEED_FM_2K 200000
#define I2C_SCL_SPEED_FM_4K 400000

/*
 * @ACK_CONTROL
 */

#define I2C_ACK_ENABLE	1
#define I2C_ACK_DISABLE	0

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1

/*
 * I2C status flags
 */
#define I2C_FLAG_TXE 	 	(1 << I2C_SR1_TxE)
#define I2C_FLAG_RXE  		(1 << I2C_SR1_RxE)
#define I2C_FLAG_SB		 	(1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR 	 	(1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF 	    (1 << I2C_SR1_BTF)
#define I2C_FLAG_STOPF	  	(1 << I2C_SR1_STOPF)
#define I2C_FLAG_OVR	  	(1 << I2C_SR1_OVR)
#define I2C_FLAG_BERR	  	(1 << I2C_SR1_BERR)


#endif /* INC_STM32F4XX_I2C_DRIVER_H_ */
