/*
 * stm32f4xx_i2c_driver.c
 *
 *  Created on: Apr 11, 2024
 *      Author: leo
 */

#include"stm32f4xx_i2c_driver.h"

 static void I2C_start_condition(I2C_RegDef_t *pI2Cx);
 static void I2C_stop_condition(I2C_RegDef_t *pI2Cx);
 static void I2C_address_phase(I2C_RegDef_t* pI2Cx, uint8_t slave_addr, uint8_t write_read);
 static void clear_ADDR_flag(I2C_Handle_t *pI2CHandle);


 /**************************************************
  * @fn             - I2C_PeriClock_Control()
  *
  * @brief          - enable cloch for i2c peripheral device
  *
  * @param[in]      - i2c register definition structure
  * @param[in]      - enable or disable option
  * @param[in]      -
  *
  * @return         - none
  *
  * @Note           - none
  */
 void I2C_PeriClock_Control(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
 {

 	if(EnorDi == ENABLE)
 	{
 		if(pI2Cx == I2C1)
 		{
 			I2C1_PCLK_EN();
 		}
 		else if(pI2Cx == I2C2)
 		{
 			I2C2_PCLK_EN();
 		}
 		else if(pI2Cx == I2C3)
 		{
 			I2C3_PCLK_EN();
 		}


 	}
 	else
 	{
 		if(pI2Cx == I2C1)
 		{
 			I2C1_PCLK_DIS();
 		}
 		else if(pI2Cx == I2C2)
 		{
 			I2C2_PCLK_DIS();
 		}
 		else if(pI2Cx == I2C3)
 		{
 			I2C3_PCLK_DIS();
 		}

 	}

 }

/**************************************************
 * @fn             - RCC_Get_PCLK_val()
 *
 * @brief          - Initialize i2c with config values
 *
 * @param[in]      - i2c Handle structure pointer
 * @param[in]      -
 * @param[in]      -
 *
 * @return         - none
 *
 * @Note           - none
 */

uint16_t AHB_PreScaler[9] = {2,4,8,16,32,64,128,256,512};
uint16_t APB1_PreScaler[4] = {2,4,8,16};

uint32_t RCC_Get_PCLK_val(void)
{
	uint32_t pclk1;
	uint32_t Systemclk;

	uint8_t clksrc;
	uint8_t temp;
	uint8_t ahb_p;
	uint8_t apb1_p;

	clksrc = ((RCC->RCC_CFGR >> 2) & 0x3);

	//Clock source calculations
	if(clksrc == 0)
	{
		Systemclk = 16000000;
	}
	if(clksrc == 1)
	{
		Systemclk = 8000000;
	}
	if(clksrc == 2)
	{
		//value would come from pll;
	}

	//Prescaler calculations
	temp = ( (RCC->RCC_CFGR >> 4) & 0XF );

	if(temp < 8)
	{
		ahb_p = 1;
	}
	else
	{
		ahb_p = AHB_PreScaler[temp-8];
	}

	temp = ( (RCC->RCC_CFGR >> 10) & 0X7 );

	if(temp < 4)
	{
		apb1_p = 1;
	}
	else
	{
		apb1_p = APB1_PreScaler[temp-4];
	}

	pclk1 = (Systemclk / ahb_p) / apb1_p;

	return pclk1;
}

/**************************************************
 * @fn             - I2C_Init()
 *
 * @brief          - Initialize i2c with config values
 *
 * @param[in]      - i2c Handle structure pointer
 * @param[in]      -
 * @param[in]      -
 *
 * @return         - none
 *
 * @Note           - none
 */
void I2C_Init(I2C_Handle_t* pI2CHandle)
{
	uint32_t tempreg = 0;

	//enable clock for i2c peripheral
	I2C_PeriClock_Control(pI2CHandle->pI2Cx, ENABLE);

	tempreg |= pI2CHandle->I2C_config.I2C_ACKControl << I2C_CR1_ACK;
	pI2CHandle->pI2Cx->I2C_CR1 = tempreg;

	//configure the freq field of cr2

	tempreg = 0;

	tempreg |= RCC_Get_PCLK_val() / 1000000U;

	pI2CHandle->pI2Cx->I2C_CR2 = (tempreg & 0x3F);

	//program the device own address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->I2C_OAR1 = tempreg;

	//CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;

	if(pI2CHandle->I2C_config.I2C_SCL_Speed <= I2C_SCL_SPEED_SM)
	{
		ccr_value = RCC_Get_PCLK_val() / (2 * pI2CHandle->I2C_config.I2C_SCL_Speed);
		tempreg |= (ccr_value & 0xFFF);
	}
	else
	{
		tempreg |= (1 << 15);
		tempreg |= (pI2CHandle->I2C_config.I2C_FMDutyCycle << 14);

		if(pI2CHandle->I2C_config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = RCC_Get_PCLK_val() / (3 * pI2CHandle->I2C_config.I2C_SCL_Speed);
		}
		else
		{
			ccr_value = RCC_Get_PCLK_val() / (25 * pI2CHandle->I2C_config.I2C_SCL_Speed);
		}
		tempreg |= (ccr_value & 0xFFF);

	}

	pI2CHandle->pI2Cx->I2C_CCR = tempreg;


	tempreg = 0;

	if(pI2CHandle->I2C_config.I2C_SCL_Speed <= I2C_SCL_SPEED_SM)
	{
		tempreg = (RCC_Get_PCLK_val() / 1000000U) + 1;
	}
	else
	{
		tempreg = ((RCC_Get_PCLK_val() * 300U) / 1000000000 ) + 1;
	}

	pI2CHandle->pI2Cx->I2C_TRISE = (tempreg & 0x3F);
}
/**************************************************
 * @fn             - I2C_DeInit()
 *
 * @brief          - Initialize i2c with config values
 *
 * @param[in]      - i2c Handle structure pointer
 * @param[in]      -
 * @param[in]      -
 *
 * @return         - none
 *
 * @Note           - none
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}
	else if(pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}
	else if(pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
}
/**************************************************
 * @fn             - I2C_MasterSendData()
 *
 * @brief          - Initialize i2c with config values
 *
 * @param[in]      - i2c Handle structure pointer
 * @param[in]      - buffer with data
 * @param[in]      - data length
 * @param[in]      - slave address
 * @return         - none
 *
 * @Note           - none
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTXbuffer, uint32_t len, uint8_t slave_addr, uint8_t stop_repeat)
{
	//Start condition
	I2C_start_condition(pI2CHandle->pI2Cx);

	//Confirm start generation is completed checking SB flasg inte sr1
	while(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//send addres to slave with r/w bit to 0 (write)
	I2C_address_phase(pI2CHandle->pI2Cx, slave_addr, I2C_WRITE);

	//confirm that address phase is completed by checking the addr flag in sr1
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//CLEAR ADDR FLAG
	clear_ADDR_flag(pI2CHandle);

	//send data until len gets to 0
	while(len > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)); //wait till txe is set
		pI2CHandle->pI2Cx->I2C_DR = *pTXbuffer;
		pTXbuffer++;
		len--;
	}

	//wait for TXE and BTF are set
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));

	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	//stop condition
	if(stop_repeat == 0)
	{
		I2C_stop_condition(pI2CHandle->pI2Cx);
	}

}


/**************************************************
 * @fn             - I2C_MasterSendData()
 *
 * @brief          - Initialize i2c with config values
 *
 * @param[in]      - i2c Handle structure pointer
 * @param[in]      - buffer with data
 * @param[in]      - data length
 * @param[in]      - slave address
 * @return         - none
 *
 * @Note           - none
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRXbuffer, uint32_t len, uint8_t slave_addr, uint8_t stop_repeat)
{

	//Start condition
	I2C_start_condition(pI2CHandle->pI2Cx);

	//Confirm start generation is completed checking SB flasg inte sr1
	while(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//send address to slave with r/w bit to 1 (write)
	I2C_address_phase(pI2CHandle->pI2Cx, slave_addr, I2C_READ);

	//confirm that address phase is completed by checking the addr flag in sr1
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));


	//if we read only 1 byte from slave
	if(len == 1)
	{
		I2C_manage_acking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);



		clear_ADDR_flag(pI2CHandle);
		while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXE));

		//stop condition
		if(stop_repeat == 0)
		{
			I2C_stop_condition(pI2CHandle->pI2Cx);
		}

		//read data into buffer
		*pRXbuffer = pI2CHandle->pI2Cx->I2C_DR;


	}

	if(len > 1)
	{

		//clear addr flag
		clear_ADDR_flag(pI2CHandle);

		for (uint32_t i = len; len > 0; len--)
		{
			while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXE));

			if(i==2)
			{
				I2C_manage_acking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
				I2C_stop_condition(pI2CHandle->pI2Cx);
			}

			*pRXbuffer = pI2CHandle->pI2Cx->I2C_DR;
			pRXbuffer++;

		}
	}

	//wait for TXE and BTF are set
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXE));

	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	//stop condition
	if(stop_repeat == 0)
	{
		I2C_stop_condition(pI2CHandle->pI2Cx);
	}

}

/**************************************************
 * @fn             - I2C_manage_acking()
 *
 * @brief          - manage ack bit
 *
 * @param[in]      - i2c Handle structure pointer
 * @param[in]      - enable or disable
 * @return         - none
 *
 * @Note           - none
 */
void I2C_manage_acking(I2C_RegDef_t *pI2Cx, uint8_t en_or_di)
{
	if(en_or_di == I2C_ACK_ENABLE)
	{
		pI2Cx->I2C_CR1 |= ( 1 << I2C_CR1_ACK);
	}
	else
	{
		pI2Cx->I2C_CR1 &= ~( 1 << I2C_CR1_ACK);
	}
}


//Private functions

static void I2C_start_condition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_START);
}



/**************************************************
 * @fn             - I2C_GetFlagStatus
 *
 * @brief          - returns the value of the given flag
 *
 * @param[in]      - Structure I2C with register definitions
 * @param[in]      - flag name
 *
 * @return         - none
 *
 * @Note           - none
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->I2C_SR1 & FlagName)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}

/**************************************************
 * @fn             - I2C_GetFlagStatus
 *
 * @brief          - executes the addres phase in the i2c protocol
 *
 * @param[in]      - Structure I2C with register definitions
 * @param[in]      - slave addres
 *
 * @return         - none
 *
 * @Note           - none
 */
static void I2C_address_phase(I2C_RegDef_t* pI2Cx, uint8_t slave_addr, uint8_t write_read)
{
	slave_addr = slave_addr << 1;
	if(write_read == I2C_WRITE)
	{
		slave_addr &= ~(1);
	}
	else
	{
		slave_addr |= (1);
	}

	pI2Cx->I2C_DR = slave_addr;
}

/**************************************************
 * @fn             - clear_ADDR_flag
 *
 * @brief          - executes the addres phase in the i2c protocol
 *
 * @param[in]      - Structure I2C with register definitions

 *
 * @return         - none
 *
 * @Note           - none
 */
static void clear_ADDR_flag(I2C_Handle_t* pI2CHandle)
{
	uint32_t dummy_read;

	if(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL))
	{
		//master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->Rxlen == 1)
			{
				I2C_manage_acking(pI2CHandle->pI2Cx, DISABLE);

				dummy_read = pI2CHandle->pI2Cx->I2C_SR1;
				dummy_read = pI2CHandle->pI2Cx->I2C_SR2;

				(void)dummy_read;
			}
		}
		else
		{
			dummy_read = pI2CHandle->pI2Cx->I2C_SR1;
			dummy_read = pI2CHandle->pI2Cx->I2C_SR2;

			(void)dummy_read;
		}
	}
	else
	{
		//slave mode
		dummy_read = pI2CHandle->pI2Cx->I2C_SR1;
		dummy_read = pI2CHandle->pI2Cx->I2C_SR2;

		(void)dummy_read;
	}
}

/**************************************************
 * @fn             - I2C_stop_condition
 *
 * @brief          - executes the stop condition in the i2c protocol
 *
 * @param[in]      - Structure I2C with register definitions

 *
 * @return         - none
 *
 * @Note           - none
 */

static void I2C_stop_condition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_STOP);
}


/**************************************************
 * @fn             - I2C_MasterSendData_IT
 *
 * @brief          - send data in interruption mode
 *
 * @param[in]      - Structure I2C with register definitions

 *
 * @return         - none
 *
 * @Note           - none
 */

uint8_t I2C_MasterSendData_IT(I2C_Handle_t *pI2CHandle, uint8_t *pTXbuffer, uint32_t len, uint8_t slave_addr, uint8_t stop_repeat)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer =pTXbuffer;
		pI2CHandle->Txlen = len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->dev_addr = slave_addr;
		pI2CHandle->store_repeated = stop_repeat;

		//Start condition
		I2C_start_condition(pI2CHandle->pI2Cx);
		//Enable itbufen control bit
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Enable itevfen control bit
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Enable itevfen control bit
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}
/**************************************************
 * @fn             - I2C_MasterReceiveData_IT
 *
 * @brief          - receive data in interruption mode
 *
 * @param[in]      - Structure I2C with register definitions

 *
 * @return         - none
 *
 * @Note           - none
 */
uint8_t I2C_MasterReceiveData_IT(I2C_Handle_t *pI2CHandle, uint8_t *pRXbuffer, uint32_t len, uint8_t slave_addr, uint8_t stop_repeat)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRXbuffer;
		pI2CHandle->Txlen = len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->dev_addr = slave_addr;
		pI2CHandle->store_repeated = stop_repeat;

		//Start condition
		I2C_start_condition(pI2CHandle->pI2Cx);
		//Enable itbufen control bit
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Enable itevfen control bit
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Enable itevfen control bit
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}


/**************************************************
 * @fn             - I2C_EV_IRQ_handling
 *
 * @brief          - event interruption handler
 *
 * @param[in]      - Structure I2C with register definitions

 *
 * @return         - none
 *
 * @Note           - none
 */
void I2C_EV_IRQ_handling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->I2C_CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->I2C_CR2 & (1 << I2C_CR2_ITBUFEN);

	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_SB);
	//Handle interrupt for SB event
	if(temp1 && temp3)
	{
		//SB event - only in master mode

		//address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_address_phase(pI2CHandle->pI2Cx, pI2CHandle->dev_addr, I2C_WRITE);
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_address_phase(pI2CHandle->pI2Cx, pI2CHandle->dev_addr, I2C_READ);
		}

	}

	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_ADDR);
	//Handle interrupt for ADDR event
	if(temp1 && temp3)
	{
		//ADDR event
		clear_ADDR_flag(pI2CHandle);
	}

	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_BTF);
	//Handle interrupt for BTF event
	if(temp1 && temp3)
	{
		//BTF event
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//make sure txe is set
			if(pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_TxE))
			{
				//BTF, TXE = 1
				if(pI2CHandle->Txlen == 0)
				{
					I2C_stop_condition(pI2CHandle->pI2Cx);

					I2C_Close_TX_data(pI2CHandle);

					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}

			}
		}
	}

	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_STOPF);
	//Handle interrupt for STOP event - slave mode
	if(temp1 && temp3)
	{
		//STOP event
		pI2CHandle->pI2Cx->I2C_CR1 |= 0x0000;
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_TxE);
	//Handle interrupt for txe event
	if(temp1 && temp2 && temp3)
	{
		//txe event

		//confirm master mode
		if(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL))
		{
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				if(pI2CHandle->Txlen > 0)
				{

					pI2CHandle->pI2Cx->I2C_DR = *(pI2CHandle->pTxBuffer);
					pI2CHandle->Txlen--;
					pI2CHandle->pTxBuffer++;
				}
			}
		}
		else //slave mode
		{
			if(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}

		}


	}

	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_RxNE);
	//Handle interrupt for rxe event
	if(temp1 && temp2 && temp3)
	{
		//rxe event
		if(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL))
		{

			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{

				if(pI2CHandle->Rxlen == 1)
				{
					*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
					pI2CHandle->Rxlen--;
				}

				if(pI2CHandle->Rxlen > 1)
				{
					if(pI2CHandle->Rxlen == 2)
					{
						I2C_manage_acking(pI2CHandle->pI2Cx, DISABLE);
					}

					*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
					pI2CHandle->pRxBuffer++;
					pI2CHandle->Rxlen--;

				}
			}

			if(pI2CHandle->Rxlen == 0)
			{
				I2C_stop_condition(pI2CHandle->pI2Cx);
				I2C_Close_RX_data(pI2CHandle);
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
			}
		}
		else //slave mode
		{
			if(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RECV);
			}

		}

	}

}
/**************************************************
 * @fn             - I2C_ERR_IRQ_handling
 *
 * @brief          - error interruption handler
 *
 * @param[in]      - Structure I2C with register definitions

 *
 * @return         - none
 *
 * @Note           - none
 */
void I2C_ERR_IRQ_handling(I2C_Handle_t *pI2CHandle)
{

}








/**************************************************
 * @fn             - I2C_Close_TX_data
 *
 * @brief          - error interruption handler
 *
 * @param[in]      - Structure I2C with register definitions

 *
 * @return         - none
 *
 * @Note           - none
 */
void I2C_Close_TX_data(I2C_Handle_t *pI2CHandle)
{
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
}
/**************************************************
 * @fn             - I2C_Close_RX_data
 *
 * @brief          - error interruption handler
 *
 * @param[in]      - Structure I2C with register definitions

 *
 * @return         - none
 *
 * @Note           - none
 */
void I2C_Close_RX_data(I2C_Handle_t *pI2CHandle)
{
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->Rxlen = 0;
	pI2CHandle->Rx_size = 0;
	I2C_manage_acking(pI2CHandle->pI2Cx, ENABLE);
}


/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Complete the code also define these macros in the driver
						header file


 */

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->I2C_CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_ARLO);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_AF);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_OVR);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_TIMEOUT);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}


void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
	pI2Cx->I2C_DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	return (uint8_t)pI2Cx->I2C_DR;
}

void I2C_Slave_en_dis_callback(I2C_RegDef_t *pI2Cx, uint8_t en_or_dis)
{
	if(en_or_dis == ENABLE)
	{
		pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITEVTEN);
		pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITBUFEN);
		pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITERREN);
	}
	else
	{
		pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITEVTEN);
		pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
		pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITERREN);
	}
}
