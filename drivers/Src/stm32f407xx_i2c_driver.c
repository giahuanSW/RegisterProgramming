/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Apr 27, 2025
 *      Author: ASUS
 */
#include "stm32f407xx_i2c_driver.h"
static uint32_t RCC_GetPCLK1Value(void);
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint16_t APB1_PreScaler[4] = {2,4,8,16};

static uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClk;

	uint8_t clksrc, temp, PreScalar_AHB, PreScalar_APB1;

	clksrc = (RCC->CFGR >> 2) & 0x3;
	if (clksrc == 0) // using HSI
	{
		SystemClk = 16000000;
	}
	else if (clksrc == 1) // using HSE
	{
		SystemClk = 80000000;
	}
	else if (clksrc == 2) // using PLL
	{

	}

	temp = (RCC->CFGR >> 4) & 0xF;

	if (temp < 8)
	{
		PreScalar_AHB = 1;
	}
	else
	{
		PreScalar_AHB = AHB_PreScaler[temp - 8];
	}

	temp = 0;
	temp = (RCC->CFGR >> 10) & 0x7;

	if (temp < 4)
	{
		PreScalar_APB1 = 1;
	}
	else
	{
		PreScalar_APB1 = APB1_PreScaler[temp - 4];
	}

	pclk1 = (SystemClk/PreScalar_AHB)/PreScalar_APB1;
	return pclk1;
}
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (pI2Cx == I2C1)
        {
            I2C1_PCLK_EN();
        }
        else if (pI2Cx == I2C2)
        {
            I2C2_PCLK_EN();
        }
        else if (pI2Cx == I2C3)
        {
            I2C3_PCLK_EN();
        }
    }
    else
    {

    }
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pI2Cx->CR1 |= (1 << I2C_CR1_PE);
    }
    else
    {
        pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
    }

}

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;
	//ack control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//configure the FREQ field of CR2
	tempreg =0;
	tempreg |= RCC_GetPCLK1Value()/1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//program the device own address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << I2C_OAR1_ADD1;
	tempreg |= (1 <<14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculation
	uint16_t ccr_value = 0;
	tempreg =0;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// mode is standard mode
		ccr_value = (RCC_GetPCLK1Value() / (2* pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);
	}
	else
	{
		// mode is fast mode
		tempreg |= (1 << I2C_CCR_FS);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);
		 if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		 {
			 ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		 }
		 else
		 {
			 ccr_value = (RCC_GetPCLK1Value() / (25* pI2CHandle->I2C_Config.I2C_SCLSpeed));
		 }
		 tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;
	//TRISE configure
	tempreg = 0;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		tempreg = (RCC_GetPCLK1Value()/1000000U) + 1;
	}
	else
	{
		//mode is fast mode
		tempreg = ((RCC_GetPCLK1Value()*300)/1000000000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{

}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr)
{
	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	//2. confirm that start generation is completed  by checking the SB flag in SR1
	// note: Until SB is cleared SCL will be stretched (pull to LOW)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG));
	// 3. Send the Address of the slave with r/w bit set to w(0) (total 8 bit)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);
	// 4. confirm that address phase is completed by checking the ADDR flag in the SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG));
	// 5. Clear ADDR flag according to its SW sequence
	// note: Until ADDR is cleared ,SCL will be stretched (pull to LOW)
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
	//6. Send data until Len become 0
	while(Len>0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG)); //waiting till TXE is SET
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}
	//7. when Len becomes zero wait for TXE = 1 and BTF = 1 before generating the STOP condition
	// note: TXE = 1, BTF = 1 mean that both SR and DR are empty and next transmission should  begin
	// when BTF = 1 SCL will be stretched (pull to LOW)]
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG));
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_BTF_FLAG));
	//8. Generate STOP condition and master need not to wait for the completion of STOP condition
	// note: generating STOP, automatically clears the BTF
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr)
{
	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	//2. confirm that start generation is completed by checking the SB flag in SR1
	// note: Until SB is cleared SCL will by stretched (pull to LOW)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG));
	//3. Send the address of slave with r/w bit set to R(1) total(8bit)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);
	//4. wait until address phase is completed by checking the ADDr flag in the SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG));
	//procedure to read only 1 byte from slave
	if(Len == 1)
	{
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
		// clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
		// wait until RxNE =1
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG));
		// Generate STOP condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		// read data into the buffer
		*pRxbuffer = pI2CHandle->pI2Cx->DR;

	}

	//procedure to read > 1 byte from slave
	if (Len >1)
	{
		// clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
		//read the data until Len == 0
		for(uint32_t i = Len; i>0; i--)
		{
			// wait until RXNE becomes 1
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG));

			if (i == 2)
			{
				// clear ask bit
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
				// generate STOP condition
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			// read the data from data register into buffer
			*pRxbuffer = pI2CHandle->pI2Cx->DR;
			// increment he buffer address
			pRxbuffer++;
		}
	}
	// re-enable ACKing
	if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == I2C_ACK_ENABLE)
	{
		//enable the ACK
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
	else
	{
		// disable the ACK
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if (pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1; // making space for read/write bit
	SlaveAddr &= ~(1<<0);
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1; // making space for read/write bit
	SlaveAddr |= (1<<0);
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void)dummyRead;
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxbuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;
}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxbuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1 = 0, temp2 = 0, temp3 = 0;
	//Interrupt handling for both master and slave mode of a device
	temp1 = (pI2CHandle->pI2Cx->CR2) & (1 << I2C_CR2_ITEVTEN);
	temp2 = (pI2CHandle->pI2Cx->CR2) & (1 << I2C_CR2_ITBUFEN);
	temp3 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_SB);
	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	if (temp1 && temp3)
	{
		//SB flag is set
	}

	temp3 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_ADDR);
	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address
	if (temp1 && temp3)
	{
		//ADDR flag is set
	}
	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event

	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set

	//5. Handle For interrupt generated by TXE event

	//6. Handle For interrupt generated by RXNE event
}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

}
