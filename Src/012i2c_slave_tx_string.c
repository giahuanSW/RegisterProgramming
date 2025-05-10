/*
 * 006spi_tx_testing.c
 *
 *  Created on: Apr 15, 2025
 *      Author: ASUS
 */


#include "stm32f407xx_i2C_driver.h"
#include "stm32f407xx_gpio_driver.h"
#include <stdint.h>
#include <string.h>

#define SLAVE_ADDR  0x68

#define MY_ADDR_SLAVE_ADDR	0x68
void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

I2C_Handle_t I2C1Handle;
uint8_t rxComplt;
//some data
uint8_t Tx_buf[32] = "STM32 Slave mode testing";
static uint8_t commandcode;
/*
 * PB6-> SCL
 * PB9 or PB7 -> SDA
 */

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	/*Note : Internal pull-up resistors are used */

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOptype = GPIO_OP_TYPE_OP;
	/*
	 * Note : In the below line use GPIO_NO_PUPD option if you want to use external pullup resistors, then you have to use 3.3K pull up resistors
	 * for both SDA and SCL lines
	 */
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins. GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//scl
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);


	//sda
	//Note : since we found a glitch on PB9 , you can also try with PB7
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;

	GPIO_Init(&I2CPins);


}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR_SLAVE_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;
	I2C_Init(&I2C1Handle);

}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);

}


int main(void)
{
	uint8_t len;
	GPIO_ButtonInit();

	//i2c pin inits
	I2C1_GPIOInits();
	 //enable the clock for the i2cx peripheral
	 I2C_PeriClockControl(I2C1,ENABLE);
	//i2c peripheral configuration
	 I2C_PeripheralControl(I2C1,1);
	 I2C1_Inits();
	 I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV,ENABLE);
	 I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER,ENABLE);

	 I2C_SlaveEnableDisableCallbackEvents(I2C1, ENABLE);

	//enable the i2c peripheral
	 I2C_ManageAcking(I2C1,I2C_ACK_ENABLE);

	 while(1)
	 {

	 }
}

void I2C1_EV_IRQHandler(void)
{
	I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler(void)
{
	I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{
	static uint8_t Cnt = 0;
	if(AppEv == I2C_EV_DATA_REQ)
	{
		//Master wants some data, Slave has to send it
		if (commandcode == 0x51)
		{
			I2C_SlaveSendData(pI2CHandle->pI2Cx,strlen((char*)Tx_buf));
		}
		else if (commandcode == 0x52)
		{
			I2C_SlaveSendData(pI2CHandle->pI2Cx,Tx_buf[Cnt++]);
		}
	}
	else if (AppEv == I2C_EV_DATA_RCV)
	{
		//Data is waiting for the slave to read, Slave has to read it
		commandcode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
	}
	else if (AppEv == I2C_ERROR_AF)
	{
		//This happens only during slave txing
		//Master has sent the NACK. So slave should understand that master doesn't need more data
		commandcode == 0xff;
		Cnt = 0;
	}
	else if (AppEv == I2C_EV_STOP)
	{
		//This happens only during slave reception
		//Master has ended the I2C communication with slave
	}
}
