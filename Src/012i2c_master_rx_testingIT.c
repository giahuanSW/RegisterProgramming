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
#define MY_ADDR 0x61;

#define SLAVE_ADDR  0x68

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

I2C_Handle_t I2C1Handle;
uint8_t rxComplt;
//some data
uint8_t rcv_buf[32];
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
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
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
	uint8_t commandcode;
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

	//enable the i2c peripheral
	 I2C_ManageAcking(I2C1,I2C_ACK_ENABLE);
	while(1)
	{
		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();
		commandcode = 0x51;
		while(I2C_MasterSendDataIT(&I2C1Handle,&commandcode,1,SLAVE_ADDR,1) != I2C_READY);
		while(I2C_MasterReceiveDataIT(&I2C1Handle,&len,1,SLAVE_ADDR,1) != I2C_READY);
		commandcode = 0x52;
		while(I2C_MasterSendDataIT(&I2C1Handle,&commandcode,1,SLAVE_ADDR,1) != I2C_READY);
		while(I2C_MasterReceiveDataIT(&I2C1Handle,rcv_buf,len,SLAVE_ADDR,0)!= I2C_READY);
		rxComplt = RESET;
		while(rxComplt != SET);
		rxComplt = RESET;
		rcv_buf[len+1]= '\0';
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
	if (AppEv == I2C_EV_TX_CMPLT)
	{
		printf("Tx is completed\n");
	}
	else if(AppEv == I2C_EV_RX_CMPLT)
	{
		printf("Rx is completed\n");
		rxComplt = SET;
	}
	else if(AppEv == I2C_ERROR_AF)
	{
		printf("Error: Ack failure");
		//in master ack failure happens when slave fails to send ack for the byte
		//sent from master
		I2C_CloseSendData(pI2CHandle);
		//generate the stop condition to release the bus
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}
}
