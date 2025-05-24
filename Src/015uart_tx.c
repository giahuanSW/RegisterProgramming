/*
 * 015uart_tx.c
 *
 *  Created on: May 11, 2025
 *      Author: ASUS
 */
#include <stdint.h>
#include <stm32f407xx.h>
#include <stm32f407xx_usart_driver.h>
#include <stm32f407xx_gpio_driver.h>
#include <string.h>

char msg[1024]="UART Tx testing...\n\r";

USART_Handle_t usart2_handle;

void USART2_Init(void)
{
	usart2_handle.pUSARTx = USART2;
	usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_9600;
	usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2_handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&usart2_handle);
}
void USART2_GPIOInits(void)
{
	GPIO_Handle_t USARTPins;
	USARTPins.pGPIOx = GPIOA;
	USARTPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	USARTPins.GPIO_PinConfig.GPIO_PinOptype = GPIO_OP_TYPE_PP;
	USARTPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	USARTPins.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	USARTPins. GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//Tx
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&USARTPins);
	//Rx
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&USARTPins);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GpioBtn;
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GpioBtn);
}

void delay(void)
{
    for(uint32_t i=0;i<500000/2;i++);
}

int main(void)
{
	GPIO_ButtonInit();
	USART2_GPIOInits();
	USART2_Init();
	USART_PeripheralControl(USART2,ENABLE);
	while(1)
	{
		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));
		delay();
		USART_SendData(&usart2_handle,(uint8_t*)msg,strlen(msg));
	}
	return;
}
