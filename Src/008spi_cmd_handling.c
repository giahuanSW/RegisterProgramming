/*
 * 006spi_tx_testing.c
 *
 *  Created on: Apr 15, 2025
 *      Author: ASUS
 */


#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_gpio_driver.h"
#include <stdint.h>
#include <string.h>
/*
 * PB14 --> MISO
 * PB15 --> MOSI
 * PB12 --> NSS
 * PB13 --> SLCK
 */
#define COMMAND_LED_CTRL      	0x50
#define COMMAND_SENSOR_READ 	0x51
#define COMMAND_LED_READ 		0x52
#define COMMAND_PRINT 			0x53
#define COMMAND_ID_READ 		0x54

#define LED_ON 0x01
#define LED_OFF 0x00

//arduino analog pins
#define ANALOG_PIN0      0
#define ANALOG_PIN1      1
#define ANALOG_PIN2      2
#define ANALOG_PIN3      3
#define ANALOG_PIN4      4

//arduino led
#define LED_PIN          9
uint8_t command_code[4] = {COMMAND_LED_CTRL,COMMAND_SENSOR_READ,COMMAND_LED_READ,COMMAND_PRINT, COMMAND_ID_READ};
void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5; //AF5
	SPIPins.GPIO_PinConfig.GPIO_PinOptype = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);
	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);
	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);
	//	NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}
uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
	if (ackbyte == 0xF5)
	{
		return 1;
	}
	return 0;
}
void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DEV8;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BIT;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI;
	SPI_Init(&SPI2handle);
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
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;
	GPIO_ButtonInit();
	SPI2_GPIOInits();
	SPI2_Inits();
	SPI_SSOEConfig(SPI2, ENABLE);
	while(1)
	{
		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));
		delay();
		SPI_PeripheralControl(SPI2,ENABLE);

		uint8_t ackbyte;
		uint8_t args[2];
		//1. CMD_LED_CTRL

		SPI_SendData(SPI2,&command_code[0],1);
		// do dummy read to clear RXNE flag
		SPI_ReceiveData(SPI2,&dummy_read,1);
		// to move this data out of shift register, have to send some dummy data
		SPI_SendData(SPI2,&dummy_write,1);
		SPI_ReceiveData(SPI2,&ackbyte,1);
		if (SPI_VerifyResponse(ackbyte))
		{
			// send arguments
			args[0] = LED_PIN;
			args[1] = LED_ON;

			SPI_SendData(SPI2,args,2);
		}

		//2. CMD_SENSOR_READ		<anolog pin number(1)>
		uint8_t anolog_read;
		dummy_read == 0xff;
		dummy_write = 0xff;
		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));
		delay();
		SPI_SendData(SPI2,&command_code[1],1);
		// do dummy read to clear RXNE flag
		SPI_ReceiveData(SPI2,&dummy_read,1);
		// to move this data out of shift register, have to send some dummy data
		SPI_SendData(SPI2,&dummy_write,1);
		SPI_ReceiveData(SPI2,&ackbyte,1);
		if (SPI_VerifyResponse(ackbyte))
		{
			// send arguments
			args[0] = ANALOG_PIN0;
			SPI_SendData(SPI2,args,1);
		}
		// do dummy read to clear RXNE flag
		SPI_ReceiveData(SPI2,&dummy_read,1);
		delay();
		// to move this data out of shift register, have to send some dummy data
		SPI_SendData(SPI2,&dummy_write,1);
		SPI_ReceiveData(SPI2,&anolog_read,1);
		while(SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG));
		SPI_PeripheralControl(SPI2,DISABLE);
	}
	return 0;
}
