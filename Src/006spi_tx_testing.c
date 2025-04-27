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
	char data[] ="Hello world";
	GPIO_ButtonInit();
	SPI2_GPIOInits();
	SPI2_Inits();
	SPI_SSOEConfig(SPI2, ENABLE);
	while(1)
	{
		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));
		delay();
		SPI_PeripheralControl(SPI2,ENABLE);
		uint8_t DataLen = strlen(data);
		SPI_SendData(SPI2,(uint8_t*)&DataLen,1);
		SPI_SendData(SPI2,(uint8_t*)data,strlen(data));
		while(SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG));
		SPI_PeripheralControl(SPI2,DISABLE);
	}
	return 0;
}
