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
 * PB12 --> SCLC
 * PB13 --> NSS
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
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
//	GPIO_Init(&SPIPins);
	//NSS
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
//	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DEV2;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BIT;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN;
	SPI_Init(&SPI2handle);
}

int main(void)
{
	char data[] ="Hello world";
	SPI2_GPIOInits();
	SPI2_Inits();
	SPI_SSIConfig(SPI2,1);
	SPI_PeripheralControl(SPI2,1);
	while(1)
	{
		SPI_SendData(SPI2,(uint8_t*)data,strlen(data));
		SPI_PeripheralControl(SPI2,0);
	}
	return 0;
}
