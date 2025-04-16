/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Apr 13, 2025
 *      Author: ASUS
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include <stm32f407xx.h>

/** 
 *  Configuration structure for SPIx peripheral
 */

// SPI device mode
#define SPI_DEVICE_MODE_MASTER				1
#define SPI_DEVICE_MODE_SLAVE				0

// SPI Busconfig
#define SPI_BUS_CONFIG_FD					1	// full duplex
#define SPI_BUS_CONFIG_HD					2	// half duplex
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3	// simplex RX only

//SPI slckspeed
#define SPI_SCLK_SPEED_DEV2					0
#define SPI_SCLK_SPEED_DEV4					1
#define SPI_SCLK_SPEED_DEV8					2
#define SPI_SCLK_SPEED_DEV16				3
#define SPI_SCLK_SPEED_DEV32				4
#define SPI_SCLK_SPEED_DEV64				5
#define SPI_SCLK_SPEED_DEV128				6
#define SPI_SCLK_SPEED_DEV256				7

// SPI Data frame format
#define SPI_DFF_8BIT						0
#define SPI_DFF_16BIT						1

// SPI Clock polarity
#define SPI_CPOL_LOW						0
#define SPI_CPOL_HIGH						1

// SPI Clock phase
#define SPI_CPHA_LOW						0
#define SPI_CPHA_HIGH						1

// SPI Software slave management
#define SPI_SSM_EN							1
#define SPI_SSM_DI							0

#define SPI_TXE_FLAG						(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG						(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG						(1 << SPI_SR_BSY)
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t	SPI_CPHA;
	uint8_t SPI_SSM;
	uint8_t SPI_SclkSpeed;
}SPI_Config_t;

/** 
 *  Handle structure for SPIx peripheral
 */

typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
}SPI_Handle_t;

/*********************************************************************************************************
*                           APIs supported by this driver     
*               For more information about the APIs, check the function definitions in the source file
 *********************************************************************************************************/
/*
* @fn			- SPI_PeriClockControl
 * @brief		- This function enables or disables peripheral clock for the given SPI peripheral
 * @param[in]	- pSPIx: base address of the SPI peripheral
 * @param[in]	- EnorDi: ENABLE or DISABLE macros
 * @return		- none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
* @fn			- SPI_Init
 * @brief		- This function initializes the SPI peripheral according to the given configuration
 * @param[in]	- pSPIHandle: pointer to the SPI handle structure
 * @return		- none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);
/*
* @fn			- SPI_SendData
 * @brief		- This function sends data using the SPI peripheral
 * @param[in]	- pSPIx: base address of the SPI peripheral
 * @param[in]	- pTxBuffer: pointer to the data buffer to be sent
 * @param[in]	- Len: length of the data to be sent
 * @return		- none
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
/*
* @fn			- SPI_ReceiveData
 * @brief		- This function receives data using the SPI peripheral
 * @param[in]	- pSPIx: base address of the SPI peripheral
 * @param[in]	- pRxBuffer: pointer to the data buffer to store received data
 * @param[in]	- Len: length of the data to be received
 * @return		- none
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);
/*
* @fn			- SPI_IRQInterruptConfig
 * @brief		- This function configures the interrupt for the given SPI peripheral
 * @param[in]	- IRQNumber: IRQ number of the SPI peripheral
 * @param[in]	- EnorDi: ENABLE or DISABLE macros
 * @return		- none
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnOrDi);
#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
