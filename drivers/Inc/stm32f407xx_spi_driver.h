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

#define SPI_READY							0
#define SPI_BUSY_IN_RX						1
#define SPI_BUSY_IN_TX						2

/*
 * Possible SPI Application events
 */
#define SPI_EVENT_TX_CMPLT	1
#define SPI_EVENT_RX_CMPLT	2
#define SPI_EVENT_OVR_ERR	3
#define SPI_EVENT_CRC_ERR	4

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
	uint8_t 	 *pTxBuffer;
	uint8_t		 *pRxBuffer;
	uint32_t	 TxLen;
	uint32_t	 RxLen;
	uint8_t		 TxState;
	uint8_t		 RxState;
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
/*
* @fn			- SPI_IRQPriorityConfig
 * @brief		- This function sets the priority for the given SPI peripheral interrupt
 * @param[in]	- IRQNumber: IRQ number of the SPI peripheral
 * @param[in]	- IRQPriority: priority value to be set
 * @return		- none
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t EnorDi);
/*
* @fn			- SPI_IRQHandling
 * @brief		- This function handles the SPI interrupt
 * @param[in]	- pSPIHandle: pointer to the SPI handle structure
 * @return		- none
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);
/*
 * @fn			- SPI_GetFlagStatus
 * @brief		- This function checks the status of the given flag in the SPI_SR register
 * @param[in]	- pSPIx: base address of the SPI peripheral
 * @param[in]	- FlagName: name of the flag to check
 * @return		- FLAG_SET or FLAG_RESET
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
/*
 * @fn			- SPI_PeripheralControl
 * @brief		- This function enables or disables the SPI peripheral
 * @param[in]	- pSPIx: base address of the SPI peripheral
 * @param[in]	- EnOrDi: ENABLE or DISABLE macros
 * @return		- none
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnOrDi);
/*
 * @fn			- SPI_SSIConfig
 * @brief		- This function configures the SSI bit in the SPI_CR1 register
 * @param[in]	- pSPIx: base address of the SPI peripheral
 * @param[in]	- EnOrDi: ENABLE or DISABLE macros
 * @return		- none
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnOrDi);
/*
 * @fn			- SPI_SSOEConfig
 * @brief		- This function configures the SSOE bit in the SPI_CR2 register
 * @param[in]	- pSPIx: base address of the SPI peripheral
 * @param[in]	- EnOrDi: ENABLE or DISABLE macros
 * @return		- none
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnOrDi);
/*
* @fn			- SPI_SendDataIT
 * @brief		- This function sends data using the SPI peripheral in interrupt mode
 * @param[in]	- pSPIHandle: pointer to the SPI handle structure
 * @param[in]	- pTxBuffer: pointer to the data buffer to be sent
 * @param[in]	- Len: length of the data to be sent
 * @return		- none
 */
uint8_t SPI_SendData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
/*
* @fn			- SPI_ReceiveDataIT
 * @brief		- This function receives data using the SPI peripheral in interrupt mode
 * @param[in]	- pSPIHandle: pointer to the SPI handle structure
 * @param[in]	- pRxBuffer: pointer to the data buffer to store received data
 * @param[in]	- Len: length of the data to be received
 * @return		- none
 */
uint8_t SPI_ReceiveData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);
#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
