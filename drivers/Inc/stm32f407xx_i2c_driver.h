/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: Apr 27, 2025
 *      Author: ASUS
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include <stm32f407xx.h>

typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;

typedef struct
{
	I2C_RegDef_t *pI2Cx;		// This holds the base address of the I2C peripheral
	I2C_Config_t I2C_Config;	// This holds I2C configuration settings
	uint8_t 	 *pTxBuffer;	// To store the app. Tx buffer address
	uint8_t 	 *pRxBuffer;	// To store the app. Rx buffer address
	uint32_t	 TxLen;			// To store Tx length
	uint32_t	 RxLen;			// To store Rx length
	uint8_t		 TxRxState;		// To store Communication state
	uint8_t		 DevAddr;		// To store slave/device address
	uint32_t	 RxSize;		// To store Rx size
	uint8_t	  	 Sr;			// To store repeated start condition
}I2C_Handle_t;

/*
 * @I2C_SCLSPEED
 */
#define I2C_SCL_SPEED_SM	100000
#define I2C_SCL_SPEED_FM4K	400000
#define I2C_SCL_SPEED_FM2K	200000

/*
 * @I2C_ACKControl
 */

#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1

/*
 * @I2C_EVENT
 */
#define I2C_READY		0
#define I2C_BUSY_IN_TX 	1
#define I2C_BUSY_IN_RX 	2

/*
 * I2C related status flags definitions
 */

#define I2C_SB_FLAG							(1 << I2C_SR1_SB)
#define I2C_ADDR_FLAG						(1 << I2C_SR1_ADDR)
#define I2C_BTF_FLAG						(1 << I2C_SR1_BTF)
#define I2C_ADD10_FLAG						(1 << I2C_SR1_ADD10)
#define I2C_STOP_FLAG						(1 << I2C_SR1_STOPF)
#define I2C_RXNE_FLAG						(1 << I2C_SR1_RXNE)
#define I2C_TXE_FLAG						(1 << I2C_SR1_TXE)
#define I2C_ARLO_FLAG						(1 << I2C_SR1_ARLO)
#define I2C_BERR_FLAG						(1 << I2C_SR1_BERR)
#define I2C_AF_FLAG							(1 << I2C_SR1_AF)
#define I2C_OVR_FLAG						(1 << I2C_SR1_OVR)
#define I2C_PECERR_FLAG						(1 << I2C_SR1_PECERR)
#define I2C_TIMEOUT_FLAG					(1 << I2C_SR1_TIMEOUT)
#define I2C_SMBALERT_FLAG					(1 << I2C_SR1_SMBALERT)
/*********************************************************************************************************
*                           APIs supported by this driver
*               For more information about the APIs, check the function definitions in the source file
 *********************************************************************************************************/
/*
* @fn			- I2C_PeriClockControl
 * @brief		- This function enables or disables peripheral clock for the given I2C peripheral
 * @param[in]	- pI2Cx: base address of the I2C peripheral
 * @param[in]	- EnorDi: ENABLE or DISABLE macros
 * @return		- none
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
* @fn			- I2C_Init
 * @brief		- This function initializes the I2C peripheral according to the given configuration
 * @param[in]	- pI2CHandle: pointer to the I2C handle structure
 * @return		- none
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
/*
* @fn			- I2C_IRQPriorityConfig
 * @brief		- This function sets the priority for the given I2C peripheral interrupt
 * @param[in]	- IRQNumber: IRQ number of the I2C peripheral
 * @param[in]	- IRQPriority: priority value to be set
 * @return		- none
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t EnorDi);
/*
* @fn           - I2C_EV_IRQHandling
 * @brief		- This function handles the I2C event interrupt
 * @param[in]	- pI2CHandle: pointer to the I2C handle structure
 * @return		- none
*/
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
/*
* @fn           - I2C_ER_IRQHandling
 * @brief		- This function handles the I2C error interrupt
 * @param[in]	- pI2CHandle: pointer to the I2C handle structure
 * @return		- none
*/
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);
/*
 * @fn			- I2C_GetFlagStatus
 * @brief		- This function checks the status of the given flag in the I2C_SR register
 * @param[in]	- pI2Cx: base address of the I2C peripheral
 * @param[in]	- FlagName: name of the flag to check
 * @return		- FLAG_SET or FLAG_RESET
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
/*
 * @fn			- I2C_PeripheralControl
 * @brief		- This function enables or disables the I2C peripheral
 * @param[in]	- pI2Cx: base address of the I2C peripheral
 * @param[in]	- EnOrDi: ENABLE or DISABLE macros
 * @return		- none
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t EnOrDi);

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

/*
 * @fn			- I2C_MasterSendData
 * @brief		- This function sends data to the specified slave device
 * @param[in]	- pI2CHandle: pointer to the I2C handle structure
 * @param[in]	- pTxbuffer: pointer to the data buffer to be sent
 * @param[in]	- Len: length of the data to be sent
 * @param[in]	- SlaveAddr: address of the slave device
 * @return		- none
*/
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr);
/*
 * @fn			- I2C_MasterReceiveData
 * @brief		- This function receives data from the specified slave device
 * @param[in]	- pI2CHandle: pointer to the I2C handle structure
 * @param[in]	- pRxbuffer: pointer to the data buffer to store received data
 * @param[in]	- Len: length of the data to be received
 * @param[in]	- SlaveAddr: address of the slave device
 * @return		- none
*/
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr);
/*
* @fn           - I2C_MasterSendDataIT
 * @brief        - This function sends data to the specified slave device using interrupt
 * @param[in]    - pI2CHandle: pointer to the I2C handle structure
 * @param[in]    - pTxbuffer: pointer to the data buffer to be sent
 * @param[in]    - Len: length of the data to be sent
 * @param[in]    - SlaveAddr: address of the slave device
 * @param[in]    - Sr: repeated start condition
 * @return       - none
 * @note         - This function is used for non-blocking communication
*/
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

/*
 * @fn			- I2C_MasterReceiveDataIT
 * @brief		- This function receives data from the specified slave device using interrupt
 * @param[in]	- pI2CHandle: pointer to the I2C handle structure
 * @param[in]	- pRxbuffer: pointer to the data buffer to store received data
 * @param[in]	- Len: length of the data to be received
 * @param[in]	- SlaveAddr: address of the slave device
 * @param[in]	- Sr: repeated start condition
 * @return		- none
 * @note		- This function is used for non-blocking communication
*/
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
/*
 * @fn			- I2C_ManageAcking
 * @brief		- This function manages the ACK/NACK response for the I2C communication
 * @param[in]	- pI2Cx: base address of the I2C peripheral
 * @param[in]	- EnorDi: ENABLE or DISABLE macros
 * @return		- none
 * @note		- This function is used to disable ACK for the last byte received
 * 				- This is required when only one byte is received from the slave
*/
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
