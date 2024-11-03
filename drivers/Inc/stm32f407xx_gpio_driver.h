/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Oct 20, 2024
 *      Author: ASUS
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/*
* @GPIO_PIN_MODES
* GPIO pin possible modes
*/
#define GPIO_MODE_INPUT         0
#define GPIO_MODE_OUTPUT        1
#define GPIO_MODE_ALTFN         2
#define GPIO_MODE_ANALOG        3
#define GPIO_MODE_IT_FT         4  //Fall edge
#define GPIO_MODE_IT_RT         5  //Rising edge
#define GPIO_MODE_IT_RFT        6  //Rising and Falling edge

/*
* @GPIO_PIN_OUTPUT_TYPE
* GPIO pin possible output types
*/

#define GPIO_OP_TYPE_PP         0
#define GPIO_OP_TYPE_OP         1

/*
* @GPIO_PIN_SPEED
* GPIO pin possible output speed
*/

#define GPIO_SPEED_LOW          0
#define GPIO_SPEED_MEDIUM       1
#define GPIO_SPEED_FAST         2
#define GPIO_SPEED_HIGH         3

/*
* @GPIO_PIN_PULL_UP_DOWN
* GPIO pin pull up pull down configuration macros
*/

#define GPIO_NO_PUPD            0
#define GPIO_PIN_PU             1
#define GPIO_PIN_PD             2

/*
* @GPIO_PIN_NUMBERS
* GPIO pin numbers
*/

#define GPIO_PIN_NO_0           0
#define GPIO_PIN_NO_1           1
#define GPIO_PIN_NO_2           2
#define GPIO_PIN_NO_3           3
#define GPIO_PIN_NO_4           4
#define GPIO_PIN_NO_5           5
#define GPIO_PIN_NO_6           6
#define GPIO_PIN_NO_7           7
#define GPIO_PIN_NO_8           8
#define GPIO_PIN_NO_9           9
#define GPIO_PIN_NO_10          10
#define GPIO_PIN_NO_11          11
#define GPIO_PIN_NO_12          12
#define GPIO_PIN_NO_13          13
#define GPIO_PIN_NO_14          14
#define GPIO_PIN_NO_15          15

/**************************************************************************************
*                           APIs supported by this driver
*               For more information about the APIs check the function definitions 
**************************************************************************************/
typedef struct
{
    uint8_t GPIO_PinNumber;
    uint8_t GPIO_PinMode;               /*possible values from @GPIO_PIN_MODES*/
    uint8_t GPIO_PinSpeed;              /*possible values from @GPIO_PIN_SPEED*/
    uint8_t GPIO_PinPuPdControl;        /*possible values from @GPIO_PIN_PULL_UP_DOWN*/
    uint8_t GPIO_PinOptype;             /*possible values from @GPIO_PIN_OUTPUT_TYPE*/
    uint8_t GPIO_PinAltFunMode;         
}GPIO_PinConfig_t;

typedef struct
{
    GPIO_RegDef_t *pGPIOx;            /*this holds the base address of the GPIO port to which the pin belongs*/
    GPIO_PinConfig_t GPIO_PinConfig;  /*this holds GPIO pin configuration setting*/
}GPIO_Handle_t;



/******************************************************************************************
*                           APIs supported by this driver
******************************************************************************************/

/*Init DeInit*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*Peripheral Clock setup*/
void GPIO_PeriClockControl(RCC_RegDef_t *pGPIOx, uint8_t EnorDi);

/*Data read and Write*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
//void GPIO_WriteFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint8_t Value);
//void GPIO_WriteFromInputPort(void);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);


/*Interupt Configuration adn ISR handling*/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
