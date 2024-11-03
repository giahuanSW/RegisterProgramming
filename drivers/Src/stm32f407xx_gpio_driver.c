#include "stm32f407xx_gpio_driver.h"

/*Init DeInit*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    uint32_t temp;
    //1. configure the mode of GPIO pin
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
    {
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << ( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));   //clear
        pGPIOHandle->pGPIOx->MODER |= temp;
    }
    else
    {
        // configure IT mode
        if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
        {
            // configure the FTSR
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            //clear the corresponding RTSR bit
            EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }
        else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
        {
            // configure the RTSR
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            //clear the corresponding RTSR bit
            EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }
        else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
        {
            // configure the RFTSR
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }

        //2 configure the GPIO port selection in SYSCFG_EXTICR
        uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
        uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;
        uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
        SYSCFG_PCLK_EN();
        SYSCFG->EXTICR[temp1] = portcode << (temp2 *4);
        //3. enable the exti interrupt delivery using IMR
        EXTI->IRM |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        
    }

    //2. configure the speed
    temp = 0;
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << ( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));   //clear
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;

    //3. configure the pull up/dow setting
    temp = 0;
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << ( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));   //clear
    pGPIOHandle->pGPIOx->PUPDR |= temp;

    //4. configure the output type
    temp = 0;
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOptype <<  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x1 << ( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));   //clear
    pGPIOHandle->pGPIOx->OTYPER |= temp;

    //5. configure the alt functionality
    temp = 0;
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
    {
        uint32_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
        uint32_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;
        pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4*temp2));
        pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));
    }

}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
    if (pGPIOx == GPIOA)
    {
        GPIOA_REG_RESET();
    }
    else if (pGPIOx == GPIOB)
    {
        GPIOB_REG_RESET();
    }
    else if (pGPIOx == GPIOC)
    {
        GPIOC_REG_RESET();
    }
    else if (pGPIOx == GPIOD)
    {
        GPIOD_REG_RESET();
    }
    else if (pGPIOx == GPIOE)
    {
        GPIOE_REG_RESET();
    }
    else if (pGPIOx == GPIOF)
    {
        GPIOF_REG_RESET();
    }
    else if (pGPIOx == GPIOG)
    {
        GPIOG_REG_RESET();
    }
    else if (pGPIOx == GPIOH)
    {
        GPIOH_REG_RESET();
    }
    else if (pGPIOx == GPIOI)
    {
        GPIOI_REG_RESET();
    }
}

/*Peripheral Clock setup*/
void GPIO_PeriClockControl(RCC_RegDef_t *pGPIOx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_EN();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_EN();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_EN();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_EN();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_EN();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_EN();
        }
        else if (pGPIOx == GPIOG)
        {
            GPIOG_PCLK_EN();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_PCLK_EN();
        }
        else if (pGPIOx == GPIOI)
        {
            GPIOI_PCLK_EN();
        }
    }
    else
    {

    }
}

/*Data read and Write*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    uint8_t value;
    value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
    return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    uint16_t value;
    value = (uint16_t)(pGPIOx->IDR);
    return value;
}
//void GPIO_WriteFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
    if(Value == GPIO_PIN_SET)
    {
        pGPIOx->ODR |= (1<<PinNumber);
    }
    else
    {
        pGPIOx->ODR &= ~(1<<PinNumber);
    }
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint8_t Value)
{
    pGPIOx->ODR = Value;
}

//void GPIO_WriteFromInputPort(void);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    pGPIOx->ODR = (pGPIOx->ODR) ^ (1<<PinNumber);
}


/*Interupt Configuration adn ISR handling*/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (IRQNumber <=31)
        {
            // program ISER0 register
            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if ((IRQNumber > 31) && (IRQNumber < 64)) //32 to 63
        {
            // program ISE1 regiter
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        }
        else if ((IRQNumber >= 64) && (IRQNumber <96))
        {
            // program ISE2 regiter
            *NVIC_ISER2 |= (1 << (IRQNumber % 64));
        }
    }
    else
    {
        if (IRQNumber <=31)
        {
            // program ISER0 register
            *NVIC_ICER0 |= (1 << IRQNumber);
        }
        else if ((IRQNumber > 31) && (IRQNumber < 64)) //32 to 63
        {
            // program ISE1 regiter
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
        }
        else if ((IRQNumber >= 64) && (IRQNumber <96))
        {
            // program ISE2 regiter
            *NVIC_ICER2 |= (1 << (IRQNumber % 64));
        }
    }
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
    //uint8_t iprx = IRQNumber/4;
    uint8_t iprx_section = IRQNumber%4;
    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BIT_IMPLEMENTED);

    *(NVIC_PR_BASE_ADDR + IRQNumber) |= (IRQPriority << shift_amount); // because a IRQ is 32 bit (4 byte)
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
    // clear the EXTI PR register corresponding to the pin number
		if (EXTI->PR & (1 << PinNumber))
        {
			EXTI->PR |= (1 << PinNumber);
        }
}

