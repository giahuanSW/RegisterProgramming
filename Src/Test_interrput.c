/*
 * Test_interrput.c
 *
 *  Created on: Nov 3, 2024
 *      Author: ASUS
 */

#include <string.h>
#include <stdint.h>
#include <stm32f407xx.h>
#include <stm32f407xx_gpio_driver.h>

#define HIGH                1
#define LOW                 0
#define BTN_PRESSED_LOW     LOW
void delay(void)
{
    for(uint32_t i=0;i<500000/2;i++);
}
void EXTI0_IRQHandler(void)
{
	delay();
    GPIO_IRQHandling(GPIO_PIN_NO_0);
    GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
}

int main(void)
{
    GPIO_Handle_t GpioLed,GPIOBtn;
    memset(&GpioLed,0,sizeof(GpioLed));
    memset(&GPIOBtn,0,sizeof(GPIOBtn));
    GpioLed.pGPIOx = GPIOD;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOptype = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    GPIO_PeriClockControl(GPIOD,ENABLE);
    GPIO_Init(&GpioLed);

    GPIOBtn.pGPIOx = GPIOA;
    GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
    GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    GPIO_PeriClockControl(GPIOA,ENABLE);
    GPIO_Init(&GPIOBtn);

    GPIO_IRQPriorityConfig(IRQ_NO_EXTI0,15);
    GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);
    while(1)
    {

    }
}
