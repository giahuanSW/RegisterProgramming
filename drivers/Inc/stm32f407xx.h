/*
 * stm32f407xx.h
 *
 *  Created on: Oct 13, 2024
 *      Author: ASUS
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#define __vo                    volatile


/*
* ARM Cortex Mx progressor NVIC ISERx register Addresses
*/
#define NVIC_ISER0              ((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1              ((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2              ((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3              ((__vo uint32_t*)0xE000E10C)

/*
* ARM Cortex Mx progressor NVIC ICERx register Addresses
*/
#define NVIC_ICER0              ((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1              ((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2              ((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3              ((__vo uint32_t*)0xE000E18C)

/*
* ARM Cortex Mx Processor Priority Register Address Calculation
*/
#define NVIC_PR_BASE_ADDR       ((__vo uint32_t*)0xE000E400)

#define NO_PR_BIT_IMPLEMENTED   4
/*
* base addresses of Flash anh SRAM memories
*/
#define FLASH_BASEADDR         0x08000000U
#define SRAM1_BASEADDR         0x20000000U
#define SRAM2_BASEADDR         0x2001C000U
#define ROM_BASEADDR           0x1FFF0000U
#define SRAM                   SRAM1_BASEADDR


/*AHBx and APBx*/

#define PERIPH_BASE            0x40000000U
#define APB1PERIPH_BASE        PERIPH_BASE
#define APB2PERIPH_BASE        0x40010000U
#define AHB1PERIPH_BASE        0x40020000U
#define AHB2PERIPH_BASE        0x50000000U


/* 
    Base address of peripherals which are hanging on AHB1 bus
*/

#define GPIOA_BASEADDR         (AHB1PERIPH_BASE + 0x0000U)
#define GPIOB_BASEADDR         (AHB1PERIPH_BASE + 0x0400U)
#define GPIOC_BASEADDR         (AHB1PERIPH_BASE + 0x0800U)
#define GPIOD_BASEADDR         (AHB1PERIPH_BASE + 0x0C00U)
#define GPIOE_BASEADDR         (AHB1PERIPH_BASE + 0x1000U)
#define GPIOF_BASEADDR         (AHB1PERIPH_BASE + 0x1400U)
#define GPIOG_BASEADDR         (AHB1PERIPH_BASE + 0x1800U)
#define GPIOH_BASEADDR         (AHB1PERIPH_BASE + 0x1C00U)
#define GPIOI_BASEADDR         (AHB1PERIPH_BASE + 0x2000U)

#define RCC_BASEADDR           (AHB1PERIPH_BASE + 0x3800U)
/* 
    Base address of peripherals which are hanging on APB1 bus
*/

#define I2C1_BASE               (APB1PERIPH_BASE + 0x5400U)
#define I2C2_BASE               (APB1PERIPH_BASE + 0x5800U)
#define I2C3_BASE               (APB1PERIPH_BASE + 0x5C00U)

#define SPI2_BASE               (APB1PERIPH_BASE + 0x3800U)
#define SPI3_BASE               (APB1PERIPH_BASE + 0x3C00U)

#define USART2_BASE             (APB1PERIPH_BASE + 0x4400U)
#define USART3_BASE             (APB1PERIPH_BASE + 0x4800U)
#define UART4_BASE              (APB1PERIPH_BASE + 0x4C00U)
#define UART5_BASE              (APB1PERIPH_BASE + 0x5000U)

/* 
    Base address of peripherals which are hanging on APB2 bus
*/

#define EXTI_BASEADDR           (APB2PERIPH_BASE + 0x3C00U)
#define SPI1_BASEADDR           (APB2PERIPH_BASE + 0x3000U)
#define SYSCFG_BASEADDR         (APB2PERIPH_BASE + 0x3800U)
#define USART1_BASEADDR         (APB2PERIPH_BASE + 0x1000U)
#define USART6_BASEADDR         (APB2PERIPH_BASE + 0x1400U)



/*********************************************************************************** */
// variable declare

/*
* peripheral register definition structure for GPIO
*/
typedef struct 
{
    __vo uint32_t MODER;                 // GPIO port mode register
    __vo uint32_t OTYPER;                // GPIO port output type register
    __vo uint32_t OSPEEDR;               // GPIO port output speed register
    __vo uint32_t PUPDR;                 // GPIO port pull-up/pull-down register
    __vo uint32_t IDR;                   // GPIO port input data register
    __vo uint32_t ODR;                   // GPIO port output data register
    __vo uint32_t BSRR;                  // GPIO port bit set/reset register
    __vo uint32_t LCKR;                  // GPIO port configuration lock register
    __vo uint32_t AFR[2];               // GPIO alternate function AFRL[0] low/AFRL[1] high register
}GPIO_RegDef_t;

/*
* peripheral register definition structure for RCC
*/
typedef struct 
{
    __vo uint32_t CR;
    __vo uint32_t PLLCFGR;
    __vo uint32_t CFGR;
    __vo uint32_t CIR;
    __vo uint32_t AHB1RSTR;
    __vo uint32_t AHB2RSTR;
    __vo uint32_t AHB3RSTR;
     uint32_t RESERVED0;
    __vo uint32_t APB1RSTR;
    __vo uint32_t APB2RSTR;
     uint32_t RESERVED1[2];
    __vo uint32_t AHB1ENR;
    __vo uint32_t AHB2ENR;
    __vo uint32_t AHB3ENR;
    uint32_t RESERVED2;
    __vo uint32_t APB1ENR;
    __vo uint32_t APB2ENR;
     uint32_t RESERVED3[2];
    __vo uint32_t AHB1LPENR;
    __vo uint32_t AHB2LPENR;
    __vo uint32_t AHB3LPENR;
     uint32_t RESERVED4;
    __vo uint32_t APB1LPENR;
    __vo uint32_t APB2LPENR;
     uint32_t RESERVED5[2];
    __vo uint32_t BDCR;
    __vo uint32_t CSR;
     uint32_t RESERVED6[2];
    __vo uint32_t SSCGR;
    __vo uint32_t PLLI2SCFGR;
    __vo uint32_t PLLSAICFGR;
    __vo uint32_t DCKCFGR;
    __vo uint32_t CKGATENR;
    __vo uint32_t DCKCFGR2;
}RCC_RegDef_t;

/*
* peripheral register definition structure for EXTI
*/

typedef struct
{
    __vo uint32_t IRM;
    __vo uint32_t ERM;
    __vo uint32_t RTSR;
    __vo uint32_t FTSR;
    __vo uint32_t SWIER;
    __vo uint32_t PR;
}EXTI_RegDef_t;

typedef struct
{
    __vo uint32_t MEMRMP;
    __vo uint32_t PMC;
    __vo uint32_t EXTICR[4];
    uint32_t RESERVED1[2];
    __vo uint32_t CMPCR;
    uint32_t RESERVED2[2];
    __vo uint32_t CFGR;
}SYSCFG_RegDef_t;


#define GPIOA               ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB               ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC               ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD               ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE               ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF               ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG               ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH               ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI               ((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC                 ((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI                ((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG              ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)
/*config clock GPIO*/
#define GPIOA_PCLK_EN()   (RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()   (RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()   (RCC->AHB1ENR |= (1<<2))  
#define GPIOD_PCLK_EN()   (RCC->AHB1ENR |= (1<<3))  
#define GPIOE_PCLK_EN()   (RCC->AHB1ENR |= (1<<4))  
#define GPIOF_PCLK_EN()   (RCC->AHB1ENR |= (1<<5))  
#define GPIOG_PCLK_EN()   (RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()   (RCC->AHB1ENR |= (1<<7))      
#define GPIOI_PCLK_EN()   (RCC->AHB1ENR |= (1<<8))

/*config clock I2C*/
#define I2C1_PCLK_EN()    (RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()    (RCC->APB1ENR |= (1<<21))
#define I2C3_PCLK_EN()    (RCC->APB1ENR |= (1<<21))

/*config clock SPI*/
#define SPI1_PCLK_EN()    (RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()    (RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()    (RCC->APB1ENR |= (1<<15))
#define SPI4_PCLK_EN()    (RCC->APB2ENR |= (1<<13))

/*config clock UART*/
#define USART2_PCLK_EN()    (RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()    (RCC->APB1ENR |= (1<<18))
#define UART4_PCLK_EN()     (RCC->APB1ENR |= (1<<19))
#define UART5_PCLK_EN()     (RCC->APB1ENR |= (1<<20))

/*config clock SYSCFG*/
#define SYSCFG_PCLK_EN()    (RCC->APB2ENR |= (1<<14))

#define GPIOA_PCLK_DI()


/*
*   IRQ(interrupt Request) Numbers of STM32F407x MCU
*   NOTE: update these macros with valid values according to your MCU
*   TODO: You may complete this list for other peripherals
*/
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40


//some generic macros
#define ENABLE  1
#define DISABLE 0
#define SET     ENABLE
#define RESET   DISABLE
#define GPIO_PIN_SET    SET
#define GPIO_PIN_RESET  RESET


/*
*  Macros to reset GPIOx peripherals
*/
#define GPIOA_REG_RESET()       do{                              \
                                    (RCC->AHB1RSTR |= (1 << 0)); \
                                    (RCC->AHB1RSTR &= ~(1 << 0));\
                                }                                \
                                while(0)

#define GPIOB_REG_RESET()       do{                             \
                                    (RCC->AHB1RSTR |= (1 << 1)); \
                                    (RCC->AHB1RSTR &= ~(1 << 1));\
                                }                                \
                                while(0)

#define GPIOC_REG_RESET()       do{                             \
                                    (RCC->AHB1RSTR |= (1 << 2)); \
                                    (RCC->AHB1RSTR &= ~(1 << 2));\
                                }                                \
                                while(0)

#define GPIOD_REG_RESET()       do{                              \
                                    (RCC->AHB1RSTR |= (1 << 3)); \
                                    (RCC->AHB1RSTR &= ~(1 << 3));\
                                }                                \
                                while(0)

#define GPIOE_REG_RESET()       do{                              \
                                    (RCC->AHB1RSTR |= (1 << 4)); \
                                    (RCC->AHB1RSTR &= ~(1 << 4));\
                                }                                \
                                while(0)

#define GPIOF_REG_RESET()       do{                              \
                                (RCC->AHB1RSTR |= (1 << 5)); \
                                    (RCC->AHB1RSTR &= ~(1 << 5));\
                                }                                \
                                while(0)

#define GPIOG_REG_RESET()       do{                              \
                                    (RCC->AHB1RSTR |= (1 << 6)); \
                                    (RCC->AHB1RSTR &= ~(1 << 6));\
                                }                                \
                                while(0)

#define GPIOH_REG_RESET()       do{                              \
                                    (RCC->AHB1RSTR |= (1 << 7)); \
                                    (RCC->AHB1RSTR &= ~(1 << 7));\
                                }                                \
                                while(0)

#define GPIOI_REG_RESET()        do{                              \
                                    (RCC->AHB1RSTR |= (1 << 8)); \
                                    (RCC->AHB1RSTR &= ~(1 << 8));\
                                }                                \
                                while(0)


#define GPIO_BASEADDR_TO_CODE(x)    ( (x == GPIOA) ? 0: \
                                    (x == GPIOB) ? 1:   \
                                    (x == GPIOC) ? 2:   \
                                    (x == GPIOD) ? 3:   \
                                    (x == GPIOE) ? 4:   \
                                    (x == GPIOF) ? 5:   \
                                    (x == GPIOG) ? 6:   \
                                    (x == GPIOH) ? 7:   \
                                    (x == GPIOI) ? 8: 0 )

#endif /* INC_STM32F407XX_H_ */
