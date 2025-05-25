/*
 * lcd.c
 *
 *  Created on: May 24, 2025
 *      Author: ASUS
 */
#include "lcd.h"
static void write_4_bits(uint8_t value);
static void lcd_enable(void);

void lcd_send_command(uint8_t cmd)
{
    /* RS = 0 for LCd command*/
    GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_RS,GPIO_PIN_RESET);

    /* R/nW = 0, for write*/
    GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_RW,GPIO_PIN_RESET);

    write_4_bits(cmd >> 4);
    write_4_bits(cmd & 0x0F);

}

/*
* This function sends a charater to the LCd
* Here we used 4 bit parallel data transmission
* First higher nibble of the data will be sent on to the data lines D4, D5, D6, D7
* Then lower nibble the data will be set on to the data lines D4, D5, D6, D7
*/
void lcd_print_char(uint8_t data)
{
    /* RS = 1 for LCd user data*/
    GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_RS,GPIO_PIN_SET);

    /* R/nW = 0, for write*/
    GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_RW,GPIO_PIN_RESET);

    write_4_bits(data >> 4);   /*Higher nibble*/
    write_4_bits(data & 0x0F); /*Lower nibble*/
}

void lcd_print_string(char *message)
{
    do
    {
        lcd_print_char((uint8_t)*message++);
    }
    while (*message != '\0');
}


void lcd_display_return_home(void)
{
    lcd_send_command(LCD_CMD_DIS_RETURN_HOME);
    //check datasheet
    mdelay(2);
}

/*
 * set lcd to a specified location given by row and column information
 * Row Number (1 to 2)
 * Column Number (1 to 16) Assuming a 2 x 16 characters display
*/
void lcd_set_cursor(uint8_t row, uint8_t column)
{
    column--;
    switch(row)
    {
        case 1:
        /* Set cursor to 1st row address and add index*/
            lcd_send_command((column |= 0x80));
        break;
        case 2:
        /* Set cursor to 2nd row address and add index*/
            lcd_send_command((column |= 0xC0));
        break;
        default:
        break;
    }
}

void lcd_init(void)
{
    GPIO_Handle_t lcd_signal;
    //1. configure the gpio pins which are used for lcd connections
    lcd_signal.pGPIOx = LCD_GPIO_PORT;
    lcd_signal.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RS;
    lcd_signal.GPIO_PinConfig.GPIO_PinOptype = GPIO_OP_TYPE_PP;
    lcd_signal.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    lcd_signal.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIO_Init(&lcd_signal);
    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RW;
    GPIO_Init(&lcd_signal);
    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_EN;
    GPIO_Init(&lcd_signal);
    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D4;
    GPIO_Init(&lcd_signal);
    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D5;
    GPIO_Init(&lcd_signal);
    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D6;
    GPIO_Init(&lcd_signal);
    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D7;
    GPIO_Init(&lcd_signal);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_RS,GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_RW,GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_EN,GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_D4,GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_D5,GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_D6,GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_D7,GPIO_PIN_RESET);
    // follow flow chart in datasheet of LCD
    //2. do the LCD initialization
    mdelay(40);

    /*RS =0, For LCD command*/
    GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_RS,GPIO_PIN_RESET);

    /*RnW = 0, Writing to LCD*/
    GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_RW,GPIO_PIN_RESET);
    write_4_bits(0x3);
    mdelay(5);   
    write_4_bits(0x3);
    udelay(150);
    write_4_bits(0x3);

    write_4_bits(0x2);

    //function set command
   lcd_send_command(LCD_CMD_4DL_2N_5X8F);

   //display ON and cursor ON
   lcd_send_command(LCD_CMD_DON_CURON);


   //display clear command
   lcd_display_clear();

    //entry mode command set
    lcd_send_command(LCD_CMD_INCADD);
}

void lcd_display_clear(void)
{
    lcd_send_command(LCD_CMD_DIS_CLEAR);
    mdelay(2);
}
/*writes 4 bits of data/command on to D4,D5,D6,D7 lines*/
static void write_4_bits(uint8_t value)
{
    GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_D4,(value >> 0) & 0x1);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_D5,(value >> 1) & 0x1);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_D6,(value >> 2) & 0x1);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_D7,(value >> 3) & 0x1);
    
    lcd_enable();
}

static void lcd_enable(void)
{
    GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_EN,GPIO_PIN_SET);
    udelay(10);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_EN,GPIO_PIN_RESET);
    udelay(100);
}

void mdelay(uint32_t cnt)
{
    for(uint32_t i = 0; i< (cnt *1000);i++);
}

void udelay(uint32_t cnt)
{
    for(uint32_t i = 0; i< (cnt *1);i++);
}
