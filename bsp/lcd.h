/*
 * lcd.h
 *
 *  Created on: May 24, 2025
 *      Author: ASUS
 */

#ifndef LCD_H_
#define LCD_H_

#include <stm32f407xx.h>
#define LCD_GPIO_PORT   GPIOD
#define LCD_GPIO_RS     GPIO_PIN_NO_0
#define LCD_GPIO_RW     GPIO_PIN_NO_1
#define LCD_GPIO_EN     GPIO_PIN_NO_2
#define LCD_GPIO_D4     GPIO_PIN_NO_4
#define LCD_GPIO_D5     GPIO_PIN_NO_5
#define LCD_GPIO_D6     GPIO_PIN_NO_6
#define LCD_GPIO_D7     GPIO_PIN_NO_7

/*LCD command*/
#define LCD_CMD_4DL_2N_5X8F     0X28
#define LCD_CMD_DON_CURON       0x0E
#define LCD_CMD_DIS_CLEAR       0x01
#define LCD_CMD_DIS_RETURN_HOME 0x02
#define LCD_CMD_INCADD          0x06


void lcd_init(void);
void lcd_send_command(uint8_t cmd);
void lcd_print_char(uint8_t data);
void lcd_display_clear(void);
void mdelay(uint32_t cnt);
void udelay(uint32_t cnt);
void lcd_print_string(char *message);
void lcd_display_return_home(void);
void lcd_print_string(char *message);

#endif /* LCD_H_ */
