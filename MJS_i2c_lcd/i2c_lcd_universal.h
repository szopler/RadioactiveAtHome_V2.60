/*
 * i2c_lcd_universal.h
 *
 *  Created on: 20 mar 2014
 *      Author: Micha³ 'Szopler' Szoplik
 */

#ifndef I2C_LCD_UNIVERSAL_H_
#define I2C_LCD_UNIVERSAL_H_

#define _Set	1
#define _Rst	0

#define _On		1
#define _Off	0

#define native		0
#define expander	1

#define _cmd		0
#define _data		1

#define lcdn_bl_port	PORTB
#define lcdn_bl_pin		PB1
#define lcdn_bl_ddr		DDRB
#define lcdn_rs		6

#define lcde_bl		3
#define lcde_en		2
#define lcde_rw		1
#define lcde_rs		0

extern uint8_t disp_addr;

void lcd_init();
void lcd_chr(char ch);
void lcd_str(char* s);
void lcd_home(void);
void lcd_locate(uint8_t y, uint8_t x);
void lcd_num(int val, int type);
void lcd_str_P(const char *str);

uint8_t type_of_display(void);
void lcd_ctrl(uint8_t ctrl_flag);

#endif /* I2C_LCD_UNIVERSAL_H_ */
