/*
 * i2c_lcd_universal.c
 *
 *  Created on: 20 mar 2014
 *      Author: Micha³ 'Szopler' Szoplik
 */

#include <avr/io.h>
#include <util/delay.h>

#include <avr/pgmspace.h>
#include <stdlib.h>

#include "../MJS_i2c/i2c_lib.h"
#include "i2c_lcd_universal.h"

extern uint8_t disp_addr;
uint8_t disp_data=0;
uint8_t	disp_ctrl=0;
uint8_t disp_type;

/*************************/
/* Function declarations */
/*************************/
void lcd_cls(void);
void lcde_pulse_en(void);

/************************/
/* Function definitions */
/************************/
void lcd_write (uint8_t Control, uint8_t Data) {
	i2c_start();
	i2c_write(disp_addr);
	if ( disp_type == native ) i2c_write( Control );
	if ( disp_type == expander ) {
		disp_data = Data;
		Data = ( (Data << 4) | (Control & 0x0F) );
	}
	i2c_write( Data );
 	i2c_stop();
}

void lcd_write_byte(uint8_t data_) {
	switch ( disp_type ) {
	case native:
		lcd_write( disp_ctrl, data_ );
		break;
	case expander:
		lcd_write( disp_ctrl, ( data_ >> 4 ) );
		lcde_pulse_en();
		lcd_write( disp_ctrl, ( data_ & 0x0F ) );
		lcde_pulse_en();
		break;
	}
}

void lcd_setorrst_ctrl(uint8_t set_or_rst, uint8_t ctrl_byte) {
	switch ( set_or_rst ) {
	case _Rst: disp_ctrl &= ~(1<<ctrl_byte); break;
	case _Set: disp_ctrl |= (1<<ctrl_byte); break;
	}
	if (disp_type == expander) lcd_write(disp_ctrl, disp_data);
}

/* Needed only for LCD via I2C expander */
void lcde_pulse_en() {
	lcd_setorrst_ctrl(_Set,lcde_en);
	_delay_us(10);
	lcd_setorrst_ctrl(_Rst,lcde_en);
	_delay_us(5);
}
/*--------------------------------------*/

void lcd_switch_rs (uint8_t set_or_rst) {
	if ( disp_type == native ) lcd_setorrst_ctrl(set_or_rst, lcdn_rs);
	if ( disp_type == expander ) lcd_setorrst_ctrl(set_or_rst, lcde_rs);
}

void lcd_write_cmdordata(uint8_t what, uint8_t _byte) {
	switch ( what ) {
	case _cmd:	lcd_switch_rs(0); break;
	case _data:	lcd_switch_rs(1); break;
	}
	lcd_write_byte(_byte);
}


void lcd_init() {
	//i2c_init(100);

	if ( (disp_addr >= 0x78) && (disp_addr <= 0x7F) ) {
		disp_type = native;
		lcd_write_cmdordata(0,0x38);	// 8 bit bus mode, 2 lines, 5x8 font
	}

	if ( (disp_addr >= 0x40) && (disp_addr <= 0x4E) ) {
		disp_type = expander;

		lcd_write(0, 0x03);
		lcde_pulse_en();
		_delay_ms(5);

		lcd_write(0, 0x03);
		lcde_pulse_en();
		_delay_us(100);

		lcd_write(0, 0x03);
		lcde_pulse_en();
		_delay_us(100);

		lcd_write(0, 0x02);
		lcde_pulse_en();
		_delay_us(100);

		lcd_write_cmdordata(0,0x28);
	}

	lcd_write_cmdordata(0,0x0C);		// Display ON, Coursor OFF, Blink OFF
	lcd_write_cmdordata(0,0x06);		// cursor shift to right - DDRAM Address Increase, Shift of the entire display not performed
	lcd_cls();							// clear display
}

uint8_t type_of_display(void) {
	return disp_type;
}

void lcd_OnOff(uint8_t OnOff) {
	if ( OnOff == _Off ) lcd_write_cmdordata(0,0x08); else lcd_write_cmdordata(0,0x0C);
}

void lcd_ctrl(uint8_t ctrl_flag) {
	switch ( ctrl_flag ) {
	case 0:
		lcd_OnOff(_On);
		if ( disp_type == 0 ) lcdn_bl_port &= ~(1<<lcdn_bl_pin); else lcd_setorrst_ctrl(_Off,lcde_bl);
		break;
	case 1:
		lcd_OnOff(_On);
		if ( disp_type == 0 ) lcdn_bl_port |= (1<<lcdn_bl_pin); else lcd_setorrst_ctrl(_On,lcde_bl);
		break;
	case 2:
		lcd_OnOff(_Off);
		if ( disp_type == 0 ) lcdn_bl_port &= ~(1<<lcdn_bl_pin); else lcd_setorrst_ctrl(_Off,lcde_bl);
		break;
	}
}

void lcd_cls() {
	lcd_write_cmdordata (0,0x01);
	_delay_ms(5);
}

void lcd_home() {
	lcd_write_cmdordata(0,0x02);
}

void lcd_locate(uint8_t y, uint8_t x) {
	switch(y) {
	case 0: y = 0x00; break;
	case 1: y = 0x40; break;
	}

	lcd_write_cmdordata(0, 0x80 + y + x);
	//_delay_us(30);
}

void lcd_chr(char ch) {
	lcd_write_cmdordata(1,ch);
}

void lcd_str(char *s) {
	while (*s != 0) {
		lcd_chr(*s);
		s++;
	}
}

void lcd_str_P(const char *str) {
	char chr;
	while ( (chr = pgm_read_byte(str++)) ) lcd_chr(chr);
}

void lcd_num(int val, int type) { // type: 2- binary, 10 - decimal etc.
	char buf[17];
	lcd_str( itoa(val, buf, type) );
}
