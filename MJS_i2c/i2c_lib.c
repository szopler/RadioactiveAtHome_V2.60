/* Name: i2c_lib.c
 * Project: Radioactive@Home KIT Firmware
 * Author: Michal 'Szopler' Szoplik
 * Creation Date: 04-2014
 * License: GNU GPL v3
 */


#include <avr/io.h>
#include "../MJS_i2c/i2c_lib.h"

void i2c_init(uint16_t bitrate) {
	uint8_t divider;

	divider = ( (F_CPU/1000UL) / bitrate );
	if(divider >= 16) divider = (divider-16) / 2;

	TWBR = divider;
}

void i2c_start(void) {
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	while ( !(TWCR&(1<<TWINT)) );
}

void i2c_write(uint8_t data) {
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);
	while ( !(TWCR&(1<<TWINT)) );
}

void i2c_stop(void) {
	TWCR |= (1<<TWINT) | (1<<TWSTO);
	while ( (TWCR&(1<<TWSTO)) );
}

uint8_t i2c_status(void) {
    uint8_t status;
    status = (TWSR & 0xF8);
    return status;
}

