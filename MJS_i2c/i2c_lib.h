/* Name: i2c_lib.h
 * Project: Radioactive@Home KIT Firmware
 * Author: Michal 'Szopler' Szoplik
 * Creation Date: 04-2014
 * License: GNU GPL v3
 */


#ifndef I2C_LIB_H_
#define I2C_LIB_H_

void i2c_init(uint16_t bitrate);
void i2c_start(void);
void i2c_write(uint8_t data);
void i2c_stop(void);
uint8_t i2c_status(void);


#endif /* I2C_TWI_H_ */
