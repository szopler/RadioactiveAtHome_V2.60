/*
 * i2c_lib.h
 *
 *  Created on: 2014-03-17
 *       Author: Micha³ 'Szopler' Szoplik
 */

#ifndef I2C_LIB_H_
#define I2C_LIB_H_

void i2c_init(uint16_t bitrate);
void i2c_start(void);
void i2c_write(uint8_t data);
void i2c_stop(void);
uint8_t i2c_status(void);


#endif /* I2C_TWI_H_ */
