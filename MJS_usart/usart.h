/*
 * usart.h
 *
 *  Created on: 26 mar 2014
 *      Author: Micha³ 'Szopler' Szoplik
 */

#ifndef USART_H_
#define USART_H_

#define BAUDRATE	9600UL
#define __UBRR		(F_CPU/(16UL*BAUDRATE))-1


void uart_init(uint16_t baud);
void uart_putc(char chr);
void uart_putstr(char *str);
void uart_putnum(int number, uint8_t base);

#endif /* USART_H_ */
