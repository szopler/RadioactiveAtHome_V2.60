/* Name: usart.h
 * Project: Radioactive@Home KIT Firmware
 * Author: Michal 'Szopler' Szoplik
 * Creation Date: 04-2014
 * License: GNU GPL v3
 */


#ifndef USART_H_
#define USART_H_

#define BAUDRATE	9600UL
#define __UBRR		( F_CPU  + BAUDRATE * 8UL  ) / (16UL * BAUDRATE) -1


void uart_init(uint16_t baud);
void uart_putc(char chr);
void uart_putstr(char *str);
void uart_putnum(int number, uint8_t base);

#endif /* USART_H_ */
