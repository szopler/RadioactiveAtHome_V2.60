/*
 * usart.c
 *
 *  Created on: 26 mar 2014
 *      Author: Micha³ 'Szopler' Szoplik
 */


#include <avr/io.h>
#include "usart.h"
#include <avr/interrupt.h>
#include <stdlib.h>


#define TxBuffSize	32						// Must be a power of 2
#define TxBuffMask	(TxBuffSize - 1)		// last sign in TxBuffer should be \0 (ASCII)

volatile char		TxBuff[TxBuffSize];
volatile uint8_t	TxHead;
volatile uint8_t	TxTail;

/*************/
/* Functions */
/*************/

void uart_init( uint16_t baud ) {
	UBRR0H = (uint8_t) (baud>>8);
	UBRR0L = (uint8_t) (baud);

	UCSR0B |= (1<<RXEN0) | (1<<TXEN0);		// | (1<<RXCIE0);
	UCSR0C |= (1<<UCSZ01) | (1<<UCSZ00);	// 8 data bit, 1 stop bit, no partity
}

void uart_putc(char chr) {
	uint8_t temp_pos = ( (TxHead + 1) & TxBuffMask );

    while ( temp_pos == TxTail ) {}

    TxBuff[temp_pos] = chr;
    TxHead = temp_pos;

    UCSR0B |= (1<<UDRIE0);
}

void uart_putstr(char *str) {
	while (*str != 0) uart_putc(*str++);
}

void uart_putnum(int number, uint8_t base) {
	char string[5];
	itoa(number, string, base);
	uart_putstr(string);
}

/***************/
/* Interrupts: */
/***************/

ISR( USART_UDRE_vect ) {
	if ( TxHead != TxTail ) {
		TxTail = ( (TxTail + 1) & TxBuffMask );
    	UDR0 = TxBuff[TxTail];
	} else {
		UCSR0B &= ~(1<<UDRIE0);
	}
}
