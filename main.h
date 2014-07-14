/* Name: main.h
 * Project: Radioactive@Home KIT Firmware
 * Author: Michal 'Szopler' Szoplik
 * Creation Date: 04-2014
 * License: GNU GPL v3
 */


#ifndef MAIN_H_
#define MAIN_H_


/* Macros */
#define sbi(byte,bit) byte|=(1<<bit)
#define cbi(byte,bit) byte&=~(1<<bit)
#define bit(byte,n) (byte>>n)&1

#define _Set	1
#define _Rst	0

#define _On		1
#define _Off	0

#define buz_port	PORTD
#define buz_pin		PD7
#define led_port	PORTD
#define led_pin		PD6


#endif /* MAIN_H_ */
