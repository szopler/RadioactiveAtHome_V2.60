/* Name: main.c
 * Project: Radioactive@Home KIT Firmware
 * Author: Michal 'Szopler' Szoplik
 * Creation Date: 04-2014
 * License: GNU GPL v3
 */


#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
//#include <avr/eeprom.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "usbdrv/usbdrv.h"
#include "MJS_i2c_lcd/i2c_lcd_universal.h"
#include "MJS_i2c/i2c_lib.h"
#include "MJS_usart/usart.h"

#include "main.h"

#define Ilosc_pomiarow	35

volatile uint8_t 	pulse_counter;
volatile uint32_t	cumul_counter;
volatile uint32_t	uptime;

volatile uint16_t 	HV_Value;

volatile uint16_t	Timer_10ms, Timer_100ms, Timer_1s;

uint8_t port_state, prev_port_state = 0x0E;

struct {
	uint8_t		LCDExist		:1;
    uint8_t		IndicatorChoice	:2;
    uint8_t		Indicate		:1;
    uint8_t		KeybCheck		:1;
    uint8_t		LCDControl		:2;
    uint8_t		DisplayRefresh	:1;
} volatile _FLAG;

char	FirstLine[17] = "Radioactive@Home";
char	SecondLine[17] = "--- 2.60 ---";

uint8_t disp_addr;

/*-------------------------*/
/* USB interface functions */
/*------------------------ */

PROGMEM const char usbHidReportDescriptor[22] = {	/* USB report descriptor */
    0x06, 0x00, 0xff,				// USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                    	// USAGE (Vendor Usage 1)
    0xa1, 0x01,                    	// COLLECTION (Application)
    0x15, 0x00,                    	// LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,				// LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    	// REPORT_SIZE (8)
    0x95, 0x80,						// REPORT_COUNT (128)=0x80
    0x09, 0x00,                    	// USAGE (Undefined)
    0xb2, 0x02, 0x01,              	// FEATURE (Data,Var,Abs,Buf)
    0xc0                           	// END_COLLECTION
};
/* Since we define only one feature report, we don't use report-IDs (which
   would be the first byte of the report). The entire report consists of 128
   opaque data bytes.*/

/* The following variables store the status of the current data transfer */
static uchar    currentAddress;
static uchar    bytesRemaining;
//static uchar mem = 0;

uchar usbFunctionRead(uchar *data, uchar len) {
/* usbFunctionRead() is called when the host requests a chunk of data from the device.
   For more information see the documentation in usbdrv/usbdrv.h.*/

    if(len > bytesRemaining) len = bytesRemaining;
    /*eeprom_read_block(data, (uchar *)0 + currentAddress, len);
    currentAddress += len;
    bytesRemaining -= len;*/

	data[0] = *(((uchar*) &uptime) + 0);
	data[1] = *(((uchar*) &uptime) + 1);
	data[2] = *(((uchar*) &uptime) + 2);
	data[3] = *(((uchar*) &uptime) + 3);
	data[4] = *(((uchar*) &cumul_counter) + 0);
	data[5] = *(((uchar*) &cumul_counter) + 1);
	data[6] = *(((uchar*) &cumul_counter) + 2);
	data[7] = *(((uchar*) &cumul_counter) + 3);
	
	bytesRemaining = 0;
    return len;
}

uchar   usbFunctionWrite(uchar *data, uchar len) {
/* usbFunctionWrite() is called when the host sends a chunk of data to the device. For more information see the documentation in usbdrv/usbdrv.h */

    /*if(bytesRemaining == 0)
        return 1;               // end of transfer
    if(len > bytesRemaining)
        len = bytesRemaining;
    //eeprom_write_block(data, (uchar *)0 + currentAddress, len);
    currentAddress += len;
    bytesRemaining -= len;
    return bytesRemaining == 0; // return 1 if this was the last chunk
	//mem = *data;*/
	switch(data[0]) {
		// case 0xa0: sbi(PORTB, GM_SUPPLY); break; // V2.53(?)
		// case 0xa1: cbi(PORTB, GM_SUPPLY); break; // V2.53(?)
		case 0xa2: TCNT1 = 0; uptime = 0; pulse_counter = 0; cumul_counter = 0; break;
		case 0x10: _FLAG.LCDControl = 0; break;				// LCD on, BL off
		case 0x11: _FLAG.LCDControl = 1; break;				// LCD on, BL on
		case 0x12: _FLAG.LCDControl = 2; break;				// New: LCD off, BL off
		// case 0x20: lcd_locate(0,0); break; // V1.00
		// case 0x21: lcd_locate(1,0); break; // V1.00
		// case 0x30: lcd_char(data[1]); break; // V1.00
		case 0x40: _FLAG.IndicatorChoice = 0; break;		// All indicators OFF
		case 0x41: _FLAG.IndicatorChoice = 1; break;		// Buzzer ON
		case 0x42: _FLAG.IndicatorChoice = 2; break;		// New: Buzzer ON, Led ON
		case 0x43: _FLAG.IndicatorChoice = 3; break;		// New: Buzzer OFF, Led ON
	}
	return 1;
}

usbMsgLen_t usbFunctionSetup(uchar data[8]) {
usbRequest_t    *rq = (void *)data;

    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* HID class request */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            /* since we have only one report type, we can ignore the report-ID */
            bytesRemaining = 8;
            currentAddress = 0;
            return USB_NO_MSG;  /* use usbFunctionRead() to obtain data */
        }else if(rq->bRequest == USBRQ_HID_SET_REPORT){
            /* since we have only one report type, we can ignore the report-ID */
            bytesRemaining = 8;
            currentAddress = 0;
            return USB_NO_MSG;  /* use usbFunctionWrite() to receive data from host */
        }
    }else{
        /* ignore vendor type requests, we don't use any */
    }
    return 0;
}


/*----------------*/
/* Initialization */
/*----------------*/

void _INIT(void) {

	// Configure Timer1 as TIMER for system time counting
    TCCR1B = (1 << WGM12) | (0 << CS12) | (1 << CS11) | (0 << CS10);	// CTC Mode & 20MHz /8 = 2.5MHz
    OCR1A = 2499;														// Compare Interrupt every 2500/2.5MHz = 1ms (1kHz)
    TIMSK1 |= (1 << OCIE1A);											// CTC from register A

    // Configure Timer2 as PWM source
    TCCR2A = (1 << COM2B1) | (0 << COM2B0);								// Clear OC0B on Compare Match, set OC0B at BOTTOM (non-inverting mode)
    TCCR2A |= (0 << WGM22) | (0 << WGM21) | (1 << WGM20);				// Phase Correct Mode ; (FastPWM:256 PhaseCorrect:510)
    TCCR2B = (0 << CS22) | (0 << CS21) | (1 << CS20);					// 20MHz /(1*510) -> fpwm = 39.215kHz
    OCR2B = 0;															// PWM 0% (HV off)
    sbi(DDRD,3);														// PD3/OC2B as output

    // Configure ADC for HV control
    ADMUX = (1 << REFS1) | (1 << REFS0);								// Internal 1.1V Voltage reference
    //ADMUX |= (0 << MUX3) | (0 << MUX2) | (0 << MUX1) | (0 << MUX0);		// ADC Channel 0
    DIDR0 = (1 << ADC0D);												// Digital function of PC0 disabled
    ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);				// 20MHz /128 = 156.25kHz
    ADCSRA |= (1 << ADEN) | (1 << ADIE);								// ADC Enable ; ADC Interrupt Enable

    // Configure input for PULSEs
    cbi(DDRB,0);														// PB2 as input for PULSE's
    sbi(PORTB,0);														// PB2 pullup on
    PCICR |= (1<<PCIE0);												// Enable Pin Change Interrupts for PCI0..PCI7
    PCMSK0 = (1<<PCINT0);												// Enable Interrupt from PCI0 (PB0)

    // Configure input's for switches
    cbi(DDRC, 1);														// PC1 as input for SW
    cbi(DDRC, 2);														// PC2 as input for SW
    cbi(DDRC, 3);														// PC3 as input for SW
    sbi(PORTC, 1);														// PC1 pullup on
    sbi(PORTC, 2);														// PC2 pullup on
    sbi(PORTC, 3);														// PC3 pullup on
    PCICR |= (1<<PCIE1);												// Enable Pin Change Interrupts for PCI8...PCI15
    PCMSK1 = (1<<PCINT9) | (1<<PCINT10) | (1<<PCINT11);					// Enable Interrupt from PCI9, 10, 11 (PC1..PC3)

    // Configure output's for pulse indicators
    sbi(DDRD,6);														// PD6 as output for LED
    cbi(PORTD,6);														// LED off
    sbi(DDRD,7);														// PD7 as output for Buzzer
    cbi(PORTD,7);														// Buzzer off

    // Configure output for LCD backlight
    sbi(DDRB,1);														// PB1 as output for backlight
    cbi(PORTB,1);														// Backlight off

    // Configure unused pins for higher noise suppression
    sbi(DDRB,2);														// PB1 as output
    sbi(DDRB,3);														// PB2 as output
    sbi(DDRB,4);														// PB3 as output
    sbi(DDRB,5);														// PB4 as output
    cbi(PORTB,2);														// PB2 = Low
    cbi(PORTB,3);														// PB3 = Low
    cbi(PORTB,4);														// PB4 = Low
    cbi(PORTB,5);														// PB5 = Low

    _FLAG.IndicatorChoice = 3;											// LED only
    _FLAG.LCDControl = 1;												// LCD On, BL On

    // I2C initialization and display searching
    i2c_init(100);

	for(uint8_t adres=0x40;adres<=0x7F;adres++) {
	    uint8_t status;

	    i2c_start();
		i2c_write(adres);
		status = i2c_status();
		i2c_stop();
		_delay_us(100);

		if ( status == 0x18) {
			disp_addr = adres;
			_FLAG.LCDExist = _Set;
			break;
		} else {
			_FLAG.LCDExist = _Rst;
		}
	}

    if ( _FLAG.LCDExist == _Set ) {
    	lcd_init();
    	lcd_ctrl(_FLAG.LCDControl);
    }
}

/*------------------------------*/
/* Interrupt Vectors			*/
/*------------------------------*/

ISR(TIMER1_COMPA_vect) {
	uint16_t T;

    uptime++;

    T = Timer_10ms;
    if (T) Timer_10ms = --T;
    T = Timer_100ms;
    if (T) Timer_100ms = --T;
    T = Timer_1s;
    if (T) Timer_1s = --T;
}

ISR(ADC_vect) {
    HV_Value = (ADCW * 652UL) /1000;
    if ( (HV_Value < 400) && (OCR2B < 127) ) OCR2B++;
    if ( (HV_Value > 400) && (OCR2B > 0) ) OCR2B--;
}

ISR(PCINT0_vect) {
    if ( bit_is_clear(PINB,0) ) {
	pulse_counter++;
	cumul_counter++;
	_FLAG.Indicate = _Set;
    }
}

ISR(PCINT1_vect) {
	_FLAG.KeybCheck = _Set;
}

/*------------------------------*/
/* Radioactive@Home functions	*/
/*------------------------------*/

static void HV_Supply_EVENT(void) {
    if (!Timer_10ms) {								// Every 0.01s (100Hz)
    	Timer_10ms = 10;
    	ADCSRA |= (1 << ADSC);						// Start ADC conversion
    }
}

static void Switch_EVENT(void) {
	if (_FLAG.KeybCheck == _Set) {

		port_state = ( PINC & 0x0E );

        switch ( prev_port_state & (~port_state) ) {
        case 2:
        	_FLAG.IndicatorChoice++;
        break;
        case 4:
        	_FLAG.LCDControl--;
        	if ( _FLAG.LCDControl > 2 ) _FLAG.LCDControl = 2;
        break;
        case 8:

        break;
        }
		prev_port_state = port_state;
		_FLAG.KeybCheck = _Rst;
	}
}

static void RadCalc_EVENT(void) {
	if (!Timer_1s) {								// Every 1s (1Hz)
		Timer_1s = 1000;

		if ( _FLAG.LCDExist == _Set ) lcd_ctrl(_FLAG.LCDControl);

		static uint16_t table[Ilosc_pomiarow];
		static uint8_t index;
		uint32_t sr1=0;
		ldiv_t result;

		table[index++] = pulse_counter;
		pulse_counter=0;
		if ( index >= Ilosc_pomiarow ) index=0;

		for (uint8_t i=0;i<Ilosc_pomiarow;i++) sr1 += table[i];

		result = ldiv(sr1,100);

		sprintf(SecondLine, " %lu.%02lu uSv/h", result.quot, result.rem);

		uart_putnum(sr1,10);
		uart_putstr("\r\n");
	}
}

/*void BeepIfChange_EVENT(void) {
	static uint32_t temp_time;

	if ( _FLAG.Indicate == _Set ) {
		_FLAG.Indicate = _Rst;
		temp_time = uptime;

		switch ( _FLAG.IndicatorChoice ) {
		case 1: sbi(buz_port,buz_pin); break;
		case 2: sbi(led_port,led_pin); sbi(buz_port,buz_pin); break;
		case 3: sbi(led_port,led_pin); break;
		}
	}

	if ( uptime >= (temp_time+5UL) ) {
		cbi(buz_port,buz_pin);
		cbi(led_port,led_pin);
	}
}*/

static void BeepIfChange_EVENT(void) {
	if ( _FLAG.Indicate == _Set ) {
		_FLAG.Indicate = _Rst;

		switch ( _FLAG.IndicatorChoice ) {
		case 1: sbi(buz_port,buz_pin); break;
		case 2: sbi(led_port,led_pin); sbi(buz_port,buz_pin); break;
		case 3: sbi(led_port,led_pin); break;
		}

		_delay_ms(5);
		cbi(buz_port,buz_pin);
		cbi(led_port,led_pin);
	}
}

inline uint8_t length(char *s) {
    uint8_t len = 0;
    while(*(s++)) len++;

    return len;
}

void DisplayRefresh(uint8_t Refresh) {
	if ( (Refresh == 1) && (_FLAG.LCDExist == _Set) ) {

		lcd_locate(0,0);
		lcd_str(FirstLine);

		//lcd_locate(1,0);
		lcd_locate( 1, (8-(length(SecondLine)/2)) );
		lcd_str(SecondLine);
	}
}

static void DisplayRefresh_EVENT(void) {
	if (!Timer_100ms) {
		Timer_100ms = 100;							// Every 0.1s (10Hz)
		DisplayRefresh(1);
	}
}

static void ClearLine(char *LineNo) {
    memset(LineNo, ' ', 16);
}


/*-----------*/
/* MAIN Loop */
/*-----------*/
int main(void) {
	uint8_t x;

	_INIT();

	wdt_enable(WDTO_1S);

    usbInit();
    usbDeviceDisconnect();	// enforce re-enumeration, do this while interrupts are disabled!
    x = 25;
    while(--x) {             // fake USB disconnect for > 250 ms
        wdt_reset();
        _delay_ms(10);
    }
    usbDeviceConnect();

	pulse_counter = 0;
	sei();

    DisplayRefresh(1);
    x = 100;
    while(--x) {
        wdt_reset();
        _delay_ms(10);
    }
	ClearLine(SecondLine);
	DisplayRefresh(1);

    uart_init(__UBRR);
    uart_putstr("\r\n");
    uart_putstr("Radioactive@Home V2.60\r\n");
    uart_putstr("Firmware 1.0 (27.03.14)\r\n");

    while(1) {
    	wdt_reset();
        usbPoll();

        HV_Supply_EVENT();
        BeepIfChange_EVENT();
        Switch_EVENT();
        RadCalc_EVENT();
        DisplayRefresh_EVENT();
    }
    return 0;
}
/* END */
