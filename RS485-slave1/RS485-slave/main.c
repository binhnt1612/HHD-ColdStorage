 /*
  * File	:main.c
  * Author	:Thanh Binh Nguyen
  * Version	:0.1
  * Date	:20-Feb-2019
  * Brief	:This file provides firmware functions for the RS485 slave
  */ 

#include <avr/io.h>
#include <stdbool.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include "config.h"
#include "UART.h"
#include "RS485.h"
#include <avr/delay.h>

bool NTCtemp_Reading(uint8_t array_tempC[]);
uint16_t LedStatus_Reading(void);
void WDT_off(void);
void WDT_Enable(void);

volatile uint8_t timeout_count = 0;

const double A[] = {-4.865e-3, -6.678e-4, -6.61e-5, -7.01e-4, -4.607e-3,
1.629e-4, 1.295e-3, 6.424e-3, 1.945e-3};

const double B[] = {4.171e-4, 4.554e-4, 3.834e-4, 4.902e-4, 1.132e-3,
3.793e-4, 1.719e-4, -9.203e-4, 1.549e-5};

const double C[] = {-1.622e-7, -3.265e-7, -1.816e-7, -6.063e-7, -3.154e-6,
-3.971e-7, 6.407e-7, 7.968e-6, 1.929e-6};

int main(void) {
    bool conv_ok;
    uint16_t led;
    uint8_t temp_C[4];
	
    struct RS485_MasterPacket M_Packet;
    struct RS485_SlavePacket S_Packet[NUM_OF_PACKET];
	
    timeout_count = 0;
	
    WDT_off();
    WDT_Enable();

    DDRD |= (1 << RS485_DE) | (1 << RS485_RE);
    
    ENABLE_RX;
    DISABLE_TX;  
    while (PORTD & (1 << RS485_DE));
    while (PORTD & (1 << RS485_RE));
	
    UART_Initialize(600);

    DDRD &= ~( (1 << DDD2) | (1 << DDD3) | (1 << DDD5) );
    DDRB &= ~(1 << DDB1);
	
    for(int i = 0; i < 6; i++)
        DDRC &= ~(1 << i);
	
    ADMUX = (1 << REFS0) | (1 << MUX2) | (1 << MUX1);
    ADCSRA = (1 << ADEN) | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);
	
    S_Packet[0].SlaveAddr	= SLAVE1_ADDR;
    S_Packet[0].Function	= GET_TEMPERATURE;
	
    S_Packet[1].SlaveAddr	= SLAVE1_ADDR;
    S_Packet[1].Function	= GET_LEDSTATUS;
	
    _delay_ms(100);
	
    while (1) {
        while (RS485_SlaveReceivePacket(&M_Packet, SLAVE1_ADDR));	
		
	_delay_ms(20);
		
        if (M_Packet.Function == GET_TEMPERATURE) {		
	    conv_ok = NTCtemp_Reading(temp_C);
	    S_Packet[0].Length = (conv_ok) ? sizeof(double) : 0;
			
	    for(int count = 0; count < S_Packet[0].Length; count++)
	        S_Packet[0].Data[count] = temp_C[count];
				
	    RS485_SlaveSendPacket(S_Packet[0]);
	    wdt_reset();
	    timeout_count = 0;
        }
		
        else if (M_Packet.Function == GET_LEDSTATUS) {
            led = LedStatus_Reading();
			
	    S_Packet[1].Length = 2;
	    S_Packet[1].Data[0] = led & 0xFF;
	    S_Packet[1].Data[1] = (led >> 8) & 0xFF;
			
            RS485_SlaveSendPacket(S_Packet[1]);	
	    wdt_reset();
	    timeout_count = 0;
        }
    }
    return 0;
}

uint16_t LedStatus_Reading(void) {
    uint16_t led = 0x00;
	
    //Led PortD check
    if (PIND & (1 << PIND3))
        led |= 1 << 1;
    if (PIND & (1 << PIND2))
	led |= 1 << 2;
    if (PIND & (1 << PIND5))
	led |= 1 << 10;
	
    //Led PortB check
    if(PINB & (1 << PINB1))
        led |= 1 << 9;
	
    //Led PortC check
    for(int i = 0; i < 6; i++) {
        if(PINC & (1 << i))
	    led |= (1 << (8 - i));
    }
	
    led >>= 1;
    return led;
}

bool NTCtemp_Reading(uint8_t array_tempC[]) {
    int i = 0;
    double voltage, current;
    double res = 0.0;
    double inv_tempK, tempC;
    const void *ptr = &tempC;
	
    for (int j = 0; j < 100; j++) {
	ADMUX |= (1 << REFS0);
	ADCSRA |= 1 << ADSC;
	while (! (ADCSRA & (1 << ADIF)) );
		
	voltage = ADCW * 5.0 / 1024.0;
		
	current = voltage / CONST_RES;
	res += (5.0 - voltage) / current;
		
	ADCSRA |= 1 << ADIF;
	_delay_ms(10);
    }
	
    res = res / 100.0;
	
    if (res <= 344600.0 && res >= 138800.0)
        i = 0;
    else if (res <= 138800.0 && res > 62740.0)
        i = 1;
    else if (res <= 62740.0 && res > 30390.0)
        i = 2;
    else if (res <= 30390.0 && res > 15670.0)
        i = 3;
    else if (res <= 15670.0 && res > 8523.0)
	i = 4;
    else if (res <= 8523.0 && res > 3049.0)
	i = 5;
    else if (res <= 3049.0 && res > 1448.0)
	i = 6;
    else if (res <= 1448.0 && res > 965.9)
	i = 7;
    else if (res <= 965.9 && res >= 700)
	i = 8;
    else
	i = -1;
	
    if (i > -1) {
	inv_tempK = A[i] + B[i] * log(res) + C[i] * pow(log(res), 3);
	tempC = (1.0 / inv_tempK - 273.15) - OFFSET_CALIB;
		
	for(int count = 0; count < sizeof(double); count++) {
	    array_tempC[count] = *(uint8_t *) ptr;
	        ptr++;
	}
	return true;
    }
    return false;
}

void WDT_off(void) {
    cli();
	
    MCUSR &= ~(1 << WDRF);
    WDTCSR |= (1 << WDCE) | (1 << WDE);
    WDTCSR = 0x00;
	
    sei();
}

void WDT_Enable(void) {
    cli();
	
    WDTCSR |= (1<<WDCE) | (1 << WDE);
    WDTCSR = (1 << WDP3) | (1 << WDP0);		//8s timeout
    WDTCSR |= (1 << WDIE);
	
    sei();
}

ISR(WDT_vect) {
    cli();
    timeout_count++;
	
    WDTCSR |= (1 << WDIE);						//Must be set after each interrupt
	
    if (timeout_count == 30) {
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	WDTCSR = (1<<WDE);
	while (1);
    }
	
    sei();
}
