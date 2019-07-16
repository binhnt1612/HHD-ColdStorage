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
volatile uint8_t pulse_count = 0;

//Temperature sensor covariance for calculation with range of resistor value
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
    //Set RS485 control pin
    DDRD |= (1 << RS485_DE) | (1 << RS485_RE);
    //Set as receive mode
    ENABLE_RX;
    DISABLE_TX;  
    while (PORTD & (1 << RS485_DE));
    while (PORTD & (1 << RS485_RE));
	
    UART_Initialize(600);
	
    //Set 10 led pins as input 
    DDRD &= ~( (1 << DDD2) | (1 << DDD3) | (1 << DDD5) );
    DDRB &= ~(1 << DDB1);
	
    for(int i = 0; i < 6; i++)
        DDRC &= ~(1 << i);
	
    //Setup ADC for reading sensor value
    ADMUX = (1 << REFS0) | (1 << MUX2) | (1 << MUX1);
    ADCSRA = (1 << ADEN) | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);
    
    //Set packet to send to master
    S_Packet[0].SlaveAddr	= SLAVE1_ADDR;
    S_Packet[0].Function	= GET_TEMPERATURE;
	
    S_Packet[1].SlaveAddr	= SLAVE1_ADDR;
    S_Packet[1].Function	= GET_LEDSTATUS;

    //Setup timer 0 for heartbeat pulse 
    TCCR0B = (1 << CS02) | (1 << CS00);

    //Setup timer 1 for heartbeat - 4sec
    TCCR1B = (1 << CS10) | (1 << CS12);
    TIMSK1 |= 1 << TOIE1;
    TCNT1 = 0xBDC;
	
    sei();
	
    _delay_ms(100);
	
    while (1) {
	//Waiting for receiving correct packet from master
        while (RS485_SlaveReceivePacket(&M_Packet, SLAVE1_ADDR));	
		
	_delay_ms(20);
	    
        //Check packet function to do: TEMP - LED
        if (M_Packet.Function == GET_TEMPERATURE) {		
	    conv_ok = NTCtemp_Reading(temp_C);
	    //Not send data if wrong value sensor read
	    S_Packet[0].Length = (conv_ok) ? sizeof(double) : 0;
	    //Put all data from sensor to packet (datapayload)	
	    for(int count = 0; count < S_Packet[0].Length; count++)
	        S_Packet[0].Data[count] = temp_C[count];
            //Reset watchdog register when system works normally
		
	    cli();  //Disable global interrupt when sending packet to avoid transmission mismatch
		
	    if (RS485_SlaveSendPacket(S_Packet[0]) == true) {
		//Reset watchdog register when system works normally
	        wdt_reset();
	        timeout_count = 0;
	    }	   
		
	    sei();  //Reenable global interrupt
		
        }
		
        else if (M_Packet.Function == GET_LEDSTATUS) {
            led = LedStatus_Reading();
	    //Store 10 leds in 2 bytes	
	    S_Packet[1].Length = 2;
	    S_Packet[1].Data[0] = led & 0xFF;
	    S_Packet[1].Data[1] = (led >> 8) & 0xFF;
			
            if (RS485_SlaveSendPacket(S_Packet[1]) == true)) {
	        //Reset watchdog register when system works normally
	        wdt_reset();
	        timeout_count = 0;
	    }	
        }
    }
    return 0;
}

/*
 * Reading LED input to indicate its status
 * Reading input with 1 if normal, 0 if malfunction
 * When normal, 10 leds has value: 0x3FF (00000011 11111111)
 */

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

/*
 * Reading NTC sensor through ADC 10 bit
 * Input: array 4 byte to store temp float value
 */
bool NTCtemp_Reading(uint8_t array_tempC[]) {
    int i = 0;
    double voltage, current;
    double res = 0.0;
    double inv_tempK, tempC;
    const void *ptr = &tempC;
	
    //Get average of 100 values reading 
    for (int j = 0; j < 100; j++) {
	ADMUX |= (1 << REFS0);
	ADCSRA |= 1 << ADSC;
	//Waiting for finish converting
	while (! (ADCSRA & (1 << ADIF)) );
	
	//Resistor calculation
	voltage = ADCW * 5.0 / 1024.0;
		
	current = voltage / CONST_RES;
	res += (5.0 - voltage) / current;
		
	ADCSRA |= 1 << ADIF;
	_delay_ms(10);
    }
	
    res = res / 100.0;
    //Get the resistor covariance according to the range of resistor value 	
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
	
    //Calculate resistor value if gets value in range
    if (i > -1) {
	inv_tempK = A[i] + B[i] * log(res) + C[i] * pow(log(res), 3);
	tempC = (1.0 / inv_tempK - 273.15) - OFFSET_CALIB;
	//Convert 4byte float to 8bit array[4]	
	for(int count = 0; count < sizeof(double); count++) {
	    array_tempC[count] = *(uint8_t *) ptr;
	        ptr++;
	}
	return true;
    }
    return false;
}

/*
 * This function to make sure watchdog not reset
 * unwilling afer a software reset
 */
void WDT_off(void) {
    cli();
	
    MCUSR &= ~(1 << WDRF);
    WDTCSR |= (1 << WDCE) | (1 << WDE);
    WDTCSR = 0x00;
	
    sei();
}

/*
 * Setup watchdog timer function, interrupt occured each 8s 
 * without wdt_reset();
 */
void WDT_Enable(void) {
    cli();
	
    WDTCSR |= (1<<WDCE) | (1 << WDE);
    WDTCSR = (1 << WDP3) | (1 << WDP0);		//8s timeout
    WDTCSR |= (1 << WDIE);
	
    sei();
}

/*
 * Interrupt service routine trigger after 8 second without wdt_reset() called
 * Designed to make sure software reset after 4' without any wdt_reset() occured
 */
ISR(WDT_vect) {
    cli();
    timeout_count++;
	
    WDTCSR |= (1 << WDIE);
	
    //Generate a software reset immediately
    if (timeout_count == 30) {
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	WDTCSR = (1<<WDE);
	while (1);
    }
	
    sei();
}

/* 
 * ISR for heartbeat pulsewidth generated 
 * Turn on after ISR1 trigger and trigger each 15ms 
 * Turn off ISR after generate a 150ms pulsewidth
*/

ISR(TIMER0_OVF_vect) {
    cli();
    pulse_count++;          //Keep counting until get 150ms 
    
    TCNT0 = 0x15;
	
    if (pulse_count == 10) {
        pulse_count = 0;
	//Finish generate a heartbeat pulse
	PORTB |= (1 << PORTB4);
	DDRB &= ~(1 << DDB4);
	//Turn off this ISR (ISR timer 0)
	TIMSK0 &= ~(1 << TOIE0);
    }
	
    sei();	
}

/* 
 * ISR for heartbeat trigger and trigger each 4second
 * when system power on
*/urn on after ISR1 trigger and trigger each 15ms 
298
 * Turn off ISR after generate a 150ms pulsewidth

ISR(TIMER1_OVF_vect) {
    cli();
    //Initial value for timer0,1
    TCNT0 = 0x15;
    TCNT1 = 0xBDC;
	
    //Enable interrupt for pulsewidth (ISR timer0)
    TIMSK0 |= 1 << TOIE0;
	
    //Start generating heartbeat pulse
    DDRB |= (1 << DDB4);
    PORTB &= ~(1 << PORTB4);
	
    sei();
}

