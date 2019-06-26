 /*
  * File       :main.c
  * Author     :Thanh Binh Nguyen
  * Version    :0.1
  * Date       :20-Feb-2019
  * Brief      :This file provides firmware functions for the RS485 master
  */ 

#include <avr/io.h>
#include <stdbool.h>
#include <avr/delay.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include "config.h"
#include "UART.h"
#include "RS485.h"
#include "HostPC.h"

volatile bool timeout = false;
volatile uint8_t time_interval = 1;	
uint8_t buzzer_count = 0;
bool buzzer_break = false;

void WDT_off(void);
void WDT_Enable(void);

volatile uint8_t timeout_count = 0;

//Default - all the led are normally
uint16_t led[5] = { 0x3FF, 0x3FF, 0x3FF, 0x3FF, 0x3FF };
//Querry function from master to slaves
const uint8_t RS485_Function[2] = { GET_TEMPERATURE, GET_LEDSTATUS };
//Using 5 slaves in a bus
const uint8_t RS485_Slave[5] = { SLAVE1_ADDR, SLAVE2_ADDR, SLAVE3_ADDR, SLAVE4_ADDR, SLAVE5_ADDR };	
//Function to send to PC
const uint8_t Host_Function[2] = { GET_TEMPERATURE, GET_LEDSTATUS };

void Master_TranscieverCommand(struct RS485_MasterPacket M_Packet[], struct HostPacket H_SendPacket[]);
void Host_TranscieverCommand(struct HostPacket H_SendPacket[]);
void Buzzer_Handler(uint16_t led[]);
	
int main(void) {
    struct RS485_MasterPacket M_Packet[NUM_OF_PACKET];
    struct HostPacket H_SendPacket[NUM_OF_PACKET]; 
		 
    //Pre-setup watchdog 
    timeout_count = 0;
    WDT_off();
    WDT_Enable();
	
    //Setup UART and RE, DE pin as output
    DDRD |= (1 << RS485_DE) | (1 << RS485_RE);
 
    //Set RS485 as receive mode
    ENABLE_RX;
    DISABLE_TX;polynomial
    while (PORTD & (1 << RS485_RE));
    while (PORTD & (1 << RS485_DE));
	
    UART_Initialize(600);
    UART_SendString("Start program\n");
    
    //Set buzzer pin as output and turn it off
    DDRB |= (1 << BUZZER_PIN);
    PORTB |= (1 << BUZZER_PIN);

    _delay_ms(20);

    //Set timer1 for recieve correct packet timeout
    TCCR1B = (1 << CS10) | (1 << CS12);  
    TIMSK1 |= 1 << TOIE1;
    TCNT1 = 0x85EE;
    
    sei();
	
    while (1) {		
        Master_TranscieverCommand(M_Packet, H_SendPacket);
    }
}
/*
 * This function is designed for master to querry all the slaves with
 * willing function, get respond from slave and then send back all data
 * with correspond function of a slave to PC
 */
void Master_TranscieverCommand(struct RS485_MasterPacket M_Packet[], struct HostPacket H_SendPacket[]) {
    uint8_t count = 0;
    uint8_t addr, func;
    struct RS485_SlavePacket S_Packet;
    
    //Send querry to 5 slaves, each slave 2 functions and waiting for correct respond
    for (addr = 0; addr < sizeof(RS485_Slave); addr++) {
			
	for (func = 0; func < sizeof(RS485_Function); func++) {
	    //Set data to send to PC(H_SendPacket) and slave (M_Packet)	
	    H_SendPacket[count].SlaveAddr = RS485_Slave[addr];

	    M_Packet[count].SlaveAddr = RS485_Slave[addr];
	    M_Packet[count].Function = RS485_Function[func];
			
	    //Turn back loop if master could not send (bus checking failed)
	    if (RS485_MasterSendPacket(M_Packet[count]) == false)
		continue;
				
            //Waiting for master get correct respond before timeout
	    timeout = false;
	    TCNT1 = 0x85EE;
 
	    while (RS485_MasterReceivePacket(&S_Packet, RS485_Slave[addr]) && !timeout);
            //If sensor works correctly and get correct packet before timeout
            if (!timeout && S_Packet.Length) {
                //Set data to send to PC: function, len, datapayload
	        H_SendPacket[count].Function = S_Packet.Function;	
		H_SendPacket[count].Length = S_Packet.Length;
				
		for (uint8_t index = 0; index < S_Packet.Length; index++) 
		    H_SendPacket[count].Data[index] = S_Packet.Data[index];
		//Store led value according to slave for buzzer control			
		if (S_Packet.Function == GET_LEDSTATUS)
		    led[S_Packet.SlaveAddr - 2] = (S_Packet.Data[1] << 8) | S_Packet.Data[0];	
				
		_delay_ms(1500);
		//System works normally, so reset watchdog register
                wdt_reset();	
		timeout_count = 0;	
            }
	
            else {
		  //Set MSB to indicate error receiving data or error sensor value
	          H_SendPacket[count].Function = M_Packet[count].Function | 0x80; 
		  H_SendPacket[count].Length = 0;
		  //Assume that alpolynomiall led works normally if could not receive led status 
		  if (M_Packet[count].Function == GET_LEDSTATUS)
		      led[M_Packet[count].SlaveAddr - 2] = 0x3FFF
	    }		
			
	    //Master transmit packet to host after each slave's data received
	    SendPacket_toPC(H_SendPacket[count]);
	    _delay_ms(1000);
		
	    count++;
        }	
    }
    //Control buzzer after getting all led status from 5 slaves
    Buzzer_Handler(led);
}

/*
 * This function to handle buzzer of master and buzzer from 1 slave
 * It called after master receive all 5 slave's led status
 * Buzzer rings if 1 led of 1 slave malfunction, ring for a while then 
 * off, continously
 * If all led normal, always off
 */

void Buzzer_Handler(uint16_t led[]) {
    bool buzzer_en = false;
    struct RS485_MasterPacket Buzzer_Node;
		
    Buzzer_Node.SlaveAddr = BUZZER_ADDR;

    for (int led_count = 0; led_count < 5; led_count++) {
	//If led of one slave turn on, set buzzer status true
        if ((led[led_count] & 0x03FF) != 0x03FF) {
	    buzzer_en = true;
	    
            //If buzzer not in break state, turn on and send to buzzer slave
	    if (!buzzer_break) {
	        PORTB &= ~(1 << BUZZER_PIN);
	        Buzzer_Node.Function = BUZZER_ON;
		
		//This code make sure buzzer turn on after a while then turn off
	        if (RS485_MasterSendPacket(Buzzer_Node) == true) {
	            buzzer_count++;
		    _delay_ms(2000);
	        }
				
	        if(buzzer_count == MAX_BUZZER) {
	            buzzer_break = true;
		    buzzer_count = 0;
	        } 
	    }
		
	    //If buzzer in break state, turn off and send to buzzer slave
	    else {
	        PORTB |= (1 << BUZZER_PIN);
	        Buzzer_Node.Function = BUZZER_OFF;
		  
		//This code make sure buzzer turn off after a while then turn on		
	        if (RS485_MasterSendPacket(Buzzer_Node) == true) {
	            buzzer_count++;
	            _delay_ms(2000);
	        }
				
	        if (buzzer_count == MAX_BUZZER) {
		    buzzer_break = false;
		    buzzer_count = 0;
	        }
	    }
	    break;	
        }
    }
    
    //Turn off and send buzzer command if all led working as normally
    if (buzzer_en == false) {
        PORTB |= (1 << BUZZER_PIN);
	Buzzer_Node.Function = BUZZER_OFF;
		
	if (RS485_MasterSendPacket(Buzzer_Node) == true)
	    _delay_ms(3000);
    }
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
        WDTCSR |= (1 << WDCE) | (1 << WDE);
	WDTCSR = (1 << WDE);
	while (1);
    }
	
    sei();
}

/*
 * Interrupt service routine trigger if receive correct packet timeout
 */
ISR(TIMER1_OVF_vect) {
    timeout = true;
}

