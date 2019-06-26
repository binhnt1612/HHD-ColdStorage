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

uint16_t led[5] = { 0x3FF, 0x3FF, 0x3FF, 0x3FF, 0x3FF };

const uint8_t RS485_Function[2] = { GET_TEMPERATURE, GET_LEDSTATUS };
const uint8_t RS485_Slave[5] = { SLAVE1_ADDR, SLAVE2_ADDR, SLAVE3_ADDR, SLAVE4_ADDR, SLAVE5_ADDR };	

const uint8_t Host_Function[2] = { GET_TEMPERATURE, GET_LEDSTATUS };

void Master_TranscieverCommand(struct RS485_MasterPacket M_Packet[], struct HostPacket H_SendPacket[]);
void Host_TranscieverCommand(struct HostPacket H_SendPacket[]);
void Buzzer_Handler(uint16_t led[]);
	
int main(void) {
    struct RS485_MasterPacket M_Packet[NUM_OF_PACKET];
	struct HostPacket H_SendPacket[NUM_OF_PACKET]; 
		
	timeout_count = 0;
		
	WDT_off();
	WDT_Enable();
	
    //Setup UART and RE, DE pin
	DDRD |= (1 << RS485_DE) | (1 << RS485_RE);
	
	ENABLE_RX;
	DISABLE_TX;
	while (PORTD & (1 << RS485_RE));
	while (PORTD & (1 << RS485_DE));
	
    UART_Initialize(600);
	UART_SendString("Start program\n");
	
	DDRB |= (1 << BUZZER_PIN);
	PORTB |= (1 << BUZZER_PIN);

	_delay_ms(20);

	//Set timer1 for data output
	TCCR1B = (1 << CS10) | (1 << CS12);
	TIMSK1 |= 1 << TOIE1;
	TCNT1 = 0x85EE;
    
    sei();
	
    while (1) {		
		Master_TranscieverCommand(M_Packet, H_SendPacket);
	}
}

void Master_TranscieverCommand(struct RS485_MasterPacket M_Packet[], struct HostPacket H_SendPacket[]) {
	uint8_t count = 0;
	uint8_t addr, func;
	struct RS485_SlavePacket S_Packet;
	
	for (addr = 0; addr < sizeof(RS485_Slave); addr++) {
			
		for (func = 0; func < sizeof(RS485_Function); func++) {
			
			H_SendPacket[count].SlaveAddr = RS485_Slave[addr];

			M_Packet[count].SlaveAddr = RS485_Slave[addr];
			M_Packet[count].Function = RS485_Function[func];
			
			//Master transcieve packet with other slaves
			if (RS485_MasterSendPacket(M_Packet[count]) == false)
				continue;
				
			timeout = false;
			TCNT1 = 0x85EE;
		
			while (RS485_MasterReceivePacket(&S_Packet, RS485_Slave[addr]) && !timeout);
			
			if (!timeout && S_Packet.Length) {
				H_SendPacket[count].Function = S_Packet.Function;	
				H_SendPacket[count].Length = S_Packet.Length;
				
				for (uint8_t index = 0; index < S_Packet.Length; index++) 
					H_SendPacket[count].Data[index] = S_Packet.Data[index];
					
				if (S_Packet.Function == GET_LEDSTATUS)
					led[S_Packet.SlaveAddr - 2] = (S_Packet.Data[1] << 8) | S_Packet.Data[0];	
				
				_delay_ms(1500);
				wdt_reset();	
				timeout_count = 0;		
			}
			
			else {
				H_SendPacket[count].Function = M_Packet[count].Function | 0x80; //31250
				H_SendPacket[count].Length = 0;
				if (M_Packet[count].Function == GET_LEDSTATUS)
					led[M_Packet[count].SlaveAddr - 2] = 0x3FF;
			}		
			
			//Master transmit packet with Host PC
			SendPacket_toPC(H_SendPacket[count]);
			_delay_ms(1000);
		
			count++;
		}	
	}
	
	Buzzer_Handler(led);
}

void Buzzer_Handler(uint16_t led[]) {
	bool buzzer_en = false;
	struct RS485_MasterPacket Buzzer_Node;
		
	Buzzer_Node.SlaveAddr = BUZZER_ADDR;
	
	for (int led_count = 0; led_count < 5; led_count++) {
		if ((led[led_count] & 0x03FF) != 0x03FF) {
			buzzer_en = true;
			
			if (!buzzer_break) {
				PORTB &= ~(1 << BUZZER_PIN);
				Buzzer_Node.Function = BUZZER_ON;
				
				if (RS485_MasterSendPacket(Buzzer_Node) == true) {
					buzzer_count++;
					_delay_ms(2000);
				}
				
				if(buzzer_count == MAX_BUZZER) {
					buzzer_break = true;
					buzzer_count = 0;
				}
			}
			
			else {
				PORTB |= (1 << BUZZER_PIN);
				Buzzer_Node.Function = BUZZER_OFF;
				
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
	
	if (buzzer_en == false) {
		PORTB |= (1 << BUZZER_PIN);
		Buzzer_Node.Function = BUZZER_OFF;
		
		if (RS485_MasterSendPacket(Buzzer_Node) == true)
		_delay_ms(3000);
	}
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
		WDTCSR |= (1 << WDCE) | (1 << WDE);		WDTCSR = (1 << WDE);
		while (1);
	}
	
	sei();
	
}

ISR(TIMER1_OVF_vect) {
	timeout = true;
}

