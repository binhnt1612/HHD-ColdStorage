 /*
  * File        :UART.c
  * Author      :Thanh Binh Nguyen
  * Version     :0.1
  * Date        :20-Feb-2019
  * Brief       :UART peripheral function for ATmega328p
  */
 
#include <avr/io.h>
#include "config.h"
#include "UART.h"

void UART_Initialize(uint32_t baudrate)	{
	/*
	uint16_t baudrate_calc;
    
    UCSR0A = (1 << U2X0);
	
    baudrate_calc = ((F_CPU + baudrate * 4L) / (baudrate * 8L)) - 1;
    UBRR0H = (uint8_t) baudrate_calc >> 8;					
    UBRR0L = (uint8_t) baudrate_calc & 0x00FF;	
	*/
	UBRR0 = 1666;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);					
    UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);					
}

void UART_SendByte(uint8_t data) {
    while (! (UCSR0A & (1 << UDRE0)) );			
    UCSR0A |= (1 << TXC0);
    UDR0 = data;								
}

uint8_t UART_ByteReceive(void) {
    uint16_t count;
    for (count = 0; count < 0xFFFF; count++) {
        if (UCSR0A & (1 << RXC0))
			return UDR0;
	}
    return 0;
}

void UART_SendString(const char *str) {
	uint16_t index;
	for (index = 0; str[index] != 0; index++)						//Scan string and send through UART
		UART_SendByte(str[index]);
}
