 /* 
  * File        :UART.h
  * Author      :Thanh Binh Nguyen
  * Version     :0.1
  * Date        :20-Feb-2019
  * Brief       :This file provides definition for UART peripheral
  */

#ifndef __UART_H
#define __UART_H

#ifndef F_CPU
    #define F_CPU 16000000UL
#endif

void UART_Initialize(uint32_t baudrate);
void UART_SendByte(uint8_t data);
void UART_SendString(const char *str);
uint8_t UART_ByteReceive(void);

#endif

