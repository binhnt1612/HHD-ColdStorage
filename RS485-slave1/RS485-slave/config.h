 /*
  * File        :config.h
  * Author      :Thanh Binh Nguyen
  * Version     :0.1
  * Date        :20-Feb-2018
  * Brief       :Provide config definition
  */

#ifndef __CONFIG_H
#define __CONFIG_H

#ifndef F_CPU
	#define F_CPU 16000000UL
#endif

#define RS485_SYNCBYTE		0x17
#define NUM_OF_PACKET		2

#define RS485_DE			6
#define RS485_RE			7

#define ENABLE_TX			PORTD |= (1 << RS485_DE)
#define DISABLE_TX			PORTD &= ~(1 << RS485_DE)

#define ENABLE_RX			PORTD &= ~(1 << RS485_RE)
#define DISABLE_RX			PORTD |= (1 << RS485_RE)

#define SLAVE1_ADDR			0x02

#define GET_TEMPERATURE     0x02
#define GET_LEDSTATUS       0x03

//NTC Temparature config
#define CONST_RES			9800
#define OFFSET_CALIB		1.15  

#endif
