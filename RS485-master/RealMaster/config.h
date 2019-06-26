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

#define HOST_SYNCBYTE       0x16
#define RS485_SYNCBYTE	    0x17
#define NUM_OF_PACKET	    10

#define RS485_DE	    6
#define RS485_RE	    7
#define BUZZER_PIN	    1
#define BUZZER_TIMEOUT	    15

#define ENABLE_TX	    PORTD |= (1 << RS485_DE)
#define DISABLE_TX	    PORTD &= ~(1 << RS485_DE)

#define ENABLE_RX	    PORTD &= ~(1 << RS485_RE)
#define DISABLE_RX	    PORTD |= (1 << RS485_RE)

#define MASTER_ADDR	    0x01
#define SLAVE1_ADDR	    0x02
#define SLAVE2_ADDR	    0x03
#define SLAVE3_ADDR	    0x04
#define SLAVE4_ADDR         0x05
#define SLAVE5_ADDR	    0x06
	
#define GET_TEMPERATURE     0x02
#define GET_LEDSTATUS       0x03

#define MAX_BUZZER	    1
#define BUZZER_ADDR	    0x07
#define BUZZER_ON	    0x01
#define BUZZER_OFF	    0x02

#endif
