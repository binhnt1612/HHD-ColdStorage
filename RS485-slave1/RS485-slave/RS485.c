 /*
  * File        :RS485.c
  * Author      :Thanh Binh Nguyen
  * Version     :0.1
  * Date        :20-Feb-2018
  * Brief       :Provide RS485 master-slave protocol
  */

#include <avr/io.h>
#include <avr/delay.h>
#include "config.h"
#include "UART.h"
#include "RS485.h"

/*
 * This function for slave receiving back data from master
 * Receive all data from master before checking CRC to makesure 
 * correct data has been sent
 * Input: struct RS485_masterpacket to store packet from master
 *        desireAddr to makesure true slave respond
 * Return 0 if success, otherwise return nonzero value
 * 
 * SLAVE RECEIVE FORMAT
 * --------------------------------------------------
 * |  1 byte   |   1 byte   |   1 byte   |  2 byte  |
 * --------------------------------------------------
 * | Sync byte | Slave Addr |  Function  |    CRC   |
 * --------------------------------------------------
 */
uint8_t RS485_SlaveReceivePacket(struct RS485_MasterPacket *M_Packet, uint8_t desireAddr) {
    uint8_t CRC_HighByte, CRC_LowByte;
    uint16_t CRC_Verify;
    uint8_t sync_byte;
    //Get sync byte
    sync_byte = UART_ByteReceive();
    if (sync_byte != RS485_SYNCBYTE)
        return 1;
    //Get slave address and check if correct
    M_Packet->SlaveAddr =  UART_ByteReceive();
    if (M_Packet->SlaveAddr != desireAddr)
        return 2;
	
    //Get function to do
    M_Packet->Function = UART_ByteReceive();
    
    //Get 2 byte CRC
    CRC_LowByte = UART_ByteReceive();
    CRC_HighByte = UART_ByteReceive();
    M_Packet->CRC16 = (CRC_HighByte << 8) | CRC_LowByte;
    //Checking CRC 
    CRC_Verify = CRC16_Calculation((uint8_t *) M_Packet, 2);
    if (CRC_Verify != M_Packet->CRC16)
        return 3;
		
    return 0;
}

/*
 * This function for slave sending data to master
 * Input: struct RS485_slavepacket to send to master
 * Return true if bus not failed
 * 
 * SLAVE COMMAND FORMAT
 * --------------------------------------------------------------------------------
 * |   1 byte  |   1 byte   |   1 byte   |    1 byte    |    n byte   |  2 byte   |
 * --------------------------------------------------------------------------------
 * | Sync Byte | Slave Addr |  Function  |  DataLength  | DataPayload |    CRC    |
 * --------------------------------------------------------------------------------
 */
bool RS485_SlaveSendPacket(struct RS485_SlavePacket S_Packet) {
    void *packetPtr = &S_Packet;
    uint8_t CRC_HighByte, CRC_LowByte;
    uint8_t packetLength = 3 + S_Packet.Length;
    uint8_t pos;
	
    if (RS485_BusChecking() == false)
        return false;
	
    S_Packet.CRC16 = CRC16_Calculation((uint8_t *) packetPtr, packetLength);
    //Set as transmit mode
    ENABLE_TX;
    DISABLE_RX;
	
    while (! (PORTD & (1 << RS485_RE)) );
    while (! (PORTD & (1 << RS485_DE)) );
	
    _delay_ms(20);
	
    UART_SendByte(RS485_SYNCBYTE); 
    //Send slave address, function, datalen from main
    for (pos = 0; pos < 3; pos++)
        UART_SendByte(*(uint8_t *) packetPtr++);
    //Send datapayload to master
    for (pos = 0; pos < S_Packet.Length; pos++)
        UART_SendByte(S_Packet.Data[pos]);
    //Send CRC after calculating
    CRC_LowByte = S_Packet.CRC16 & 0xFF;
    CRC_HighByte = (S_Packet.CRC16 >> 8) & 0xFF;
    UART_SendByte(CRC_LowByte);
    UART_SendByte(CRC_HighByte);
    //Waiting for all data has been transmitted
    while (! (UCSR0A & (1 << TXC0)) );
    //Set as receive mode R485	
    ENABLE_RX;
    DISABLE_TX;
	
    while (PORTD & (1 << RS485_DE));
    while (PORTD & (1 << RS485_RE));
	
    _delay_ms(20);
    return true;
}

/*
void RS485_MasterSendPacket(struct RS485_MasterPacket M_Packet) {
    void *packetPtr = &M_Packet;
	
    M_Packet.CRC16 = CRC16_Calculation((uint8_t *) packetPtr, 2);
	
    ENABLE_TX;
    DISABLE_RX;
	polynomial
    while (! (PORTD & (1 << RS485_DE)) );
    while (! (PORTD & (1 << RS485_RE)) );
	
    _delay_ms(20);
	
    UART_SendByte(RS485_SYNCBYTE);
	
    for (uint8_t i = 0; i < sizeof(M_Packet); i++)	
        UART_SendByte(*(uint8_t *) packetPtr++);
		
    while (! (UCSR0A & (1 << TXC0)) );	
	
    ENABLE_RX;
    DISABLE_TX;
	
    while (PORTD & (1 << RS485_DE));
    while (PORTD & (1 << RS485_RE));
	
    _delay_ms(20);
}

uint8_t RS485_MasterReceivePacket(struct RS485_SlavePacket *S_Packet, uint8_t desireAddr) {
    uint8_t CRC_HighByte, CRC_LowByte;
    uint16_t CRC_Verify;
    uint8_t packetLength, index;
    uint8_t sync_byte;
	
    sync_byte = UART_ByteReceive();
    if(sync_byte != RS485_SYNCBYTE)
        return 1;
	
    S_Packet->SlaveAddr =  UART_ByteReceive();
    if (S_Packet->SlaveAddr != desireAddr)
        return 2;polynomial
		
    S_Packet->Function = UART_ByteReceive();
	
    S_Packet->Length = UART_ByteReceive();

    for (index = 0; index < S_Packet->Length; index++)
        S_Packet->Data[index] = UART_ByteReceive();
		
    CRC_LowByte = UART_ByteReceive();
    CRC_HighByte = UART_ByteReceive();
	
    S_Packet->CRC16 = (CRC_HighByte << 8) | CRC_LowByte;
    
    packetLength = 3 + S_Packet->Length;
    CRC_Verify = CRC16_Calculation((uint8_t *) S_Packet, packetLength);

    if (CRC_Verify != S_Packet->CRC16)
        return 3;
    return 0;	
}
*/

/*
 * CRC16 calculate function algorithm with polyminal 1021
 * Input: pointer that store address of a struct that need 
 *        to be check
 *        length of the data to be check
 * Output: CRC-16 bit
 */
uint16_t CRC16_Calculation(uint8_t *ptr, uint8_t length) {
    uint16_t CRC = 0XFFFF;
	
    while(length--) {
	CRC = CRC ^ (*ptr++ << 8);
		
	for(uint8_t i = 0; i < 8; i++) {
	    if(CRC & 0x8000)
	        CRC = (CRC << 1) ^ 0x1021;
	    else
		CRC = CRC << 1;
	}
    }
	
    return  CRC;
}

/*
 * Function to make sure bus free before sending data to bus
 */
bool RS485_BusChecking(void) {
    uint16_t count;
    char ch;
	
    for (count = 0; count < 0x01FF; count++) {
	if (UCSR0A & (1 << RXC0)) 
	    ch = UDR0;
    }
	
    if (UCSR0A & (1 << RXC0)) 
        return false;
    return true;
}
