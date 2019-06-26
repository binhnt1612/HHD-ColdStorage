 /*
  * File        :HostPC.c
  * Author      :Thanh Binh Nguyen
  * Version     :0.1
  * Date        :20-Feb-2018
  * Brief       :Provide PC UART protocol
  */
  
#include <avr/io.h>
#include "config.h"
#include "UART.h"
#include "HostPC.h"
#include "RS485.h"

/*
	PACKET FORMAT
	-----------------------------------------------------------------------------------
	|   1 byte   |    1 byte    |  1 byte   |    1 byte    |    n byte   |   2 byte   |
	-----------------------------------------------------------------------------------
	| Sync Byte  | Slave Addr   | Function  |  DataLength  | DataPayload |     CRC    |
*/

uint8_t ReceivePacket_fromPC(struct HostPacket *H_Packet, uint8_t desireAddr) {
    uint8_t CRC_HighByte, CRC_LowByte;
    uint16_t CRC_Verify;
    uint8_t packetLength;
    uint8_t index, sync_byte;
	
    sync_byte = UART_ByteReceive();
    if (sync_byte != HOST_SYNCBYTE)
        return 1;
	
    H_Packet->SlaveAddr = UART_ByteReceive();
    if(H_Packet->SlaveAddr != desireAddr)
        return 2;
	
    H_Packet->Function = UART_ByteReceive();
	
    H_Packet->Length = UART_ByteReceive();

    for (index = 0; index < H_Packet->Length; index++)
        H_Packet->Data[index] = UART_ByteReceive();
		
    CRC_LowByte = UART_ByteReceive();
    CRC_HighByte = UART_ByteReceive();
    H_Packet->CRC16 = (CRC_HighByte << 8) | CRC_LowByte;

    packetLength = 3 + H_Packet->Length;
    CRC_Verify = CRC16_Calculation((uint8_t *) H_Packet, packetLength);

    if (CRC_Verify != H_Packet->CRC16)
        return 3; 

    return 0;	
}

/*
	PACKET FORMAT
	----------------------------------------------------------------------------------
	|   1 byte   |   1 byte	  |   1 byte   |    1 byte    |    n byte   |   2 byte   |
	---------------------------------------------------------------------------------
	| Sync Byte  | Slave Addr |  Function  |  DataLength  | DataPayload |     CRC    |
*/

void SendPacket_toPC(struct HostPacket H_Packet) {
    void *packetPtr = &H_Packet;
    uint8_t CRC_HighByte, CRC_LowByte;
    uint8_t packetLength = 3 + H_Packet.Length;
	uint8_t pos;
	
    H_Packet.CRC16 = CRC16_Calculation((uint8_t *) packetPtr, packetLength);
	
    UART_SendByte(HOST_SYNCBYTE);
	
    for (pos = 0; pos < 3; pos++)
        UART_SendByte(*(uint8_t *) packetPtr++);
	
    for (pos = 0; pos < H_Packet.Length; pos++)
        UART_SendByte(H_Packet.Data[pos]);
	
    CRC_LowByte = H_Packet.CRC16 & 0xFF;
    CRC_HighByte = (H_Packet.CRC16 >> 8) & 0xFF;
    UART_SendByte(CRC_LowByte);
    UART_SendByte(CRC_HighByte);
	
    while ( !(UCSR0A & (1 << TXC0)) );
}
