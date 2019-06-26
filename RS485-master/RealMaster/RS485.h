 /*
  * File        :RS485.h
  * Author      :Thanh Binh Nguyen
  * Version     :0.1
  * Date        :20-Feb-2018
  * Brief       :Provide RS485 master-slave definition
  */

#ifndef __RS485_H
#define __RS485_H

#include <stdbool.h>

struct RS485_MasterPacket {
    uint8_t SlaveAddr;
    uint8_t Function;
    uint16_t CRC16;
};

struct RS485_SlavePacket {
    uint8_t SlaveAddr;
    uint8_t Function;
    uint8_t Length;
    uint8_t Data[50];
    uint16_t CRC16;	
};

bool RS485_MasterSendPacket(struct RS485_MasterPacket M_Packet);
uint8_t RS485_SlaveReceivePacket(struct RS485_MasterPacket *M_Packet, uint8_t desireAddr);
void RS485_SlaveSendPacket(struct RS485_SlavePacket S_Packet);
uint8_t RS485_MasterReceivePacket(struct RS485_SlavePacket *S_Packet, uint8_t desireAddr);
uint16_t CRC16_Calculation(uint8_t *ptr, uint8_t length);
bool RS485_BusChecking(void);

#endif
