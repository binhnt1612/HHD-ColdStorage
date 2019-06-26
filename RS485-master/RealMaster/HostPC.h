 /*
  * File        :HostPC.h
  * Author      :Thanh Binh Nguyen
  * Version     :0.1
  * Date        :20-Feb-2018
  * Brief       :Provide PC UART definition
  */

#ifndef __HOSTPC_H
#define __HOSTPC_H

struct HostPacket {
    uint8_t SlaveAddr;
    uint8_t Function;
    uint8_t Length;
    uint8_t Data[50];
    uint16_t CRC16;	
};

uint8_t ReceivePacket_fromPC(struct HostPacket *H_Packet, uint8_t desireAddr);
void SendPacket_toPC(struct HostPacket H_Packet);

#endif
  
