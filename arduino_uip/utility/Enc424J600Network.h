/*
 Enc424J600NetworkClass.h
 UIPEthernet network driver for Microchip ENC28J60 Ethernet Interface.

 Copyright (c) 2013 Norbert Truchsess <norbert.truchsess@t-online.de>
 All rights reserved.

 inspired by enc28j60.c file from the AVRlib library by Pascal Stang.
 For AVRlib See http://www.procyonengineering.com/

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef Enc424J600Network_H_
#define Enc424J600Network_H_

#include "mempool.h"

#define ENC28J60_CONTROL_CS     A7
#define SPI_MOSI        MOSI
#define SPI_MISO        MISO
#define SPI_SCK         SCK
#define SPI_SS          SS

#define UIP_RECEIVEBUFFERHANDLE 0xff

//#define ENC28J60DEBUG

/*
 * Empfangen von ip-header, arp etc...
 * wenn tcp/udp -> tcp/udp-callback -> assign new packet to connection
 */

class Enc424J600Network : public MemoryPool
{

private:
  static uint16_t nextPacketPtr;
  static uint8_t bank;

  static struct memblock receivePkt;

  /* basic functions */
  static void enc_SBI(uint8_t instruction, bool keepEnabled = false); // single byte instruction
  static void enc_writeOp(uint8_t op, uint8_t address, uint8_t* data, uint8_t len); // perform write operation
  static void enc_readOp(uint8_t op, uint8_t address, uint8_t* data, uint8_t len); // perform read operation
  static void enc_setBank(uint8_t address, bool keepEnabled = false); // select the memory bank
  
  static void writeControlRegister(uint8_t address, uint8_t data); // select bank and write control register
  static void writeControlRegister16(uint8_t address, uint16_t data); // select bank and write 2 bytes to control register
  static uint8_t readControlRegister(uint8_t address); // select bank and read control register
  static uint16_t readControlRegister16(uint8_t address); // select bank and read control register
  static void writeBitField(uint8_t address, uint8_t data); // select bank and write control register bit
  static void writePointer(uint8_t instruction, uint16_t address, bool keepEnabled = false); // select bank and write 2 bytes to a pointer
  
  static void mempool_block_move(memaddress dest, memaddress src, memaddress len, uint16_t bufstart, uint16_t bufend);
  
  

  static void writeOp(uint8_t op, uint8_t address, uint8_t data);
  static uint16_t setReadPtr(memhandle handle, memaddress position, uint16_t len);
  static void setERXRDPT();
  static void readBuffer(uint16_t len, uint8_t* data);
  static void writeBuffer(uint16_t len, uint8_t* data);
  static uint8_t readByte(uint16_t addr);
  static uint8_t readByteTX(uint16_t addr);
  static void writeByte(uint16_t addr, uint8_t data);
  
  
  
  static void phyWrite(uint8_t address, uint16_t data);
  static uint16_t phyRead(uint8_t address);
  static void clkout(uint8_t clk);

 // static void ENC624J600WritePTR(uint8_t instruction, uint16_t address, uint8_t disable_cs);

  friend void mempool_block_move_callback(memaddress,memaddress,memaddress);

public:

  uint8_t getrev(void);
  void powerOn();
  void powerOff();
  bool linkStatus();

  static void init(uint8_t* macaddr);
  static memhandle receivePacket();
  static void freePacket();
  static memaddress blockSize(memhandle handle);
  static void sendPacket(memhandle handle);
  static uint16_t readPacket(memhandle handle, memaddress position, uint8_t* buffer, uint16_t len);
  static uint16_t writePacket(memhandle handle, memaddress position, uint8_t* buffer, uint16_t len);
  static void copyPacket(memhandle dest, memaddress dest_pos, memhandle src, memaddress src_pos, uint16_t len, uint8_t buffertype);
  static uint16_t chksum(uint16_t sum, memhandle handle, memaddress pos, uint16_t len);
};

extern Enc424J600Network Enc28J60;
#endif /* Enc424J600NetworkClass_H_ */
