/*
 Enc424J600NetworkClass.h
 UIPEthernet network driver for Microchip ENC28J60 Ethernet Interface.

 Copyright (c) 2013 Norbert Truchsess <norbert.truchsess@t-online.de>
 All rights reserved.

 based on enc28j60.c file from the AVRlib library by Pascal Stang.
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

#include "Enc424J600Network.h"
#include "Arduino.h"
#include <SPI.h>
extern "C" {
//#include <avr/io.h>
#include "Enc424J600.h"
#include "uip.h"
}

bool isActive = false;

// set CS to 0 = active
#define CSACTIVE if (!isActive) {SPI.beginTransaction(mySetting);  digitalWrite(ENC28J60_CONTROL_CS,LOW);} isActive = true
// set CS to 1 = passive
#define CSPASSIVE if (isActive) { digitalWrite(ENC28J60_CONTROL_CS,HIGH);  SPI.endTransaction();} isActive = false
//
SPISettings mySetting(14000000, MSBFIRST, SPI_MODE0);
uint16_t Enc424J600Network::nextPacketPtr;
uint8_t Enc424J600Network::bank=0xff;

struct memblock Enc424J600Network::receivePkt;

/* issue an single byte instruction */
void Enc424J600Network::enc_SBI(uint8_t instruction, bool keepEnabled)
{

	CSACTIVE;

	// issue the instruction
	SPI.transfer(instruction);

	if (!keepEnabled)
	    CSPASSIVE;
}


void Enc424J600Network::enc_writeOp(uint8_t op, uint8_t address, uint8_t* data, uint8_t len)
{
	CSACTIVE;

	// issue write command
	SPI.transfer( op | (address & ADDR_MASK));
	
	for ( int i = 0; i<len; i++)
	  SPI.transfer(  *data++);

	CSPASSIVE;
}


void Enc424J600Network::enc_readOp(uint8_t op, uint8_t address, uint8_t* data, uint8_t len)
{

	CSACTIVE;

	// issue write command
	SPI.transfer( op | (address & ADDR_MASK));
	
	for (int i = 0; i< len; i++)
	*data++ = SPI.transfer(  0x00);

	CSPASSIVE;

}

/* select bank and write control register
will always put CS high to end operation
*/
void Enc424J600Network::writeControlRegister(uint8_t address, uint8_t data)
{
  // set the bank
  enc_setBank(address,true);
  // do the write
  enc_writeOp(ENC624J600_WRITE_CONTROL_REGISTER, address, &data, 1);
}

void Enc424J600Network::writeControlRegister16(uint8_t address, uint16_t data)
{
  // set the bank
  enc_setBank(address);
  // do the write
  

  enc_writeOp(ENC624J600_WRITE_CONTROL_REGISTER, address&0x1F,(uint8_t*) &data , 2);

}



void Enc424J600Network::writePointer(uint8_t instruction, uint16_t address, bool keepEnabled)
{
	CSACTIVE;
	
	SPI.transfer(instruction);
	SPI.transfer( address&0x00FF);
	SPI.transfer( address>>8);

  if (!keepEnabled)
	  CSPASSIVE;

}

/* select bank and write control register
will always put CS high to end operation
*/
uint8_t Enc424J600Network::readControlRegister(uint8_t address)
{
  uint16_t retval;
  // set the bank
  enc_setBank(address,true);
  // do the write
  enc_readOp(ENC624J600_READ_CONTROL_REGISTER, address&0x1F,(uint8_t*) &retval ,1);
  
  return retval;
}

/* select bank and read control register
will always put CS high to end operation
*/
uint16_t Enc424J600Network::readControlRegister16(uint8_t address)
{
  uint16_t retval;
  // set the bank
  enc_setBank(address,true);
  // do the write
  enc_readOp(ENC624J600_READ_CONTROL_REGISTER, address&0x1F,(uint8_t*) &retval ,2);
  
  return retval;
}


/* select bank and write control register bit
will always put CS high to end operation
*/
void Enc424J600Network::writeBitField(uint8_t address, uint8_t data) {
  // set the bank
  enc_setBank(address,true);
  // do the write
  enc_writeOp(ENC624J600_BIT_FIELD_SET, address, &data,1);
}


void Enc424J600Network::enc_setBank(uint8_t address, bool keepEnabled)
{
  // set the bank (if needed)
  if (((address & BANK_MASK) != bank) && ((address & BANK_MASK) != 0xE0))
  {
    // set the bank

    bank = (address & BANK_MASK);
    
    switch((bank)>>5){
			case 0 :
				enc_SBI(ENC624J600_BANK0_SELECT, keepEnabled);
				break;
			case 1 :
				enc_SBI(ENC624J600_BANK1_SELECT, keepEnabled);
				break;
			case 2 :
				enc_SBI(ENC624J600_BANK2_SELECT, keepEnabled);
				break;
			case 3 :
				enc_SBI(ENC624J600_BANK3_SELECT, keepEnabled);
				break;
		}
  }
}







void Enc424J600Network::init(uint8_t* macaddr)
{
  MemoryPool::init(); // 1 byte in between RX_STOP_INIT and pool to allow prepending of controlbyte

#ifdef ENC28J60DEBUG
  SerialUSB.println("ENC624J600Init");
#endif
  /* enable SPI */
	pinMode(ENC28J60_CONTROL_CS, OUTPUT); 
	digitalWrite(ENC28J60_CONTROL_CS, HIGH); 
  SPI.begin();
  
	//8.1 RESET
	//STEP ONE
	writeControlRegister16(EUDASTL,0x1234);

	//STEP TWO
	while(readControlRegister16(EUDASTL)!=0x1234)
	{
		writeControlRegister16(EUDASTL,0x1234);
	}
  
	//STEP THREE
	while(readControlRegister(ESTATH) & ESTAT_CLKRDY);

	//STEP FOUR
	// reset command
	enc_SBI(ENC624J600_ETH_RESET);

	//STEP FIVE
	delayMicroseconds(25);
		  
	//STEP SIX
	if (readControlRegister16(EUDASTL)==0x0000)
	{
		delayMicroseconds(265);		
		//8.2 CLKOUT Frequency
		// Arduino : 16MHz =>  COCON=0100 
		// We do not use the clkout
		//writeBitField( ECON2H,ECON2_COCON2>>8);
		//8.3 reception
		nextPacketPtr = RXSTART_INIT;
		writeControlRegister16(ERXSTL, RXSTART_INIT);

		
		writeControlRegister16(ERXTAILL, RXSTOP_INIT);
			
 		// USER buffer : EUDAST Pointer at a higher memory address relative to the end address.
 		writeControlRegister16(EUDASTL, 0x5FFF);
 		writeControlRegister16(EUDANDL, 0x5FFF);
			/*
#ifndef IPV6
 		// fill user-defined area with arpResponse
		// only for IPv4
 		unsigned char arpResponse[ARP_RESPONSE_PACKET_SIZE-MAC_ADDR_SIZE] = { 
			0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
			0x08, 0x06,
			0x00, 0x01,
			0x08, 0x00,
			0x06,
			0x04,
			0x00, 0x02,
			ENC624J600_MAC0, ENC624J600_MAC1, ENC624J600_MAC2, ENC624J600_MAC3, ENC624J600_MAC4, ENC624J600_MAC5,
			IP_ADDR_0, IP_ADDR_1, IP_ADDR_2, IP_ADDR_3,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00
		};

 		writeRegGPBuffer(USER_START_INIT,arpResponse,ARP_PACKET_SIZE-MAC_ADDR_SIZE);
#endif*/

		//8.4 RAF

		//8.5 RECEIVE FILTER TODO!!!
#ifdef IPV6
		// We need multicast for neighbor sollicitation
	d	writeControlRegister(ERXFCONL,ERXFCON_CRCEN|ERXFCON_RUNTEN|ERXFCON_UCEN|ERXFCON_MCEN);
#else
		// crc ERROR FILTER => disabled
		// frames shorter than 64 bits => disabled
		// CRC error rejection => enabled
		// Unicast collection filter => enabled
		// Not me unicast filter => disabled
		// Multicast collection filter 
		writeControlRegister(ERXFCONL,ERXFCON_CRCEN|ERXFCON_RUNTEN|ERXFCON_BCEN|ERXFCON_UCEN);//ERXFCON_CRCEN|ERXFCON_RUNTEN|ERXFCON_UCEN);
		// brodcast collection filter => enabled
		// Hash table collection filter.. c
		// Magic packet => disabled TODO
		// PAttern
#endif
			
#ifdef CHECKSUM_PATTERN
		// Checksum only for IPv4
		//window 
		writeControlRegister16(EPMOL,0x0000);
			
		//pattern
		writeControlRegister(EPMM1L, WINDOW_PATTERN_0);
		writeControlRegister(EPMM1H, WINDOW_PATTERN_1);
		writeControlRegister(EPMM2L, WINDOW_PATTERN_2);
		writeControlRegister(EPMM2H, WINDOW_PATTERN_3);
		writeControlRegister(EPMM3L, WINDOW_PATTERN_4);
		writeControlRegister(EPMM3H, WINDOW_PATTERN_5);
		writeControlRegister(EPMM4L, WINDOW_PATTERN_6);
		writeControlRegister(EPMM4H, WINDOW_PATTERN_7);
		//CheckSum
		writeControlRegister(EPMCSL,CHECKSUM_PATTERN&0xFF);
		writeControlRegister(EPMCSH,CHECKSUM_PATTERN>>8);
#endif
		//exact pattern
		//writeControlRegister(ERXFCONH,0x01);
					      
		// 8.6 MAC initialization ...
		//flow control ???
		writeBitField( MACON2L, MACON2_TXCRCEN|MACON2_PADCFG0|MACON2_PADCFG1|MACON2_PADCFG2|MACON2_HFRMEN);

		writeControlRegister16(MAMXFLL, MAX_FRAMELEN);



		writeControlRegister(MAADR1L, macaddr[0]);
		writeControlRegister(MAADR1H, macaddr[1]);
		writeControlRegister(MAADR2L, macaddr[2]);
		writeControlRegister(MAADR2H, macaddr[3]);
		writeControlRegister(MAADR3L, macaddr[4]);
		writeControlRegister(MAADR3H, macaddr[5]);
		

		///writeControlRegister(ECON2H, 0xa0);
		//writeControlRegister(ECON2H, 224);
		
		//8.7 PHY initialization 
		// auto-negotiation ?
		//	ENC624J600PhyWrite(PHANA,0x05E1);
		// 8.8 OTHER considerations
		//half-duplex mode
			//writeBitField( MACON2H,MACON2_DEFER|MACON2_BPEN|MACON2_NOBKOFF);$
			
		// enable interuption
		//writeControlRegister(EIEL,0x40);
		//writeControlRegister(EIEH,0x80);
		// configuration LED
		//		ENC624J600WCRU(EIDLEDH, 0x06);

			 //	ENC624J600PhyWrite(PHCON1,PHCON1_PFULDPX);
		// enable reception
		enc_SBI(ENC624J600_ENABLE_RX);

	}
	else
	{
	  SerialUSB.println ("Error in initialization!");
		// Oops something went wrong
	}
#ifdef ENC28J60DEBUG
	SerialUSB.println ("ENC624J600Init complete");
#endif
}

uint16_t Enc424J600Network::readPacket(memhandle handle, memaddress position, uint8_t* buffer, uint16_t len)
{ 

  #ifdef ENC28J60DEBUG
  memblock *packet = handle == UIP_RECEIVEBUFFERHANDLE ? &receivePkt : &blocks[handle];
  memaddress start = handle == UIP_RECEIVEBUFFERHANDLE && packet->begin + position > RXSTOP_INIT ? packet->begin + position-RXSTOP_INIT+RXSTART_INIT : packet->begin + position;
	SerialUSB.print ("readPacket (");
	SerialUSB.print (handle);
	SerialUSB.print (") [");
	SerialUSB.print (start,HEX);
	SerialUSB.print ("-");
	SerialUSB.print (start+len,HEX);
	SerialUSB.print ("] ");
	SerialUSB.print (len);
	SerialUSB.print (" bytes: ");
#endif
  len = setReadPtr(handle, position, len); 
  readBuffer(len, buffer);
  return len;
}

int prev = 0;
memhandle
Enc424J600Network::receivePacket()
{
  uint8_t rxstat;
  uint16_t len;
  // check if a packet has been received and buffered
  //if( !(readControlRegister(EIR) & EIR_PKTIF) ){
  // The above does not work. See Rev. B4 Silicon Errata point 6.

  //SerialUSB.println(readControlRegister(ESTATL));

  if( (readControlRegister(EIRL) & EIR_PKTIF) ){

      uint16_t readPtr = nextPacketPtr+8 > RXSTOP_INIT ? nextPacketPtr+8-RXSTOP_INIT+RXSTART_INIT : nextPacketPtr+8 ;
      // Set the read pointer to the start of the received packet
     // writeControlRegister16(ENC624J600_WRITE_ERXRDPT, nextPacketPtr);
      writePointer(ENC624J600_WRITE_ERXRDPT, nextPacketPtr,true);
      
      SPI.transfer(ENC624J600_READ_ERXDATA);
      // read the next packet pointer
      nextPacketPtr = SPI.transfer( 0x00);
      nextPacketPtr |= SPI.transfer( 0x00) << 8;
      
      // read the packet length (see datasheet page 43)
      len = SPI.transfer( 0x00);
      len |= SPI.transfer( 0x00) << 8;
      len -= 4; //remove the CRC count
      // read the receive status (see datasheet page 43)
      //rxstat = enc_readOp(ENC624J600_READ_ERXDATA, 0);
     // rxstat |= enc_readOp(ENC624J600_READ_ERXDATA, 0) << 8;
      CSPASSIVE;
      
#ifdef ENC28J60DEBUG
      SerialUSB.print("receivePacket [");
      SerialUSB.print(readPtr,HEX);
      SerialUSB.print("-");
      SerialUSB.print((readPtr+len) % (RXSTOP_INIT+1),HEX);
      SerialUSB.print("], next: ");
      SerialUSB.print(nextPacketPtr,HEX);
      SerialUSB.print(", stat: ");
      SerialUSB.print(rxstat,HEX);
      SerialUSB.print(", count: ");
      SerialUSB.println(len);
      //SerialUSB.print(readControlRegister(EPKTCNT));
      //SerialUSB.print(" -> ");
      //SerialUSB.println((rxstat & 0x80)!=0 ? "OK" : "failed");
#endif
      // decrement the packet counter indicate we are done with this packet
      
      // check CRC and symbol errors (see datasheet page 44, table 7-3):
      // The ERXFCON.CRCEN is set by default. Normally we should not
      // need to check this.
		setERXRDPT();

		writeControlRegister16(ERXTAILL, readPtr-1);
		enc_SBI(ENC624J600_SETPKTDEC);

		if (len > 200) {
			SerialUSB.println("Packet discarded!");

			return (NOBLOCK);
		}
				receivePkt.begin = readPtr;
			receivePkt.size = len;	



		return UIP_RECEIVEBUFFERHANDLE;
   // Move the RX read pointer to the start of the next received packet
      // This frees the memory we just read out
      
    }
  return (NOBLOCK);
}

void
Enc424J600Network::setERXRDPT()
{

  writePointer(ENC624J600_WRITE_ERXRDPT, nextPacketPtr == RXSTART_INIT ? RXSTOP_INIT : nextPacketPtr-1);
}

memaddress
Enc424J600Network::blockSize(memhandle handle)
{
  return handle == NOBLOCK ? 0 : handle == UIP_RECEIVEBUFFERHANDLE ? receivePkt.size : blocks[handle].size;
}




void
Enc424J600Network::sendPacket(memhandle handle)
{
  memblock *packet = &blocks[handle];
  uint16_t start = packet->begin;
  uint16_t end = start + packet->size-1;

  // backup data at control-byte position
  uint8_t data = readByteTX(start-1);
  // write control-byte (if not 0 anyway)
  if (data)
    writeByte(start-1, 0);

#ifdef ENC28J60DEBUG
  SerialUSB.print("sendPacket(");
  SerialUSB.print(handle);
  SerialUSB.print(") [");
  SerialUSB.print(start+1,HEX);
  SerialUSB.print("-");
  SerialUSB.print(end+1,HEX);
  SerialUSB.print("]: ");
  for (uint16_t i=start; i<=end; i++)
    {
      SerialUSB.print(readByteTX(i),HEX);
      SerialUSB.print(" ");
    }
  SerialUSB.println();
#endif

  // TX start
  //writeControlRegister16(ETXSTL, start);
  // Set the TXND pointer to correspond to the packet size given
  //writeControlRegister16(ETXLENL, packet->size);
  
    writeControlRegister(ETXSTL,(start)&0x00FF);
    writeControlRegister(ETXSTH,(start)>>8);
    // Set the TXND pointer to correspond to the packet size given
    writeControlRegister(ETXLENL, (packet->size)&0x00FF);
    writeControlRegister(ETXLENH, (packet->size)>>8);


 
    // send the contents of the transmit buffer onto the network
    enc_SBI(ENC624J600_SETTXRTS);

    // send the contents of the transmit buffer onto the network
    writeBitField( ECON1, ECON1_TXRTS);
    // Reset the transmit logic problem. See Rev. B4 Silicon Errata point 12.
  /*if( (readControlRegister(EIR) & EIR_TXERIF) )
    {
      enc_writeOp(ENC624J600_BIT_FIELD_CLEAR, ECON1, ECON1_TXRTS);
    }*/



  //restore data on control-byte position
  if (data)
    writeByte(start-1, data);

	#ifdef ENC28J60DEBUG
SerialUSB.println("Sendpacket Done");
#endif
}

uint16_t
Enc424J600Network::setReadPtr(memhandle handle, memaddress position, uint16_t len)
{
  memblock *packet = handle == UIP_RECEIVEBUFFERHANDLE ? &receivePkt : &blocks[handle];
  memaddress start = handle == UIP_RECEIVEBUFFERHANDLE && packet->begin + position > RXSTOP_INIT ? packet->begin + position-RXSTOP_INIT+RXSTART_INIT : packet->begin + position;

  writePointer(ENC624J600_WRITE_ERXRDPT,start,true);


  if (len > packet->size - position)
    len = packet->size - position;
  return len;
}




uint16_t
Enc424J600Network::writePacket(memhandle handle, memaddress position, uint8_t* buffer, uint16_t len)
{


//SerialUSB.print("handle: ");
//SerialUSB.print(handle);

  memblock *packet = &blocks[handle];
  uint16_t start = packet->begin + position;
//SerialUSB.print(" start: 0x");
//SerialUSB.print(start,HEX);
//SerialUSB.print(" packetsize: ");
//SerialUSB.print(packet->size);

writePointer(ENC624J600_WRITE_EGPWRPT,start);
  if (len > packet->size - position)
    len = packet->size - position;
    
//    SerialUSB.print(" len: ");
//SerialUSB.println(len);
  writeBuffer(len, buffer);
  return len;
}



uint8_t Enc424J600Network::readByteTX(uint16_t addr)
{

  uint8_t retval;

writePointer(ENC624J600_WRITE_EGPRDPT, addr,false);

  CSACTIVE;
  // issue read command
  SPI.transfer(ENC624J600_READ_EGPDATA);

  // read data
  retval = SPI.transfer( 0x00);

  CSPASSIVE;
  return retval;
}


uint8_t Enc424J600Network::readByte(uint16_t addr)
{

  uint8_t retval;

writePointer(ENC624J600_WRITE_ERXRDPT, addr,false);

  CSACTIVE;
  // issue read command
  SPI.transfer(ENC624J600_READ_ERXDATA);

  // read data
  retval = SPI.transfer( 0x00);

  CSPASSIVE;
  return retval;
}

void Enc424J600Network::writeByte(uint16_t addr, uint8_t data)
{
  uint8_t retval;

writePointer(ENC624J600_WRITE_EGPWRPT, addr,false);


  CSACTIVE;
  // issue read command
  SPI.transfer(ENC624J600_WRITE_EGPDATA);

  // read data
  SPI.transfer(data);

  CSPASSIVE;

}

void
Enc424J600Network::copyPacket(memhandle dest_pkt, memaddress dest_pos, memhandle src_pkt, memaddress src_pos, uint16_t len, uint8_t buffertype)
{
  memblock *dest = &blocks[dest_pkt];
  memblock *src = src_pkt == UIP_RECEIVEBUFFERHANDLE ? &receivePkt : &blocks[src_pkt];
  memaddress start = src_pkt == UIP_RECEIVEBUFFERHANDLE && src->begin + src_pos > RXSTOP_INIT ? src->begin + src_pos-RXSTOP_INIT+RXSTART_INIT : src->begin + src_pos;
  mempool_block_move(dest->begin+dest_pos,start,len,buffertype==0? RXSTART_INIT: TXSTART_INIT,buffertype==0? RXSTOP_INIT: TXSTOP_INIT);
  // Move the RX read pointer to the start of the next received packet
  // This frees the memory we just read out
  setERXRDPT();
}



void
Enc424J600Network::mempool_block_move(memaddress dest, memaddress src, memaddress len, uint16_t bufstart, uint16_t bufend)
{
//void
//Enc424J600Network::memblock_mv_cb(uint16_t dest, uint16_t src, uint16_t len)
//{
  //as ENC28J60 DMA is unable to copy single bytes:
/* SerialUSB.print("block copy action, src: 0x");
    SerialUSB.print(src, HEX);
    SerialUSB.print(" dest: 0x");
    SerialUSB.print(dest, HEX);
    SerialUSB.print(" len: 0x");
    SerialUSB.println(len, HEX);
 */
 /* for (int i =0; i<len-1; i++) {
  Enc424J600Network::writeByte(dest+i-1,Enc424J600Network::readByte(src+i+1));
  }*/
  
  //Enc424J600Network::writeByte(dest+len-1,0x0A);
  //return;
  
  if (len == 1)
    {
      Enc424J600Network::writeByte(dest,Enc424J600Network::readByte(src));
    }
  else
    {
    
    int o = readControlRegister(ECON1L);
      while (o & ECON1_DMAST){
      
      o=readControlRegister(ECON1L);
      }  
      // calculate address of last byte
      //len += src - 1;
      // copy, no checksum
      writeBitField( ECON1L, ECON1_DMACPY | ECON1_DMANOCS);

      /*  1. Appropriately program the EDMAST, EDMAND
       and EDMADST register pairs. The EDMAST
       registers should point to the first byte to copy
       from, the EDMAND registers should point to the
       last byte to copy and the EDMADST registers
       should point to the first byte in the destination
       range. The destination range will always be
       linear, never wrapping at any values except from
       8191 to 0 (the 8-Kbyte memory boundary).
       Extreme care should be taken when
       programming the start and end pointers to
       prevent a never ending DMA operation which
       would overwrite the entire 8-Kbyte buffer.
       */

		writeControlRegister16(EUDASTL, bufstart);
 		writeControlRegister16(EUDANDL, bufend);
      Enc424J600Network::writeControlRegister16(EDMASTL, src);
      Enc424J600Network::writeControlRegister16(EDMADSTL, dest);

      //if ((src <= RXSTOP_INIT)&& (len > RXSTOP_INIT))len -= (RXSTOP_INIT-RXSTART_INIT);
      Enc424J600Network::writeControlRegister16(EDMALENL, len);

     /*   SerialUSB.print("DMA start pointer should be: ");
        SerialUSB.print(src);
        SerialUSB.print(" and is: ");
        
        SerialUSB.println(((Enc424J600Network::enc_readOp(ENC624J600_READ_CONTROL_REGISTER, EDMASTH) <<8) | Enc424J600Network::readOp(ENC624J600_READ_CONTROL_REGISTER, EDMASTL)));
        
        SerialUSB.print("DMA dest pointer should be: ");
        SerialUSB.print(dest);
        SerialUSB.print(" and is: ");
        
        SerialUSB.println(((Enc424J600Network::enc_readOp(ENC624J600_READ_CONTROL_REGISTER, EDMADSTH) <<8) | Enc424J600Network::enc_readOp(ENC624J600_READ_CONTROL_REGISTER, EDMADSTL)));
        
                SerialUSB.print("DMA len pointer should be: ");
        SerialUSB.print(len);
        SerialUSB.print(" and is: ");
        
        SerialUSB.println(((Enc424J600Network::enc_readOp(ENC624J600_READ_CONTROL_REGISTER, EDMALENH) <<8) | Enc424J600Network::enc_readOp(ENC624J600_READ_CONTROL_REGISTER, EDMALENL)));*/
               
      /*
       2. If an interrupt at the end of the copy process is
       desired, set EIE.DMAIE and EIE.INTIE and
       clear EIR.DMAIF.

       3. Verify that ECON1.CSUMEN is clear. */
      //writeControlRegister(ENC624J600_BIT_FIELD_CLEAR, ECON1L, 0x10);

      /* 4. Start the DMA copy by setting ECON1.DMAST. */
      writeBitField( ECON1L, ECON1_DMAST);

      // wait until runnig DMA is completed
      o = readControlRegister(ECON1L);
      while (o & ECON1_DMAST){
      
      o=readControlRegister(ECON1L);
      }      

		writeControlRegister16(EUDASTL, 0x5FFF);
 		writeControlRegister16(EUDANDL, 0x5FFF);
     
    }
}

void mempool_block_move_callback(memaddress dest, memaddress src, memaddress len) {
  Enc424J600Network::mempool_block_move(dest,src,len,0,0);
}

void
Enc424J600Network::freePacket()
{
    setERXRDPT();
}





void
Enc424J600Network::readBuffer(uint16_t len, uint8_t* data)
{
  CSACTIVE;
  // issue read command
  SPI.transfer(  ENC624J600_READ_ERXDATA);
  
  #ifdef ENC28J60DEBUG
    SerialUSB.print("Readbuffer: ");
  #endif
  while(len)
  {
    len--;
    // read data

    *data = SPI.transfer(0x00);
    #ifdef ENC28J60DEBUG
    SerialUSB.print(" ");
    SerialUSB.print(*data,HEX);
    #endif
    data++;
  }
  //*data='\0';
  CSPASSIVE;
  #ifdef ENC28J60DEBUG
  SerialUSB.println(" ");
  #endif
}

void
Enc424J600Network::writeBuffer(uint16_t len, uint8_t* data)
{
  CSACTIVE;
  // issue write command
	SPI.transfer(ENC624J600_WRITE_EGPDATA);
  //SerialUSB.print("writeBuffer: ");
	while(len--)
	{
    //SerialUSB.print(*data,HEX);
    //SerialUSB.print(" ");
    SPI.transfer( *data);
    data++;
	}
  //SerialUSB.println("");   
  CSPASSIVE;
}





void
Enc424J600Network::phyWrite(uint8_t address, uint16_t data)
{
  // set the PHY register address
  writeControlRegister(MIREGADRL, address);
  // write the PHY data
  writeControlRegister16(MIWRL, data);
  // wait until the PHY write completes
  while(readControlRegister(MISTATL) & MISTAT_BUSY){
    delayMicroseconds(15);
  }
}

uint16_t
Enc424J600Network::phyRead(uint8_t address)
{
  writeControlRegister(MIREGADRL,address);
  writeControlRegister(MICMDL, MICMD_MIIRD);
  // wait until the PHY read completes
  while(readControlRegister(MISTATL) & MISTAT_BUSY){
    delayMicroseconds(15);
  }  //and MIRDH
  writeControlRegister(MICMDL, 0);
  return (readControlRegister(MIRDL) | readControlRegister(MIRDH) << 8);
}

void
Enc424J600Network::clkout(uint8_t clk)
{
  //setup clkout: 2 is 12.5MHz:
 // writeControlRegister(ECOCON, clk & 0x7);
}

// read the revision of the chip:
uint8_t
Enc424J600Network::getrev(void)
{
  //return(readControlRegister(EREVID));
}

uint16_t
Enc424J600Network::chksum(uint16_t sum, memhandle handle, memaddress pos, uint16_t len)
{
  uint16_t t;
  len = setReadPtr(handle, pos, len)-1;

  // issue read command
  SPI.transfer(ENC624J600_READ_ERXDATA);

  uint16_t i;
  for (i = 0; i < len; i+=2)
  {
    // read data
    
    t = SPI.transfer(0x00) << 8;
    t += SPI.transfer(0x00);
    sum += t;
    if(sum < t) {
      sum++;            /* carry */
    }
  }
  if(i == len) {

    t = (SPI.transfer(0x00) << 8) + 0;
    sum += t;
    if(sum < t) {
      sum++;            /* carry */
    }
  }
  CSPASSIVE;

  /* Return sum in host byte order. */
  return sum;
}

void
Enc424J600Network::powerOff()
{
  /*enc_writeOp(ENC624J600_BIT_FIELD_CLEAR, ECON1, ECON1_RXEN);
  delay(50);
  writeBitField( ECON2, ECON2_VRPS);
  delay(50);
  writeBitField( ECON2, ECON2_PWRSV);*/
}

void
Enc424J600Network::powerOn()
{
  /*enc_writeOp(ENC624J600_BIT_FIELD_CLEAR, ECON2, ECON2_PWRSV);
  delay(50);
  writeBitField( ECON1, ECON1_RXEN);
  delay(50);*/
}

bool
Enc424J600Network::linkStatus()
{
  return (phyRead(PHSTAT2) & 0x0400) > 0;
}

Enc424J600Network Enc28J60;
