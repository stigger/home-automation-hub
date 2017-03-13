/**
 * Copyright (c) 2011 panStamp <contact@panstamp.com>
 * Copyright (c) 2016 Tyler Sommer <contact@tylersommer.pro>
 * 
 * This file is part of the CC1101 project.
 * 
 * CC1101 is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * any later version.
 * 
 * CC1101 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with CC1101; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 
 * USA
 * 
 * Author: Daniel Berenguer
 * Creation date: 03/03/2011
 */

#include "cc1101.h"

/**
 * Macros
 */
#define CC1101_SS 8
// Select (SPI) CC1101
#define cc1101_Select()  digitalWrite(CC1101_SS, LOW)
// Deselect (SPI) CC1101
#define cc1101_Deselect()  digitalWrite(CC1101_SS, HIGH)
// Wait until SPI MISO line goes low
#define wait_Miso()  while(digitalRead(MISO)>0)
#define getGDO2state()  digitalRead(CC1101_GDO2)

const uint8_t PROGMEM MORITZ_CFG[59] = {
        0x00, 0x07, //IOCFG2: GDO2_CFG=7: Asserts when a packet has been received with CRC OK. De-asserts when the first byte is read from the RX FIFO
        0x02, 0x46, //IOCFG0
        0x04, 0xC6, //SYNC1
        0x05, 0x26, //SYNC0
        0x0B, 0x06, //FSCTRL1
        0x10, 0xC8, //MDMCFG4 DRATE_E=8,
        0x11, 0x93, //MDMCFG3 DRATE_M=147, data rate = (256+DRATE_M)*2^DRATE_E/2^28*f_xosc = (9992.599) 1kbit/s (at f_xosc=26 Mhz)
        0x12, 0x03, //MDMCFG2 see above
        0x13, 0x22,  //MDMCFG1 CHANSPC_E=2, NUM_PREAMBLE=2 (4 bytes), FEC_EN = 0 (disabled)
        0x15, 0x34, //DEVIATN
        0x17, 0x3F, //MCSM1: TXOFF=RX, RXOFF=RX, CCA_MODE=3:If RSSI below threshold unless currently receiving a packet
        0x18, 0x18, //MCSM0: PO_TIMEOUT=64, FS_AUTOCAL=2: When going from idle to RX or TX automatically
        0x19, 0x16, //FOCCFG
        0x1B, 0x43, //AGCTRL2
        0x21, 0x56, //FREND1
        0x25, 0x00, //FSCAL1
        0x26, 0x11, //FSCAL0
        0x0D, 0x21, //FREQ2
        0x0E, 0x65, //FREQ1
        0x0F, 0x6A, //FREQ0
        0x07, 0x0C, //PKTCTRL1
        0x16, 0x07, //MCSM2 RX_TIME = 7 (Timeout for sync word search in RX for both WOR mode and normal RX operation = Until end of packet) RX_TIME_QUAL=0 (check if sync word is found)
        0x20, 0xF8, //WORCTRL, WOR_RES=00 (1.8-1.9 sec) EVENT1=7 (48, i.e. 1.333 – 1.385 ms)
        0x1E, 0x87, //WOREVT1 EVENT0[high]
        0x1F, 0x6B, //WOREVT0 EVENT0[low]
        0x29, 0x59, //FSTEST
        0x2C, 0x81, //TEST2
        0x2D, 0x35, //TEST1
        0x3E, 0xC2, //?? Readonly PATABLE?
        0xff
};

/**
 * CC1101
 * 
 * Class constructor
 */
CC1101::CC1101()
{
  lastSendingMillis = 0;
}

/**
 * writeReg
 * 
 * Write single register into the CC1101 IC via SPI
 * 
 * 'regAddr'	Register address
 * 'value'	Value to be writen
 */
void CC1101::writeReg(byte regAddr, byte value) 
{
  cc1101_Select();                      // Select CC1101
  wait_Miso();                          // Wait until MISO goes low
  SPI.transfer(regAddr);                // Send register address
  SPI.transfer(value);                  // Send value
  cc1101_Deselect();                    // Deselect CC1101
}

/**
 * writeBurstReg
 * 
 * Write multiple registers into the CC1101 IC via SPI
 * 
 * 'regAddr'	Register address
 * 'buffer'	Data to be writen
 * 'len'	Data length
 */
void CC1101::writeBurstReg(byte regAddr, const byte* buffer, byte len)
{
  byte addr, i;
  
  addr = regAddr | WRITE_BURST;         // Enable burst transfer
  cc1101_Select();                      // Select CC1101
  wait_Miso();                          // Wait until MISO goes low
  SPI.transfer(addr);                   // Send register address
  
  for(i=0 ; i<len ; i++)
    SPI.transfer(buffer[i]);            // Send value

  cc1101_Deselect();                    // Deselect CC1101  
}

/**
 * cmdStrobe
 * 
 * Send command strobe to the CC1101 IC via SPI
 * 
 * 'cmd'	Command strobe
 */     
byte CC1101::cmdStrobe(byte cmd)
{
  byte res = 0;
  for (int i = 0; i < 5; i++) {
    cc1101_Select();                      // Select CC1101
    wait_Miso();                          // Wait until MISO goes low
    res = SPI.transfer(cmd);              // Send strobe command
    cc1101_Deselect();                    // Deselect CC1101
    if (!(res & CC1100_STATUS_CHIP_RDYn_BM)) break;
    delayMicroseconds(20);
  }
  return res;
}

/**
 * readReg
 * 
 * Read CC1101 register via SPI
 * 
 * 'regAddr'	Register address
 * 'regType'	Type of register: CC1101_CONFIG_REGISTER or CC1101_STATUS_REGISTER
 * 
 * Return:
 * 	Data byte returned by the CC1101 IC
 */
byte CC1101::readReg(byte regAddr, byte regType)
{
  byte addr, val;

  addr = regAddr | regType;
  cc1101_Select();                      // Select CC1101
  wait_Miso();                          // Wait until MISO goes low
  SPI.transfer(addr);                   // Send register address
  val = SPI.transfer(0x00);             // Read result
  cc1101_Deselect();                    // Deselect CC1101

  return val;
}

/**
 * readBurstReg
 * 
 * Read burst data from CC1101 via SPI
 * 
 * 'buffer'	Buffer where to copy the result to
 * 'regAddr'	Register address
 * 'len'	Data length
 */
void CC1101::readBurstReg(byte * buffer, byte regAddr, byte len) 
{
  byte addr, i;
  
  addr = regAddr | READ_BURST;
  cc1101_Select();                      // Select CC1101
  wait_Miso();                          // Wait until MISO goes low
  SPI.transfer(addr);                   // Send register address
  for(i=0 ; i<len ; i++)
    buffer[i] = SPI.transfer(0x00);     // Read result byte by byte
  cc1101_Deselect();                    // Deselect CC1101
}

/**
 * reset
 * 
 * Reset CC1101
 */
void CC1101::reset(void) 
{
  cc1101_Deselect();                    // Deselect CC1101
  delayMicroseconds(5);
  cc1101_Select();                      // Select CC1101
  delayMicroseconds(10);
  cc1101_Deselect();                    // Deselect CC1101
  delayMicroseconds(41);
  cc1101_Select();                      // Select CC1101

  wait_Miso();                          // Wait until MISO goes low
  SPI.transfer(CC1101_SRES);            // Send reset command strobe
  wait_Miso();                          // Wait until MISO goes low

  cc1101_Deselect();                    // Deselect CC1101

  setCCregs();                          // Reconfigure CC1101
  cmdStrobe(CC1101_SCAL);
  delay(4);
  cmdStrobe(CC1101_SRX);
}

/**
 * setCCregs
 * 
 * Configure CC1101 registers
 */
void CC1101::setCCregs(void) 
{
  for (uint8_t i = 0; i<60; i += 2) {
    if (pgm_read_byte( &MORITZ_CFG[i] )>0x40)
      break;
    writeReg(pgm_read_byte(&MORITZ_CFG[i]), pgm_read_byte(&MORITZ_CFG[i+1]));
  }
}

/**
 * init
 * 
 * Initialize CC1101 radio
 *
 * @param freq Carrier frequency
 * @param mode Working mode (speed, ...)
 */
void CC1101::init()
{
  SPI.begin();                          // Initialize SPI interface
  pinMode(CC1101_GDO2, INPUT);          // Config GDO0 as input
  pinMode(CC1101_SS, OUTPUT);

  reset();                              // Reset CC1101
}

/**
 * sendData
 * 
 * Send data packet via RF
 * 
 * 'packet'	Packet to be transmitted. First byte is the destination address
 *
 *  Return:
 *    True if the transmission succeeds
 *    False otherwise
 */
bool CC1101::sendData(const byte *data, bool longPreamble)
{
  if (readStatusReg(CC1101_MARCSTATE) != MARCSTATE_RX) {
    Serial.println("cc1101 error1");
    init();
    return false;
  }

//  if (lastSendingMillis) {
//    // 20ms silence between sends
//    unsigned long diff;
//    do {
//      diff = millis() - lastSendingMillis;
//    } while (diff > 0 && diff < 20);
//  }

  cmdStrobe(CC1101_SIDLE);
  cmdStrobe(CC1101_SFTX);
  cmdStrobe(CC1101_STX);

  uint8_t cnt = 0xff;
  while (cnt-- && readStatusReg(CC1101_MARCSTATE) != MARCSTATE_TX) {
    delayMicroseconds(10);
  }

  if (readStatusReg(CC1101_MARCSTATE) != MARCSTATE_TX) {
    Serial.println("cc1101 error2");
    Serial.println(readStatusReg(CC1101_MARCSTATE), HEX);
    init();
    return false;
  }

  if (longPreamble) {
    delay(1000);
  }

  writeBurstReg(CC1101_TXFIFO, data, data[0] + 1);

//  for (size_t i = 0; i < 200; i++) {
//    if (readStatusReg(CC1101_MARCSTATE) == MARCSTATE_RX) {
//      break;
//    }
//    if (readStatusReg(CC1101_MARCSTATE) != MARCSTATE_TX) {
//      break;
//    }
//    delay(1);
//  }
//
//  if (readStatusReg(CC1101_MARCSTATE) != MARCSTATE_RX) {
//    Serial.println("cc1101 error3");
//    init();
//    return false;
//  }

  lastSendingMillis = millis();

  return true;
}

/**
 * receiveData
 * 
 * Read data packet from RX FIFO
 *
 * 'packet'	Container for the packet received
 * 
 * Return:
 * 	Amount of bytes received
 */
byte CC1101::receiveData(CCPACKET * packet)
{
  if (getGDO2state()) {
    packet->length = readConfigReg(CC1101_RXFIFO) & 0x7f;
    if (packet->length > CCPACKET_DATA_LEN) {
      packet->length = 0;
    }
    else {
      readBurstReg(packet->data, CC1101_RXFIFO, packet->length);
      packet->rssi = readConfigReg(CC1101_RXFIFO);
      byte val = readConfigReg(CC1101_RXFIFO);
      packet->lqi = val & 0x7F;
      packet->crc_ok = bitRead(val, 7) != 0;
    }
  }
  else {
    packet->length = 0;
    if (readStatusReg(CC1101_MARCSTATE) == MARCSTATE_RXFIFO_OVERFLOW) {
      cmdStrobe(CC1101_SFRX);
      cmdStrobe(CC1101_SIDLE);
      cmdStrobe(CC1101_SRX);
    }
  }
  return packet->length;
}

