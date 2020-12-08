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
#include <mbed.h>

DigitalOut CC1101_SS(p18);
#define cc1101_Select()  CC1101_SS = 0
#define cc1101_Deselect()  CC1101_SS = 1
#define getGDO2state()  digitalRead(CC1101_GDO2)

const uint8_t LIGHT_CFG[89] = {
        0x02, 0x06, //IOCFG0
        0x03, 0x07,
        0x04, 0xD3, //SYNC1
        0x05, 0x91, //SYNC0
        0x0B, 0x09, //FSCTRL1
        0x0C, 0x00,
        0x0D, 0x5D, //FREQ2
        0x0E, 0x93, //FREQ1
        0x0F, 0xB1, //FREQ0
        0x10, 0x2D, //MDMCFG4
        0x11, 0x3B, //MDMCFG3
        0x12, 0x73, //MDMCFG2
        0x13, 0xA2,  //MDMCFG1
        0x14, 0xF8,
        0x0A, 0x10,
        0x15, 0x01,
        0x21, 0xB6, //FREND1
        0x22, 0x10,
        0x16, 0x07, //MCSM2 RX_TIME = 7 (Timeout for sync word search in RX for both WOR mode and normal RX operation = Until end of packet) RX_TIME_QUAL=0 (check if sync word is found)
        0x17, 0x30, //MCSM1
        0x18, 0x18, //MCSM0: PO_TIMEOUT=64, FS_AUTOCAL=2: When going from idle to RX or TX automatically
        0x19, 0x1D, //FOCCFG
        0x1A, 0x1C,
        0x1B, 0xC7, //AGCTRL2
        0x1C, 0x00,
        0x1D, 0xB2,
        0x23, 0xEA,
        0x24, 0x0A,
        0x25, 0x00, //FSCAL1
        0x26, 0x11, //FSCAL0
        0x27, 0x41,
        0x28, 0x00,
        0x1E, 0x87, //WOREVT1 EVENT0[high]
        0x1F, 0x6B, //WOREVT0 EVENT0[low]
        0x20, 0xF8, //WORCTRL, WOR_RES=00 (1.8-1.9 sec) EVENT1=7 (48, i.e. 1.333 – 1.385 ms)
        0x29, 0x59, //FSTEST
        0x2C, 0x88, //TEST2
        0x2D, 0x31, //TEST1
        0x2E, 0x0B,
        0x00, 0x29, //IOCFG2
        0x07, 0x04,
        0x08, 0x05,
        0x09, 0x01,
        0x06, 0xFF,
        0xff
};
//const uint8_t PROGMEM MORITZ_CFG[59] = {
//        0x00, 0x07, //IOCFG2: GDO2_CFG=7: Asserts when a packet has been received with CRC OK. De-asserts when the first byte is read from the RX FIFO
//        0x02, 0x46, //IOCFG0
//        0x04, 0xC6, //SYNC1
//        0x05, 0x26, //SYNC0
//        0x0B, 0x06, //FSCTRL1
//        0x10, 0xC8, //MDMCFG4 DRATE_E=8,
//        0x11, 0x93, //MDMCFG3 DRATE_M=147, data rate = (256+DRATE_M)*2^DRATE_E/2^28*f_xosc = (9992.599) 1kbit/s (at f_xosc=26 Mhz)
//        0x12, 0x03, //MDMCFG2 see above
//        0x13, 0x22,  //MDMCFG1 CHANSPC_E=2, NUM_PREAMBLE=2 (4 bytes), FEC_EN = 0 (disabled)
//        0x15, 0x34, //DEVIATN
//        0x17, 0x3F, //MCSM1: TXOFF=RX, RXOFF=RX, CCA_MODE=3:If RSSI below threshold unless currently receiving a packet
//        0x18, 0x18, //MCSM0: PO_TIMEOUT=64, FS_AUTOCAL=2: When going from idle to RX or TX automatically
//        0x19, 0x16, //FOCCFG
//        0x1B, 0x43, //AGCTRL2
//        0x21, 0x56, //FREND1
//        0x25, 0x00, //FSCAL1
//        0x26, 0x11, //FSCAL0
//        0x0D, 0x21, //FREQ2
//        0x0E, 0x65, //FREQ1
//        0x0F, 0x6A, //FREQ0
//        0x07, 0x0C, //PKTCTRL1
//        0x16, 0x07, //MCSM2 RX_TIME = 7 (Timeout for sync word search in RX for both WOR mode and normal RX operation = Until end of packet) RX_TIME_QUAL=0 (check if sync word is found)
//        0x20, 0xF8, //WORCTRL, WOR_RES=00 (1.8-1.9 sec) EVENT1=7 (48, i.e. 1.333 – 1.385 ms)
//        0x1E, 0x87, //WOREVT1 EVENT0[high]
//        0x1F, 0x6B, //WOREVT0 EVENT0[low]
//        0x29, 0x59, //FSTEST
//        0x2C, 0x81, //TEST2
//        0x2D, 0x35, //TEST1
//        0x3E, 0xC2, //?? Readonly PATABLE?
//        0xff
//};

CC1101::CC1101(): SPI(SPI_PSELMOSI0, SPI_PSELMISO0, SPI_PSELSCK0)
{
}

void CC1101::writeReg(uint8_t regAddr, uint8_t value)
{
  cc1101_Select();                      // Select CC1101
  SPI.write(regAddr);                // Send register address
  SPI.write(value);                  // Send value
  cc1101_Deselect();                    // Deselect CC1101
}

void CC1101::writeBurstReg(uint8_t regAddr, const uint8_t* buffer, uint8_t len)
{
  uint8_t addr, i;
  
  addr = regAddr | WRITE_BURST;         // Enable burst transfer
  cc1101_Select();                      // Select CC1101
  SPI.write(addr);                   // Send register address
  
  for(i=0 ; i<len ; i++)
    SPI.write(buffer[i]);            // Send value

  cc1101_Deselect();                    // Deselect CC1101  
}

uint8_t CC1101::cmdStrobe(uint8_t cmd)
{
  uint8_t res = 0;
  for (int i = 0; i < 5; i++) {
    cc1101_Select();                      // Select CC1101
    res = SPI.write(cmd);              // Send strobe command
    cc1101_Deselect();                    // Deselect CC1101
    if (!(res & CC1100_STATUS_CHIP_RDYn_BM)) break;
    wait_us(20);
  }
  return res;
}

uint8_t CC1101::readReg(uint8_t regAddr, uint8_t regType)
{
  uint8_t addr, val;

  addr = regAddr | regType;
  cc1101_Select();                      // Select CC1101
  SPI.write(addr);                   // Send register address
  val = SPI.write(0x00);             // Read result
  cc1101_Deselect();                    // Deselect CC1101

  return val;
}

void CC1101::readBurstReg(uint8_t * buffer, uint8_t regAddr, uint8_t len)
{
  uint8_t addr, i;
  
  addr = regAddr | READ_BURST;
  cc1101_Select();                      // Select CC1101
  SPI.write(addr);                   // Send register address
  for(i=0 ; i<len ; i++)
    buffer[i] = SPI.write(0x00);     // Read result byte by byte
  cc1101_Deselect();                    // Deselect CC1101
}

void CC1101::reset()
{
  cc1101_Deselect();                    // Deselect CC1101
  wait_us(5);
  cc1101_Select();                      // Select CC1101
  wait_us(10);
  cc1101_Deselect();                    // Deselect CC1101
  wait_us(41);
  cc1101_Select();                      // Select CC1101

  SPI.write(CC1101_SRES);            // Send reset command strobe

  cc1101_Deselect();                    // Deselect CC1101

  setCCregs();                          // Reconfigure CC1101
  cmdStrobe(CC1101_SCAL);
  ThisThread::sleep_for(4ms);
  cmdStrobe(CC1101_SIDLE);
  cmdStrobe(CC1101_SFRX);
  cmdStrobe(CC1101_SRX);
}

void CC1101::setCCregs()
{
  for (uint8_t i = 0; i<90; i += 2) {
    if (LIGHT_CFG[i]>0x40)
      break;
    writeReg(LIGHT_CFG[i], LIGHT_CFG[i+1]);
  }
}

void CC1101::init()
{
  SPI.format(8);                          // Initialize SPI interface
//  pinMode(CC1101_GDO2, INPUT);          // Config GDO0 as input

  reset();                              // Reset CC1101
}

bool CC1101::sendData(const uint8_t *data, bool longPreamble)
{
//  if (readStatusReg(CC1101_MARCSTATE) != MARCSTATE_RX) {
//    Serial.println("cc1101 error1");
//    init();
//    return false;
//  }

//  if (lastSendingMillis) {
//    // 20ms silence between sends
//    unsigned long diff;
//    do {
//      diff = millis() - lastSendingMillis;
//    } while (diff > 0 && diff < 20);
//  }

  cmdStrobe(CC1101_SIDLE);
  cmdStrobe(CC1101_SFTX);

//  uint8_t cnt = 0xff;
//  while (cnt-- && readStatusReg(CC1101_MARCSTATE) != MARCSTATE_TX) {
//    delayMicroseconds(10);
//  }

//  if (readStatusReg(CC1101_MARCSTATE) != MARCSTATE_TX) {
//    Serial.println("cc1101 error2");
//    Serial.println(readStatusReg(CC1101_MARCSTATE), HEX);
//    init();
//    return false;
//  }

  if (longPreamble) {
    ThisThread::sleep_for(1s);
  }

  writeBurstReg(CC1101_TXFIFO, data, 16);

  cmdStrobe(CC1101_STX);

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

  return true;
}

uint8_t CC1101::receiveData(CCPACKET * packet)
{
//  if (getGDO2state()) {
    packet->length = readConfigReg(CC1101_RXFIFO) & 0x7f;
    if (packet->length > CCPACKET_DATA_LEN) {
      packet->length = 0;
    }
    else {
      readBurstReg(packet->data, CC1101_RXFIFO, packet->length);
    }
//  }
//  else {
//    packet->length = 0;
//  }
  return packet->length;
}

