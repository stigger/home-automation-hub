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

#define cc1101_Select()  CC1101_SS = 0
#define cc1101_Deselect()  CC1101_SS = 1

const uint8_t LACROSSE_CFG[59] = {
        CC1101_FSCTRL1, 0x06,
        CC1101_FREQ2, 0x21,   // FREQ2     Frequency control word, high byte.
        CC1101_FREQ1, 0x65,   // FREQ1     Frequency control word, middle byte.
        CC1101_FREQ0, 0x6A,   // FREQ0     Frequency control word, low byte.

        CC1101_MCSM1, 0x00,   // always go into IDLE
        CC1101_MCSM0, 0x18,
        CC1101_FOCCFG, 0x16,
        CC1101_AGCCTRL2, 0x43,
        CC1101_AGCCTRL1, 0x68,
        CC1101_AGCCTRL1, 0x92,
        CC1101_FSCAL3, 0xe9,
        CC1101_FSCAL2, 0x2a,
        CC1101_FSCAL1, 0x00,
        CC1101_FSCAL0, 0x11,

        CC1101_IOCFG0, 0x01,

        CC1101_SYNC1, 0x2d,
        CC1101_SYNC0, 0xd4,

        CC1101_FIFOTHR, 2,     // 12 byte in RX - see page 72 of CC1101.pdf

        CC1101_PKTCTRL1, 0x00,   // PKTCTRL1  Packet automation control.
        CC1101_PKTCTRL0, 0x00,   // PKTCTRL0  Packet automation control

        CC1101_MDMCFG4, 0x89,   // MDMCFG4   Modem configuration.
        CC1101_MDMCFG3, 0x5C,   // MDMCFG3   Modem configuration.
        CC1101_MDMCFG2, 0x06,   // !! 05 !! MDMCFG2   Modem configuration.
        CC1101_MDMCFG1, 0x23,
        CC1101_MDMCFG0, 0xb9,

        CC1101_DEVIATN, 0x56,   // DEVIATN   Modem deviation setting (when FSK modulation is enabled).
        CC1101_PKTLEN, 5,

        0xff
};

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

CC1101::CC1101(bool light, PinName ss, PinName irq): CC1101_SS(ss), SPI(SPI_PSELMOSI1, SPI_PSELMISO1, SPI_PSELSCK1), light(light)
{
  packetLength = light ? 16 : 5;
  init();
  if (irq != NC) {
    intn = new InterruptIn(irq);
    intn->rise([this] {
        mbed_event_queue()->call([this] {
            unique_ptr<uint8_t[]> ptr = receiveData();
            if (ptr != nullptr) {
              callback(ptr.get());
            }
        });
    });
  }
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
  auto cfg = light ? LIGHT_CFG : LACROSSE_CFG;

  for (uint8_t i = 0; i<90; i += 2) {
    if (cfg[i] > 0x40)
      break;
    writeReg(cfg[i], cfg[i + 1]);
  }
}

void CC1101::init()
{
  SPI.format(8);                          // Initialize SPI interface
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

  for (size_t i = 0; i < 200; i++) {
    uint8_t state = readStatusReg(CC1101_MARCSTATE);
    if (state != MARCSTATE_TX) {
      break;
    }
    ThisThread::sleep_for(1ms);
  }

  return true;
}

unique_ptr<uint8_t[]> CC1101::receiveData() {
  uint8_t length = readReg(CC1101_RXBYTES, CC1101_STATUS_REGISTER) & 0x7f;
  if (length == packetLength) {
    unique_ptr<uint8_t[]> packet(new uint8_t[length]);
    readBurstReg(packet.get(), CC1101_RXFIFO, length);
    cmdStrobe(CC1101_SRX);
    return packet;
  }
  return nullptr;
}

void CC1101::setCallback(Callback<void(uint8_t[])> const &callback_) {
  CC1101::callback = callback_;
}