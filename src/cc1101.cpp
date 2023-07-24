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
#include <vector>

#define cc1101_Select()  CC1101_SS = 0
#define cc1101_Deselect()  CC1101_SS = 1

#define LONG_PACKET_CHUNK_SIZE 60
#define LONG_PACKET_SIZE_THRESHOLD 64
#define LONG_PACKET_FIFO_THRESHOLD         0x0e

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

const uint8_t WMBUS_TMODE_CFG[79] = {
        CC1101_SYNC1, 0x54,
        CC1101_SYNC0, 0x3D,
        CC1101_MCSM1, 0x00,

        CC1101_IOCFG2, 0x06,   // IOCFG2  GDO2 output pin configuration.
        CC1101_IOCFG0, 0x00,   // IOCFG0  GDO0 output pin configuration.

        CC1101_FSCTRL1, 0x08,   // FSCTRL1   Frequency synthesizer control.
        CC1101_FSCTRL0, 0x00,   // FSCTRL0   Frequency synthesizer control.
        CC1101_FREQ2, 0x21,   // FREQ2     Frequency control word, high byte.
        CC1101_FREQ1, 0x6B,   // FREQ1     Frequency control word, middle byte.
        CC1101_FREQ0, 0xD0,   // FREQ0     Frequency control word, low byte.
        CC1101_MDMCFG4, 0x5C,   // MDMCFG4   Modem configuration.  - 103 kBaud
        CC1101_MDMCFG3, 0x04,   // MDMCFG3   Modem configuration.
        CC1101_MDMCFG2, 0x06,   // !! 05 !! MDMCFG2 Modem configuration.
        CC1101_MDMCFG1, 0x22,   // MDMCFG1   Modem configuration.
        CC1101_MDMCFG0, 0xF8,   // MDMCFG0   Modem configuration.
        CC1101_CHANNR, 0x00,   // CHANNR    Channel number.
        CC1101_DEVIATN, 0x44,   // DEVIATN   Modem deviation setting (when FSK modulation is enabled).
        CC1101_FREND1, 0xB6,   // FREND1    Front end RX configuration.
        CC1101_FREND0, 0x10,   // FREND0    Front end RX configuration.
        CC1101_MCSM0, 0x18,   // MCSM0     Main Radio Control State Machine configuration.
        CC1101_FOCCFG, 0x2E,   // FOCCFG    Frequency Offset Compensation Configuration.
        CC1101_BSCFG, 0xBF,   // BSCFG     Bit synchronization Configuration.
        CC1101_AGCCTRL2, 0x43,   // AGCCTRL2  AGC control.
        CC1101_AGCCTRL1, 0x09,   // AGCCTRL1  AGC control.
        CC1101_AGCCTRL0, 0xB5,   // AGCCTRL0  AGC control.
        CC1101_FSCAL3, 0xEA,   // FSCAL3    Frequency synthesizer calibration.
        CC1101_FSCAL2, 0x2A,   // FSCAL2    Frequency synthesizer calibration.
        CC1101_FSCAL1, 0x00,   // FSCAL1    Frequency synthesizer calibration.
        CC1101_FSCAL0, 0x1F,   // FSCAL0    Frequency synthesizer calibration.
        CC1101_FSTEST, 0x59,   // FSTEST    Frequency synthesizer calibration.
        CC1101_TEST2, 0x81,   // TEST2     Various test settings.
        CC1101_TEST1, 0x35,   // TEST1     Various test settings.
        CC1101_TEST0, 0x09,   // TEST0     Various test settings.
        CC1101_PKTCTRL1, 0x00,   // !! 00 !! PKTCTRL1  Packet automation control.
        CC1101_PKTCTRL0, 0x00,   // PKTCTRL0  Packet automation control.
        CC1101_ADDR, 0x00,   // ADDR      Device address.
        CC1101_PKTLEN, 0xFF,    // PKTLEN    Packet length.

        CC1101_PATABLE, 0xC2,  // PATABLE
        0xff
};

CC1101::CC1101(Mode mode, PinName ss, PinName irq, Callback<void(uint16_t, std::shared_ptr<uint8_t[]>)> dataHandler): CC1101_SS(ss), SPI(p20, p16, p18), mode(mode), dataHandler(dataHandler)
{
  queue = mbed_highprio_event_queue();
  init();
  if (irq != NC) {
    intn = new InterruptIn(irq);
    intn->rise([this] {
        if (this->dataHandler == nullptr || canceled) return;

        queue->call([this] {
            while (intn->read()) {
              auto [len, data] = receiveData();
              std::shared_ptr<uint8_t[]> data_ptr = std::move(data);
              this->dataHandler(len, data_ptr);
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

  ThisThread::sleep_for(1ms);
  setCCregs(mode);                          // Reconfigure CC1101
  cmdStrobe(CC1101_SCAL);

  setPacketLength(mode == Light ? 16 : mode == LaCrosse ? 5 : 3);
  resetRxMode();
}

void CC1101::resetRxMode() {
  cmdStrobe(CC1101_SIDLE);
  cmdStrobe(CC1101_SFRX);
  cmdStrobe(CC1101_SRX);

  longPacket = false;
  canceled = false;
}

void CC1101::setCCregs(Mode _mode)
{
  auto cfg = _mode == Light ? LIGHT_CFG : _mode == LaCrosse ? LACROSSE_CFG : WMBUS_TMODE_CFG;

  for (uint8_t i = 0; i<90; i += 2) {
    if (cfg[i] > 0x40)
      break;
    writeReg(cfg[i], cfg[i + 1]);
  }
}

void CC1101::setMode(Mode _mode) {
  mode = None;
  setCCregs(_mode);
  setPacketLength(_mode == wMBus ? 3 : 5);
  resetRxMode();
  mode = _mode;
}

CC1101::Mode CC1101::getMode() {
  return mode;
}

void CC1101::init()
{
  SPI.format(8);                          // Initialize SPI interface
  SPI.frequency(4000000);
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

unique_ptr<uint8_t[]> CC1101::readBytes(int len) {
  unique_ptr<uint8_t[]> packet(new uint8_t[len]);
  readBurstReg(packet.get(), CC1101_RXFIFO, len);
  if (mode != wMBus) {
    cmdStrobe(CC1101_SRX);
  }
  return packet;
}

tuple<uint16_t, unique_ptr<uint8_t[]>> CC1101::receiveData() {
  int len = longPacket ? (LONG_PACKET_CHUNK_SIZE - 1) : longPacketPending > 0 ? longPacketPending : packetLength;
  unique_ptr<uint8_t[]> packet = readBytes(len);
  if (longPacket) {
    longPacketPending -= len;
    if (longPacketPending < len) {
      schedulePacketEndCheck(false);
    }
  }
  return {len, std::move(packet)};
}

void CC1101::schedulePacketEndCheck(bool later) {
  if (later) {
    queue->call_in(1ms, callback(this, &CC1101::checkPacketEnd));
  }
  else {
    queue->call(callback(this, &CC1101::checkPacketEnd));
  }
}

void CC1101::checkPacketEnd() {
  uint8_t i = readStatusReg(CC1101_PKTSTATUS);
  bool gdo2 = (i & 1 << 2) != 0;
  if (gdo2) {
    schedulePacketEndCheck(true);
  }
  else {
    std::shared_ptr<uint8_t[]> data_ptr = readBytes(longPacketPending);
    dataHandler(longPacketPending, data_ptr);
    packetEventsCounter++;
    setPacketLength(3);
    resetRxMode();
  }
}

void CC1101::setPacketLength(uint8_t _packetLength, uint8_t alreadyReceived) {
  packetLength = _packetLength;
  longPacketPending = packetLength - alreadyReceived;

  if (packetLength < LONG_PACKET_SIZE_THRESHOLD) {
    if (alreadyReceived == 0) {
      writeReg(CC1101_FIFOTHR, packetLength == 3 ? 0x00 : 0x02);
    }
    else {
      writeReg(CC1101_FIFOTHR, 0x0f);
      schedulePacketEndCheck(true);
    }
  }
  else {
    writeReg(CC1101_PKTLEN, packetLength);
    writeReg(CC1101_FIFOTHR, LONG_PACKET_FIFO_THRESHOLD);
    longPacket = true;
  }

  if (alreadyReceived != 0) {
    uint8_t c = ++packetEventsCounter;
    mbed_event_queue()->call_in(30ms, [c, this] {
      if (c == packetEventsCounter) {
        canceled = true;
        dataHandler(0, nullptr);
        setPacketLength(3);
        resetRxMode();
      }
    });
  }
}
