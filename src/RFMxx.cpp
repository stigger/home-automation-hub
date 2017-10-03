#include <SPI.h>
#include "RFMxx.h"

// Usage with ESP8266: to use this lib with ESP8266, we need a: #define ESP8266 
// This can be done in some IDEs like visual micro on the project level.
// In Arduino IDE it's not possible.
// In this case, uncomment the following line:
//// #define ESP8266

void RFMxx::Receive() {
  if (ReadReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY) {
    for (int i = 0; i < PAYLOADSIZE; i++) {
      byte bt = GetByteFromFifo();
      m_payload[i] = bt;
    }
    m_payloadReady = true;
  }
}

void RFMxx::GetPayload(byte *data) {
  m_payloadReady = false;
  for (int i = 0; i < PAYLOADSIZE; i++) {
    data[i] = m_payload[i];
  }
}

byte RFMxx::ReadRSSI() {
  return ReadReg(REG_RSSIVALUE);
}


void RFMxx::SetDataRate(unsigned long dataRate) {
  word r = ((32000000UL + (dataRate / 2)) / dataRate);
  WriteReg(0x03, r >> 8);
  WriteReg(0x04, r & 0xFF);
}

void RFMxx::SetFrequency(unsigned long kHz) {
  unsigned long f = (((kHz * 1000) << 2) / (32000000L >> 11)) << 6;
  WriteReg(0x07, f >> 16);
  WriteReg(0x08, f >> 8);
  WriteReg(0x09, f);
}

void RFMxx::EnableReceiver(bool enable){
  if (enable) {
    WriteReg(REG_OPMODE, (ReadReg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
  }
  else {
    WriteReg(REG_OPMODE, (ReadReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
  }
  ClearFifo();
}

void RFMxx::EnableTransmitter(bool enable){
  if (enable) {
    WriteReg(REG_OPMODE, (ReadReg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
  }
  else {
    WriteReg(REG_OPMODE, (ReadReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
  }
}

byte RFMxx::GetByteFromFifo() {
  return ReadReg(0x00);
}

bool RFMxx::PayloadIsReady() {
  return m_payloadReady;
}


void RFMxx::ClearFifo() {
  WriteReg(REG_IRQFLAGS2, 16);
}

void RFMxx::PowerDown(){
  WriteReg(REG_OPMODE, (ReadReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
}

void RFMxx::InitializeMoritz() {
  digitalWrite(m_ss, HIGH);
  EnableReceiver(false);

  /* 0x01 */ WriteReg(REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY);
  /* 0x02 */ WriteReg(REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00);
  /* 0x05 */ WriteReg(REG_FDEVMSB, 0x01);
  /* 0x06 */ WriteReg(REG_FDEVLSB, 0x38);
  /* 0x11 */ WriteReg(REG_PALEVEL, RF_PALEVEL_PA0_OFF | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111);
  /* 0x13 */ WriteReg(REG_OCP, RF_OCP_OFF);
  /* 0x19 */ WriteReg(REG_RXBW, 0x4a);
             WriteReg(REG_AFCFEI, RF_AFCFEI_AFCAUTO_OFF | RF_AFCFEI_AFCAUTOCLEAR_OFF);
  /* 0x28 */ WriteReg(REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);
  /* 0x29 */ WriteReg(REG_RSSITHRESH, 160);
  /* 0x2E */ WriteReg(REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_4 | RF_SYNC_TOL_0);
  /* 0x2F */ WriteReg(REG_SYNCVALUE1, 0xC6);
  /* 0x30 */ WriteReg(REG_SYNCVALUE2, 0x26);
  /* 0x2F */ WriteReg(REG_SYNCVALUE3, 0xC6);
  /* 0x30 */ WriteReg(REG_SYNCVALUE4, 0x26);
  /* 0x37 */ WriteReg(REG_PACKETCONFIG1, RF_PACKET1_CRC_OFF | RF_PACKET1_CRCAUTOCLEAR_OFF);
  /* 0x38 */ WriteReg(REG_PAYLOADLENGTH, 0x1a);
  /* 0x3C */ WriteReg(REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE);
  /* 0x3D */ WriteReg(REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_OFF | RF_PACKET2_AES_OFF);
  /* 0x6F */ WriteReg(REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0);
  WriteReg(REG_LNA, RF_LNA_GAINSELECT_MAXMINUS24 | RF_LNA_ZIN_200);

  WriteReg(REG_BITRATEMSB, 0x0C);
  WriteReg(REG_BITRATELSB, 0x80);

  WriteReg(REG_PREAMBLEMSB, 0x00);
  WriteReg(REG_PREAMBLELSB, 0x04);

  WriteReg(REG_FRFMSB, 0xD9);
  WriteReg(REG_FRFMID, 0x13);
  WriteReg(REG_FRFLSB, 0x70);

  ClearFifo();
}


void RFMxx::InitializeLaCrosse() {
  digitalWrite(m_ss, HIGH);
  EnableReceiver(false);

  WriteReg(REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY);
  WriteReg(REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00);
  WriteReg(REG_FDEVMSB, RF_FDEVMSB_50000);
  WriteReg(REG_FDEVLSB, RF_FDEVLSB_50000);
  WriteReg(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111);
  WriteReg(REG_OCP, RF_OCP_OFF);
  WriteReg(REG_RXBW, 0x4a);
  WriteReg(REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);
  WriteReg(REG_RSSITHRESH, 160);
  WriteReg(REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0);
  WriteReg(REG_SYNCVALUE1, 0x2D);
  WriteReg(REG_SYNCVALUE2, 0xD4);
  WriteReg(REG_PACKETCONFIG1, RF_PACKET1_CRCAUTOCLEAR_OFF);
  WriteReg(REG_PAYLOADLENGTH, PAYLOADSIZE);
  WriteReg(REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE);
  WriteReg(REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_OFF| RF_PACKET2_AES_OFF);
  WriteReg(REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0);
  WriteReg(REG_LNA, RF_LNA_GAINSELECT_MAXMINUS24 | RF_LNA_ZIN_200);
  WriteReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01);


  SetFrequency(868300);
  SetDataRate(17241);

  ClearFifo();
}


byte RFMxx::spi8(byte value) {
  return SPI.transfer(value);
}


byte RFMxx::ReadReg(byte addr) {
  digitalWrite(m_ss, LOW);
  spi8(addr & 0x7F);
  byte regval = spi8(0);
  digitalWrite(m_ss, HIGH);
  return regval;

}

void RFMxx::WriteReg(byte addr, byte value) {
  digitalWrite(m_ss, LOW);
  spi8(addr | 0x80);
  spi8(value);
  digitalWrite(m_ss, HIGH);
}

bool RFMxx::IsConnected() {
  return m_isConnected;
}

RFMxx::RFMxx(byte ss) {
  SPI.begin();

  m_ss = ss;

  m_payloadReady = false;


  pinMode(m_ss, OUTPUT);

  digitalWrite(m_ss, HIGH);

  // No radio found until now
  m_isConnected = false;

  // Is there a RFM69 ?
  WriteReg(REG_PAYLOADLENGTH, 0xA);
  if (ReadReg(REG_PAYLOADLENGTH) == 0xA) {
    WriteReg(REG_PAYLOADLENGTH, 0x40);
    if (ReadReg(REG_PAYLOADLENGTH) == 0x40) {
      m_isConnected = true;
    }
  }
}


void RFMxx::SendArray(byte *data, byte length) {
  WriteReg(REG_PACKETCONFIG2, (ReadReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART);
  EnableReceiver(false);
  ClearFifo();
  digitalWrite(m_ss, LOW);
  spi8(REG_FIFO | 0x80);
  for (byte i = 0; i < length; i++) {
    spi8(data[i]);
  }
  digitalWrite(m_ss, HIGH);
  EnableTransmitter(true);
  unsigned long txStart = millis();
  while (!(ReadReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT) && millis() - txStart < 500);
  EnableTransmitter(false);
}

