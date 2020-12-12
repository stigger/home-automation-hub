//
// Created by Vyacheslav Karpukhin on 01.02.20.
//

#include "AEG2.h"
#include "crc.h"

void AEG2::off() {
  button = 0x21;
  send(40, 4);
}

void AEG2::on() {
  button = 0x11;
  send(40, 4);
}

void AEG2::brightness_up(bool long_time) {
  button = 0x31;
  send(long_time ? 500 : 40, 1);
}

void AEG2::brightness_down(bool long_time) {
  button = 0x41;
  send(long_time ? 500 : 40, 1);
}

void AEG2::color_up(bool long_time) {
  button = 0x51;
  send(long_time ? 500 : 40, 4);
}

void AEG2::color_down(bool long_time) {
  button = 0x61;
  send(long_time ? 500 : 40, 4);
}

void AEG2::send(int length, int delayMs) {
  c1 = 0;
  c2 = 0;
  packet[53] = 0;
  AEG::send(length, delayMs);
}

void AEG2::radio_init() {
  NRF_RADIO->PACKETPTR = (uint32_t) packet;
  NRF_RADIO->MODE = RADIO_MODE_MODE_Nrf_2Mbit;
  NRF_RADIO->PCNF1 = sizeof(packet) << RADIO_PCNF1_STATLEN_Pos |
                     sizeof(packet) << RADIO_PCNF1_MAXLEN_Pos |
                     RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos;
}

void AEG2::next() {
  if (c1 == 5) {
    packet[53] = button;
    c2 = 5;
  }
  if (c1 > 5 && c1 < 20 && c2 < c2_limit) {
    c2++;
  }
  if (c1 == c2_limit + 17) {
    packet[53] = 0x0;
    c2 = 5;
  }
  if (c1 > 5 && packet[53] == 0x0 && c1 % 4 == 1) {
    c2++;
  }

  c1 &= 0b00111111u;
  packet[54] = c1 << 2u | 0b01u;
  packet[55] = c2;
  uint16_t crc = crc16(0, packet + 47, sizeof(packet) - 2 - 47);
  packet[61] = crc >> 8u;
  packet[62] = crc & 0xff;
  c1++;
}
