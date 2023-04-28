//
// Created by Vyacheslav Karpukhin on 01.02.20.
//

#include "AEG1.h"
#include "crc.h"

void AEG1::off() {
  packet[11] = 0xf0;
  // 000 111 11 1 001 10 11
  f1 = 0, f2 = 7, f3 = 3, r1 = 1, r2 = 1, r3 = 2, r4 = 3;
  send();
}

void AEG1::on() {
  packet[11] = 0x30;
  // 100 101 11 1 100 00 11
  f1 = 4; f2 = 5; f3 = 3; r1 = 1; r2 = 4; r3 = 0; r4 = 3;
  send();
}

void AEG1::color_up() {
  packet[11] = 0x90;
  // 100 001 01 0 111 00 01
  f1 = 4; f2 = 1; f3 = 1; r1 = 0; r2 = 7; r3 = 0; r4 = 1;
  send(200, 10);
}

void AEG1::color_down() {
  packet[11] = 0xb4;
  // 101 100 01 0 110 00 01
  f1 = 5; f2 = 4; f3 = 1; r1 = 0; r2 = 6; r3 = 0; r4 = 1;
  send(200, 10);
}

void AEG1::brightness_up() {
  packet[11] = 0xa0;
  // 000 011 11 1 111 10 11
  f1 = 0; f2 = 3; f3 = 3; r1 = 1; r2 = 7; r3 = 2; r4 = 3;
  send(200, 10);
}

void AEG1::brightness_down() {
  packet[11] = 0xb2;
  // 100 011 01 0 110 10 11
  f1 = 4; f2 = 3; f3 = 1; r1 = 0; r2 = 6; r3 = 2; r4 = 3;
  send(200, 10);
}

void AEG1::night_light() {
  packet[11] = 0xb8;
  // 001 100 01 1 110 01 01
  f1 = 1; f2 = 4; f3 = 1; r1 = 1; r2 = 6; r3 = 1; r4 = 1;
  send();
}

void AEG1::switch_color() {
  packet[11] = 0xb1;
  // 011 010 11 0 100 10 00
  f1 = 3; f2 = 2; f3 = 3; r1 = 0; r2 = 4; r3 = 2; r4 = 0;
  send();
}

uint8_t AEG1::receive() {
  uint16_t crc = crc16(0x8cfe, rcv_packet, sizeof(rcv_packet) - 2);
  if (crc == (rcv_packet[16] << 8u | rcv_packet[17])) {
    return rcv_packet[11];
  }
  return 0xff;
}

void AEG1::init_for_receive() {
  radio_init();

  NRF_RADIO->PACKETPTR = (uint32_t) (rcv_packet + 3);
  NRF_RADIO->EVENTS_END = 0;
  NRF_RADIO->INTENSET = RADIO_INTENSET_END_Set << RADIO_INTENSET_END_Pos;
  NRF_RADIO->TASKS_RXEN = 1;
}

void AEG1::radio_init() {
  NRF_RADIO->PACKETPTR = (uint32_t) (packet + 3);
  NRF_RADIO->BASE0 = packet[1] << 24u | packet[0] << 16u;
  NRF_RADIO->RXADDRESSES = RADIO_RXADDRESSES_ADDR0_Enabled << RADIO_RXADDRESSES_ADDR0_Pos;
  NRF_RADIO->PREFIX0 = packet[2] << RADIO_PREFIX0_AP0_Pos;
  NRF_RADIO->MODE = RADIO_MODE_MODE_Nrf_1Mbit;
  NRF_RADIO->PCNF1 = (sizeof(packet) - 3) << RADIO_PCNF1_STATLEN_Pos |
                     (sizeof(packet) - 3)  << RADIO_PCNF1_MAXLEN_Pos |
                     2 << RADIO_PCNF1_BALEN_Pos |
                     RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos;
}

void AEG1::send(int length, int delayMs) {
  AEG::send(length, delayMs * 1000);
}

void AEG1::next() {
  r4 -= 1;
  r4 &= 0b11u;
  if (r4 != 0b11u) {
    r3 += 1;
    r3 &= 0b11u;
    if (r3 == 0b10u) {
      r2 += 1;
      r2 &= 0b111u;
      if (r2 == 0b100u) {
        r1 += 1;
        r1 &= 0b1u;
      }
    }
  }
  f3 += 1;
  f3 &= 0b11u;
  if (f3 != 0b10u) {
    f2 += 1;
    f2 &= 0b111u;
    if (f2 == 0b100) {
      f1 += 1;
      f1 &= 0b111u;
    }
  }
  packet[14] = f1 << 5u | f2 << 2u | f3;
  packet[15] = r1 << 7u | r2 << 4u | r3 << 2u | r4;
  uint16_t crc = crc16(0x8cfe, packet, sizeof(packet) - 2);
  packet[16] = crc >> 8u;
  packet[17] = crc & 0xff;
}
