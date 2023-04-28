//
// Created by Vyacheslav Karpukhin on 25.12.20.
//

#include "OSRAM.h"
#include "crc.h"

void OSRAM::off() {
  memcpy(packet, packet_off, sizeof(packet));
  send();
}

void OSRAM::on() {
  memcpy(packet, packet_on, sizeof(packet));
  send();
}

void OSRAM::send() {
  for (int i = 0; i < 16; i++) {
    AEG::send(16, 340);
    wait_us(5600);
  }
}


void OSRAM::radio_init() {
  NRF_RADIO->PACKETPTR = (uint32_t) (packet + 3);
  NRF_RADIO->BASE0 = reflect_table[packet[1]] << 24u | reflect_table[packet[0]] << 16u;
  NRF_RADIO->PREFIX0 = reflect_table[packet[2]] << RADIO_PREFIX0_AP0_Pos;
  NRF_RADIO->MODE = RADIO_MODE_MODE_Nrf_1Mbit;
  NRF_RADIO->PCNF1 = (sizeof(packet) - 3) << RADIO_PCNF1_STATLEN_Pos |
                     (sizeof(packet) - 3)  << RADIO_PCNF1_MAXLEN_Pos |
                     2 << RADIO_PCNF1_BALEN_Pos |
                     RADIO_PCNF1_ENDIAN_Big << RADIO_PCNF1_ENDIAN_Pos;
}

void OSRAM::next() {
}
