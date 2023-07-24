#include "crc.h"

uint16_t crc16_light(uint16_t crc, const uint8_t *c, size_t len) {
  while (len--) {
    crc = crc << 8u ^ crctable[crc >> 8u ^ reflect_table[*c++]];
    crc &= 0xFFFF;
  }
  return reflect_table[crc >> 8u] << 8u | reflect_table[crc & 0xff];
}

uint16_t crc16_wmbus(const uint8_t *c, size_t len) {
  uint16_t crc = 0;
  while (len--) {
    crc = crc << 8u ^ wmbus_crctable[crc >> 8u ^ *c++];
  }
  return crc;
}
