#ifndef __MAX_CRC_H
#define __MAX_CRC_H

#include <stdint.h>
#include <stddef.h>

uint16_t calc_cc1101_crc(uint8_t *buf, size_t len);
int fromhex(const char *in, uint8_t *out, uint8_t buflen);

#endif // __MAX_CRC_H

