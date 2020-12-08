#include "LaCrosse.h"

/*
* Message Format:
*
* .- [0] -. .- [1] -. .- [2] -. .- [3] -. .- [4] -.
* |       | |       | |       | |       | |       |
* SSSS.DDDD DDN_.TTTT TTTT.TTTT WHHH.HHHH CCCC.CCCC
* |  | |     ||  |  | |  | |  | ||      | |       |
* |  | |     ||  |  | |  | |  | ||      | `--------- CRC
* |  | |     ||  |  | |  | |  | |`-------- Humidity
* |  | |     ||  |  | |  | |  | |
* |  | |     ||  |  | |  | |  | `---- weak battery
* |  | |     ||  |  | |  | |  |
* |  | |     ||  |  | |  | `----- Temperature T * 0.1
* |  | |     ||  |  | |  |
* |  | |     ||  |  | `---------- Temperature T * 1
* |  | |     ||  |  |
* |  | |     ||  `--------------- Temperature T * 10
* |  | |     | `--- new battery
* |  | `---------- ID
* `---- START = 9
*
*/

uint8_t LaCrosse::CalculateCRC(const uint8_t data[]) {
  int i, j;
  uint8_t res = 0;
  for (j = 0; j < FRAME_LENGTH - 1; j++) {
    uint8_t val = data[j];
    for (i = 0; i < 8; i++) {
      auto tmp = (uint8_t)((res ^ val) & 0x80);
      res <<= 1;
      if (0 != tmp) {
        res ^= 0x31;
      }
      val <<= 1;
    }
  }
  return res;
}

void LaCrosse::DecodeFrame(uint8_t *bytes, struct Frame *frame) {
  frame->IsValid = true;

  frame->CRC = bytes[4];
  if (frame->CRC != CalculateCRC(bytes)) {
    frame->IsValid = false;
  }

  // SSSS.DDDD DDN_.TTTT TTTT.TTTT WHHH.HHHH CCCC.CCCC
  frame->ID = 0;
  frame->ID |= (bytes[0] & 0xF) << 2;
  frame->ID |= (bytes[1] & 0xC0) >> 6;


  frame->Header = (bytes[0] & 0xF0) >> 4;
  if (frame->Header != 9) {
    frame->IsValid = false;
  }

  frame->NewBatteryFlag = (bytes[1] & 0x20) >> 5;

  frame->Bit12 = (bytes[1] & 0x10) >> 4;

  uint8_t bcd[3];
  bcd[0] = bytes[1] & 0xF;
  bcd[1] = (bytes[2] & 0xF0) >> 4;
  bcd[2] = (bytes[2] & 0xF);
  float t = 0;
  t += bcd[0] * 100.0;
  t += bcd[1] * 10.0;
  t += bcd[2] * 1.0;
  t = t / 10;
  t -= 40;
  frame->Temperature = t;

  frame->WeakBatteryFlag = (bytes[3] & 0x80) >> 7;

  frame->Humidity = bytes[3] & 0b01111111;
}
