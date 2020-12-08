#ifndef _LACROSSE_h
#define _LACROSSE_h

#include <cstdint>

class LaCrosse {
public:
  struct Frame {
    uint8_t  Header;
    uint8_t  ID;
    bool  NewBatteryFlag;
    bool  Bit12;
    float Temperature;
    bool  WeakBatteryFlag;
    uint8_t  Humidity;
    uint8_t  CRC;
    bool  IsValid;
  };

  static const uint8_t FRAME_LENGTH = 5;
  static uint8_t CalculateCRC(const uint8_t data[]);

  static void DecodeFrame(uint8_t *bytes, struct LaCrosse::Frame *frame);
};

#endif

