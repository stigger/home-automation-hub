#ifndef _LACROSSE_h
#define _LACROSSE_h

#include "Arduino.h"


class LaCrosse {
public:
  struct Frame {
    byte  Header;
    byte  ID;
    bool  NewBatteryFlag;
    bool  Bit12;
    float Temperature;
    bool  WeakBatteryFlag;
    byte  Humidity;
    byte  CRC;
    bool  IsValid;
  };

  static const byte FRAME_LENGTH = 5;
  static byte CalculateCRC(byte data[]);

  static void DecodeFrame(byte *bytes, struct LaCrosse::Frame *frame);
};

#endif

