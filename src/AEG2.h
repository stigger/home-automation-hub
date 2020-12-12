//
// Created by Vyacheslav Karpukhin on 01.02.20.
//

#ifndef NRF52_TEST_AEG2_H
#define NRF52_TEST_AEG2_H

#include "AEG.h"

class AEG2: AEG {
public:
    void off();
    void on();
    void brightness_up(bool long_time = true);
    void brightness_down(bool long_time = true);
    void color_up(bool long_time = true);
    void color_down(bool long_time = true);

protected:
    void send(int length, int delayMs) override;

private:
    uint8_t packet[63] = {0x8a, 0xc9, 0xf8, 0x11,
                          0x8a, 0xc9, 0xf8, 0x11,
                          0x8a, 0xc9, 0xf8, 0x11,
                          0x8a, 0xc9, 0xf8, 0x11,
                          0x8a, 0xc9, 0xf8, 0x11,
                          0x8a, 0xc9, 0xf8, 0x11,
                          0x8a, 0xc9, 0xf8, 0x11,
                          0x8a, 0xc9, 0xf8, 0x11,
                          0x8a, 0xc9, 0xf8, 0x11,
                          0x8a, 0xc9, 0xf8, 0x11,
                          0x8a, 0xc9, 0x37, 0x07, 0xee, 0x75,
                          0x10, 0x53, 0x54, 0x25, 0x72, 0x03,
                          0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00};

    void radio_init() override;

private:
    uint8_t c1{}, c2{}, c2_limit = 7, button{}; //c2_until seen from 5 to 13, seemingly random, doesn't appear to make a difference
    void next() override;
};

#endif //NRF52_TEST_AEG2_H
