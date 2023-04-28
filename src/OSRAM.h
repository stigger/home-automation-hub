//
// Created by Vyacheslav Karpukhin on 25.12.20.
//

#ifndef NRF52_LIGHTS_OSRAM_H
#define NRF52_LIGHTS_OSRAM_H

#include "AEG.h"

class OSRAM: AEG {
public:
    void off();
    void on();
private:
    uint8_t packet_off[19] = { 0xaa,
                               0x52, 0x6c, 0x46, 0xb2,
                               0x2a, 0x57, 0xa4, 0x80,
                               0xc0, 0x89, 0x64, 0x00,
                               0xff, 0xff, 0x42, 0x4b,
                               0x54, 0xc2 };
    uint8_t packet_on[19] = { 0xaa,
                              0x52, 0x6c, 0x46, 0xb2,
                              0x2a, 0x51, 0xa4, 0x81,
                              0xc0, 0x89, 0x64, 0x00,
                              0xff, 0xff, 0x41, 0x49,
                              0x3b, 0x04 };
    uint8_t packet[19];

    void radio_init() override;
    void next() override;
    void send();
};


#endif //NRF52_LIGHTS_OSRAM_H
