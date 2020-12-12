//
// Created by Vyacheslav Karpukhin on 01.02.20.
//

#ifndef NRF52_TEST_AEG1_H
#define NRF52_TEST_AEG1_H

#include "AEG.h"

class AEG1: AEG {
public:
    // first byte only for crc, actually sent as base address
    uint8_t packet[18] = {0x8e, 0xf0, 0xaa, 0xf4, 0xbe, 0xe1, 0x64, 0x92, 0x35, 0x8d, 0xee, 0x00, 0x43, 0xa4, 0x00, 0x00, 0x00, 0x00};
    uint8_t rcv_packet[18] = {0x8e, 0xf0, 0xaa, 0xf4, 0xbe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    void off();
    void on();
    void color_up();
    void color_down();
    void brightness_up();
    void brightness_down();
    void night_light();
    void switch_color();
    uint8_t receive();
    void init_for_receive();

private:
    void radio_init() override;
    void send(int length = 30, int delayMs = 4);
    void next() override;
    uint8_t f1{}, f2{}, f3{}, r1{}, r2{}, r3{}, r4{};
};


#endif //NRF52_TEST_AEG1_H
