//
// Created by Vyacheslav Karpukhin on 01.02.20.
//

#ifndef NRF52_TEST_AEG_H
#define NRF52_TEST_AEG_H

#include "mbed.h"

class AEG {
protected:
    virtual void next() = 0;
    virtual void radio_init() = 0;

    virtual void send(int length, int delayUs);
};


#endif //NRF52_TEST_AEG_H
