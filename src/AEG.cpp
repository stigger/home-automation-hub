//
// Created by Vyacheslav Karpukhin on 01.02.20.
//

#include "AEG.h"

void AEG::send(int length, int delayUs) {
  radio_init();
  for (int i = 0; i < length; i++) {
    next();
    NRF_RADIO->TASKS_TXEN = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0);
    NRF_RADIO->EVENTS_DISABLED = 0;
    if (delayUs < 1000) {
      wait_us(delayUs);
    }
    else {
      ThisThread::sleep_for(std::chrono::milliseconds(delayUs / 1000));
    }
  }
}
