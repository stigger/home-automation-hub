//
// Created by Vyacheslav Karpukhin on 01.02.20.
//

#include "AEG.h"

void AEG::send(int length, int delayMs) {
  radio_init();
  for (int i = 0; i < length; i++) {
    next();
    NRF_RADIO->TASKS_TXEN = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0);
    NRF_RADIO->EVENTS_DISABLED = 0;
    ThisThread::sleep_for(std::chrono::milliseconds(delayMs));
  }
}
