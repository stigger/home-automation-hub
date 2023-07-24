//
// Created by Vyacheslav Karpukhin on 15.07.23.
//

#include "Meter.h"

namespace wmBus {
  bool Meter::isPeriodEstablished() {
    return lastSlowUpdate != never && slowPeriod != 450s && (!fastEnabled || (lastFastUpdate != never));
  }

  void Meter::setFastEnabled(int hour) {
    bool enabled = fastFirstHour <= hour && hour <= fastLastHour;
    if (enabled != fastEnabled) {
      fastEnabled = enabled;
      lastFastUpdate = never;
    }
  }

  Kernel::Clock::duration Meter::driftFormula(uint8_t from, uint8_t to) {
    return (to - from) * drift_constant * (from > 0 && from <= 0x80 ? -1 : 1);
  }

  Kernel::Clock::duration Meter::drift(int32_t from, int32_t to) {
    if (to > 0x80 && from <= 0x80) {
      return driftFormula(from, 0x81) + driftFormula(0x81, to);
    }
    if (to - from < 0 && to >= 0x00 && from <= 0xFF) {
      return driftFormula(from, 0xFF) + driftFormula(0x00, to) + drift_constant;
    }
    return driftFormula(from, to);
  }

  Kernel::Clock::duration Meter::drift(int16_t acc) const {
    return drift(lastAcc, acc);
  }

  void Meter::updateNextExpectedUpdate(UpdateType update, int16_t acc, int16_t p) {
    auto now = Kernel::Clock::now();
    if (update == Fast) {
      lastFastUpdate = now;
    }
    else if (update == Slow) {
      if (lastSlowUpdate != never) {
        auto diff = now - nextExpectedUpdate.timestamp;
        if (diff < -30s) {
          diff += slowPeriod;
        }
        std::chrono::milliseconds cur_drift;
        if (isPeriodEstablished()) {
          cur_drift = drift(acc);
          if (-800ms < diff && diff < 800ms) {
            slowPeriod += diff;
          }
        }
        else {
          cur_drift = now - lastSlowUpdate - slowPeriod;
        }
        if (cur_drift != 0ms && cur_drift >= -50s && cur_drift <= 50s) {
          slowPeriod += cur_drift;
        }
      }
      lastSlowUpdate = now;
    }
    auto nextSlowUpdate = lastSlowUpdate;
    if (nextSlowUpdate != never) {
      auto period = slowPeriod;
      int32_t _acc = lastAcc == 0 ? 0xff : lastAcc;
      int count = 0;
      while (nextSlowUpdate <= now) {
        int32_t nextAcc = _acc == 0xff ? 0 : _acc + 1;
        period += drift(_acc, nextAcc);
        _acc = nextAcc;
        nextSlowUpdate += period;
        count++;
      }
      if (count >= 5) {
        lastSlowUpdate = never; // syncing again
        slowPeriod = 450s;
      }
    }
    auto nextFastUpdate = lastFastUpdate;
    if (fastEnabled && (lastP != -1 || update == Fast)) {
      int32_t _acc = update == Fast ? acc : lastFastAcc;
      int32_t _p = update == Fast ? p : lastP;
      while (nextFastUpdate <= now) {
        auto duration = (*fastTable)[_acc * 4 + _p];
        nextFastUpdate += duration;
        _acc = _p == 3 ? (_acc == 0xff ? 0 : _acc + 1) : _acc;
        _p = _p == 3 ? 0 : _p + 1;
      }
    }
    if (!fastEnabled || nextFastUpdate > nextSlowUpdate) {
      nextExpectedUpdate = {nextSlowUpdate, 885ms};
    }
    else {
      nextExpectedUpdate = {nextFastUpdate, 475ms};
    }
    if (acc != -1) {
      if (p != -1) {
        lastP = p;
        lastFastAcc = acc;
      }
      else {
        lastAcc = acc;
      }
    }
  }
} // wmBus