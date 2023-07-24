//
// Created by Vyacheslav Karpukhin on 15.07.23.
//

#ifndef ARDUINO_HUB_METER_H
#define ARDUINO_HUB_METER_H

#include "mbed.h"

namespace wmBus {
  auto static constexpr never = Kernel::Clock::time_point(0ms);
  static constexpr Kernel::Clock::duration drift_constant = 215ms;

  class Meter {

    Kernel::Clock::duration slowPeriod = 450s;
    int16_t lastAcc = -1, lastFastAcc = -1, lastP = -1;
    const Kernel::Clock::duration (*fastTable)[1024];
    int fastFirstHour{}, fastLastHour{};
    bool fastEnabled{};
    Kernel::Clock::time_point lastSlowUpdate = never, lastFastUpdate = never;

    static Kernel::Clock::duration driftFormula(uint8_t from, uint8_t to);

    [[nodiscard]] static Kernel::Clock::duration drift(int32_t from, int32_t to);

    [[nodiscard]] Kernel::Clock::duration drift(int16_t acc) const;

  public:
    enum UpdateType {
      Fast,
      Slow,
      NoUpdate,
    };

    struct ExpectedUpdate {
      Kernel::Clock::time_point timestamp;
      Kernel::Clock::duration margin;

      [[nodiscard]] bool isExpired() const {
        return timestamp - Kernel::Clock::now() > margin;
      }

      [[nodiscard]] bool isExpectedNow() const {
        return timestamp - Kernel::Clock::now() < margin;
      };

      friend bool operator<(const ExpectedUpdate &lhs, const ExpectedUpdate &rhs) {
        return std::tie(lhs.timestamp, lhs.margin) < std::tie(rhs.timestamp, rhs.margin);
      }
    };

    explicit Meter(const Kernel::Clock::duration (*fastTable)[1024],
                   int fastFirstHour = 6,
                   int fastLastHour = 18) : fastTable(fastTable), fastFirstHour(fastFirstHour),
                                            fastLastHour(fastLastHour) {}

    bool isPeriodEstablished();

    void setFastEnabled(int hour);

    void updateNextExpectedUpdate(UpdateType update, int16_t acc, int16_t p);

    ExpectedUpdate nextExpectedUpdate = {never, 10s};
  };
} // wmBus

#endif //ARDUINO_HUB_METER_H
