//
// Created by Vyacheslav Karpukhin on 29.04.23.
//

#ifndef ARDUINO_HUB_WMBUSMANAGER_H
#define ARDUINO_HUB_WMBUSMANAGER_H


#include "cc1101.h"
#include "Meter.h"
#include <vector>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <MQTTClientMbedOs.h>

namespace wmBus {
  class Manager {
  private:
    CC1101 &cc1101;
    MQTTClient &mqtt;

    std::vector<uint8_t> bytes;
    std::shared_ptr<vector<uint8_t>> packet;
    uint16_t receivedLength;

    static bool verifyAndCopyBytes(std::vector<uint8_t>::const_iterator &pByte, std::vector<uint8_t> &pPacket);
    static bool verifyAndCopyBlock(std::vector<uint8_t>::const_iterator &pByte, std::vector<uint8_t> &pPacket, uint16_t length);

    std::set<Meter::ExpectedUpdate> wmBus_expectedTransmissions;
    std::unordered_set<uint32_t> wmBus_expectingMeters;
    std::unordered_map<uint32_t, std::shared_ptr<Meter>> wmBus_meters;

    void scheduleNext(uint32_t serial, Meter::UpdateType updateType, const shared_ptr<Meter> &meter, int16_t acc = -1, int16_t p = -1);
    void HandleReceivedWMBus(std::shared_ptr<vector<uint8_t>> payload_ptr);

  public:
    explicit Manager(CC1101 &cc1101, MQTTClient &mqtt);
    void receiveBytes(uint16_t len, const uint8_t data[]);
    void reset();
  };
}

#endif //ARDUINO_HUB_WMBUSMANAGER_H
