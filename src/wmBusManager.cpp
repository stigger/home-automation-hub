//
// Created by Vyacheslav Karpukhin on 29.04.23.
//

#include "wmBusManager.h"

#include <memory>
#include "crc.h"
#include "Meter.h"
#include "wmBusMeterTables.h"
#include "private.h"

namespace wmBus {
  Manager::Manager(CC1101 &cc1101, MQTTClient &mqtt) : cc1101(cc1101),
                                                       mqtt(mqtt),
                                                       bytes(),
                                                       packet(),
                                                       receivedLength() {
    wmBus_meters = {
        {METER1, std::make_shared<Meter>(&wmBus::METER1_TABLE)},
        {METER2, std::make_shared<Meter>(&wmBus::METER2_TABLE)},
        {METER3, std::make_shared<Meter>(&wmBus::METER3_TABLE)},
        {METER4, std::make_shared<Meter>(&wmBus::METER4_TABLE)},
        {METER5, std::make_shared<Meter>(&wmBus::METER5_TABLE, 5, 17)},
    };
    wmBus_expectingMeters.reserve(wmBus_meters.size());
    for (auto [key, _]: wmBus_meters) {
      wmBus_expectingMeters.insert(key);
    }
    reset();
  }

  void Manager::reset() {
    receivedLength = 0;
    packet = std::make_shared<vector<uint8_t>>();
  }

  void Manager::receiveBytes(uint16_t len, const uint8_t data[]) {
    if (len == 0) {
      reset();
      return;
    }
    if (receivedLength == 0) {
      if (*(uint16_t *) data == 0xcd54) {
        uint8_t lengthField = data[2];
        if (lengthField < 9 || lengthField > 200) {
          cc1101.resetRxMode();
          return;
        }

        receivedLength = 3 + lengthField + 2 * (2 + (lengthField - 10) / 16);
        bytes = vector<uint8_t>();
        bytes.reserve(receivedLength);

        cc1101.setPacketLength(receivedLength, 3);
      }
      else {
        cc1101.resetRxMode();
        return;
      }
    }
    bytes.insert(bytes.end(), data, data + len);
    if (bytes.size() == receivedLength) {
      packet->reserve(bytes[2] + 1);
      auto it = bytes.cbegin() + 2;
      if (verifyAndCopyBytes(it, *packet)) {
        mbed_event_queue()->call(callback(this, &Manager::HandleReceivedWMBus), packet);
      }
      reset();
    }
  }

  bool Manager::verifyAndCopyBytes(std::vector<uint8_t>::const_iterator &pByte, std::vector<uint8_t> &pPacket) {
    if (!verifyAndCopyBlock(pByte, pPacket, 10)) {
      return false;
    }

    size_t cycles = (pPacket[0] - pPacket.size() + 1) / 16;
    while (cycles-- > 0) {
      if (!verifyAndCopyBlock(pByte, pPacket, 16)) {
        return false;
      }
    }

    size_t left = pPacket[0] - pPacket.size() + 1;
    if (left > 0 && !verifyAndCopyBlock(pByte, pPacket, left)) {
      return false;
    }

    return true;
  }

  bool Manager::verifyAndCopyBlock(std::vector<uint8_t>::const_iterator &pByte, std::vector<uint8_t> &pPacket,
                                   uint16_t length) {
    uint16_t crc = crc16_wmbus(pByte.base(), length);

    if (((~crc) & 0xffff) != (pByte[length] << 8 | (pByte)[length + 1])) {
      return false;
    }

    pPacket.insert(pPacket.end(), pByte, pByte + length);

    pByte += length + 2;

    return true;
  }

  void Manager::scheduleNext(uint32_t serial, Meter::UpdateType updateType, const shared_ptr<Meter> &meter, int16_t acc,
                             int16_t p) {
    auto now = Kernel::Clock::now();
    wmBus_expectedTransmissions.erase(meter->nextExpectedUpdate);
    meter->updateNextExpectedUpdate(updateType, acc, p);
    if (!meter->isPeriodEstablished()) {
      wmBus_expectingMeters.insert(serial);
      return;
    }

    auto expectedUpdate = meter->nextExpectedUpdate;
    wmBus_expectedTransmissions.insert(expectedUpdate);

    mbed_event_queue()->call_in(expectedUpdate.timestamp - now - expectedUpdate.margin, [this, serial, meter] {
      auto expectedUpdate = meter->nextExpectedUpdate;
      if (expectedUpdate.isExpired()) {
        return;
      }
      if (cc1101.getMode() != CC1101::wMBus) {
        reset();
        cc1101.setMode(CC1101::wMBus);
      }
      wmBus_expectingMeters.insert(serial);

      mbed_event_queue()->call_in(expectedUpdate.margin * 2, [this, serial, meter] {
        auto now = Kernel::Clock::now();
        if (meter->nextExpectedUpdate.timestamp < now) {
          if (meter->isPeriodEstablished()) {
            wmBus_expectingMeters.erase(serial);
          }
          scheduleNext(serial, Meter::NoUpdate, meter);
        }

        if (cc1101.getMode() == CC1101::wMBus && wmBus_expectingMeters.empty()) {
          auto expectedUpdate = *wmBus_expectedTransmissions.begin();
          if (!expectedUpdate.isExpectedNow()) {
            cc1101.setMode(CC1101::LaCrosse);
          }
        }
      });
    });
  }

  void Manager::HandleReceivedWMBus(
      std::shared_ptr<vector<uint8_t>> payload_ptr) { // NOLINT(performance-unnecessary-value-param)
    uint32_t serial;
    std::vector<uint8_t> &payload = *payload_ptr;
    serial = *(uint32_t *) (&payload[4]);
    if (wmBus_meters.find(serial) == wmBus_meters.end()) {
      return;
    }
    uint8_t length = payload[0];
    Meter::UpdateType updateType;
    int16_t acc, p = -1;
    switch (length) {
      case 0x49:
      case 0x53:
        acc = payload[length == 0x49 ? 17 : 27];
        p = payload[length == 0x49 ? 20 : 30];

        if (p >= 0xee) {
          p = 1;
        }
        else if (p >= 0x7e) {
          p = 2;
        }
        else if (p >= 0x5e) {
          p = 0;
        }
        else {
          p = 3;
        }
        updateType = Meter::Fast;
        break;
      case 0x34:
      case 0x39:
      case 0x3C:
      case 0x40:
        acc = length == 0x34 || length == 0x39 ? payload[11] : payload[19];
        updateType = Meter::Slow;
        break;
      default:
        return;
    }

    auto meter = wmBus_meters[serial];

    if (length != 0x34 && length != 0x40) {
      auto hour = (payload[payload.size() - 3]) & 0x1f;
      meter->setFastEnabled(hour);
    }

    scheduleNext(serial, updateType, meter, acc, p);

    if (mqtt.isConnected()) {
      string topic = "wmbus/" + to_string(serial);
      MQTT::Message msg{};
      msg.payload = payload_ptr->data();
      msg.payloadlen = payload_ptr->size();
      mqtt.publish(topic.c_str(), msg);
    }

    if (meter->isPeriodEstablished()) {
      wmBus_expectingMeters.erase(serial);
    }
    auto expectedUpdate = *wmBus_expectedTransmissions.begin();
    if (!expectedUpdate.isExpectedNow() && wmBus_expectingMeters.empty()) {
      cc1101.setMode(CC1101::LaCrosse);
    }
  }
}