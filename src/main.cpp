#pragma clang diagnostic push
#pragma ide diagnostic ignored "cert-err58-cpp"
#pragma ide diagnostic ignored "readability-static-accessed-through-instance"
#include <PubSubClient.h>
#include <IRremote.h>
#include <SPI.h>
#include <Ethernet.h>
#include "RFMxx.h"
#include "util.h"
#include "LaCrosse.h"
#include "cc1101.h"

#define NRF_SS 19

uint8_t mac[] = {0x90, 0xA2, 0xDA, 0x0D, 0xCA, 0x21};

IPAddress mqttServer(192, 168, 132, 243);
EthernetClient mqttEthClient;
PubSubClient mqttClient(mqttEthClient);

RFMxx rfm2(6);
CC1101 cc1101;

IRrecv irrecv(1);


class tempStuff {
public:
    byte ID = 0;
    float lastTemp = -1;
};

uint8_t prev_lights_state;
//  byte unknown_sent_right_after_rf_init[16] {0x0e, 0x68, 0x04, 0x1f, 0x46, 0x50, 0xa7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xaa, 0x16, 0xff};
const byte off[16] {0x0e, 0x68, 0x04, 0x1f, 0x46, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5f, 0x16, 0xff};
const byte on[16] {0x0e, 0x68, 0x04, 0x1f, 0x46, 0x01, 0xa7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfb, 0x16, 0xff};

void mqttCallback(char *, uint8_t *payload, unsigned int length) {
  if (length == 1) {
    uint8_t data = payload[0];
    if ((prev_lights_state & 0b100u) != (data & 0b100u)) {
      for (int i = 0; i < 5; i++) {
        cc1101.sendData((data & 0b100u) != 0 ? on : off, false);
        delay(30);
      }
    }
    digitalWrite(NRF_SS, LOW);
    SPI.transfer(data);
    digitalWrite(NRF_SS, HIGH);
    prev_lights_state = data;
  }
}

decode_results results;
tempStuff sensors[5];
boolean check_rfm69;
boolean mqtt_subscribed;

void rfm69_asserted() {
  check_rfm69 = true;
}

void setup() {
  // serial currently conflicts with IR on pin 1
//  Serial.begin(57600);
  pinMode(NRF_SS, OUTPUT);

  irrecv.enableIRIn();

  cc1101.init();
//  Serial.print(F("CC2500_PARTNUM "));
//  Serial.println(cc1101.readReg(CC1101_PARTNUM, CC1101_STATUS_REGISTER));
//  Serial.print(F("CC2500_VERSION "));
//  Serial.println(cc1101.readReg(CC1101_VERSION, CC1101_STATUS_REGISTER));

  rfm2.InitializeLaCrosse();
//  Serial.println(rfm2.IsConnected());

  rfm2.EnableReceiver(true);

  Ethernet.begin(mac);
  delay(1500);
//  Serial.println(Ethernet.localIP());

  mqttClient.setServer(mqttServer, 1883);
  mqttClient.setCallback(&mqttCallback);

  check_rfm69 = false;
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(3), rfm69_asserted, RISING);
}

void HandleReceivedLaCrosseData(RFMxx *rfm) {
  rfm->EnableReceiver(false);

  byte payload[PAYLOADSIZE];
  rfm->GetPayload(payload);

  struct LaCrosse::Frame frame {};
  LaCrosse::DecodeFrame(payload, &frame);
  if (frame.IsValid) {
    tempStuff *stuff = nullptr;
    for (size_t i = 0; i < sizeof(sensors); i++) {
      if (frame.ID == sensors[i].ID || sensors[i].ID == 0) {
        stuff = &(sensors[i]);
        sensors[i].ID = frame.ID;
        break;
      }
    }
    if (stuff == nullptr) {
//      Serial.println("sensors overflow");
    }
    else {
      if (stuff->lastTemp == -1 || fabs(stuff->lastTemp - frame.Temperature) < 5) {
        stuff->lastTemp = frame.Temperature;

        if (mqttClient.connected()) {
          String topic("lacrosse/");
          topic += frame.ID;
          {
            String temperature(frame.Temperature);
            mqttClient.publish((topic + "/temperature").c_str(), temperature.c_str());
          }
          {
            String humidity(frame.Humidity);
            mqttClient.publish((topic + "/humidity").c_str(), humidity.c_str());
          }
          mqttClient.publish((topic + "/weak_battery").c_str(), frame.WeakBatteryFlag ? "1" : "0");
        }
      }
    }
  }
  else {
    if (payload[0] >> 4u == 7 && payload[4] == LaCrosse::CalculateCRC(payload)) {
      byte id = 0;
      id |= (payload[0] & 0xFu) << 2u;
      id |= (payload[1] & 0xC0u) >> 6u;

      short co2 = ((short) payload[2] << 8u) + (short)payload[3];

      if (mqttClient.connected()) {
        String topic("custom/");
        topic += id;
        topic += "/co2";
        String value(co2);
        mqttClient.publish(topic.c_str(), value.c_str());
      }
    }
  }

  rfm->EnableReceiver(true);
}

void mqttReconnect() {
  if (!mqttClient.connect("arduinoClient")) {
//    Serial.print("mqtt reconnect failed");
    delay(1000);
  }
}

void loop() {
  if (check_rfm69) {
    check_rfm69 = false;
    rfm2.Receive();
    if (rfm2.PayloadIsReady()) {
      HandleReceivedLaCrosseData(&rfm2);
    }
  }

  if (irrecv.decode(&results)) {
    if (mqttClient.connected()) {
      String value(results.value, 16);
      mqttClient.publish("ir_sensor", value.c_str());
    }
    irrecv.resume();
  }

  if (!mqttClient.connected()) {
    mqtt_subscribed = false;
    mqttReconnect();
  }
  else if (!mqtt_subscribed) {
    mqtt_subscribed = mqttClient.subscribe("arduinoHub/lights");
  }
  mqttClient.loop();

  Ethernet.maintain();
}
