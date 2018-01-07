#include <PubSubClient.h>
#include <IRremote.h>
#include <SPI.h>
#include <Ethernet.h>
#include "RFMxx.h"
#include "Pn9.h"
#include "util.h"
#include "LaCrosse.h"
#include "cc1101.h"

uint8_t mac[] = {0x90, 0xA2, 0xDA, 0x0D, 0xCA, 0x21};

IPAddress mqttServer(192, 168, 132, 243);
EthernetClient mqttEthClient;
PubSubClient mqttClient(mqttEthClient);

EthernetServer server(2323);

CC1101 cc1101;
RFMxx rfm2(7);

IRrecv irrecv(5);


class tempStuff {
public:
    byte ID = 0;
    float lastTemp = -1;
};

decode_results results;
tempStuff sensors[5];
boolean check_cc1101;
boolean check_rfm69;

void cc1101_asserted() {
  check_cc1101 = true;
}

void rfm69_asserted() {
  check_rfm69 = true;
}

void setup() {
  Serial.begin(57600);

  irrecv.enableIRIn();

  cc1101.init();
  rfm2.InitializeLaCrosse();
  Serial.print(F("CC1101_PARTNUM "));
  Serial.println(cc1101.readReg(CC1101_PARTNUM, CC1101_STATUS_REGISTER));
  Serial.print(F("CC1101_VERSION "));
  Serial.println(cc1101.readReg(CC1101_VERSION, CC1101_STATUS_REGISTER));
  Serial.println(rfm2.IsConnected());

  rfm2.EnableReceiver(true);

  mqttClient.setServer(mqttServer, 1883);

  Ethernet.begin(mac);
  delay(1500);
  Serial.println(Ethernet.localIP());

  check_cc1101 = false;
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), cc1101_asserted, RISING);

  check_rfm69 = false;
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(3), rfm69_asserted, RISING);
}

void HandleReceivedMAXData(CC1101 *r) {
  CCPACKET packet;
  if (r->receiveData(&packet) > 0 && packet.crc_ok) {
    String message('Z');
    char tmp[5];
    sprintf(tmp, "%02X", packet.length);
    message += tmp;
    for (byte i = 0; i < packet.length + 1; i++) {
      sprintf(tmp, "%02X", packet.data[i]);
      message += tmp;
    }
    String rssiString = String(packet.rssi, HEX);
    rssiString.toUpperCase();
    message += rssiString;
    message += '\n';
    server.write(message.c_str());
  }
}

void SendMAXData(char const *buf, CC1101 *radio) {
  byte enc[50];
  boolean fast;
  if (buf[1] == 's' || buf[1] == 'f') {
    fast = buf[1] == 'f';
  }
  else {
    return;
  }

  fromhex(buf + 2, enc, sizeof(enc) - 3);
  radio->sendData(enc, !fast);
}

void HandleReceivedLaCrosseData(RFMxx *rfm) {
  rfm->EnableReceiver(false);

  byte payload[PAYLOADSIZE];
  rfm->GetPayload(payload);

  struct LaCrosse::Frame frame;
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
      Serial.println("sensors overflow");
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
    if (payload[0] >> 4 == 7 && payload[4] == LaCrosse::CalculateCRC(payload)) {
      byte id = 0;
      id |= (payload[0] & 0xF) << 2;
      id |= (payload[1] & 0xC0) >> 6;

      short co2 = (((short)payload[2]) << 8) + (short)payload[3];

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
    Serial.print("mqtt reconnect failed");
    delay(1000);
  }
}

void loop() {
  if (check_cc1101) {
    check_cc1101 = false;
    HandleReceivedMAXData(&cc1101);
  }

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

  if (EthernetClient client = server.available()) {
    char buf[256];
    int read = client.read((uint8_t *) buf, sizeof(buf));
    if (read > 0) {
      if (buf[0] != 'Z') return;
      SendMAXData(buf, &cc1101);
    }
  }

  if (!mqttClient.connected()) {
    mqttReconnect();
  }
  mqttClient.loop();

  Ethernet.maintain();
}
