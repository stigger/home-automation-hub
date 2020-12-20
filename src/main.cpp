#include "mbed.h"
#include "MQTTClientMbedOs.h"
#include "WIZnetInterface.h"
#include "LaCrosse.h"
#include "cc1101.h"
#include "AEG1.h"
#include "AEG2.h"

#define IRMP_INPUT_PIN   p25
#define IRMP_SUPPORT_SAMSUNG_PROTOCOL           1
#define IRMP_USE_COMPLETE_CALLBACK       1
#define F_INTERRUPTS                          15625
#include "irmp.c.h"

uint8_t mac[] = {0x90, 0xA2, 0xDA, 0x0D, 0xCA, 0x21};

WIZnetInterface net(SPI_PSELMOSI0, SPI_PSELMISO0, SPI_PSELSCK0, SPI_PSELSS0);
TCPSocket tcp; // mosquitto doesn't support UDP
MQTTClient mqtt(&tcp);

CC1101 cc1101_lacrosse(false, p14, p15);
CC1101 cc2500_light(true, p12);

class tempStuff {
public:
    uint8_t ID = 0;
    float lastTemp = -1;
};

uint8_t prev_lights_state;
//  byte unknown_sent_right_after_rf_init[16] {0x0e, 0x68, 0x04, 0x1f, 0x46, 0x50, 0xa7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xaa, 0x16, 0xff};
const uint8_t off[16] {0x0e, 0x68, 0x04, 0x1f, 0x46, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5f, 0x16, 0xff};
const uint8_t on[16] {0x0e, 0x68, 0x04, 0x1f, 0x46, 0x01, 0xa7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfb, 0x16, 0xff};

AEG1 aeg1;
AEG2 aeg2;

std::unique_ptr<Thread> spawn(Kernel::Clock::duration_u32 delay, void (*f)()) {
  std::unique_ptr<Thread> t(new Thread());
  t->start([&delay, f]{
      ThisThread::sleep_for(delay);
      f();
  });
  return t;
}

void updateLights(uint8_t data) {
  NRF_RADIO->INTENCLR = RADIO_INTENCLR_END_Clear << RADIO_INTENCLR_END_Pos;
  unique_ptr<Thread> t1, t2, t3;
  if ((prev_lights_state & 0b001u) != (data & 0b001u)) {
    if (data & 0b01) {
      t1 = spawn(0ms, [] { aeg1.on(); });
    }
    else {
      t1 = spawn(0ms, [] { aeg1.off(); });
    }
  }
  if (t1) t1->join();
  if ((prev_lights_state & 0b010u) != (data & 0b010u)) {
    if (data & 0b10) {
      t2 = spawn(82ms, [] { aeg2.on(); });
    }
    else {
      t2 = spawn(314ms, [] { aeg2.off(); });
    }
  }
  if ((prev_lights_state & 0b100u) != (data & 0b100u)) {
    if ((data & 0b100u) != 0) {
      t3 = spawn(133ms, [] { cc2500_light.sendData(on); });
    }
    else {
      t3 = spawn(308ms, [] { cc2500_light.sendData(off); });
    }
  }
  if (t2) t2->join();
  if (t3) t3->join();

  aeg1.init_for_receive();
  prev_lights_state = data;
}

void messageArrived(MQTT::MessageData &md) {
  MQTT::Message &message = md.message;
  if (message.payloadlen == 1) {
    uint8_t data = ((uint8_t*)message.payload)[0];
    NRF_RADIO->TASKS_STOP = 1; //todo test commenting out
    updateLights(data);
  }
}

//decode_results results;
tempStuff sensors[5];

MQTT::Message& fillMsg(MQTT::Message &msg, string &s) {
  msg.payload = (void*) s.c_str();
  msg.payloadlen = s.length();
  return msg;
}

void HandleReceivedLaCrosseData(uint8_t payload[]) {
  struct LaCrosse::Frame frame {};
  MQTT::Message msg = {};
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
    if (stuff != nullptr) {
      if (stuff->lastTemp == -1 || fabs(stuff->lastTemp - frame.Temperature) < 5) {
        stuff->lastTemp = frame.Temperature;

        if (mqtt.isConnected()) {
          string topic("lacrosse/");
          topic += to_string(frame.ID);
          {
            char buf[20];
            msg.payload = buf;
            msg.payloadlen = snprintf(buf, sizeof(buf), "%f", frame.Temperature);
            mqtt.publish((topic + "/temperature").c_str(), msg);
          }
          {
            string str = to_string(frame.Humidity);
            mqtt.publish((topic + "/humidity").c_str(), fillMsg(msg, str));
          }
          msg.payload = (void*) (frame.WeakBatteryFlag ? "1" : "0");
          msg.payloadlen = 1;
          mqtt.publish((topic + "/weak_battery").c_str(), msg);
        }
      }
    }
  }
  else {
    if (payload[0] >> 4u == 7 && payload[4] == LaCrosse::CalculateCRC(payload)) {
      uint8_t id = 0;
      id |= (payload[0] & 0xFu) << 2u;
      id |= (payload[1] & 0xC0u) >> 6u;

      short co2 = ((short) payload[2] << 8u) + (short)payload[3]; // NOLINT(cppcoreguidelines-narrowing-conversions)

      if (mqtt.isConnected()) {
        string topic("custom/");
        topic += to_string(id);
        topic += "/co2";
        string s = to_string(co2);
        mqtt.publish(topic.c_str(), fillMsg(msg, s));
      }
    }
  }
}

extern "C" {
void RADIO_IRQHandler() {
  mbed_event_queue()->call([] {
      uint8_t i = aeg1.receive();
      if (i != 0xff) {
        // todo hangs on using AEG remote
        switch (i) {
          case 0xf0:
            prev_lights_state &= 0xfe;
            updateLights(0b000);
            break;
          case 0x30:
            prev_lights_state |= 1;
            updateLights(0b111);
            break;
          case 0xa0:
            aeg2.brightness_up(false);
            break;
          case 0xb2:
            aeg2.brightness_down(false);
            break;
          case 0x90:
            aeg2.color_up(false);
            break;
          case 0xb4:
            aeg2.color_down(false);
            break;
          default:;
        }
      }
      if (i != 0xf0 && i != 0x30) {
        aeg1.init_for_receive();
      }
  });
  NRF_RADIO->EVENTS_END = 0;
}
}

int main() {
  ThisThread::sleep_for(2s);// on power up W5500 needs to initialize or something

  NVIC_ClearPendingIRQ(RADIO_IRQn);
  NVIC_EnableIRQ(RADIO_IRQn);
  NRF_RADIO->FREQUENCY = 5;
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos |
                      RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos;
  NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_Pos4dBm;

  NRF_RNG->TASKS_START = 1;
  while (!NRF_RNG->EVENTS_VALRDY) {
    ThisThread::sleep_for(1ms);
  }
  NRF_RNG->TASKS_STOP = 1;
  srand(NRF_RNG->VALUE);

  net.init(mac, "192.168.132.3", "255.255.255.0", "127.0.0.1");

  cc1101_lacrosse.setCallback(callback(&HandleReceivedLaCrosseData));
  aeg1.init_for_receive();

  irmp_init();
  mbed::Ticker sMbedTimer;
  sMbedTimer.attach(irmp_ISR, std::chrono::microseconds(1000000 / F_INTERRUPTS));

  MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
  data.clientID.cstring = (char *) "arduinoClient";

  auto connect = [&]() {
      tcp.open(&net);
      tcp.connect(SocketAddress("192.168.132.243", 1883));

      mqtt.connect(data);
      mqtt.subscribe("arduinoHub/lights", MQTT::QOS0, &messageArrived);

      tcp.sigio(callback([]{ mqtt.yield(1); }));
  };

  connect();

  EventQueue *q = mbed_event_queue();
  q->call_every(std::chrono::seconds(data.keepAliveInterval), [&]() {
      if (!mqtt.isConnected()) {
        tcp.sigio(Callback<void()>());
        tcp.close();
        connect();
      }
      else {
        mqtt.yield(1); // keepalive
      }
  });

  irmp_register_complete_callback_function([]{
      mbed_event_queue()->call([]{
          IRMP_DATA irmp_data;
          if (irmp_get_data(&irmp_data) && mqtt.isConnected()) {
            MQTT::Message msg = {};
            char buf[20];
            msg.payloadlen = snprintf(buf, sizeof(buf), "%0x%0x", irmp_data.address, irmp_data.command);
            msg.payload = buf;
            mqtt.publish("ir_sensor", msg);
          }
      });
  });

  q->dispatch_forever();
}
