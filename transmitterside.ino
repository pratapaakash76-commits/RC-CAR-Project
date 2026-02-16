#include "ESP32_NOW.h"
#include "WiFi.h"

#define ESPNOW_WIFI_CHANNEL 6
#define RXD2 16
#define TXD2 17

HardwareSerial UnoSerial(2);

typedef struct {
  char cmd;
} ControlPacket;

ControlPacket packet;

class ESP_NOW_Broadcast_Peer : public ESP_NOW_Peer {
public:
  ESP_NOW_Broadcast_Peer(uint8_t channel,
                         wifi_interface_t iface,
                         const uint8_t *lmk)
  : ESP_NOW_Peer(ESP_NOW.BROADCAST_ADDR,
                 channel, iface, lmk) {}

  bool begin() {
    return ESP_NOW.begin() && add();
  }

  bool sendPacket(const uint8_t *data, size_t len) {
    return send(data, len);
  }
};

ESP_NOW_Broadcast_Peer broadcast_peer(
  ESPNOW_WIFI_CHANNEL,
  WIFI_IF_STA,
  nullptr
);

void setup() {

  Serial.begin(115200);
  UnoSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);

  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);

  Serial.println("ESP32 TX starting...");

  if (!broadcast_peer.begin()) {
    Serial.println("ESP-NOW failed!");
    ESP.restart();
  }
}

void loop() {

  if (UnoSerial.available()) {

    char c = UnoSerial.read();

    if (c=='F'||c=='B'||c=='L'||c=='R'||c=='S') {

      packet.cmd = c;

      broadcast_peer.sendPacket(
        (uint8_t*)&packet,
        sizeof(packet)
      );

      Serial.print("TX → ");
      Serial.println(c);
    }
  }
}
