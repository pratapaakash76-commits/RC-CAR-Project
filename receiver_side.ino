#include "ESP32_NOW.h"
#include "WiFi.h"

#define ESPNOW_WIFI_CHANNEL 6

//L- LEFT motors
#define LF1 25
#define LF2 26
#define LR1 33
#define LR2 32

// R-RIGHT motors
#define RF1 27
#define RF2 14
#define RR1 18
#define RR2 19

typedef struct {
  char cmd;
} ControlPacket;

ControlPacket incoming;

void setupMotorPins() {

  pinMode(LF1,OUTPUT); pinMode(LF2,OUTPUT);
  pinMode(LR1,OUTPUT); pinMode(LR2,OUTPUT);
  pinMode(RF1,OUTPUT); pinMode(RF2,OUTPUT);
  pinMode(RR1,OUTPUT); pinMode(RR2,OUTPUT);
}

void stopCar(){
  digitalWrite(LF1,LOW); digitalWrite(LF2,LOW);
  digitalWrite(LR1,LOW); digitalWrite(LR2,LOW);
  digitalWrite(RF1,LOW); digitalWrite(RF2,LOW);
  digitalWrite(RR1,LOW); digitalWrite(RR2,LOW);
}

void forward(){
  digitalWrite(LF1,HIGH); digitalWrite(LF2,LOW);
  digitalWrite(LR1,HIGH); digitalWrite(LR2,LOW);
  digitalWrite(RF1,HIGH); digitalWrite(RF2,LOW);
  digitalWrite(RR1,HIGH); digitalWrite(RR2,LOW);
}

void backward(){
  digitalWrite(LF1,LOW); digitalWrite(LF2,HIGH);
  digitalWrite(LR1,LOW); digitalWrite(LR2,HIGH);
  digitalWrite(RF1,LOW); digitalWrite(RF2,HIGH);
  digitalWrite(RR1,LOW); digitalWrite(RR2,HIGH);
}

void left(){
  digitalWrite(LF1,LOW); digitalWrite(LF2,HIGH);
  digitalWrite(LR1,LOW); digitalWrite(LR2,HIGH);
  digitalWrite(RF1,HIGH); digitalWrite(RF2,LOW);
  digitalWrite(RR1,HIGH); digitalWrite(RR2,LOW);
}

void right(){
  digitalWrite(LF1,HIGH); digitalWrite(LF2,LOW);
  digitalWrite(LR1,HIGH); digitalWrite(LR2,LOW);
  digitalWrite(RF1,LOW); digitalWrite(RF2,HIGH);
  digitalWrite(RR1,LOW); digitalWrite(RR2,HIGH);
}

class RC_Peer : public ESP_NOW_Peer {
public:
  RC_Peer(const uint8_t *mac,
          uint8_t ch,
          wifi_interface_t iface,
          const uint8_t *lmk)
  : ESP_NOW_Peer(mac,ch,iface,lmk) {}

  bool addPeer(){ return add(); }

  void onReceive(const uint8_t *data,
                 size_t len,
                 bool broadcast) {

    memcpy(&incoming,data,sizeof(incoming));

    char cmd = incoming.cmd;

    Serial.print("RX ← ");
    Serial.println(cmd);

    switch(cmd){
      case 'F': forward(); break;
      case 'B': backward(); break;
      case 'L': left(); break;
      case 'R': right(); break;
      default: stopCar();
    }
  }
};

std::vector<ESP_NOW_Peer*> masters;

void registerPeer(const esp_now_recv_info_t *info,
                  const uint8_t *data,
                  int len,
                  void *arg) {

  if(memcmp(info->des_addr,
            ESP_NOW.BROADCAST_ADDR,6)==0){

    RC_Peer *p = new RC_Peer(
      info->src_addr,
      ESPNOW_WIFI_CHANNEL,
      WIFI_IF_STA,
      nullptr
    );

    if(p->addPeer()) masters.push_back(p);

    Serial.println("Peer registered");
  }
}

void setup(){

  Serial.begin(115200);

  setupMotorPins();
  stopCar();

  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);

  if(!ESP_NOW.begin()){
    Serial.println("ESP-NOW fail!");
    ESP.restart();
  }

  ESP_NOW.onNewPeer(registerPeer,nullptr);

  Serial.println("Receiver ready");
}

void loop(){}
