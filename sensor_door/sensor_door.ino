#include <esp_now.h>
#include <WiFi.h>

uint8_t coreAddress[] = {0xD4, 0xE9, 0xF4, 0xE3, 0x67, 0xE8};

#define DOOR_PIN 26

typedef struct {
  uint8_t nodeId;
  uint8_t msgType;      // 0: 주기 전송, 1: 이벤트 전송
  int lightValue;
  bool motion;
  bool doorOpen;
  unsigned long timestamp;
} SensorData;

SensorData data;

bool lastDoorState;

unsigned long lastPeriodicSend = 0;
const unsigned long PERIODIC_INTERVAL = 30000;

bool readDoorState() {
  return digitalRead(DOOR_PIN) == HIGH;
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send : ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void sendData(uint8_t type) {
  data.nodeId = 2;
  data.msgType = type;
  data.lightValue = -1;
  data.motion = false;
  data.doorOpen = readDoorState();
  data.timestamp = millis();

  esp_now_send(coreAddress, (uint8_t *)&data, sizeof(data));

  Serial.print("Door : ");
  Serial.println(data.doorOpen ? "OPEN" : "CLOSED");
}

void setup() {
  Serial.begin(115200);

  pinMode(DOOR_PIN, INPUT);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }

  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, coreAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Peer Add Failed");
    return;
  }

  lastDoorState = readDoorState();

  Serial.println("Door Sensor Ready");

  // 초기 상태 전송
  sendData(0);
  lastPeriodicSend = millis();
}

void loop() {

  bool currentDoorState = readDoorState();

  // 문 상태가 바뀌면 즉시 전송
  if (currentDoorState != lastDoorState) {

    delay(50);

    currentDoorState = readDoorState();

    if (currentDoorState != lastDoorState) {

      lastDoorState = currentDoorState;
      sendData(1);

    }
  }

  // 30초마다 현재 상태 전송
  if (millis() - lastPeriodicSend >= PERIODIC_INTERVAL) {

    sendData(0);
    lastPeriodicSend = millis();

  }

  delay(50);
}