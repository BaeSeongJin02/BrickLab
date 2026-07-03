#include <esp_now.h>
#include <WiFi.h>

uint8_t coreAddress[] = {0xd4, 0xe9, 0xf4, 0xe3, 0x67, 0xe8};

#define PIR_PIN   27
#define LIGHT_PIN 34

typedef struct struct_message {
  uint8_t nodeId;
  uint8_t msgType;        // 0: 주기 전송, 1: 이벤트 전송
  int lightValue;
  bool motion;
  bool doorOpen;
  unsigned long timestamp;
} struct_message;

struct_message myData;

bool lastMotionState = false;

unsigned long lastPeriodicSend = 0;
const unsigned long PERIODIC_INTERVAL = 30000;

unsigned long lastMotionSend = 0;
const unsigned long MOTION_DEBOUNCE = 3000;

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("전송 상태: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "성공" : "실패");
}

void sendData(uint8_t msgType) {
  myData.nodeId = 1;
  myData.msgType = msgType;
  myData.lightValue = analogRead(LIGHT_PIN);
  myData.motion = digitalRead(PIR_PIN);
  myData.doorOpen = false;
  myData.timestamp = millis();

  esp_err_t result = esp_now_send(coreAddress, (uint8_t *)&myData, sizeof(myData));

  Serial.printf("[전송 시도] type=%d light=%d motion=%d -> %s\n",
                msgType,
                myData.lightValue,
                myData.motion,
                result == ESP_OK ? "OK" : "FAIL");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIR_PIN, INPUT);
  pinMode(LIGHT_PIN, INPUT);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW 초기화 실패");
    return;
  }

  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, coreAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("피어(코어 모듈) 추가 실패");
    return;
  }

  lastMotionState = digitalRead(PIR_PIN);

  Serial.println("조도+PIR 센서 노드 시작");

  // 시작하자마자 현재 상태 1회 전송
  sendData(0);
  lastPeriodicSend = millis();
}

void loop() {
  bool currentMotion = digitalRead(PIR_PIN);

  if (currentMotion && !lastMotionState &&
      (millis() - lastMotionSend > MOTION_DEBOUNCE)) {
    sendData(1);
    lastMotionSend = millis();
  }

  lastMotionState = currentMotion;

  if (millis() - lastPeriodicSend > PERIODIC_INTERVAL) {
    sendData(0);
    lastPeriodicSend = millis();
  }

  delay(100);
}