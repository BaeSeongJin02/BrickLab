#include <WiFi.h>
#include <esp_now.h>

const int buttonPin = 33;

uint8_t receiverMac[] = {0xD4, 0xE9, 0xF4, 0xC4, 0x97, 0x28};

typedef struct {
  bool buttonState;
} Message;

Message msg;

void setup() {
  Serial.begin(115200);

  pinMode(buttonPin, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW 초기화 실패");
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMac, 6);

  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Peer 추가 실패");
    return;
  }

  Serial.println("송신기 준비 완료");
}

void loop() {

  int buttonState = digitalRead(buttonPin);

  // 버튼 눌리면 LOW
  if (buttonState == LOW) {
    msg.buttonState = true;
  } else {
    msg.buttonState = false;
  }

  esp_now_send(receiverMac, (uint8_t *)&msg, sizeof(msg));

  Serial.print("보낸 상태: ");
  Serial.println(msg.buttonState);

  delay(100);
}