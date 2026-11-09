#include <WiFi.h>
#include <esp_now.h>

#define PIR_PIN 13

uint8_t coreMac[] = {0xD4, 0xE9, 0xF4, 0xE3, 0x67, 0xE8};

typedef struct {
  uint8_t sensorType; // 1 = PIR
  int value;
} SensorData;

SensorData data;

void setup() {
  Serial.begin(115200);
  pinMode(PIR_PIN, INPUT);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, coreMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add core peer");
    return;
  }
}

void loop() {
  int pirState = digitalRead(PIR_PIN);

  data.sensorType = 1;
  data.value = pirState ? 255 : 0;

  esp_now_send(coreMac, (uint8_t *)&data, sizeof(data));

  Serial.print("PIR Send: ");
  Serial.println(data.value);

  delay(300);
}