#include <WiFi.h>
#include <esp_mac.h>

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Wi-Fi 초기화
  WiFi.mode(WIFI_STA);
  delay(100);

  Serial.println("ESP32 MAC 주소 확인");

  Serial.print("Wi-Fi MAC 주소: ");
  Serial.println(WiFi.macAddress());

  uint8_t mac[6];

  // Bluetooth MAC 주소 읽기
  esp_read_mac(mac, ESP_MAC_BT);

  Serial.printf(
    "Bluetooth MAC 주소: %02X:%02X:%02X:%02X:%02X:%02X\n",
    mac[0], mac[1], mac[2],
    mac[3], mac[4], mac[5]
  );
}

void loop() {
}