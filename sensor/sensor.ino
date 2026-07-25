#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// =====================================================
// 센서 핀
// =====================================================
const uint8_t LDR_PIN = 34;
const uint8_t PIR_PIN = 15;

// =====================================================
// 코어 모듈 MAC 주소
// =====================================================
const uint8_t CORE_MAC[6] = {
    0xD4, 0xE9, 0xF4, 0xE3, 0x67, 0xE8
};

// 코어 모듈이 연결된 Wi-Fi 채널과 같아야 함
const uint8_t ESPNOW_CHANNEL = 1;

// 전송 주기: 30초
const unsigned long SEND_INTERVAL_MS = 30000;

unsigned long lastSendTime = 0;

// =====================================================
// 전송 패킷
// 코어 모듈에서도 완전히 동일해야 함
// =====================================================
typedef struct __attribute__((packed)) {
    uint16_t lightRaw;
    uint8_t motion;
} SensorPacket;

// =====================================================
// ESP-NOW 초기화
// =====================================================
bool initializeEspNow() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    delay(100);

    if (esp_wifi_set_channel(
            ESPNOW_CHANNEL,
            WIFI_SECOND_CHAN_NONE
        ) != ESP_OK) {
        Serial.println("Wi-Fi 채널 설정 실패");
        return false;
    }

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW 초기화 실패");
        return false;
    }

    esp_now_peer_info_t peerInfo = {};

    memcpy(
        peerInfo.peer_addr,
        CORE_MAC,
        sizeof(CORE_MAC)
    );

    peerInfo.channel = ESPNOW_CHANNEL;
    peerInfo.encrypt = false;
    peerInfo.ifidx = WIFI_IF_STA;

    if (esp_now_is_peer_exist(CORE_MAC)) {
        esp_now_del_peer(CORE_MAC);
    }

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("코어 모듈 등록 실패");
        return false;
    }

    return true;
}

// =====================================================
// 센서 읽기 및 전송
// =====================================================
void sendSensorData() {
    SensorPacket packet = {};

    packet.lightRaw = analogRead(LDR_PIN);
    packet.motion = digitalRead(PIR_PIN) == HIGH ? 1 : 0;

    esp_err_t result = esp_now_send(
        CORE_MAC,
        reinterpret_cast<const uint8_t*>(&packet),
        sizeof(packet)
    );

    if (result != ESP_OK) {
        Serial.printf(
            "ESP-NOW 전송 요청 실패: %d\n",
            static_cast<int>(result)
        );
        return;
    }

    Serial.printf(
        "전송 완료 요청 | 조도 RAW: %u | PIR: %s\n",
        packet.lightRaw,
        packet.motion ? "감지" : "없음"
    );
}

// =====================================================
// 초기 설정
// =====================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    analogReadResolution(12);

    pinMode(LDR_PIN, INPUT);
    pinMode(PIR_PIN, INPUT);

    Serial.println();
    Serial.println("조도/PIR 센서 모듈 시작");

    if (!initializeEspNow()) {
        Serial.println("ESP-NOW 설정 실패");
        return;
    }

    Serial.print("센서 모듈 MAC: ");
    Serial.println(WiFi.macAddress());

    Serial.printf(
        "ESP-NOW 채널: %u\n",
        ESPNOW_CHANNEL
    );

    // PIR 워밍업
    Serial.println("PIR 워밍업 중...");

    delay(30000);

    Serial.println("PIR 준비 완료");

    // 시작 직후 한 번 전송
    sendSensorData();
    lastSendTime = millis();
}

// =====================================================
// 반복 실행
// =====================================================
void loop() {
    if (millis() - lastSendTime >= SEND_INTERVAL_MS) {
        lastSendTime = millis();
        sendSensorData();
    }

    delay(20);
}