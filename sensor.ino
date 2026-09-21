#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// =====================================================
// PIR 센서 핀
// =====================================================
const uint8_t PIR_PIN = 14;

// =====================================================
// 조도 센서 핀 (아날로그)
// =====================================================
const uint8_t LDR_PIN = 34;

// =====================================================
// 코어 모듈 MAC 주소
// =====================================================
const uint8_t CORE_MAC[6] = {
    0xD4, 0xE9, 0xF4, 0xE3, 0x67, 0xE8
};

const uint8_t ESPNOW_CHANNEL = 1;

// 전송 주기: 30초
const unsigned long SEND_INTERVAL_MS = 30000;

unsigned long lastSendTime = 0;

// =====================================================
// 전송 패킷
// nodeId: 0x01 = PIR/조도 슬레이브 (이 노드)
//         0x02 = TF-Luna 슬레이브
// motion: 0 = 감지 없음, 1 = 움직임 감지
// =====================================================
typedef struct __attribute__((packed)) {
    uint8_t  nodeId;      // 0x01 고정
    uint16_t lightRaw;    // 조도 ADC 원시값 (0~4095)
    uint8_t  motion;      // PIR 상태 (0 or 1)
} SensorPacket;

// =====================================================
// ESP-NOW 전송 완료 콜백 플래그
// =====================================================
static volatile bool s_sendDone    = false;
static volatile bool s_sendSuccess = false;

void IRAM_ATTR onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
    s_sendSuccess = (status == ESP_NOW_SEND_SUCCESS);
    s_sendDone    = true;
}

// =====================================================
// ESP-NOW 초기화 + 전송 콜백 등록
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

    esp_now_register_send_cb(onDataSent);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, CORE_MAC, sizeof(CORE_MAC));
    peerInfo.channel = ESPNOW_CHANNEL;
    peerInfo.encrypt = false;
    peerInfo.ifidx   = WIFI_IF_STA;

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
    packet.nodeId   = 0x01;
    packet.lightRaw = static_cast<uint16_t>(analogRead(LDR_PIN));
    packet.motion   = digitalRead(PIR_PIN) ? 1 : 0;

    s_sendDone = s_sendSuccess = false;

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

    // 콜백 대기 (최대 2초)
    const unsigned long cbDeadline = millis() + 2000;
    while (!s_sendDone && millis() < cbDeadline) {
        delay(10);
    }

    Serial.printf(
        "전송 %s | 조도: %u | 움직임: %s\n",
        s_sendSuccess ? "성공" : "실패(ACK없음)",
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

    pinMode(PIR_PIN, INPUT);
    analogReadResolution(12);  // 12비트 ADC (0~4095)

    Serial.println();
    Serial.println("PIR+조도 슬레이브 노드 시작");

    if (!initializeEspNow()) {
        Serial.println("ESP-NOW 설정 실패 — 재부팅 필요");
        return;
    }

    Serial.print("슬레이브 MAC: ");
    Serial.println(WiFi.macAddress());
    Serial.printf("ESP-NOW 채널: %u\n", ESPNOW_CHANNEL);

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
