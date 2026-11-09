#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// =====================================================
// RTC 메모리 — 딥슬립 후에도 값 유지
// =====================================================
RTC_DATA_ATTR static uint8_t  lastBedState = 2;
RTC_DATA_ATTR static uint32_t bootCount    = 0;

// =====================================================
// TF-Luna UART 핀 (Serial2)
// =====================================================
const uint8_t  LUNA_RX_PIN = 16;
const uint8_t  LUNA_TX_PIN = 17;
const uint32_t LUNA_BAUD   = 115200;

// =====================================================
// 침대 재실 판단 임계값 (cm)
// =====================================================
const uint16_t OCCUPIED_THRESHOLD = 80;
const uint16_t EMPTY_THRESHOLD    = 120;
const uint16_t MIN_STRENGTH       = 100;
const uint8_t  SAMPLE_COUNT       = 5;

// =====================================================
// 코어 모듈 MAC 주소
// =====================================================
const uint8_t CORE_MAC[6] = {
    0xD4, 0xE9, 0xF4, 0xE3, 0x67, 0xE8
};

// ★ 채널 4로 수정 (AP 채널 확인 결과)
const uint8_t ESPNOW_CHANNEL = 4;

// =====================================================
// 딥슬립 주기 (26초 슬립 + ~4초 작업 ≒ 30초 주기)
// =====================================================
const uint64_t DEEP_SLEEP_US = 26ULL * 1000000ULL;

// =====================================================
// 전송 패킷
// =====================================================
typedef struct __attribute__((packed)) {
    uint8_t  nodeId;
    uint16_t distance;
    uint16_t strength;
    uint8_t  bedOccupied;
} LunaPacket;

// =====================================================
// ESP-NOW 전송 완료 콜백
// =====================================================
static volatile bool s_sendDone    = false;
static volatile bool s_sendSuccess = false;

void IRAM_ATTR onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
    s_sendSuccess = (status == ESP_NOW_SEND_SUCCESS);
    s_sendDone    = true;
}

// =====================================================
// TF-Luna 출력률 설정
// =====================================================
void setLunaFrameRate(uint16_t hz) {
    uint8_t cmd[6];
    cmd[0] = 0x5A; cmd[1] = 0x06; cmd[2] = 0x03;
    cmd[3] = static_cast<uint8_t>(hz & 0xFF);
    cmd[4] = static_cast<uint8_t>((hz >> 8) & 0xFF);
    cmd[5] = 0;
    for (uint8_t i = 0; i < 5; i++) cmd[5] += cmd[i];
    Serial2.write(cmd, 6);
    delay(150);
    while (Serial2.available()) Serial2.read();
}

// =====================================================
// TF-Luna 단일 프레임 읽기 (개선된 동기화)
// =====================================================
bool readLunaFrame(uint16_t &dist, uint16_t &strength) {
    uint8_t buf[9];
    unsigned long deadline = millis() + 300;

    while (millis() < deadline) {
        if (!Serial2.available()) { delay(1); continue; }

        // 헤더 첫 바이트 탐색
        if (Serial2.peek() != 0x59) { Serial2.read(); continue; }

        // 9바이트 대기 (최대 100ms)
        unsigned long byteDeadline = millis() + 100;
        while (Serial2.available() < 9 && millis() < byteDeadline) delay(1);
        if (Serial2.available() < 9) continue;

        Serial2.readBytes(buf, 9);

        if (buf[0] != 0x59 || buf[1] != 0x59) continue;

        uint8_t checksum = 0;
        for (uint8_t i = 0; i < 8; i++) checksum += buf[i];
        if (checksum != buf[8]) continue;

        dist     = buf[2] | (static_cast<uint16_t>(buf[3]) << 8);
        strength = buf[4] | (static_cast<uint16_t>(buf[5]) << 8);
        return true;
    }
    return false;
}

// =====================================================
// SAMPLE_COUNT 번 측정 → 평균
// =====================================================
bool readLunaAverage(uint16_t &avgDist, uint16_t &avgStr) {
    while (Serial2.available()) Serial2.read();
    delay(50);

    uint32_t sumDist = 0, sumStr = 0;
    uint8_t  valid   = 0;

    for (uint8_t i = 0; i < SAMPLE_COUNT; i++) {
        uint16_t d = 0, s = 0;
        if (readLunaFrame(d, s) && s >= MIN_STRENGTH && d > 0 && d < 1200) {
            sumDist += d;
            sumStr  += s;
            valid++;
        }
        delay(120);
    }

    if (valid == 0) return false;
    avgDist = static_cast<uint16_t>(sumDist / valid);
    avgStr  = static_cast<uint16_t>(sumStr  / valid);
    return true;
}

// =====================================================
// 거리 → 침대 재실 상태 (하이스테리시스)
// =====================================================
uint8_t classifyBedState(uint16_t dist) {
    if (dist <= OCCUPIED_THRESHOLD) { lastBedState = 1; return 1; }
    if (dist >= EMPTY_THRESHOLD)    { lastBedState = 0; return 0; }
    return lastBedState;
}

// =====================================================
// ESP-NOW 초기화
// =====================================================
bool initializeEspNow() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    if (esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
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

    if (esp_now_is_peer_exist(CORE_MAC)) esp_now_del_peer(CORE_MAC);

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("코어 모듈 등록 실패");
        return false;
    }
    return true;
}

// =====================================================
// 센서 읽기 → 패킷 전송
// =====================================================
void sendSensorData() {
    uint16_t avgDist = 0, avgStr = 0;
    const bool ok = readLunaAverage(avgDist, avgStr);

    LunaPacket packet = {};
    packet.nodeId = 0x02;

    if (!ok) {
        Serial.println("TF-Luna 읽기 실패 — 불확실 상태로 전송");
        packet.distance    = 0xFFFF;
        packet.strength    = 0;
        packet.bedOccupied = 2;
    } else {
        packet.distance    = avgDist;
        packet.strength    = avgStr;
        packet.bedOccupied = classifyBedState(avgDist);
    }

    s_sendDone = s_sendSuccess = false;

    esp_err_t result = esp_now_send(
        CORE_MAC,
        reinterpret_cast<const uint8_t*>(&packet),
        sizeof(packet)
    );

    if (result != ESP_OK) {
        Serial.printf("ESP-NOW 전송 요청 실패: %d\n", static_cast<int>(result));
        return;
    }

    // ACK 대기 (최대 2초)
    const unsigned long cbDeadline = millis() + 2000;
    while (!s_sendDone && millis() < cbDeadline) delay(10);

    static const char* const STATE_LABEL[] = { "비어있음", "재실", "불확실" };
    const uint8_t stateIdx = (packet.bedOccupied < 3) ? packet.bedOccupied : 2;

    Serial.printf("전송 %s | 거리: %u cm | 강도: %u | 침대: %s\n",
                  s_sendSuccess ? "성공" : "실패(ACK없음)",
                  packet.distance, packet.strength,
                  STATE_LABEL[stateIdx]);
}

// =====================================================
// setup (딥슬립 구조에서 매 주기 메인 루틴)
// =====================================================
void setup() {
    Serial.begin(115200);
    delay(200);

    bootCount++;
    Serial.printf("\n[부팅 #%u] TF-Luna 슬레이브 노드 시작\n", bootCount);

    Serial2.begin(LUNA_BAUD, SERIAL_8N1, LUNA_RX_PIN, LUNA_TX_PIN);
    delay(200);
    setLunaFrameRate(10);

    if (!initializeEspNow()) {
        Serial.println("ESP-NOW 설정 실패 — 딥슬립 진입");
    } else {
        Serial.print("슬레이브 MAC: ");
        Serial.println(WiFi.macAddress());
        Serial.printf("ESP-NOW 채널: %u\n", ESPNOW_CHANNEL);
        sendSensorData();
    }

    // ★ ESP-NOW 정리 후 WiFi OFF → 딥슬립 (안정적인 순서)
    esp_now_deinit();
    WiFi.mode(WIFI_OFF);
    Serial.printf("딥슬립 %.0fs 진입 (총 주기 ~30초)\n", DEEP_SLEEP_US / 1e6);
    Serial.flush();
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_US);   // ★ 타이머 소스 명시 등록
    esp_deep_sleep_start();                          // ★ 슬립 진입
}

void loop() {}
