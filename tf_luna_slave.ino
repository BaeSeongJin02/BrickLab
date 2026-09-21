#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// =====================================================
// RTC 메모리 — 딥슬립 후에도 값 유지
// =====================================================
RTC_DATA_ATTR static uint8_t  lastBedState = 2;   // 하이스테리시스: 이전 침대 상태
RTC_DATA_ATTR static uint32_t bootCount    = 0;   // 부팅 횟수 (디버그용)

// =====================================================
// TF-Luna UART 핀 (Serial2)
// ESP32 GPIO16(RX2) ← TF-Luna TX
// ESP32 GPIO17(TX2) → TF-Luna RX
// =====================================================
const uint8_t  LUNA_RX_PIN = 16;
const uint8_t  LUNA_TX_PIN = 17;
const uint32_t LUNA_BAUD   = 115200;

// =====================================================
// 침대 재실 판단 임계값 (cm)
// 실제 설치 후 환경에 맞게 조정 권장
// =====================================================
const uint16_t OCCUPIED_THRESHOLD = 80;   // 이하 → 재실
const uint16_t EMPTY_THRESHOLD    = 120;  // 이상 → 비어있음
// 80~120cm 구간: 하이스테리시스로 이전 상태 유지

// 신호 강도 최솟값 (이하면 노이즈로 판단, 샘플 제외)
const uint16_t MIN_STRENGTH = 100;

// 한 번 전송 시 측정 횟수 (평균 계산용)
const uint8_t SAMPLE_COUNT = 5;

// =====================================================
// 코어 모듈 MAC 주소
// =====================================================
const uint8_t CORE_MAC[6] = {
    0xD4, 0xE9, 0xF4, 0xE3, 0x67, 0xE8
};

const uint8_t ESPNOW_CHANNEL = 1;

// =====================================================
// 딥슬립 주기
// 측정+전송 소요 ~3~4초 + 딥슬립 26초 ≒ 30초 주기
// =====================================================
const uint64_t DEEP_SLEEP_US = 26ULL * 1000000ULL;

// =====================================================
// 전송 패킷
// nodeId: 0x01 = PIR/조도 슬레이브
//         0x02 = TF-Luna 슬레이브 (이 노드)
// bedOccupied: 0=비어있음, 1=재실, 2=불확실
// =====================================================
typedef struct __attribute__((packed)) {
    uint8_t  nodeId;
    uint16_t distance;    // 평균 거리 (cm), 읽기 실패 시 0xFFFF
    uint16_t strength;    // 평균 신호 강도
    uint8_t  bedOccupied;
} LunaPacket;

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
// TF-Luna 프레임 출력률 설정
// 명령 프레임: 5A 06 03 [Hz_L] [Hz_H] [checksum]
// 측정 전 10Hz로 낮춰 안정된 샘플 확보 + 전력 절약
// =====================================================
void setLunaFrameRate(uint16_t hz) {
    uint8_t cmd[6];
    cmd[0] = 0x5A;
    cmd[1] = 0x06;
    cmd[2] = 0x03;
    cmd[3] = static_cast<uint8_t>(hz & 0xFF);
    cmd[4] = static_cast<uint8_t>((hz >> 8) & 0xFF);
    cmd[5] = 0;
    for (uint8_t i = 0; i < 5; i++) cmd[5] += cmd[i];

    Serial2.write(cmd, 6);
    delay(150);
    // 응답 비우기
    while (Serial2.available()) Serial2.read();
}

// =====================================================
// TF-Luna 단일 프레임 읽기
// 프레임 구조 (9바이트):
//   [0] 0x59  [1] 0x59  헤더
//   [2] Dist_L  [3] Dist_H   거리 (cm)
//   [4] Str_L   [5] Str_H    신호 강도
//   [6] Temp_L  [7] Temp_H   온도 (사용 안 함)
//   [8] 체크섬 (바이트 0~7 합산 mod 256)
// ※ 버퍼 플러시는 readLunaAverage()에서 한 번만 수행
// =====================================================
bool readLunaFrame(uint16_t &dist, uint16_t &strength) {
    uint8_t buf[9];
    // 10Hz 출력 기준 100ms 간격, 여유를 두어 300ms 타임아웃
    unsigned long deadline = millis() + 300;

    while (millis() < deadline) {
        if (Serial2.available() < 1) continue;

        // 헤더 첫 바이트(0x59) 아니면 버림
        if (Serial2.peek() != 0x59) {
            Serial2.read();
            continue;
        }

        // 9바이트 전부 쌓일 때까지 대기
        if (Serial2.available() < 9) continue;

        Serial2.readBytes(buf, 9);

        // 헤더 재확인
        if (buf[0] != 0x59 || buf[1] != 0x59) continue;

        // 체크섬 검증
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
// SAMPLE_COUNT 번 측정 → 유효 샘플 평균 반환
// 측정 시작 전 버퍼를 한 번만 비움
// =====================================================
bool readLunaAverage(uint16_t &avgDist, uint16_t &avgStr) {
    // 쌓인 이전 데이터 한 번만 비움
    while (Serial2.available()) Serial2.read();
    delay(50);  // TF-Luna가 새 프레임 시작할 여유 시간

    uint32_t sumDist = 0;
    uint32_t sumStr  = 0;
    uint8_t  valid   = 0;

    for (uint8_t i = 0; i < SAMPLE_COUNT; i++) {
        uint16_t d = 0, s = 0;

        if (readLunaFrame(d, s)
            && s >= MIN_STRENGTH
            && d > 0
            && d < 1200) {
            sumDist += d;
            sumStr  += s;
            valid++;
        }

        // 10Hz → 100ms 간격, 다음 프레임 도착 전 여유 20ms
        delay(120);
    }

    if (valid == 0) return false;

    avgDist = static_cast<uint16_t>(sumDist / valid);
    avgStr  = static_cast<uint16_t>(sumStr  / valid);
    return true;
}

// =====================================================
// 거리 → 침대 재실 상태 분류
// 하이스테리시스: 80~120cm 불확실 구간에서 이전 상태 유지
// lastBedState는 RTC 메모리 → 딥슬립 후에도 보존
// =====================================================
uint8_t classifyBedState(uint16_t dist) {
    if (dist <= OCCUPIED_THRESHOLD) {
        lastBedState = 1;
        return 1;  // 재실
    }
    if (dist >= EMPTY_THRESHOLD) {
        lastBedState = 0;
        return 0;  // 비어있음
    }
    // 불확실 구간 → RTC 메모리의 이전 상태 반환
    return lastBedState;
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

    // 전송 결과 확인 콜백 등록
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
// 센서 읽기 → 패킷 구성 → 전송 → ACK 대기
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

    // 전송 전 플래그 초기화
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

    static const char* const STATE_LABEL[] = {
        "비어있음", "재실", "불확실"
    };
    const uint8_t stateIdx = (packet.bedOccupied < 3)
                             ? packet.bedOccupied : 2;

    Serial.printf(
        "전송 %s | 거리: %u cm | 강도: %u | 침대: %s\n",
        s_sendSuccess ? "성공" : "실패(ACK없음)",
        packet.distance,
        packet.strength,
        STATE_LABEL[stateIdx]
    );
}

// =====================================================
// 초기 설정
// 딥슬립 구조에서 setup()이 매 주기 메인 루틴 역할
// =====================================================
void setup() {
    Serial.begin(115200);
    delay(200);

    bootCount++;
    Serial.printf("\n[부팅 #%u] TF-Luna 슬레이브 노드 시작\n", bootCount);

    // TF-Luna UART 시작
    Serial2.begin(LUNA_BAUD, SERIAL_8N1, LUNA_RX_PIN, LUNA_TX_PIN);
    delay(200);

    // TF-Luna 출력률 10Hz로 설정 (기본 100Hz → 전력 절약, 버퍼 제어)
    setLunaFrameRate(10);

    if (!initializeEspNow()) {
        Serial.println("ESP-NOW 설정 실패 — 딥슬립 진입");
    } else {
        Serial.print("슬레이브 MAC: ");
        Serial.println(WiFi.macAddress());
        Serial.printf("ESP-NOW 채널: %u\n", ESPNOW_CHANNEL);

        sendSensorData();
    }

    // Wi-Fi 끄고 딥슬립 진입
    WiFi.mode(WIFI_OFF);
    Serial.printf("딥슬립 %.0fs 진입 (총 주기 ~30초)\n",
                  DEEP_SLEEP_US / 1e6);
    Serial.flush();
    esp_deep_sleep(DEEP_SLEEP_US);
}

// =====================================================
// 딥슬립 구조에서 loop()는 실행되지 않음
// =====================================================
void loop() {}
