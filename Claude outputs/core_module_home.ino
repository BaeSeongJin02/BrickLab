#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <esp_now.h>
#include <time.h>

// =====================================================
// Wi-Fi 설정 (2.4GHz 필수)
// =====================================================
const char* WIFI_SSID = "U+Net91DC";
const char* WIFI_PASS = "40584HEF@5";

// =====================================================
// Firebase 설정
// =====================================================
const char* FIREBASE_HOST =
    "sleep-monitoring-f630c-default-rtdb.firebaseio.com";

// =====================================================
// 한국 시간 (UTC+9)
// =====================================================
const long GMT_OFFSET_SEC      = 9 * 3600;
const int  DAYLIGHT_OFFSET_SEC = 0;

// =====================================================
// 로컬 센서 핀
// =====================================================
const uint8_t PIR_PIN = 14;   // PIR OUT
const uint8_t LDR_PIN = 34;   // LDR S (아날로그)

// 로컬 센서 전송 주기 (ms)
const uint32_t LOCAL_SEND_INTERVAL = 30000;  // 30초

// =====================================================
// TF-Luna 수신 패킷 구조체 (슬레이브와 동일해야 함)
// =====================================================
typedef struct __attribute__((packed)) {
    uint8_t  nodeId;
    uint16_t distance;
    uint16_t strength;
    uint8_t  bedOccupied;
} LunaPacket;

// =====================================================
// 수신 버퍼
// =====================================================
static LunaPacket        lunaBuf = {};
static volatile bool     newLuna = false;
portMUX_TYPE packetMux = portMUX_INITIALIZER_UNLOCKED;

// =====================================================
// 타임스탬프 (ISO8601)
// =====================================================
String getTimestamp() {
    struct tm t;
    if (!getLocalTime(&t)) return "unknown";
    char buf[30];
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &t);
    return String(buf);
}

// =====================================================
// Firebase PUT
// =====================================================
bool sendToFirebase(const String& path, const String& body) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Firebase 전송 실패: Wi-Fi 끊김");
        return false;
    }
    HTTPClient http;
    String url = "https://" + String(FIREBASE_HOST) + path + ".json";
    http.begin(url);
    http.addHeader("Content-Type", "application/json");
    int code = http.PUT(body);
    Serial.printf("Firebase [%s] → %d\n", path.c_str(), code);
    http.end();
    return code >= 200 && code < 300;
}

// =====================================================
// 로컬 PIR + 조도 읽기 → Firebase 전송
// =====================================================
void sendLocalSensors() {
    uint16_t lightRaw = analogRead(LDR_PIN);
    uint8_t  motion   = digitalRead(PIR_PIN);
    int      lightPct = map(lightRaw, 0, 4095, 0, 100);
    String   ts       = getTimestamp();

    Serial.printf("[로컬] 조도: %d%% (raw:%u) | 움직임: %s\n",
                  lightPct, lightRaw, motion ? "감지" : "없음");

    // 조도 전송
    {
        JsonDocument doc;
        doc["value"]     = lightPct;
        doc["raw"]       = lightRaw;
        doc["timestamp"] = ts;
        String body;
        serializeJson(doc, body);
        sendToFirebase("/sensor_logs/light", body);
    }

    // PIR 전송
    {
        JsonDocument doc;
        doc["motion"]    = (bool)motion;
        doc["timestamp"] = ts;
        String body;
        serializeJson(doc, body);
        sendToFirebase("/sensor_logs/pir", body);
    }
}

// =====================================================
// ESP-NOW 수신 콜백 (TF-Luna 슬레이브 nodeId=0x02)
// =====================================================
void onDataReceived(const esp_now_recv_info_t* info,
                    const uint8_t* data, int len) {
    if (len < 1) return;
    uint8_t nodeId = data[0];

    portENTER_CRITICAL(&packetMux);
    if (nodeId == 0x02 && len == (int)sizeof(LunaPacket)) {
        memcpy(&lunaBuf, data, sizeof(LunaPacket));
        newLuna = true;
    }
    portEXIT_CRITICAL(&packetMux);
}

// =====================================================
// setup
// =====================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== 코어 모듈 시작 ===");

    pinMode(PIR_PIN, INPUT);

    // WiFi 연결
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Wi-Fi 연결 중");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\n연결됨: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("코어 MAC: %s\n", WiFi.macAddress().c_str());
    Serial.printf("AP 채널: %d\n", WiFi.channel());   // ★ 채널 확인용

    // NTP 시간 동기화
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC,
               "pool.ntp.org", "time.nist.gov");
    Serial.print("NTP 동기화 중");
    for (int i = 0; i < 10; i++) {
        delay(500);
        Serial.print(".");
        struct tm t;
        if (getLocalTime(&t, 0)) { Serial.println(" 완료"); break; }
    }

    // ESP-NOW 초기화 (WiFi STA 유지한 채로)
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW 초기화 실패");
        return;
    }
    esp_now_register_recv_cb(onDataReceived);
    Serial.println("ESP-NOW 수신 대기 중 (TF-Luna)...");
}

// =====================================================
// loop
// =====================================================
void loop() {
    // 로컬 센서 주기적 전송
    static uint32_t lastLocal = 0;
    if (millis() - lastLocal >= LOCAL_SEND_INTERVAL) {
        lastLocal = millis();
        sendLocalSensors();
    }

    // TF-Luna ESP-NOW 데이터 처리
    if (newLuna) {
        LunaPacket p;
        portENTER_CRITICAL(&packetMux);
        p = lunaBuf;
        newLuna = false;
        portEXIT_CRITICAL(&packetMux);

        static const char* BED_LABEL[] = { "비어있음", "재실", "불확실" };
        uint8_t idx = (p.bedOccupied < 3) ? p.bedOccupied : 2;

        Serial.printf("[TF-Luna] 거리: %u cm | 강도: %u | 침대: %s\n",
                      p.distance, p.strength, BED_LABEL[idx]);

        String ts = getTimestamp();
        JsonDocument doc;
        doc["distance"]    = p.distance;
        doc["strength"]    = p.strength;
        doc["bedOccupied"] = p.bedOccupied;
        doc["timestamp"]   = ts;
        String body;
        serializeJson(doc, body);
        sendToFirebase("/sensor_logs/lidar", body);
    }

    delay(10);
}
