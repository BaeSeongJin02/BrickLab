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
// 한국 시간
// =====================================================
const long GMT_OFFSET_SEC     = 9 * 3600;
const int  DAYLIGHT_OFFSET_SEC = 0;

// =====================================================
// 수신 패킷 구조체 (슬레이브와 동일해야 함)
// =====================================================
typedef struct __attribute__((packed)) {
    uint8_t  nodeId;
    uint16_t lightRaw;
    uint8_t  motion;
} SensorPacket;

typedef struct __attribute__((packed)) {
    uint8_t  nodeId;
    uint16_t distance;
    uint16_t strength;
    uint8_t  bedOccupied;
} LunaPacket;

// =====================================================
// 수신 버퍼
// =====================================================
static SensorPacket sensorBuf = {};
static LunaPacket   lunaBuf   = {};
static volatile bool newSensor = false;
static volatile bool newLuna   = false;
portMUX_TYPE packetMux = portMUX_INITIALIZER_UNLOCKED;

// =====================================================
// 타임스탬프
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
// ESP-NOW 수신 콜백
// =====================================================
void onDataReceived(const esp_now_recv_info_t* info,
                    const uint8_t* data, int len) {
    if (len < 1) return;
    uint8_t nodeId = data[0];

    portENTER_CRITICAL(&packetMux);
    if (nodeId == 0x01 && len == sizeof(SensorPacket)) {
        memcpy(&sensorBuf, data, sizeof(SensorPacket));
        newSensor = true;
    } else if (nodeId == 0x02 && len == sizeof(LunaPacket)) {
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

    // WiFi 연결
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Wi-Fi 연결 중");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500); Serial.print(".");
    }
    Serial.printf("\n연결됨: %s\n", WiFi.localIP().toString().c_str());

    // NTP 시간 동기화
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC,
               "pool.ntp.org", "time.nist.gov");
    Serial.println("NTP 동기화 중...");
    delay(2000);

    // ESP-NOW 초기화 (WiFi STA 유지한 채로)
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW 초기화 실패");
        return;
    }
    esp_now_register_recv_cb(onDataReceived);
    Serial.println("ESP-NOW 수신 대기 중...");
    Serial.printf("코어 MAC: %s\n", WiFi.macAddress().c_str());
}

// =====================================================
// loop
// =====================================================
void loop() {
    // PIR+조도 데이터 처리
    if (newSensor) {
        SensorPacket p;
        portENTER_CRITICAL(&packetMux);
        p = sensorBuf;
        newSensor = false;
        portEXIT_CRITICAL(&packetMux);

        String ts = getTimestamp();
        int lightPct = map(p.lightRaw, 0, 4095, 0, 100);

        Serial.printf("[PIR+조도] 조도: %d%% | 움직임: %s\n",
                      lightPct, p.motion ? "감지" : "없음");

        StaticJsonDocument<128> doc;
        doc["value"] = lightPct;
        doc["raw"]   = p.lightRaw;
        doc["timestamp"] = ts;
        String body; serializeJson(doc, body);
        sendToFirebase("/sensor_logs/light", body);

        doc.clear();
        doc["motion"]    = (bool)p.motion;
        doc["timestamp"] = ts;
        body = ""; serializeJson(doc, body);
        sendToFirebase("/sensor_logs/pir", body);
    }

    // TF-Luna 데이터 처리
    if (newLuna) {
        LunaPacket p;
        portENTER_CRITICAL(&packetMux);
        p = lunaBuf;
        newLuna = false;
        portEXIT_CRITICAL(&packetMux);

        static const char* BED_LABEL[] = {"비어있음","재실","불확실"};
        uint8_t idx = (p.bedOccupied < 3) ? p.bedOccupied : 2;

        Serial.printf("[TF-Luna] 거리: %u cm | 침대: %s\n",
                      p.distance, BED_LABEL[idx]);

        String ts = getTimestamp();
        StaticJsonDocument<192> doc;
        doc["distance"]    = p.distance;
        doc["strength"]    = p.strength;
        doc["bedOccupied"] = p.bedOccupied;
        doc["timestamp"]   = ts;
        String body; serializeJson(doc, body);
        sendToFirebase("/sensor_logs/lidar", body);
    }

    delay(10);
}