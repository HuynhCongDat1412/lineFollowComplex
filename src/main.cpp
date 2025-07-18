#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <LittleFS.h>

// ==== WIFI CONFIG ====
const char* ssid = "DAT PHUONG";
const char* password = "19201974";

// ==== SERVER ====
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ==== CONFIG STRUCTS ====
#define MAX_PATTERN_PER_SEG 5
#define MAX_PID_PER_SEG 5
#define MAX_SEGMENTS_PER_STAGE 4
#define MAX_STAGES 3

struct SegmentConfig {
    char type[16];
    char pattern[MAX_PATTERN_PER_SEG][8];
    int patternCount;
    float pid[MAX_PID_PER_SEG];
    int pidCount;
    uint16_t time; // optional
};

struct StageConfig {
    int stage;
    SegmentConfig segments[MAX_SEGMENTS_PER_STAGE];
    int segmentCount;
};

struct RobotState {
    int currentStage;
    int currentSegment;
    uint16_t segmentTime;
};

// ==== MOTOR CONTROL ====
const int PWMA = 5, AIN1 = 18, AIN2 = 22;
const int BIN1 = 4, BIN2 = 21, PWMB = 16;
const uint8_t sensors[5] = {35, 32, 33, 25, 26};
int M1_speed = 40, M2_speed = 40;

// ==== JSON CONFIG ====
StaticJsonDocument<2048> pidConfig;
const char* defaultConfig = R"([
  {
    "stage": 1,
    "segments": [
      {
        "type": "straight",
        "pattern": ["00100", "01100", "00110"],
        "pid": [40, 40, 0.1, 0, 0.2]
      },
      {
        "type": "square",
        "pattern": ["11000", "00001"],
        "pid": [40, 40, 0.5, 0, 0.2]
      }
    ]
  },
  {
    "stage": 2,
    "segments": [
      {
        "type": "straight",
        "pattern": ["00100", "01100", "00110"],
        "pid": [40, 40, 0.1, 0, 0.2]
      },
      {
        "type": "curve",
        "pattern": ["11100", "00111"],
        "pid": [40, 40, 0.5, 0, 0.2]
      }
    ]
  }
])";

uint8_t lastValidPattern[5] = {0,0,0,0,0};
uint8_t pattern[5] = {0,0,0,0,0};

StageConfig stages[MAX_STAGES];
int stageCount = 0;
RobotState robotState = {0, 0, 0};

void parseStages(const JsonDocument& doc, StageConfig* stages, int& stageCount) {
    stageCount = 0;
    if (!doc.is<JsonArray>()) {
        Serial.println("JSON root is not an array!");
        serializeJson(doc, Serial); // In ra để kiểm tra
        return;
    }
    JsonArray arr = doc.as<JsonArray>();
    for (JsonObject stageObj : arr) {
        if (stageCount >= MAX_STAGES) break;
        StageConfig& stage = stages[stageCount];
        stage.stage = stageObj["stage"];
        stage.segmentCount = 0;
        for (JsonObject segObj : stageObj["segments"].as<JsonArray>()) {
            if (stage.segmentCount >= MAX_SEGMENTS_PER_STAGE) break;
            SegmentConfig& seg = stage.segments[stage.segmentCount];
            strncpy(seg.type, segObj["type"] | "", sizeof(seg.type));
            seg.patternCount = 0;
            for (const char* pat : segObj["pattern"].as<JsonArray>()) {
                if (seg.patternCount >= MAX_PATTERN_PER_SEG) break;
                strncpy(seg.pattern[seg.patternCount], pat, sizeof(seg.pattern[0]));
                seg.patternCount++;
            }
            seg.pidCount = 0;
            for (JsonVariant v : segObj["pid"].as<JsonArray>()) {
                if (seg.pidCount >= MAX_PID_PER_SEG) break;
                seg.pid[seg.pidCount++] = v.as<float>();
            }
            seg.time = segObj.containsKey("time") ? (uint16_t)segObj["time"] : 0;
            stage.segmentCount++;
        }
        stageCount++;
    }
}

void saveToFS(const char* filename, const JsonDocument& doc) {
    File file = LittleFS.open(filename, "w");
    if (!file) {
        Serial.println("Failed to open file for writing");
        return;
    }
    serializeJson(doc, file);
    file.close();
    Serial.println("Configuration saved to " + String(filename));
}

bool loadFromFS(const char* filename, JsonDocument& doc) {
    File file = LittleFS.open(filename, "r");
    if (!file) {
        Serial.println("Failed to open file for reading");
        return false;
    }
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    if (error) {
        Serial.print("Failed to read file: ");
        Serial.println(error.c_str());
        return false;
    }
    return true;
}

// Đọc trạng thái line sensor vào mảng pattern
void getLinePattern(uint8_t pattern[5]) {
    bool hasLine = false;
    for (int i = 0; i < 5; i++) {
        pattern[i] = (digitalRead(sensors[i]) == LOW) ? 1 : 0;
        if (pattern[i] == 1) {
            hasLine = true;
        }
    }
    if (hasLine) {
        for (int i = 0; i < 5; i++) lastValidPattern[i] = pattern[i];
    } else {
        for (int i = 0; i < 5; i++) pattern[i] = lastValidPattern[i];
    }
}

// So khớp tuyệt đối pattern với chuỗi mẫu
bool matchPattern(const uint8_t* pattern, const char* patternStr) {
    for (int i = 0; i < 5; i++) {
        if ((patternStr[i] == '1' && pattern[i] != 1) ||
            (patternStr[i] == '0' && pattern[i] != 0)) return false;
    }
    return true;
}

int scoreMatch(const uint8_t* input, const char* pattern) {
    int score = 0;
    for (int i = 0; i < 5; i++) {
        if ((pattern[i] == '1' && input[i] == 1) ||
            (pattern[i] == '0' && input[i] == 0)) score++;
    }
    return score;
}

// Tìm loại segment khớp nhất với pattern hiện tại
const char* bestMatchType(const uint8_t* input, StageConfig* stages, int stageCount, uint16_t& segmentTime) {
    static char bestType[16] = "unknown";
    int bestScore = -1;
    for (int s = 0; s < stageCount; s++) {
        for (int seg = 0; seg < stages[s].segmentCount; seg++) {
            SegmentConfig& segment = stages[s].segments[seg];
            for (int p = 0; p < segment.patternCount; p++) {
                int score = scoreMatch(input, segment.pattern[p]);
                if (score > bestScore) {
                    segmentTime = segment.time;
                    bestScore = score;
                    strcpy(bestType, segment.type);
                }
            }
        }
    }
    return bestType;
}

// Cập nhật PID và tốc độ từ cấu hình theo loại segment
void setPIDFromType(const char* type, StageConfig* stages, int stageCount) {
    for (int s = 0; s < stageCount; s++) {
        for (int seg = 0; seg < stages[s].segmentCount; seg++) {
            SegmentConfig& segment = stages[s].segments[seg];
            if (strcmp(segment.type, type) == 0) {
                M1_speed = (int)segment.pid[0];
                M2_speed = (int)segment.pid[1];
                // Giả sử bạn có struct PID pid;
                // PID_init(&pid, segment.pid[2], segment.pid[3], segment.pid[4], -max(M1_speed, M2_speed), max(M1_speed, M2_speed));
                return;
            }
        }
    }
}

// Tính sai số line theo trọng số
float computeLineError(const uint8_t pattern[5]) {
    const int weights[5] = {-2, -1, 0, 1, 2};
    int sum = 0, count = 0;
    for (int i = 0; i < 5; i++) {
        if (pattern[i] == 1) {
            sum += weights[i];
            count++;
        }
    }
    if (count == 0) return 0;
    return (float)sum / count;
}

// Điều khiển động cơ
void setMotor(int pwmChannel, int in1, int in2, int speed) {
    if (speed > 0) {
        digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
        ledcWrite(pwmChannel, speed);
    } else {
        digitalWrite(in1, LOW); digitalWrite(in2, LOW);
        ledcWrite(pwmChannel, 0);
    }
}

// Truyền tốc độ cơ bản và correction cho từng động cơ
void driveMotors(int baseSpeedLeft, int baseSpeedRight, float correction) {
    int left = constrain(baseSpeedLeft + correction, 0, 100);
    int right = constrain(baseSpeedRight - correction, 0, 100);
    setMotor(0, AIN1, AIN2, map(left, 0, 100, 0, 255));
    setMotor(1, BIN2, BIN1, map(right, 0, 100, 0, 255));
}

// Xử lý tin nhắn WebSocket (Async)
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            String msg = "";
            for (size_t i = 0; i < len; i++) msg += (char)data[i];
            Serial.printf("[%u] Received text: %s\n", client->id(), msg.c_str());
            DeserializationError error = deserializeJson(pidConfig, msg);
            if (error) {
                Serial.print("JSON parse error: ");
                Serial.println(error.c_str());
            } else {
                saveToFS("/pid_config.json", pidConfig);
                parseStages(pidConfig, stages, stageCount);
                Serial.println("PID config updated from WebSocket!");
            }
            client->text(msg); // Echo lại cho client
        }
    }
}

void sendStatusPacket(const uint8_t* pattern, const char* type, float lineError, int leftSpeed, int rightSpeed) {
    String patternStr = "";
    for (int i = 0; i < 5; i++) patternStr += String(pattern[i]);
    String msg = "[";
    msg += "{\"cmd\":\"sensor\",\"value\":\"" + patternStr + "\"},";
    msg += "{\"cmd\":\"SegmentType\",\"value\":\"" + String(type) + "\"},";
    msg += "{\"cmd\":\"linePos\",\"value\":" + String(lineError, 3) + "},";
    msg += "{\"cmd\":\"pid\",\"value\":[" + String(leftSpeed) + "," + String(rightSpeed) + "]}";
    msg += "]";
    ws.textAll(msg);
}

void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);
    LittleFS.begin();
    if (!loadFromFS("/pid_config.json", pidConfig)) {
        deserializeJson(pidConfig, defaultConfig);
    }
    parseStages(pidConfig, stages, stageCount);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500); Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    Serial.println(WiFi.localIP());

    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    server.begin();

    pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
    for (int i = 0; i < 5; i++) pinMode(sensors[i], INPUT_PULLUP);
    ledcSetup(0, 1000, 8); ledcAttachPin(PWMA, 0);
    ledcSetup(1, 1000, 8); ledcAttachPin(PWMB, 1);
}

static unsigned long fregControl = 10;
void loop() {
    static unsigned long lastControlTime = millis();
    static uint16_t segmentTime = 0;
    static char currentType[16] = "unknown";

    if (millis() - lastControlTime >= fregControl) {
        lastControlTime = millis();
        getLinePattern(pattern);
        const char* type = bestMatchType(pattern, stages, stageCount, segmentTime);
        if (strcmp(currentType, type) != 0) {
            strcpy(currentType, type);
            setPIDFromType(type, stages, stageCount);
        }
        float error = computeLineError(pattern);
        float correction = error * 10; // Thay bằng PID thực tế nếu có
        driveMotors(M1_speed, M2_speed, correction);
    }

    static unsigned long lastStatusSend = 0;
    if (millis() - lastStatusSend >= 2000) {
        lastStatusSend = millis();
        sendStatusPacket(pattern, currentType, computeLineError(pattern), M1_speed, M2_speed);
    }
}