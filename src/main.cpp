#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include "pid.h"
#include <littleFS.h>

// ==== WIFI CONFIG ====
const char* ssid = "DAT PHUONG";
const char* password = "19201974";

// ==== SERVER ====
WebServer server(80);
WebSocketsServer webSocketServer = WebSocketsServer(81);

// ==== MOTOR CONTROL ====
const int PWMA = 5, AIN1 = 18, AIN2 = 22;
const int BIN1 = 4, BIN2 = 21, PWMB = 16;

// ==== SENSOR ====
const uint8_t sensors[5] = {35, 32, 33, 25, 26};
uint8_t pattern[5] = {0};

// ==== PID ====
PIDController pid;
uint8_t M1_speed = 0, M2_speed = 0;

// ==== JSON CONFIG ====
StaticJsonDocument<2048> pidConfig;
const char* defaultConfig = R"([
  {
    "stage": 0,
    "segments": [
      {
        "type": "straight",
        "pattern": ["00100"],
        "pid": [0, 0, 0, 0, 0]
      }
    ]
  }
])";

uint8_t lastValidPattern[5] = {0,0,0,0,0};


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
    }
    else if (!hasLine) {
        // Nếu không có line, giữ nguyên pattern cuối cùng
        for (int i = 0; i < 5; i++) {pattern[i] = lastValidPattern[i];}
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
uint8_t segmentTime = 0;
const char* bestMatchType(const uint8_t* input, JsonDocument& doc) {
    static char bestType[16] = "unknown";
    int bestScore = -1;
    JsonArray segments = doc[0]["segments"];
    for (JsonObject segment : segments) {
        const char* type = segment["type"];
        JsonArray patterns = segment["pattern"];
        for (const char* pat : patterns) {
            int score = scoreMatch(input, pat);
            if (score > bestScore) {
                segmentTime = segment.containsKey("time") ? segment["time"] : 0;
                bestScore = score;
                strcpy(bestType, type);
            }
        }
    }
    return bestType;
}

// Tìm loại segment khớp nhất với pattern hiện tại
const char* detectSegmentType(const uint8_t* pattern, JsonDocument& doc) {
    static char resultType[16] = "unknown";
    JsonArray segments = doc[0]["segments"];
    for (JsonObject segment : segments) {
        const char* type = segment["type"];
        JsonArray patterns = segment["pattern"];
        for (const char* p : patterns) {
            if (matchPattern(pattern, p)) {
                strcpy(resultType, type);
                return resultType;
            }
        }
    }
    return "unknown";
}


// Cập nhật PID và tốc độ từ cấu hình JSON theo loại segment
void setPIDFromType(const char* type, JsonDocument& doc) {
    JsonArray segments = doc[0]["segments"];
    for (JsonObject segment : segments) {
        if (strcmp(segment["type"], type) == 0) {
            JsonArray pidArray = segment["pid"];
            M1_speed = pidArray[0].as<int>();
            M2_speed = pidArray[1].as<int>();
            int16_t maxPidOutput = max(M1_speed, M2_speed);
            float Kp = pidArray[2];
            float Ki = pidArray[3];
            float Kd = pidArray[4];
            PID_init(&pid, Kp, Ki, Kd, -maxPidOutput, maxPidOutput);
            break;
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

bool pidConfigJustUpdated = false;
// Xử lý tin nhắn WebSocket
void handleWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_TEXT: {
            Serial.printf("[%u] Received text: %s\n", num, payload);
            DeserializationError error = deserializeJson(pidConfig, (char*)payload);
            if (error) {
                Serial.print("JSON parse error: ");
                Serial.println(error.c_str());
            } else {
                Serial.println("------------------------------------------------------------");
                saveToFS("/pid_config.json", pidConfig);
                pidConfigJustUpdated = true; // Đánh dấu để cập nhật lại PID
                Serial.println("PID config updated from WebSocket!");
            }
            webSocketServer.sendTXT(num, payload, length); // Echo lại cho client
            break;
        }
        default:
            break;
    }
}

void sendStatusPacket(const uint8_t* pattern, const char* type, float lineError, int leftSpeed, int rightSpeed) {
    String patternStr = "";
    for (int i = 0; i < 5; i++) patternStr += String(pattern[i]);

    String msg = "[";
    msg += "{\"cmd\":\"sensor\",\"value\":\"" + patternStr + "\"},";
    msg += "{\"cmd\":\"SegmentType\",\"value\":\"" + String(type) + "\"},";
    msg += "{\"cmd\":\"linePos\",\"value\":" + String(lineError, 3) + "},";
    msg += "{\"cmd\":\"pid\",\"value\":[" + String(leftSpeed) + "," + String(rightSpeed) + "," +
           String(pid.kp, 3) + "," + String(pid.ki, 3) + "," + String(pid.kd, 3) + "]}";
    msg += "]";

    webSocketServer.broadcastTXT(msg);
}
String currentType = "unknown";
String lastDetectedType = "unknown";
int stableCount = 0;
const int stableThreshold = 3;
unsigned long lastTypeChange = 0;
const unsigned long minTypeDuration = 200;

void updateSegmentType(const char* newType, JsonDocument& doc) {
    if (segmentTime*1000 > millis() - lastTypeChange) {
        return; // Không thay đổi nếu chưa đủ thời gian
    }
  if (strcmp(newType, lastDetectedType.c_str()) == 0) {
    stableCount++;
  } else {
    stableCount = 1;
    lastDetectedType = newType;
  }

  if ((millis() - lastTypeChange > minTypeDuration) && (stableCount >= stableThreshold)) {
    if (currentType != newType) {
      currentType = newType;
      lastTypeChange = millis();
      Serial.print("[Segment] Type changed to: ");
      Serial.println(currentType);
      setPIDFromType(newType, doc);  // Truyền đúng kiểu JsonDocument&
      PID_reset(&pid);
    }
  }
}



void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);
    LittleFS.begin();
    if (!loadFromFS("/pid_config.json", pidConfig)) {
        deserializeJson(pidConfig, defaultConfig);
    }
    while (WiFi.status() != WL_CONNECTED) {
        delay(500); Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    Serial.println(WiFi.localIP());
    server.begin();
    webSocketServer.begin();
    webSocketServer.onEvent(handleWebSocketEvent);
    // Setup pins
    pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
    for (int i = 0; i < 5; i++) pinMode(sensors[i], INPUT_PULLUP);
    ledcSetup(0, 1000, 8); ledcAttachPin(PWMA, 0);
    ledcSetup(1, 1000, 8); ledcAttachPin(PWMB, 1);
    // Default PID
    DeserializationError error = deserializeJson(pidConfig, defaultConfig);
    if (error) {
        Serial.println("Default PID config parse error!");
    }
    setPIDFromType(detectSegmentType(pattern, pidConfig), pidConfig);
}


static unsigned long fregControl = 10;
void loop() {
    webSocketServer.loop();  // Cập nhật WebSocket'
    static unsigned long lastControlTime = millis();


    if (millis() - lastControlTime >= fregControl) {
        lastControlTime = millis();
            // Đọc line sensor
        getLinePattern(pattern);
        // Tìm loại segment phù hợp nhất
        const char* type = bestMatchType(pattern, pidConfig);

        // Cập nhật loại segment nếu ổn định đủ lâu và vượt quá thời gian mỗi segment
        updateSegmentType(type, pidConfig);
        // Nếu loại segment thay đổi, cập nhật PID
        if (pidConfigJustUpdated) {
            setPIDFromType(currentType.c_str(), pidConfig);  // cập nhật lại PID hiện tại
            pidConfigJustUpdated = false;
        }
        // Tính sai số line
        float error = computeLineError(pattern);

        // Tính toán điều khiển PID
        float correction = PID_compute(&pid,0.0, error);

        // Truyền tín hiệu điều khiển động cơ
        driveMotors(M1_speed, M2_speed, correction);

        static unsigned long lastSend = 0;

    } else {
        return; // Không thực hiện điều khiển nếu chưa đến thời gian
    }
    static unsigned long lastStatusSend = 0;
    if (millis() - lastStatusSend >= 2000) {
        lastStatusSend = millis();
        // Gửi gói trạng thái qua WebSocket
        sendStatusPacket(pattern, currentType.c_str(), computeLineError(pattern), M1_speed, M2_speed);
    }
}