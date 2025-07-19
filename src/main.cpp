#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include "pid.h"
#include <littleFS.h>

// ==== WIFI CONFIG ====
// const char* ssid = "AndroidAP";
// const char* password = "12345678";
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

bool pidConfigJustUpdated = false;

String lastType = "unknown";
uint16_t lastSegmentTime = 0;
bool inSegmentTiming = false;
unsigned long segmentStartTime = 0;
uint16_t segmentTime = 0;

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
void handleJsonMessage(const String& msg);
void handleJsonMessage(const String& msg) ;

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

bool invertLine = false; // trường hợp line đen nền trắng
// Đọc trạng thái line sensor vào mảng pattern
void getLinePattern(uint8_t pattern[5]) {
    bool hasLine = false;
    for (int i = 0; i < 5; i++) {
        uint8_t val = (digitalRead(sensors[i]) == LOW) ? 1 : 0;
        if (invertLine) {
                val = 1 - val;
            }
        pattern[i] = val;
       
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
const char* bestMatchType(const uint8_t* input, JsonDocument& doc, uint16_t& segmentTime) {
    static char bestType[16] = "unknown";
    int bestScore = -1;
    JsonArray segments = doc[0]["segments"];
    for (JsonObject segment : segments) {
        const char* type = segment["type"];
        JsonArray patterns = segment["pattern"];
        for (const char* pat : patterns) {
            int score = scoreMatch(input, pat);
            // Serial.print("Pattern: ");
            // for (int i = 0; i < 5; i++) Serial.print(input[i]);
            // Serial.print(" vs ");
            // Serial.print(pat);
            // Serial.print(" | Score: ");
            // Serial.println(score);
            if (score > bestScore) {
                if (segment.containsKey("time")) {
                    segmentTime = segment["time"];
                    segmentTime *= 1000;
                } else {
                    segmentTime = 0;
                }
                bestScore = score;
                strcpy(bestType, type);
            }
//             Serial.print("Pattern: ");
// for (int i = 0; i < 5; i++) Serial.print(pattern[i]);
// Serial.print(" | bestMatchType: "); Serial.println(type);
        }
    }
    // Serial.print("Best type: "); Serial.println(bestType);
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

bool isSegmentGap = false;
// Cập nhật PID và tốc độ từ cấu hình JSON theo loại segment
void setPIDFromType(const char* type, JsonDocument& doc) {
    isSegmentGap = (strcmp(type, "gap") == 0);

    JsonArray segments = doc[0]["segments"];
    for (JsonObject segment : segments) {
        if (!segment.containsKey("type")) continue;
        if (strcmp(segment["type"], type) == 0) {
            if (segment.containsKey("pid")) {
                JsonArray pidArray = segment["pid"];
                M1_speed = pidArray[0].as<int>();
                M2_speed = pidArray[1].as<int>();
                int16_t maxPidOutput = 100;
                float Kp = pidArray[2];
                float Ki = pidArray[3];
                float Kd = pidArray[4];
                PID_init(&pid, Kp, Ki, Kd, -maxPidOutput, maxPidOutput);
            }
            break;
        }
    }
}

enum priorTurnDirection {
    PRIOR_LEFT,
    PRIOR_RIGHT,
    PRIOR_NONE
};

priorTurnDirection priorTurn = PRIOR_NONE;
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
    if (isSegmentGap) {
        // Quẹo phải mạnh nếu thấy line bên phải
        if ((pattern[0] == 0 && pattern[4] == 1) && (priorTurn != PRIOR_LEFT)) {
            priorTurn = PRIOR_RIGHT;
            Serial.println("Prior turn right");
            webSocketServer.broadcastTXT("Prior turn right");
            
            return -2.0;
        }
        // Quẹo trái mạnh nếu thấy line bên trái
        else if (pattern[0] == 1 && pattern[4] == 0 && (priorTurn != PRIOR_RIGHT)) {
            Serial.println("Prior turn left");
            webSocketServer.broadcastTXT("Prior turn left");

            priorTurn = PRIOR_LEFT;
            return 2.0;
        }
    }
    if (count == 0) {
        if(priorTurn == PRIOR_LEFT) {
            Serial.println("Turn LEFT");
            webSocketServer.broadcastTXT("Turn LEFT");

            return 2.0; // Quẹo trái mạnh nếu không thấy line
        } else if (priorTurn == PRIOR_RIGHT) {
            Serial.println("Turn RIGHT");
            webSocketServer.broadcastTXT("Turn RIGHT");

            return -2.0; // Quẹo phải mạnh nếu không thấy line
        }
        return 0;
    }
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

// Xử lý tin nhắn WebSocket
void handleWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_TEXT: {
            Serial.printf("[%u] Received text: %s\n", num, payload);
            handleJsonMessage(String((char*)payload));
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
    // Serial.println("Status packet sent: " + msg);
}
String currentType = "unknown";
String lastDetectedType = "unknown";
int stableCount = 0;
const int stableThreshold = 3;
unsigned long lastTypeChange = 0;
const unsigned long minTypeDuration = 200;

void updateSegmentType(const char* newType, JsonDocument& doc) {
  if (strcmp(newType, lastDetectedType.c_str()) == 0) {
    stableCount++;
  } else {
    stableCount = 1;
    lastDetectedType = newType;
    // Nếu currentType đang là "unknown" và vừa phát hiện type mới, cập nhật luôn
    if (currentType == "unknown" && strcmp(newType, "unknown") != 0) {
      currentType = newType;
      lastTypeChange = millis();
      Serial.print("[Segment] Type changed to: ");
      Serial.println(currentType);
      setPIDFromType(newType, doc);
      PID_reset(&pid);
    }
  }

  if ((millis() - lastTypeChange > minTypeDuration) && (stableCount >= stableThreshold)) {
    if (currentType != newType) {
        currentType = newType;
        lastTypeChange = millis();
        Serial.print("[Segment] Type changed to: ");
        Serial.println(currentType);
        setPIDFromType(newType, doc);
        PID_reset(&pid);

        // Cập nhật timing đúng lúc đổi type
        uint16_t newSegmentTime = 0;
        bestMatchType(lastValidPattern, doc, newSegmentTime); // lấy lại segmentTime cho currentType
        segmentStartTime = millis();
        segmentTime = newSegmentTime;
        lastType = currentType;
        lastSegmentTime = newSegmentTime;
        inSegmentTiming = (segmentTime > 0);
    }
}
}

void handleJsonMessage(const String& msg) {
    StaticJsonDocument<2048> doc;
    DeserializationError error = deserializeJson(doc, msg);
    if (error) {
        Serial.println("JSON parse error");
        return;
    }
    String cmd = doc["cmd"] | "";
    if (cmd == "pidConfig") {
        currentType = "unknown"; // Reset type khi nhận cấu hình mới
        lastType = "unknown";
        lastSegmentTime = 0;
        inSegmentTiming = false;
        segmentStartTime = 0;
        segmentTime = 0;
        String configStr = doc["value"] | "";
        DeserializationError err2 = deserializeJson(pidConfig, configStr);
        if (!err2) {
            saveToFS("/pid_config.json", pidConfig);

            // Ưu tiên cập nhật PID/tốc độ cho type đầu tiên trong cấu hình mới (nếu muốn robot chạy luôn)
            JsonArray segments = pidConfig[0]["segments"];
            if (segments.size() > 0) {
                const char* firstType = segments[0]["type"];
                setPIDFromType(firstType, pidConfig);
                currentType = firstType;
                Serial.print("[Config] Set PID for type: ");
                Serial.println(firstType);
            } else {
                setPIDFromType("unknown", pidConfig);
            }

            Serial.println("OK");
        } else {
            Serial.println("ERR");
        }
    }
    else if (cmd == "loadFile") {
    String tenfile = doc["value"]["tenfile"] | "";
    if (tenfile.length() > 0) {
        if (loadFromFS(("/" + tenfile).c_str(), pidConfig)) {
            setPIDFromType(currentType.c_str(), pidConfig);
            Serial.print("PID Config: ");
            serializeJson(pidConfig, Serial);
            Serial.println();
            Serial.println("OK");
        } else {
            Serial.println("ERR");
        }
    }
}
    else if (cmd == "saveConfig") {
        String tenfile = doc["value"] | "";
        File f = LittleFS.open(("/" + tenfile).c_str(), "w");
        if (f) {
            serializeJson(pidConfig, f);
            Serial.println();
            f.close();
            Serial.println("OK");
            Serial.println("Saved to " + tenfile);
        } else {
            Serial.println("ERR");
        }
    }
    else if (cmd == "WifiConnect") {
    String ssid = doc["value"]["ssid"] | "";
    String pass = doc["value"]["pass"] | "";
    if (ssid.length() > 0) {
        Serial.println("WiFi reconnecting to: " + ssid);
        WiFi.disconnect();
        WiFi.begin(ssid.c_str(), pass.c_str());
        unsigned long start = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
            delay(500);
            Serial.print(".");
        }
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\nWiFi connected: " + WiFi.localIP().toString());
        } else {
            Serial.println("\nWiFi connect failed!");
        }
    }
}
}


void setup() {
    Serial.begin(115200);
    LittleFS.begin();
    if (!loadFromFS("/pid_config.json", pidConfig)) {
        deserializeJson(pidConfig, defaultConfig);
    }

    // Kết nối WiFi mặc định (nếu muốn)
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
        delay(500);
        Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected");
        Serial.println(WiFi.localIP());
        server.begin();
        webSocketServer.begin();
        webSocketServer.onEvent(handleWebSocketEvent);
    } else {
        Serial.println("\nWiFi connect failed!");
        // Không khởi động server khi chưa có WiFi
    }

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

void handleSerialCommand() {
    static String inputBuffer = "";
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (inputBuffer.length() > 0) {
                Serial.println("Serial received: " + inputBuffer);
                handleJsonMessage(inputBuffer);
                inputBuffer = "";
            }
        } else {
            inputBuffer += c;
        }
    }
}


void handleSegmentTiming(const uint8_t* pattern, JsonDocument& doc) {
// filepath: d:\hoc tap_2\lineFollowComplex\src\main.cpp
    uint16_t newSegmentTime = 0;
    const char* type = bestMatchType(pattern, doc, newSegmentTime);

    bool isCurrentUnknown = (currentType == "unknown");

    // Nếu hết thời gian giữ segment thì cho phép dò lại type
    if (segmentTime > 0 && millis() - segmentStartTime >= segmentTime) {
        inSegmentTiming = false;
    }

    // Nếu vẫn đang trong thời gian giữ và không phải "unknown" thì bỏ qua
    if (inSegmentTiming && !isCurrentUnknown) {
        Serial.println("Still in segment timing, skipping update");
        return;
    }

    // Luôn gọi updateSegmentType để debounce hoạt động đúng
    updateSegmentType(type, doc);

}

void loop() {
    handleSerialCommand();
    webSocketServer.loop();

    static unsigned long lastControlTime = millis();

    if (millis() - lastControlTime >= fregControl) {
        lastControlTime = millis();
        getLinePattern(pattern);

        // Ưu tiên cập nhật nếu vừa nhận cấu hình mới
        if (pidConfigJustUpdated) {
            setPIDFromType(currentType.c_str(), pidConfig);
            pidConfigJustUpdated = false;
        }

        // Gọi hàm quản lý segment timing
        handleSegmentTiming(pattern, pidConfig);

        float error = computeLineError(pattern);
        float correction = PID_compute(&pid, 0.0, error);
        driveMotors(M1_speed, M2_speed, correction);
    }

    static unsigned long lastStatusSend = 0;
    // if (millis() - lastStatusSend >= 200) {
    //     lastStatusSend = millis();
        sendStatusPacket(pattern, currentType.c_str(), computeLineError(pattern), M1_speed, M2_speed);
    // }
}