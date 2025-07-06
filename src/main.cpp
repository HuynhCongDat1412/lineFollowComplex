// main.cpp - ESP32 gửi web interface và giao tiếp WebSocket
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include "pid.h"

// ==== WIFI CONFIG ====
const char* ssid = "DAT PHUONG";
const char* password = "19201974";

// ==== WEB SERVER ====
WebServer server(80);
WebSocketsServer webSocketServer = WebSocketsServer(81);
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
// ==== MOTOR CONTROL (TB6612) ====
const int PWMA = 18, AIN1 = 19, AIN2 = 22;
const int BIN1 = 16, BIN2 = 21, PWMB = 4;
// ==== SENSOR PINS ====
// 5 mắt cảm biến: 35, 32, 33, 25, 26
const uint8_t sensors[5] = {35, 32, 33, 25, 26};
int8_t sensorValues[5];


typedef enum {
  LEFT,
  RIGHT,
} MOTOR;

int8_t motor1_speed = 0;
int8_t motor2_speed = 0;
PIDController pid;

// ==== HTML PAGE ====
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head><meta charset="UTF-8"><title>ESP32 Line Control</title></head>
<body>
  <h2>Motor Control</h2>
  <textarea id="data" rows="4" cols="30">{"motor1": 60, "motor2": 0}</textarea><br>
  <button onclick="sendData()">Send</button>
  <h3>Sensor Values:</h3>
  <div id="sensors"></div>

<script>
let socket = new WebSocket("ws://" + location.hostname + ":81/");

socket.onmessage = function(event) {
  let data = JSON.parse(event.data);
  if (data.sensor) {
    document.getElementById("sensors").innerText = "[" + data.sensor.join(", ") + "]";
  }
};

function sendData() {
  let txt = document.getElementById("data").value;
  socket.send(txt);
}
</script>
</body>
</html>
)rawliteral";

void handle_msg(StaticJsonDocument<200> &doc);
void handle_motor(StaticJsonDocument<200> doc);
void handle_reset();
void handle_sensor();
void motor_set(int pwmChannel, int in1, int in2, int speed);
// ==== SETUP FUNCTIONS ====

void setup_WS() {
  IPAddress local_IP(192, 168, 1, 65);
  IPAddress gateway(192, 168, 1, 1);
  IPAddress subnet(255, 255, 255, 0);
  IPAddress primaryDNS(8, 8, 8, 8);   // Optional
  IPAddress secondaryDNS(8, 8, 4, 4); // Optional

  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }
  // Setup WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\nWiFi connected: " + WiFi.localIP().toString());

  // Start HTTP server and webSocketServer
  server.on("/", []() {
    server.send_P(200, "text/html", index_html);
  });
  server.begin();
  webSocketServer.begin();
  webSocketServer.onEvent(webSocketEvent);
}

void setup_pin(){
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);  
  for (int i = 0; i < 5; i++) {
    pinMode(sensors[i], INPUT);
  }
  ledcSetup(0, 1000, 8); ledcAttachPin(PWMA, 0);
  ledcSetup(1, 1000, 8); ledcAttachPin(PWMB, 1);

  PID_init(&pid, 25.0, 0.0, 8.0, -100, 100); // Kp, Ki, Kd, min, max
}
// ==== SETUP ====

void setup() {
  Serial.begin(115200);
  setup_WS();
  setup_pin(); // Cấu hình chân GPIO  
  // Cấu hình IP tĩnh
  
}

// ==== LOOP ====
void loop() {
  webSocketServer.loop();
  server.handleClient();

}

// ==== MOTOR DRIVE FUNCTION ====
void motor_set(int pwmChannel, int in1, int in2, int speed) {
  if (speed > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
    ledcWrite(pwmChannel, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
    ledcWrite(pwmChannel, -speed);
  } else {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
    ledcWrite(pwmChannel, 0);
  }
}


// ==== HANDLE WEBSOCKET EVENTS ====
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT) {
    Serial.printf("Message from client %u: %s\n", num, payload);
    StaticJsonDocument<200> doc;
    DeserializationError err = deserializeJson(doc, payload);
    if (!err && doc.containsKey("cmd")) {
      handle_msg(doc);
    }
  }
}

void handle_msg(StaticJsonDocument<200> &doc) {
  if (doc["cmd"] == "motor") {
    handle_motor(doc);
  }
  else if (doc["cmd"] == "reset") {
    handle_reset();
  }
  else {
    Serial.println("Unknown command");
  }
}

void handle_motor (StaticJsonDocument<200> doc) {
  if (doc.containsKey("motor1")) {
    motor1_speed = doc["motor1"] | 0; // Default to 0 if not specified
    motor_set(0, AIN1, AIN2, map(motor1_speed, 0, 100, 0, 255));
  }
  if (doc.containsKey("motor2")) {
    motor2_speed = doc["motor2"] | 0;
    motor_set(1, BIN1, BIN2, map(motor2_speed, 0, 100, 0, 255));
  }
}

void handle_reset() {
  Serial.println("Resetting ESP32...");
  ESP.restart();
}

void handle_sensor() {
  StaticJsonDocument<200> doc;
  JsonArray arr = doc.createNestedArray("sensor");
  for (int i = 0; i < 5; i++) {
    arr.add(digitalRead(sensors[i]));
  }
  String msg;
  serializeJson(doc, msg);
  webSocketServer.broadcastTXT(msg);
}



void handle_pid (uint8_t baseSpeed ) {
  int8_t linePosition = readLineSensor(); 
  const int8_t targetPosition = 0; // Giả sử đường thẳng ở giữa
  float correction = PID_compute(&pid, targetPosition, linePosition);
  
  int8_t leftSpeed = baseSpeed + correction;
  int8_t rightSpeed = baseSpeed - correction;
  leftSpeed = constrain(leftSpeed, 0, 100);
  rightSpeed = constrain(rightSpeed, 0, 100);
  motor_control(leftSpeed, LEFT);
  motor_control(rightSpeed, RIGHT);

}

int readLineSensor() {
  int position = 0;
  for (int i = 0; i < 5; i++) {
    sensorValues[i] = digitalRead(sensors[i]);
    if (sensorValues[i] == HIGH) {
      position += (i - 2); // Gán trọng số -2, -1, 0, 1, 2
    }
}
  return position;
}


// 0->100%
void motor_control(int baseSpeed,MOTOR motor) {
  if (motor == LEFT) {
    motor_set(0, AIN1, AIN2, map(baseSpeed, 0, 100, 0, 255)); // Motor trái chạy
  } else if (motor == RIGHT) {
    motor_set(1, BIN1, BIN2, map(baseSpeed, 0, 100, 0, 255)); // Motor phải chạy
  }
}