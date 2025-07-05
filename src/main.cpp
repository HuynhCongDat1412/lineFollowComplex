// main.cpp - ESP32 gửi web interface và giao tiếp WebSocket
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

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


// ==== MOTOR SPEED VARIABLES ====
int motor1_speed = 0;
int motor2_speed = 0;

// ==== TIMER ====
unsigned long lastSend = 0;
const unsigned long sendInterval = 1000; // 1 giây

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

// ==== SETUP ====
void setup() {
  Serial.begin(115200);

  // Setup motor pins
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);  
  // Setup sensor pins
  for (int i = 0; i < 5; i++) {
    pinMode(sensors[i], INPUT);
  }
  // Cấu hình IP tĩnh
  IPAddress local_IP(192, 168, 1, 225);
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

  // Setup PWM channels
  ledcSetup(0, 1000, 8); ledcAttachPin(PWMA, 0);
  ledcSetup(1, 1000, 8); ledcAttachPin(PWMB, 1);
}

// ==== MOTOR DRIVE FUNCTION ====
void setMotor(int pwmChannel, int in1, int in2, int speed) {
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

// ==== LOOP ====
void loop() {
  webSocketServer.loop();
  server.handleClient();

  if (millis() - lastSend >= sendInterval) {
    lastSend = millis();
    for (int i = 0; i < 5; i++) {
      sensorValues[i] = digitalRead(sensors[i]);
    }
    StaticJsonDocument<200> doc;
    JsonArray arr = doc.createNestedArray("sensor");
    for (int i = 0; i < 5; i++) arr.add(sensorValues[i]);

    String msg;
    serializeJson(doc, msg);
    webSocketServer.broadcastTXT(msg);
  }

  setMotor(0, AIN1, AIN2, map(motor1_speed, 0, 100, 0, 255));
  setMotor(1, BIN1, BIN2, map(motor2_speed, 0, 100, 0, 255));
}

// ==== HANDLE WEBSOCKET EVENTS ====
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT) {
    StaticJsonDocument<200> doc;
    DeserializationError err = deserializeJson(doc, payload);
    if (!err) {
      if (doc.containsKey("motor1")) motor1_speed = doc["motor1"];
      if (doc.containsKey("motor2")) motor2_speed = doc["motor2"];
    }
  }
}
