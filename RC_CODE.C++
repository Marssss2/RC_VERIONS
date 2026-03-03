#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>
#include <Preferences.h>
#include <DNSServer.h>

AsyncWebServer server(80);
Preferences preferences;
DNSServer dnsServer;

String ssid;
String password;


#define MOTOR_A_IN1  25
#define MOTOR_A_IN2  26
#define MOTOR_A_EN   32  
#define MOTOR_B_IN3  27
#define MOTOR_B_IN4  14
#define MOTOR_B_EN   33  
#define MOTOR_SPEED  255  


void stopMotors() {
  digitalWrite(MOTOR_A_IN1, LOW); digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN3, LOW); digitalWrite(MOTOR_B_IN4, LOW);
  analogWrite(MOTOR_A_EN, 0); analogWrite(MOTOR_B_EN, 0);
}

void moveForward() {
  digitalWrite(MOTOR_A_IN1, HIGH); digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN3, HIGH); digitalWrite(MOTOR_B_IN4, LOW);
  analogWrite(MOTOR_A_EN, MOTOR_SPEED); analogWrite(MOTOR_B_EN, MOTOR_SPEED);
}

void moveBackward() {
  digitalWrite(MOTOR_A_IN1, LOW); digitalWrite(MOTOR_A_IN2, HIGH);
  digitalWrite(MOTOR_B_IN3, LOW); digitalWrite(MOTOR_B_IN4, HIGH);
  analogWrite(MOTOR_A_EN, MOTOR_SPEED); analogWrite(MOTOR_B_EN, MOTOR_SPEED);
}

void turnLeft() {
  digitalWrite(MOTOR_A_IN1, LOW);  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN3, HIGH); digitalWrite(MOTOR_B_IN4, LOW);
  analogWrite(MOTOR_A_EN, 0); analogWrite(MOTOR_B_EN, MOTOR_SPEED);
}

void turnRight() {
  digitalWrite(MOTOR_A_IN1, HIGH); digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN3, LOW);  digitalWrite(MOTOR_B_IN4, LOW);
  analogWrite(MOTOR_A_EN, MOTOR_SPEED); analogWrite(MOTOR_B_EN, 0);
}

void startAP() {
  WiFi.softAP(ssid.c_str(), password.c_str());
  Serial.println("\n--- Access Point Started ---");
  Serial.print("SSID: "); Serial.println(ssid);
  Serial.print("PASS: "); Serial.println(password);
  Serial.print("IP Address: "); Serial.println(WiFi.softAPIP());
  dnsServer.start(53, "*", WiFi.softAPIP());
}

void recvMsg(uint8_t *data, size_t len) {
  String message = "";
  for (int i = 0; i < len; i++) { message += char(data[i]); }
  message.trim();
  message.toLowerCase();

  if (message == "forward" || message == "f") {
    moveForward(); WebSerial.println("FORWARD"); delay(2000); stopMotors();
  }
  else if (message == "backward" || message == "b") {
    moveBackward(); WebSerial.println("BACKWARD"); delay(2000); stopMotors();
  }
  else if (message == "left" || message == "l") {
    turnLeft(); WebSerial.println("LEFT"); delay(2000); stopMotors();
  }
  else if (message == "right" || message == "r") {
    turnRight(); WebSerial.println("RIGHT"); delay(2000); stopMotors();
  }
  else if (message == "stop" || message == "s") {
    stopMotors(); WebSerial.println("STOPPED");
  }
  else if (message.startsWith("set ssid ")) {
    ssid = message.substring(9);
    preferences.putString("ssid", ssid);
    WebSerial.println("SSID Updated. Restarting...");
    delay(1000); ESP.restart();
  }
  else if (message.startsWith("set pass ")) {
    password = message.substring(9);
    if (password.length() < 8) { WebSerial.println("Error: Min 8 chars"); return; }
    preferences.putString("pass", password);
    WebSerial.println("Password Updated. Restarting...");
    delay(1000); ESP.restart();
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_A_IN1, OUTPUT); pinMode(MOTOR_A_IN2, OUTPUT); pinMode(MOTOR_A_EN, OUTPUT);
  pinMode(MOTOR_B_IN3, OUTPUT); pinMode(MOTOR_B_IN4, OUTPUT); pinMode(MOTOR_B_EN, OUTPUT);
  stopMotors();

  preferences.begin("wifi-config", false);


  Serial.println("Clearing old WiFi settings for recovery...");
  preferences.clear(); 
  // ----------------------

  ssid     = preferences.getString("ssid", "ESP32_RC_NEW");
  password = preferences.getString("pass", "12345678");

  startAP();

  server.onNotFound([](AsyncWebServerRequest *request) { request->redirect("/webserial"); });
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) { request->redirect("/webserial"); });

  WebSerial.begin(&server);
  WebSerial.onMessage(recvMsg);
  server.begin();
}

void loop() {
  dnsServer.processNextRequest();
}