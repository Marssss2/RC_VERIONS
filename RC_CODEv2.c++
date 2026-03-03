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

// -------------------------------------------------------
// MOTOR PINS
// L298N Wiring:
//   Motor A (OUT1/OUT2) = LEFT wheel
//   Motor B (OUT3/OUT4) = RIGHT wheel
//
// ESP32 -> L298N:
//   Pin 25 -> IN1   (Left motor direction)
//   Pin 26 -> IN2   (Left motor direction)
//   Pin 32 -> ENA   (Left motor speed - PWM)
//   Pin 27 -> IN3   (Right motor direction)
//   Pin 14 -> IN4   (Right motor direction)
//   Pin 33 -> ENB   (Right motor speed - PWM)
// -------------------------------------------------------
#define LEFT_IN1   25
#define LEFT_IN2   26
#define LEFT_EN    32

#define RIGHT_IN3  27
#define RIGHT_IN4  14
#define RIGHT_EN   33

#define MOTOR_SPEED 200  // 0-255, hindi full para hindi mabilis agad

// -------------------------------------------------------
// HELPER: Control individual motors
// direction: 1 = forward, -1 = backward, 0 = stop
// -------------------------------------------------------
void setLeftMotor(int direction, int speed) {
  if (direction == 1) {
    digitalWrite(LEFT_IN1, HIGH);
    digitalWrite(LEFT_IN2, LOW);
  } else if (direction == -1) {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, HIGH);
  } else {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, LOW);
  }
  analogWrite(LEFT_EN, (direction == 0) ? 0 : speed);
}

void setRightMotor(int direction, int speed) {
  if (direction == 1) {
    digitalWrite(RIGHT_IN3, HIGH);
    digitalWrite(RIGHT_IN4, LOW);
  } else if (direction == -1) {
    digitalWrite(RIGHT_IN3, LOW);
    digitalWrite(RIGHT_IN4, HIGH);
  } else {
    digitalWrite(RIGHT_IN3, LOW);
    digitalWrite(RIGHT_IN4, LOW);
  }
  analogWrite(RIGHT_EN, (direction == 0) ? 0 : speed);
}

// -------------------------------------------------------
// MOVEMENT FUNCTIONS
// -------------------------------------------------------
void stopMotors() {
  setLeftMotor(0, 0);
  setRightMotor(0, 0);
}

void moveForward() {
  // Both motors forward
  setLeftMotor(1, MOTOR_SPEED);
  setRightMotor(1, MOTOR_SPEED);
}

void moveBackward() {
  // Both motors backward
  setLeftMotor(-1, MOTOR_SPEED);
  setRightMotor(-1, MOTOR_SPEED);
}

void turnLeft() {
  // Left motor stops, Right motor forward = pivots left
  setLeftMotor(0, 0);
  setRightMotor(1, MOTOR_SPEED);
}

void turnRight() {
  // Right motor stops, Left motor forward = pivots right
  setLeftMotor(1, MOTOR_SPEED);
  setRightMotor(0, 0);
}

// -------------------------------------------------------
// ACCESS POINT SETUP
// -------------------------------------------------------
void startAP() {
  WiFi.softAP(ssid.c_str(), password.c_str());
  Serial.println("\n--- Access Point Started ---");
  Serial.print("SSID: "); Serial.println(ssid);
  Serial.print("PASS: "); Serial.println(password);
  Serial.print("IP:   "); Serial.println(WiFi.softAPIP());
  dnsServer.start(53, "*", WiFi.softAPIP());
}

// -------------------------------------------------------
// WEBSERIAL COMMAND HANDLER
//
// AVAILABLE COMMANDS:
//   f / forward   = Move forward
//   b / backward  = Move backward
//   l / left      = Turn left
//   r / right     = Turn right
//   s / stop      = Stop
//
// DIAGNOSTIC COMMANDS (para i-test bawat motor):
//   test left     = Runs LEFT motor only (forward 2 sec)
//   test right    = Runs RIGHT motor only (forward 2 sec)
//   test all      = Runs both motors forward (2 sec)
//
// SETTINGS:
//   set ssid <name>   = Change WiFi AP name
//   set pass <pass>   = Change WiFi password (min 8 chars)
// -------------------------------------------------------
void recvMsg(uint8_t *data, size_t len) {
  String message = "";
  for (int i = 0; i < len; i++) { message += char(data[i]); }
  message.trim();
  message.toLowerCase();

  // --- Movement Commands ---
  if (message == "forward" || message == "f") {
    moveForward();
    WebSerial.println("[OK] FORWARD");
    delay(2000);
    stopMotors();
  }
  else if (message == "backward" || message == "b") {
    moveBackward();
    WebSerial.println("[OK] BACKWARD");
    delay(2000);
    stopMotors();
  }
  else if (message == "left" || message == "l") {
    turnLeft();
    WebSerial.println("[OK] LEFT");
    delay(2000);
    stopMotors();
  }
  else if (message == "right" || message == "r") {
    turnRight();
    WebSerial.println("[OK] RIGHT");
    delay(2000);
    stopMotors();
  }
  else if (message == "stop" || message == "s") {
    stopMotors();
    WebSerial.println("[OK] STOPPED");
  }

  // --- Diagnostic Commands ---
  else if (message == "test left") {
    WebSerial.println("[TEST] Running LEFT motor only for 2 sec...");
    setLeftMotor(1, MOTOR_SPEED);
    setRightMotor(0, 0);
    delay(2000);
    stopMotors();
    WebSerial.println("[TEST] Done. Did the LEFT wheel spin forward?");
    WebSerial.println("       If RIGHT wheel moved instead, swap your OUT1/OUT2 and OUT3/OUT4 wires.");
  }
  else if (message == "test right") {
    WebSerial.println("[TEST] Running RIGHT motor only for 2 sec...");
    setLeftMotor(0, 0);
    setRightMotor(1, MOTOR_SPEED);
    delay(2000);
    stopMotors();
    WebSerial.println("[TEST] Done. Did the RIGHT wheel spin forward?");
    WebSerial.println("       If LEFT wheel moved instead, swap your OUT1/OUT2 and OUT3/OUT4 wires.");
  }
  else if (message == "test all") {
    WebSerial.println("[TEST] Running BOTH motors forward for 2 sec...");
    moveForward();
    delay(2000);
    stopMotors();
    WebSerial.println("[TEST] Done. Did both wheels spin in the same forward direction?");
    WebSerial.println("       If one wheel spins backward, reverse that motor's two wires on L298N output.");
  }

  // --- Settings ---
  else if (message.startsWith("set ssid ")) {
    ssid = message.substring(9);
    preferences.putString("ssid", ssid);
    WebSerial.println("[OK] SSID updated. Restarting...");
    delay(1000);
    ESP.restart();
  }
  else if (message.startsWith("set pass ")) {
    password = message.substring(9);
    if (password.length() < 8) {
      WebSerial.println("[ERROR] Password must be at least 8 characters.");
      return;
    }
    preferences.putString("pass", password);
    WebSerial.println("[OK] Password updated. Restarting...");
    delay(1000);
    ESP.restart();
  }

  // --- Unknown Command ---
  else {
    WebSerial.println("[?] Unknown command. Try: f, b, l, r, s, test left, test right, test all");
  }
}

// -------------------------------------------------------
// SETUP
// -------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // Motor pin setup
  pinMode(LEFT_IN1,  OUTPUT);
  pinMode(LEFT_IN2,  OUTPUT);
  pinMode(LEFT_EN,   OUTPUT);
  pinMode(RIGHT_IN3, OUTPUT);
  pinMode(RIGHT_IN4, OUTPUT);
  pinMode(RIGHT_EN,  OUTPUT);
  stopMotors(); // Safety: motors off on boot

  // Load saved WiFi credentials
  // Default: ESP32_RC / 12345678 (only used on first boot or if never changed)
  preferences.begin("wifi-config", false);
  ssid     = preferences.getString("ssid", "ESP32_RC");
  password = preferences.getString("pass", "12345678");
  // NOTE: Credentials are saved permanently in flash memory.
  // To change: type "set ssid <name>" or "set pass <password>" in WebSerial.

  startAP();

  // Redirect all URLs to WebSerial
  server.onNotFound([](AsyncWebServerRequest *request) {
    request->redirect("/webserial");
  });
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->redirect("/webserial");
  });

  WebSerial.begin(&server);
  WebSerial.onMessage(recvMsg);
  server.begin();

  Serial.println("Ready! Connect to WiFi then go to http://192.168.4.1/webserial");
}

// -------------------------------------------------------
// LOOP
// -------------------------------------------------------
void loop() {
  dnsServer.processNextRequest();
}
