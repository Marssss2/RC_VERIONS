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
// -------------------------------------------------------
#define LEFT_IN1   25
#define LEFT_IN2   26
#define LEFT_EN    32
#define RIGHT_IN3  27
#define RIGHT_IN4  14
#define RIGHT_EN   33
#define MOTOR_SPEED     180  // Normal speed
#define MOTOR_SLOW      80   // Correction speed (para sa ~19mm electrical tape)

// -------------------------------------------------------
// IR SENSOR PINS (TCRT5000)
// LOW  = detects BLACK line
// HIGH = detects white surface
// -------------------------------------------------------
#define IR_LEFT   34
#define IR_CENTER 35
#define IR_RIGHT  36

// -------------------------------------------------------
// MODE
// -------------------------------------------------------
bool lineTrackingMode = false;

// -------------------------------------------------------
// MOTOR HELPERS
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

void stopMotors() {
  setLeftMotor(0, 0);
  setRightMotor(0, 0);
}

void moveForward() {
  setLeftMotor(1, MOTOR_SPEED);
  setRightMotor(1, MOTOR_SPEED);
}

void moveBackward() {
  setLeftMotor(-1, MOTOR_SPEED);
  setRightMotor(-1, MOTOR_SPEED);
}

void turnLeft() {
  setLeftMotor(0, 0);
  setRightMotor(1, MOTOR_SPEED);
}

void turnRight() {
  setLeftMotor(1, MOTOR_SPEED);
  setRightMotor(0, 0);
}

void softLeft() {
  setLeftMotor(1, MOTOR_SLOW);
  setRightMotor(1, MOTOR_SPEED);
}

void softRight() {
  setLeftMotor(1, MOTOR_SPEED);
  setRightMotor(1, MOTOR_SLOW);
}

// -------------------------------------------------------
// LINE TRACKING LOGIC (3 sensors)
//
//  L    C    R   | Action
// ----+----+---- |----------------------------
// HIGH LOW  HIGH | Forward (center on line = perfect)
// HIGH LOW  LOW  | Soft Right
// LOW  LOW  HIGH | Soft Left
// HIGH HIGH LOW  | Hard Right pivot
// LOW  HIGH HIGH | Hard Left pivot
// HIGH HIGH HIGH | Forward (gap / lost briefly)
// LOW  LOW  LOW  | Stop (end of line)
// LOW  HIGH LOW  | Forward (intersection)
// -------------------------------------------------------
void lineTrack() {
  int L = digitalRead(IR_LEFT);
  int C = digitalRead(IR_CENTER);
  int R = digitalRead(IR_RIGHT);

  if (C == LOW && L == HIGH && R == HIGH) {
    moveForward();
  }
  else if (C == LOW && L == HIGH && R == LOW) {
    softRight();
  }
  else if (C == LOW && L == LOW && R == HIGH) {
    softLeft();
  }
  else if (C == HIGH && L == HIGH && R == LOW) {
    turnRight();
  }
  else if (C == HIGH && L == LOW && R == HIGH) {
    turnLeft();
  }
  else if (C == HIGH && L == HIGH && R == HIGH) {
    moveForward();
  }
  else if (L == LOW && C == LOW && R == LOW) {
    stopMotors();
    lineTrackingMode = false;
    WebSerial.println("[TRACK] End of line detected. Stopping.");
  }
  else if (L == LOW && C == HIGH && R == LOW) {
    moveForward();
  }
}

// -------------------------------------------------------
// ACCESS POINT
// -------------------------------------------------------
void startAP() {
  WiFi.softAP(ssid.c_str(), password.c_str());
  Serial.println("\n--- Access Point Started ---");
  Serial.print("SSID: "); Serial.println(ssid);
  Serial.print("IP:   "); Serial.println(WiFi.softAPIP());
  dnsServer.start(53, "*", WiFi.softAPIP());
}

// -------------------------------------------------------
// WEBSERIAL COMMANDS
//
//  LINE TRACKING:
//    track on      = Start line following
//    track off     = Stop line following
//    sensor test   = Read all 3 sensor values
//
//  MANUAL MOVEMENT:
//    f / forward   = Move forward (2 sec)
//    b / backward  = Move backward (2 sec)
//    l / left      = Turn left (2 sec)
//    r / right     = Turn right (2 sec)
//    s / stop      = Stop
//
//  DIAGNOSTICS:
//    test left     = Test LEFT motor only
//    test right    = Test RIGHT motor only
//    test all      = Test both motors
//
//  SETTINGS:
//    set ssid <name>  = Change AP SSID
//    set pass <pass>  = Change AP password (min 8 chars)
// -------------------------------------------------------
void recvMsg(uint8_t *data, size_t len) {
  String message = "";
  for (int i = 0; i < len; i++) { message += char(data[i]); }
  message.trim();
  message.toLowerCase();

  if (message == "track on") {
    lineTrackingMode = true;
    WebSerial.println("[OK] LINE TRACKING ON");
    WebSerial.println("     Robot will follow the black line.");
    WebSerial.println("     Type 'track off' or 's' to stop.");
  }
  else if (message == "track off") {
    lineTrackingMode = false;
    stopMotors();
    WebSerial.println("[OK] LINE TRACKING OFF");
  }
  else if (message == "sensor test") {
    int L = digitalRead(IR_LEFT);
    int C = digitalRead(IR_CENTER);
    int R = digitalRead(IR_RIGHT);
    WebSerial.println("=== SENSOR READINGS ===");
    WebSerial.print("  LEFT   (GPIO34): "); WebSerial.println(L == LOW ? "BLACK LINE" : "white");
    WebSerial.print("  CENTER (GPIO35): "); WebSerial.println(C == LOW ? "BLACK LINE" : "white");
    WebSerial.print("  RIGHT  (GPIO36): "); WebSerial.println(R == LOW ? "BLACK LINE" : "white");
    WebSerial.println("  (LOW = black line detected)");
  }
  else if (message == "forward" || message == "f") {
    lineTrackingMode = false;
    moveForward();
    WebSerial.println("[OK] FORWARD");
    delay(2000);
    stopMotors();
  }
  else if (message == "backward" || message == "b") {
    lineTrackingMode = false;
    moveBackward();
    WebSerial.println("[OK] BACKWARD");
    delay(2000);
    stopMotors();
  }
  else if (message == "left" || message == "l") {
    lineTrackingMode = false;
    turnLeft();
    WebSerial.println("[OK] LEFT");
    delay(2000);
    stopMotors();
  }
  else if (message == "right" || message == "r") {
    lineTrackingMode = false;
    turnRight();
    WebSerial.println("[OK] RIGHT");
    delay(2000);
    stopMotors();
  }
  else if (message == "stop" || message == "s") {
    lineTrackingMode = false;
    stopMotors();
    WebSerial.println("[OK] STOPPED");
  }
  else if (message == "test left") {
    lineTrackingMode = false;
    WebSerial.println("[TEST] LEFT motor only, 2 sec...");
    setLeftMotor(1, MOTOR_SPEED);
    setRightMotor(0, 0);
    delay(2000);
    stopMotors();
    WebSerial.println("[TEST] Done. Did LEFT wheel spin forward?");
  }
  else if (message == "test right") {
    lineTrackingMode = false;
    WebSerial.println("[TEST] RIGHT motor only, 2 sec...");
    setLeftMotor(0, 0);
    setRightMotor(1, MOTOR_SPEED);
    delay(2000);
    stopMotors();
    WebSerial.println("[TEST] Done. Did RIGHT wheel spin forward?");
  }
  else if (message == "test all") {
    lineTrackingMode = false;
    WebSerial.println("[TEST] Both motors forward, 2 sec...");
    moveForward();
    delay(2000);
    stopMotors();
    WebSerial.println("[TEST] Done. Did both wheels go forward?");
  }
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
  else {
    WebSerial.println("[?] Unknown command.");
    WebSerial.println("    f b l r s | track on/off | sensor test | test left/right/all");
  }
}

// -------------------------------------------------------
// SETUP
// -------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(LEFT_IN1,  OUTPUT);
  pinMode(LEFT_IN2,  OUTPUT);
  pinMode(LEFT_EN,   OUTPUT);
  pinMode(RIGHT_IN3, OUTPUT);
  pinMode(RIGHT_IN4, OUTPUT);
  pinMode(RIGHT_EN,  OUTPUT);
  stopMotors();

  pinMode(IR_LEFT,   INPUT);
  pinMode(IR_CENTER, INPUT);
  pinMode(IR_RIGHT,  INPUT);

  preferences.begin("wifi-config", false);
  ssid     = preferences.getString("ssid", "ESP32_RC");
  password = preferences.getString("pass", "12345678");

  startAP();

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

  if (lineTrackingMode) {
    lineTrack();
    delay(10);
  }
}