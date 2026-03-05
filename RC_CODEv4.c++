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

// -------------------------------------------------------
// SPEED SETTINGS
// I-tune ito kung masyadong mabilis o mabagal
// -------------------------------------------------------
#define BASE_SPEED      200  // Normal straight speed
#define TURN_SPEED      220  // Outer wheel speed during correction
#define INNER_SPEED     60   // Inner wheel speed during correction
#define HARD_TURN_SPEED 230  // Hard turn outer wheel
#define HARD_INNER      0    // Hard turn inner wheel (full pivot)

// -------------------------------------------------------
// IR SENSOR PINS (TCRT5000)
// LOW  = BLACK line detected
// HIGH = white surface
// -------------------------------------------------------
#define IR_LEFT   34
#define IR_CENTER 35
#define IR_RIGHT  36

// -------------------------------------------------------
// MODE & LOST LINE HANDLING
// -------------------------------------------------------
bool lineTrackingMode = false;
int  lastError        = 0;   // -1 = was turning left, 1 = was turning right
unsigned long lostTime = 0;  // When did we lose the line
#define LOST_TIMEOUT 1500    // ms before giving up (stop motor)

// -------------------------------------------------------
// MOTOR HELPERS
// -------------------------------------------------------
void setLeftMotor(int direction, int speed) {
  speed = constrain(speed, 0, 255);
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
  speed = constrain(speed, 0, 255);
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

// -------------------------------------------------------
// LINE TRACKING LOGIC
//
// Weighted position system:
//   Sensor value:  LEFT  CENTER  RIGHT
//   On line (LOW):   -1     0      1
//
// Position = weighted average ng lahat ng sensors
//
// Position | Meaning          | Action
// ---------+------------------+----------------------
//    0      | Perfect center   | Full speed forward
//   -1      | Slightly left    | Soft right correction
//    1      | Slightly right   | Soft left correction
//   -2      | Far left         | Hard right pivot
//    2      | Far right        | Hard left pivot
//  none     | Lost line        | Search based on lastError
// -------------------------------------------------------
void lineTrack() {
  int L = digitalRead(IR_LEFT);    // LOW = on line
  int C = digitalRead(IR_CENTER);
  int R = digitalRead(IR_RIGHT);

  // Convert: LOW (line) = 1, HIGH (white) = 0
  int sL = (L == LOW) ? 1 : 0;
  int sC = (C == LOW) ? 1 : 0;
  int sR = (R == LOW) ? 1 : 0;

  int total = sL + sC + sR;

  // --- END OF LINE: all 3 on black ---
  if (sL == 1 && sC == 1 && sR == 1) {
    stopMotors();
    lineTrackingMode = false;
    WebSerial.println("[TRACK] End of line! Finished.");
    return;
  }

  // --- LOST LINE: no sensor on black ---
  if (total == 0) {
    if (lostTime == 0) lostTime = millis(); // Start lost timer

    if (millis() - lostTime > LOST_TIMEOUT) {
      stopMotors();
      lineTrackingMode = false;
      WebSerial.println("[TRACK] Line lost! Stopping.");
      return;
    }

    // Search: pivot toward last known direction
    if (lastError >= 0) {
      // Last seen line on right = pivot right
      setLeftMotor(1, HARD_TURN_SPEED);
      setRightMotor(1, HARD_INNER);
    } else {
      // Last seen line on left = pivot left
      setLeftMotor(1, HARD_INNER);
      setRightMotor(1, HARD_TURN_SPEED);
    }
    return;
  }

  // Line found — reset lost timer
  lostTime = 0;

  // --- COMPUTE WEIGHTED POSITION ---
  // Position: -1 = left, 0 = center, 1 = right
  // Weighted average: (sL*-1 + sC*0 + sR*1) / total
  float position = ((float)(sL * -1) + (float)(sC * 0) + (float)(sR * 1)) / total;

  // Save last error for lost-line recovery
  lastError = (position > 0) ? 1 : (position < 0) ? -1 : 0;

  // --- APPLY CORRECTION ---
  if (position == 0) {
    // Perfect center
    setLeftMotor(1, BASE_SPEED);
    setRightMotor(1, BASE_SPEED);
  }
  else if (position < 0 && position > -1) {
    // Slightly left of center = soft right correction
    setLeftMotor(1, TURN_SPEED);
    setRightMotor(1, INNER_SPEED);
  }
  else if (position > 0 && position < 1) {
    // Slightly right of center = soft left correction
    setLeftMotor(1, INNER_SPEED);
    setRightMotor(1, TURN_SPEED);
  }
  else if (position <= -1) {
    // Far left = hard right pivot
    setLeftMotor(1, HARD_TURN_SPEED);
    setRightMotor(1, HARD_INNER);
  }
  else if (position >= 1) {
    // Far right = hard left pivot
    setLeftMotor(1, HARD_INNER);
    setRightMotor(1, HARD_TURN_SPEED);
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
    lostTime = 0;
    lastError = 0;
    lineTrackingMode = true;
    WebSerial.println("[OK] LINE TRACKING ON");
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
    setLeftMotor(0, 0);
    setRightMotor(1, BASE_SPEED);
    WebSerial.println("[OK] LEFT");
    delay(2000);
    stopMotors();
  }
  else if (message == "right" || message == "r") {
    lineTrackingMode = false;
    setLeftMotor(1, BASE_SPEED);
    setRightMotor(0, 0);
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
    setLeftMotor(1, BASE_SPEED);
    setRightMotor(0, 0);
    delay(2000);
    stopMotors();
    WebSerial.println("[TEST] Done.");
  }
  else if (message == "test right") {
    lineTrackingMode = false;
    WebSerial.println("[TEST] RIGHT motor only, 2 sec...");
    setLeftMotor(0, 0);
    setRightMotor(1, BASE_SPEED);
    delay(2000);
    stopMotors();
    WebSerial.println("[TEST] Done.");
  }
  else if (message == "test all") {
    lineTrackingMode = false;
    WebSerial.println("[TEST] Both motors, 2 sec...");
    setLeftMotor(1, BASE_SPEED);
    setRightMotor(1, BASE_SPEED);
    delay(2000);
    stopMotors();
    WebSerial.println("[TEST] Done.");
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

  Serial.println("Ready! http://192.168.4.1/webserial");
}

// -------------------------------------------------------
// LOOP — walang delay, maximum response speed
// -------------------------------------------------------
void loop() {
  dnsServer.processNextRequest();

  if (lineTrackingMode) {
    lineTrack(); // No delay — runs as fast as possible
  }
}