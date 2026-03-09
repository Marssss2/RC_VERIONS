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
// -------------------------------------------------------
int BASE_SPEED      = 200;
int HARD_TURN_SPEED = 230;
#define HARD_INNER  0

// -------------------------------------------------------
// IR SENSOR PINS (TCRT5000)
// -------------------------------------------------------
#define IR_LEFT   34
#define IR_CENTER 35
#define IR_RIGHT  36

// -------------------------------------------------------
// SENSOR THRESHOLD & AUTO-CALIBRATE
// -------------------------------------------------------
int SENSOR_THRESHOLD = 2000;

void autoCalibrate() {
  WebSerial.println("[CAL] Starting calibration...");
  WebSerial.println("[CAL] STEP 1: Place robot on WHITE surface. Sampling in 3 sec...");
  delay(3000);

  int wL=0, wC=0, wR=0;
  for (int i=0; i<50; i++) {
    wL += analogRead(IR_LEFT);
    wC += analogRead(IR_CENTER);
    wR += analogRead(IR_RIGHT);
    delay(10);
  }
  int calWhiteL = wL/50, calWhiteC = wC/50, calWhiteR = wR/50;
  WebSerial.print("[CAL] White: L="); WebSerial.print(calWhiteL);
  WebSerial.print(" C="); WebSerial.print(calWhiteC);
  WebSerial.print(" R="); WebSerial.println(calWhiteR);

  WebSerial.println("[CAL] STEP 2: Place robot on BLACK LINE. Sampling in 3 sec...");
  delay(3000);

  int bL=0, bC=0, bR=0;
  for (int i=0; i<50; i++) {
    bL += analogRead(IR_LEFT);
    bC += analogRead(IR_CENTER);
    bR += analogRead(IR_RIGHT);
    delay(10);
  }
  int calBlackL = bL/50, calBlackC = bC/50, calBlackR = bR/50;
  WebSerial.print("[CAL] Black: L="); WebSerial.print(calBlackL);
  WebSerial.print(" C="); WebSerial.print(calBlackC);
  WebSerial.print(" R="); WebSerial.println(calBlackR);

  int avgWhite = (calWhiteL + calWhiteC + calWhiteR) / 3;
  int avgBlack = (calBlackL + calBlackC + calBlackR) / 3;
  SENSOR_THRESHOLD = (avgWhite + avgBlack) / 2;

  WebSerial.print("[CAL] Auto threshold set to: "); WebSerial.println(SENSOR_THRESHOLD);
  WebSerial.println("[CAL] Calibration complete! Done.");
}

// -------------------------------------------------------
// SENSOR READERS
// -------------------------------------------------------
int readLeft()   { return analogRead(IR_LEFT)   < SENSOR_THRESHOLD ? 1 : 0; }
int readCenter() { return analogRead(IR_CENTER) < SENSOR_THRESHOLD ? 1 : 0; }
int readRight()  { return analogRead(IR_RIGHT)  < SENSOR_THRESHOLD ? 1 : 0; }

// -------------------------------------------------------
// PID CONTROL
// -------------------------------------------------------
float Kp = 80.0;
float Ki = 0.0;
float Kd = 20.0;
float lastPID  = 0;
float integral = 0;
unsigned long lastPIDTime = 0;

// -------------------------------------------------------
// MODE & LOST LINE HANDLING
// -------------------------------------------------------
bool lineTrackingMode = false;
int  lastError        = 0;
unsigned long lostTime = 0;
#define LOST_TIMEOUT 1500

// -------------------------------------------------------
// LAP TIMER
// -------------------------------------------------------
bool lapTimerActive    = false;
unsigned long lapStart = 0;
int lapCount           = 0;
unsigned long lapTimes[10];
bool wasOnStart        = false;

void checkLap(int sL, int sC, int sR) {
  bool onStart = (sL==1 && sC==1 && sR==1);
  if (onStart && !wasOnStart) {
    if (!lapTimerActive) {
      lapTimerActive = true;
      lapStart = millis();
      lapCount = 0;
      WebSerial.println("[LAP] Timer started!");
    } else {
      unsigned long lapTime = millis() - lapStart;
      lapStart = millis();
      if (lapCount < 10) lapTimes[lapCount] = lapTime;
      lapCount++;
      WebSerial.print("[LAP] Lap "); WebSerial.print(lapCount);
      WebSerial.print(": "); WebSerial.print(lapTime / 1000.0, 2);
      WebSerial.println(" sec");
    }
  }
  wasOnStart = onStart;
}

void printLapTimes() {
  if (lapCount == 0) { WebSerial.println("[LAP] No laps recorded yet."); return; }
  WebSerial.println("=== LAP TIMES ===");
  unsigned long best = 999999, total = 0;
  int show = min(lapCount, 10);
  for (int i=0; i<show; i++) {
    WebSerial.print("  Lap "); WebSerial.print(i+1);
    WebSerial.print(": "); WebSerial.print(lapTimes[i]/1000.0, 2);
    WebSerial.println(" sec");
    if (lapTimes[i] < best) best = lapTimes[i];
    total += lapTimes[i];
  }
  WebSerial.print("  Best: "); WebSerial.print(best/1000.0, 2); WebSerial.println(" sec");
  WebSerial.print("  Avg:  "); WebSerial.print((total/show)/1000.0, 2); WebSerial.println(" sec");
}

// -------------------------------------------------------
// MOTOR HELPERS
// -------------------------------------------------------
void setLeftMotor(int direction, int speed) {
  speed = constrain(speed, 0, 255);
  if (direction == 1) {
    digitalWrite(LEFT_IN1, HIGH); digitalWrite(LEFT_IN2, LOW);
  } else if (direction == -1) {
    digitalWrite(LEFT_IN1, LOW);  digitalWrite(LEFT_IN2, HIGH);
  } else {
    digitalWrite(LEFT_IN1, LOW);  digitalWrite(LEFT_IN2, LOW);
  }
  analogWrite(LEFT_EN, (direction == 0) ? 0 : speed);
}

void setRightMotor(int direction, int speed) {
  speed = constrain(speed, 0, 255);
  if (direction == 1) {
    digitalWrite(RIGHT_IN3, HIGH); digitalWrite(RIGHT_IN4, LOW);
  } else if (direction == -1) {
    digitalWrite(RIGHT_IN3, LOW);  digitalWrite(RIGHT_IN4, HIGH);
  } else {
    digitalWrite(RIGHT_IN3, LOW);  digitalWrite(RIGHT_IN4, LOW);
  }
  analogWrite(RIGHT_EN, (direction == 0) ? 0 : speed);
}

void stopMotors()   { setLeftMotor(0,0); setRightMotor(0,0); }
void moveForward()  { setLeftMotor(1,BASE_SPEED); setRightMotor(1,BASE_SPEED); }
void moveBackward() { setLeftMotor(-1,BASE_SPEED); setRightMotor(-1,BASE_SPEED); }

// -------------------------------------------------------
// PID LINE TRACKING
// -------------------------------------------------------
void lineTrack() {
  int sL = readLeft();
  int sC = readCenter();
  int sR = readRight();
  int total = sL + sC + sR;

  // End of line check FIRST
  if (sL==1 && sC==1 && sR==1) {
    delay(200);
    if (readLeft()==1 && readCenter()==1 && readRight()==1) {
      stopMotors();
      lineTrackingMode = false;
      WebSerial.println("[TRACK] End of line! Finished.");
      printLapTimes();
      return;
    }
  }

  // Lap timer check AFTER end of line
  checkLap(sL, sC, sR);

  // Lost line
  if (total == 0) {
    if (lostTime == 0) lostTime = millis();
    if (millis() - lostTime > LOST_TIMEOUT) {
      stopMotors();
      lineTrackingMode = false;
      WebSerial.println("[TRACK] Line lost! Stopping.");
      return;
    }
    if (lastError >= 0) {
      setLeftMotor(1, HARD_TURN_SPEED);
      setRightMotor(1, HARD_INNER);
    } else {
      setLeftMotor(1, HARD_INNER);
      setRightMotor(1, HARD_TURN_SPEED);
    }
    return;
  }

  lostTime = 0;

  float position = ((float)(sL*-1) + (float)(sC*0) + (float)(sR*1)) / total;
  lastError = (position > 0) ? 1 : (position < 0) ? -1 : 0;

  // PID
  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0;
  if (dt <= 0 || dt > 0.5) dt = 0.01;
  lastPIDTime = now;

  float error      = position;
  integral        += error * dt;
  integral         = constrain(integral, -1.0, 1.0);
  float derivative = (error - lastPID) / dt;
  lastPID          = error;

  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  correction = constrain(correction, -255, 255);

  int leftSpeed  = constrain(BASE_SPEED + (int)correction, 0, 255);
  int rightSpeed = constrain(BASE_SPEED - (int)correction, 0, 255);

  setLeftMotor(1, leftSpeed);
  setRightMotor(1, rightSpeed);
}

// -------------------------------------------------------
// GAMEPAD HTML
// -------------------------------------------------------
const char GAMEPAD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
  <title>ESP32 Gamepad</title>
  <style>
    * { margin:0; padding:0; box-sizing:border-box; -webkit-user-select:none; user-select:none; }
    body {
      background:#0f0f1a; color:white; font-family:Arial,sans-serif;
      display:flex; flex-direction:column; align-items:center;
      height:100vh; padding:15px; overflow:hidden; touch-action:none;
    }
    h2 { color:#e94560; font-size:16px; letter-spacing:3px; margin-bottom:8px; }
    .top-bar { display:flex; gap:8px; margin-bottom:10px; flex-wrap:wrap; justify-content:center; }
    .top-btn { padding:10px 14px; border:none; border-radius:10px; font-size:12px; font-weight:bold; cursor:pointer; }
    .btn-on   { background:#00ff88; color:#0f0f1a; }
    .btn-off  { background:#e94560; color:white; }
    .btn-stop { background:#f5a623; color:#0f0f1a; }
    .btn-ws   { background:#444; color:white; }
    .speed-wrap { width:280px; margin-bottom:10px; text-align:center; }
    .speed-label { font-size:11px; color:#888; margin-bottom:4px; }
    input[type=range] { width:100%; accent-color:#e94560; cursor:pointer; }
    .joystick-area {
      position:relative; width:200px; height:200px;
      background:rgba(255,255,255,0.04); border-radius:50%;
      border:2px solid rgba(233,69,96,0.3); touch-action:none;
    }
    .knob {
      width:70px; height:70px;
      background:radial-gradient(circle at 35% 35%, #e94560, #8b0000);
      border-radius:50%; position:absolute; top:50%; left:50%;
      transform:translate(-50%,-50%);
      box-shadow:0 0 20px rgba(233,69,96,0.6); pointer-events:none;
    }
    .dir { position:absolute; color:rgba(255,255,255,0.25); font-size:11px; }
    .dir.up    { top:6px;    left:50%; transform:translateX(-50%); }
    .dir.down  { bottom:6px; left:50%; transform:translateX(-50%); }
    .dir.left  { left:6px;   top:50%; transform:translateY(-50%); }
    .dir.right { right:6px;  top:50%; transform:translateY(-50%); }
    .cmd-label { margin-top:12px; font-size:14px; color:#e94560; letter-spacing:2px; font-weight:bold; height:20px; }
  </style>
</head>
<body>
  <h2>🎮 RC CONTROLLER</h2>
  <div class="top-bar">
    <button class="top-btn btn-on"   ontouchstart="sendCmd('track on')"  onclick="sendCmd('track on')">▶ TRACK ON</button>
    <button class="top-btn btn-stop" ontouchstart="sendCmd('s')"         onclick="sendCmd('s')">■ STOP</button>
    <button class="top-btn btn-off"  ontouchstart="sendCmd('track off')" onclick="sendCmd('track off')">✕ TRACK OFF</button>
    <button class="top-btn btn-ws"   onclick="location.href='/webserial'">📟 WebSerial</button>
  </div>
  <div class="speed-wrap">
    <div class="speed-label">SPEED: <span id="spd">200</span></div>
    <input type="range" min="50" max="255" value="200" id="spSlider" oninput="setSpeed(this.value)">
  </div>
  <div class="joystick-area" id="joyArea">
    <div class="knob" id="knob"></div>
    <span class="dir up">▲</span>
    <span class="dir down">▼</span>
    <span class="dir left">◄</span>
    <span class="dir right">►</span>
  </div>
  <div class="cmd-label" id="cmdLabel">READY</div>
<script>
  const joyArea  = document.getElementById('joyArea');
  const knob     = document.getElementById('knob');
  const cmdLabel = document.getElementById('cmdLabel');
  const RADIUS = 65, CENTER = 100;
  let isDragging = false, currentCmd = '';

  function sendCmd(cmd) { fetch('/cmd?v=' + encodeURIComponent(cmd)); }
  function setSpeed(val) { document.getElementById('spd').innerText = val; sendCmd('set speed ' + val); }

  function getPos(e) {
    const rect = joyArea.getBoundingClientRect();
    const touch = e.touches ? e.touches[0] : e;
    return { x: touch.clientX - rect.left - CENTER, y: touch.clientY - rect.top - CENTER };
  }

  function updateJoystick(x, y) {
    const dist  = Math.sqrt(x*x + y*y);
    const clamp = Math.min(dist, RADIUS);
    const angle = Math.atan2(y, x);
    knob.style.left = (CENTER + Math.cos(angle)*clamp) + 'px';
    knob.style.top  = (CENTER + Math.sin(angle)*clamp) + 'px';
    knob.style.transform = 'translate(-50%,-50%)';
    const deadzone = 20;
    let cmd = '';
    if (dist < deadzone) {
      cmd = 's'; cmdLabel.innerText = '■ STOP';
    } else {
      const deg = angle * 180 / Math.PI;
      if      (deg >= -45  && deg <  45)  { cmd = 'r'; cmdLabel.innerText = '► RIGHT'; }
      else if (deg >=  45  && deg < 135)  { cmd = 'b'; cmdLabel.innerText = '▼ BACKWARD'; }
      else if (deg >= 135  || deg < -135) { cmd = 'l'; cmdLabel.innerText = '◄ LEFT'; }
      else                                { cmd = 'f'; cmdLabel.innerText = '▲ FORWARD'; }
    }
    if (cmd !== currentCmd) { currentCmd = cmd; sendCmd(cmd); }
  }

  function resetKnob() {
    knob.style.left = '50%'; knob.style.top = '50%';
    knob.style.transform = 'translate(-50%,-50%)';
    cmdLabel.innerText = '■ STOP';
    sendCmd('s'); currentCmd = '';
  }

  joyArea.addEventListener('touchstart', e => { e.preventDefault(); isDragging=true; const p=getPos(e); updateJoystick(p.x,p.y); }, {passive:false});
  joyArea.addEventListener('touchmove',  e => { e.preventDefault(); if(isDragging){const p=getPos(e); updateJoystick(p.x,p.y);} }, {passive:false});
  joyArea.addEventListener('touchend',   e => { e.preventDefault(); isDragging=false; resetKnob(); }, {passive:false});
  joyArea.addEventListener('mousedown',  e => { isDragging=true; const p=getPos(e); updateJoystick(p.x,p.y); });
  document.addEventListener('mousemove', e => { if(!isDragging)return; const rect=joyArea.getBoundingClientRect(); updateJoystick(e.clientX-rect.left-CENTER, e.clientY-rect.top-CENTER); });
  document.addEventListener('mouseup',   () => { if(isDragging){isDragging=false; resetKnob();} });
</script>
</body>
</html>
)rawliteral";

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
// HANDLE JOYSTICK COMMANDS
// -------------------------------------------------------
void handleCmd(AsyncWebServerRequest *request) {
  if (request->hasParam("v")) {
    String cmd = request->getParam("v")->value();
    cmd.trim(); cmd.toLowerCase();
    if      (cmd == "f") { lineTrackingMode=false; moveForward(); }
    else if (cmd == "b") { lineTrackingMode=false; moveBackward(); }
    else if (cmd == "l") { lineTrackingMode=false; setLeftMotor(1,BASE_SPEED); setRightMotor(0,0); }
    else if (cmd == "r") { lineTrackingMode=false; setLeftMotor(0,0); setRightMotor(1,BASE_SPEED); }
    else if (cmd == "s") { lineTrackingMode=false; stopMotors(); }
    else if (cmd == "track on")  { lostTime=0; lastError=0; integral=0; lastPID=0; lineTrackingMode=true; }
    else if (cmd == "track off") { lineTrackingMode=false; stopMotors(); }
    else if (cmd.startsWith("set speed ")) {
      int spd = cmd.substring(10).toInt();
      if (spd >= 50 && spd <= 255) {
        BASE_SPEED      = spd;
        HARD_TURN_SPEED = constrain(spd+30, 0, 255);
      }
    }
  }
  request->send(200, "text/plain", "OK");
}

// -------------------------------------------------------
// WEBSERIAL COMMANDS
// -------------------------------------------------------
void recvMsg(uint8_t *data, size_t len) {
  String message = "";
  for (int i = 0; i < len; i++) { message += char(data[i]); }
  message.trim(); message.toLowerCase();

  // FIXED: control command — nagpapakita ng clickable link
  if (message == "control") {
    WebSerial.println("[GAMEPAD] Opening RC Controller...");
    WebSerial.println("  >> http://192.168.4.1/control <<");
    WebSerial.println("  (I-click o i-open sa browser)");
    return;
  }
  else if (message == "track on") {
    lostTime=0; lastError=0; integral=0; lastPID=0;
    lapTimerActive=false; lapCount=0; wasOnStart=false;
    lineTrackingMode=true;
    WebSerial.println("[OK] LINE TRACKING ON");
    WebSerial.print("     Speed: "); WebSerial.println(BASE_SPEED);
    WebSerial.print("     Threshold: "); WebSerial.println(SENSOR_THRESHOLD);
    WebSerial.println("     Type 'track off' or 's' to stop.");
  }
  else if (message == "track off") {
    lineTrackingMode=false; stopMotors();
    WebSerial.println("[OK] LINE TRACKING OFF");
  }
  else if (message == "calibrate") {
    lineTrackingMode=false; stopMotors();
    autoCalibrate();
  }
  else if (message == "laps") {
    printLapTimes();
  }
  else if (message == "lap reset") {
    lapTimerActive=false; lapCount=0; wasOnStart=false;
    memset(lapTimes, 0, sizeof(lapTimes));
    WebSerial.println("[LAP] Timer reset.");
  }
  else if (message == "sensor test") {
    int rawL = analogRead(IR_LEFT);
    int rawC = analogRead(IR_CENTER);
    int rawR = analogRead(IR_RIGHT);
    WebSerial.println("=== SENSOR READINGS ===");
    WebSerial.print("  LEFT   (GPIO34): raw="); WebSerial.print(rawL);
    WebSerial.print("  → "); WebSerial.println(rawL < SENSOR_THRESHOLD ? "BLACK LINE" : "white");
    WebSerial.print("  CENTER (GPIO35): raw="); WebSerial.print(rawC);
    WebSerial.print("  → "); WebSerial.println(rawC < SENSOR_THRESHOLD ? "BLACK LINE" : "white");
    WebSerial.print("  RIGHT  (VP/36):  raw="); WebSerial.print(rawR);
    WebSerial.print("  → "); WebSerial.println(rawR < SENSOR_THRESHOLD ? "BLACK LINE" : "white");
    WebSerial.print("  Threshold: "); WebSerial.println(SENSOR_THRESHOLD);
  }
  else if (message.startsWith("set threshold ")) {
    int t = message.substring(14).toInt();
    if (t < 0 || t > 4095) { WebSerial.println("[ERROR] Threshold 0-4095."); }
    else { SENSOR_THRESHOLD=t; WebSerial.print("[OK] Threshold: "); WebSerial.println(SENSOR_THRESHOLD); }
  }
  else if (message == "threshold") {
    WebSerial.print("[INFO] Threshold: "); WebSerial.println(SENSOR_THRESHOLD);
  }
  else if (message.startsWith("set kp ")) {
    Kp = message.substring(7).toFloat();
    WebSerial.print("[OK] Kp: "); WebSerial.println(Kp);
  }
  else if (message.startsWith("set kd ")) {
    Kd = message.substring(7).toFloat();
    WebSerial.print("[OK] Kd: "); WebSerial.println(Kd);
  }
  else if (message.startsWith("set speed ")) {
    int spd = message.substring(10).toInt();
    if (spd < 0 || spd > 255) { WebSerial.println("[ERROR] Speed 0-255."); }
    else {
      BASE_SPEED      = spd;
      HARD_TURN_SPEED = constrain(spd+30, 0, 255);
      WebSerial.print("[OK] Speed: "); WebSerial.println(BASE_SPEED);
    }
  }
  else if (message == "speed") {
    WebSerial.print("[INFO] Speed: "); WebSerial.println(BASE_SPEED);
  }
  else if (message == "pid") {
    WebSerial.print("[PID] Kp="); WebSerial.print(Kp);
    WebSerial.print(" Ki="); WebSerial.print(Ki);
    WebSerial.print(" Kd="); WebSerial.println(Kd);
  }
  else if (message == "forward" || message == "f") {
    lineTrackingMode=false; moveForward();
    WebSerial.println("[OK] FORWARD"); delay(2000); stopMotors();
  }
  else if (message == "backward" || message == "b") {
    lineTrackingMode=false; moveBackward();
    WebSerial.println("[OK] BACKWARD"); delay(2000); stopMotors();
  }
  else if (message == "left" || message == "l") {
    lineTrackingMode=false;
    setLeftMotor(1,BASE_SPEED); setRightMotor(0,0);
    WebSerial.println("[OK] LEFT"); delay(2000); stopMotors();
  }
  else if (message == "right" || message == "r") {
    lineTrackingMode=false;
    setLeftMotor(0,0); setRightMotor(1,BASE_SPEED);
    WebSerial.println("[OK] RIGHT"); delay(2000); stopMotors();
  }
  else if (message == "stop" || message == "s") {
    lineTrackingMode=false; stopMotors();
    WebSerial.println("[OK] STOPPED");
  }
  else if (message == "test left") {
    lineTrackingMode=false;
    WebSerial.println("[TEST] LEFT motor, 2 sec...");
    setLeftMotor(1,BASE_SPEED); setRightMotor(0,0);
    delay(2000); stopMotors(); WebSerial.println("[TEST] Done.");
  }
  else if (message == "test right") {
    lineTrackingMode=false;
    WebSerial.println("[TEST] RIGHT motor, 2 sec...");
    setLeftMotor(0,0); setRightMotor(1,BASE_SPEED);
    delay(2000); stopMotors(); WebSerial.println("[TEST] Done.");
  }
  else if (message == "test all") {
    lineTrackingMode=false;
    WebSerial.println("[TEST] Both motors, 2 sec...");
    moveForward(); delay(2000); stopMotors();
    WebSerial.println("[TEST] Done.");
  }
  else if (message.startsWith("set ssid ")) {
    ssid = message.substring(9);
    preferences.putString("ssid", ssid);
    WebSerial.println("[OK] SSID updated. Restarting...");
    delay(1000); ESP.restart();
  }
  else if (message.startsWith("set pass ")) {
    password = message.substring(9);
    if (password.length() < 8) { WebSerial.println("[ERROR] Min 8 chars."); return; }
    preferences.putString("pass", password);
    WebSerial.println("[OK] Password updated. Restarting...");
    delay(1000); ESP.restart();
  }
  else {
    WebSerial.println("[?] Unknown. Commands:");
    WebSerial.println("    f b l r s | track on/off | sensor test");
    WebSerial.println("    calibrate | laps | lap reset");
    WebSerial.println("    set speed <0-255> | set threshold <0-4095>");
    WebSerial.println("    set kp <val> | set kd <val> | pid");
    WebSerial.println("    test left/right/all | control");
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

  // FIXED: root → webserial, hindi na gamepad agad
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->redirect("/webserial");
  });
  server.on("/control", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", GAMEPAD_HTML);
  });
  server.on("/cmd", HTTP_GET, handleCmd);
  server.onNotFound([](AsyncWebServerRequest *request) {
    request->redirect("/webserial");
  });

  WebSerial.begin(&server);
  WebSerial.onMessage(recvMsg);
  server.begin();

  Serial.println("Ready!");
  Serial.println("WebSerial: http://192.168.4.1/webserial");
  Serial.println("Gamepad:   http://192.168.4.1/control");
}

// -------------------------------------------------------
// LOOP
// -------------------------------------------------------
void loop() {
  dnsServer.processNextRequest();
  if (lineTrackingMode) {
    lineTrack();
  }
}
