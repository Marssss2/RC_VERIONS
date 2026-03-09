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
int TURN_SPEED      = 220;
int INNER_SPEED     = 60;
int HARD_TURN_SPEED = 230;
#define HARD_INNER  0

// -------------------------------------------------------
// IR SENSOR PINS (TCRT5000)
// -------------------------------------------------------
#define IR_LEFT   34
#define IR_CENTER 35
#define IR_RIGHT  36

// -------------------------------------------------------
// SENSOR THRESHOLD
// -------------------------------------------------------
int SENSOR_THRESHOLD = 2000;

// -------------------------------------------------------
// MODE & LOST LINE HANDLING
// -------------------------------------------------------
bool lineTrackingMode = false;
int  lastError        = 0;
unsigned long lostTime = 0;
#define LOST_TIMEOUT 1500

// -------------------------------------------------------
// SENSOR READERS
// -------------------------------------------------------
int readLeft()   { return analogRead(IR_LEFT)   < SENSOR_THRESHOLD ? 1 : 0; }
int readCenter() { return analogRead(IR_CENTER) < SENSOR_THRESHOLD ? 1 : 0; }
int readRight()  { return analogRead(IR_RIGHT)  < SENSOR_THRESHOLD ? 1 : 0; }

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

void moveForward() {
  setLeftMotor(1, BASE_SPEED);
  setRightMotor(1, BASE_SPEED);
}

void moveBackward() {
  setLeftMotor(-1, BASE_SPEED);
  setRightMotor(-1, BASE_SPEED);
}

// -------------------------------------------------------
// LINE TRACKING LOGIC
// -------------------------------------------------------
void lineTrack() {
  int sL = readLeft();
  int sC = readCenter();
  int sR = readRight();
  int total = sL + sC + sR;

  if (sL == 1 && sC == 1 && sR == 1) {
    stopMotors();
    lineTrackingMode = false;
    WebSerial.println("[TRACK] End of line! Finished.");
    return;
  }

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
  float position = ((float)(sL * -1) + (float)(sC * 0) + (float)(sR * 1)) / total;
  lastError = (position > 0) ? 1 : (position < 0) ? -1 : 0;

  if (position == 0) {
    setLeftMotor(1, BASE_SPEED);
    setRightMotor(1, BASE_SPEED);
  } else if (position < 0 && position > -1) {
    setLeftMotor(1, TURN_SPEED);
    setRightMotor(1, INNER_SPEED);
  } else if (position > 0 && position < 1) {
    setLeftMotor(1, INNER_SPEED);
    setRightMotor(1, TURN_SPEED);
  } else if (position <= -1) {
    setLeftMotor(1, HARD_TURN_SPEED);
    setRightMotor(1, HARD_INNER);
  } else if (position >= 1) {
    setLeftMotor(1, HARD_INNER);
    setRightMotor(1, HARD_TURN_SPEED);
  }
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
    .top-bar { display:flex; gap:8px; margin-bottom:12px; flex-wrap:wrap; justify-content:center; }
    .top-btn { padding:10px 16px; border:none; border-radius:10px; font-size:13px; font-weight:bold; cursor:pointer; }
    .btn-on  { background:#00ff88; color:#0f0f1a; }
    .btn-off { background:#e94560; color:white; }
    .btn-stop{ background:#f5a623; color:#0f0f1a; }
    .speed-wrap { width:280px; margin-bottom:12px; text-align:center; }
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
// HANDLE JOYSTICK COMMANDS from /cmd?v=xxx
// FIXED: left/right swapped
// -------------------------------------------------------
void handleCmd(AsyncWebServerRequest *request) {
  if (request->hasParam("v")) {
    String cmd = request->getParam("v")->value();
    cmd.trim(); cmd.toLowerCase();

    if      (cmd == "f") { lineTrackingMode=false; moveForward(); }
    else if (cmd == "b") { lineTrackingMode=false; moveBackward(); }
    else if (cmd == "l") { lineTrackingMode=false; setLeftMotor(1,BASE_SPEED); setRightMotor(0,0); }  // FIXED
    else if (cmd == "r") { lineTrackingMode=false; setLeftMotor(0,0); setRightMotor(1,BASE_SPEED); }  // FIXED
    else if (cmd == "s") { lineTrackingMode=false; stopMotors(); }
    else if (cmd == "track on")  { lostTime=0; lastError=0; lineTrackingMode=true; }
    else if (cmd == "track off") { lineTrackingMode=false; stopMotors(); }
    else if (cmd.startsWith("set speed ")) {
      int spd = cmd.substring(10).toInt();
      if (spd >= 50 && spd <= 255) {
        BASE_SPEED      = spd;
        TURN_SPEED      = constrain(spd+20, 0, 255);
        INNER_SPEED     = constrain(spd-120, 0, 255);
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

  if (message == "control") {
    WebSerial.println("[GAMEPAD] Open this URL in your browser:");
    WebSerial.println("  http://192.168.4.1/control");
    WebSerial.println("  Para makabalik: http://192.168.4.1/webserial");
  }
  else if (message == "track on") {
    lostTime=0; lastError=0; lineTrackingMode=true;
    WebSerial.println("[OK] LINE TRACKING ON");
    WebSerial.print("     Speed: "); WebSerial.println(BASE_SPEED);
    WebSerial.print("     Threshold: "); WebSerial.println(SENSOR_THRESHOLD);
    WebSerial.println("     Type 'track off' or 's' to stop.");
  }
  else if (message == "track off") {
    lineTrackingMode=false; stopMotors();
    WebSerial.println("[OK] LINE TRACKING OFF");
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
    if (t < 0 || t > 4095) { WebSerial.println("[ERROR] Threshold must be 0-4095."); }
    else { SENSOR_THRESHOLD=t; WebSerial.print("[OK] Threshold: "); WebSerial.println(SENSOR_THRESHOLD); }
  }
  else if (message == "threshold") {
    WebSerial.print("[INFO] Threshold: "); WebSerial.println(SENSOR_THRESHOLD);
  }
  else if (message.startsWith("set speed ")) {
    int spd = message.substring(10).toInt();
    if (spd < 0 || spd > 255) { WebSerial.println("[ERROR] Speed 0-255."); }
    else {
      BASE_SPEED      = spd;
      TURN_SPEED      = constrain(spd+20, 0, 255);
      INNER_SPEED     = constrain(spd-120, 0, 255);
      HARD_TURN_SPEED = constrain(spd+30, 0, 255);
      WebSerial.print("[OK] Speed: "); WebSerial.println(BASE_SPEED);
    }
  }
  else if (message == "speed") {
    WebSerial.print("[INFO] Speed: "); WebSerial.println(BASE_SPEED);
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
    setLeftMotor(1, BASE_SPEED); setRightMotor(0, 0);  // FIXED
    WebSerial.println("[OK] LEFT"); delay(2000); stopMotors();
  }
  else if (message == "right" || message == "r") {
    lineTrackingMode=false;
    setLeftMotor(0, 0); setRightMotor(1, BASE_SPEED);  // FIXED
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
    WebSerial.println("    set speed <0-255> | set threshold <0-4095>");
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

  server.on("/control", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", GAMEPAD_HTML);
  });
  server.on("/cmd", HTTP_GET, handleCmd);
  server.onNotFound([](AsyncWebServerRequest *request) {
    request->redirect("/webserial");
  });
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
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