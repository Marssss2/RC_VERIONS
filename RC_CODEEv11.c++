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
// SENSOR THRESHOLD
// -------------------------------------------------------
int SENSOR_THRESHOLD = 2000;

// -------------------------------------------------------
// CALIBRATION STATE
// 0=idle,1=wait white,2=sample white,3=wait black,4=sample black,5=done
// -------------------------------------------------------
volatile int calState = 0;
String calStatusMsg   = "Ready";
int calProgress       = 0;

void autoCalibrate() {
  calState = 1; calStatusMsg = "Place on WHITE surface..."; calProgress = 0;
  WebSerial.println("[CAL] STEP 1: Place on WHITE. Sampling in 3 sec...");
  for (int i=0;i<30;i++) { delay(100); calProgress=i; }

  calState = 2; calStatusMsg = "Sampling WHITE...";
  int wL=0,wC=0,wR=0;
  for (int i=0;i<50;i++) { wL+=analogRead(IR_LEFT); wC+=analogRead(IR_CENTER); wR+=analogRead(IR_RIGHT); delay(10); calProgress=30+i; }
  int calWhiteL=wL/50, calWhiteC=wC/50, calWhiteR=wR/50;
  WebSerial.print("[CAL] White: L="); WebSerial.print(calWhiteL); WebSerial.print(" C="); WebSerial.print(calWhiteC); WebSerial.print(" R="); WebSerial.println(calWhiteR);

  calState = 3; calStatusMsg = "Place on BLACK LINE...";
  WebSerial.println("[CAL] STEP 2: Place on BLACK LINE. Sampling in 3 sec...");
  for (int i=0;i<30;i++) { delay(100); calProgress=80+i/3; }

  calState = 4; calStatusMsg = "Sampling BLACK...";
  int bL=0,bC=0,bR=0;
  for (int i=0;i<50;i++) { bL+=analogRead(IR_LEFT); bC+=analogRead(IR_CENTER); bR+=analogRead(IR_RIGHT); delay(10); }
  int calBlackL=bL/50, calBlackC=bC/50, calBlackR=bR/50;
  WebSerial.print("[CAL] Black: L="); WebSerial.print(calBlackL); WebSerial.print(" C="); WebSerial.print(calBlackC); WebSerial.print(" R="); WebSerial.println(calBlackR);

  int avgWhite=(calWhiteL+calWhiteC+calWhiteR)/3;
  int avgBlack=(calBlackL+calBlackC+calBlackR)/3;
  SENSOR_THRESHOLD=(avgWhite+avgBlack)/2;

  calState=5; calProgress=100;
  calStatusMsg="Done! Threshold="+String(SENSOR_THRESHOLD);
  WebSerial.print("[CAL] Threshold set to: "); WebSerial.println(SENSOR_THRESHOLD);
  WebSerial.println("[CAL] Calibration complete!");

  delay(3000);
  calState=0; calStatusMsg="Ready"; calProgress=0;
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
float Kp = 40.0;
float Ki = 0.0;
float Kd = 35.0;
float lastPID    = 0;
float integral   = 0;
float lastCorrection = 0;
unsigned long lastPIDTime = 0;

// -------------------------------------------------------
// ADAPTIVE SPEED
// -------------------------------------------------------
bool adaptiveSpeed = true;
#define ADAPTIVE_MIN_SPEED 120

// -------------------------------------------------------
// MODE & LOST LINE HANDLING
// -------------------------------------------------------
bool lineTrackingMode = false;
int  lastError        = 0;
unsigned long lostTime = 0;
#define LOST_TIMEOUT 1500

// -------------------------------------------------------
// REVERSE SEARCH
// -------------------------------------------------------
bool reverseSearch = true;
#define REVERSE_DURATION 400

// -------------------------------------------------------
// LAP TIMER
// -------------------------------------------------------
bool lapTimerActive   = false;
unsigned long lapStart = 0;
int lapCount          = 0;
unsigned long lapTimes[10];
bool wasOnStart       = false;

void checkLap(int sL, int sC, int sR) {
  bool onStart = (sL==1 && sC==1 && sR==1);
  if (onStart && !wasOnStart) {
    if (!lapTimerActive) {
      lapTimerActive=true; lapStart=millis(); lapCount=0;
      WebSerial.println("[LAP] Timer started!");
    } else {
      unsigned long lapTime=millis()-lapStart;
      lapStart=millis();
      if (lapCount<10) lapTimes[lapCount]=lapTime;
      lapCount++;
      WebSerial.print("[LAP] Lap "); WebSerial.print(lapCount);
      WebSerial.print(": "); WebSerial.print(lapTime/1000.0,2); WebSerial.println(" sec");
    }
  }
  wasOnStart=onStart;
}

String getLapTimesJSON() {
  String json="{\"count\":"+String(lapCount)+",\"laps\":[";
  int show=min(lapCount,10);
  unsigned long best=999999,total=0;
  for (int i=0;i<show;i++) {
    if (i>0) json+=",";
    json+=String(lapTimes[i]/1000.0,2);
    if (lapTimes[i]<best) best=lapTimes[i];
    total+=lapTimes[i];
  }
  json+="]";
  if (show>0) { json+=",\"best\":"+String(best/1000.0,2); json+=",\"avg\":"+String((total/show)/1000.0,2); }
  json+="}";
  return json;
}

void printLapTimes() {
  if (lapCount==0) { WebSerial.println("[LAP] No laps yet."); return; }
  WebSerial.println("=== LAP TIMES ===");
  unsigned long best=999999,total=0;
  int show=min(lapCount,10);
  for (int i=0;i<show;i++) {
    WebSerial.print("  Lap "); WebSerial.print(i+1);
    WebSerial.print(": "); WebSerial.print(lapTimes[i]/1000.0,2); WebSerial.println(" sec");
    if (lapTimes[i]<best) best=lapTimes[i];
    total+=lapTimes[i];
  }
  WebSerial.print("  Best: "); WebSerial.print(best/1000.0,2); WebSerial.println(" sec");
  WebSerial.print("  Avg:  "); WebSerial.print((total/show)/1000.0,2); WebSerial.println(" sec");
}

// -------------------------------------------------------
// SETTINGS SAVE / LOAD / RESET
// -------------------------------------------------------
void saveSettings() {
  preferences.begin("rc-settings", false);
  preferences.putFloat("kp", Kp);
  preferences.putFloat("ki", Ki);
  preferences.putFloat("kd", Kd);
  preferences.putInt("speed", BASE_SPEED);
  preferences.putInt("thresh", SENSOR_THRESHOLD);
  preferences.putBool("adaptive", adaptiveSpeed);
  preferences.putBool("reverse", reverseSearch);
  preferences.end();
  WebSerial.println("[SAVE] Settings saved!");
  WebSerial.print("  Kp="); WebSerial.print(Kp);
  WebSerial.print(" Kd="); WebSerial.print(Kd);
  WebSerial.print(" Speed="); WebSerial.print(BASE_SPEED);
  WebSerial.print(" Thresh="); WebSerial.println(SENSOR_THRESHOLD);
}

void loadSettings() {
  preferences.begin("rc-settings", true);
  Kp              = preferences.getFloat("kp",    40.0);
  Ki              = preferences.getFloat("ki",     0.0);
  Kd              = preferences.getFloat("kd",    35.0);
  BASE_SPEED      = preferences.getInt("speed",   200);
  SENSOR_THRESHOLD= preferences.getInt("thresh", 2000);
  adaptiveSpeed   = preferences.getBool("adaptive", true);
  reverseSearch   = preferences.getBool("reverse",  true);
  preferences.end();
  HARD_TURN_SPEED = constrain(BASE_SPEED+30, 0, 255);
  WebSerial.println("[LOAD] Settings loaded!");
}

void resetSettings() {
  preferences.begin("rc-settings", false);
  preferences.clear();
  preferences.end();
  Kp=40.0; Ki=0.0; Kd=35.0;
  BASE_SPEED=200; HARD_TURN_SPEED=230;
  SENSOR_THRESHOLD=2000;
  adaptiveSpeed=true; reverseSearch=true;
  WebSerial.println("[RESET] Settings reset to defaults!");
}

// -------------------------------------------------------
// MOTOR HELPERS
// -------------------------------------------------------
void setLeftMotor(int direction, int speed) {
  speed=constrain(speed,0,255);
  if (direction==1)       { digitalWrite(LEFT_IN1,HIGH); digitalWrite(LEFT_IN2,LOW); }
  else if (direction==-1) { digitalWrite(LEFT_IN1,LOW);  digitalWrite(LEFT_IN2,HIGH); }
  else                    { digitalWrite(LEFT_IN1,LOW);  digitalWrite(LEFT_IN2,LOW); }
  analogWrite(LEFT_EN,(direction==0)?0:speed);
}

void setRightMotor(int direction, int speed) {
  speed=constrain(speed,0,255);
  if (direction==1)       { digitalWrite(RIGHT_IN3,HIGH); digitalWrite(RIGHT_IN4,LOW); }
  else if (direction==-1) { digitalWrite(RIGHT_IN3,LOW);  digitalWrite(RIGHT_IN4,HIGH); }
  else                    { digitalWrite(RIGHT_IN3,LOW);  digitalWrite(RIGHT_IN4,LOW); }
  analogWrite(RIGHT_EN,(direction==0)?0:speed);
}

void stopMotors()   { setLeftMotor(0,0); setRightMotor(0,0); }
void moveForward()  { setLeftMotor(1,BASE_SPEED); setRightMotor(1,BASE_SPEED); }
void moveBackward() { setLeftMotor(-1,BASE_SPEED); setRightMotor(-1,BASE_SPEED); }

// -------------------------------------------------------
// PID LINE TRACKING
// -------------------------------------------------------
void lineTrack() {
  int sL=readLeft(), sC=readCenter(), sR=readRight();
  int total=sL+sC+sR;

  // End of line
  if (sL==1&&sC==1&&sR==1) {
    delay(200);
    if (readLeft()==1&&readCenter()==1&&readRight()==1) {
      stopMotors(); lineTrackingMode=false;
      WebSerial.println("[TRACK] End of line! Finished.");
      printLapTimes(); return;
    }
  }

  checkLap(sL,sC,sR);

  // Lost line
  if (total==0) {
    if (lostTime==0) lostTime=millis();
    if (millis()-lostTime>LOST_TIMEOUT) {
      stopMotors(); lineTrackingMode=false;
      WebSerial.println("[TRACK] Line lost! Stopping."); return;
    }
    // REVERSE SEARCH: umatras muna bago mag-spin
    if (reverseSearch && millis()-lostTime < REVERSE_DURATION) {
      setLeftMotor(-1, BASE_SPEED/2);
      setRightMotor(-1, BASE_SPEED/2);
      return;
    }
    // Then spin toward last known direction
    if (lastError>=0) { setLeftMotor(1,HARD_TURN_SPEED); setRightMotor(1,HARD_INNER); }
    else              { setLeftMotor(1,HARD_INNER); setRightMotor(1,HARD_TURN_SPEED); }
    return;
  }

  lostTime=0;
  float position=((float)(sL*-1)+(float)(sC*0)+(float)(sR*1))/total;
  lastError=(position>0)?1:(position<0)?-1:0;

  unsigned long now=millis();
  float dt=(now-lastPIDTime)/1000.0;
  if (dt<=0||dt>0.5) dt=0.01;
  lastPIDTime=now;

  float error=position;
  integral+=error*dt;
  integral=constrain(integral,-1.0,1.0);
  float derivative=(error-lastPID)/dt;
  lastPID=error;

  float correction=(Kp*error)+(Ki*integral)+(Kd*derivative);
  correction=constrain(correction,-255,255);
  lastCorrection=correction;

  // ADAPTIVE SPEED: bumabagal kung malaki ang correction (sharp curve)
  int effectiveSpeed = BASE_SPEED;
  if (adaptiveSpeed) {
    float absCor=abs(correction);
    effectiveSpeed=map((int)absCor, 0, 255, BASE_SPEED, ADAPTIVE_MIN_SPEED);
    effectiveSpeed=constrain(effectiveSpeed, ADAPTIVE_MIN_SPEED, BASE_SPEED);
  }

  setLeftMotor(1,  constrain(effectiveSpeed+(int)correction, 0, 255));
  setRightMotor(1, constrain(effectiveSpeed-(int)correction, 0, 255));
}

// -------------------------------------------------------
// TELEMETRY ENDPOINT → /telemetry
// -------------------------------------------------------
void handleTelemetry(AsyncWebServerRequest *request) {
  int rawL=analogRead(IR_LEFT), rawC=analogRead(IR_CENTER), rawR=analogRead(IR_RIGHT);
  // Battery voltage: ESP32 ADC pin 35 (or use 3.3V ref estimate)
  // Using internal supply estimate via hall sensor / simple divider approx
  float batt = analogRead(34) * (3.3 / 4095.0) * 2.0; // adjust multiplier per divider
  String json="{";
  json+="\"rawL\":"+String(rawL)+",";
  json+="\"rawC\":"+String(rawC)+",";
  json+="\"rawR\":"+String(rawR)+",";
  json+="\"L\":"+String(rawL<SENSOR_THRESHOLD?1:0)+",";
  json+="\"C\":"+String(rawC<SENSOR_THRESHOLD?1:0)+",";
  json+="\"R\":"+String(rawR<SENSOR_THRESHOLD?1:0)+",";
  json+="\"thresh\":"+String(SENSOR_THRESHOLD)+",";
  json+="\"tracking\":"+String(lineTrackingMode?1:0)+",";
  json+="\"correction\":"+String((int)lastCorrection)+",";
  json+="\"speed\":"+String(BASE_SPEED)+",";
  json+="\"kp\":"+String(Kp,1)+",";
  json+="\"kd\":"+String(Kd,1)+",";
  json+="\"adaptive\":"+String(adaptiveSpeed?1:0)+",";
  json+="\"reverse\":"+String(reverseSearch?1:0)+",";
  json+="\"batt\":"+String(batt,2);
  json+="}";
  request->send(200,"application/json",json);
}

// -------------------------------------------------------
// GAMEPAD HTML — RC_CODEv11
// -------------------------------------------------------
const char GAMEPAD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
  <title>RC Controller v11</title>
  <style>
    *{margin:0;padding:0;box-sizing:border-box;-webkit-user-select:none;user-select:none;}
    body{background:#0f0f1a;color:white;font-family:Arial,sans-serif;
      display:flex;flex-direction:column;align-items:center;
      min-height:100vh;padding:10px 10px 24px;overflow-y:auto;touch-action:pan-y;}
    h2{color:#e94560;font-size:15px;letter-spacing:3px;margin-bottom:6px;}

    /* TRACK MODE BADGE */
    .mode-badge{
      padding:4px 14px;border-radius:99px;font-size:11px;font-weight:bold;
      letter-spacing:1px;margin-bottom:8px;transition:all 0.3s;}
    .mode-badge.off{background:#1a1a2e;color:#555;border:1px solid #333;}
    .mode-badge.on {background:#00ff88;color:#0f0f1a;border:1px solid #00ff88;
      box-shadow:0 0 12px rgba(0,255,136,0.4);}

    /* TOP BAR */
    .top-bar{display:flex;gap:6px;margin-bottom:8px;flex-wrap:wrap;justify-content:center;}
    .top-btn{padding:9px 12px;border:none;border-radius:10px;font-size:11px;font-weight:bold;cursor:pointer;}
    .btn-on  {background:#00ff88;color:#0f0f1a;}
    .btn-off {background:#e94560;color:white;}
    .btn-stop{background:#f5a623;color:#0f0f1a;}

    /* SLIDERS */
    .sliders-wrap{width:300px;margin-bottom:8px;background:rgba(255,255,255,0.04);border-radius:12px;padding:10px 14px;border:1px solid rgba(255,255,255,0.07);}
    .slider-row{display:flex;align-items:center;gap:8px;margin-bottom:6px;}
    .slider-row:last-child{margin-bottom:0;}
    .slider-label{font-size:10px;color:#888;width:85px;flex-shrink:0;}
    .slider-label span{color:#e94560;font-weight:bold;}
    input[type=range]{flex:1;accent-color:#e94560;cursor:pointer;height:4px;}
    .slider-val{font-size:10px;color:#ccc;width:36px;text-align:right;flex-shrink:0;}

    /* JOYSTICK */
    .joystick-area{position:relative;width:190px;height:190px;background:rgba(255,255,255,0.04);border-radius:50%;border:2px solid rgba(233,69,96,0.3);touch-action:none;flex-shrink:0;}
    .knob{width:66px;height:66px;background:radial-gradient(circle at 35% 35%,#e94560,#8b0000);border-radius:50%;position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);box-shadow:0 0 20px rgba(233,69,96,0.6);pointer-events:none;}
    .dir{position:absolute;color:rgba(255,255,255,0.25);font-size:11px;}
    .dir.up   {top:6px;left:50%;transform:translateX(-50%);}
    .dir.down {bottom:6px;left:50%;transform:translateX(-50%);}
    .dir.left {left:6px;top:50%;transform:translateY(-50%);}
    .dir.right{right:6px;top:50%;transform:translateY(-50%);}
    .cmd-label{margin-top:8px;font-size:13px;color:#e94560;letter-spacing:2px;font-weight:bold;height:18px;}

    /* PANEL */
    .panel{width:100%;max-width:340px;margin-top:10px;display:flex;flex-direction:column;gap:8px;}
    .section{background:rgba(255,255,255,0.04);border-radius:12px;padding:10px 12px;}
    .section-title{font-size:10px;color:#888;letter-spacing:2px;margin-bottom:7px;text-transform:uppercase;}

    /* SENSOR BOXES */
    .sensor-row{display:flex;gap:8px;justify-content:center;}
    .sensor-col{display:flex;flex-direction:column;align-items:center;flex:1;}
    .sensor-box{width:100%;padding:7px 4px;border-radius:8px;text-align:center;font-size:10px;font-weight:bold;background:#1a1a2e;border:1px solid #333;transition:background 0.15s,color 0.15s;}
    .sensor-box.on{background:#e94560;color:white;border-color:#e94560;}
    .sensor-label{font-size:9px;color:#666;margin-top:3px;}
    .sensor-raw{font-size:9px;color:#444;margin-top:1px;}

    /* ADC BARS */
    .adc-bar-bg{width:100%;height:4px;background:#1a1a2e;border-radius:99px;margin-top:3px;overflow:hidden;}
    .adc-bar-fill{height:100%;border-radius:99px;background:#e94560;transition:width 0.15s;}

    /* PID CORRECTION METER */
    .correction-wrap{margin-top:6px;}
    .correction-label{font-size:9px;color:#666;margin-bottom:3px;display:flex;justify-content:space-between;}
    .correction-track{position:relative;height:8px;background:#1a1a2e;border-radius:99px;overflow:hidden;}
    .correction-fill{position:absolute;height:100%;background:linear-gradient(90deg,#7f5af0,#e94560);border-radius:99px;transition:all 0.1s;}
    .correction-center{position:absolute;left:50%;top:0;width:1px;height:100%;background:#333;}

    /* BATTERY */
    .batt-row{display:flex;align-items:center;gap:8px;margin-top:6px;}
    .batt-label{font-size:10px;color:#888;flex-shrink:0;}
    .batt-bar-bg{flex:1;height:8px;background:#1a1a2e;border-radius:99px;overflow:hidden;}
    .batt-bar-fill{height:100%;border-radius:99px;transition:width 0.5s,background 0.5s;}
    .batt-val{font-size:10px;color:#ccc;width:38px;text-align:right;flex-shrink:0;}

    /* BUTTONS */
    .btn-row{display:flex;gap:6px;flex-wrap:wrap;}
    .panel-btn{flex:1;min-width:70px;padding:9px 6px;border:none;border-radius:9px;font-size:11px;font-weight:bold;cursor:pointer;text-align:center;}
    .pbtn-cal  {background:#7c3aed;color:white;}
    .pbtn-cal:disabled{opacity:0.5;cursor:not-allowed;}
    .pbtn-sens {background:#0ea5e9;color:white;}
    .pbtn-save {background:#10b981;color:white;}
    .pbtn-reset{background:#374151;color:#f87171;}
    .pbtn-laps {background:#10b981;color:white;}
    .pbtn-lapr {background:#374151;color:#9ca3af;}
    .pbtn-ws   {background:#374151;color:white;}

    /* TOGGLE SWITCHES */
    .toggle-row{display:flex;align-items:center;justify-content:space-between;margin-bottom:6px;}
    .toggle-row:last-child{margin-bottom:0;}
    .toggle-label{font-size:11px;color:#aaa;}
    .toggle-desc{font-size:9px;color:#555;margin-top:1px;}
    .switch{position:relative;width:36px;height:20px;flex-shrink:0;}
    .switch input{opacity:0;width:0;height:0;}
    .slider-sw{position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;background:#333;border-radius:99px;transition:0.3s;}
    .slider-sw:before{position:absolute;content:"";height:14px;width:14px;left:3px;bottom:3px;background:white;border-radius:50%;transition:0.3s;}
    input:checked+.slider-sw{background:#00ff88;}
    input:checked+.slider-sw:before{transform:translateX(16px);}

    /* CALIBRATE STATUS */
    .cal-status{display:none;margin-top:8px;background:rgba(127,90,240,0.15);border:1px solid rgba(127,90,240,0.4);border-radius:8px;padding:8px 10px;}
    .cal-status.active{display:block;}
    .cal-status-text{font-size:11px;color:#c4b5fd;margin-bottom:5px;text-align:center;}
    .cal-bar-bg{background:rgba(255,255,255,0.1);border-radius:99px;height:6px;overflow:hidden;}
    .cal-bar-fill{height:100%;background:linear-gradient(90deg,#7f5af0,#2cb67d);border-radius:99px;width:0%;transition:width 0.3s;}

    /* LAP TABLE */
    .lap-table{width:100%;font-size:11px;display:none;}
    .lap-table.show{display:table;}
    .lap-table td{padding:3px 6px;border-bottom:1px solid #222;}
    .lap-table .best{color:#00ff88;font-weight:bold;}
    .lap-table .avg {color:#f5a623;}
    .lap-empty{font-size:11px;color:#555;text-align:center;padding:6px 0;display:none;}
    .lap-empty.show{display:block;}

    /* STATUS BAR */
    .status-bar{width:100%;max-width:340px;margin-top:6px;background:rgba(255,255,255,0.03);border-radius:8px;padding:6px 12px;font-size:10px;color:#555;text-align:center;}
    .status-bar span{color:#e94560;font-weight:bold;}
  </style>
</head>
<body>
  <h2>🎮 RC CONTROLLER v11</h2>

  <!-- TRACK MODE BADGE -->
  <div class="mode-badge off" id="modeBadge">● LINE TRACKING OFF</div>

  <!-- TOP BAR -->
  <div class="top-bar">
    <button class="top-btn btn-on"  ontouchstart="sc('track on')"  onclick="sc('track on')">▶ TRACK ON</button>
    <button class="top-btn btn-stop"ontouchstart="sc('s')"         onclick="sc('s')">■ STOP</button>
    <button class="top-btn btn-off" ontouchstart="sc('track off')" onclick="sc('track off')">✕ TRACK OFF</button>
  </div>

  <!-- SLIDERS: Speed + Sensitivity -->
  <div class="sliders-wrap">
    <div class="slider-row">
      <div class="slider-label">SPEED <span id="spdLbl">200</span></div>
      <input type="range" min="50" max="255" value="200" id="spSlider" oninput="setSpd(this.value)">
      <div class="slider-val" id="spdVal">200</div>
    </div>
    <div class="slider-row">
      <div class="slider-label">SENS <span id="sensLbl">2000</span></div>
      <input type="range" min="100" max="4000" value="2000" id="sensSlider" oninput="setSens(this.value)">
      <div class="slider-val" id="sensVal">2000</div>
    </div>
    <!-- PID SLIDERS -->
    <div class="slider-row">
      <div class="slider-label">Kp <span id="kpLbl">40</span></div>
      <input type="range" min="0" max="150" value="40" step="1" id="kpSlider" oninput="setKp(this.value)">
      <div class="slider-val" id="kpVal">40</div>
    </div>
    <div class="slider-row">
      <div class="slider-label">Kd <span id="kdLbl">35</span></div>
      <input type="range" min="0" max="100" value="35" step="1" id="kdSlider" oninput="setKd(this.value)">
      <div class="slider-val" id="kdVal">35</div>
    </div>
  </div>

  <!-- JOYSTICK -->
  <div class="joystick-area" id="joyArea">
    <div class="knob" id="knob"></div>
    <span class="dir up">▲</span>
    <span class="dir down">▼</span>
    <span class="dir left">◄</span>
    <span class="dir right">►</span>
  </div>
  <div class="cmd-label" id="cmdLabel">READY</div>

  <!-- PANEL -->
  <div class="panel">

    <!-- SENSOR STATUS + ADC BARS -->
    <div class="section">
      <div class="section-title">📡 Sensor Status</div>
      <div class="sensor-row">
        <div class="sensor-col">
          <div class="sensor-box" id="sL">L</div>
          <div class="adc-bar-bg"><div class="adc-bar-fill" id="barL" style="width:0%"></div></div>
          <div class="sensor-raw" id="rawL">0</div>
          <div class="sensor-label">LEFT</div>
        </div>
        <div class="sensor-col">
          <div class="sensor-box" id="sC">C</div>
          <div class="adc-bar-bg"><div class="adc-bar-fill" id="barC" style="width:0%"></div></div>
          <div class="sensor-raw" id="rawC">0</div>
          <div class="sensor-label">CENTER</div>
        </div>
        <div class="sensor-col">
          <div class="sensor-box" id="sR">R</div>
          <div class="adc-bar-bg"><div class="adc-bar-fill" id="barR" style="width:0%"></div></div>
          <div class="sensor-raw" id="rawR">0</div>
          <div class="sensor-label">RIGHT</div>
        </div>
      </div>
      <!-- PID CORRECTION METER -->
      <div class="correction-wrap">
        <div class="correction-label"><span>◄ LEFT</span><span>PID Correction</span><span>RIGHT ►</span></div>
        <div class="correction-track">
          <div class="correction-fill" id="corrFill" style="left:50%;width:0%"></div>
          <div class="correction-center"></div>
        </div>
      </div>
      <!-- BATTERY -->
      <div class="batt-row">
        <div class="batt-label">🔋 Batt</div>
        <div class="batt-bar-bg"><div class="batt-bar-fill" id="battFill" style="width:0%;background:#00ff88"></div></div>
        <div class="batt-val" id="battVal">--V</div>
      </div>
    </div>

    <!-- TOGGLES -->
    <div class="section">
      <div class="section-title">⚙️ Features</div>
      <div class="toggle-row">
        <div>
          <div class="toggle-label">Adaptive Speed</div>
          <div class="toggle-desc">Bumabagal sa curves</div>
        </div>
        <label class="switch">
          <input type="checkbox" id="adaptToggle" checked onchange="setAdaptive(this.checked)">
          <span class="slider-sw"></span>
        </label>
      </div>
      <div class="toggle-row">
        <div>
          <div class="toggle-label">Reverse Search</div>
          <div class="toggle-desc">Umatras bago mag-spin</div>
        </div>
        <label class="switch">
          <input type="checkbox" id="revToggle" checked onchange="setReverse(this.checked)">
          <span class="slider-sw"></span>
        </label>
      </div>
    </div>

    <!-- TOOLS -->
    <div class="section">
      <div class="section-title">🔧 Tools</div>
      <div class="btn-row" style="margin-bottom:6px;">
        <button class="panel-btn pbtn-cal"  id="calBtn" onclick="startCalibrate()">⚙️ Calibrate</button>
        <button class="panel-btn pbtn-save" onclick="doSave()">💾 Save Settings</button>
        <button class="panel-btn pbtn-reset"onclick="doReset()">🔄 Reset</button>
      </div>
      <div class="btn-row">
        <button class="panel-btn pbtn-ws" onclick="location.href='/webserial'">📟 WebSerial</button>
      </div>
      <!-- Calibrate progress -->
      <div class="cal-status" id="calStatus">
        <div class="cal-status-text" id="calText">Initializing...</div>
        <div class="cal-bar-bg"><div class="cal-bar-fill" id="calBar"></div></div>
      </div>
    </div>

    <!-- LAP TIMES -->
    <div class="section">
      <div class="section-title">🏁 Lap Times</div>
      <div class="btn-row" style="margin-bottom:8px;">
        <button class="panel-btn pbtn-laps" onclick="doLaps()">📋 Show Laps</button>
        <button class="panel-btn pbtn-lapr" onclick="doLapReset()">🔄 Reset</button>
      </div>
      <div class="lap-empty show" id="lapEmpty">No laps recorded yet.</div>
      <table class="lap-table" id="lapTable"><tbody id="lapBody"></tbody></table>
    </div>

    <!-- COMMANDS REF -->
    <div class="section">
      <div class="section-title">📋 WebSerial Commands</div>
      <div style="font-size:10px;color:#555;line-height:1.9;">
        <span style="color:#e94560">track on/off</span> — line following<br>
        <span style="color:#e94560">f b l r s</span> — manual move<br>
        <span style="color:#e94560">calibrate</span> — auto threshold<br>
        <span style="color:#e94560">sensor test</span> — raw values<br>
        <span style="color:#e94560">set speed/threshold/kp/kd</span> — tune<br>
        <span style="color:#e94560">save</span> — save all settings<br>
        <span style="color:#e94560">reset settings</span> — factory defaults<br>
        <span style="color:#e94560">laps / lap reset</span> — lap timer<br>
        <span style="color:#e94560">test left/right/all</span> — motor test<br>
        <span style="color:#e94560">set ssid / set pass</span> — WiFi<br>
        <span style="color:#e94560">control</span> — this page
      </div>
    </div>

  </div>

  <div class="status-bar" id="statusBar">Ready — <span>192.168.4.1</span></div>

<script>
  const joyArea=document.getElementById('joyArea');
  const knob=document.getElementById('knob');
  const cmdLabel=document.getElementById('cmdLabel');
  const RADIUS=62,CENTER=95;
  let isDragging=false,currentCmd='';

  function sc(cmd){fetch('/cmd?v='+encodeURIComponent(cmd));}
  function setStatus(msg){document.getElementById('statusBar').innerHTML=msg;}

  function setSpd(v){document.getElementById('spdLbl').innerText=v;document.getElementById('spdVal').innerText=v;sc('set speed '+v);}
  function setSens(v){document.getElementById('sensLbl').innerText=v;document.getElementById('sensVal').innerText=v;sc('set threshold '+v);}
  function setKp(v){document.getElementById('kpLbl').innerText=v;document.getElementById('kpVal').innerText=v;sc('set kp '+v);}
  function setKd(v){document.getElementById('kdLbl').innerText=v;document.getElementById('kdVal').innerText=v;sc('set kd '+v);}
  function setAdaptive(v){sc('set adaptive '+(v?'1':'0'));}
  function setReverse(v){sc('set reverse '+(v?'1':'0'));}
  function doSave(){sc('save');setStatus('💾 <span>Settings saved!</span>');}
  function doReset(){if(confirm('Reset to factory defaults?')){sc('reset settings');setStatus('🔄 <span>Settings reset!</span>');location.reload();}}

  // ---- CALIBRATE ----
  let calPolling=null;
  function startCalibrate(){
    document.getElementById('calBtn').disabled=true;
    document.getElementById('calStatus').classList.add('active');
    document.getElementById('calText').innerText='Starting...';
    document.getElementById('calBar').style.width='0%';
    sc('calibrate');
    calPolling=setInterval(pollCalStatus,500);
  }
  function pollCalStatus(){
    fetch('/calstatus').then(r=>r.json()).then(d=>{
      document.getElementById('calText').innerText=d.msg;
      document.getElementById('calBar').style.width=d.progress+'%';
      if(d.state===5){
        clearInterval(calPolling);
        setTimeout(()=>{
          document.getElementById('calStatus').classList.remove('active');
          document.getElementById('calBtn').disabled=false;
          const nt=parseInt(d.msg.split('=')[1])||2000;
          document.getElementById('sensSlider').value=nt;
          document.getElementById('sensLbl').innerText=nt;
          document.getElementById('sensVal').innerText=nt;
          setStatus('✅ Calibrated! Threshold=<span>'+nt+'</span>');
        },2500);
      } else if(d.state===0&&d.msg==='Ready'){
        clearInterval(calPolling);
        document.getElementById('calStatus').classList.remove('active');
        document.getElementById('calBtn').disabled=false;
      }
    }).catch(()=>{});
  }

  // ---- TELEMETRY POLL every 300ms ----
  setInterval(()=>{
    fetch('/telemetry').then(r=>r.json()).then(d=>{
      // Sensor boxes
      document.getElementById('sL').className='sensor-box '+(d.L?'on':'');
      document.getElementById('sC').className='sensor-box '+(d.C?'on':'');
      document.getElementById('sR').className='sensor-box '+(d.R?'on':'');
      // ADC bars (0-4095)
      document.getElementById('barL').style.width=(d.rawL/4095*100)+'%';
      document.getElementById('barC').style.width=(d.rawC/4095*100)+'%';
      document.getElementById('barR').style.width=(d.rawR/4095*100)+'%';
      // Raw values
      document.getElementById('rawL').innerText=d.rawL;
      document.getElementById('rawC').innerText=d.rawC;
      document.getElementById('rawR').innerText=d.rawR;
      // PID correction meter
      const cor=d.correction; // -255 to 255
      const pct=Math.abs(cor)/255*50;
      const fill=document.getElementById('corrFill');
      if(cor>=0){fill.style.left='50%';fill.style.width=pct+'%';}
      else{fill.style.left=(50-pct)+'%';fill.style.width=pct+'%';}
      // Track mode badge
      const badge=document.getElementById('modeBadge');
      if(d.tracking){badge.className='mode-badge on';badge.innerText='● LINE TRACKING ON';}
      else{badge.className='mode-badge off';badge.innerText='● LINE TRACKING OFF';}
      // Battery
      const bv=d.batt;
      const bp=Math.min(100,Math.max(0,(bv-3.0)/(4.2-3.0)*100));
      document.getElementById('battFill').style.width=bp+'%';
      document.getElementById('battFill').style.background=bp>50?'#00ff88':bp>20?'#f5a623':'#e94560';
      document.getElementById('battVal').innerText=bv.toFixed(1)+'V';
      // Sync sliders if changed from WebSerial
      document.getElementById('spSlider').value=d.speed;
      document.getElementById('spdLbl').innerText=d.speed;
      document.getElementById('spdVal').innerText=d.speed;
      document.getElementById('kpSlider').value=d.kp;
      document.getElementById('kpLbl').innerText=d.kp;
      document.getElementById('kpVal').innerText=d.kp;
      document.getElementById('kdSlider').value=d.kd;
      document.getElementById('kdLbl').innerText=d.kd;
      document.getElementById('kdVal').innerText=d.kd;
      document.getElementById('adaptToggle').checked=d.adaptive===1;
      document.getElementById('revToggle').checked=d.reverse===1;
    }).catch(()=>{});
  },300);

  // ---- LAP TIMES ----
  function doLaps(){
    fetch('/lapdata').then(r=>r.json()).then(d=>{
      const tbody=document.getElementById('lapBody');
      const empty=document.getElementById('lapEmpty');
      const table=document.getElementById('lapTable');
      tbody.innerHTML='';
      if(d.count===0){empty.className='lap-empty show';table.className='lap-table';return;}
      empty.className='lap-empty';table.className='lap-table show';
      for(let i=0;i<d.laps.length;i++){
        const tr=document.createElement('tr');
        tr.innerHTML='<td style="color:#666">Lap '+(i+1)+'</td><td style="color:white;text-align:right">'+d.laps[i]+' s</td>';
        tbody.appendChild(tr);
      }
      if(d.best!==undefined){
        tbody.innerHTML+='<tr><td class="best">🏆 Best</td><td class="best" style="text-align:right">'+d.best+' s</td></tr>';
        tbody.innerHTML+='<tr><td class="avg">⌀ Avg</td><td class="avg" style="text-align:right">'+d.avg+' s</td></tr>';
      }
      setStatus('🏁 <span>'+d.count+' lap(s)</span> recorded');
    }).catch(()=>setStatus('❌ Lap data failed'));
  }
  function doLapReset(){
    sc('lap reset');
    document.getElementById('lapBody').innerHTML='';
    document.getElementById('lapTable').className='lap-table';
    document.getElementById('lapEmpty').className='lap-empty show';
    setStatus('🔄 Lap timer reset.');
  }

  // ---- JOYSTICK ----
  function getPos(e){const r=joyArea.getBoundingClientRect();const t=e.touches?e.touches[0]:e;return{x:t.clientX-r.left-CENTER,y:t.clientY-r.top-CENTER};}
  function updateJoy(x,y){
    const dist=Math.sqrt(x*x+y*y);const cl=Math.min(dist,RADIUS);const ang=Math.atan2(y,x);
    knob.style.left=(CENTER+Math.cos(ang)*cl)+'px';knob.style.top=(CENTER+Math.sin(ang)*cl)+'px';knob.style.transform='translate(-50%,-50%)';
    let cmd='';
    if(dist<20){cmd='s';cmdLabel.innerText='■ STOP';}
    else{
      const deg=ang*180/Math.PI;
      if(deg>=-45&&deg<45){cmd='r';cmdLabel.innerText='► RIGHT';}
      else if(deg>=45&&deg<135){cmd='b';cmdLabel.innerText='▼ BACKWARD';}
      else if(deg>=135||deg<-135){cmd='l';cmdLabel.innerText='◄ LEFT';}
      else{cmd='f';cmdLabel.innerText='▲ FORWARD';}
    }
    if(cmd!==currentCmd){currentCmd=cmd;sc(cmd);}
  }
  function resetKnob(){knob.style.left='50%';knob.style.top='50%';knob.style.transform='translate(-50%,-50%)';cmdLabel.innerText='■ STOP';sc('s');currentCmd='';}
  joyArea.addEventListener('touchstart',e=>{e.preventDefault();isDragging=true;const p=getPos(e);updateJoy(p.x,p.y);},{passive:false});
  joyArea.addEventListener('touchmove', e=>{e.preventDefault();if(isDragging){const p=getPos(e);updateJoy(p.x,p.y);}},{passive:false});
  joyArea.addEventListener('touchend',  e=>{e.preventDefault();isDragging=false;resetKnob();},{passive:false});
  joyArea.addEventListener('mousedown', e=>{isDragging=true;const p=getPos(e);updateJoy(p.x,p.y);});
  document.addEventListener('mousemove',e=>{if(!isDragging)return;const r=joyArea.getBoundingClientRect();updateJoy(e.clientX-r.left-CENTER,e.clientY-r.top-CENTER);});
  document.addEventListener('mouseup',()=>{if(isDragging){isDragging=false;resetKnob();}});
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
// HANDLE COMMANDS
// -------------------------------------------------------
void handleCmd(AsyncWebServerRequest *request) {
  if (request->hasParam("v")) {
    String cmd=request->getParam("v")->value();
    cmd.trim(); cmd.toLowerCase();
    if      (cmd=="f") { lineTrackingMode=false; moveForward(); }
    else if (cmd=="b") { lineTrackingMode=false; moveBackward(); }
    else if (cmd=="l") { lineTrackingMode=false; setLeftMotor(1,BASE_SPEED); setRightMotor(0,0); }
    else if (cmd=="r") { lineTrackingMode=false; setLeftMotor(0,0); setRightMotor(1,BASE_SPEED); }
    else if (cmd=="s") { lineTrackingMode=false; stopMotors(); }
    else if (cmd=="track on")  { lostTime=0; lastError=0; integral=0; lastPID=0; lastCorrection=0; lineTrackingMode=true; }
    else if (cmd=="track off") { lineTrackingMode=false; stopMotors(); }
    else if (cmd=="calibrate") { lineTrackingMode=false; stopMotors(); autoCalibrate(); }
    else if (cmd=="save")      { saveSettings(); }
    else if (cmd=="reset settings") { resetSettings(); }
    else if (cmd=="lap reset") { lapTimerActive=false; lapCount=0; wasOnStart=false; memset(lapTimes,0,sizeof(lapTimes)); }
    else if (cmd.startsWith("set speed "))     { int s=cmd.substring(10).toInt(); if(s>=50&&s<=255){BASE_SPEED=s;HARD_TURN_SPEED=constrain(s+30,0,255);} }
    else if (cmd.startsWith("set threshold ")) { int t=cmd.substring(14).toInt(); if(t>=100&&t<=4000)SENSOR_THRESHOLD=t; }
    else if (cmd.startsWith("set kp "))        { float v=cmd.substring(7).toFloat(); if(v>=0&&v<=150)Kp=v; }
    else if (cmd.startsWith("set kd "))        { float v=cmd.substring(7).toFloat(); if(v>=0&&v<=100)Kd=v; }
    else if (cmd=="set adaptive 1")            { adaptiveSpeed=true; }
    else if (cmd=="set adaptive 0")            { adaptiveSpeed=false; }
    else if (cmd=="set reverse 1")             { reverseSearch=true; }
    else if (cmd=="set reverse 0")             { reverseSearch=false; }
  }
  request->send(200,"text/plain","OK");
}

// -------------------------------------------------------
// ENDPOINTS
// -------------------------------------------------------
void handleTelemetryRoute(AsyncWebServerRequest *request) { handleTelemetry(request); }

void handleLapData(AsyncWebServerRequest *request) {
  request->send(200,"application/json",getLapTimesJSON());
}

void handleCalStatus(AsyncWebServerRequest *request) {
  String json="{\"state\":"+String(calState)+",\"msg\":\""+calStatusMsg+"\",\"progress\":"+String(calProgress)+",\"threshold\":"+String(SENSOR_THRESHOLD)+"}";
  request->send(200,"application/json",json);
}

// -------------------------------------------------------
// WEBSERIAL
// -------------------------------------------------------
void recvMsg(uint8_t *data, size_t len) {
  String message="";
  for (int i=0;i<len;i++) message+=char(data[i]);
  message.trim(); message.toLowerCase();

  if (message=="control")    { WebSerial.println("[GAMEPAD] >> http://192.168.4.1/control <<"); }
  else if (message=="track on") {
    lostTime=0;lastError=0;integral=0;lastPID=0;lastCorrection=0;
    lapTimerActive=false;lapCount=0;wasOnStart=false;lineTrackingMode=true;
    WebSerial.println("[OK] LINE TRACKING ON");
    WebSerial.print("  Speed=");WebSerial.print(BASE_SPEED);WebSerial.print(" Kp=");WebSerial.print(Kp);WebSerial.print(" Kd=");WebSerial.print(Kd);WebSerial.print(" Thresh=");WebSerial.println(SENSOR_THRESHOLD);
  }
  else if (message=="track off")     { lineTrackingMode=false; stopMotors(); WebSerial.println("[OK] TRACK OFF"); }
  else if (message=="calibrate")     { lineTrackingMode=false; stopMotors(); autoCalibrate(); }
  else if (message=="save")          { saveSettings(); }
  else if (message=="reset settings"){ resetSettings(); WebSerial.println("[RESET] Done."); }
  else if (message=="laps")          { printLapTimes(); }
  else if (message=="lap reset")     { lapTimerActive=false;lapCount=0;wasOnStart=false;memset(lapTimes,0,sizeof(lapTimes));WebSerial.println("[LAP] Reset."); }
  else if (message=="sensor test") {
    int rL=analogRead(IR_LEFT),rC=analogRead(IR_CENTER),rR=analogRead(IR_RIGHT);
    WebSerial.println("=== SENSOR READINGS ===");
    WebSerial.print("  L=");WebSerial.print(rL);WebSerial.print(rL<SENSOR_THRESHOLD?" [LINE]":" [white]");
    WebSerial.print("  C=");WebSerial.print(rC);WebSerial.print(rC<SENSOR_THRESHOLD?" [LINE]":" [white]");
    WebSerial.print("  R=");WebSerial.print(rR);WebSerial.println(rR<SENSOR_THRESHOLD?" [LINE]":" [white]");
    WebSerial.print("  Threshold=");WebSerial.println(SENSOR_THRESHOLD);
  }
  else if (message.startsWith("set threshold ")) { int t=message.substring(14).toInt(); if(t>=0&&t<=4095){SENSOR_THRESHOLD=t;WebSerial.print("[OK] Thresh=");WebSerial.println(t);} else WebSerial.println("[ERR] 0-4095"); }
  else if (message.startsWith("set kp "))        { Kp=message.substring(7).toFloat(); WebSerial.print("[OK] Kp=");WebSerial.println(Kp); }
  else if (message.startsWith("set kd "))        { Kd=message.substring(7).toFloat(); WebSerial.print("[OK] Kd=");WebSerial.println(Kd); }
  else if (message.startsWith("set speed "))     { int s=message.substring(10).toInt(); if(s>=0&&s<=255){BASE_SPEED=s;HARD_TURN_SPEED=constrain(s+30,0,255);WebSerial.print("[OK] Speed=");WebSerial.println(s);}else WebSerial.println("[ERR] 0-255"); }
  else if (message=="speed")     { WebSerial.print("[INFO] Speed=");WebSerial.println(BASE_SPEED); }
  else if (message=="threshold") { WebSerial.print("[INFO] Thresh=");WebSerial.println(SENSOR_THRESHOLD); }
  else if (message=="pid")       { WebSerial.print("[PID] Kp=");WebSerial.print(Kp);WebSerial.print(" Ki=");WebSerial.print(Ki);WebSerial.print(" Kd=");WebSerial.println(Kd); }
  else if (message=="forward"||message=="f")  { lineTrackingMode=false;moveForward();WebSerial.println("[OK] FORWARD");delay(2000);stopMotors(); }
  else if (message=="backward"||message=="b") { lineTrackingMode=false;moveBackward();WebSerial.println("[OK] BACKWARD");delay(2000);stopMotors(); }
  else if (message=="left"||message=="l")     { lineTrackingMode=false;setLeftMotor(1,BASE_SPEED);setRightMotor(0,0);WebSerial.println("[OK] LEFT");delay(2000);stopMotors(); }
  else if (message=="right"||message=="r")    { lineTrackingMode=false;setLeftMotor(0,0);setRightMotor(1,BASE_SPEED);WebSerial.println("[OK] RIGHT");delay(2000);stopMotors(); }
  else if (message=="stop"||message=="s")     { lineTrackingMode=false;stopMotors();WebSerial.println("[OK] STOPPED"); }
  else if (message=="test left")  { lineTrackingMode=false;WebSerial.println("[TEST] LEFT 2sec...");setLeftMotor(1,BASE_SPEED);setRightMotor(0,0);delay(2000);stopMotors();WebSerial.println("[TEST] Done."); }
  else if (message=="test right") { lineTrackingMode=false;WebSerial.println("[TEST] RIGHT 2sec...");setLeftMotor(0,0);setRightMotor(1,BASE_SPEED);delay(2000);stopMotors();WebSerial.println("[TEST] Done."); }
  else if (message=="test all")   { lineTrackingMode=false;WebSerial.println("[TEST] Both 2sec...");moveForward();delay(2000);stopMotors();WebSerial.println("[TEST] Done."); }
  else if (message.startsWith("set ssid ")) { ssid=message.substring(9);preferences.putString("ssid",ssid);WebSerial.println("[OK] SSID updated. Restarting...");delay(1000);ESP.restart(); }
  else if (message.startsWith("set pass ")) { password=message.substring(9);if(password.length()<8){WebSerial.println("[ERR] Min 8 chars.");return;}preferences.putString("pass",password);WebSerial.println("[OK] Pass updated. Restarting...");delay(1000);ESP.restart(); }
  else {
    WebSerial.println("[?] Commands:");
    WebSerial.println("  f b l r s | track on/off | sensor test");
    WebSerial.println("  calibrate | laps | lap reset | control");
    WebSerial.println("  set speed/threshold/kp/kd <val>");
    WebSerial.println("  save | reset settings | pid");
    WebSerial.println("  test left/right/all | set ssid/pass");
  }
}

// -------------------------------------------------------
// SETUP
// -------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(LEFT_IN1,OUTPUT);pinMode(LEFT_IN2,OUTPUT);pinMode(LEFT_EN,OUTPUT);
  pinMode(RIGHT_IN3,OUTPUT);pinMode(RIGHT_IN4,OUTPUT);pinMode(RIGHT_EN,OUTPUT);
  stopMotors();
  pinMode(IR_LEFT,INPUT);pinMode(IR_CENTER,INPUT);pinMode(IR_RIGHT,INPUT);

  preferences.begin("wifi-config",false);
  ssid     = preferences.getString("ssid","ESP32_RC");
  password = preferences.getString("pass","12345678");
  preferences.end();

  loadSettings();
  startAP();

  server.on("/",          HTTP_GET,[](AsyncWebServerRequest *r){r->redirect("/webserial");});
  server.on("/control",   HTTP_GET,[](AsyncWebServerRequest *r){r->send_P(200,"text/html",GAMEPAD_HTML);});
  server.on("/cmd",       HTTP_GET,handleCmd);
  server.on("/telemetry", HTTP_GET,handleTelemetryRoute);
  server.on("/lapdata",   HTTP_GET,handleLapData);
  server.on("/calstatus", HTTP_GET,handleCalStatus);
  server.onNotFound([](AsyncWebServerRequest *r){r->redirect("/webserial");});

  WebSerial.begin(&server);
  WebSerial.onMessage(recvMsg);
  server.begin();

  Serial.println("Ready! RC_CODEv11");
  Serial.println("WebSerial: http://192.168.4.1/webserial");
  Serial.println("Gamepad:   http://192.168.4.1/control");
}

// -------------------------------------------------------
// LOOP
// -------------------------------------------------------
void loop() {
  dnsServer.processNextRequest();
  if (lineTrackingMode) lineTrack();
}
