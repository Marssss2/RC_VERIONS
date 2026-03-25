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
  int calWhiteL=wL/50, calWhiteC=wC/50, calWhiteR=wR/50;
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
  int calBlackL=bL/50, calBlackC=bC/50, calBlackR=bR/50;
  WebSerial.print("[CAL] Black: L="); WebSerial.print(calBlackL);
  WebSerial.print(" C="); WebSerial.print(calBlackC);
  WebSerial.print(" R="); WebSerial.println(calBlackR);

  int avgWhite = (calWhiteL+calWhiteC+calWhiteR)/3;
  int avgBlack = (calBlackL+calBlackC+calBlackR)/3;
  SENSOR_THRESHOLD = (avgWhite+avgBlack)/2;
  WebSerial.print("[CAL] Auto threshold set to: "); WebSerial.println(SENSOR_THRESHOLD);
  WebSerial.println("[CAL] Calibration complete!");
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
      WebSerial.print(": "); WebSerial.print(lapTime/1000.0, 2);
      WebSerial.println(" sec");
    }
  }
  wasOnStart = onStart;
}

String getLapTimesJSON() {
  String json = "{\"count\":" + String(lapCount) + ",\"laps\":[";
  int show = min(lapCount, 10);
  unsigned long best = 999999, total = 0;
  for (int i=0; i<show; i++) {
    if (i > 0) json += ",";
    json += String(lapTimes[i]/1000.0, 2);
    if (lapTimes[i] < best) best = lapTimes[i];
    total += lapTimes[i];
  }
  json += "]";
  if (show > 0) {
    json += ",\"best\":" + String(best/1000.0, 2);
    json += ",\"avg\":"  + String((total/show)/1000.0, 2);
  }
  json += "}";
  return json;
}

void printLapTimes() {
  if (lapCount == 0) { WebSerial.println("[LAP] No laps recorded yet."); return; }
  WebSerial.println("=== LAP TIMES ===");
  unsigned long best=999999, total=0;
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
  if (direction == 1)       { digitalWrite(LEFT_IN1,HIGH); digitalWrite(LEFT_IN2,LOW); }
  else if (direction == -1) { digitalWrite(LEFT_IN1,LOW);  digitalWrite(LEFT_IN2,HIGH); }
  else                      { digitalWrite(LEFT_IN1,LOW);  digitalWrite(LEFT_IN2,LOW); }
  analogWrite(LEFT_EN, (direction==0)?0:speed);
}

void setRightMotor(int direction, int speed) {
  speed = constrain(speed, 0, 255);
  if (direction == 1)       { digitalWrite(RIGHT_IN3,HIGH); digitalWrite(RIGHT_IN4,LOW); }
  else if (direction == -1) { digitalWrite(RIGHT_IN3,LOW);  digitalWrite(RIGHT_IN4,HIGH); }
  else                      { digitalWrite(RIGHT_IN3,LOW);  digitalWrite(RIGHT_IN4,LOW); }
  analogWrite(RIGHT_EN, (direction==0)?0:speed);
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
  int total = sL+sC+sR;

  // End of line FIRST
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

  checkLap(sL, sC, sR);

  if (total == 0) {
    if (lostTime == 0) lostTime = millis();
    if (millis()-lostTime > LOST_TIMEOUT) {
      stopMotors();
      lineTrackingMode = false;
      WebSerial.println("[TRACK] Line lost! Stopping.");
      return;
    }
    if (lastError >= 0) { setLeftMotor(1,HARD_TURN_SPEED); setRightMotor(1,HARD_INNER); }
    else                { setLeftMotor(1,HARD_INNER); setRightMotor(1,HARD_TURN_SPEED); }
    return;
  }

  lostTime = 0;
  float position = ((float)(sL*-1)+(float)(sC*0)+(float)(sR*1))/total;
  lastError = (position>0)?1:(position<0)?-1:0;

  unsigned long now = millis();
  float dt = (now-lastPIDTime)/1000.0;
  if (dt<=0||dt>0.5) dt=0.01;
  lastPIDTime = now;

  float error      = position;
  integral        += error*dt;
  integral         = constrain(integral,-1.0,1.0);
  float derivative = (error-lastPID)/dt;
  lastPID          = error;

  float correction = (Kp*error)+(Ki*integral)+(Kd*derivative);
  correction = constrain(correction,-255,255);

  setLeftMotor(1,  constrain(BASE_SPEED+(int)correction, 0,255));
  setRightMotor(1, constrain(BASE_SPEED-(int)correction, 0,255));
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
  <title>ESP32 RC Controller</title>
  <style>
    *{margin:0;padding:0;box-sizing:border-box;-webkit-user-select:none;user-select:none;}
    body{background:#0f0f1a;color:white;font-family:Arial,sans-serif;
      display:flex;flex-direction:column;align-items:center;
      min-height:100vh;padding:10px 10px 20px;overflow-y:auto;touch-action:pan-y;}
    h2{color:#e94560;font-size:15px;letter-spacing:3px;margin-bottom:8px;}

    /* TOP BAR */
    .top-bar{display:flex;gap:6px;margin-bottom:8px;flex-wrap:wrap;justify-content:center;}
    .top-btn{padding:9px 12px;border:none;border-radius:10px;font-size:11px;font-weight:bold;cursor:pointer;}
    .btn-on  {background:#00ff88;color:#0f0f1a;}
    .btn-off {background:#e94560;color:white;}
    .btn-stop{background:#f5a623;color:#0f0f1a;}
    .btn-ws  {background:#444;color:white;}

    /* SPEED */
    .speed-wrap{width:260px;margin-bottom:8px;text-align:center;}
    .speed-label{font-size:11px;color:#888;margin-bottom:3px;}
    input[type=range]{width:100%;accent-color:#e94560;cursor:pointer;}

    /* JOYSTICK */
    .joystick-area{
      position:relative;width:190px;height:190px;
      background:rgba(255,255,255,0.04);border-radius:50%;
      border:2px solid rgba(233,69,96,0.3);touch-action:none;flex-shrink:0;}
    .knob{
      width:66px;height:66px;
      background:radial-gradient(circle at 35% 35%,#e94560,#8b0000);
      border-radius:50%;position:absolute;top:50%;left:50%;
      transform:translate(-50%,-50%);
      box-shadow:0 0 20px rgba(233,69,96,0.6);pointer-events:none;}
    .dir{position:absolute;color:rgba(255,255,255,0.25);font-size:11px;}
    .dir.up   {top:6px;   left:50%;transform:translateX(-50%);}
    .dir.down {bottom:6px;left:50%;transform:translateX(-50%);}
    .dir.left {left:6px;  top:50%; transform:translateY(-50%);}
    .dir.right{right:6px; top:50%;transform:translateY(-50%);}
    .cmd-label{margin-top:8px;font-size:13px;color:#e94560;letter-spacing:2px;font-weight:bold;height:18px;}

    /* BOTTOM PANEL */
    .panel{width:100%;max-width:340px;margin-top:12px;display:flex;flex-direction:column;gap:8px;}

    /* SECTION */
    .section{background:rgba(255,255,255,0.04);border-radius:12px;padding:10px 12px;}
    .section-title{font-size:10px;color:#888;letter-spacing:2px;margin-bottom:7px;text-transform:uppercase;}

    /* SECTION BUTTONS */
    .btn-row{display:flex;gap:6px;flex-wrap:wrap;}
    .panel-btn{
      flex:1;min-width:70px;padding:9px 6px;border:none;border-radius:9px;
      font-size:11px;font-weight:bold;cursor:pointer;text-align:center;}
    .pbtn-cal  {background:#7c3aed;color:white;}
    .pbtn-sens {background:#0ea5e9;color:white;}
    .pbtn-laps {background:#10b981;color:white;}
    .pbtn-lapr {background:#374151;color:#9ca3af;}
    .pbtn-ws   {background:#374151;color:white;}

    /* SENSOR STATUS */
    .sensor-row{display:flex;gap:8px;justify-content:center;margin-top:4px;}
    .sensor-box{
      flex:1;padding:7px 4px;border-radius:8px;text-align:center;
      font-size:10px;font-weight:bold;background:#1a1a2e;
      border:1px solid #333;transition:background 0.2s,color 0.2s;}
    .sensor-box.on {background:#e94560;color:white;border-color:#e94560;}
    .sensor-label{font-size:9px;color:#666;margin-top:2px;}

    /* LAP TABLE */
    .lap-table{width:100%;font-size:11px;display:none;}
    .lap-table.show{display:table;}
    .lap-table td{padding:3px 6px;border-bottom:1px solid #222;}
    .lap-table .best{color:#00ff88;font-weight:bold;}
    .lap-table .avg {color:#f5a623;}
    .lap-empty{font-size:11px;color:#555;text-align:center;padding:6px 0;display:none;}
    .lap-empty.show{display:block;}

    /* STATUS BAR */
    .status-bar{
      width:100%;max-width:340px;margin-top:6px;
      background:rgba(255,255,255,0.03);border-radius:8px;
      padding:6px 12px;font-size:10px;color:#555;text-align:center;}
    .status-bar span{color:#e94560;font-weight:bold;}
  </style>
</head>
<body>
  <h2>🎮 RC CONTROLLER</h2>

  <!-- TOP BAR -->
  <div class="top-bar">
    <button class="top-btn btn-on"  ontouchstart="sc('track on')"  onclick="sc('track on')">▶ TRACK ON</button>
    <button class="top-btn btn-stop"ontouchstart="sc('s')"         onclick="sc('s')">■ STOP</button>
    <button class="top-btn btn-off" ontouchstart="sc('track off')" onclick="sc('track off')">✕ TRACK OFF</button>
  </div>

  <!-- SPEED -->
  <div class="speed-wrap">
    <div class="speed-label">SPEED: <span id="spd">200</span></div>
    <input type="range" min="50" max="255" value="200" oninput="setSpd(this.value)">
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

  <!-- BOTTOM PANEL -->
  <div class="panel">

    <!-- SENSOR STATUS -->
    <div class="section">
      <div class="section-title">📡 Sensor Status</div>
      <div class="sensor-row">
        <div>
          <div class="sensor-box" id="sL">L</div>
          <div class="sensor-label" style="text-align:center">LEFT</div>
        </div>
        <div>
          <div class="sensor-box" id="sC">C</div>
          <div class="sensor-label" style="text-align:center">CENTER</div>
        </div>
        <div>
          <div class="sensor-box" id="sR">R</div>
          <div class="sensor-label" style="text-align:center">RIGHT</div>
        </div>
      </div>
    </div>

    <!-- TOOLS -->
    <div class="section">
      <div class="section-title">🔧 Tools</div>
      <div class="btn-row">
        <button class="panel-btn pbtn-cal"  onclick="doCalibrate()">⚙️ Calibrate</button>
        <button class="panel-btn pbtn-sens" onclick="doSensorTest()">📡 Sensor Test</button>
        <button class="panel-btn pbtn-ws"   onclick="location.href='/webserial'">📟 WebSerial</button>
      </div>
    </div>

    <!-- LAP TIMES -->
    <div class="section">
      <div class="section-title">🏁 Lap Times</div>
      <div class="btn-row" style="margin-bottom:8px;">
        <button class="panel-btn pbtn-laps" onclick="doLaps()">📋 Show Laps</button>
        <button class="panel-btn pbtn-lapr" onclick="doLapReset()">🔄 Reset Laps</button>
      </div>
      <div class="lap-empty" id="lapEmpty">No laps recorded yet.</div>
      <table class="lap-table" id="lapTable">
        <tbody id="lapBody"></tbody>
      </table>
    </div>

    <!-- COMMANDS LIST -->
    <div class="section">
      <div class="section-title">📋 WebSerial Commands</div>
      <div style="font-size:10px;color:#555;line-height:1.9;">
        <span style="color:#e94560">track on/off</span> — line following<br>
        <span style="color:#e94560">f b l r s</span> — forward/back/left/right/stop<br>
        <span style="color:#e94560">calibrate</span> — auto-detect threshold<br>
        <span style="color:#e94560">sensor test</span> — raw sensor values<br>
        <span style="color:#e94560">set speed &lt;0-255&gt;</span> — change speed<br>
        <span style="color:#e94560">set threshold &lt;0-4095&gt;</span> — manual threshold<br>
        <span style="color:#e94560">set kp/kd &lt;val&gt;</span> — PID tuning<br>
        <span style="color:#e94560">laps / lap reset</span> — lap timer<br>
        <span style="color:#e94560">test left/right/all</span> — motor test<br>
        <span style="color:#e94560">set ssid / set pass</span> — change WiFi<br>
        <span style="color:#e94560">control</span> — open this page link
      </div>
    </div>

  </div>

  <!-- STATUS BAR -->
  <div class="status-bar" id="statusBar">Ready — <span>192.168.4.1</span></div>

<script>
  // ---- Joystick ----
  const joyArea  = document.getElementById('joyArea');
  const knob     = document.getElementById('knob');
  const cmdLabel = document.getElementById('cmdLabel');
  const RADIUS = 62, CENTER = 95;
  let isDragging=false, currentCmd='';

  function sc(cmd){ fetch('/cmd?v='+encodeURIComponent(cmd)); }
  function setSpd(v){ document.getElementById('spd').innerText=v; sc('set speed '+v); }

  function getPos(e){
    const r=joyArea.getBoundingClientRect();
    const t=e.touches?e.touches[0]:e;
    return{x:t.clientX-r.left-CENTER,y:t.clientY-r.top-CENTER};
  }
  function updateJoy(x,y){
    const dist=Math.sqrt(x*x+y*y);
    const cl=Math.min(dist,RADIUS);
    const ang=Math.atan2(y,x);
    knob.style.left=(CENTER+Math.cos(ang)*cl)+'px';
    knob.style.top =(CENTER+Math.sin(ang)*cl)+'px';
    knob.style.transform='translate(-50%,-50%)';
    let cmd='';
    if(dist<20){cmd='s';cmdLabel.innerText='■ STOP';}
    else{
      const deg=ang*180/Math.PI;
      if(deg>=-45&&deg<45)       {cmd='r';cmdLabel.innerText='► RIGHT';}
      else if(deg>=45&&deg<135)  {cmd='b';cmdLabel.innerText='▼ BACKWARD';}
      else if(deg>=135||deg<-135){cmd='l';cmdLabel.innerText='◄ LEFT';}
      else                       {cmd='f';cmdLabel.innerText='▲ FORWARD';}
    }
    if(cmd!==currentCmd){currentCmd=cmd;sc(cmd);}
  }
  function resetKnob(){
    knob.style.left='50%';knob.style.top='50%';
    knob.style.transform='translate(-50%,-50%)';
    cmdLabel.innerText='■ STOP';sc('s');currentCmd='';
  }
  joyArea.addEventListener('touchstart',e=>{e.preventDefault();isDragging=true;const p=getPos(e);updateJoy(p.x,p.y);},{passive:false});
  joyArea.addEventListener('touchmove', e=>{e.preventDefault();if(isDragging){const p=getPos(e);updateJoy(p.x,p.y);}},{passive:false});
  joyArea.addEventListener('touchend',  e=>{e.preventDefault();isDragging=false;resetKnob();},{passive:false});
  joyArea.addEventListener('mousedown', e=>{isDragging=true;const p=getPos(e);updateJoy(p.x,p.y);});
  document.addEventListener('mousemove',e=>{if(!isDragging)return;const r=joyArea.getBoundingClientRect();updateJoy(e.clientX-r.left-CENTER,e.clientY-r.top-CENTER);});
  document.addEventListener('mouseup',  ()=>{if(isDragging){isDragging=false;resetKnob();}});

  // ---- Status bar ----
  function setStatus(msg){document.getElementById('statusBar').innerHTML=msg;}

  // ---- Calibrate ----
  function doCalibrate(){
    sc('calibrate');
    setStatus('⚙️ <span>Calibrating...</span> Check WebSerial for instructions');
  }

  // ---- Sensor Test ----
  function doSensorTest(){
    fetch('/sensordata')
      .then(r=>r.json())
      .then(d=>{
        const bl='on', wh='';
        document.getElementById('sL').className='sensor-box '+(d.L?bl:wh);
        document.getElementById('sC').className='sensor-box '+(d.C?bl:wh);
        document.getElementById('sR').className='sensor-box '+(d.R?bl:wh);
        setStatus('📡 L='+d.rawL+' C='+d.rawC+' R='+d.rawR+' | Thresh='+d.thresh);
      })
      .catch(()=>setStatus('❌ Sensor read failed'));
  }

  // ---- Laps ----
  function doLaps(){
    fetch('/lapdata')
      .then(r=>r.json())
      .then(d=>{
        const tbody=document.getElementById('lapBody');
        const empty=document.getElementById('lapEmpty');
        const table=document.getElementById('lapTable');
        tbody.innerHTML='';
        if(d.count===0){
          empty.className='lap-empty show';
          table.className='lap-table';
          return;
        }
        empty.className='lap-empty';
        table.className='lap-table show';
        for(let i=0;i<d.laps.length;i++){
          const tr=document.createElement('tr');
          tr.innerHTML='<td style="color:#666">Lap '+(i+1)+'</td><td style="color:white;text-align:right">'+d.laps[i]+' s</td>';
          tbody.appendChild(tr);
        }
        if(d.best!==undefined){
          const tr1=document.createElement('tr');
          tr1.innerHTML='<td class="best">🏆 Best</td><td class="best" style="text-align:right">'+d.best+' s</td>';
          tbody.appendChild(tr1);
          const tr2=document.createElement('tr');
          tr2.innerHTML='<td class="avg">⌀ Avg</td><td class="avg" style="text-align:right">'+d.avg+' s</td>';
          tbody.appendChild(tr2);
        }
        setStatus('🏁 <span>'+d.count+' lap(s)</span> recorded');
      })
      .catch(()=>setStatus('❌ Lap data failed'));
  }

  function doLapReset(){
    sc('lap reset');
    document.getElementById('lapBody').innerHTML='';
    document.getElementById('lapTable').className='lap-table';
    document.getElementById('lapEmpty').className='lap-empty show';
    setStatus('🔄 Lap timer reset.');
  }

  // ---- Auto sensor poll every 500ms ----
  setInterval(doSensorTest, 500);
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
    if      (cmd=="f") { lineTrackingMode=false; moveForward(); }
    else if (cmd=="b") { lineTrackingMode=false; moveBackward(); }
    else if (cmd=="l") { lineTrackingMode=false; setLeftMotor(1,BASE_SPEED); setRightMotor(0,0); }
    else if (cmd=="r") { lineTrackingMode=false; setLeftMotor(0,0); setRightMotor(1,BASE_SPEED); }
    else if (cmd=="s") { lineTrackingMode=false; stopMotors(); }
    else if (cmd=="track on")  { lostTime=0; lastError=0; integral=0; lastPID=0; lineTrackingMode=true; }
    else if (cmd=="track off") { lineTrackingMode=false; stopMotors(); }
    else if (cmd=="calibrate") { lineTrackingMode=false; stopMotors(); autoCalibrate(); }
    else if (cmd=="lap reset") {
      lapTimerActive=false; lapCount=0; wasOnStart=false;
      memset(lapTimes,0,sizeof(lapTimes));
    }
    else if (cmd.startsWith("set speed ")) {
      int spd=cmd.substring(10).toInt();
      if (spd>=50&&spd<=255) {
        BASE_SPEED=spd;
        HARD_TURN_SPEED=constrain(spd+30,0,255);
      }
    }
  }
  request->send(200,"text/plain","OK");
}

// -------------------------------------------------------
// SENSOR DATA ENDPOINT → /sensordata
// -------------------------------------------------------
void handleSensorData(AsyncWebServerRequest *request) {
  int rawL = analogRead(IR_LEFT);
  int rawC = analogRead(IR_CENTER);
  int rawR = analogRead(IR_RIGHT);
  String json = "{";
  json += "\"rawL\":" + String(rawL) + ",";
  json += "\"rawC\":" + String(rawC) + ",";
  json += "\"rawR\":" + String(rawR) + ",";
  json += "\"L\":"    + String(rawL < SENSOR_THRESHOLD ? 1 : 0) + ",";
  json += "\"C\":"    + String(rawC < SENSOR_THRESHOLD ? 1 : 0) + ",";
  json += "\"R\":"    + String(rawR < SENSOR_THRESHOLD ? 1 : 0) + ",";
  json += "\"thresh\":" + String(SENSOR_THRESHOLD);
  json += "}";
  request->send(200,"application/json",json);
}

// -------------------------------------------------------
// LAP DATA ENDPOINT → /lapdata
// -------------------------------------------------------
void handleLapData(AsyncWebServerRequest *request) {
  request->send(200,"application/json",getLapTimesJSON());
}

// -------------------------------------------------------
// WEBSERIAL COMMANDS
// -------------------------------------------------------
void recvMsg(uint8_t *data, size_t len) {
  String message="";
  for (int i=0;i<len;i++) message+=char(data[i]);
  message.trim(); message.toLowerCase();

  if (message=="control") {
    WebSerial.println("[GAMEPAD] Opening RC Controller...");
    WebSerial.println("  >> http://192.168.4.1/control <<");
    return;
  }
  else if (message=="track on") {
    lostTime=0; lastError=0; integral=0; lastPID=0;
    lapTimerActive=false; lapCount=0; wasOnStart=false;
    lineTrackingMode=true;
    WebSerial.println("[OK] LINE TRACKING ON");
    WebSerial.print("     Speed: "); WebSerial.println(BASE_SPEED);
    WebSerial.print("     Threshold: "); WebSerial.println(SENSOR_THRESHOLD);
  }
  else if (message=="track off") { lineTrackingMode=false; stopMotors(); WebSerial.println("[OK] TRACK OFF"); }
  else if (message=="calibrate") { lineTrackingMode=false; stopMotors(); autoCalibrate(); }
  else if (message=="laps")      { printLapTimes(); }
  else if (message=="lap reset") {
    lapTimerActive=false; lapCount=0; wasOnStart=false;
    memset(lapTimes,0,sizeof(lapTimes));
    WebSerial.println("[LAP] Timer reset.");
  }
  else if (message=="sensor test") {
    int rL=analogRead(IR_LEFT), rC=analogRead(IR_CENTER), rR=analogRead(IR_RIGHT);
    WebSerial.println("=== SENSOR READINGS ===");
    WebSerial.print("  LEFT   (GPIO34): raw="); WebSerial.print(rL);   WebSerial.print("  → "); WebSerial.println(rL<SENSOR_THRESHOLD?"BLACK LINE":"white");
    WebSerial.print("  CENTER (GPIO35): raw="); WebSerial.print(rC);   WebSerial.print("  → "); WebSerial.println(rC<SENSOR_THRESHOLD?"BLACK LINE":"white");
    WebSerial.print("  RIGHT  (VP/36):  raw="); WebSerial.print(rR);   WebSerial.print("  → "); WebSerial.println(rR<SENSOR_THRESHOLD?"BLACK LINE":"white");
    WebSerial.print("  Threshold: "); WebSerial.println(SENSOR_THRESHOLD);
  }
  else if (message.startsWith("set threshold ")) {
    int t=message.substring(14).toInt();
    if (t<0||t>4095) WebSerial.println("[ERROR] Threshold 0-4095.");
    else { SENSOR_THRESHOLD=t; WebSerial.print("[OK] Threshold: "); WebSerial.println(SENSOR_THRESHOLD); }
  }
  else if (message=="threshold") { WebSerial.print("[INFO] Threshold: "); WebSerial.println(SENSOR_THRESHOLD); }
  else if (message.startsWith("set kp ")) { Kp=message.substring(7).toFloat(); WebSerial.print("[OK] Kp: "); WebSerial.println(Kp); }
  else if (message.startsWith("set kd ")) { Kd=message.substring(7).toFloat(); WebSerial.print("[OK] Kd: "); WebSerial.println(Kd); }
  else if (message.startsWith("set speed ")) {
    int spd=message.substring(10).toInt();
    if (spd<0||spd>255) WebSerial.println("[ERROR] Speed 0-255.");
    else { BASE_SPEED=spd; HARD_TURN_SPEED=constrain(spd+30,0,255); WebSerial.print("[OK] Speed: "); WebSerial.println(BASE_SPEED); }
  }
  else if (message=="speed") { WebSerial.print("[INFO] Speed: "); WebSerial.println(BASE_SPEED); }
  else if (message=="pid")   { WebSerial.print("[PID] Kp="); WebSerial.print(Kp); WebSerial.print(" Ki="); WebSerial.print(Ki); WebSerial.print(" Kd="); WebSerial.println(Kd); }
  else if (message=="forward"||message=="f")  { lineTrackingMode=false; moveForward();  WebSerial.println("[OK] FORWARD");  delay(2000); stopMotors(); }
  else if (message=="backward"||message=="b") { lineTrackingMode=false; moveBackward(); WebSerial.println("[OK] BACKWARD"); delay(2000); stopMotors(); }
  else if (message=="left"||message=="l")     { lineTrackingMode=false; setLeftMotor(1,BASE_SPEED); setRightMotor(0,0); WebSerial.println("[OK] LEFT");  delay(2000); stopMotors(); }
  else if (message=="right"||message=="r")    { lineTrackingMode=false; setLeftMotor(0,0); setRightMotor(1,BASE_SPEED);  WebSerial.println("[OK] RIGHT"); delay(2000); stopMotors(); }
  else if (message=="stop"||message=="s")     { lineTrackingMode=false; stopMotors(); WebSerial.println("[OK] STOPPED"); }
  else if (message=="test left")  { lineTrackingMode=false; WebSerial.println("[TEST] LEFT 2sec..."); setLeftMotor(1,BASE_SPEED); setRightMotor(0,0); delay(2000); stopMotors(); WebSerial.println("[TEST] Done."); }
  else if (message=="test right") { lineTrackingMode=false; WebSerial.println("[TEST] RIGHT 2sec..."); setLeftMotor(0,0); setRightMotor(1,BASE_SPEED); delay(2000); stopMotors(); WebSerial.println("[TEST] Done."); }
  else if (message=="test all")   { lineTrackingMode=false; WebSerial.println("[TEST] Both 2sec..."); moveForward(); delay(2000); stopMotors(); WebSerial.println("[TEST] Done."); }
  else if (message.startsWith("set ssid ")) {
    ssid=message.substring(9);
    preferences.putString("ssid",ssid);
    WebSerial.println("[OK] SSID updated. Restarting..."); delay(1000); ESP.restart();
  }
  else if (message.startsWith("set pass ")) {
    password=message.substring(9);
    if (password.length()<8) { WebSerial.println("[ERROR] Min 8 chars."); return; }
    preferences.putString("pass",password);
    WebSerial.println("[OK] Password updated. Restarting..."); delay(1000); ESP.restart();
  }
  else {
    WebSerial.println("[?] Unknown. Commands:");
    WebSerial.println("  f b l r s | track on/off | sensor test");
    WebSerial.println("  calibrate | laps | lap reset | control");
    WebSerial.println("  set speed <0-255> | set threshold <0-4095>");
    WebSerial.println("  set kp/kd <val> | pid | test left/right/all");
  }
}

// -------------------------------------------------------
// SETUP
// -------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(LEFT_IN1,OUTPUT); pinMode(LEFT_IN2,OUTPUT); pinMode(LEFT_EN,OUTPUT);
  pinMode(RIGHT_IN3,OUTPUT); pinMode(RIGHT_IN4,OUTPUT); pinMode(RIGHT_EN,OUTPUT);
  stopMotors();
  pinMode(IR_LEFT,INPUT); pinMode(IR_CENTER,INPUT); pinMode(IR_RIGHT,INPUT);

  preferences.begin("wifi-config",false);
  ssid     = preferences.getString("ssid","ESP32_RC");
  password = preferences.getString("pass","12345678");

  startAP();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r){ r->redirect("/webserial"); });
  server.on("/control",    HTTP_GET, [](AsyncWebServerRequest *r){ r->send_P(200,"text/html",GAMEPAD_HTML); });
  server.on("/cmd",        HTTP_GET, handleCmd);
  server.on("/sensordata", HTTP_GET, handleSensorData);
  server.on("/lapdata",    HTTP_GET, handleLapData);
  server.onNotFound([](AsyncWebServerRequest *r){ r->redirect("/webserial"); });

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
  if (lineTrackingMode) lineTrack();
}
