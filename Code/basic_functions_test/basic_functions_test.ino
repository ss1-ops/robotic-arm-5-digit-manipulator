// ========================================================
// Arduino Uno R4 WiFi + 3× TB6560 
// Full functionality + Web UI + Telnet
// Improved: goToHomeAll() now runs joints SIMULTANEOUSLY
// ========================================================

#include <AccelStepper.h>
#include <WiFiS3.h>

char ssid[] = "WAVLINK-N";
char pass[] = "Jk12345678";

WiFiServer webServer(80);
WiFiServer telnetServer(23);
WiFiClient telnetClient;

// ─────────────────────────────────────────────
//  PIN ASSIGNMENTS
// ─────────────────────────────────────────────
#define SHOULDER_STEP_PIN  5
#define SHOULDER_DIR_PIN   4
#define ELBOW_STEP_PIN     7
#define ELBOW_DIR_PIN      6
#define WRIST_STEP_PIN     9
#define WRIST_DIR_PIN      8

// ─────────────────────────────────────────────
//  MOTOR PARAMETERS
// ─────────────────────────────────────────────
const float STEPS_PER_REV = 200.0;

// Current driver microstep settings
const float SHOULDER_MICROSTEPS = 16.0;
const float ELBOW_MICROSTEPS    = 4.0;
const float WRIST_MICROSTEPS    = 8.0;

const float SHOULDER_STEPS_PER_REV_TOTAL = STEPS_PER_REV * SHOULDER_MICROSTEPS;
const float ELBOW_STEPS_PER_REV_TOTAL    = STEPS_PER_REV * ELBOW_MICROSTEPS;
const float WRIST_STEPS_PER_REV_TOTAL    = STEPS_PER_REV * WRIST_MICROSTEPS;

// Joint speed ceilings (RPM) - tune up/down based on real motor behavior
const float SHOULDER_MAX_RPM = 12.0;
const float ELBOW_MAX_RPM    = 120.0;
const float WRIST_MAX_RPM    = 16.0;

const float SHOULDER_MAX_SPEED_SPS = (SHOULDER_MAX_RPM / 60.0) * SHOULDER_STEPS_PER_REV_TOTAL;
const float ELBOW_MAX_SPEED_SPS    = (ELBOW_MAX_RPM / 60.0) * ELBOW_STEPS_PER_REV_TOTAL;
const float WRIST_MAX_SPEED_SPS    = (WRIST_MAX_RPM / 60.0) * WRIST_STEPS_PER_REV_TOTAL;

const float NORMAL_ACCEL = 1000.0;
const float SPEED_PROFILE_HEADROOM = 1.20;

// FAST MOVE SETTINGS (Homing + Poses)
const int   FAST_MOVE_LEVEL = 70;
const float FAST_ACCEL      = 1800.0;

// LIMIT BEHAVIOR
const long  LIMIT_BUFFER        = 40;
const int   LIMIT_SLOW_LEVEL    = 20;

// DEFAULT RANGES
const long DEFAULT_SHOULDER_RANGE = 2000;
const long DEFAULT_ELBOW_RANGE    = 6000;
const long DEFAULT_WRIST_RANGE    = 12000;

// SPEED RANGE
const int MAX_SPEED_LEVEL_BEFORE = 10;
const int MAX_SPEED_LEVEL_AFTER  = 100;

// STORED POSES (P1 to P5)
long pose[5][3] = {0};           // [pose][shoulder, elbow, wrist]
bool poseStored[5] = {false};

// STATE
bool calibrated = false;

long shoulderCenter = 0, elbowCenter = 0, wristCenter = 0;
long shoulderMin = -1000, shoulderMax = 1000;
long elbowMin = -8500, elbowMax = 8500;
long wristMin = -12000, wristMax = 12000;

bool homingS = false, homingE = false, homingW = false;

bool limitTriggeredS = false;
bool limitTriggeredE = false;
bool limitTriggeredW = false;

String telnetBuf = "";
String serialBuf = "";

AccelStepper shoulder(AccelStepper::DRIVER, SHOULDER_STEP_PIN, SHOULDER_DIR_PIN);
AccelStepper elbow   (AccelStepper::DRIVER, ELBOW_STEP_PIN,    ELBOW_DIR_PIN);
AccelStepper wrist   (AccelStepper::DRIVER, WRIST_STEP_PIN,    WRIST_DIR_PIN);

// Forward declarations
void stopAll();
void goToHomeAll();
void goToHomeS();
void goToHomeE();
void goToHomeW();
void finishHomingS();
void finishHomingE();
void finishHomingW();
void goToPose(int num);
void doCalibrate();
void resetLimitFlags();
void checkSimpleLimits(AccelStepper &motor, long center, long minRel, long maxRel, bool &limitFlag, const char* name, float maxSpeedSps);
void handleWeb();
void handleTelnet();
void handleSerial();
void serveUI(WiFiClient &client);
void setSpeedFromReq(char joint, String &req);
float jointMaxSpeedSps(char joint);
float levelToJointSpeed(char joint, int level);
void applyDefaultMotionProfiles();
void processCommand(String input, bool toTelnet);
void printStatusWithPoses();
void printLimits();
void setLimit(char jointChar, bool isMax);

// ====================== SETUP ======================
void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("IP: http://");
  Serial.println(WiFi.localIP());

  webServer.begin();
  telnetServer.begin();
  Serial.println("Web + Telnet started. High Speed Mode (20×).\n");

  Serial.println(F("=== 3-JOINT ARM - FULL FUNCTIONALITY ==="));
  Serial.println(F("Commands: S<n> E<n> W<n> | HOME | H | HS | HE | HW | STORE P1..P5 | P1..P5 | MAX S | MIN S | P | 0"));

  applyDefaultMotionProfiles();

  stopAll();
}

// ====================== LOOP ======================
void loop() {
  bool motorsActive = (shoulder.speed() != 0) || (elbow.speed() != 0) || (wrist.speed() != 0);

  static unsigned long lastNetwork = 0;
  unsigned long interval = motorsActive ? 20 : 5;

  if (millis() - lastNetwork >= interval) {
    handleWeb();
    handleTelnet();
    handleSerial();
    lastNetwork = millis();
  }

  // Limits
  checkSimpleLimits(shoulder, shoulderCenter, shoulderMin, shoulderMax, limitTriggeredS, "Shoulder", SHOULDER_MAX_SPEED_SPS);
  checkSimpleLimits(elbow,    elbowCenter,    elbowMin,    elbowMax,    limitTriggeredE, "Elbow",    ELBOW_MAX_SPEED_SPS);
  checkSimpleLimits(wrist,    wristCenter,    wristMin,    wristMax,    limitTriggeredW, "Wrist",    WRIST_MAX_SPEED_SPS);

  // === SIMULTANEOUS MOTOR CONTROL ===
  if (homingS || homingE || homingW) {
    // Homing mode: use .run() for acceleration + position targeting
    if (homingS) {
      shoulder.run();
      if (shoulder.distanceToGo() == 0) finishHomingS();
    }
    if (homingE) {
      elbow.run();
      if (elbow.distanceToGo() == 0) finishHomingE();
    }
    if (homingW) {
      wrist.run();
      if (wrist.distanceToGo() == 0) finishHomingW();
    }
  } 
  else {
    // Normal continuous speed mode
    shoulder.runSpeed();
    elbow.runSpeed();
    wrist.runSpeed();
  }
}

// ====================== TELNET ======================
void handleTelnet() {
  WiFiClient newClient = telnetServer.available();
  if (newClient) {
    if (telnetClient && telnetClient.connected()) telnetClient.stop();
    telnetClient = newClient;
    telnetBuf = "";

    uint8_t neg[] = {255, 253, 34, 255, 251, 3};
    telnetClient.write(neg, 6);

    telnetClient.println("\n=== Robotic Arm Telnet ===");
    telnetClient.println("Commands same as Serial");
    telnetClient.print("> ");

    while (telnetClient.available()) telnetClient.read();
    return;
  }

  if (!telnetClient || !telnetClient.connected()) return;

  int readBudget = 32;
  while (telnetClient.available() && readBudget-- > 0) {
    char c = telnetClient.read();
    if (c == '\r') continue;
    if (c == '\n') {
      telnetBuf.trim();
      if (telnetBuf.length() > 0) {
        Serial.print("Telnet: "); Serial.println(telnetBuf);
        processCommand(telnetBuf, true);
        if (telnetClient && telnetClient.connected()) {
          telnetClient.print("> ");
        }
      }
      telnetBuf = "";
    } else if (telnetBuf.length() < 64) {
      telnetBuf += c;
    }
  }
}

// ====================== WEB HANDLER ======================
void handleWeb() {
  WiFiClient client = webServer.available();
  if (!client) return;

  String reqLine = "";
  bool gotLine = false;
  unsigned long start = millis();

  while (millis() - start < 4) {
    while (client.available()) {
      char c = client.read();
      if (c == '\r') continue;
      if (c == '\n') {
        gotLine = true;
        break;
      }
      if (reqLine.length() < 140) reqLine += c;
    }
    if (gotLine) break;
  }

  if (!gotLine) {
    client.stop();
    return;
  }

  reqLine.trim();

  unsigned long drainStart = millis();
  while (client.available() && (millis() - drainStart < 2)) client.read();

  Serial.print("Web request: "); Serial.println(reqLine);

  String msg = "Unknown command";

  if (reqLine.indexOf("GET / ") != -1 || reqLine.indexOf("GET /index") != -1) {
    serveUI(client);
    client.stop();
    return;
  }

  if (reqLine.indexOf("GET /home") != -1) {
    if (calibrated) { goToHomeAll(); msg = "Homing ALL (simultaneous)"; }
    else msg = "Calibrate first";
  }
  else if (reqLine.indexOf("GET /stop ") != -1) {
    stopAll(); msg = "ALL STOPPED";
  }
  else if (reqLine.indexOf("GET /cal ") != -1) {
    doCalibrate(); msg = "Calibrated!";
  }
  else if (reqLine.indexOf("GET /hs ") != -1) { if (calibrated) { goToHomeS(); msg = "Homing Shoulder"; } else msg = "Calibrate first"; }
  else if (reqLine.indexOf("GET /he ") != -1) { if (calibrated) { goToHomeE(); msg = "Homing Elbow"; }    else msg = "Calibrate first"; }
  else if (reqLine.indexOf("GET /hw ") != -1) { if (calibrated) { goToHomeW(); msg = "Homing Wrist"; }    else msg = "Calibrate first"; }

  else if (reqLine.indexOf("GET /stopS") != -1) { shoulder.setSpeed(0); msg = "Shoulder stopped"; }
  else if (reqLine.indexOf("GET /stopE") != -1) { elbow.setSpeed(0);    msg = "Elbow stopped"; }
  else if (reqLine.indexOf("GET /stopW") != -1) { wrist.setSpeed(0);    msg = "Wrist stopped"; }

  else if (reqLine.indexOf("GET /setmin/S") != -1) { setLimit('S', false); msg = "Shoulder MIN set"; }
  else if (reqLine.indexOf("GET /setmin/E") != -1) { setLimit('E', false); msg = "Elbow MIN set"; }
  else if (reqLine.indexOf("GET /setmin/W") != -1) { setLimit('W', false); msg = "Wrist MIN set"; }
  else if (reqLine.indexOf("GET /setmax/S") != -1) { setLimit('S', true);  msg = "Shoulder MAX set"; }
  else if (reqLine.indexOf("GET /setmax/E") != -1) { setLimit('E', true);  msg = "Elbow MAX set"; }
  else if (reqLine.indexOf("GET /setmax/W") != -1) { setLimit('W', true);  msg = "Wrist MAX set"; }

  else if (reqLine.indexOf("GET /s/") != -1) { setSpeedFromReq('S', reqLine); msg = "Shoulder speed updated"; }
  else if (reqLine.indexOf("GET /e/") != -1) { setSpeedFromReq('E', reqLine); msg = "Elbow speed updated"; }
  else if (reqLine.indexOf("GET /w/") != -1) { setSpeedFromReq('W', reqLine); msg = "Wrist speed updated"; }

  String body = "{\"msg\":\"" + msg + "\"}";
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json");
  client.println("Connection: close");
  client.print("Content-Length: "); client.println(body.length());
  client.println();
  client.print(body);
  client.stop();
}

// ====================== WEB UI ======================
void serveUI(WiFiClient &client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: close");
  client.println();
  client.println(R"rawhtml(<!DOCTYPE html>
<html><head>
<meta charset='utf-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>Robot Arm</title>
<style>
  body{font-family:sans-serif;background:#1a1a2e;color:#eee;display:flex;flex-direction:column;align-items:center;padding:20px}
  h1{color:#e94560;margin-bottom:4px}
  #status{background:#16213e;padding:8px 16px;border-radius:8px;margin:8px 0;min-width:260px;text-align:center;color:#0f9}
  .card{background:#16213e;border-radius:12px;padding:16px;margin:10px;width:300px}
  .card h3{margin:0 0 12px;color:#e94560;display:flex;align-items:center;justify-content:space-between}
  .row{display:flex;gap:8px;justify-content:center;flex-wrap:wrap}
  button{padding:10px 14px;border:none;border-radius:8px;background:#0f3460;color:#fff;cursor:pointer;font-size:14px;transition:background .2s}
  button:hover{background:#e94560}
  button.stop{background:#c0392b;font-weight:bold}
  button.min{background:#c0392b}
  button.max{background:#27ae60}
  input[type=range]{width:100%;accent-color:#e94560}
  label{font-size:13px;color:#aaa}
</style>
</head><body>
<h1>&#129470; Robot Arm — Full Features</h1>
<div id='status'>Ready</div>

<div class='card'>
  <h3>System</h3>
  <div class='row'>
    <button onclick='cmd("/cal")'>&#128208; Calibrate</button>
    <button onclick='cmd("/home")'>&#127968; Home All</button>
    <button class='stop' onclick='cmd("/stop")'>&#9940; STOP ALL</button>
  </div>
</div>

<!-- Shoulder -->
<div class='card'>
  <h3>Shoulder 
    <button class='stop' onclick='stopJoint("S")' style='font-size:12px;padding:6px 10px;'>STOP</button>
  </h3>
  <div class='row'>
    <button class='min' onclick='cmd("/setmin/S")'>Set Min</button>
    <button onclick='cmd("/hs")'>&#127968; Home</button>
    <button class='max' onclick='cmd("/setmax/S")'>Set Max</button>
  </div>
  <label>Speed <span id='sv'>0</span></label>
  <input type='range' min='-100' max='100' value='0' id='ss' 
    oninput='document.getElementById("sv").textContent=this.value' 
    onchange='cmd("/s/"+this.value)'>
</div>

<!-- Elbow -->
<div class='card'>
  <h3>Elbow 
    <button class='stop' onclick='stopJoint("E")' style='font-size:12px;padding:6px 10px;'>STOP</button>
  </h3>
  <div class='row'>
    <button class='min' onclick='cmd("/setmin/E")'>Set Min</button>
    <button onclick='cmd("/he")'>&#127968; Home</button>
    <button class='max' onclick='cmd("/setmax/E")'>Set Max</button>
  </div>
  <label>Speed <span id='ev'>0</span></label>
  <input type='range' min='-100' max='100' value='0' id='es' 
    oninput='document.getElementById("ev").textContent=this.value' 
    onchange='cmd("/e/"+this.value)'>
</div>

<!-- Wrist -->
<div class='card'>
  <h3>Wrist 
    <button class='stop' onclick='stopJoint("W")' style='font-size:12px;padding:6px 10px;'>STOP</button>
  </h3>
  <div class='row'>
    <button class='min' onclick='cmd("/setmin/W")'>Set Min</button>
    <button onclick='cmd("/hw")'>&#127968; Home</button>
    <button class='max' onclick='cmd("/setmax/W")'>Set Max</button>
  </div>
  <label>Speed <span id='wv'>0</span></label>
  <input type='range' min='-100' max='100' value='0' id='ws' 
    oninput='document.getElementById("wv").textContent=this.value' 
    onchange='cmd("/w/"+this.value)'>
</div>

<script>
async function cmd(path){
  const statusEl = document.getElementById('status');
  statusEl.textContent = 'Sending...';
  try {
    const r = await fetch(path, {signal: AbortSignal.timeout(8000)});
    const j = await r.json();
    statusEl.textContent = j.msg || 'Done';
  } catch(e) {
    statusEl.textContent = 'Error: ' + e.message;
  }
}

function stopJoint(joint) {
  const statusEl = document.getElementById('status');
  statusEl.textContent = 'Stopping ' + joint + '...';
  fetch('/stop' + joint)
    .then(r => r.json())
    .then(j => statusEl.textContent = j.msg || 'Stopped ' + joint)
    .catch(e => statusEl.textContent = 'Error stopping ' + joint);
}
</script>
</body></html>)rawhtml");
}

// ====================== SPEED ======================
float jointMaxSpeedSps(char joint) {
  switch (toupper(joint)) {
    case 'S': return SHOULDER_MAX_SPEED_SPS;
    case 'E': return ELBOW_MAX_SPEED_SPS;
    case 'W': return WRIST_MAX_SPEED_SPS;
    default:  return 0.0;
  }
}

float levelToJointSpeed(char joint, int level) {
  int constrained = constrain(level, -MAX_SPEED_LEVEL_AFTER, MAX_SPEED_LEVEL_AFTER);
  float norm = constrained / (float)MAX_SPEED_LEVEL_AFTER;
  return norm * jointMaxSpeedSps(joint);
}

void applyDefaultMotionProfiles() {
  shoulder.setMaxSpeed(SHOULDER_MAX_SPEED_SPS * SPEED_PROFILE_HEADROOM);
  shoulder.setAcceleration(NORMAL_ACCEL);
  shoulder.setMinPulseWidth(50);

  elbow.setMaxSpeed(ELBOW_MAX_SPEED_SPS * SPEED_PROFILE_HEADROOM);
  elbow.setAcceleration(NORMAL_ACCEL);
  elbow.setMinPulseWidth(40);

  wrist.setMaxSpeed(WRIST_MAX_SPEED_SPS * SPEED_PROFILE_HEADROOM);
  wrist.setAcceleration(NORMAL_ACCEL);
  wrist.setMinPulseWidth(40);
}

void setSpeedFromReq(char joint, String &req) {
  String prefix = "/" + String((char)tolower(joint)) + "/";
  int idx = req.indexOf(prefix);
  if (idx == -1) return;

  idx += prefix.length();
  int endIdx = req.indexOf(' ', idx);
  if (endIdx == -1) endIdx = req.length();

  String valStr = req.substring(idx, endIdx);
  int level = valStr.toInt();
  int maxAllowed = calibrated ? MAX_SPEED_LEVEL_AFTER : MAX_SPEED_LEVEL_BEFORE;
  level = constrain(level, -maxAllowed, maxAllowed);

  float speedVal = levelToJointSpeed(joint, level);

  if (joint == 'S') shoulder.setSpeed(speedVal);
  else if (joint == 'E') elbow.setSpeed(speedVal);
  else if (joint == 'W') wrist.setSpeed(speedVal);

  Serial.print("Web speed set → ");
  Serial.print(joint);
  Serial.print(" level ");
  Serial.println(level);
}

// ====================== COMMAND PROCESSOR ======================
void processCommand(String input, bool toTelnet) {
  String cmd = input;
  cmd.toUpperCase();

  auto reply = [&](const String& text) {
    Serial.println(text);
    if (toTelnet && telnetClient && telnetClient.connected()) {
      telnetClient.println(text);
      telnetClient.flush();
    }
  };

  if (input == "0" || cmd == "0") {
    stopAll();
    reply("→ ALL STOPPED");
    return;
  }

  if (cmd.startsWith("STORE P")) {
    if (!calibrated) { reply("→ HOME first!"); return; }
    int num = cmd.charAt(7) - '1';
    if (num >= 0 && num <= 4) {
      pose[num][0] = shoulder.currentPosition();
      pose[num][1] = elbow.currentPosition();
      pose[num][2] = wrist.currentPosition();
      poseStored[num] = true;
      reply("→ Pose P" + String(num+1) + " stored");
    } else reply("→ Use STORE P1 to STORE P5");
    return;
  }

  if (input.length() == 2 && toupper(input[0]) == 'P') {
    int num = input.charAt(1) - '1';
    if (num >= 0 && num <= 4 && poseStored[num]) {
      goToPose(num);
    } else reply("→ Pose not stored or invalid");
    return;
  }

  if (cmd == "HOME") {
    doCalibrate();
    reply("=== CALIBRATION COMPLETE ===");
    return;
  }

  if (cmd == "H" || cmd == "HOME ALL") { if (calibrated) goToHomeAll(); else reply("→ Calibrate first"); return; }
  if (cmd == "HS") { if (calibrated) goToHomeS(); else reply("→ Calibrate first"); return; }
  if (cmd == "HE") { if (calibrated) goToHomeE(); else reply("→ Calibrate first"); return; }
  if (cmd == "HW") { if (calibrated) goToHomeW(); else reply("→ Calibrate first"); return; }

  if (cmd == "P" || cmd == "STATUS") { printStatusWithPoses(); return; }

  if (cmd.startsWith("MAX ") || cmd.startsWith("MIN ")) {
    if (!calibrated) { reply("→ HOME first!"); return; }
    bool isMax = cmd.startsWith("MAX ");
    char joint = cmd.charAt(4);
    setLimit(joint, isMax);
    return;
  }

  if (input.length() >= 2) {
    char jointChar = toupper(input[0]);
    int speedLevel = input.substring(1).toInt();
    int maxAllowed = calibrated ? MAX_SPEED_LEVEL_AFTER : MAX_SPEED_LEVEL_BEFORE;
    speedLevel = constrain(speedLevel, -maxAllowed, maxAllowed);

    float targetSpeed = levelToJointSpeed(jointChar, speedLevel);
    AccelStepper* motor = nullptr;
    const char* name = "?";

    switch (jointChar) {
      case 'S': motor = &shoulder; name = "Shoulder"; break;
      case 'E': motor = &elbow;    name = "Elbow";    break;
      case 'W': motor = &wrist;    name = "Wrist";    break;
    }

    if (motor) {
      if (jointChar == 'S') limitTriggeredS = false;
      if (jointChar == 'E') limitTriggeredE = false;
      if (jointChar == 'W') limitTriggeredW = false;

      motor->setSpeed(targetSpeed);

      String out = "→ " + String(name) + ": level " + String(speedLevel);
      if (speedLevel == 0) out += " → STOP";
      else {
        out += " → " + String(targetSpeed >= 0 ? "FWD " : "REV ") + String(abs(speedLevel));
        out += " ~" + String(abs(targetSpeed), 1) + " steps/s";
      }
      reply(out);
    }
  } else {
    reply("Cmds: S5 E-3 W0 | HOME | H | HS | HE | HW | STORE P1 | P1 | MAX S | MIN S | P | 0");
  }
}

void handleSerial() {
  int readBudget = 32;
  while (Serial.available() && readBudget-- > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      serialBuf.trim();
      if (serialBuf.length() > 0) processCommand(serialBuf, false);
      serialBuf = "";
      continue;
    }
    if (serialBuf.length() < 64) serialBuf += c;
  }
}

// ====================== CALIBRATE ======================
void doCalibrate() {
  shoulderCenter = shoulder.currentPosition();
  elbowCenter    = elbow.currentPosition();
  wristCenter    = wrist.currentPosition();

  shoulderMin = -DEFAULT_SHOULDER_RANGE;
  shoulderMax =  DEFAULT_SHOULDER_RANGE;
  elbowMin    = -DEFAULT_ELBOW_RANGE;
  elbowMax    =  DEFAULT_ELBOW_RANGE;
  wristMin    = -DEFAULT_WRIST_RANGE;
  wristMax    =  DEFAULT_WRIST_RANGE;

  calibrated = true;
  resetLimitFlags();
  printLimits();
}

// ====================== LIMITS ======================
void checkSimpleLimits(AccelStepper &motor, long center, long minRel, long maxRel, bool &limitFlag, const char* name, float maxSpeedSps) {
  if (!calibrated) return;

  long relPos = motor.currentPosition() - center;
  float currentSpeed = motor.speed();

  if ((currentSpeed > 0 && relPos >= maxRel) || (currentSpeed < 0 && relPos <= minRel)) {
    motor.setSpeed(0);
    if (!limitFlag) {
      limitFlag = true;
      Serial.print("→ "); Serial.print(name); Serial.println(": HARD LIMIT - STOPPED");
    }
    return;
  }

  if (!limitFlag) {
    if ((currentSpeed > 0 && relPos >= maxRel - LIMIT_BUFFER) ||
        (currentSpeed < 0 && relPos <= minRel + LIMIT_BUFFER)) {

      float slowSpeed = (LIMIT_SLOW_LEVEL / (float)MAX_SPEED_LEVEL_AFTER) * maxSpeedSps;
      if (currentSpeed < 0) slowSpeed = -slowSpeed;

      motor.setSpeed(slowSpeed);

      if (!limitFlag) {
        limitFlag = true;
        Serial.print("→ "); Serial.print(name); Serial.print(": Near limit - forced to level "); Serial.println(LIMIT_SLOW_LEVEL);
      }
    }
  }
}

void resetLimitFlags() {
  limitTriggeredS = limitTriggeredE = limitTriggeredW = false;
}

// ====================== POSES & HOMING ======================
void goToPose(int num) {
  stopAll();

  shoulder.setMaxSpeed(abs(levelToJointSpeed('S', FAST_MOVE_LEVEL)));
  shoulder.setAcceleration(FAST_ACCEL);
  shoulder.moveTo(pose[num][0]);
  homingS = (shoulder.distanceToGo() != 0);

  elbow.setMaxSpeed(abs(levelToJointSpeed('E', FAST_MOVE_LEVEL)));
  elbow.setAcceleration(FAST_ACCEL);
  elbow.moveTo(pose[num][1]);
  homingE = (elbow.distanceToGo() != 0);

  wrist.setMaxSpeed(abs(levelToJointSpeed('W', FAST_MOVE_LEVEL)));
  wrist.setAcceleration(FAST_ACCEL);
  wrist.moveTo(pose[num][2]);
  homingW = (wrist.distanceToGo() != 0);

  Serial.print("→ Moving to Pose P");
  Serial.print(num + 1);
  Serial.print(" at level ");
  Serial.print(FAST_MOVE_LEVEL);
  Serial.println("...");
}

void goToHomeAll() {
  stopAll();
  shoulder.setMaxSpeed(abs(levelToJointSpeed('S', FAST_MOVE_LEVEL)));
  shoulder.setAcceleration(FAST_ACCEL);
  shoulder.moveTo(shoulderCenter);
  homingS = true;

  elbow.setMaxSpeed(abs(levelToJointSpeed('E', FAST_MOVE_LEVEL)));
  elbow.setAcceleration(FAST_ACCEL);
  elbow.moveTo(elbowCenter);
  homingE = true;

  wrist.setMaxSpeed(abs(levelToJointSpeed('W', FAST_MOVE_LEVEL)));
  wrist.setAcceleration(FAST_ACCEL);
  wrist.moveTo(wristCenter);
  homingW = true;

  Serial.print("→ Homing ALL simultaneously at level ");
  Serial.print(FAST_MOVE_LEVEL);
  Serial.println("...");
}

void goToHomeS() {
  shoulder.setMaxSpeed(abs(levelToJointSpeed('S', FAST_MOVE_LEVEL)));
  shoulder.setAcceleration(FAST_ACCEL);
  shoulder.moveTo(shoulderCenter);
  homingS = true;
  Serial.print("→ Homing Shoulder at level ");
  Serial.print(FAST_MOVE_LEVEL);
  Serial.println("...");
}

void goToHomeE() {
  elbow.setMaxSpeed(abs(levelToJointSpeed('E', FAST_MOVE_LEVEL)));
  elbow.setAcceleration(FAST_ACCEL);
  elbow.moveTo(elbowCenter);
  homingE = true;
  Serial.print("→ Homing Elbow at level ");
  Serial.print(FAST_MOVE_LEVEL);
  Serial.println("...");
}

void goToHomeW() {
  wrist.setMaxSpeed(abs(levelToJointSpeed('W', FAST_MOVE_LEVEL)));
  wrist.setAcceleration(FAST_ACCEL);
  wrist.moveTo(wristCenter);
  homingW = true;
  Serial.print("→ Homing Wrist at level ");
  Serial.print(FAST_MOVE_LEVEL);
  Serial.println("...");
}

void stopAll() {
  shoulder.setSpeed(0);
  elbow.setSpeed(0);
  wrist.setSpeed(0);
  resetLimitFlags();
}

void finishHomingS() { 
  homingS = false; 
  shoulder.setSpeed(0); 
  applyDefaultMotionProfiles();
  Serial.println("Shoulder homed"); 
  resetLimitFlags(); 
}

void finishHomingE() { 
  homingE = false; 
  elbow.setSpeed(0); 
  applyDefaultMotionProfiles();
  Serial.println("Elbow homed"); 
  resetLimitFlags(); 
}

void finishHomingW() { 
  homingW = false; 
  wrist.setSpeed(0); 
  applyDefaultMotionProfiles();
  Serial.println("Wrist homed"); 
  resetLimitFlags(); 
}

void setLimit(char jointChar, bool isMax) {
  Serial.println("Custom limit set (full dynamic limits not expanded here)");
  printLimits();
}

void printStatusWithPoses() {
  printLimits();
  Serial.println("\nStored Poses:");
  for (int i = 0; i < 5; i++) {
    Serial.print("P"); Serial.print(i+1); Serial.print(": ");
    if (poseStored[i]) {
      Serial.print("S="); Serial.print(pose[i][0]);
      Serial.print(" E="); Serial.print(pose[i][1]);
      Serial.print(" W="); Serial.println(pose[i][2]);
    } else Serial.println("empty");
  }
}

void printLimits() {
  Serial.println("\n--- Centers & Limits ---");
  Serial.print("Calib: "); Serial.println(calibrated ? "YES" : "NO");
  Serial.print("S: C="); Serial.print(shoulderCenter); Serial.print(" Min="); Serial.print(shoulderMin); Serial.print(" Max="); Serial.println(shoulderMax);
  Serial.print("E: C="); Serial.print(elbowCenter); Serial.print(" Min="); Serial.print(elbowMin); Serial.print(" Max="); Serial.println(elbowMax);
  Serial.print("W: C="); Serial.print(wristCenter); Serial.print(" Min="); Serial.print(wristMin); Serial.print(" Max="); Serial.println(wristMax);
}