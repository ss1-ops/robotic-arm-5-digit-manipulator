#include <SCServo.h>
#include <WiFiS3.h>
#include <ArduinoMDNS.h>

// === WiFi credentials ===
const char* ssid     = "RPi";
const char* password = "1l0v3y0uS4m";

// Static IP settings for the RPi 2.4 GHz network
IPAddress localIP(192, 168, 10, 50);
IPAddress gateway(192, 168, 10, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns1(8, 8, 8, 8);

HLSCL hls;
#define BAUD_RATE 1000000

WiFiServer server(80);
WiFiUDP udp;
MDNS mdns(udp);

// === Servo configuration ===
const uint8_t servoIDs[] = {0, 1, 2, 3, 4, 5, 6};
const int numServos = sizeof(servoIDs) / sizeof(servoIDs[0]);

// === HARDCODED LIMITS — TUNE THESE PER SERVO ===
// Order matches servoIDs[] above:  ID 3,    ID 4,    ID 5,    ID 6
// lowerLimit = "Closed" position (slider all the way left)
// upperLimit = "Open"   position (slider all the way right)
// Range: -2048 to 4096
int lowerLimit[numServos] = {
  364,    // ID 0 — closed
  2000,    // ID 1 — closed
  -1560,    // ID 2 — closed
  -1875,    // ID 3 — closed
  -2440,    // ID 4 — closed
  120,    // ID 5 — closed
  -225     // ID 6 — closed
};

int upperLimit[numServos] = {
  1536, // ID 0 — open
  3000, // ID 1 — open
  1000, // ID 2 — open
  1365, // ID 3 — open
  1024, // ID 4 — open
  3500, // ID 5 — open
  2750  // ID 6 — open
};

int lastPos[numServos] = {0, 0, 0, 0};

int indexForID(int id) {
  for (int i = 0; i < numServos; i++) if (servoIDs[i] == id) return i;
  return -1;
}

void moveServoToPos(int id, int position) {
  position = constrain(position, -4096, 4096);
  int idx = indexForID(id);
  if (idx >= 0) lastPos[idx] = position;
  hls.WritePosEx(id, position, 150, 50, 500);
}

void setHandPosition(bool open) {
  for (int i = 0; i < numServos; i++) {
    int id = servoIDs[i];
    int target;
    if (open) {
      target = upperLimit[i];
    } else {
      switch (id) {
        case 0: target = 989; break;
        case 1: target = 2390; break;
        case 2: target = lowerLimit[i]; break; // fully closed
        default: target = lowerLimit[i]; break;
      }
    }
    moveServoToPos(id, target);
    delay(100); // add a short pause so commands are sent sequentially, reducing bus/power stress
  }
}

void setHookEmGesture() {
  setHandPosition(false);
  moveServoToPos(3, upperLimit[3]);
  delay(100);
  moveServoToPos(6, upperLimit[6]);
  delay(100);
}

void setIdiGesture() {
  setHandPosition(false);
  delay(100);
  moveServoToPos(4, upperLimit[4]);
  delay(100);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(BAUD_RATE);
  hls.pSerial = &Serial1;
  delay(1000);

  Serial.println("=== HLS3606M Hand Controller ===");

  for (int i = 0; i < numServos; i++) {
    hls.EnableTorque(servoIDs[i], 1);
    Serial.print("Torque ON for ID ");
    Serial.println(servoIDs[i]);
    delay(50);
  }

  Serial.print("Connecting to WiFi");
  WiFi.config(localIP, gateway, subnet, dns1);
  WiFi.begin(ssid, password);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Connected. IP: ");
    Serial.println(WiFi.localIP());
    mdns.begin(WiFi.localIP(), "hand");
    Serial.println("mDNS started: http://hand.local");
    server.begin();
    Serial.println("HTTP server started on port 80");
  } else {
    Serial.println("WiFi connection failed. Serial commands still work.");
  }

  Serial.println();
  Serial.println("Serial commands: setid <old> <new>, move <id> <angle>, pos <id> <pos>, torque <id> <0|1>");
}

const char HTML_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Hand Control</title>
<style>
  body { font-family: -apple-system, sans-serif; max-width: 600px; margin: 20px auto; padding: 0 16px; background: #1a1a1a; color: #eee; }
  h1 { font-size: 1.4em; }
  .servo { background: #2a2a2a; padding: 16px; margin: 12px 0; border-radius: 8px; }
  .servo-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 8px; }
  .servo-title { font-weight: bold; font-size: 1.1em; }
  .servo-pos { color: #8af; font-family: monospace; }
  .hand-actions { display: flex; gap: 10px; margin: 12px 0; }
  .hand-actions button { flex: 1; padding: 12px 16px; font-size: 1em; border: none; border-radius: 8px; background: #0077cc; color: #fff; cursor: pointer; }
  .hand-actions button:hover { background: #005fa3; }
  .slider-row { display: flex; align-items: center; gap: 10px; margin: 8px 0; }
  .slider-row .label { min-width: 60px; font-size: 0.85em; color: #aaa; }
  input[type=range] { flex: 1; }
  .limits-display { font-size: 0.8em; color: #888; margin-top: 6px; }
</style>
</head>
<body>
<h1>Hand Control</h1>
<div class="hand-actions">
  <button id="btn-close" onclick="setAll(false)">Close all fingers</button>
  <button id="btn-open" onclick="setAll(true)">Open all fingers</button>
  <button id="btn-hook" onclick="setGesture('hookem')">Hook 'em</button>
  <button id="btn-idi" onclick="setGesture('idi')">Иди на хуй</button>
</div>
<div id="servos"></div>
<script>
const SERVO_IDS = [0, 1, 2, 3, 4, 5, 6];
const SERVO_LABELS = {
  0: 'Thumb 1',
  1: 'Thumb 2',
  2: 'Thumb 3',
  3: 'Index',
  4: 'Middle',
  5: 'Ring',
  6: 'Pinky'
};
let limits = {};
let positions = {};

function render() {
  const container = document.getElementById('servos');
  container.innerHTML = '';
  for (const id of SERVO_IDS) {
    const lo = limits[id]?.lower ?? 0;
    const hi = limits[id]?.upper ?? 4096;
    const pos = positions[id] ?? lo;
    const div = document.createElement('div');
    div.className = 'servo';
    const name = SERVO_LABELS[id] || `Servo ${id}`;
    div.innerHTML = `
      <div class="servo-header">
        <span class="servo-title">${name}</span>
        <span class="servo-pos" id="pos-${id}">${pos}</span>
      </div>
      <div class="slider-row">
        <span class="label">Closed</span>
        <input type="range" id="slider-${id}" min="${lo}" max="${hi}" value="${pos}">
        <span class="label" style="text-align:right">Open</span>
      </div>
      <div class="limits-display">Range: ${lo} (closed) to ${hi} (open)</div>
    `;
    container.appendChild(div);
    const slider = div.querySelector(`#slider-${id}`);
    slider.addEventListener('input', (e) => {
      document.getElementById(`pos-${id}`).textContent = e.target.value;
    });
    slider.addEventListener('change', (e) => {
      sendPos(id, parseInt(e.target.value));
    });
  }
}

async function loadState() {
  const r = await fetch('/state');
  const data = await r.json();
  limits = data.limits;
  positions = data.positions;
  render();
}

async function sendPos(id, pos) {
  await fetch(`/pos?id=${id}&pos=${pos}`);
}

async function setAll(open) {
  await fetch(`/hand?cmd=${open ? 'open' : 'close'}`);
}

async function setGesture(cmd) {
  await fetch(`/hand?cmd=${cmd}`);
}

loadState();
</script>
</body>
</html>
)HTML";

String getQueryParam(const String& query, const String& key) {
  int start = query.indexOf(key + "=");
  if (start == -1) return "";
  start += key.length() + 1;
  int end = query.indexOf('&', start);
  if (end == -1) end = query.length();
  return query.substring(start, end);
}

void handleClient(WiFiClient& client) {
  String requestLine = client.readStringUntil('\r');
  client.readStringUntil('\n');

  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r" || line.length() <= 1) break;
  }

  int firstSpace = requestLine.indexOf(' ');
  int secondSpace = requestLine.indexOf(' ', firstSpace + 1);
  String fullPath = requestLine.substring(firstSpace + 1, secondSpace);

  String path = fullPath;
  String query = "";
  int qIdx = fullPath.indexOf('?');
  if (qIdx != -1) {
    path = fullPath.substring(0, qIdx);
    query = fullPath.substring(qIdx + 1);
  }

  if (path == "/" || path == "/index.html") {
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println();
    client.print(HTML_PAGE);
  }
  else if (path == "/state") {
    String json = "{\"limits\":{";
    for (int i = 0; i < numServos; i++) {
      if (i > 0) json += ",";
      json += "\"" + String(servoIDs[i]) + "\":{\"lower\":" + lowerLimit[i] + ",\"upper\":" + upperLimit[i] + "}";
    }
    json += "},\"positions\":{";
    for (int i = 0; i < numServos; i++) {
      if (i > 0) json += ",";
      json += "\"" + String(servoIDs[i]) + "\":" + lastPos[i];
    }
    json += "}}";

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: application/json");
    client.println("Connection: close");
    client.println();
    client.print(json);
  }
  else if (path == "/pos") {
    int id = getQueryParam(query, "id").toInt();
    int pos = getQueryParam(query, "pos").toInt();
    Serial.print("Web pos: ID ");
    Serial.print(id);
    Serial.print(" -> ");
    Serial.println(pos);
    moveServoToPos(id, pos);

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/plain");
    client.println("Connection: close");
    client.println();
    client.print("OK");
  }
  else if (path == "/hand") {
    String cmd = getQueryParam(query, "cmd");
    if (cmd == "open") {
      Serial.println("Web: open hand");
      setHandPosition(true);
    } else if (cmd == "close") {
      Serial.println("Web: close hand");
      setHandPosition(false);
    } else if (cmd == "hookem") {
      Serial.println("Web: hook 'em gesture");
      setHookEmGesture();
    } else if (cmd == "idi") {
      Serial.println("Web: idi na huy gesture");
      setIdiGesture();
    }
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/plain");
    client.println("Connection: close");
    client.println();
    client.print("OK");
  }
  else {
    client.println("HTTP/1.1 404 Not Found");
    client.println("Connection: close");
    client.println();
  }
}

void handleSerialCommand() {
  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0) return;

  int firstSpace = input.indexOf(' ');
  if (firstSpace == -1) { Serial.println("Bad command"); return; }
  String cmd = input.substring(0, firstSpace);
  String rest = input.substring(firstSpace + 1);
  rest.trim();
  int secondSpace = rest.indexOf(' ');
  if (secondSpace == -1) { Serial.println("Need two args"); return; }
  int arg1 = rest.substring(0, secondSpace).toInt();
  int arg2 = rest.substring(secondSpace + 1).toInt();

  if (cmd == "pos") {
    int position = constrain(arg2, -4096, 4096);
    Serial.print("-> Servo ID "); Serial.print(arg1);
    Serial.print("  -> Position "); Serial.println(position);
    moveServoToPos(arg1, position);
  }
  else if (cmd == "move") {
    int angle = constrain(arg2, -360, 360);
    int position = (long)angle * 4096 / 360;
    Serial.print("-> Servo ID "); Serial.print(arg1);
    Serial.print("  -> Angle "); Serial.print(angle);
    Serial.print(" (pos "); Serial.print(position); Serial.println(")");
    moveServoToPos(arg1, position);
  }
  else if (cmd == "torque") {
    hls.EnableTorque(arg1, arg2 ? 1 : 0);
    Serial.print("Torque ID "); Serial.print(arg1);
    Serial.print(" = "); Serial.println(arg2 ? "ON" : "OFF");
  }
  else if (cmd == "setid") {
    Serial.print("Changing ID "); Serial.print(arg1);
    Serial.print(" -> "); Serial.println(arg2);
    hls.EnableTorque(arg1, 0); delay(20);
    hls.unLockEprom(arg1); delay(20);
    hls.writeByte(arg1, 5, arg2); delay(20);
    hls.LockEprom(arg2); delay(20);
    Serial.println("Sent.");
  }
  else {
    Serial.print("Unknown: "); Serial.println(cmd);
  }
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    handleClient(client);
    client.stop();
  }
  mdns.run();
  if (Serial.available()) {
    handleSerialCommand();
  }
}