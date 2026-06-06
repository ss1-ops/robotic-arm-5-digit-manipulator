// moveo_serial_test.ino
// Diagnostic sketch — identical stepper math/loop, no micro-ROS overhead.
// ElegantOTA included for wireless reflashing.
//
// Command format (newline-terminated):  "j0,j1,j2,j3,j4"   (radians, float)
// Example:  0.5,0.0,0.0,0.0,0.0
// Status output every 500 ms: actual step rate per joint (Hz)
//
// Flash initial upload via USB, then use OTA at http://ArmESP/update
// Test from Pi:  python3 ~/ros_nodes/serial_test.py
//   (kill micro_ros_agent first so the port is free)

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ElegantOTA.h>

// --- WiFi credentials (OTA only) ---
const char* ssid     = "WAVLINK-N";
const char* password = "Jk12345678";

WebServer server(80);

#define BAUD_RATE 1000000

// --- Stepper pins (identical to main firmware) ---
#define WAIST_STEP_PIN        5
#define WAIST_DIR_PIN         4
#define SHOULDER_STEP_PIN     7
#define SHOULDER_DIR_PIN      6
#define ELBOW_STEP_PIN        9
#define ELBOW_DIR_PIN         8
#define WRIST_ROLL_STEP_PIN   11
#define WRIST_ROLL_DIR_PIN    10
#define WRIST_PITCH_STEP_PIN  13
#define WRIST_PITCH_DIR_PIN   12

// --- Identical constants to main firmware ---
//                              Waist  Shoulder  Elbow  WristRoll  WristPitch
const uint8_t MICROSTEPS[5]   = {  4,       16,    16,        16,          8 };
const float   DEG_PER_STEP[5] = {1.8,      1.8,   1.8,       1.8,        1.8};
const float   GEAR_RATIO[5]   = {5.0,      1.0,   5.0,       5.0,        1.0};
const float   BELT_RATIO[5]   = {14.45625f, 5.11875f, 4.65304275f, 1.0f, 4.0f};

const float ANGULAR_SPEED_RAD_S = 0.35f; // rad/s — 35% of max for safe testing

float steps_per_rad[5];
unsigned long step_interval_us[5];

float current_pos[5] = {0.0f};
float target_pos[5]  = {0.0f};
unsigned long last_step_time[5] = {0};

// Step rate measurement
unsigned long step_count[5] = {0};
unsigned long last_report_ms = 0;

// Serial input buffer
char serial_buf[64];
uint8_t buf_idx = 0;

// ----------------------------------------------------------------

void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial) {}

  // WiFi for ElegantOTA (non-blocking, 10s timeout)
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
    delay(200);
  }
  if (WiFi.status() == WL_CONNECTED) {
    MDNS.begin("ArmESP");
    ElegantOTA.begin(&server);
    server.begin();
    Serial.print("OTA ready at http://");
    Serial.print(WiFi.localIP());
    Serial.println("/update");
  } else {
    Serial.println("WiFi not connected — OTA unavailable");
  }

  // Stepper output pins
  const int pins[] = {
    WAIST_STEP_PIN, WAIST_DIR_PIN,
    SHOULDER_STEP_PIN, SHOULDER_DIR_PIN,
    ELBOW_STEP_PIN, ELBOW_DIR_PIN,
    WRIST_ROLL_STEP_PIN, WRIST_ROLL_DIR_PIN,
    WRIST_PITCH_STEP_PIN, WRIST_PITCH_DIR_PIN
  };
  for (int p : pins) pinMode(p, OUTPUT);

  // Pre-compute step rate constants (same as main firmware setup())
  for (int i = 0; i < 5; i++) {
    steps_per_rad[i]    = (MICROSTEPS[i] * GEAR_RATIO[i] * BELT_RATIO[i]
                          / DEG_PER_STEP[i]) * (180.0f / PI);
    step_interval_us[i] = (unsigned long)(1e6f / (ANGULAR_SPEED_RAD_S * steps_per_rad[i]));
  }

  Serial.println("READY  send: j0,j1,j2,j3,j4  (radians)");
  Serial.println("--- computed step parameters ---");
  for (int i = 0; i < 5; i++) {
    Serial.print("  joint"); Serial.print(i);
    Serial.print("  steps/rad="); Serial.print(steps_per_rad[i], 1);
    Serial.print("  interval_us="); Serial.print(step_interval_us[i]);
    Serial.print("  max_hz="); Serial.println((unsigned long)(1e6f / step_interval_us[i]));
  }
  Serial.println("--------------------------------");
}

// ----------------------------------------------------------------

void parse_command(char* buf) {
  float v[5] = {0, 0, 0, 0, 0};
  int n = sscanf(buf, "%f,%f,%f,%f,%f", &v[0], &v[1], &v[2], &v[3], &v[4]);
  if (n == 5) {
    for (int i = 0; i < 5; i++) target_pos[i] = v[i];
    Serial.print("CMD [");
    for (int i = 0; i < 5; i++) {
      Serial.print(v[i], 3);
      if (i < 4) Serial.print(", ");
    }
    Serial.print("]  steps needed: [");
    for (int i = 0; i < 5; i++) {
      Serial.print((int)((v[i] - current_pos[i]) * steps_per_rad[i]));
      if (i < 4) Serial.print(", ");
    }
    Serial.println("]");
  } else {
    Serial.print("ERR: expected 5 floats, got ");
    Serial.print(n);
    Serial.print(" from: '");
    Serial.print(buf);
    Serial.println("'");
  }
}

// ----------------------------------------------------------------

void loop() {
  // --- Read serial commands ---
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (buf_idx > 0) {
        serial_buf[buf_idx] = '\0';
        parse_command(serial_buf);
        buf_idx = 0;
      }
    } else if (buf_idx < (uint8_t)(sizeof(serial_buf) - 1)) {
      serial_buf[buf_idx++] = c;
    }
  }

  // --- Stepper loop (byte-for-byte identical to main firmware) ---
  for (int i = 0; i < 5; i++) {
    unsigned long now = micros();
    if (now - last_step_time[i] < step_interval_us[i]) continue;
    float error = target_pos[i] - current_pos[i];
    float steps_needed = error * steps_per_rad[i];
    if (fabsf(steps_needed) >= 1.0f) {
      int step_pin = -1, dir_pin = -1;
      switch (i) {
        case 0: step_pin = WAIST_STEP_PIN;       dir_pin = WAIST_DIR_PIN;       break;
        case 1: step_pin = SHOULDER_STEP_PIN;    dir_pin = SHOULDER_DIR_PIN;    break;
        case 2: step_pin = ELBOW_STEP_PIN;       dir_pin = ELBOW_DIR_PIN;       break;
        case 3: step_pin = WRIST_ROLL_STEP_PIN;  dir_pin = WRIST_ROLL_DIR_PIN;  break;
        case 4: step_pin = WRIST_PITCH_STEP_PIN; dir_pin = WRIST_PITCH_DIR_PIN; break;
      }
      if (step_pin != -1) {
        bool dir = steps_needed > 0;
        digitalWrite(dir_pin, dir ? HIGH : LOW);
        digitalWrite(step_pin, HIGH);
        delayMicroseconds(2);
        digitalWrite(step_pin, LOW);
        current_pos[i] += (dir ? 1.0f : -1.0f) / steps_per_rad[i];
        step_count[i]++;
        last_step_time[i] = now;
      }
    }
  }

  // --- ElegantOTA: rate-limited to every 10ms ---
  static unsigned long last_wifi_ms = 0;
  {
    unsigned long now_ms2 = millis();
    if (now_ms2 - last_wifi_ms >= 10) {
      server.handleClient();
      ElegantOTA.loop();
      last_wifi_ms = now_ms2;
    }
  }

  // --- Status report every 500 ms ---
  unsigned long now_ms = millis();
  if (now_ms - last_report_ms >= 500) {
    Serial.print("POS:");
    for (int i = 0; i < 5; i++) {
      Serial.print(" j"); Serial.print(i);
      Serial.print("="); Serial.print(current_pos[i], 4);
      Serial.print("rad @"); Serial.print(step_count[i] * 2); Serial.print("Hz");
      step_count[i] = 0;
    }
    Serial.println();
    last_report_ms = now_ms;
  }
}
