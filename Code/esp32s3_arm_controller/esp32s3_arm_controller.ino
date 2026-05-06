// Increase loopTask stack from 8192 (default) to 16384 bytes.
// rclc_support_init needs ~16KB — without this it silently crashes before
// sending a single serial byte to the micro-ROS agent.
uint32_t loopTaskStackSize = 16384;

// ESP32-S3 micro-ROS Joint State Subscriber + ElegantOTA
//
// micro-ROS subscribes to /joint_commands over WiFi UDP.
// ElegantOTA also runs over WiFi on the same connection.
//
// Pi-side agent:
//   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
//
// Arduino library required: micro_ros_arduino
//   https://github.com/micro-ROS/micro_ros_arduino/releases
//   Install via Sketch → Include Library → Add .ZIP Library

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float32.h>

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ElegantOTA.h>

// --- WiFi credentials ---
const char* ssid        = "WAVLINK-N";
const char* password    = "Jk12345678";
const char* agent_ip    = "192.168.1.142";  // Pi IP
const uint16_t agent_port = 8888;

WebServer server(80);

// --- Stepper pins ---
#define WAIST_STEP_PIN        4
#define WAIST_DIR_PIN         3
#define SHOULDER_STEP_PIN     6
#define SHOULDER_DIR_PIN      5
#define ELBOW_STEP_PIN        8
#define ELBOW_DIR_PIN         7
#define WRIST_ROLL_STEP_PIN   10
#define WRIST_ROLL_DIR_PIN    9
#define WRIST_PITCH_STEP_PIN  12
#define WRIST_PITCH_DIR_PIN   11

// ================== PER-JOINT STEPPER CONSTANTS ==================
//                             Waist  Shoulder  Elbow  WristRoll  WristPitch
const uint8_t MICROSTEPS[5]     = {  4,        8,      16,       16,        8   };
const float   DEG_PER_STEP[5]   = {1.8,      1.8,     1.8,      1.8,       1.8 };
const float   GEAR_RATIO[5]     = {5.0,      1.0,     5.0,      5.0,       1.0 };
const float   BELT_RATIO[5]     = {14.45625f, 5.11875f, 4.65304275f, 1.0f, 4.0f};
// Per-joint direction invert (true = flip HIGH/LOW on DIR pin)
const bool    DIR_INVERT[5]     = {false,    true,    false,    false,     true};
// Per-joint speed ceiling as a fraction of MAX_SPEED_RAD_S (1.0 = full speed)
const float   JOINT_SPEED_FACTOR[5] = {1.0f, 0.5f,   1.0f,     1.0f,      1.0f};
// Empirical calibration: ratio of (radians commanded) to (radians physically moved).
// Measured by commanding a known angle and observing actual joint rotation.
// < 1.0 → arm overshoots (too many steps); > 1.0 → arm undershoots.
//                                   Waist   Shoulder  Elbow   WristRoll  WristPitch
//   measurement:              90°→1.15r  90°→1.60r 90°→1.48r 180°→0.81r  90°→1.70r
const float   CALIB_FACTOR[5] = {1.15f/(PI/2), 1.60f/(PI/2), 1.48f/(PI/2), 0.81f/PI, 1.70f/(PI/2)};

float steps_per_rad[5];
unsigned long step_interval_us[5];

const float MAX_SPEED_RAD_S = 1.0;   // top speed — set this to the fastest safe value
float       speed_scale    = 1.0f;  // runtime 0.0–1.0; updated via /speed_scale topic

float current_pos[5] = {0.0};
float target_pos[5]  = {0.0};
unsigned long last_step_time[5] = {0};

// ================== Sinusoidal ramp state ==================
// Each move's velocity follows v(s) = v_max * sin(π * s/S),
// giving smooth acceleration from rest and deceleration to rest.
// RAMP_MIN_V clamps the start/end fraction to avoid infinite intervals.
#define RAMP_MIN_V      0.05f   // 5% of peak speed at start/end
// RAMP_EXPONENT < 1.0 sharpens the tails: the motor reaches full speed
// faster and lingers there longer.  1.0 = pure sine; 0.3 = very snappy.
#define RAMP_EXPONENT   0.35f
float ramp_total[5] = {1.0f}; // total steps in current move segment
float ramp_done[5]  = {0.0f}; // steps issued so far in this segment

// ================== micro-ROS ==================
rcl_subscription_t subscriber;
rcl_subscription_t speed_subscriber;
rcl_subscription_t home_subscriber;
sensor_msgs__msg__JointState joint_state_msg;
std_msgs__msg__Float32       speed_msg;
std_msgs__msg__Float32       home_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Static storage for JointState message arrays (5 joints max)
double   js_pos[5], js_vel[5], js_eff[5];
rosidl_runtime_c__String js_names[5];
char     js_name_buf[5][32];

bool uros_ok = false;

// === LOOP TIMING DEBUG GLOBALS ===
unsigned long dbg_loop_count     = 0;
unsigned long dbg_time_executor_us = 0;  // time in rclc_executor_spin_some
unsigned long dbg_time_ota_us      = 0;  // time in server.handleClient + ElegantOTA
unsigned long dbg_time_stepper_us  = 0;  // time in stepper for-loop
unsigned long dbg_steps_issued[5]  = {0};
unsigned long dbg_steps_skipped[5] = {0};  // interval not yet elapsed
unsigned long dbg_last_report_ms   = 0;

// Reconnect attempt interval (ms)
#define UROS_RECONNECT_MS    3000
// Ping agent every N ms to detect silent disconnection (UDP has no hard disconnect)
#define UROS_PING_INTERVAL_MS 2000

#define RCCHECK(fn)     { if ((fn) != RCL_RET_OK) { uros_ok = false; return; } }
#define RCSOFTCHECK(fn) { (void)(fn); }

void joint_state_callback(const void* msgin) {
  const sensor_msgs__msg__JointState* msg =
    (const sensor_msgs__msg__JointState*)msgin;

  size_t n = msg->position.size > 0 ? msg->position.size : msg->position.capacity;
  if (n > 5) n = 5;

  Serial.printf("[CB] %lu ms | %zu joints\n", millis(), n);
  for (size_t i = 0; i < n; i++) {
    float tgt = (float)msg->position.data[i];
    float err = tgt - current_pos[i];
    float steps = fabsf(err * steps_per_rad[i]);
    Serial.printf("  j%zu: cur=%.3f tgt=%.3f err=%.3f steps=%.1f\n",
                  i + 1, current_pos[i], tgt, err, steps);
    // Reset ramp whenever a new move requires at least 1 step
    if (steps >= 1.0f) {
      ramp_total[i] = steps;
      ramp_done[i]  = 0.0f;
    }
    target_pos[i] = tgt;
  }
}

void home_callback(const void* msgin) {
  const std_msgs__msg__Float32* msg = (const std_msgs__msg__Float32*)msgin;
  if (msg->data < 0.5f) return;  // only act on 1.0
  Serial.println("[HOME] zeroing all joint positions");
  for (int i = 0; i < 5; i++) {
    current_pos[i] = 0.0f;
    target_pos[i]  = 0.0f;
    ramp_total[i]  = 1.0f;
    ramp_done[i]   = 0.0f;
  }
}

void speed_callback(const void* msgin) {
  const std_msgs__msg__Float32* msg = (const std_msgs__msg__Float32*)msgin;
  float s = msg->data;
  if (s < 0.0f) s = 0.0f;
  if (s > 1.0f) s = 1.0f;
  speed_scale = s;
  Serial.printf("[SPEED] scale=%.2f\n", speed_scale);
}

void uros_cleanup() {
  RCSOFTCHECK(rclc_executor_fini(&executor));
  RCSOFTCHECK(rcl_subscription_fini(&home_subscriber, &node));
  RCSOFTCHECK(rcl_subscription_fini(&speed_subscriber, &node));
  RCSOFTCHECK(rcl_subscription_fini(&subscriber, &node));
  RCSOFTCHECK(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_support_fini(&support));
}

void uros_init() {
  Serial.printf("[uROS] init start @ %lu ms\n", millis());
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "moveo_esp32", "", &support));
  RCCHECK(rclc_subscription_init_best_effort(
    &subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/joint_commands"));

  // Wire static storage into message struct
  joint_state_msg.position.data     = js_pos;
  joint_state_msg.position.size     = 0;
  joint_state_msg.position.capacity = 5;
  joint_state_msg.velocity.data     = js_vel;
  joint_state_msg.velocity.size     = 0;
  joint_state_msg.velocity.capacity = 5;
  joint_state_msg.effort.data       = js_eff;
  joint_state_msg.effort.size       = 0;
  joint_state_msg.effort.capacity   = 5;
  joint_state_msg.name.data         = js_names;
  joint_state_msg.name.size         = 0;
  joint_state_msg.name.capacity     = 5;
  for (int i = 0; i < 5; i++) {
    js_names[i].data     = js_name_buf[i];
    js_names[i].size     = 0;
    js_names[i].capacity = 32;
  }

  RCCHECK(rclc_subscription_init_best_effort(
    &speed_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/speed_scale"));

  RCCHECK(rclc_subscription_init_best_effort(
    &home_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/home_cmd"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &subscriber, &joint_state_msg,
    &joint_state_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &speed_subscriber, &speed_msg,
    &speed_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &home_subscriber, &home_msg,
    &home_callback, ON_NEW_DATA));

  uros_ok = true;
  Serial.printf("[uROS] init OK @ %lu ms\n", millis());
}

void setup() {
  // Serial for debug output (not used for micro-ROS transport)
  Serial.begin(115200);

  // Stepper pins
  pinMode(WAIST_STEP_PIN, OUTPUT);      pinMode(WAIST_DIR_PIN, OUTPUT);
  pinMode(SHOULDER_STEP_PIN, OUTPUT);   pinMode(SHOULDER_DIR_PIN, OUTPUT);
  pinMode(ELBOW_STEP_PIN, OUTPUT);      pinMode(ELBOW_DIR_PIN, OUTPUT);
  pinMode(WRIST_ROLL_STEP_PIN, OUTPUT); pinMode(WRIST_ROLL_DIR_PIN, OUTPUT);
  pinMode(WRIST_PITCH_STEP_PIN, OUTPUT);pinMode(WRIST_PITCH_DIR_PIN, OUTPUT);

  for (int i = 0; i < 5; i++) {
    steps_per_rad[i]    = (MICROSTEPS[i] * GEAR_RATIO[i] * BELT_RATIO[i] / DEG_PER_STEP[i]) * (180.0f / PI) * CALIB_FACTOR[i];
    step_interval_us[i] = (unsigned long)(1e6f / (MAX_SPEED_RAD_S * JOINT_SPEED_FACTOR[i] * steps_per_rad[i]));
  }

  // Connect WiFi and configure micro-ROS UDP transport to Pi agent
  // set_microros_wifi_transports blocks until WiFi is connected
  Serial.printf("[WiFi] connecting to %s...\n", ssid);
  set_microros_wifi_transports((char*)ssid, (char*)password,
                               (char*)agent_ip, agent_port);

  // ElegantOTA — WiFi already connected above
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi] connected, IP: %s\n", WiFi.localIP().toString().c_str());
    MDNS.begin("ArmESP");
    ElegantOTA.begin(&server);
    server.begin();
  } else {
    Serial.println("[WiFi] NOT connected!");
  }

  delay(2000);
  uros_init();
}

void loop() {
  // === STATS DUMP every 1000ms ===
  dbg_loop_count++;
  {
    unsigned long _now = millis();
    if (_now - dbg_last_report_ms >= 1000) {
      unsigned long _dt = _now - dbg_last_report_ms;
      dbg_last_report_ms = _now;
      Serial.printf("[DBG] loops=%lu dt=%lums | exec=%luus ota=%luus step=%luus\n",
                    dbg_loop_count, _dt,
                    dbg_time_executor_us, dbg_time_ota_us, dbg_time_stepper_us);
      Serial.printf("[DBG] steps j1=%lu j2=%lu j3=%lu j4=%lu j5=%lu\n",
                    dbg_steps_issued[0], dbg_steps_issued[1], dbg_steps_issued[2],
                    dbg_steps_issued[3], dbg_steps_issued[4]);
      Serial.printf("[DBG] skips j1=%lu j2=%lu j3=%lu j4=%lu j5=%lu\n",
                    dbg_steps_skipped[0], dbg_steps_skipped[1], dbg_steps_skipped[2],
                    dbg_steps_skipped[3], dbg_steps_skipped[4]);
      dbg_loop_count = 0;
      dbg_time_executor_us = dbg_time_ota_us = dbg_time_stepper_us = 0;
      memset(dbg_steps_issued,  0, sizeof(dbg_steps_issued));
      memset(dbg_steps_skipped, 0, sizeof(dbg_steps_skipped));
    }
  }

  // --- micro-ROS: spin executor + ping-based disconnect detection ---
  if (uros_ok) {
    { unsigned long _t = micros(); rclc_executor_spin_some(&executor, 0); dbg_time_executor_us += micros() - _t; }

    // Ping agent periodically — catches silent UDP disconnects
    static unsigned long last_ping_ms = 0;
    unsigned long now_ms = millis();
    if (now_ms - last_ping_ms >= UROS_PING_INTERVAL_MS) {
      last_ping_ms = now_ms;
      unsigned long ping_start = millis();
      bool ping_ok = (rmw_uros_ping_agent(200, 1) == RMW_RET_OK);
      unsigned long ping_dur = millis() - ping_start;
      Serial.printf("[PING] %s (%lu ms)\n", ping_ok ? "OK" : "FAIL", ping_dur);
      if (!ping_ok) {
        // Agent unreachable — cleanup and schedule reconnect
        uros_cleanup();
        uros_ok = false;
        Serial.println("[uROS] disconnected, will reconnect");
      }
    }
  } else {
    // Retry connecting to agent periodically
    static unsigned long last_reconnect_ms = 0;
    unsigned long now_ms = millis();
    if (now_ms - last_reconnect_ms >= UROS_RECONNECT_MS) {
      last_reconnect_ms = now_ms;
      // Cleanup any partially-initialized state before retrying
      uros_cleanup();
      // Re-set transport before each reconnect attempt (UDP requires it after cleanup)
      set_microros_wifi_transports((char*)ssid, (char*)password,
                                   (char*)agent_ip, agent_port);
      uros_init();
    }
  }

  // --- ElegantOTA: rate-limited to every 10ms ---
  static unsigned long last_wifi_ms = 0;
  {
    unsigned long now_ms = millis();
    if (now_ms - last_wifi_ms >= 10) {
      { unsigned long _t = micros(); server.handleClient(); ElegantOTA.loop(); dbg_time_ota_us += micros() - _t; }
      last_wifi_ms = now_ms;
    }
  }

  // --- Stepper loop ---
  { unsigned long _st = micros();
  for (int i = 0; i < 5; i++) {
    unsigned long now = micros();
    // Sinusoidal ramp: slow at start, peak in middle, slow at end
    {
      float _prog = (ramp_total[i] > 0.0f) ? (ramp_done[i] / ramp_total[i]) : 0.5f;
      if (_prog > 0.999f) _prog = 0.999f;
      float _vf = powf(sinf(PI * _prog), RAMP_EXPONENT);
      if (_vf < RAMP_MIN_V) _vf = RAMP_MIN_V;
      float _eff_scale = speed_scale < 0.01f ? 0.01f : speed_scale;
      unsigned long ramp_interval_us = (unsigned long)(step_interval_us[i] / (_vf * _eff_scale));
      if (now - last_step_time[i] < ramp_interval_us) { dbg_steps_skipped[i]++; continue; }
    }
    float error = target_pos[i] - current_pos[i];
    float steps_needed = error * steps_per_rad[i];
    static bool was_moving[5] = {false};
    bool moving = fabs(steps_needed) >= 1.0f;
    if (moving != was_moving[i]) {
      Serial.printf("[STEP] j%d %s | cur=%.3f tgt=%.3f steps=%.1f prog=%.2f\n",
                    i+1, moving ? "START" : "STOP ",
                    current_pos[i], target_pos[i], steps_needed,
                    ramp_done[i] / max(ramp_total[i], 1.0f));
      was_moving[i] = moving;
    }
    if (fabs(steps_needed) >= 1.0f) {
      int step_pin = -1, dir_pin = -1;
      switch (i) {
        case 0: step_pin = WAIST_STEP_PIN;       dir_pin = WAIST_DIR_PIN;       break;
        case 1: step_pin = SHOULDER_STEP_PIN;    dir_pin = SHOULDER_DIR_PIN;    break;
        case 2: step_pin = ELBOW_STEP_PIN;       dir_pin = ELBOW_DIR_PIN;       break;
        case 3: step_pin = WRIST_ROLL_STEP_PIN;  dir_pin = WRIST_ROLL_DIR_PIN;  break;
        case 4: step_pin = WRIST_PITCH_STEP_PIN; dir_pin = WRIST_PITCH_DIR_PIN; break;
      }
      if (step_pin != -1) {
        bool move_positive = (steps_needed > 0);
        bool dir = move_positive ^ DIR_INVERT[i];
        digitalWrite(dir_pin, dir ? HIGH : LOW);
        digitalWrite(step_pin, HIGH);
        delayMicroseconds(2);
        digitalWrite(step_pin, LOW);
        current_pos[i] += (move_positive ? 1.0f : -1.0f) / steps_per_rad[i];
        ramp_done[i]++;
        dbg_steps_issued[i]++;
      }
      last_step_time[i] = now;
    }
  }
  dbg_time_stepper_us += micros() - _st; }
}
