// ESP32-S3 micro-ROS Joint State Subscriber + ElegantOTA
//
// micro-ROS subscribes to /joint_states over USB Serial (ttyACM0).
// ElegantOTA runs independently over WiFi — no conflict.
//
// Pi-side setup (one time):
//   sudo apt install ros-jazzy-micro-ros-agent
//   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/moveo_arduino -b 1000000
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

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ElegantOTA.h>

// --- WiFi credentials (OTA only) ---
const char* ssid     = "WAVLINK-N";
const char* password = "Jk12345678";

WebServer server(80);

// --- Stepper pins ---
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

// ================== PER-JOINT STEPPER CONSTANTS ==================
//                          Waist  Shoulder  Elbow  WristRoll  WristPitch
const uint8_t MICROSTEPS[5]   = {  4,          16,            16,            16,         8          };
const float   DEG_PER_STEP[5] = {1.8,         1.8,           1.8,           1.8,        1.8        };
const float   GEAR_RATIO[5]   = {5.0,         1.0,           5.0,           5.0,        1.0        };
const float   BELT_RATIO[5]   = {14.45625f,    5.11875f,      4.65304275f,   1.0f,       4.0f       };

float steps_per_rad[5];
unsigned long step_interval_us[5];

const float ANGULAR_SPEED_RAD_S = 1.0;

float current_pos[5] = {0.0};
float target_pos[5]  = {0.0};
unsigned long last_step_time[5] = {0};

// ================== micro-ROS ==================
rcl_subscription_t subscriber;
sensor_msgs__msg__JointState joint_state_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Static storage for JointState message arrays (5 joints max)
double   js_pos[5], js_vel[5], js_eff[5];
rosidl_runtime_c__String js_names[5];
char     js_name_buf[5][32];

bool uros_ok = false;

// Reconnect attempt interval (ms)
#define UROS_RECONNECT_MS 3000

#define RCCHECK(fn)     { if ((fn) != RCL_RET_OK) { uros_ok = false; return; } }
#define RCSOFTCHECK(fn) { (void)(fn); }

void joint_state_callback(const void* msgin) {
  const sensor_msgs__msg__JointState* msg =
    (const sensor_msgs__msg__JointState*)msgin;
  size_t n = msg->position.size < 5 ? msg->position.size : 5;
  for (size_t i = 0; i < n; i++) {
    target_pos[i] = (float)msg->position.data[i];
  }
}

void uros_cleanup() {
  RCSOFTCHECK(rclc_executor_fini(&executor));
  RCSOFTCHECK(rcl_subscription_fini(&subscriber, &node));
  RCSOFTCHECK(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_support_fini(&support));
}

void uros_init() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "moveo_esp32", "", &support));
  RCCHECK(rclc_subscription_init_default(
    &subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/joint_states"));

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

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &subscriber, &joint_state_msg,
    &joint_state_callback, ON_NEW_DATA));

  uros_ok = true;
}

void setup() {
  // Serial is the micro-ROS USB transport — do NOT use Serial.print() after this
  Serial.begin(1000000);

  // Stepper pins
  pinMode(WAIST_STEP_PIN, OUTPUT);      pinMode(WAIST_DIR_PIN, OUTPUT);
  pinMode(SHOULDER_STEP_PIN, OUTPUT);   pinMode(SHOULDER_DIR_PIN, OUTPUT);
  pinMode(ELBOW_STEP_PIN, OUTPUT);      pinMode(ELBOW_DIR_PIN, OUTPUT);
  pinMode(WRIST_ROLL_STEP_PIN, OUTPUT); pinMode(WRIST_ROLL_DIR_PIN, OUTPUT);
  pinMode(WRIST_PITCH_STEP_PIN, OUTPUT);pinMode(WRIST_PITCH_DIR_PIN, OUTPUT);

  for (int i = 0; i < 5; i++) {
    steps_per_rad[i]    = (MICROSTEPS[i] * GEAR_RATIO[i] * BELT_RATIO[i] / DEG_PER_STEP[i]) * (180.0f / PI);
    step_interval_us[i] = (unsigned long)(1e6f / (ANGULAR_SPEED_RAD_S * steps_per_rad[i]));
  }

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
  }

  // Assign USB Serial as micro-ROS transport and wait for agent
  set_microros_transports();
  delay(2000);

  uros_init();
}

void loop() {
  // --- micro-ROS: spin executor (non-blocking, 0.5ms budget) ---
  if (uros_ok) {
    if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)) != RCL_RET_OK) {
      // Agent disconnected — schedule reconnect
      uros_cleanup();
      uros_ok = false;
    }
  } else {
    // Retry connecting to agent periodically
    static unsigned long last_reconnect_ms = 0;
    unsigned long now_ms = millis();
    if (now_ms - last_reconnect_ms >= UROS_RECONNECT_MS) {
      last_reconnect_ms = now_ms;
      uros_init();
    }
  }

  // --- ElegantOTA: rate-limited to every 10ms ---
  static unsigned long last_wifi_ms = 0;
  {
    unsigned long now_ms = millis();
    if (now_ms - last_wifi_ms >= 10) {
      server.handleClient();
      ElegantOTA.loop();
      last_wifi_ms = now_ms;
    }
  }

  // --- Stepper loop: unchanged from original ---
  for (int i = 0; i < 5; i++) {
    unsigned long now = micros();
    if (now - last_step_time[i] < step_interval_us[i]) continue;
    float error = target_pos[i] - current_pos[i];
    float steps_needed = error * steps_per_rad[i];
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
        bool dir = steps_needed > 0;
        digitalWrite(dir_pin, dir ? HIGH : LOW);
        digitalWrite(step_pin, HIGH);
        delayMicroseconds(2);
        digitalWrite(step_pin, LOW);
        current_pos[i] += (dir ? 1.0f : -1.0f) / steps_per_rad[i];
      }
      last_step_time[i] = now;
    }
  }
}