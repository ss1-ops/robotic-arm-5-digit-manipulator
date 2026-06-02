// Increase loopTask stack from 8192 (default) to 16384 bytes.
// rclc_support_init needs ~16KB — without this it silently crashes before
// sending a single serial byte to the micro-ROS agent.
uint32_t loopTaskStackSize = 16384;

// ESP32-S3 micro-ROS Joint State Subscriber — USB CDC transport.
//
// micro-ROS uses Serial (= USB CDC, when compiled with CDCOnBoot=cdc) as its
// XRCE-DDS transport. The Pi runs the agent against /dev/ttyACM0:
//
//   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
//
// IMPORTANT: while the agent is running it owns /dev/ttyACM0 — kill it before
// flashing or arduino-cli upload will fail. After flashing, restart the agent.
//
// IMPORTANT: do NOT call Serial.print* anywhere — Serial is the transport,
// debug bytes would corrupt the protocol stream. All previous Serial debug
// output has been removed for this reason.
//
// Compile with FQBN: esp32:esp32:esp32s3:CDCOnBoot=cdc
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
//   measurement:              90°→1.15r  90°→1.60r 90°→1.48r  (removed)  90°→1.70r
const float   CALIB_FACTOR[5] = {1.15f/(PI/2), 1.60f/(PI/2), 1.48f/(PI/2), 1.0f, 1.70f/(PI/2)};

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
#define RAMP_MIN_V       0.25f  // minimum velocity fraction at start/end of move
#define RAMP_ACCEL_FRAC  0.15f  // fraction of steps spent accelerating
#define RAMP_DECEL_FRAC  0.15f  // fraction of steps spent decelerating
// Velocity profile: linear ramp-up for ACCEL_FRAC, full speed for middle, linear ramp-down for DECEL_FRAC.
float ramp_total[5] = {1.0f}; // total steps in current move segment
float ramp_done[5]  = {0.0f}; // steps issued so far in this segment
// Per-move speed fraction for each joint (0.0–1.0).
// Set in joint_state_callback so all joints finish their move simultaneously:
// the joint with the most steps runs at 1.0; all others are scaled down.
float dynamic_speed_factor[5] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

// ================== micro-ROS ==================
rcl_subscription_t subscriber;
rcl_subscription_t speed_subscriber;
rcl_subscription_t home_subscriber;
rcl_subscription_t reboot_subscriber;
sensor_msgs__msg__JointState joint_state_msg;
std_msgs__msg__Float32       speed_msg;
std_msgs__msg__Float32       home_msg;
std_msgs__msg__Float32       reboot_msg;
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
#define UROS_RECONNECT_MS    3000
// Ping agent every N ms to detect silent disconnection.
// Serial transport is much more reliable than UDP — disconnects show up
// immediately as transport read errors — but a periodic ping costs nothing
// and gives us a definitive heartbeat.
#define UROS_PING_INTERVAL_MS 2000

#define RCCHECK(fn)     { if ((fn) != RCL_RET_OK) { uros_ok = false; return; } }
#define RCSOFTCHECK(fn) { (void)(fn); }

void joint_state_callback(const void* msgin) {
  const sensor_msgs__msg__JointState* msg =
    (const sensor_msgs__msg__JointState*)msgin;

  size_t n = msg->position.size > 0 ? msg->position.size : msg->position.capacity;
  if (n > 5) n = 5;

  // Pass 1: compute per-joint step counts and time-to-finish at each joint's
  // own full speed. The slowest joint becomes the "pacemaker" — it runs at
  // full speed and all others are scaled down so everyone finishes together.
  float steps_arr[5] = {0};
  float time_arr[5]  = {0};
  float max_time = 1.0f;
  for (size_t i = 0; i < n; i++) {
    float tgt = (float)msg->position.data[i];
    steps_arr[i] = fabsf((tgt - current_pos[i]) * steps_per_rad[i]);
    time_arr[i]  = steps_arr[i] * (float)step_interval_us[i];
    if (time_arr[i] > max_time) max_time = time_arr[i];
  }

  // Pass 2: apply targets, ramp resets, and per-joint speed fractions.
  for (size_t i = 0; i < n; i++) {
    float tgt = (float)msg->position.data[i];
    float steps = steps_arr[i];
    dynamic_speed_factor[i] = (steps >= 1.0f) ? (time_arr[i] / max_time) : 1.0f;

    // Ramp reset: if mid-move, skip re-acceleration by starting at peak velocity.
    if (steps >= 1.0f) {
      bool was_moving = (ramp_done[i] < ramp_total[i] * 0.99f);
      ramp_total[i] = steps;
      ramp_done[i]  = was_moving ? (steps * 0.5f) : 0.0f;
    }
    target_pos[i] = tgt;
  }
}

void home_callback(const void* msgin) {
  const std_msgs__msg__Float32* msg = (const std_msgs__msg__Float32*)msgin;
  if (msg->data < 0.5f) return;  // only act on 1.0
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
}

// Remote reboot. Publish data >= 0.5 to /reboot to trigger ESP.restart().
// Useful when the chip wedges and the reset button is hard to reach — the
// Pi can do it via:  ros2 topic pub --once /reboot std_msgs/msg/Float32 "{data: 1.0}"
// A short delay before restart lets micro-ROS ACK the message so the
// publisher doesn't see a transport error.
void reboot_callback(const void* msgin) {
  const std_msgs__msg__Float32* msg = (const std_msgs__msg__Float32*)msgin;
  if (msg->data < 0.5f) return;
  delay(100);
  ESP.restart();
}

void uros_cleanup() {
  RCSOFTCHECK(rclc_executor_fini(&executor));
  RCSOFTCHECK(rcl_subscription_fini(&reboot_subscriber, &node));
  RCSOFTCHECK(rcl_subscription_fini(&home_subscriber, &node));
  RCSOFTCHECK(rcl_subscription_fini(&speed_subscriber, &node));
  RCSOFTCHECK(rcl_subscription_fini(&subscriber, &node));
  RCSOFTCHECK(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_support_fini(&support));
}

void uros_init() {
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

  RCCHECK(rclc_subscription_init_best_effort(
    &reboot_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/reboot"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &subscriber, &joint_state_msg,
    &joint_state_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &speed_subscriber, &speed_msg,
    &speed_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &home_subscriber, &home_msg,
    &home_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &reboot_subscriber, &reboot_msg,
    &reboot_callback, ON_NEW_DATA));

  uros_ok = true;
}

void setup() {
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

  // micro-ROS over USB CDC. set_microros_transports() binds Serial as the
  // XRCE-DDS transport and calls Serial.begin(115200) internally. Anything
  // else writing to Serial after this point will corrupt the protocol stream.
  set_microros_transports();

  // Give the host (Pi-side micro-ROS agent) time to enumerate and open
  // /dev/ttyACM0 before we try to negotiate a session.
  delay(2000);
  uros_init();
}

void loop() {
  // Recompute "is any joint mid-move?" up front. Used to gate the periodic
  // ping so it never blocks the stepper for-loop.
  bool stepping_active = false;
  for (int i = 0; i < 5; i++) {
    if (fabsf((target_pos[i] - current_pos[i]) * steps_per_rad[i]) >= 1.0f) {
      stepping_active = true;
      break;
    }
  }

  // --- micro-ROS: spin executor + ping-based disconnect detection ---
  if (uros_ok) {
    rclc_executor_spin_some(&executor, 0);

    // Ping is gated while moving — see WiFi version commit history for why.
    static unsigned long last_ping_ms = 0;
    unsigned long now_ms = millis();
    if (!stepping_active && now_ms - last_ping_ms >= UROS_PING_INTERVAL_MS) {
      last_ping_ms = now_ms;
      if (rmw_uros_ping_agent(200, 1) != RMW_RET_OK) {
        uros_cleanup();
        uros_ok = false;
      }
    }
  } else {
    // Retry connecting to agent periodically
    static unsigned long last_reconnect_ms = 0;
    unsigned long now_ms = millis();
    if (now_ms - last_reconnect_ms >= UROS_RECONNECT_MS) {
      last_reconnect_ms = now_ms;
      uros_cleanup();
      // Serial transport: nothing to re-bind, just retry the session.
      uros_init();
    }
  }

  // --- Stepper loop ---
  for (int i = 0; i < 5; i++) {
    unsigned long now = micros();
    // Sinusoidal-ish ramp (linear up, flat, linear down).
    {
      float _prog = (ramp_total[i] > 0.0f) ? (ramp_done[i] / ramp_total[i]) : 0.5f;
      if (_prog > 0.999f) _prog = 0.999f;
      float _vf;
      if (_prog < RAMP_ACCEL_FRAC)
        _vf = RAMP_MIN_V + (1.0f - RAMP_MIN_V) * (_prog / RAMP_ACCEL_FRAC);
      else if (_prog > (1.0f - RAMP_DECEL_FRAC))
        _vf = RAMP_MIN_V + (1.0f - RAMP_MIN_V) * ((1.0f - _prog) / RAMP_DECEL_FRAC);
      else
        _vf = 1.0f;
      if (_vf < RAMP_MIN_V) _vf = RAMP_MIN_V;
      float _eff_scale = speed_scale < 0.01f ? 0.01f : speed_scale;
      float _dyn = dynamic_speed_factor[i] < 0.01f ? 0.01f : dynamic_speed_factor[i];
      unsigned long ramp_interval_us = (unsigned long)(step_interval_us[i] / (_vf * _eff_scale * _dyn));
      if (now - last_step_time[i] < ramp_interval_us) continue;
    }
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
        bool move_positive = (steps_needed > 0);
        bool dir = move_positive ^ DIR_INVERT[i];
        digitalWrite(dir_pin, dir ? HIGH : LOW);
        digitalWrite(step_pin, HIGH);
        delayMicroseconds(2);
        digitalWrite(step_pin, LOW);
        current_pos[i] += (move_positive ? 1.0f : -1.0f) / steps_per_rad[i];
        ramp_done[i]++;
      }
      last_step_time[i] = now;
    }
  }
}
