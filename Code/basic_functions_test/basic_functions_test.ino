// ESP32-S3 micro-ROS for BCN3D MOVEO (5-Axis) - Absolute Positioning
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/bool.h>

// ====================== Pin Definitions (5-Axis) ======================
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

// Current and target absolute positions (radians, 0 = home)
float current_pos[5] = {0.0};
float target_pos[5] = {0.0};

// Timing for non-blocking stepping
unsigned long last_step_time[5] = {0};
const unsigned long step_interval = 1200;   // microseconds - increase to slow down

// ROS 2 variables
rcl_subscription_t command_sub;
rcl_subscription_t home_sub;
std_msgs__msg__Float64MultiArray command_msg;
std_msgs__msg__Bool home_msg;
rclc_executor_t executor;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;

// Callback for normal joint commands (absolute positions)
void command_callback(const void * msgin) {
  const std_msgs__msg__Float64MultiArray * received = (const std_msgs__msg__Float64MultiArray *)msgin;
  
  Serial.print("Received absolute targets: ");
  for (size_t i = 0; i < received->data.size; i++) {
    target_pos[i] = received->data.data[i];
    Serial.print(target_pos[i], 3);
    Serial.print(" ");
  }
  Serial.println();
}

// Callback for homing command
void home_callback(const void * msgin) {
  const std_msgs__msg__Bool * received = (const std_msgs__msg__Bool *)msgin;
  if (received->data) {
    for (int i = 0; i < 5; i++) {
      current_pos[i] = 0.0;
      target_pos[i] = 0.0;
    }
    Serial.println("Homing command received - All joints set to 0 (home position)");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Setup pins
  pinMode(WAIST_STEP_PIN, OUTPUT);
  pinMode(WAIST_DIR_PIN, OUTPUT);
  pinMode(SHOULDER_STEP_PIN, OUTPUT);
  pinMode(SHOULDER_DIR_PIN, OUTPUT);
  pinMode(ELBOW_STEP_PIN, OUTPUT);
  pinMode(ELBOW_DIR_PIN, OUTPUT);
  pinMode(WRIST_ROLL_STEP_PIN, OUTPUT);
  pinMode(WRIST_ROLL_DIR_PIN, OUTPUT);
  pinMode(WRIST_PITCH_STEP_PIN, OUTPUT);
  pinMode(WRIST_PITCH_DIR_PIN, OUTPUT);

  // Initialize micro-ROS over USB
  set_microros_transports();

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "moveo_esp32_node", "", &support);

  // Subscription for absolute joint commands
  rclc_subscription_init_default(
    &command_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "/forward_position_controller/commands"
  );

  // Subscription for homing command
  rclc_subscription_init_default(
    &home_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/homing"
  );

  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &command_sub, &command_msg, &command_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &home_sub, &home_msg, &home_callback, ON_NEW_DATA);

  Serial.println("ESP32-S3 micro-ROS Ready - Absolute Positioning Mode");
  Serial.println("Waiting for absolute joint targets and homing commands...");
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

  // Non-blocking stepper movement toward absolute targets
  unsigned long now = micros();
  for (int i = 0; i < 5; i++) {
    if (abs(target_pos[i] - current_pos[i]) > 0.01 && (now - last_step_time[i] > step_interval)) {
      int step_pin = -1, dir_pin = -1;
      switch (i) {
        case 0: step_pin = WAIST_STEP_PIN;        dir_pin = WAIST_DIR_PIN; break;
        case 1: step_pin = SHOULDER_STEP_PIN;     dir_pin = SHOULDER_DIR_PIN; break;
        case 2: step_pin = ELBOW_STEP_PIN;        dir_pin = ELBOW_DIR_PIN; break;
        case 3: step_pin = WRIST_ROLL_STEP_PIN;   dir_pin = WRIST_ROLL_DIR_PIN; break;
        case 4: step_pin = WRIST_PITCH_STEP_PIN;  dir_pin = WRIST_PITCH_DIR_PIN; break;
      }
      if (step_pin != -1) {
        digitalWrite(dir_pin, target_pos[i] > current_pos[i] ? HIGH : LOW);
        digitalWrite(step_pin, HIGH);
        delayMicroseconds(10);
        digitalWrite(step_pin, LOW);
        current_pos[i] += (target_pos[i] > current_pos[i] ? 0.001 : -0.001);
        last_step_time[i] = now;
      }
    }
  }
  delay(5);
}