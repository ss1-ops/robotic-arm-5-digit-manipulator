// ESP32-S3 micro-ROS comms test — single-threaded, no steppers
//
// Same structure as the original working firmware:
//   setup() → Serial.begin → WiFi → set_microros_transports → uros_init
//   loop()  → spin executor → reconnect on failure
//
// UDP debug: on Pi run:  nc -ul 9999

// Arduino loopTask default stack is only 8192 bytes — rclc_support_init needs more.
// This must be at global scope before any includes.
#if defined(ESP32)
uint32_t loopTaskStackSize = 16384;
#endif

#include <stdarg.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>
#include <WiFi.h>

const char* ssid     = "WAVLINK-N";
const char* password = "Jk12345678";

// UDP debug — listen on Pi with: nc -ul 9999
#define DEBUG_HOST "192.168.1.142"
#define DEBUG_PORT 9999
WiFiUDP debugUDP;
bool wifi_ok = false;

void dbg(const char* fmt, ...) {
  if (!wifi_ok) return;
  char buf[160];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  debugUDP.beginPacket(DEBUG_HOST, DEBUG_PORT);
  debugUDP.write((const uint8_t*)buf, strlen(buf));
  debugUDP.endPacket();
}

// micro-ROS objects
rcl_subscription_t            subscriber;
sensor_msgs__msg__JointState  joint_state_msg;
rclc_executor_t               executor;
rclc_support_t                support;
rcl_allocator_t               allocator;
rcl_node_t                    node;

double   js_pos[5], js_vel[5], js_eff[5];
rosidl_runtime_c__String js_names[5];
char     js_name_buf[5][32];

bool uros_ok = false;
unsigned long uptime_s = 0;
unsigned long last_alive_ms = 0;

#define RCCHECK(fn) { if ((fn) != RCL_RET_OK) { uros_ok = false; return; } }

void joint_state_callback(const void* msgin) {
  const sensor_msgs__msg__JointState* msg =
    (const sensor_msgs__msg__JointState*)msgin;
  size_t n = msg->position.size < 5 ? msg->position.size : 5;
  dbg("CB recv n=%d pos0=%.3f", (int)n, n > 0 ? msg->position.data[0] : 0.0);
}

void uros_cleanup() {
  rcl_ret_t ret;
  ret = rclc_executor_fini(&executor);
  ret = rcl_subscription_fini(&subscriber, &node);
  ret = rcl_node_fini(&node);
  ret = rclc_support_fini(&support);
  (void)ret;
}

void uros_init() {
  dbg("uros_init start heap=%d", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  dbg("STEP1 OK");

  RCCHECK(rclc_node_init_default(&node, "moveo_esp32", "", &support));
  dbg("STEP2 OK");

  RCCHECK(rclc_subscription_init_default(
    &subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/joint_commands"));
  dbg("STEP3 OK");

  joint_state_msg.position.data     = js_pos;   joint_state_msg.position.capacity = 5;
  joint_state_msg.velocity.data     = js_vel;   joint_state_msg.velocity.capacity = 5;
  joint_state_msg.effort.data       = js_eff;   joint_state_msg.effort.capacity   = 5;
  joint_state_msg.name.data         = js_names; joint_state_msg.name.capacity     = 5;
  for (int i = 0; i < 5; i++) {
    js_names[i].data = js_name_buf[i]; js_names[i].size = 0; js_names[i].capacity = 32;
  }

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &subscriber, &joint_state_msg,
    &joint_state_callback, ON_NEW_DATA));
  dbg("uros_init DONE heap=%d", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));

  uros_ok = true;
}

void setup() {
  // Same order as original working firmware
  Serial.begin(1000000);

  // WiFi first — matches original (10s timeout)
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
    delay(200);
  }
  wifi_ok = (WiFi.status() == WL_CONNECTED);
  dbg("WiFi %s IP=%s heap=%d",
      wifi_ok ? "OK" : "FAILED",
      wifi_ok ? WiFi.localIP().toString().c_str() : "N/A",
      heap_caps_get_free_size(MALLOC_CAP_DEFAULT));

  // Transport AFTER WiFi — matches original
  set_microros_transports();
  delay(2000);

  uros_init();
}

void loop() {
  if (uros_ok) {
    if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)) != RCL_RET_OK) {
      dbg("spin fail - cleanup");
      uros_cleanup();
      uros_ok = false;
    }
  } else {
    static unsigned long last_reconnect_ms = 0;
    if (millis() - last_reconnect_ms >= 3000) {
      last_reconnect_ms = millis();
      dbg("reconnect attempt heap=%d", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
      uros_init();   // no cleanup, no transport reset — matches original working firmware
    }
  }

  // Alive heartbeat every 5s
  if (millis() - last_alive_ms >= 5000) {
    last_alive_ms = millis();
    uptime_s += 5;
    dbg("ALIVE uptime=%lus uros_ok=%d heap=%d",
        uptime_s, (int)uros_ok, heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
  }
}
