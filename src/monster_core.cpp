#include "Monster.h"

// ---- micro-ROS ----
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

namespace MonsterCore {

// ---- Instances ----
FourWheelMotor    motors;
MonsterSensor     sensors;
MonsterController controller;
MonsterDiagnosis  diagnosis;

// -------------------- PLAIN MODE --------------------
static uint32_t plain_next_us = 0;

void beginPlain(const char* model_name) {
  (void)model_name;

  motors.begin();
  motors.configureAll();
  sensors.begin();
  controller.init(MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY);
  controller.attachMotor(&motors);

  diagnosis.init();
  diagnosis.setMode(MonsterDiagnosis::Mode::HEARTBEAT);

  plain_next_us = micros();
}

void runPlain() {
  uint32_t now = micros();
  if ((int32_t)(now - plain_next_us) >= 0) {
    plain_next_us += CONTROL_PERIOD;
    sensors.update();
    controller.update();
    diagnosis.tick();
  }
}

// -------------------- ROS2 MODE --------------------
static rcl_allocator_t          g_alloc;
static rclc_support_t           g_support;
static rcl_node_t               g_node;
static rcl_subscription_t       g_sub_cmdvel;
static geometry_msgs__msg__Twist g_twist;
static rclc_executor_t          g_exec;
static bool                     g_ros_ok = false;

static float    g_v_cmd = 0.0f;
static float    g_w_cmd = 0.0f;
static uint32_t g_last_cmd_ms = 0;
static const    uint32_t CMD_TIMEOUT_MS = 500;
static const    float LOW_BATT_V   = 10.8f;
static const    float SONAR_STOP_M = 0.20f;
static uint32_t ros_next_us = 0;

static void cmdvel_cb(const void* msgin) {
  auto* msg = (const geometry_msgs__msg__Twist*)msgin;
  g_v_cmd = msg->linear.x;
  g_w_cmd = msg->angular.z;
  g_last_cmd_ms = millis();
}

static void apply_safety_and_drive() {
  sensors.update();

  bool timed_out     = (millis() - g_last_cmd_ms) > CMD_TIMEOUT_MS;
  float vbatt        = sensors.batteryVoltage();
  float sonar_m      = sensors.sonarMeters();
  bool low_batt_hold = (vbatt > 0.1f && vbatt < LOW_BATT_V);
  bool obstacle_hold = (sonar_m > 0.0f && sonar_m < SONAR_STOP_M);

  float v = (timed_out || low_batt_hold) ? 0.0f : g_v_cmd;
  float w = (timed_out || low_batt_hold) ? 0.0f : g_w_cmd;
  if (obstacle_hold) v = 0.0f;

  if (v >  MAX_LINEAR_VELOCITY)  v =  MAX_LINEAR_VELOCITY;
  if (v < -MAX_LINEAR_VELOCITY)  v = -MAX_LINEAR_VELOCITY;
  if (w >  MAX_ANGULAR_VELOCITY) w =  MAX_ANGULAR_VELOCITY;
  if (w < -MAX_ANGULAR_VELOCITY) w = -MAX_ANGULAR_VELOCITY;

  controller.setGoal(v, w);
  controller.update();

  if (low_batt_hold) {
    diagnosis.flashError(2);
  } else {
    diagnosis.setConnected(!timed_out);
  }
  diagnosis.tick();
}

void beginROS2(const char* model_name) {
  (void)model_name;

  set_microros_transports();

  motors.begin();
  motors.configureAll();
  sensors.begin();
  controller.init(MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY);
  controller.attachMotor(&motors);
  diagnosis.init();
  diagnosis.setMode(MonsterDiagnosis::Mode::HEARTBEAT);

  delay(150);

  g_alloc = rcl_get_default_allocator();
  rcl_ret_t rc = rclc_support_init(&g_support, 0, nullptr, &g_alloc);
  if (rc != RCL_RET_OK) { g_ros_ok = false; return; }

  rc = rclc_node_init_default(&g_node, "monster_opencr", "", &g_support);
  if (rc != RCL_RET_OK) { g_ros_ok = false; return; }

  rc = rclc_subscription_init_default(
          &g_sub_cmdvel, &g_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
          "cmd_vel");
  if (rc != RCL_RET_OK) { g_ros_ok = false; return; }

  rc = rclc_executor_init(&g_exec, &g_support.context, 1, &g_alloc);
  if (rc != RCL_RET_OK) { g_ros_ok = false; return; }

  rclc_executor_add_subscription(&g_exec, &g_sub_cmdvel, &g_twist, &cmdvel_cb, ON_NEW_DATA);

  g_last_cmd_ms = millis();
  ros_next_us   = micros();
  g_ros_ok      = true;
}

void spinROS2() {
  if (g_ros_ok) {
    rclc_executor_spin_some(&g_exec, RCL_MS_TO_NS(5));
  } else {
    static uint32_t last_try = 0;
    if (millis() - last_try > 1000) {
      last_try = millis();
      beginROS2();
    }
    delay(5);
  }

  uint32_t now = micros();
  if ((int32_t)(now - ros_next_us) >= 0) {
    ros_next_us += CONTROL_PERIOD;
    apply_safety_and_drive();
  }
}

} // namespace MonsterCore
