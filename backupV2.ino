/********************************************************************************************
 * Monster micro-ROS (OpenCR) — Minimal: /imu + /cmd_vel (sub & pub SAME TOPIC)
 * ------------------------------------------------------------------------------------------
 * SUBSCRIBE:
 *   - /cmd_vel    geometry_msgs/Twist   (linear.x, angular.z)  → updates motor controller
 *
 * PUBLISH:
 *   - /imu        sensor_msgs/Imu       @ 50 Hz
 *   - /cmd_vel    geometry_msgs/Twist   @ 50 Hz (APPLIED command on SAME TOPIC)
 *       - If no cmd received within CMD_TIMEOUT_MS → publish zeros
 *       - When a new cmd arrives → controller updates immediately; periodic publisher
 *         will reflect it (same /cmd_vel topic)
 *
 * IMPORTANT:
 *   - Do NOT use Serial.print after micro-ROS transport is set (Serial is XRCE).
 *   - Ensure ROS_DOMAIN_ID matches host.
 *
 * HOST (two shells, same ROS_DOMAIN_ID):
 *   export ROS_DOMAIN_ID=35
 *   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/serial/by-id/usb-ROBOTIS_OpenCR-if00 -v6
 *
 *   export ROS_DOMAIN_ID=35
 *   ros2 topic echo /imu
 *   ros2 topic echo /cmd_vel
 *   ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.15}, angular: {z: 0.0}}"
 ********************************************************************************************/

#include <Arduino.h>
#include <Monster.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>

#include <rosidl_runtime_c/string_functions.h>
#include <rmw/qos_profiles.h>

using namespace MonsterCore;

// --------------------------- Config --------------------------- //
#define ROS_DOMAIN_ID_SET     35
#define PUB_IMU_HZ            50
#define PUB_CMD_HZ            50
#define CMD_TIMEOUT_MS        500    // publish zeros if no cmd within this window
static const char* FRAME_IMU = "imu_link";

// ------------------------ micro-ROS handles ------------------- //
static rcl_allocator_t        g_alloc;
static rclc_support_t         g_support;
static rcl_node_t             g_node;

static rcl_subscription_t     sub_cmd_in;
static geometry_msgs__msg__Twist msg_cmd_in;

static rcl_publisher_t        pub_cmd_out;     // same topic: /cmd_vel
static geometry_msgs__msg__Twist msg_cmd_out;
static rcl_timer_t            tmr_cmd;

static rcl_publisher_t        pub_imu;
static sensor_msgs__msg__Imu  msg_imu;
static rcl_timer_t            tmr_imu;

static rclc_executor_t        g_exec;

// ----------------------------- State -------------------------- //
static float v_applied = 0.0f;       // current applied linear.x
static float w_applied = 0.0f;       // current applied angular.z
static uint32_t last_cmd_ms = 0;     // last time a cmd was received

// ---------------------------- Helpers ------------------------- //
static inline void zero_stamp(builtin_interfaces__msg__Time &t) { t.sec = 0; t.nanosec = 0; }
static inline void set_cmd(float v, float w) {
  v_applied = v; w_applied = w;
  controller.setGoal(v_applied, w_applied);
  controller.update();               // push to motors immediately
}

// --------------------------- Callbacks ------------------------ //
// Incoming /cmd_vel
static void cb_cmd_in(const void* msg_in) {
  const auto* tw = static_cast<const geometry_msgs__msg__Twist*>(msg_in);
  last_cmd_ms = millis();
  set_cmd((float)tw->linear.x, (float)tw->angular.z);
}

// Periodic publisher for /cmd_vel (same topic) with watchdog
static void cb_cmd_timer(rcl_timer_t*, int64_t) {
  // Watchdog: if stale, force zeros
  const uint32_t now = millis();
  if (now - last_cmd_ms > CMD_TIMEOUT_MS) {
    if (v_applied != 0.0f || w_applied != 0.0f) {
      set_cmd(0.0f, 0.0f);
    }
  }

  // Publish the applied command on /cmd_vel
  msg_cmd_out.linear.x  = v_applied;
  msg_cmd_out.linear.y  = 0.0;
  msg_cmd_out.linear.z  = 0.0;
  msg_cmd_out.angular.x = 0.0;
  msg_cmd_out.angular.y = 0.0;
  msg_cmd_out.angular.z = w_applied;

  rcl_publish(&pub_cmd_out, &msg_cmd_out, NULL);
}

// IMU publisher
static void cb_imu_timer(rcl_timer_t*, int64_t) {
  sensors.update();

  const float* q = sensors.quat();
  const float* g = sensors.gyro();
  const float* a = sensors.accel();

  zero_stamp(msg_imu.header.stamp);
  rosidl_runtime_c__String__assign(&msg_imu.header.frame_id, FRAME_IMU);

  msg_imu.orientation.x = q[0];
  msg_imu.orientation.y = q[1];
  msg_imu.orientation.z = q[2];
  msg_imu.orientation.w = q[3];

  msg_imu.angular_velocity.x = g[0];
  msg_imu.angular_velocity.y = g[1];
  msg_imu.angular_velocity.z = g[2];

  msg_imu.linear_acceleration.x = a[0];
  msg_imu.linear_acceleration.y = a[1];
  msg_imu.linear_acceleration.z = a[2];

  // Mark orientation unknown (let downstream fuse accel/gyro if needed)
  msg_imu.orientation_covariance[0] = -1.0;

  rcl_publish(&pub_imu, &msg_imu, NULL);
}

// ------------------------------- Setup ------------------------ //
void setup() {
  // Early debug only (don’t print after transport):
  Serial.begin(115200);
  delay(30);

  // Hardware bring-up
  motors.begin();
  motors.configureAll();
  controller.attachMotor(&motors);
  sensors.begin();
  sensors.calibrateGyro();
  diagnosis.init();
  diagnosis.setMode(monster_diagnosis::Mode::HEARTBEAT);

  // micro-ROS transport (works on your OpenCR)
  set_microros_transports();
  // After this call: DO NOT use Serial.print

  // micro-ROS core + domain
  g_alloc = rcl_get_default_allocator();
  rcl_init_options_t opts = rcl_get_zero_initialized_init_options();
  if (rcl_init_options_init(&opts, g_alloc) != RCL_RET_OK) { return; }
  if (rcl_init_options_set_domain_id(&opts, ROS_DOMAIN_ID_SET) != RCL_RET_OK) { return; }
  if (rclc_support_init_with_options(&g_support, 0, NULL, &opts, &g_alloc) != RCL_RET_OK) { return; }
  rcl_init_options_fini(&opts);

  // Node
  if (rclc_node_init_default(&g_node, "tb3_min_imu_cmdvel_node", "", &g_support) != RCL_RET_OK) { return; }

  // QoS
  // Use RELIABLE so CLI tools and nav stacks match by default.
  rmw_qos_profile_t qos_rel = rmw_qos_profile_default;
  qos_rel.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  qos_rel.history     = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  qos_rel.depth       = 10;

  // Subscriber: /cmd_vel (reliable)
  if (rclc_subscription_init(
        &sub_cmd_in, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel", &qos_rel) != RCL_RET_OK) { return; }

  // Publisher: /cmd_vel (same topic, reliable)
  if (rclc_publisher_init(
        &pub_cmd_out, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel", &qos_rel) != RCL_RET_OK) { return; }

  // Publisher: /imu (reliable)
  if (rclc_publisher_init(
        &pub_imu, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu", &qos_rel) != RCL_RET_OK) { return; }

  // Timers
  const uint64_t ns_cmd = (uint64_t)(1000000000ULL / PUB_CMD_HZ);
  const uint64_t ns_imu = (uint64_t)(1000000000ULL / PUB_IMU_HZ);

  if (rclc_timer_init_default(&tmr_cmd, &g_support, ns_cmd, &cb_cmd_timer) != RCL_RET_OK) { return; }
  if (rclc_timer_init_default(&tmr_imu, &g_support, ns_imu, &cb_imu_timer) != RCL_RET_OK) { return; }

  // Executor (1 sub + 2 timers = 3 → give headroom 5)
  if (rclc_executor_init(&g_exec, &g_support.context, 5, &g_alloc) != RCL_RET_OK) { return; }
  if (rclc_executor_add_subscription(&g_exec, &sub_cmd_in, &msg_cmd_in, &cb_cmd_in, ON_NEW_DATA) != RCL_RET_OK) { return; }
  if (rclc_executor_add_timer(&g_exec, &tmr_cmd) != RCL_RET_OK) { return; }
  if (rclc_executor_add_timer(&g_exec, &tmr_imu) != RCL_RET_OK) { return; }

  // Initialize applied cmd to zeros and “pretend” a recent command so we publish immediately
  last_cmd_ms = millis();
  set_cmd(0.0f, 0.0f);
}

// -------------------------------- Loop -------------------------------------- //
void loop() {
  rclc_executor_spin_some(&g_exec, RCL_MS_TO_NS(2));
}
