/********************************************************************************************
 * Monster micro-ROS (OpenCR) — Minimal: /imu + /cmd_vel (RX-only) + /odom
 * - SUBSCRIBE: /cmd_vel  geometry_msgs/Twist  → updates motor controller
 * - PUBLISH:   /imu      sensor_msgs/Imu      @ 50 Hz
 * - PUBLISH:   /odom     nav_msgs/Odometry    @ 50 Hz
 *
 * Notes:
 *  - Keep streaming /cmd_vel faster than CMD_TIMEOUT_MS or increase the timeout.
 *  - Do NOT use Serial.print after set_microros_transports() (Serial becomes XRCE).
 ********************************************************************************************/

#include <Arduino.h>
#include <Monster.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <nav_msgs/msg/odometry.h>

#include <rosidl_runtime_c/string_functions.h>
#include <rmw/qos_profiles.h>
#include <math.h>

using namespace MonsterCore;

// --------------------------- Config --------------------------- //
#define ROS_DOMAIN_ID_SET     35
#define PUB_IMU_HZ            5
#define PUB_CMD_HZ            100
#define PUB_ODOM_HZ           100
#define CMD_TIMEOUT_MS        500    // auto-stop if no /cmd_vel arrives within this window

static const char* FRAME_IMU   = "imu_link";
static const char* FRAME_ODOM  = "odom";
static const char* FRAME_BASE  = "base_footprint";

// ------------------------ micro-ROS handles ------------------- //
static rcl_allocator_t        g_alloc;
static rclc_support_t         g_support;
static rcl_node_t             g_node;

static rcl_subscription_t     sub_cmd_in;
static geometry_msgs__msg__Twist msg_cmd_in;

static rcl_timer_t            tmr_cmd;

static rcl_publisher_t        pub_imu;
static sensor_msgs__msg__Imu  msg_imu;
static rcl_timer_t            tmr_imu;

static rcl_publisher_t        pub_odom;
static nav_msgs__msg__Odometry msg_odom;
static rcl_timer_t            tmr_odom;

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

static inline void yaw_to_quat(float yaw, float& qx, float& qy, float& qz, float& qw) {
  const float h = 0.5f * yaw;
  qx = 0.0f; qy = 0.0f;
  qz = sinf(h);
  qw = cosf(h);
}

// wheel→body twist using your odom sensor wheel rates (rad/s)
static inline void compute_body_twist(float& v, float& w) {
  const float wl = 0.5f * (sensors.wheelVelRad(0) + sensors.wheelVelRad(1));
  const float wr = 0.5f * (sensors.wheelVelRad(2) + sensors.wheelVelRad(3));
  v = WHEEL_RADIUS * 0.5f * (ODOM_SCALE_L * wl + ODOM_SCALE_R * wr);
  w = WHEEL_RADIUS * (ODOM_SCALE_R * wr - ODOM_SCALE_L * wl) / WHEEL_SEPARATION;
}


// --------------------------- Callbacks ------------------------ //
// Incoming /cmd_vel (subscriber)
static void cb_cmd_in(const void* msg_in) {
  const auto* tw = static_cast<const geometry_msgs__msg__Twist*>(msg_in);
  last_cmd_ms = millis();
  set_cmd((float)tw->linear.x, (float)tw->angular.z);
}

// Periodic controller service + watchdog (NO publishing on /cmd_vel)
static void cb_cmd_timer(rcl_timer_t*, int64_t) {
  const uint32_t now = millis();
  if (now - last_cmd_ms > CMD_TIMEOUT_MS) {
    if (v_applied != 0.0f || w_applied != 0.0f) set_cmd(0.0f, 0.0f);
  } else {
    controller.update();
  }
}

// IMU publisher
static void cb_imu_timer(rcl_timer_t*, int64_t) {
  //sensors.update();  // updates IMU and keeps odometry integration fresh

  const float* q = sensors.quat();
  const float* g = sensors.gyro();
  const float* a = sensors.accel();

  zero_stamp(msg_imu.header.stamp);  // SBC can stamp later

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

  msg_imu.orientation_covariance[0] = -1.0;  // unknown orientation
  rcl_publish(&pub_imu, &msg_imu, NULL);
}

// ODOM publisher
static void cb_odom_timer(rcl_timer_t*, int64_t) {
  sensors.update();  // ensure encoders integrated frequently

  // Pose
  const float x  = sensors.odomX();
  const float y  = sensors.odomY();
  const float th = sensors.odomTheta();

  // Twist
  float v, w; compute_body_twist(v, w);

  // Header
  zero_stamp(msg_odom.header.stamp);

  // Pose (x,y,theta)
  msg_odom.pose.pose.position.x = x;
  msg_odom.pose.pose.position.y = y;
  msg_odom.pose.pose.position.z = 0.0f;

  float qx,qy,qz,qw; yaw_to_quat(th, qx,qy,qz,qw);
  msg_odom.pose.pose.orientation.x = qx;
  msg_odom.pose.pose.orientation.y = qy;
  msg_odom.pose.pose.orientation.z = qz;
  msg_odom.pose.pose.orientation.w = qw;

  // Twist (v,w)
  msg_odom.twist.twist.linear.x  = v;
  msg_odom.twist.twist.linear.y  = 0.0f;
  msg_odom.twist.twist.linear.z  = 0.0f;
  msg_odom.twist.twist.angular.x = 0.0f;
  msg_odom.twist.twist.angular.y = 0.0f;
  msg_odom.twist.twist.angular.z = w;

  rcl_publish(&pub_odom, &msg_odom, NULL);
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
  if (rclc_node_init_default(&g_node, "tb3_min_imu_cmdvel_odom_rx_node", "", &g_support) != RCL_RET_OK) { return; }

  // QoS: RELIABLE so CLI tools and nav stacks match by default.
  rmw_qos_profile_t qos_rel = rmw_qos_profile_default;
  qos_rel.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  qos_rel.history     = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  qos_rel.depth       = 10;

  // Subscriber: /cmd_vel (reliable)
  if (rclc_subscription_init(
        &sub_cmd_in, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel", &qos_rel) != RCL_RET_OK) { return; }

  // Publisher: /imu (reliable)
  if (rclc_publisher_init(
        &pub_imu, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu", &qos_rel) != RCL_RET_OK) { return; }

  // Publisher: /odom (reliable)
  if (rclc_publisher_init(
        &pub_odom, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom", &qos_rel) != RCL_RET_OK) { return; }

  // Timers
  const uint64_t ns_cmd  = (uint64_t)(1000000000ULL / PUB_CMD_HZ);
  const uint64_t ns_imu  = (uint64_t)(1000000000ULL / PUB_IMU_HZ);
  const uint64_t ns_odom = (uint64_t)(1000000000ULL / PUB_ODOM_HZ);

  if (rclc_timer_init_default(&tmr_cmd,  &g_support, ns_cmd,  &cb_cmd_timer)  != RCL_RET_OK) { return; }
  if (rclc_timer_init_default(&tmr_imu,  &g_support, ns_imu,  &cb_imu_timer)  != RCL_RET_OK) { return; }
  if (rclc_timer_init_default(&tmr_odom, &g_support, ns_odom, &cb_odom_timer) != RCL_RET_OK) { return; }

  // Executor (1 sub + 3 timers)
  if (rclc_executor_init(&g_exec, &g_support.context, 6, &g_alloc) != RCL_RET_OK) { return; }
  if (rclc_executor_add_subscription(&g_exec, &sub_cmd_in, &msg_cmd_in, &cb_cmd_in, ON_NEW_DATA) != RCL_RET_OK) { return; }
  if (rclc_executor_add_timer(&g_exec, &tmr_cmd)  != RCL_RET_OK) { return; }
  if (rclc_executor_add_timer(&g_exec, &tmr_imu)  != RCL_RET_OK) { return; }
  if (rclc_executor_add_timer(&g_exec, &tmr_odom) != RCL_RET_OK) { return; }

  // Static fields once (avoid per-tick allocations)
  rosidl_runtime_c__String__assign(&msg_imu.header.frame_id,  FRAME_IMU);
  rosidl_runtime_c__String__assign(&msg_odom.header.frame_id, FRAME_ODOM);
  rosidl_runtime_c__String__assign(&msg_odom.child_frame_id,  FRAME_BASE);

  // Covariances (TB3-style)
  for (int i=0;i<36;i++) { msg_odom.pose.covariance[i]=0.0; msg_odom.twist.covariance[i]=0.0; }
  msg_odom.pose.covariance[0]  = 1e-3;   // x
  msg_odom.pose.covariance[7]  = 1e-3;   // y
  msg_odom.pose.covariance[14] = 1e6;    // z
  msg_odom.pose.covariance[21] = 1e6;    // roll
  msg_odom.pose.covariance[28] = 1e6;    // pitch
  msg_odom.pose.covariance[35] = 1e-3;   // yaw

  msg_odom.twist.covariance[0]  = 1e-3;  // vx
  msg_odom.twist.covariance[7]  = 1e-3;  // vy
  msg_odom.twist.covariance[14] = 1e6;   // vz
  msg_odom.twist.covariance[21] = 1e6;   // wx
  msg_odom.twist.covariance[28] = 1e6;   // wy
  msg_odom.twist.covariance[35] = 1e-3;  // wz

  // Seed zeros so robot is idle until first /cmd_vel
  last_cmd_ms = millis();
  set_cmd(0.0f, 0.0f);
}

// -------------------------------- Loop -------------------------------------- //
void loop() {
  rclc_executor_spin_some(&g_exec, RCL_MS_TO_NS(2));
}
