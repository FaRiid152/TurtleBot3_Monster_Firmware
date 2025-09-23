/********************************************************************************************
 *  Monster TB3-Compatible Firmware (micro-ROS, OpenCR)
 *  -----------------------------------------------------------------------------------------
 *  This sketch makes your Monster robot speak the same ROS 2 language as TurtleBot3 Burger:
 *
 *   SUBSCRIBE:
 *     - /cmd_vel                  geometry_msgs/Twist     (linear.x, angular.z)
 *
 *   PUBLISH:
 *     - /odom                     nav_msgs/Odometry       (pose + twist)
 *     - /imu                      sensor_msgs/Imu         (orientation, ang vel, lin acc)
 *     - /magnetic_field           sensor_msgs/MagneticField
 *     - /joint_states             sensor_msgs/JointState  (wheel positions/velocities)
 *     - /battery_state            sensor_msgs/BatteryState (voltage)
 *
 *  NOTES:
 *   - /scan, /tf, /tf_static, /robot_description are typically produced on the HOST (Pi/PC)
 *     by the LIDAR driver and robot_state_publisher; they’re not MCU responsibilities.
 *
 *  BOARD:
 *    - OpenCR R1.0 (USB CDC serial transport for micro-ROS)
 *
 *  BUILD PREREQS:
 *    - Arduino core for OpenCR, micro_ros_arduino library
 *    - Your Monster headers/sources available (Monster.h, etc.)
 *
 *  RUNTIME:
 *    1) Flash with BOOT=ON, then set BOOT=OFF and RESET to run.
 *    2) On host (same domain as below), run micro-ROS agent:
 *         export ROS_DOMAIN_ID=0
 *         ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/serial/by-id/usb-ROBOTIS_OpenCR-if00 -v6
 *    3) ros2 topic list / echo the topics above.
 ********************************************************************************************/

// --------------------------------- Includes --------------------------------- //
#include <Arduino.h>
#include <Monster.h>  // brings in MonsterCore::{motors,controller,sensors,diagnosis}

// micro-ROS & RCL/C
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// ROS 2 message types
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/battery_state.h>

// Helpers for JointState string/primitive sequences
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

using namespace MonsterCore;

// ----------------------------- User Configuration ---------------------------- //
// Domain ID must match the host (export ROS_DOMAIN_ID=<same>)
#define ROS_DOMAIN_ID_SET          0          // TB3 default is 0
// Control and publish rates
#define CONTROL_HZ                 125        // safety + motor update loop
#define PUB_ODOM_HZ                50
#define PUB_IMU_HZ                 50
#define PUB_MAG_HZ                 50
#define PUB_JOINTS_HZ              50
#define PUB_BATTERY_HZ             1

// Frame and joint names (TB3-ish)
static const char* FRAME_ODOM  = "odom";
static const char* FRAME_BASE  = "base_footprint";
static const char* FRAME_IMU   = "imu_link";
static const char* JOINT_LEFT  = "wheel_left_joint";
static const char* JOINT_RIGHT = "wheel_right_joint";

// Safety thresholds
static const float SAFETY_CMD_TIMEOUT_MS = 500.0f;  // if stale cmd_vel -> stop
static const float SAFETY_SONAR_MIN_M    = 0.20f;   // stop linear if obstacle < 20cm
static const float SAFETY_VBAT_MIN_V     = 10.8f;   // 3S Li-ion conservative cutoff

// Geometry (prefer your headers; keep fallbacks to compile standalone)
#ifndef WHEEL_RADIUS
#  define WHEEL_RADIUS      0.033f
#endif
#ifndef WHEEL_SEPARATION
#  define WHEEL_SEPARATION  0.160f
#endif

// ----------------------------- micro-ROS Handles ----------------------------- //
static rcl_allocator_t  g_alloc;
static rclc_support_t   g_support;
static rcl_node_t       g_node;

static rcl_subscription_t   sub_cmd_vel;
static geometry_msgs__msg__Twist msg_cmd_vel;

static rcl_publisher_t      pub_odom;
static nav_msgs__msg__Odometry msg_odom;

static rcl_publisher_t      pub_imu;
static sensor_msgs__msg__Imu msg_imu;

static rcl_publisher_t      pub_mag;
static sensor_msgs__msg__MagneticField msg_mag;

static rcl_publisher_t      pub_joint;
static sensor_msgs__msg__JointState msg_joint;

static rcl_publisher_t      pub_batt;
static sensor_msgs__msg__BatteryState msg_batt;

static rcl_timer_t          tmr_control;   // 125 Hz control/safety loop
static rcl_timer_t          tmr_odom;
static rcl_timer_t          tmr_imu;
static rcl_timer_t          tmr_mag;
static rcl_timer_t          tmr_joints;
static rcl_timer_t          tmr_batt;

static rclc_executor_t      g_exec;

// --------------------------------- State ------------------------------------ //
// Goals from /cmd_vel
static float v_goal = 0.0f;              // linear m/s
static float w_goal = 0.0f;              // angular rad/s
static uint32_t last_cmd_ms = 0;

// Odometry state (integrated from (v,w); for higher fidelity, integrate wheel feedback)
static float x_odom = 0.0f;
static float y_odom = 0.0f;
static float th_odom = 0.0f;

// Joint positions/velocities (rad, rad/s)
static double pos_left = 0.0;
static double pos_right = 0.0;
static double vel_left = 0.0;
static double vel_right = 0.0;

// --------------------------------- Helpers ---------------------------------- //
static inline void yaw_to_quat(float yaw, double &qx, double &qy, double &qz, double &qw) {
  const double h = 0.5 * (double)yaw;
  qx = 0.0; qy = 0.0;
  qz = sin(h);
  qw = cos(h);
}

// (Optional) time stamps; zero is acceptable if no time source is available.
// If you synchronize time via rmw_uros_epoch_nanos(), you can populate stamps.
static inline void zero_stamp(builtin_interfaces__msg__Time &t) {
  t.sec = 0; t.nanosec = 0;
}

// --------------------------------- Callbacks -------------------------------- //
// /cmd_vel subscriber: cache the most recent velocity command and time
static void cb_cmd_vel(const void* msg_in) {
  const auto* tw = static_cast<const geometry_msgs__msg__Twist*>(msg_in);
  v_goal = (float)tw->linear.x;   // TB3 uses linear.x
  w_goal = (float)tw->angular.z;  // and angular.z
  last_cmd_ms = millis();
}

// High-rate control/safety loop (125 Hz):
// - Enforce stale timeout, sonar stop, battery cutoff
// - Push goals into MonsterController
// - Integrate odometry from applied commands
static void cb_control_timer(rcl_timer_t*, int64_t) {
  const uint32_t now = millis();
  const bool stale = (now - last_cmd_ms) > (uint32_t)SAFETY_CMD_TIMEOUT_MS;

  float v = stale ? 0.0f : v_goal;
  float w = stale ? 0.0f : w_goal;

  // Battery cutoff
  const float vbat = sensors.batteryVoltage();
  if (vbat > 0.0f && vbat < SAFETY_VBAT_MIN_V) {
    v = 0.0f; w = 0.0f;
    diagnosis.flashError(2);
  }

  // Sonar stop (block forward/back; allow rotation to reorient)
  const float d = sensors.sonarMeters();
  if (d > 0.0f && d < SAFETY_SONAR_MIN_M) {
    v = 0.0f;
  }

  // Apply command to controller/motors (controller handles clamping to limits)
  controller.setGoal(v, w);
  controller.update();

  // Odometry: integrate from commanded (v,w) over dt
  const float dt = 1.0f / (float)CONTROL_HZ;
  x_odom  += v * cosf(th_odom) * dt;
  y_odom  += v * sinf(th_odom) * dt;
  th_odom += w * dt;

  // Joint state: derive wheel angular velocity (rad/s) and integrate position (rad)
  const float vL = v - w * (WHEEL_SEPARATION * 0.5f);
  const float vR = v + w * (WHEEL_SEPARATION * 0.5f);
  vel_left  = (double)(vL / WHEEL_RADIUS);
  vel_right = (double)(vR / WHEEL_RADIUS);
  pos_left  += vel_left  * (double)dt;
  pos_right += vel_right * (double)dt;

  // Keep hardware/LED state machines alive
  sensors.update();
  diagnosis.tick();
}

// /odom publisher (50 Hz)
static void cb_odom_timer(rcl_timer_t*, int64_t) {
  zero_stamp(msg_odom.header.stamp);
  rosidl_runtime_c__String__assign(&msg_odom.header.frame_id, FRAME_ODOM);
  rosidl_runtime_c__String__assign(&msg_odom.child_frame_id, FRAME_BASE);


  msg_odom.pose.pose.position.x = x_odom;
  msg_odom.pose.pose.position.y = y_odom;
  msg_odom.pose.pose.position.z = 0.0;

  double qx,qy,qz,qw;
  yaw_to_quat(th_odom, qx,qy,qz,qw);
  msg_odom.pose.pose.orientation.x = qx;
  msg_odom.pose.pose.orientation.y = qy;
  msg_odom.pose.pose.orientation.z = qz;
  msg_odom.pose.pose.orientation.w = qw;

  msg_odom.twist.twist.linear.x  = (double)v_goal;
  msg_odom.twist.twist.linear.y  = 0.0;
  msg_odom.twist.twist.linear.z  = 0.0;
  msg_odom.twist.twist.angular.x = 0.0;
  msg_odom.twist.twist.angular.y = 0.0;
  msg_odom.twist.twist.angular.z = (double)w_goal;

  rcl_publish(&pub_odom, &msg_odom, NULL);
}

// /imu publisher (50 Hz)
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

  // Optional: mark orientation covariance unknown for robot_localization
  msg_imu.orientation_covariance[0] = -1.0;

  rcl_publish(&pub_imu, &msg_imu, NULL);
}

// /magnetic_field publisher (50 Hz)
// NOTE: MonsterSensor::mag() units should be Tesla; if Gauss, multiply by 1e-4
static void cb_mag_timer(rcl_timer_t*, int64_t) {
  sensors.update();
  const float* m = sensors.mag();

  zero_stamp(msg_mag.header.stamp);
  rosidl_runtime_c__String__assign(&msg_imu.header.frame_id, FRAME_IMU);

  msg_mag.magnetic_field.x = m[0];
  msg_mag.magnetic_field.y = m[1];
  msg_mag.magnetic_field.z = m[2];

  rcl_publish(&pub_mag, &msg_mag, NULL);
}

// /joint_states publisher (50 Hz)
// Uses rosidl sequences for names/positions/velocities.
static void cb_joints_timer(rcl_timer_t*, int64_t) {
  zero_stamp(msg_joint.header.stamp);

  // Positions/velocities are already sequences; update values in-place:
  if (msg_joint.position.size >= 2) {
    msg_joint.position.data[0] = pos_left;
    msg_joint.position.data[1] = pos_right;
  }
  if (msg_joint.velocity.size >= 2) {
    msg_joint.velocity.data[0] = vel_left;
    msg_joint.velocity.data[1] = vel_right;
  }
  // effort left unpopulated (size==0)

  rcl_publish(&pub_joint, &msg_joint, NULL);
}

// /battery_state publisher (1 Hz) – voltage only
static void cb_batt_timer(rcl_timer_t*, int64_t) {
  zero_stamp(msg_batt.header.stamp);
  msg_batt.voltage = sensors.batteryVoltage();  // OpenCR helper
  msg_batt.current = NAN;
  msg_batt.charge = NAN;
  msg_batt.capacity = NAN;
  msg_batt.design_capacity = NAN;
  msg_batt.percentage = NAN;
  msg_batt.power_supply_status = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_UNKNOWN;
  msg_batt.power_supply_health = sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_UNKNOWN;
  msg_batt.power_supply_technology = sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  msg_batt.present = true;

  rcl_publish(&pub_batt, &msg_batt, NULL);
}

// ------------------------------------ Setup --------------------------------- //
void setup() {
  // --- Serial for debug ---
  Serial.begin(115200);
  delay(50);

  // --- Hardware bring-up (motors/sensors/controller/LEDs) ---
  motors.begin();
  motors.configureAll();
  controller.attachMotor(&motors);
  sensors.begin();
  sensors.calibrateGyro();
  diagnosis.init();
  diagnosis.setMode(MonsterDiagnosis::Mode::HEARTBEAT);

  // --- micro-ROS transport (force the USB CDC serial on OpenCR) ---
  // If your core maps the USB CDC to SerialUSB, switch to set_microros_serial_transports(SerialUSB);
  set_microros_transports();

  // --- micro-ROS core init with custom Domain ID via rcl_init_options ---
  g_alloc = rcl_get_default_allocator();

  rcl_init_options_t init_opts = rcl_get_zero_initialized_init_options();
  if (rcl_init_options_init(&init_opts, g_alloc) != RCL_RET_OK) {
    Serial.println("ERROR: rcl_init_options_init"); return;
  }
  if (rcl_init_options_set_domain_id(&init_opts, ROS_DOMAIN_ID_SET) != RCL_RET_OK) {
    Serial.println("ERROR: rcl_init_options_set_domain_id"); return;
  }
  if (rclc_support_init_with_options(&g_support, 0, NULL, &init_opts, &g_alloc) != RCL_RET_OK) {
    Serial.println("ERROR: rclc_support_init_with_options"); return;
  }
  rcl_init_options_fini(&init_opts);

  // --- Node ---
  if (rclc_node_init_default(&g_node, "turtlebot3_compat_node", "", &g_support) != RCL_RET_OK) {
    Serial.println("ERROR: rclc_node_init_default"); return;
  }

  // --- Subscriber: /cmd_vel ---
  if (rclc_subscription_init_default(
        &sub_cmd_vel, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel") != RCL_RET_OK) {
    Serial.println("ERROR: sub /cmd_vel"); return;
  }

  // --- Publishers: /odom, /imu, /magnetic_field, /joint_states, /battery_state ---
  if (rclc_publisher_init_default(&pub_odom, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom") != RCL_RET_OK) {
    Serial.println("ERROR: pub /odom"); return;
  }

  if (rclc_publisher_init_default(&pub_imu, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu") != RCL_RET_OK) {
    Serial.println("ERROR: pub /imu"); return;
  }

  if (rclc_publisher_init_default(&pub_mag, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField), "magnetic_field") != RCL_RET_OK) {
    Serial.println("ERROR: pub /magnetic_field"); return;
  }

  if (rclc_publisher_init_default(&pub_joint, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "joint_states") != RCL_RET_OK) {
    Serial.println("ERROR: pub /joint_states"); return;
  }

  if (rclc_publisher_init_default(&pub_batt, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), "battery_state") != RCL_RET_OK) {
    Serial.println("ERROR: pub /battery_state"); return;
  }

  // --- Initialize JointState name & array sizes (2 wheels) ---
  // name: rosidl_runtime_c__String__Sequence
  if (!rosidl_runtime_c__String__Sequence__init(&msg_joint.name, 2)) {
    Serial.println("ERROR: JointState name seq init"); return;
  }
  rosidl_runtime_c__String__assign(&msg_joint.name.data[0], JOINT_LEFT);
  rosidl_runtime_c__String__assign(&msg_joint.name.data[1], JOINT_RIGHT);

  // position/velocity sequences (2 each); effort left empty (0)
  if (!rosidl_runtime_c__double__Sequence__init(&msg_joint.position, 2)) {
    Serial.println("ERROR: JointState position seq init"); return;
  }
  if (!rosidl_runtime_c__double__Sequence__init(&msg_joint.velocity, 2)) {
    Serial.println("ERROR: JointState velocity seq init"); return;
  }
  // effort stays size=0 by default

  // --- Executor (1 sub + 5 timers = 6 handles) ---
  if (rclc_executor_init(&g_exec, &g_support.context, 6, &g_alloc) != RCL_RET_OK) {
    Serial.println("ERROR: rclc_executor_init"); return;
  }
  if (rclc_executor_add_subscription(&g_exec, &sub_cmd_vel, &msg_cmd_vel, &cb_cmd_vel, ON_NEW_DATA) != RCL_RET_OK) {
    Serial.println("ERROR: add sub /cmd_vel"); return;
  }

  // --- Timers (Hz -> ns) ---
  const uint64_t ns_control = (uint64_t)(1000000000ULL / CONTROL_HZ);
  const uint64_t ns_odom    = (uint64_t)(1000000000ULL / PUB_ODOM_HZ);
  const uint64_t ns_imu     = (uint64_t)(1000000000ULL / PUB_IMU_HZ);
  const uint64_t ns_mag     = (uint64_t)(1000000000ULL / PUB_MAG_HZ);
  const uint64_t ns_joints  = (uint64_t)(1000000000ULL / PUB_JOINTS_HZ);
  const uint64_t ns_batt    = (uint64_t)(1000000000ULL / PUB_BATTERY_HZ);

  rclc_timer_init_default(&tmr_control, &g_support, ns_control, &cb_control_timer);
  rclc_timer_init_default(&tmr_odom,    &g_support, ns_odom,    &cb_odom_timer);
  rclc_timer_init_default(&tmr_imu,     &g_support, ns_imu,     &cb_imu_timer);
  rclc_timer_init_default(&tmr_mag,     &g_support, ns_mag,     &cb_mag_timer);
  rclc_timer_init_default(&tmr_joints,  &g_support, ns_joints,  &cb_joints_timer);
  rclc_timer_init_default(&tmr_batt,    &g_support, ns_batt,    &cb_batt_timer);

  rclc_executor_add_timer(&g_exec, &tmr_control);
  rclc_executor_add_timer(&g_exec, &tmr_odom);
  rclc_executor_add_timer(&g_exec, &tmr_imu);
  rclc_executor_add_timer(&g_exec, &tmr_mag);
  rclc_executor_add_timer(&g_exec, &tmr_joints);
  rclc_executor_add_timer(&g_exec, &tmr_batt);

  Serial.println("TB3-compatible firmware running.");
  Serial.println("Topics:");
  Serial.println("  sub  : /cmd_vel");
  Serial.println("  pubs : /odom, /imu, /magnetic_field, /joint_states, /battery_state");
}

// ------------------------------------- Loop --------------------------------- //
void loop() {
  // Spin executor to service timers/subscriptions
  rclc_executor_spin_some(&g_exec, RCL_MS_TO_NS(2));
}
