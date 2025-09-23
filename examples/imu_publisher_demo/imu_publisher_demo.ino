#include <Arduino.h>
#include <Monster.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/imu.h>

using namespace MonsterCore;

#define PUBLISH_PERIOD_MS 50
#define ROS_DOMAIN_ID_SET 35   // must match your host

rcl_allocator_t  allocator;
rclc_support_t   support;
rcl_node_t       node;
rcl_publisher_t  imu_pub;
sensor_msgs__msg__Imu imu_msg;

unsigned long last_pub = 0;

void setup() {
  Serial.begin(115200);
  delay(50);

  // 1) Transport FIRST (force serial CDC)
  set_microros_transports();   // try SerialUSB if your core needs it

  // 2) Sensors
  sensors.begin();
  sensors.calibrateGyro();

  // 3) Init options with custom domain id
  allocator = rcl_get_default_allocator();

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_ret_t rc = rcl_init_options_init(&init_options, allocator);
  if (rc != RCL_RET_OK) { Serial.println("rcl_init_options_init failed"); return; }

  rc = rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID_SET);
  if (rc != RCL_RET_OK) { Serial.println("set_domain_id failed"); return; }

  // 4) Create support with options
  rc = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  if (rc != RCL_RET_OK) { Serial.println("support_init_with_options failed"); return; }

  // init_options no longer needed
  rcl_init_options_fini(&init_options);

  // 5) Node
  rc = rclc_node_init_default(&node, "monster_imu_node", "", &support);
  if (rc != RCL_RET_OK) { Serial.println("node_init failed"); return; }

  // 6) Publisher
  rc = rclc_publisher_init_default(
        &imu_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data_raw");
  if (rc != RCL_RET_OK) { Serial.println("publisher_init failed"); return; }

  // 7) Optional covariance defaults
  for (int i = 0; i < 9; ++i) {
    imu_msg.orientation_covariance[i]         = 0.0;
    imu_msg.angular_velocity_covariance[i]    = 0.0;
    imu_msg.linear_acceleration_covariance[i] = 0.0;
  }
  imu_msg.orientation_covariance[0] = -1.0;

  Serial.print("IMU publisher ready on /imu/data_raw, domain ");
  Serial.println(ROS_DOMAIN_ID_SET);
}

void loop() {
  sensors.update();

  const unsigned long now = millis();
  if (now - last_pub >= PUBLISH_PERIOD_MS) {
    last_pub = now;

    const float* q = sensors.quat();
    const float* g = sensors.gyro();
    const float* a = sensors.accel();

    imu_msg.orientation.x = q[0];
    imu_msg.orientation.y = q[1];
    imu_msg.orientation.z = q[2];
    imu_msg.orientation.w = q[3];

    imu_msg.angular_velocity.x = g[0];
    imu_msg.angular_velocity.y = g[1];
    imu_msg.angular_velocity.z = g[2];

    imu_msg.linear_acceleration.x = a[0];
    imu_msg.linear_acceleration.y = a[1];
    imu_msg.linear_acceleration.z = a[2];

    rcl_ret_t rc = rcl_publish(&imu_pub, &imu_msg, NULL);
    if (rc != RCL_RET_OK) {
      Serial.println("rcl_publish failed");
    }
  }
}
