#ifndef MONSTER_CONFIG_H
#define MONSTER_CONFIG_H

#include <stdint.h>  // for int32_t

#define MONSTER_EXT_SENSORS 0   // 0 = no OLLO/sonar hardware present

// Robot geometry
#define WHEEL_RADIUS        0.033f
#define WHEEL_SEPARATION    0.160f

// Velocity limits
#define MAX_LINEAR_VELOCITY   (WHEEL_RADIUS * 2.0f * 3.14159265359f * 61.0f / 60.0f)
#define MIN_LINEAR_VELOCITY   (-MAX_LINEAR_VELOCITY)
#define MAX_ANGULAR_VELOCITY  (MAX_LINEAR_VELOCITY / (WHEEL_SEPARATION * 0.5f))
#define MIN_ANGULAR_VELOCITY  (-MAX_ANGULAR_VELOCITY)

// Dynamixel bus (OpenCR TTL)
#define DXL_SERIAL     Serial3
#define DXL_DIR_PIN    84
#define DXL_BAUD       57600
#define DXL_PROTOCOL   2.0f

// Motor IDs
#define ID_L1  1
#define ID_R1  2
#define ID_L2  3
#define ID_R2  4

// Signs for wheel direction
#define SIGN_L1  (+1)
#define SIGN_L2  (+1)
#define SIGN_R1  (+1)
#define SIGN_R2  (-1)

// Global timing
#define CONTROL_PERIOD                  8000

// DEG2RAD/RAD2DEG conversion:
static inline float deg2rad(float x){ return x * 0.01745329252f; }
static inline float rad2deg(float x){ return x * 57.2957795131f; }


// Helpers in a namespace to avoid clashes
namespace monster_config {

// m/s -> Dynamixel Goal Velocity units
static inline int32_t mpsToGoalU(float mps) {
  // 1 unit = 0.229 rpm;  v = rpm * (2πR)/60  => units = v*60 / (0.229*2πR)
  const float denom = 0.229f * 2.0f * 3.14159265359f * WHEEL_RADIUS;
  const float u = (mps * 60.0f) / denom;
  if (u >  1023.0f) return  1023;
  if (u < -1023.0f) return -1023;
  return (int32_t)u;
}

// Optional reverse (feedback units -> m/s)
static inline float goalUToMps(int32_t u) {
  const float rpm = u * 0.229f;
  return rpm * (2.0f * 3.14159265359f * WHEEL_RADIUS) / 60.0f;
}

} // namespace monster_config

#endif
