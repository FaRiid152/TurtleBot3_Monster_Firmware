#ifndef MONSTER_CONTROLLER_H
#define MONSTER_CONTROLLER_H

#include <Arduino.h>
#include <stdint.h>
#include <math.h>
#include "monster_config.h"


// ===== Constants =====

#define WHEEL_POS_FROM_CENTER_X_1       -0.100
#define WHEEL_POS_FROM_CENTER_Y_1       -0.128
#define WHEEL_POS_FROM_CENTER_X_2        0.100
#define WHEEL_POS_FROM_CENTER_Y_2       -0.128
#define WHEEL_POS_FROM_CENTER_X_3       -0.100
#define WHEEL_POS_FROM_CENTER_Y_3        0.128
#define WHEEL_POS_FROM_CENTER_X_4        0.100
#define WHEEL_POS_FROM_CENTER_Y_4        0.128

#define ENCODER_MIN                     -2147483648
#define ENCODER_MAX                      2147483648

//#define VELOCITY_CONSTANT_VAULE         1263.632956882  // not used in controller anymore


//#define MAX_LINEAR_VELOCITY             0.22
//#define MAX_ANGULAR_VELOCITY            2.84
#define VELOCITY_LINEAR_X               0.01
#define VELOCITY_ANGULAR_Z              0.1
#define SCALE_VELOCITY_LINEAR_X         1
#define SCALE_VELOCITY_ANGULAR_Z        1

//#define DEG2RAD(x)                      ((x) * 0.01745329252)
//#define RAD2DEG(x)                      ((x) * 57.2957795131)

// Forward-declare to keep header light. We'll include the real header in the .cpp.
class monster_motor;

class monster_controller {
public:
  monster_controller() {}

  bool init(float max_lin_vel, float max_ang_vel);
  void setGoal(float linear_mps, float angular_rps);
  bool update();                          // now calls monster_motor::commandSides()
  void getRCdata(float* cmd_vel);

  void attachMotor(monster_motor* drv) { motor_ = drv; }

private:
  float max_lin_ = MAX_LINEAR_VELOCITY;
  float max_ang_ = MAX_ANGULAR_VELOCITY;
  float v_goal_  = 0.0f;                  // m/s
  float w_goal_  = 0.0f;                  // rad/s

  monster_motor* motor_{nullptr};

  static float clamp(float x, float lo, float hi) {
    return (x < lo) ? lo : ((x > hi) ? hi : x);
  }
};

#endif // MONSTER_CONTROLLER_H
