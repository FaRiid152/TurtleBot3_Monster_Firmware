#ifndef MONSTER_H_
#define MONSTER_H_

#include "../include/monster_config.h"

#include "../include/monster_motor.h"
#include "../include/monster_sensor.h"
#include "../include/monster_controller.h"
#include "../include/monster_diagnosis.h"

namespace MonsterCore {

  // Shared singletons
  extern monster_motor   motors;
  extern monster_sensor    sensors;
  extern monster_controller controller;
  extern monster_diagnosis diagnosis;

  // ---------------- Plain (no ROS) ----------------
  // Bring-up and periodic loop
  void beginPlain(const char* model_name = "Monster");
  void runPlain();

  // ---------------- ROS 2 (micro-ROS) -------------
  // Bring-up with micro-ROS + cmd_vel subscriber
  void beginROS2(const char* model_name = "Monster");
  // Call in Arduino loop(); handles ROS executor + control tick
  void spinROS2();
}

#endif // MONSTER_H_
