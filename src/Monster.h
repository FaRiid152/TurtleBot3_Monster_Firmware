#ifndef MONSTER_H_
#define MONSTER_H_

//#include "../include/monster_config.h"
#include "../include/monster_motor.h"
#include "../include/monster_sensor.h"
//#include "../include/monster_controller.h"
//#include "../include/monster_diagnosis.h"

namespace MonsterCore {
  extern FourWheelMotor    motors;
  extern MonsterSensor     sensors;
  //extern MonsterController controller;
  //extern MonsterDiagnosis  diagnosis;
  
  void begin(const char* model_name = "Monster");
  void run();
}

#endif // MONSTER_H_
