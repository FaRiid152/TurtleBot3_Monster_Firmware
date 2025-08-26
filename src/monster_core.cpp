#include "Monster.h"

// Global instances
//FourWheelMotor motors;
//MonsterSensor sensors;
//MonsterController controller;
//MonsterDiagnosis diagnosis;

namespace MonsterCore {
FourWheelMotor    motors;
MonsterSensor     sensors;
MonsterController controller;
MonsterDiagnosis  diagnosis;

void begin(const char* model_name) {
  (void)model_name; // not used yet, but can log

  motors.begin();
  motors.configureAll();

  sensors.begin();
  
  controller.init(MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY);
  controller.attachMotor(&motors);
  
  diagnosis.init();
  diagnosis.setMode(MonsterDiagnosis::Mode::HEARTBEAT);

}

void run() {
  // Update sensors
  sensors.update();

  controller.update();

  // Example: LED heartbeat
  diagnosis.tick();
}

} // namespace MonsterCore
