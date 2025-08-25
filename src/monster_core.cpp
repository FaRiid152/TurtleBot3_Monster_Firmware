#include "Monster.h"

// Global instances
FourWheelMotor motors;
MonsterSensor sensors;
//MonsterController controller;
//MonsterDiagnosis diagnosis;

namespace MonsterCore {
FourWheelMotor    motors;
MonsterSensor     sensors;
//MonsterController controller;
//MonsterDiagnosis  diagnosis;

void begin(const char* model_name) {
  (void)model_name; // not used yet, but can log

  motors.begin();
  motors.configureAll();

  //sensors.init();
  //controller.init(MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY);
  //diagnosis.init();
}

void run() {
  // Update sensors
  //sensors.updateIMU();

  // Example: LED heartbeat
  //diagnosis.showLedStatus(true);
}

} // namespace MonsterCore
