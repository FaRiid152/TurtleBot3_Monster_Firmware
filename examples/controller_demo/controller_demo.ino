#include <Monster.h>

void setup() {
  Serial.begin(115200);
  MonsterCore::motors.begin();
  MonsterCore::motors.configureAll();
  MonsterCore::controller.attachMotor(&MonsterCore::motors);
}

void loop() {
  // Drive forward at 0.15 m/s for 2 sec
  MonsterCore::controller.setGoal(0.15, 0.0);
  MonsterCore::controller.update();
  delay(2000);

  // Spin in place
  MonsterCore::controller.setGoal(0.0, 1.0);
  MonsterCore::controller.update();
  delay(2000);

  // Stop
  MonsterCore::controller.setGoal(0.0, 0.0);
  MonsterCore::controller.update();
  delay(2000);
}
