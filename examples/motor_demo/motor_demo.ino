#include <Monster.h>

void setup() {
  Serial.begin(115200);
  MonsterCore::motors.begin();
  MonsterCore::motors.configureAll();
}

void loop() {
  // Spin left forward, right backward (turn in place)
  MonsterCore::motors.commandSides(0.1, -0.1);
  delay(2000);

  // Stop
  MonsterCore::motors.commandSides(0, 0);
  delay(1000);

  // Forward
  MonsterCore::motors.commandSides(0.1, 0.1);
  delay(2000);

  // Stop again
  MonsterCore::motors.commandSides(0, 0);
  delay(1000);
}
