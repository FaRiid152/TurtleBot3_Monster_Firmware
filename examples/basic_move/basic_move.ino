#include <Monster.h>
using namespace MonsterCore;

void setup() {
  begin("Monster");   // Initialize motors only
}

void loop() {
  // Forward 0.10 m/s for 2s
  motors.commandSides(0.10f, 0.10f);
  delay(2000);

  // Stop 2s
  motors.commandSides(0.0f, 0.0f);
  delay(2000);

  // Spin left 1s
  motors.commandSides(-0.10f, 0.10f);
  delay(1000);

  // Stop 1s
  motors.commandSides(0.0f, 0.0f);
  delay(1000);
}
