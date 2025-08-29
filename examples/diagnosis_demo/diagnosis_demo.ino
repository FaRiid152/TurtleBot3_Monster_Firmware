#include <Monster.h>

void setup() {
  MonsterCore::diagnosis.init();
  MonsterCore::diagnosis.setMode(MonsterDiagnosis::Mode::HEARTBEAT);
}

void loop() {
  MonsterCore::diagnosis.tick();

  // Press user button → trigger error blink code 3
  if (MonsterCore::sensors.userButton()) {
    MonsterCore::diagnosis.flashError(3);
  }
}
