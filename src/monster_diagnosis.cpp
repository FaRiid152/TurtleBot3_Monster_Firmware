#include "../include/monster_diagnosis.h"

bool MonsterDiagnosis::init() {
  pinMode(LED_BUILTIN, OUTPUT);
  return true;
}

void MonsterDiagnosis::showLedStatus(bool connected) {
  digitalWrite(LED_BUILTIN, connected ? HIGH : LOW);
}
