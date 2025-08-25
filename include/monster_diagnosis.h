#ifndef MONSTER_DIAGNOSIS_H
#define MONSTER_DIAGNOSIS_H

#include <Arduino.h>

class MonsterDiagnosis {
public:
  MonsterDiagnosis() {}
  bool init();
  void showLedStatus(bool connected);
};

#endif
