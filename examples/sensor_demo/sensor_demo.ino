#include <Monster.h>

void setup() {
  Serial.begin(115200);
  while (!Serial) ;  // wait for USB
  if (!MonsterCore::sensors.begin()) {
    Serial.println("Sensor init failed!");
  }
}

void loop() {
  MonsterCore::sensors.update();

  const float* q = MonsterCore::sensors.quat();
  const float* g = MonsterCore::sensors.gyro();
  const float* a = MonsterCore::sensors.accel();

  Serial.print("Battery: "); Serial.println(MonsterCore::sensors.batteryVoltage());
  Serial.print("Sonar: ");   Serial.println(MonsterCore::sensors.sonarMeters());

  Serial.print("Quat: [");
  for (int i=0;i<4;i++) { Serial.print(q[i],3); if (i<3) Serial.print(","); }
  Serial.println("]");

  delay(200);
}
