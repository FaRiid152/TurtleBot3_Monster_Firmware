#include "../include/monster_sensor.h"
#include <IMU.h>

#ifndef LED_WORKING_CHECK
#define LED_WORKING_CHECK 13
#endif

MonsterSensor::MonsterSensor() {}

bool MonsterSensor::begin() {
  pinMode(LED_WORKING_CHECK, OUTPUT);
  digitalWrite(LED_WORKING_CHECK, LOW);

  initBumper();
  initIR();
  initSonar();
  initLEDs();

  // IMU init
  uint8_t err = imu_.begin();
  delay(50);
  if (err != 0x00) {
    imu_ok_ = false;
    return false;
  }

  // Gyro calibration with a quick blink
  imu_.SEN.gyro_cali_start();
  uint32_t t0 = millis(), blink_t = t0;
  while (!imu_.SEN.gyro_cali_get_done()) {
    imu_.update();
    if (millis() - t0 > 5000) break;  // safety timeout
    if (millis() - blink_t > 100) {
      blink_t = millis();
      digitalWrite(LED_WORKING_CHECK, !digitalRead(LED_WORKING_CHECK));
    }
  }
  digitalWrite(LED_WORKING_CHECK, LOW);

  imu_ok_ = true;
  return true;
}

void MonsterSensor::update() {
  updateIMU();
  updateSonar(millis());
}

/* ---------------- IMU ---------------- */

void MonsterSensor::updateIMU() {
  if (!imu_ok_) return;
  imu_.update();

  // Quaternion w,x,y,z
  quat_[0] = imu_.quat[0];
  quat_[1] = imu_.quat[1];
  quat_[2] = imu_.quat[2];
  quat_[3] = imu_.quat[3];

  // Scaled sensors (match TB3 factors)
  gyro_[0]  = imu_.SEN.gyroADC[0] * GYRO_FACTOR;
  gyro_[1]  = imu_.SEN.gyroADC[1] * GYRO_FACTOR;
  gyro_[2]  = imu_.SEN.gyroADC[2] * GYRO_FACTOR;

  accel_[0] = imu_.SEN.accADC[0] * ACCEL_FACTOR;
  accel_[1] = imu_.SEN.accADC[1] * ACCEL_FACTOR;
  accel_[2] = imu_.SEN.accADC[2] * ACCEL_FACTOR;

  mag_[0]   = imu_.SEN.magADC[0] * MAG_FACTOR;
  mag_[1]   = imu_.SEN.magADC[1] * MAG_FACTOR;
  mag_[2]   = imu_.SEN.magADC[2] * MAG_FACTOR;
}

void MonsterSensor::calibrateGyro() {
  if (!imu_ok_) return;
  imu_.SEN.gyro_cali_start();
  const uint32_t t0 = millis();
  while (!imu_.SEN.gyro_cali_get_done()) {
    imu_.update();
    if (millis() - t0 > 5000) break;
  }
}

/* --------- OLLO (touch/IR) & Sonar --------- */

void MonsterSensor::initBumper() {
  // OLLO ports 3 & 4 for touch (TB3 convention). Safe if not present.
  ollo_.begin(3, TOUCH_SENSOR);
  ollo_.begin(4, TOUCH_SENSOR);
}

void MonsterSensor::initIR() {
  // OLLO port 2 for IR (TB3 convention). Safe if not present.
  ollo_.begin(2, IR_SENSOR);
}

void MonsterSensor::initSonar() {
  sonar_.trig = MONSTER_SONAR_TRIG_PIN;
  sonar_.echo = MONSTER_SONAR_ECHO_PIN;
  pinMode(sonar_.trig, OUTPUT);
  pinMode(sonar_.echo, INPUT);
  sonar_store_ = 0.0f;
}

void MonsterSensor::updateSonar(uint32_t /*now_ms*/) {
  // Standard HC-SR04 ping
  digitalWrite(sonar_.trig, LOW);
  delayMicroseconds(2);
  digitalWrite(sonar_.trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(sonar_.trig, LOW);

  // Timeout ~30ms to avoid blocking too long
  const unsigned long dur_us = pulseIn(sonar_.echo, HIGH, 30000UL);

  if (dur_us > 0) {
    // distance = (speed_of_sound * time) / 2
    // speed_of_sound ≈ 340 m/s = 0.00034 m/us
    sonar_store_ = (0.00034f * (float)dur_us) * 0.5f;
  }
  // else: keep last value (acts as a simple hold on timeout)
}

float MonsterSensor::sonarMeters() const {
  return sonar_store_;
}

/* ------------- Illumination ------------- */

float MonsterSensor::lightLevel() const {
  // A1 on OpenCR is a free analog pin; if you wire a photoresistor you can use this.
  return (float)analogRead(A1);
}

/* ---------------- LEDs ---------------- */

void MonsterSensor::initLEDs() {
  led_.front_left  = MONSTER_LED_FL;
  led_.front_right = MONSTER_LED_FR;
  led_.back_left   = MONSTER_LED_BL;
  led_.back_right  = MONSTER_LED_BR;

  pinMode(led_.front_left,  OUTPUT);
  pinMode(led_.front_right, OUTPUT);
  pinMode(led_.back_left,   OUTPUT);
  pinMode(led_.back_right,  OUTPUT);

  digitalWrite(led_.front_left,  LOW);
  digitalWrite(led_.front_right, LOW);
  digitalWrite(led_.back_left,   LOW);
  digitalWrite(led_.back_right,  LOW);
}

void MonsterSensor::setLedPattern(double lin, double ang) {
  auto FL=[&](int s){ digitalWrite(led_.front_left,  s); };
  auto FR=[&](int s){ digitalWrite(led_.front_right, s); };
  auto BL=[&](int s){ digitalWrite(led_.back_left,   s); };
  auto BR=[&](int s){ digitalWrite(led_.back_right,  s); };

  if (lin > 0.0 && ang == 0.0) {          // forward
    FL(HIGH); FR(HIGH); BL(LOW);  BR(LOW);
  } else if (lin >= 0.0 && ang > 0.0) {   // forward-left
    FL(HIGH); FR(LOW);  BL(LOW);  BR(LOW);
  } else if (lin >= 0.0 && ang < 0.0) {   // forward-right
    FL(LOW);  FR(HIGH); BL(LOW);  BR(LOW);
  } else if (lin < 0.0 && ang == 0.0) {   // back
    FL(LOW);  FR(LOW);  BL(HIGH); BR(HIGH);
  } else if (lin <= 0.0 && ang > 0.0) {   // back-right
    FL(LOW);  FR(LOW);  BL(LOW);  BR(HIGH);
  } else if (lin <= 0.0 && ang < 0.0) {   // back-left
    FL(LOW);  FR(LOW);  BL(HIGH); BR(LOW);
  } else {
    FL(LOW);  FR(LOW);  BL(LOW);  BR(LOW);
  }
}
