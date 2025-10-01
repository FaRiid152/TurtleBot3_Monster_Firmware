#include "../include/monster_sensor.h"
#include "Monster.h"
#include <IMU.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef LED_WORKING_CHECK
#define LED_WORKING_CHECK 13
#endif

// ---- ODOMETRY CONSTANTS ----
static constexpr float TICKS_PER_REV = 4096.0f;                         // XL430 resolution
static constexpr float TICK_TO_RAD   = (2.0f * M_PI) / TICKS_PER_REV;   // rad per tick
static constexpr float GOALU_TO_RAD_PER_S = (0.229f * 2.0f * M_PI) / 60.0f; // 0.229 rpm/unit

monster_sensor::monster_sensor() {}

bool monster_sensor::begin() {
  pinMode(LED_WORKING_CHECK, OUTPUT);
  digitalWrite(LED_WORKING_CHECK, LOW);
#if MONSTER_EXT_SENSORS
  initBumper();
  initIR();
  initSonar();
#endif
  initLEDs();

  // IMU init
  uint8_t err = imu_.begin();
  delay(50);
  if (err != 0x00) { imu_ok_ = false; return false; }

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

  resetOdometry(0,0,0);
  return true;
}

void monster_sensor::update() {
  updateIMU();
#if MONSTER_EXT_SENSORS
  updateSonar(millis());
#endif
  updateOdometry();
}

/* ---------------- Odom ---------------- */
void monster_sensor::resetOdometry(float x, float y, float theta) {
  odom_x_ = x; odom_y_ = y; odom_th_ = theta;
  enc_have_prev_ = false;
  for (int i=0;i<4;i++) { wheel_pos_rad_[i]=0.0f; wheel_vel_rad_[i]=0.0f; }
  last_odom_us_ = micros();
}

// ---- CORE ODOMETRY ----
void monster_sensor::updateOdometry() {
  int32_t p[4];
  if (!MonsterCore::motors.readPositions(p)) return;   // requires RAW ticks (no signs)

  // Try bulk velocities (raw goal units). OK if not supported.
  int32_t vU[4];
  const bool have_vel = MonsterCore::motors.readVelocities(vU);

  // Unwrap helper for 12-bit circle (0..4095)
  auto unwrap = [](int32_t dticks) -> int32_t {
    if (dticks >  2048) dticks -= 4096;
    if (dticks < -2048) dticks += 4096;
    return dticks;
  };

  const uint32_t now_us = micros();
  const float dt = (now_us - last_odom_us_) * 1e-6f;
  last_odom_us_ = now_us;
  if (dt <= 0.0f || dt > 0.5f) enc_have_prev_ = false; // ignore bad dt/leaps

  int32_t dL1=0, dL2=0, dR1=0, dR2=0;
  if (enc_have_prev_) {
    // Apply SIGN_* only to deltas (assumes raw ticks above)
    dL1 = SIGN_L1 * unwrap(p[0] - enc_prev_[0]);
    dL2 = SIGN_L2 * unwrap(p[1] - enc_prev_[1]);
    dR1 = SIGN_R1 * unwrap(p[2] - enc_prev_[2]);
    dR2 = SIGN_R2 * unwrap(p[3] - enc_prev_[3]);
  }
  for (int i=0;i<4;i++) enc_prev_[i] = p[i];
  if (!enc_have_prev_) { enc_have_prev_ = true; return; }

  // Wheel angle increments (rad)
  const float dphi_L1 = dL1 * TICK_TO_RAD;
  const float dphi_L2 = dL2 * TICK_TO_RAD;
  const float dphi_R1 = dR1 * TICK_TO_RAD;
  const float dphi_R2 = dR2 * TICK_TO_RAD;

  // Integrate continuous wheel angles
  wheel_pos_rad_[0] += dphi_L1;
  wheel_pos_rad_[1] += dphi_L2;
  wheel_pos_rad_[2] += dphi_R1;
  wheel_pos_rad_[3] += dphi_R2;

  // Velocities: prefer Present_Velocity if available, else finite-difference
  if (have_vel) {
    wheel_vel_rad_[0] = (SIGN_L1 * vU[0]) * GOALU_TO_RAD_PER_S;
    wheel_vel_rad_[1] = (SIGN_L2 * vU[1]) * GOALU_TO_RAD_PER_S;
    wheel_vel_rad_[2] = (SIGN_R1 * vU[2]) * GOALU_TO_RAD_PER_S;
    wheel_vel_rad_[3] = (SIGN_R2 * vU[3]) * GOALU_TO_RAD_PER_S;
  } else if (dt > 0.0f && dt < 0.2f) {
    wheel_vel_rad_[0] = dphi_L1 / dt;
    wheel_vel_rad_[1] = dphi_L2 / dt;
    wheel_vel_rad_[2] = dphi_R1 / dt;
    wheel_vel_rad_[3] = dphi_R2 / dt;
  }
  /*
  static uint32_t last_dbg=0;
  if (millis() - last_dbg >= 200) {
    last_dbg = millis();
    Serial.print(F("# p:"));
    Serial.print(p[0]);Serial.print(',');Serial.print(p[1]);Serial.print(',');
    Serial.print(p[2]);Serial.print(',');Serial.print(p[3]);
    Serial.print(F(" dt:")); Serial.print(dt,3);
    Serial.print(F(" d:"));
    Serial.print(dL1);Serial.print(',');Serial.print(dL2);Serial.print(',');
    Serial.print(dR1);Serial.print(',');Serial.println(dR2);
  }
    */

  // Differential-drive body increment using side averages
  const float dphi_L = 0.5f * (dphi_L1 + dphi_L2);
  const float dphi_R = 0.5f * (dphi_R1 + dphi_R2);

  const float sL = WHEEL_RADIUS * dphi_L;
  const float sR = WHEEL_RADIUS * dphi_R;

  const float ds  = 0.5f * (sL + sR);
  const float dth = (sR - sL) / WHEEL_SEPARATION;

  const float th_mid = odom_th_ + 0.5f * dth;
  odom_x_  += ds * cosf(th_mid);
  odom_y_  += ds * sinf(th_mid);
  odom_th_  = normalizeAngle(odom_th_ + dth);
}

/* ---------------- IMU ---------------- */
void monster_sensor::updateIMU() {
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

void monster_sensor::calibrateGyro() {
  if (!imu_ok_) return;
  imu_.SEN.gyro_cali_start();
  const uint32_t t0 = millis();
  while (!imu_.SEN.gyro_cali_get_done()) {
    imu_.update();
    if (millis() - t0 > 5000) break;
  }
}

/* --------- OLLO (touch/IR) & Sonar --------- */
#if MONSTER_EXT_SENSORS
void monster_sensor::initBumper() {
  // OLLO ports 3 & 4 for touch (TB3 convention). Safe if not present.
  ollo_.begin(3, TOUCH_SENSOR);
  ollo_.begin(4, TOUCH_SENSOR);
}

void monster_sensor::initIR() {
  // OLLO port 2 for IR (TB3 convention). Safe if not present.
  ollo_.begin(2, IR_SENSOR);
}

void monster_sensor::initSonar() {
  sonar_.trig = MONSTER_SONAR_TRIG_PIN;
  sonar_.echo = MONSTER_SONAR_ECHO_PIN;
  pinMode(sonar_.trig, OUTPUT);
  pinMode(sonar_.echo, INPUT);
  sonar_store_ = 0.0f;
}

void monster_sensor::updateSonar(uint32_t /*now_ms*/) {
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
  // else: keep last value (simple hold on timeout)
}

float monster_sensor::sonarMeters() const {
  return sonar_store_;
}
#endif

/* ------------- Illumination ------------- */
float monster_sensor::lightLevel() const {
  // A1 on OpenCR is a free analog pin; if you wire a photoresistor you can use this.
  return (float)analogRead(A1);
}

/* ---------------- LEDs ---------------- */
void monster_sensor::initLEDs() {
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

void monster_sensor::setLedPattern(double lin, double ang) {
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
