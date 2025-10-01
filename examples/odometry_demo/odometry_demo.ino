/*
 * OdometryStepDemo.ino  (fixed: rename y0 -> y0_)
 * Forward in 0.20 m steps (print only at each step),
 * then rotate +pi (print only at each pi/3),
 * reset odom,
 * then reverse in 0.20 m steps and rotate -pi (same print cadence).
 */

#include "Monster.h"
#include "monster_config.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef SERIAL_BAUD
#define SERIAL_BAUD 115200
#endif

// ---- Parameters ----
static const float STEP_DIST_M         = 0.20f;      // print every 0.2 m
static const float TOTAL_DIST_M        = 1.00f;      // go 1.0 m each translation leg
static const float ROT_STEP_RAD        = (float)(M_PI/3.0); // print every pi/3
static const float ROT_TOTAL_RAD       = (float)M_PI;       // rotate pi radians
static const float V_WHEEL_MPS         = 0.08f;      // per-side wheel speed (m/s)
static const uint32_t SAFETY_TIMEOUT_MS= 15000;      // per phase safety

// ---- State machine ----
enum class Phase : uint8_t { FwdInit, FwdRun, RotPInit, RotPRun, ResetInit, RevInit, RevRun, RotNInit, RotNRun, Done };
static Phase phase = Phase::FwdInit;

// ---- Bookkeeping ----
static float x0_=0.f, y0_=0.f, th0_=0.f;     // start pose of current phase
static float next_dist_target=STEP_DIST_M;   // next distance print threshold
static float next_rot_target = ROT_STEP_RAD; // next rotation print threshold (magnitude)
static uint32_t phase_t0_ms = 0;

// ---- Helpers ----
static inline void setWheels(float vL, float vR) { MonsterCore::motors.commandSides(vL, vR); }
static inline void stopWheels()                  { MonsterCore::motors.commandSides(0.f, 0.f); }

static inline float wrapToPi(float a){
  while (a >  M_PI) a -= 2.f*(float)M_PI;
  while (a < -M_PI) a += 2.f*(float)M_PI;
  return a;
}

static inline float distFromStart(float x, float y) {
  const float dx = x - x0_;
  const float dy = y - y0_;
  return sqrtf(dx*dx + dy*dy);
}

static inline float rotDeltaFromStart(float th) {
  return wrapToPi(th - th0_);
}

static inline void printCSV(uint32_t t_ms) {
  const float x  = MonsterCore::sensors.odomX();
  const float y  = MonsterCore::sensors.odomY();
  const float th = MonsterCore::sensors.odomTheta();
  Serial.print(t_ms); Serial.print(',');
  Serial.print(x,6);  Serial.print(',');
  Serial.print(y,6);  Serial.print(',');
  Serial.println(th,6);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {}
  MonsterCore::beginPlain();
  MonsterCore::sensors.resetOdometry(0.f,0.f,0.f);

  Serial.println(F("# OdometryStepDemo"));
  Serial.println(F("# Columns: t_ms,x[m],y[m],theta[rad]"));
  phase = Phase::FwdInit;
}

void loop() {
  MonsterCore::sensors.update();
  const uint32_t now = millis();

  switch (phase) {
    case Phase::FwdInit: {
      x0_ = MonsterCore::sensors.odomX();
      y0_ = MonsterCore::sensors.odomY();
      th0_= MonsterCore::sensors.odomTheta();
      next_dist_target = STEP_DIST_M;
      phase_t0_ms = now;
      Serial.println(F("# PHASE: Forward (print every 0.20 m)"));
      phase = Phase::FwdRun;
      break;
    }
    case Phase::FwdRun: {
      setWheels(+V_WHEEL_MPS, +V_WHEEL_MPS);
      const float d = distFromStart(MonsterCore::sensors.odomX(), MonsterCore::sensors.odomY());
      while (d >= next_dist_target && next_dist_target <= TOTAL_DIST_M + 1e-3f) {
        printCSV(now);
        next_dist_target += STEP_DIST_M;
      }
      if (d >= TOTAL_DIST_M || (now - phase_t0_ms) > SAFETY_TIMEOUT_MS) {
        stopWheels();
        phase = Phase::RotPInit;
      }
      break;
    }

    case Phase::RotPInit: {
      th0_= MonsterCore::sensors.odomTheta();
      next_rot_target = ROT_STEP_RAD;
      phase_t0_ms = now;
      Serial.println(F("# PHASE: Rotate +pi (print every pi/3)"));
      phase = Phase::RotPRun;
      break;
    }
    case Phase::RotPRun: {
      setWheels(-V_WHEEL_MPS, +V_WHEEL_MPS);     // +omega
      const float dth = rotDeltaFromStart(MonsterCore::sensors.odomTheta());
      while (dth >= next_rot_target && next_rot_target <= ROT_TOTAL_RAD + 1e-6f) {
        printCSV(now);
        next_rot_target += ROT_STEP_RAD;
      }
      if (dth >= ROT_TOTAL_RAD || (now - phase_t0_ms) > SAFETY_TIMEOUT_MS) {
        stopWheels();
        phase = Phase::ResetInit;
      }
      break;
    }

    case Phase::ResetInit: {
      stopWheels();
      MonsterCore::sensors.resetOdometry(0.f,0.f,0.f);
      Serial.println(F("# PHASE: Reset odometry to (0,0,0)"));
      phase = Phase::RevInit;
      break;
    }

    case Phase::RevInit: {
      x0_ = MonsterCore::sensors.odomX();
      y0_ = MonsterCore::sensors.odomY();
      th0_= MonsterCore::sensors.odomTheta();
      next_dist_target = STEP_DIST_M;
      phase_t0_ms = now;
      Serial.println(F("# PHASE: Reverse (print every 0.20 m)"));
      phase = Phase::RevRun;
      break;
    }
    case Phase::RevRun: {
      setWheels(-V_WHEEL_MPS, -V_WHEEL_MPS);
      const float d = distFromStart(MonsterCore::sensors.odomX(), MonsterCore::sensors.odomY());
      while (d >= next_dist_target && next_dist_target <= TOTAL_DIST_M + 1e-3f) {
        printCSV(now);
        next_dist_target += STEP_DIST_M;
      }
      if (d >= TOTAL_DIST_M || (now - phase_t0_ms) > SAFETY_TIMEOUT_MS) {
        stopWheels();
        phase = Phase::RotNInit;
      }
      break;
    }

    case Phase::RotNInit: {
      th0_= MonsterCore::sensors.odomTheta();
      next_rot_target = ROT_STEP_RAD;
      phase_t0_ms = now;
      Serial.println(F("# PHASE: Rotate -pi (print every pi/3)"));
      phase = Phase::RotNRun;
      break;
    }
    case Phase::RotNRun: {
      setWheels(+V_WHEEL_MPS, -V_WHEEL_MPS);     // -omega
      const float dth = rotDeltaFromStart(MonsterCore::sensors.odomTheta()); // negative toward -pi
      const float mag = -dth;
      while (mag >= next_rot_target && next_rot_target <= ROT_TOTAL_RAD + 1e-6f) {
        printCSV(now);
        next_rot_target += ROT_STEP_RAD;
      }
      if (mag >= ROT_TOTAL_RAD || (now - phase_t0_ms) > SAFETY_TIMEOUT_MS) {
        stopWheels();
        Serial.println(F("# PHASE: Done"));
        phase = Phase::Done;
      }
      break;
    }

    case Phase::Done: {
      stopWheels();
      break;
    }
  }
}
