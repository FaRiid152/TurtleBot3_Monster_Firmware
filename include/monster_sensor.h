#ifndef MONSTER_SENSOR_H
#define MONSTER_SENSOR_H

#include <Arduino.h>
#include <IMU.h>     // OpenCR IMU (cIMU)
#include "OLLO.h"    // OLLO sensors (touch/IR)

//=================== Compatibility / Fallbacks ===================//

#ifndef BDPIN_GPIO_1
  // Sonar default pins (HC-SR04 style): Trig=D8, Echo=D9
  #define MONSTER_SONAR_TRIG_PIN  8
  #define MONSTER_SONAR_ECHO_PIN  9
#else
  // OpenCR default (same as TurtleBot3)
  #define MONSTER_SONAR_TRIG_PIN  BDPIN_GPIO_1
  #define MONSTER_SONAR_ECHO_PIN  BDPIN_GPIO_2
#endif

#ifndef BDPIN_GPIO_4
  // LED cluster defaults if not OpenCR
  #define MONSTER_LED_FL  4
  #define MONSTER_LED_FR  5
  #define MONSTER_LED_BL  6
  #define MONSTER_LED_BR  7
#else
  // OpenCR mapping (same as TB3)
  #define MONSTER_LED_FL  BDPIN_GPIO_4
  #define MONSTER_LED_FR  BDPIN_GPIO_6
  #define MONSTER_LED_BL  BDPIN_GPIO_8
  #define MONSTER_LED_BR  BDPIN_GPIO_10
#endif

// User button fallback (active LOW on INPUT_PULLUP)
#ifndef getPushButton
  #ifndef MONSTER_BTN_PIN
    #define MONSTER_BTN_PIN  2
  #endif
  static inline uint8_t monster_getPushButton() {
    pinMode(MONSTER_BTN_PIN, INPUT_PULLUP);
    return digitalRead(MONSTER_BTN_PIN) == LOW ? 1 : 0;
  }
  #define getPushButton() monster_getPushButton()
#endif

//=================== Battery Voltage ===================//
float getPowerInVoltage(void);
// Prefer OpenCR's built-in function if available
/*#if defined(ARDUINO_OPENCR) || defined(__OPENCR__)
  float getPowerInVoltage(void);   // Provided by OpenCR core
#else
  // Generic analog fallback (only if compiling on other boards)
  #ifndef MONSTER_VBAT_PIN
    #define MONSTER_VBAT_PIN  A0
  #endif
  #ifndef MONSTER_VBAT_VREF
    #define MONSTER_VBAT_VREF  3.3f
  #endif
  #ifndef MONSTER_VBAT_DIVIDER
    #define MONSTER_VBAT_DIVIDER  11.0f   // Adjust if you wire your own divider
  #endif
  static inline float getPowerInVoltage() {
    int raw = analogRead(MONSTER_VBAT_PIN);
    return (raw * (MONSTER_VBAT_VREF / 1023.0f)) * MONSTER_VBAT_DIVIDER;
  }
#endif
*/
//===============================================================//

// TB3-compatible scaling
#define ACCEL_FACTOR  0.000598550415f
#define GYRO_FACTOR   0.0010642f
#define MAG_FACTOR    15e-8f

struct MonsterLedPins {
  int front_left;
  int front_right;
  int back_left;
  int back_right;
};

struct MonsterSonarPins {
  int trig;
  int echo;
};

class MonsterSensor {
public:
  MonsterSensor();
  bool begin();
  void update();

  // IMU
  void calibrateGyro();
  const float* quat()   const { return quat_;  }
  const float* gyro()   const { return gyro_;  }
  const float* accel()  const { return accel_; }
  const float* mag()    const { return mag_;   }

  // Battery & button
  float    batteryVoltage() const { return getPowerInVoltage(); }
  uint8_t  userButton()     const { return getPushButton(); }

  // Sonar & illumination
  float    sonarMeters() const;
  float    lightLevel() const;

  // LEDs
  void setLedPattern(double v_linear, double v_angular);

private:
  void initBumper();
  void initIR();
  void initSonar();
  void initLEDs();

  void updateIMU();
  void updateSonar(uint32_t now_ms);

private:
  cIMU imu_;
  OLLO ollo_;
  bool imu_ok_ = false;

  MonsterLedPins   led_{};
  MonsterSonarPins sonar_{};
  float sonar_store_ = 0.0f;

  float quat_[4]  = {0,0,0,1};
  float gyro_[3]  = {0,0,0};
  float accel_[3] = {0,0,0};
  float mag_[3]   = {0,0,0};
};

#endif // MONSTER_SENSOR_H
