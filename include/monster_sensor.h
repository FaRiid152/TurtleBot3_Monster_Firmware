#ifndef MONSTER_SENSOR_H
#define MONSTER_SENSOR_H

#include "monster_config.h"
#include "monster_motor.h"
#include <Arduino.h>
#include <IMU.h>     // OpenCR IMU (cIMU)
#if MONSTER_EXT_SENSORS
  #include "OLLO.h"  // OLLO sensors (touch/IR)
#endif  

//=================== Compatibility / Fallbacks ===================//

//
#if MONSTER_EXT_SENSORS
  #ifndef BDPIN_GPIO_1
  // Sonar default pins (HC-SR04 style): Trig=D8, Echo=D9
    #define MONSTER_SONAR_TRIG_PIN  8
    #define MONSTER_SONAR_ECHO_PIN  9
  #else
  // OpenCR default 
    #define MONSTER_SONAR_TRIG_PIN  BDPIN_GPIO_1
    #define MONSTER_SONAR_ECHO_PIN  BDPIN_GPIO_2
  #endif
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

#if MONSTER_EXT_SENSORS
  struct MonsterSonarPins {
    int trig;
    int echo;
  };
#endif

class monster_sensor {
public:
  monster_sensor();
  bool begin();
  void update();

  // ---- ODOMETRY ----
  void resetOdometry(float x=0.0f, float y=0.0f, float theta=0.0f);
  void updateOdometry();

  // Pose getters (meters, radians)
  float odomX()     const { return odom_x_; }
  float odomY()     const { return odom_y_; }
  float odomTheta() const { return odom_th_; }

  // Wheel feedback (radians, rad/s). Index order: 0=L1,1=L2,2=R1,3=R2
  float wheelPosRad(uint8_t i) const { return (i<4) ? wheel_pos_rad_[i] : 0.0f; }
  float wheelVelRad(uint8_t i) const { return (i<4) ? wheel_vel_rad_[i] : 0.0f; }

  // existing API continues...
  bool imuOk() const { return imu_ok_; }

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
  #if MONSTER_EXT_SENSORS
    void initBumper();
    void initIR();
    void initSonar();
  #endif
  void initLEDs();

  void updateIMU();
  #if MONSTER_EXT_SENSORS
    void updateSonar(uint32_t now_ms);
  #endif
  
private:
  cIMU imu_;
  #if MONSTER_EXT_SENSORS
    OLLO ollo_;
  #endif
  bool imu_ok_ = false;

  // ---- ODOMETRY STATE ----
  // raw encoder last-read (XL430 Present_Position, signed int32)
  int32_t enc_prev_[4] = {0,0,0,0};
  bool    enc_have_prev_ = false;

  // continuous wheel angle and angular velocity
  float wheel_pos_rad_[4] = {0,0,0,0};
  float wheel_vel_rad_[4] = {0,0,0,0};

  // robot pose in odom frame
  float odom_x_ = 0.0f, odom_y_ = 0.0f, odom_th_ = 0.0f;

  // timing for velocity calc
  uint32_t last_odom_us_ = 0;

  // helpers
  static inline float normalizeAngle(float a) {
    while (a >  M_PI) a -= 2.0f*M_PI;
    while (a < -M_PI) a += 2.0f*M_PI;
    return a;
  }


  MonsterLedPins   led_{};

  #if MONSTER_EXT_SENSORS
    MonsterSonarPins sonar_{};
    float sonar_store_ = 0.0f;
  #endif

  float quat_[4]  = {0,0,0,1};
  float gyro_[3]  = {0,0,0};
  float accel_[3] = {0,0,0};
  float mag_[3]   = {0,0,0};
};

#endif // MONSTER_SENSOR_H
