#ifndef MONSTER_DIAGNOSIS_H
#define MONSTER_DIAGNOSIS_H

#include <Arduino.h>

// Config is outside the class to avoid old-GCC nested default-arg issues.
struct MonsterDiagConfig {
  uint8_t  led_pin       = LED_BUILTIN; // status LED pin
  uint16_t heartbeat_ms  = 500;         // heartbeat period
  uint16_t short_ms      = 120;         // short blink ON time (error flash)
  uint16_t gap_ms        = 180;         // gap between flashes in a group
  uint16_t pause_ms      = 900;         // pause between groups
  bool     active_high   = true;        // true=HIGH is ON, false=LOW is ON
};

class monster_diagnosis {
public:
  enum class Mode : uint8_t {
    OFF,
    ON,
    HEARTBEAT,
    ERROR_CODE
  };

  monster_diagnosis() = default;

  // Overloads instead of default argument
  bool init();                                   // uses default MonsterDiagConfig()
  bool init(const MonsterDiagConfig& cfg);       // custom config

  inline void setConnected(bool connected) { setMode(connected ? Mode::ON : Mode::HEARTBEAT); }

  void setMode(Mode m);
  void setHeartbeat(uint16_t period_ms);
  void flashError(uint8_t code);                 // 0 cancels -> OFF
  void forceOn();
  void forceOff();
  void tick();                                   // call frequently

  // Back-compat alias
  inline void showLedStatus(bool connected) { setConnected(connected); }

private:
  void setLed(bool on);
  void updateHeartbeat(uint32_t now);
  void updateError(uint32_t now);

  MonsterDiagConfig cfg_{};
  Mode    mode_       = Mode::OFF;
  bool    led_state_  = false;

  // Heartbeat
  uint32_t hb_last_   = 0;

  // Error code state
  uint8_t  err_code_  = 0;
  uint8_t  err_count_ = 0;
  uint32_t err_last_  = 0;
  enum class ErrPhase : uint8_t { IDLE, FLASH_ON, FLASH_GAP, GROUP_PAUSE } err_phase_ = ErrPhase::IDLE;
};

#endif // MONSTER_DIAGNOSIS_H
