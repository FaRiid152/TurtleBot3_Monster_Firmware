#include "../include/monster_diagnosis.h"

bool MonsterDiagnosis::init() {
  MonsterDiagConfig def;        // default settings
  return init(def);
}

bool MonsterDiagnosis::init(const MonsterDiagConfig& cfg) {
  cfg_ = cfg;
  pinMode(cfg_.led_pin, OUTPUT);
  led_state_ = false;
  setLed(false);
  hb_last_  = millis();
  err_last_ = hb_last_;
  err_phase_ = ErrPhase::IDLE;
  mode_ = Mode::OFF;
  err_code_ = 0;
  err_count_ = 0;
  return true;
}

void MonsterDiagnosis::setMode(Mode m) {
  mode_ = m;
  if (mode_ == Mode::ON)  setLed(true);
  if (mode_ == Mode::OFF) setLed(false);

  // Reset dynamic timers
  hb_last_   = millis();
  err_last_  = hb_last_;
  err_phase_ = ErrPhase::IDLE;
  err_count_ = 0;
  if (mode_ != Mode::ERROR_CODE) err_code_ = 0;
}

void MonsterDiagnosis::setHeartbeat(uint16_t period_ms) {
  cfg_.heartbeat_ms = period_ms;
}

void MonsterDiagnosis::flashError(uint8_t code) {
  if (code == 0) {
    err_code_ = 0;
    setMode(Mode::OFF);
    return;
  }
  err_code_  = code;
  err_count_ = 0;
  err_phase_ = ErrPhase::IDLE;
  err_last_  = millis();
  mode_      = Mode::ERROR_CODE;
}

void MonsterDiagnosis::forceOn()  { setLed(true); }
void MonsterDiagnosis::forceOff() { setLed(false); }

void MonsterDiagnosis::tick() {
  const uint32_t now = millis();
  switch (mode_) {
    case Mode::OFF:        break;
    case Mode::ON:         break;
    case Mode::HEARTBEAT:  updateHeartbeat(now); break;
    case Mode::ERROR_CODE: updateError(now);     break;
  }
}

void MonsterDiagnosis::setLed(bool on) {
  led_state_ = on;
  const bool phys = cfg_.active_high ? on : !on;
  digitalWrite(cfg_.led_pin, phys ? HIGH : LOW);
}

void MonsterDiagnosis::updateHeartbeat(uint32_t now) {
  if (cfg_.heartbeat_ms == 0) return;
  if (now - hb_last_ >= cfg_.heartbeat_ms / 2) {
    hb_last_ = now;
    setLed(!led_state_);
  }
}

void MonsterDiagnosis::updateError(uint32_t now) {
  if (err_code_ == 0) { setLed(false); return; }

  switch (err_phase_) {
    case ErrPhase::IDLE:
      err_count_ = 0;
      err_phase_ = ErrPhase::FLASH_ON;
      err_last_  = now;
      setLed(true);
      break;

    case ErrPhase::FLASH_ON:
      if (now - err_last_ >= cfg_.short_ms) {
        setLed(false);
        err_last_  = now;
        err_phase_ = ErrPhase::FLASH_GAP;
        err_count_++;
      }
      break;

    case ErrPhase::FLASH_GAP:
      if (now - err_last_ >= cfg_.gap_ms) {
        if (err_count_ >= err_code_) {
          err_phase_ = ErrPhase::GROUP_PAUSE;
          err_last_  = now;
        } else {
          err_phase_ = ErrPhase::FLASH_ON;
          err_last_  = now;
          setLed(true);
        }
      }
      break;

    case ErrPhase::GROUP_PAUSE:
      if (now - err_last_ >= cfg_.pause_ms) {
        err_phase_ = ErrPhase::FLASH_ON;
        err_last_  = now;
        err_count_ = 0;
        setLed(true);
      }
      break;
  }
}
