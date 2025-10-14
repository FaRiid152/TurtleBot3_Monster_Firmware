#include "../include/monster_controller.h"
#include "../include/monster_motor.h"   // <-- include the real motor header here

bool monster_controller::init(float max_lin_vel, float max_ang_vel) {
  max_lin_ = max_lin_vel;
  max_ang_ = max_ang_vel;
  v_goal_  = 0.0f;
  w_goal_  = 0.0f;
  return true;
}

void monster_controller::setGoal(float linear_mps, float angular_rps) {
  v_goal_ = clamp(linear_mps, -max_lin_, max_lin_);
  w_goal_ = clamp(angular_rps, -max_ang_, max_ang_);
}
/*
bool monster_controller::update() {
  if (!motor_) return false;

  // Differential drive kinematics:
  // left = v - w * (track/2), right = v + w * (track/2)
  const float half_track = 0.5f * WHEEL_SEPARATION;
  float v_left  = v_goal_ - w_goal_ * half_track;
  float v_right = v_goal_ + w_goal_ * half_track;

  // Saturate to linear limits (m/s)
  v_left  = clamp(v_left,  -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  v_right = clamp(v_right, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);

  // Send in m/s; motor class converts to goal units & applies per-wheel signs.
  motor_->commandSides(v_left, v_right);
  return true;
}
*/

bool monster_controller::update() {
  if (!motor_) return false;

  const float half_track = 0.5f * WHEEL_SEPARATION;

  // --- tiny state machine held locally (no header edits) ---
  static bool     spin_mode = false;
  static uint32_t last_us   = 0;

  const uint32_t now = micros();
  const float dt = (last_us == 0) ? 0.0f : (now - last_us) * 1e-6f;
  last_us = now;

  // Hysteresis + pulse spec (locals; no new macros)
  const float W_ENTER = deg2rad(15.0f);      // enter spin if |ω| ≥ 3°/s
  const float W_EXIT  = deg2rad(5.0f);      // exit spin when |ω| ≤ 1°/s
  const float PULSE_PERIOD = 0.05f;          // seconds
  const float PULSE_ANGLE  = deg2rad(5.0f); // 1° per pulse
  float w_pulse = PULSE_ANGLE / PULSE_PERIOD;

  // physical ω limit from wheel rail
  const float w_ps = MAX_LINEAR_VELOCITY / half_track;
  if (w_pulse > w_ps) w_pulse = w_ps;       // never demand impossible ω

  // mode decision from incoming ω request
  const float w_req = (w_goal_ > 0) ? w_goal_ : -w_goal_;
  if (!spin_mode && w_req >= W_ENTER) spin_mode = true;
  if (spin_mode  && w_req <= W_EXIT)  spin_mode = false;

  float v_cmd = 0.0f, w_cmd = 0.0f;

  if (spin_mode) {
    // Quantized heading: v=0, constant small ω whose sign follows the request.
    v_cmd = 0.0f;
    w_cmd = (w_goal_ >= 0.0f) ? w_pulse : -w_pulse;
    // (Optionally hold w_cmd piecewise-constant for PULSE_PERIOD; not required—
    // the constant value already yields 1° every 0.5 s.)
  } else {
    // Normal heading-priority limiter (ω gets first claim on wheel headroom)
    float w = w_goal_;
    if (w >  w_ps) w =  w_ps;
    if (w < -w_ps) w = -w_ps;
    float v = v_goal_;
    // headroom for translation given current ω
    const float v_head = MAX_LINEAR_VELOCITY - half_track * fabsf(w);
    v = (v_head <= 0.0f) ? 0.0f : clamp(v, -v_head, v_head);
    w_cmd = clamp(w, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
    v_cmd = clamp(v, -MAX_LINEAR_VELOCITY,  MAX_LINEAR_VELOCITY);
  }

  // Map body twist → wheel rim speeds (m/s)
  float v_left  = v_cmd - w_cmd * half_track;
  float v_right = v_cmd + w_cmd * half_track;

  // Let motor_->commandSides() do curvature-preserving rail if anything still exceeds wheel caps.
  motor_->commandSides(v_left, v_right);
  return true;
}

void monster_controller::getRCdata(float* cmd_vel) {
  cmd_vel[0] = v_goal_;
  cmd_vel[1] = w_goal_;
}
