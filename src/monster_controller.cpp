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

void monster_controller::getRCdata(float* cmd_vel) {
  cmd_vel[0] = v_goal_;
  cmd_vel[1] = w_goal_;
}
