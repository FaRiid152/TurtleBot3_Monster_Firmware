#include "../include/monster_controller.h"

bool MonsterController::init(float max_lin_vel, float max_ang_vel) {
  max_lin_ = max_lin_vel;
  max_ang_ = max_ang_vel;
  return true;
}

void MonsterController::getRCdata(float* cmd_vel) {
  // placeholder RC input (0,0)
  cmd_vel[0] = 0.0f; // linear
  cmd_vel[1] = 0.0f; // angular
}
