#ifndef MONSTER_CONTROLLER_H
#define MONSTER_CONTROLLER_H

class MonsterController {
public:
  MonsterController() {}
  bool init(float max_lin_vel, float max_ang_vel);
  void getRCdata(float* cmd_vel);

private:
  float max_lin_;
  float max_ang_;
};

#endif
