#include "../include/monster_motor.h"

// We’ll use your conversion helper from monster_config.h:
//   inline int32_t mps_to_goalU(float mps)
// which maps m/s → XL430 goalVelocity units (~0.229 rpm/unit)

monster_motor::monster_motor() {}

void monster_motor::begin() {
  const char* log = nullptr;
  bool ok = dxl_wb.init(DEVICE_NAME, DXL_BAUD, &log);
  (void)ok; (void)log;   // You can print log to Serial if desired
  // NOTE: On OpenCR, DEVICE_NAME "" selects the built-in Dynamixel port.
}

bool monster_motor::isPresent(uint8_t id) {
  const char* log = nullptr;
  uint16_t model = 0;
  bool ok = dxl_wb.ping(id, &model, &log);
  return ok;
}

void monster_motor::configureOne(uint8_t id) {
  if (!isPresent(id)) return;

  // Set wheel (velocity) mode. In Workbench, wheelMode() sets
  // Operating Mode to Velocity; you can pass accel=0 to skip profile accel.
  const char* log = nullptr;
  (void)dxl_wb.wheelMode(id, 0, &log);

  // Zero any previous motion
  (void)dxl_wb.goalVelocity(id, (int32_t)0);
}

void monster_motor::configureAll() {
  configureOne(ID_L1);
  configureOne(ID_L2);
  configureOne(ID_R1);
  configureOne(ID_R2);
}

void monster_motor::commandSides(float v_left_mps, float v_right_mps) {
  // Convert to Dynamixel goalVelocity units
  const int32_t uL = monster_config::mpsToGoalU(v_left_mps);
  const int32_t uR = monster_config::mpsToGoalU(v_right_mps);

  const char* log = nullptr;

  // Left pair
  if (isPresent(ID_L1)) (void)dxl_wb.goalVelocity(ID_L1, (int32_t)(SIGN_L1 * uL), &log);
  if (isPresent(ID_L2)) (void)dxl_wb.goalVelocity(ID_L2, (int32_t)(SIGN_L2 * uL), &log);

  // Right pair
  if (isPresent(ID_R1)) (void)dxl_wb.goalVelocity(ID_R1, (int32_t)(SIGN_R1 * uR), &log);
  if (isPresent(ID_R2)) (void)dxl_wb.goalVelocity(ID_R2, (int32_t)(SIGN_R2 * uR), &log);
}
/*
bool monster_motor::readPositions(int32_t p[4]) {
  const char* log = nullptr; bool ok = true; int32_t v = 0;
  // Left (raw ticks, no sign)
  if (isPresent(ID_L1)) { ok &= dxl_wb.itemRead(ID_L1, "Present_Position", &v, &log); p[0] = v; } else { p[0]=0; ok=false; }
  if (isPresent(ID_L2)) { ok &= dxl_wb.itemRead(ID_L2, "Present_Position", &v, &log); p[1] = v; } else { p[1]=0; ok=false; }
  // Right (raw ticks, no sign)
  if (isPresent(ID_R1)) { ok &= dxl_wb.itemRead(ID_R1, "Present_Position", &v, &log); p[2] = v; } else { p[2]=0; ok=false; }
  if (isPresent(ID_R2)) { ok &= dxl_wb.itemRead(ID_R2, "Present_Position", &v, &log); p[3] = v; } else { p[3]=0; ok=false; }
  return ok;
}


bool monster_motor::readVelocities(int32_t u[4]) {
  const char* log = nullptr; bool ok = true; int32_t v=0;
  if (isPresent(ID_L1)) { ok &= dxl_wb.itemRead(ID_L1, "Present_Velocity", &v, &log); u[0] = SIGN_L1 * v; } else { u[0]=0; ok=false; }
  if (isPresent(ID_L2)) { ok &= dxl_wb.itemRead(ID_L2, "Present_Velocity", &v, &log); u[1] = SIGN_L2 * v; } else { u[1]=0; ok=false; }
  if (isPresent(ID_R1)) { ok &= dxl_wb.itemRead(ID_R1, "Present_Velocity", &v, &log); u[2] = SIGN_R1 * v; } else { u[2]=0; ok=false; }
  if (isPresent(ID_R2)) { ok &= dxl_wb.itemRead(ID_R2, "Present_Velocity", &v, &log); u[3] = SIGN_R2 * v; } else { u[3]=0; ok=false; }
  return ok;
}
  */
 bool monster_motor::readPositions(int32_t p[4]) {
  const char* log=nullptr; bool ok=true; int32_t v=0;
  ok &= dxl_wb.itemRead(ID_L1, "Present_Position", &v, &log); p[0]=v;
  ok &= dxl_wb.itemRead(ID_L2, "Present_Position", &v, &log); p[1]=v;
  ok &= dxl_wb.itemRead(ID_R1, "Present_Position", &v, &log); p[2]=v;
  ok &= dxl_wb.itemRead(ID_R2, "Present_Position", &v, &log); p[3]=v;
  return ok;
}
bool monster_motor::readVelocities(int32_t u[4]) {
  const char* log=nullptr; bool ok=true; int32_t v=0;
  ok &= dxl_wb.itemRead(ID_L1, "Present_Velocity", &v, &log); u[0]=v;
  ok &= dxl_wb.itemRead(ID_L2, "Present_Velocity", &v, &log); u[1]=v;
  ok &= dxl_wb.itemRead(ID_R1, "Present_Velocity", &v, &log); u[2]=v;
  ok &= dxl_wb.itemRead(ID_R2, "Present_Velocity", &v, &log); u[3]=v;
  return ok;
}

