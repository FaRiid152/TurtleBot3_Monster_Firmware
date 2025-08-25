#include "../include/monster_motor.h"

// We’ll use your conversion helper from monster_config.h:
//   inline int32_t mps_to_goalU(float mps)
// which maps m/s → XL430 goalVelocity units (~0.229 rpm/unit)

FourWheelMotor::FourWheelMotor() {}

void FourWheelMotor::begin() {
  const char* log = nullptr;
  bool ok = dxl_wb.init(DEVICE_NAME, DXL_BAUD, &log);
  (void)ok; (void)log;   // You can print log to Serial if desired
  // NOTE: On OpenCR, DEVICE_NAME "" selects the built-in Dynamixel port.
}

bool FourWheelMotor::isPresent(uint8_t id) {
  const char* log = nullptr;
  uint16_t model = 0;
  bool ok = dxl_wb.ping(id, &model, &log);
  return ok;
}

void FourWheelMotor::configureOne(uint8_t id) {
  if (!isPresent(id)) return;

  // Set wheel (velocity) mode. In Workbench, wheelMode() sets
  // Operating Mode to Velocity; you can pass accel=0 to skip profile accel.
  const char* log = nullptr;
  (void)dxl_wb.wheelMode(id, 0, &log);

  // Zero any previous motion
  (void)dxl_wb.goalVelocity(id, (int32_t)0);
}

void FourWheelMotor::configureAll() {
  configureOne(ID_L1);
  configureOne(ID_L2);
  configureOne(ID_R1);
  configureOne(ID_R2);
}

void FourWheelMotor::commandSides(float v_left_mps, float v_right_mps) {
  // Convert to Dynamixel goalVelocity units
  const int32_t uL = monster::mpsToGoalU(v_left_mps);
  const int32_t uR = monster::mpsToGoalU(v_right_mps);

  const char* log = nullptr;

  // Left pair
  if (isPresent(ID_L1)) (void)dxl_wb.goalVelocity(ID_L1, (int32_t)(SIGN_L1 * uL), &log);
  if (isPresent(ID_L2)) (void)dxl_wb.goalVelocity(ID_L2, (int32_t)(SIGN_L2 * uL), &log);

  // Right pair
  if (isPresent(ID_R1)) (void)dxl_wb.goalVelocity(ID_R1, (int32_t)(SIGN_R1 * uR), &log);
  if (isPresent(ID_R2)) (void)dxl_wb.goalVelocity(ID_R2, (int32_t)(SIGN_R2 * uR), &log);
}
