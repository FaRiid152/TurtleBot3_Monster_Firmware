#ifndef MONSTER_MOTOR_H
#define MONSTER_MOTOR_H

#include <stdint.h>
#include <DynamixelWorkbench.h>
#include "monster_config.h"

// DEVICE_NAME selection exactly like your working sample
#if defined(__OPENCM904__)
  #define DEVICE_NAME "3"      // OpenCM 485EXP (Serial3)
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""       // OpenCR native Dynamixel port
#else
  // Fallback: let user override at build time if needed
  #ifndef DEVICE_NAME
    #define DEVICE_NAME ""     // default
  #endif
#endif

class monster_motor {
public:
  monster_motor();

  // Bring up workbench at DXL_BAUD (from monster_config.h)
  void begin();

  // Put each present ID into velocity (wheel) mode
  void configureAll();

  // Command left/right side linear speeds (m/s)
  void commandSides(float v_left_mps, float v_right_mps);

  // Feedback (XL430 Present_Position/Velocity). Returns false if any read fails.
  bool readPositions(int32_t out_pos[4]);    // order: L1,L2,R1,R2 (signed ticks)
  bool readVelocities(int32_t out_vel[4]);   // order: L1,L2,R1,R2 (goal units)

  // Optional: expose workbench if you need debugging
  DynamixelWorkbench& wb() { return dxl_wb; }

private:
  DynamixelWorkbench dxl_wb;

  void configureOne(uint8_t id);
  bool isPresent(uint8_t id);
};

#endif
