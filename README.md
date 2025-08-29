# Monster Robot Firmware

> Embedded firmware for a four-wheel differential-drive robot using OpenCR/OpenCM, Dynamixel motors, and micro-ROS integration.

![Arduino](https://img.shields.io/badge/Arduino-OpenCR%2FOpenCM-blue.svg)
![ROS 2](https://img.shields.io/badge/ROS2-micro--ROS-green.svg)
![License](https://img.shields.io/badge/license-MIT-lightgrey.svg)

---

## Overview

This repository contains the **firmware stack** for the Monster robot — a four-wheel drive mobile platform designed for experimentation with embedded control, sensing, and ROS 2 connectivity.  

The system is built around:
- **OpenCR/OpenCM controllers**,
- **Dynamixel smart servos** in velocity mode,
- **On-board IMU, sonar, LEDs, and user button**,
- **micro-ROS integration** for command and telemetry exchange.

The firmware provides both a **standalone (plain)** mode and a **ROS 2 integrated** mode, with safety interlocks and diagnostic feedback.

---

## Features

- 🔧 **Differential-drive kinematics** with configurable wheel geometry  
- ⚙️ **Dynamixel motor driver** (velocity mode, four motors, left/right control)  
- 📡 **Sensor integration**:
  - IMU (quaternion, gyro, accel, mag)
  - Sonar (HC-SR04 style, TB3-compatible pins)
  - Battery voltage monitoring
  - User button
  - LED cluster for motion intent
- 🟢 **Diagnostics module**:
  - Heartbeat LED
  - Error code blinking
- 🖥️ **Controller modes**:
  - **Plain mode**: fixed-period loop, no ROS
  - **ROS 2 mode**: subscribes to `cmd_vel` (geometry_msgs/Twist) via micro-ROS, with safety checks
- 🛡️ **Safety gates**:
  - Command timeout → stop
  - Low battery cutoff
  - Sonar-based obstacle stop
  - Velocity clamping

---

## Repository Structure

Monster.h # High-level interface
monster_core.cpp # Bring-up, main loop, Plain/ROS2 integration
monster_config.h # Geometry, motor IDs, velocity limits, conversions
monster_controller.* # Motion controller (maps v,w -> wheel speeds)
monster_motor.* # Dynamixel driver for 4 motors
monster_sensor.* # IMU, sonar, battery, LEDs, button
monster_diagnosis.* # LED heartbeat and error codes

---

## Getting Started

### Requirements
- [Arduino IDE](https://www.arduino.cc/en/software) or [PlatformIO](https://platformio.org/)
- [OpenCR board packages](https://emanual.robotis.com/docs/en/parts/controller/opencr10/)
- [DynamixelWorkbench library](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/)
- [micro-ROS Arduino library](https://micro.ros.org/docs/tutorials/)

### Build & Flash
1. Clone this repository:
   ```bash
   git clone https://github.com/your-username/monster-robot.git
   cd monster-robot
Open Monster.h (or a provided .ino sketch) in Arduino IDE.

Select OpenCR (or OpenCM) as the board.

Build and upload.