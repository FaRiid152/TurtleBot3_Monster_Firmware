# Monster Robot Examples

This folder contains minimal sketches to test and verify the **sanity** of each module in the Monster firmware stack.

## How to Use
1. Open the desired `.ino` in Arduino IDE or PlatformIO.
2. Select your board (OpenCR / OpenCM).
3. Upload and monitor serial output at `115200 baud`.

## Demos
- **sensor_demo**  
  Confirms IMU, sonar, and battery readings. Watch serial monitor for values.

- **diagnosis_demo**  
  Tests LED heartbeat and error blink codes. Press the user button to trigger an error flash.

- **motor_demo**  
  Exercises Dynamixel motors in velocity mode. Robot should alternate between forward, spin, and stop.

- **controller_demo**  
  Validates the differential-drive controller. Issues linear + angular velocity goals and applies them to motors.

## Notes
- Ensure robot is on a stand when testing motor code to prevent runaway.  
- For sonar demo, place an object ~20–50 cm in front and check reported distance.  
- Battery readings depend on wiring / divider ratio — compare with a multimeter.  
