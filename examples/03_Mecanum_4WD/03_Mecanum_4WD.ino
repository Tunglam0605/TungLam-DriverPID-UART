/*
  03_Mecanum_4WD
  Wheel mapping:
    M1 = Front Left
    M2 = Rear Left
    M3 = Rear Right
    M4 = Front Right

  Body convention:
    +vx = forward
    +vy = left
    +wz = rotate left / CCW
*/

#include <TungLam_DriverPID_UART.h>

TungLamPIDBus pid;
TungLamPID4WD robot(pid);

void setup() {
  pid.begin(Serial2, 115200);

  robot.setMotorIDs(1, 2, 3, 4);
  robot.setMotionProfile(450.0f, 900.0f);

  // Optional safety: if application commands stop arriving for 300 ms,
  // force all registered motors to zero.
  pid.setCommandTimeout(300, TungLamPIDTimeoutAction::ImmediateStop);
}

void loop() {
  // In a real robot, vx/vy/wz normally come from PS2, RC, ROS, etc.
  const int16_t vx = 160;
  const int16_t vy = 0;
  const int16_t wz = 50;

  // Re-issue live commands so the watchdog knows the controller is healthy.
  robot.drive(vx, vy, wz);
  robot.update();

  delay(10);
}
