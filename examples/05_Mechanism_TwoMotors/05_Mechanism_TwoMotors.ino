/*
  05_Mechanism_TwoMotors
  Demonstrates that the library is not tied to a robot base.

  Motor 10 and motor 11 can be any independent mechanisms on the same UART bus.
*/

#include <TungLam_DriverPID_UART.h>

TungLamPIDBus pid;

void setup() {
  pid.begin(Serial2, 115200);

  pid.setMotorMotionProfile(10, 200.0f, 500.0f);
  pid.setMotorMotionProfile(11, 600.0f, 900.0f);

  pid.setTarget(10, 180);
  pid.setTarget(11, -100);
}

void loop() {
  pid.update();
}
