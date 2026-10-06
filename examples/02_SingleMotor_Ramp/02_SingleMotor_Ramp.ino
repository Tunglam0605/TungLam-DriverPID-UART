/*
  02_SingleMotor_Ramp
  Non-blocking time-based acceleration/deceleration.

  The ramp is based on elapsed time, not the number of loop() iterations.
*/

#include <TungLam_DriverPID_UART.h>

TungLamPIDBus pid;

uint32_t phaseStart = 0;
uint8_t phase = 0;

void setup() {
  pid.begin(Serial2, 115200);

  // 300 command-units/s acceleration.
  // 700 command-units/s deceleration.
  pid.setMotorMotionProfile(1, 300.0f, 700.0f);

  pid.setTarget(1, 220);
  phaseStart = millis();
}

void loop() {
  pid.update();

  const uint32_t elapsed = millis() - phaseStart;

  if (phase == 0 && elapsed >= 2000) {
    pid.softStop(1);
    phase = 1;
    phaseStart = millis();
  } else if (phase == 1 && elapsed >= 1000) {
    pid.setTarget(1, -180);
    phase = 2;
    phaseStart = millis();
  } else if (phase == 2 && elapsed >= 2000) {
    pid.stop(1);
    phase = 3;
  }
}
