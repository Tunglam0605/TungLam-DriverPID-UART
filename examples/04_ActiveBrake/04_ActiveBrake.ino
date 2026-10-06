/*
  04_ActiveBrake

  hardBrake() is an active reverse pulse implemented by this library.
  It is NOT a separate documented brake opcode in the 3-byte UART frame.

  Start with LOW strength and SHORT pulse duration.
*/

#include <TungLam_DriverPID_UART.h>

TungLamPIDBus pid;

void setup() {
  pid.begin(Serial2, 115200);

  pid.setMotor(1, 150);
  delay(1500);

  // Brief opposite command, then zero.
  pid.hardBrake(1, 40, 25);
}

void loop() {
  // Required so the non-blocking brake pulse can finish and send zero.
  pid.update();
}
