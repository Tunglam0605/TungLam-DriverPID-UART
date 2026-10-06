/*
  01_SingleMotor_Basic
  Board example: Arduino Mega 2560
  Driver UART: Serial2, 115200 baud

  Positive command -> DIR bit = 1
  Negative command -> DIR bit = 0
*/

#include <TungLam_DriverPID_UART.h>

TungLamPIDBus pid;

void setup() {
  Serial.begin(115200);
  pid.begin(Serial2, 115200);

  // Motor ID 1, immediate command.
  pid.setMotor(1, 120);
  delay(1000);

  pid.stop(1);
  delay(500);

  pid.setMotor(1, -120);
  delay(1000);

  pid.stop(1);
}

void loop() {
  // update() is safe to call even when using only immediate commands.
  pid.update();
}
