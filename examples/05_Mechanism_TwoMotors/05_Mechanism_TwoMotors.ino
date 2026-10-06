/*
  05_Mechanism_TwoMotors
  ==============================================================================
  MỤC ĐÍCH
  ------------------------------------------------------------------------------
  Chứng minh thư viện không chỉ dành cho đế robot Mecanum.

  Hai motor ID 10 và 11 có thể là hai cơ cấu hoàn toàn độc lập, ví dụ:
  - cơ cấu nâng;
  - cơ cấu gắp;
  - băng tải;
  - cơ cấu xoay;
  - motor phụ trên robot.

  Mỗi motor có thể có profile tăng/giảm tốc riêng.
*/

#include <TungLam_DriverPID_UART.h>

TungLamPIDBus pid;

void setup() {
  // Khởi tạo bus UART dùng chung.
  pid.begin(Serial2, 115200);

  // Motor 10 chạy êm hơn, tăng chậm và giảm tốc vừa phải.
  pid.setMotorMotionProfile(
      10,
      200.0f,  // tăng tốc [command/giây]
      500.0f   // giảm tốc [command/giây]
  );

  // Motor 11 phản ứng nhanh hơn.
  pid.setMotorMotionProfile(
      11,
      600.0f,  // tăng tốc [command/giây]
      900.0f   // giảm tốc [command/giây]
  );

  // Hai cơ cấu có thể chạy khác chiều và khác tốc độ trên cùng một UART.
  pid.setTarget(10, 180);
  pid.setTarget(11, -100);
}

void loop() {
  // Cập nhật đồng thời motion profile và lịch TX cho cả hai motor.
  pid.update();
}
