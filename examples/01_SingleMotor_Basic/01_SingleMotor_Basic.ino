/*
  01_SingleMotor_Basic
  ==============================================================================
  MỤC ĐÍCH
  ------------------------------------------------------------------------------
  Ví dụ cơ bản nhất để kiểm tra giao tiếp UART với một Driver Smart PID.

  Phần cứng minh họa:
  - Arduino Mega 2560.
  - Driver PID nối với UART phần cứng Serial2.
  - Baudrate: 115200.

  QUY ƯỚC
  ------------------------------------------------------------------------------
  Lệnh dương  -> bit DIR = 1.
  Lệnh âm     -> bit DIR = 0.
  Giá trị 0   -> tốc độ bằng 0, giữ bit chiều gần nhất.

  Ví dụ này dùng setMotor(), tức là gửi lệnh trực tiếp và không đi qua
  motion profile tăng/giảm tốc.
*/

#include <TungLam_DriverPID_UART.h>

TungLamPIDBus pid;

void setup() {
  // Serial dùng để debug nếu sau này cần in thông tin ra máy tính.
  Serial.begin(115200);

  // Khởi tạo UART điều khiển Driver PID trên Serial2.
  pid.begin(Serial2, 115200);

  // Cho motor ID 1 chạy ngay theo chiều dương với tốc độ 120.
  pid.setMotor(1, 120);
  delay(1000);

  // Dừng ngay motor ID 1 bằng cách gửi speed = 0.
  pid.stop(1);
  delay(500);

  // Cho motor chạy ngược lại với cùng độ lớn tốc độ.
  pid.setMotor(1, -120);
  delay(1000);

  // Dừng motor sau khi kết thúc chuỗi kiểm tra.
  pid.stop(1);
}

void loop() {
  // update() vẫn có thể gọi liên tục ngay cả khi ví dụ chủ yếu dùng setMotor().
  // Hàm này cần thiết cho watchdog, refresh định kỳ, ramp và hardBrake nếu các
  // chức năng đó được bật ở những project thực tế.
  pid.update();
}
