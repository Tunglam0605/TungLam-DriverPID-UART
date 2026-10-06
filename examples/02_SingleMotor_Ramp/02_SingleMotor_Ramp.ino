/*
  02_SingleMotor_Ramp
  ==============================================================================
  MỤC ĐÍCH
  ------------------------------------------------------------------------------
  Minh họa tăng tốc/giảm tốc theo thời gian thực và hoàn toàn không block.

  Khác với cách tăng tốc dựa vào số vòng lặp, thư viện tính độ thay đổi command
  dựa trên dt thực tế giữa hai lần gọi update(). Vì vậy tốc độ loop() thay đổi
  sẽ ít làm thay đổi thời gian tăng/giảm tốc đã cấu hình.
*/

#include <TungLam_DriverPID_UART.h>

TungLamPIDBus pid;

// Mốc thời gian bắt đầu của pha chuyển động hiện tại.
uint32_t phaseStart = 0;

// Biến trạng thái dùng để lần lượt chạy các pha trong ví dụ.
uint8_t phase = 0;

void setup() {
  // Dùng UART phần cứng Serial2 của Arduino Mega.
  pid.begin(Serial2, 115200);

  // Motor ID 1:
  // - tăng tốc 300 command/giây;
  // - giảm tốc 700 command/giây.
  //
  // Với acceleration = 300, về lý thuyết motor cần khoảng 0,73 giây để
  // command tăng từ 0 lên 220 nếu không có giới hạn khác.
  pid.setMotorMotionProfile(1, 300.0f, 700.0f);

  // Bắt đầu pha 0: tăng dần đến +220.
  pid.setTarget(1, 220);
  phaseStart = millis();
}

void loop() {
  // Phải gọi liên tục để motion profile được cập nhật.
  pid.update();

  const uint32_t elapsed = millis() - phaseStart;

  if (phase == 0 && elapsed >= 2000) {
    // Sau 2 giây, yêu cầu dừng mềm về 0 theo deceleration.
    pid.softStop(1);

    phase = 1;
    phaseStart = millis();

  } else if (phase == 1 && elapsed >= 1000) {
    // Sau khi dừng, yêu cầu chạy chiều âm.
    // Thư viện sẽ tự xử lý quy tắc không nhảy trực tiếp qua điểm 0.
    pid.setTarget(1, -180);

    phase = 2;
    phaseStart = millis();

  } else if (phase == 2 && elapsed >= 2000) {
    // Cuối ví dụ dùng stop() để dừng ngay, bỏ qua ramp.
    pid.stop(1);
    phase = 3;
  }
}
