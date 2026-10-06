/*
  03_Mecanum_4WD
  ==============================================================================
  MỤC ĐÍCH
  ------------------------------------------------------------------------------
  Minh họa điều khiển đế robot 4 bánh Mecanum bằng 4 Driver PID trên cùng bus
  UART.

  THỨ TỰ MOTOR
  ------------------------------------------------------------------------------
                    ĐẦU XE
                       +X
                        ^

             M1                  M4
          TRƯỚC-TRÁI          TRƯỚC-PHẢI

             M2                  M3
           SAU-TRÁI            SAU-PHẢI

  HỆ TỌA ĐỘ
  ------------------------------------------------------------------------------
  +vx = tiến
  -vx = lùi
  +vy = đi ngang trái
  -vy = đi ngang phải
  +wz = quay trái / CCW
  -wz = quay phải / CW
*/

#include <TungLam_DriverPID_UART.h>

TungLamPIDBus pid;
TungLamPID4WD robot(pid);

void setup() {
  // Khởi tạo đường UART dùng chung cho 4 Driver PID.
  pid.begin(Serial2, 115200);

  // Gán ID cho 4 bánh theo đúng thứ tự M1, M2, M3, M4.
  // Bốn ID phải khác nhau và nằm trong khoảng 1..127.
  robot.setMotorIDs(1, 2, 3, 4);

  // Áp dụng cùng một profile cho cả 4 bánh:
  // - tăng tốc 450 command/giây;
  // - giảm tốc 900 command/giây.
  robot.setMotionProfile(450.0f, 900.0f);

  // Watchdog an toàn:
  // Nếu 300 ms không nhận lệnh mới từ ứng dụng, toàn bộ motor đã đăng ký sẽ
  // được đưa về 0 ngay lập tức.
  pid.setCommandTimeout(
      300,
      TungLamPIDTimeoutAction::ImmediateStop);
}

void loop() {
  // Trong robot thật, ba giá trị này thường đến từ PS2, RC, ROS2 hoặc thuật
  // toán tự động. Ví dụ hiện tại cho robot tiến đồng thời quay trái.
  const int16_t vx = 160;
  const int16_t vy = 0;
  const int16_t wz = 50;

  // Phát lại command sống đều đặn để watchdog biết bộ điều khiển vẫn hoạt động.
  robot.drive(vx, vy, wz);

  // Cập nhật ramp, watchdog và lịch gửi UART.
  robot.update();

  // Chỉ dùng để mô phỏng vòng điều khiển khoảng 100 Hz trong ví dụ.
  // Thư viện bản thân không phụ thuộc delay() này.
  delay(10);
}
