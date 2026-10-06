/*
  04_ActiveBrake
  ==============================================================================
  MỤC ĐÍCH
  ------------------------------------------------------------------------------
  Minh họa chức năng phanh ngược chủ động hardBrake().

  NGUYÊN LÝ
  ------------------------------------------------------------------------------
  hardBrake() KHÔNG phải một opcode BRAKE riêng của giao thức UART 3 byte.

  Thư viện thực hiện:
  1. xác định chiều motor đang chạy;
  2. gửi một command ngược chiều với độ lớn strength;
  3. giữ command đó trong pulseMs;
  4. tự động gửi speed = 0 sau khi hết thời gian hãm.

  CẢNH BÁO
  ------------------------------------------------------------------------------
  Hãm ngược có thể tạo dòng điện lớn và xung lực cơ khí. Khi thử trên robot
  thật, luôn bắt đầu với strength nhỏ và pulseMs ngắn.
*/

#include <TungLam_DriverPID_UART.h>

TungLamPIDBus pid;

void setup() {
  // Khởi tạo Driver PID qua Serial2.
  pid.begin(Serial2, 115200);

  // Cho motor ID 1 chạy trước để tạo trạng thái đang chuyển động.
  pid.setMotor(1, 150);
  delay(1500);

  // Hãm ngược:
  // - strength = 40;
  // - thời gian xung hãm = 25 ms.
  //
  // Nên coi đây là thông số thử ban đầu, không phải giá trị tối ưu cho mọi
  // động cơ và mọi nguồn cấp.
  pid.hardBrake(1, 40, 25);
}

void loop() {
  // Bắt buộc gọi update() để thư viện theo dõi thời gian pulse hãm và tự động
  // gửi lệnh speed = 0 khi hết 25 ms.
  pid.update();
}
