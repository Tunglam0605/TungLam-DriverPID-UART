# TungLam DriverPID UART

Thư viện Arduino do **Nguyễn Khắc Tùng Lâm — Tung Lâm Automation** phát triển để điều khiển **Driver Smart PID qua UART** theo hướng tái sử dụng, dễ mở rộng và phù hợp cho cả robot di động lẫn các cơ cấu độc lập.

Thư viện được phát triển từ cách điều khiển UART đã chạy thực tế trong project `TungLamvsRoBo_2025/Robot0x01`, sau đó tách lại thành kiến trúc rõ ràng hơn để không còn phụ thuộc vào một robot hay một bài thi cụ thể.

## Mục tiêu thiết kế

- Điều khiển motor bằng một giá trị tốc độ có dấu `-255..255`.
- Ẩn phần đóng gói `DIR + ADDRESS + SPEED` khỏi code ứng dụng.
- Dùng tăng tốc/giảm tốc theo **thời gian thực**, không phụ thuộc số vòng `loop()`.
- Không dùng `delay()` trong motion profile và quá trình hãm.
- Hỗ trợ nhiều motor trên cùng một đường UART.
- Có profile riêng cho từng motor hoặc profile mặc định dùng chung.
- Có dừng mềm, dừng ngay, watchdog và phanh ngược chủ động.
- Hạn chế gửi UART dư thừa bằng TX scheduler.
- Có lớp riêng điều khiển đế 4 bánh Mecanum.
- Giữ thứ tự bánh tương thích với hệ sinh thái thư viện Tung Lâm Automation.
- Dùng bộ nhớ tĩnh, không cấp phát động trong quá trình chạy.

## Kiến trúc

```text
Chương trình người dùng / PS2 / ROS / thuật toán
                    |
                    v
      +-----------------------------+
      |  Cơ cấu độc lập / PID4WD    |
      +-----------------------------+
                    |
                    v
          Motion Profile từng motor
                    |
                    v
              TungLamPIDBus
                    |
                    v
             Đóng frame UART
                    |
                    v
          Driver Smart PID vật lý
```

Nguyên tắc chính là:

> Phần Mecanum không cần biết frame UART được đóng như thế nào, và tầng UART không cần biết motor đang thuộc đế xe hay một cơ cấu khác.

## Giao thức UART

Khung lệnh đang sử dụng gồm 3 byte:

```text
Byte 0: [DIR:1][ADDRESS:7]
Byte 1: SPEED 0..255
Byte 2: 0xFF
```

Ví dụ motor ID 3 chạy theo chiều dương với tốc độ 200:

```text
Byte0 = 0x83
Byte1 = 0xC8
Byte2 = 0xFF
```

API bên ngoài dùng tốc độ có dấu:

| Lệnh | Ý nghĩa |
|---:|---|
| `+1 .. +255` | chạy theo chiều dương logic |
| `0` | dừng |
| `-1 .. -255` | chạy theo chiều âm logic |

Nhờ đó code ứng dụng không cần tự tách `speed` và `direction`.

## Bắt đầu nhanh

```cpp
#include <TungLam_DriverPID_UART.h>

TungLamPIDBus pid;

void setup() {
  // Arduino Mega: dùng UART phần cứng Serial2, baud 115200.
  pid.begin(Serial2, 115200);

  // Tăng tốc 400 command/s, giảm tốc 800 command/s.
  pid.setDefaultMotionProfile(400.0f, 800.0f);
}

void loop() {
  // Motor ID 1 tiến dần đến tốc độ +180.
  pid.setTarget(1, 180);

  // Bắt buộc gọi liên tục để motion profile, watchdog và TX scheduler hoạt động.
  pid.update();
}
```

## Điều khiển trực tiếp một motor

```cpp
pid.setMotor(1, 180);   // Gửi ngay tốc độ +180.
pid.setMotor(1, -180);  // Gửi ngay tốc độ -180.
pid.stop(1);            // Gửi tốc độ 0 ngay lập tức.
```

`setMotor()` phù hợp khi cần phản ứng tức thời. Nếu cần chuyển động êm, dùng `setTarget()`.

## Tăng tốc và giảm tốc

```cpp
pid.setMotorMotionProfile(
  1,       // ID motor
  350.0f,  // tăng tốc, command/s
  700.0f   // giảm tốc, command/s
);

pid.setTarget(1, 220);
```

Profile được tính theo `dt` từ `micros()`, vì vậy robot không bị thay đổi gia tốc chỉ vì chương trình có thêm hoặc bớt code trong `loop()`.

Khi đổi chiều từ dương sang âm hoặc ngược lại, thư viện mặc định sẽ:

```text
+tốc độ
   |
   v
giảm dần về 0
   |
   v
đổi chiều
   |
   v
tăng dần theo chiều mới
```

Cách này giảm sốc cơ khí và tránh nhảy command trực tiếp qua điểm 0.

## Các chế độ dừng

### Dừng mềm

```cpp
pid.softStop(1);
```

Target được đưa về 0 nhưng motor giảm tốc theo `deceleration` đã cấu hình.

### Dừng ngay

```cpp
pid.stop(1);
```

Bỏ qua ramp và ép gửi `speed = 0` ngay.

### Dừng khẩn cấp toàn bộ motor

```cpp
pid.emergencyStop();
```

Tất cả motor đã đăng ký được đưa về 0 ngay lập tức.

> Đây vẫn là dừng bằng phần mềm qua UART, không thay thế nút E-Stop phần cứng trong hệ thống yêu cầu an toàn máy.

## Phanh cứng / hãm ngược chủ động

```cpp
pid.hardBrake(
  1,   // ID motor
  40,  // lực hãm
  25   // thời gian hãm, ms
);
```

`hardBrake()` hiện được triển khai theo nguyên lý:

```text
đang chạy
   |
   v
phát lệnh ngược chiều trong thời gian ngắn
   |
   v
speed = 0
```

Đây **không phải một opcode BRAKE riêng** của frame UART 3 byte. Nó là chiến lược hãm do thư viện tạo ra.

> Hãm ngược có thể gây dòng điện lớn và sốc cơ khí. Khi thử thực tế nên bắt đầu với `strength` nhỏ và `pulseMs` ngắn.

## TX Scheduler

Thư viện không gửi 4 motor ở mọi vòng `loop()` một cách vô điều kiện.

Có 3 tham số chính:

```cpp
pid.setMinTxIntervalMs(5);
pid.setTransmitThreshold(1);
pid.setRefreshPeriodMs(100);
```

Ý nghĩa:

- `setMinTxIntervalMs()`: giới hạn khoảng thời gian tối thiểu giữa các lần gửi khi command thay đổi.
- `setTransmitThreshold()`: chỉ coi thay đổi đủ lớn mới cần phát frame mới.
- `setRefreshPeriodMs()`: dù command không đổi vẫn gửi lại định kỳ.

Cách này giúp giảm tải UART nhưng vẫn giữ khả năng refresh lệnh.

## Watchdog điều khiển

```cpp
pid.setCommandTimeout(
  300,
  TungLamPIDTimeoutAction::ImmediateStop
);
```

Nếu chương trình không phát lệnh ứng dụng mới trong 300 ms, thư viện có thể tự dừng motor.

Ứng dụng điển hình là tay điều khiển PS2:

```cpp
if (!ps2.connected()) {
  pid.emergencyStop();
  return;
}
```

## Điều khiển đế Mecanum 4 bánh

Thứ tự motor được giữ giống thư viện `TungLam-OmniMecanum-4WD`:

```text
                    ĐẦU XE
                       +X
                        ^

             M1                  M4
          TRƯỚC-TRÁI          TRƯỚC-PHẢI

             M2                  M3
           SAU-TRÁI            SAU-PHẢI
```

Quy ước chuyển động:

```text
+vx = tiến
-vx = lùi

+vy = ngang trái
-vy = ngang phải

+wz = quay trái / CCW
-wz = quay phải / CW
```

Ví dụ:

```cpp
#include <TungLam_DriverPID_UART.h>

TungLamPIDBus pid;
TungLamPID4WD robot(pid);

void setup() {
  pid.begin(Serial2, 115200);

  // M1, M2, M3, M4 tương ứng ID 1, 2, 3, 4.
  robot.setMotorIDs(1, 2, 3, 4);

  robot.setMotionProfile(
    450.0f,  // tăng tốc
    900.0f   // giảm tốc
  );
}

void loop() {
  // Tiến đồng thời quay trái.
  robot.drive(180, 0, 60);
  robot.update();
}
```

Nếu một motor được lắp ngược chiều:

```cpp
robot.setWheelInverted(3, true);
```

Không cần sửa lại công thức Mecanum.

## Dùng cho cơ cấu thay vì đế xe

Thư viện không bị khóa vào Mecanum.

Ví dụ hai cơ cấu độc lập:

```cpp
pid.setMotorMotionProfile(10, 200.0f, 500.0f);
pid.setMotorMotionProfile(11, 600.0f, 900.0f);

pid.setTarget(10, 180);
pid.setTarget(11, -100);
```

Motor 10 và 11 có thể là:

- cơ cấu nâng,
- băng tải,
- cơ cấu gắp,
- cơ cấu xoay,
- motor phụ của robot,
- hoặc bất kỳ cụm truyền động nào dùng cùng giao thức UART.

## Các ví dụ đi kèm

- `01_SingleMotor_Basic` — điều khiển trực tiếp một motor.
- `02_SingleMotor_Ramp` — tăng tốc/giảm tốc không block.
- `03_Mecanum_4WD` — điều khiển đế Mecanum 4 bánh.
- `04_ActiveBrake` — thử phanh ngược chủ động.
- `05_Mechanism_TwoMotors` — dùng hai motor cho hai cơ cấu độc lập.

## Khả năng tương thích

Lõi thư viện dựa trên Arduino `Stream` và bộ nhớ tĩnh.

Với Serial có hàm `begin(baud)`:

```cpp
pid.begin(Serial2, 115200);
```

Với một `Stream` đã được cấu hình từ bên ngoài:

```cpp
mySerial.begin(115200);
pid.attach(mySerial);
```

## Cấu trúc thư viện

```text
TungLam-DriverPID-UART/
├── src/
│   ├── TungLam_DriverPID_UART.h
│   └── TungLam_DriverPID_UART.cpp
├── examples/
│   ├── 01_SingleMotor_Basic/
│   ├── 02_SingleMotor_Ramp/
│   ├── 03_Mecanum_4WD/
│   ├── 04_ActiveBrake/
│   └── 05_Mechanism_TwoMotors/
├── extras/
│   ├── PROTOCOL.md
│   └── MIGRATION_ROBOT0x01.md
├── library.properties
├── keywords.txt
├── CHANGELOG.md
└── LICENSE
```

## Giấy phép

MIT License.

Copyright (c) 2026 Nguyễn Khắc Tùng Lâm — Tung Lâm Automation.
