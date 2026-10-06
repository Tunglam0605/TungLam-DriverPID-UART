# Chuyển đổi từ Robot0x01 sang TungLam DriverPID UART

## Code cũ

Project Robot0x01 tự đóng frame UART trực tiếp:

```cpp
DataTX[i][0] = (direction << 7) | (address & 0x7F);
DataTX[i][1] = speed;
DataTX[i][2] = 0xFF;
Serial2.write(DataTX[i], 3);
```

Tăng tốc/giảm tốc được chia thành nhiều hàm theo từng kiểu chuyển động:

```text
TangtocUart()
TangtocUart2()
TangtocUart3()
TangtocUart4()
TangtocUart5()
StopUart()
```

Cách này đã chạy được trên robot thật nhưng phần UART, chuyển động Mecanum và motion profile bị gắn chặt vào nhau nên khó tái sử dụng cho project khác.

## Kiến trúc mới

```text
Ứng dụng / PS2 / ROS
        |
        v
Mecanum hoặc cơ cấu riêng
        |
        v
Motion Profile từng motor
        |
        v
TungLamPIDBus
        |
        v
Khung UART 3 byte
        |
        v
Driver PID
```

## Ví dụ thay thế

Đi thẳng:

```cpp
// Code cũ
TienTx();
Prepare_Data();

// Code mới
robot.forward(180);
robot.update();
```

Dừng mềm:

```cpp
// Code cũ
StopUart();

// Code mới
robot.softStop();
```

Dừng ngay:

```cpp
robot.stop();
```

Điều khiển một cơ cấu độc lập:

```cpp
pid.setMotorMotionProfile(7, 300.0f, 700.0f);
pid.setTarget(7, 180);
pid.update();
```

## Khác biệt quan trọng

Ramp mới dựa trên **thời gian thực** thay vì số lần hàm được gọi. Vì vậy khi chương trình thêm xử lý cảm biến, PS2, LCD hay giao tiếp khác làm thay đổi tốc độ `loop()`, thời gian tăng tốc/giảm tốc vẫn gần như giữ nguyên.

Ngoài ra, thư viện mới có:

- zero-crossing khi đảo chiều;
- TX scheduler;
- refresh command định kỳ;
- watchdog;
- soft stop;
- immediate stop;
- active reverse braking;
- lớp Mecanum 4 bánh tách riêng khỏi tầng UART.
