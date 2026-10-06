# Lịch sử phiên bản

## 0.1.1 - 2026-10-06

Bản chuẩn hóa tài liệu tiếng Việt.

- Viết lại toàn bộ comment API/Doxygen bằng tiếng Việt chi tiết.
- Bổ sung giải thích kiến trúc, giao thức UART, motion profile và watchdog.
- Viết lại README hoàn toàn bằng tiếng Việt.
- Chuyển toàn bộ comment trong các ví dụ sang tiếng Việt.
- Viết lại tài liệu giao thức và hướng dẫn chuyển đổi từ Robot0x01.
- Chuyển mô tả thư viện và GitHub About sang tiếng Việt.
- Không thay đổi logic điều khiển so với v0.1.0.

## 0.1.0 - 2026-10-06

Bản phát hành công khai đầu tiên.

- Tạo lớp bus UART Driver PID có thể tái sử dụng.
- Hỗ trợ frame UART 3 byte tương thích với Robot0x01.
- API tốc độ có dấu -255..255.
- Hỗ trợ HardwareSerial và Stream.
- Tăng tốc/giảm tốc theo thời gian cho từng motor.
- Đưa motor về 0 trước khi đảo chiều.
- Dừng mềm và dừng ngay.
- Watchdog lệnh điều khiển.
- Giảm TX dư thừa và refresh command định kỳ.
- Hãm ngược chủ động không block.
- Lớp Mecanum 4WD theo thứ tự M1=FL, M2=RL, M3=RR, M4=FR.
