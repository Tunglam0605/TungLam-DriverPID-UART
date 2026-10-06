# Giao thức UART

## Cấu trúc frame

Mỗi lệnh điều khiển một motor gồm **3 byte**:

| Byte | Ý nghĩa |
|---|---|
| 0 | bit7 = chiều quay, bit6..0 = địa chỉ motor |
| 1 | độ lớn tốc độ 0..255 |
| 2 | byte kết thúc cố định 0xFF |

Thư viện sử dụng địa chỉ motor từ **1..127**.

## Ánh xạ lệnh tốc độ có dấu

API công khai dùng một giá trị tốc độ có dấu thay vì bắt người dùng tự quản lý riêng SPEED và DIR:

| Giá trị API | Bit chiều | Byte tốc độ |
|---:|---:|---:|
| +180 | 1 | 180 |
| -180 | 0 | 180 |
| 0 | giữ chiều gần nhất | 0 |

Khi command bằng 0, thư viện giữ lại bit chiều gần nhất và chỉ đưa SPEED về 0. Cách này phù hợp với logic cũ của Robot0x01, nơi chiều và tốc độ được lưu riêng.

## Ví dụ đóng gói frame

Motor ID 3 chạy chiều dương với tốc độ 200:

```text
Byte0 = 0b10000011 = 0x83
Byte1 = 0xC8
Byte2 = 0xFF
```

Motor ID 3 chạy chiều âm với tốc độ 200:

```text
Byte0 = 0b00000011 = 0x03
Byte1 = 0xC8
Byte2 = 0xFF
```

## Chính sách truyền TX

`setTarget()` chỉ cập nhật tốc độ mục tiêu. `update()` mới thực hiện motion profile và quyết định có cần phát frame hay không.

Ba tham số chính dùng để giảm tải UART:

- `setMinTxIntervalMs()`: khoảng TX tối thiểu khi command thay đổi.
- `setTransmitThreshold()`: độ thay đổi tối thiểu để phát frame mới.
- `setRefreshPeriodMs()`: chu kỳ gửi lại command dù giá trị không đổi.

Nhờ đó ứng dụng có thể gọi `update()` liên tục với tần số cao mà không làm UART bị flood vô ích.

## Đảo chiều an toàn

Khi target đổi dấu, ví dụ từ +200 sang -200, thư viện không nhảy trực tiếp qua 0:

```text
+200
  |
  v
giảm dần
  |
  v
  0
  |
  v
đổi chiều
  |
  v
-200
```

Mục tiêu là giảm sốc cơ khí và tránh tạo một bước nhảy mô-men quá lớn.

## Phanh ngược chủ động

`activeBrake()` và `hardBrake()` **không phải opcode UART riêng**.

Thư viện thực hiện:

1. xác định chiều motor đang chạy;
2. phát một command ngược chiều với độ lớn `strength`;
3. giữ trong `pulseMs`;
4. sau đó phát command 0.

Hàm hoàn toàn không block; `update()` chịu trách nhiệm kết thúc pha hãm.

> Cần tune trên động cơ, nguồn và driver thật vì hãm ngược có thể tạo dòng điện lớn.
