/*==============================================================================
  TUNGLAM DRIVER PID UART LIBRARY
  ==============================================================================
  TÁC GIẢ
  ------------------------------------------------------------------------------
  Họ và tên  : Nguyễn Khắc Tùng Lâm
  Thương hiệu: Tung Lâm Automation

  MỤC TIÊU
  ------------------------------------------------------------------------------
  - Điều khiển Driver Smart PID qua UART bằng giao thức 3 byte đã được sử dụng
    thực tế trong project Robot0x01.
  - Tách phần giao tiếp UART khỏi phần điều khiển đế Mecanum để thư viện có thể
    tái sử dụng cho cả động cơ cơ cấu, băng tải, cơ cấu nâng, tay máy...
  - Dùng lệnh tốc độ có dấu từ -255..255 để code ứng dụng không phải tự quản lý
    riêng SPEED và DIR.
  - Hỗ trợ tăng tốc/giảm tốc theo thời gian thực, không phụ thuộc tốc độ loop().
  - Hạn chế gửi UART dư thừa bằng ngưỡng thay đổi, chu kỳ refresh và khoảng TX
    tối thiểu.
  - Có watchdog lệnh điều khiển và các chế độ dừng phục vụ an toàn vận hành.
  - Có lớp TungLamPID4WD để điều khiển đế robot 4 bánh Mecanum theo đúng quy ước
    M1/M2/M3/M4 đang dùng trong hệ sinh thái thư viện Tung Lâm Automation.

  KHUNG TRUYỀN UART
  ------------------------------------------------------------------------------
    Byte 0: [DIR:1][ADDRESS:7]
    Byte 1: SPEED 0..255
    Byte 2: 0xFF

    DIR = 1: chiều dương logic của thư viện.
    DIR = 0: chiều âm logic của thư viện.

  QUY ƯỚC LỆNH TỐC ĐỘ
  ------------------------------------------------------------------------------
    +255 ... +1 : chạy theo chiều dương.
       0        : dừng.
     -1 ... -255: chạy theo chiều âm.

  QUY ƯỚC ĐẾ 4 BÁNH
  ------------------------------------------------------------------------------
                    ĐẦU XE / FRONT
                         +X
                          ^

             M1                        M4
        TRƯỚC-TRÁI                TRƯỚC-PHẢI

             M2                        M3
          SAU-TRÁI                 SAU-PHẢI

    +vx = tiến
    -vx = lùi
    +vy = đi ngang trái
    -vy = đi ngang phải
    +wz = quay trái / CCW
    -wz = quay phải / CW

  LƯU Ý VỀ PHANH
  ------------------------------------------------------------------------------
  hardBrake()/activeBrake() hiện được triển khai theo nguyên lý hãm ngược chủ
  động: phát một xung điều khiển ngược chiều trong thời gian ngắn rồi đưa tốc
  độ về 0. Đây KHÔNG phải là một opcode "BRAKE" riêng của giao thức UART.

  Hãm ngược có thể tạo dòng điện lớn và xung lực cơ khí. Khi thử trên phần cứng,
  cần bắt đầu với strength nhỏ và pulseMs ngắn rồi tăng dần sau khi đo thực tế.
==============================================================================*/

#ifndef TUNGLAM_DRIVERPID_UART_H
#define TUNGLAM_DRIVERPID_UART_H

#include <Arduino.h>

/**
 * @brief Số động cơ tối đa mà một đối tượng TungLamPIDBus quản lý.
 *
 * Có thể định nghĩa TUNGLAM_PID_MAX_MOTORS trước khi include thư viện nếu cần
 * thay đổi giới hạn mặc định. Thư viện dùng bộ nhớ tĩnh, không cấp phát động.
 */
#ifndef TUNGLAM_PID_MAX_MOTORS
#define TUNGLAM_PID_MAX_MOTORS 16
#endif

/**
 * @brief Hành động khi watchdog phát hiện ứng dụng ngừng gửi lệnh mới.
 */
enum class TungLamPIDTimeoutAction : uint8_t {
  SoftStop = 0,       ///< Giảm tốc theo profile rồi về 0.
  ImmediateStop = 1  ///< Đưa tốc độ về 0 và gửi lệnh dừng ngay lập tức.
};

/**
 * @brief Quản lý giao tiếp UART và trạng thái điều khiển nhiều Driver PID.
 *
 * TungLamPIDBus là lớp tầng thấp. Lớp này không biết robot đang dùng Mecanum,
 * cơ cấu nâng hay băng tải; nó chỉ quản lý motor theo địa chỉ, profile chuyển
 * động và lịch truyền UART.
 *
 * Mỗi motor được nhận diện bằng ID từ 1..127.
 */
class TungLamPIDBus {
 public:
  /** @brief Tạo bus UART ở trạng thái chưa gắn cổng Serial. */
  TungLamPIDBus();

  /**
   * @brief Gắn một Stream đã được cấu hình baud từ bên ngoài.
   * @param serial Đối tượng Stream dùng để truyền dữ liệu.
   *
   * Cách này phù hợp khi ứng dụng tự gọi begin() cho HardwareSerial,
   * SoftwareSerial hoặc một lớp Stream tương thích khác.
   */
  void attach(Stream& serial);

  /**
   * @brief Khởi tạo nhanh Serial rồi gắn vào bus.
   * @tparam TSerial Kiểu Serial có hàm begin(baud).
   * @param serial Đối tượng Serial cần sử dụng.
   * @param baud Tốc độ baud, ví dụ 115200.
   */
  template <typename TSerial>
  void begin(TSerial& serial, uint32_t baud) {
    serial.begin(baud);
    attach(serial);
  }

  /** @return true khi bus đã được gắn với một Stream hợp lệ. */
  bool attached() const;

  /**
   * @brief Đăng ký một motor vào bảng quản lý.
   * @param id Địa chỉ motor từ 1..127.
   * @return true nếu ID hợp lệ và còn chỗ trong bảng.
   *
   * Thông thường không bắt buộc gọi hàm này trước vì setMotor(), setTarget()
   * và các hàm cấu hình profile sẽ tự đăng ký motor khi cần.
   */
  bool registerMotor(uint8_t id);

  /** @brief Kiểm tra một ID đã được đăng ký trong bus hay chưa. */
  bool isRegistered(uint8_t id) const;

  /**
   * @brief Gửi tốc độ ngay lập tức, bỏ qua ramp tăng/giảm tốc.
   * @param id Địa chỉ motor 1..127.
   * @param signedSpeed Tốc độ logic -255..255.
   * @return true nếu motor hợp lệ và frame được xử lý thành công.
   *
   * Dùng hàm này cho lệnh trực tiếp hoặc khi cần phản ứng ngay. Nếu cần chuyển
   * động êm, nên dùng setTarget() rồi gọi update() liên tục.
   */
  bool setMotor(uint8_t id, int16_t signedSpeed);

  /**
   * @brief Đặt tốc độ mục tiêu để motor tiến tới theo profile.
   * @param id Địa chỉ motor 1..127.
   * @param signedSpeed Tốc độ mục tiêu -255..255.
   * @return true nếu motor hợp lệ.
   *
   * Hàm không block. Cần gọi update() liên tục trong loop() để profile chạy.
   */
  bool setTarget(uint8_t id, int16_t signedSpeed);

  /**
   * @brief Cấu hình profile tăng tốc/giảm tốc mặc định.
   * @param acceleration Độ dốc tăng tốc, đơn vị command/giây.
   * @param deceleration Độ dốc giảm tốc, đơn vị command/giây.
   * @return false nếu một trong hai giá trị <= 0.
   *
   * Với full-scale 255, acceleration = 255 nghĩa là lý tưởng cần khoảng 1 giây
   * để tăng từ 0 lên 255.
   */
  bool setDefaultMotionProfile(float acceleration, float deceleration);

  /**
   * @brief Gán profile riêng cho một motor.
   * @param id Địa chỉ motor.
   * @param acceleration Độ dốc tăng tốc, command/giây.
   * @param deceleration Độ dốc giảm tốc, command/giây.
   */
  bool setMotorMotionProfile(uint8_t id,
                             float acceleration,
                             float deceleration);

  /**
   * @brief Xóa profile riêng và cho motor quay lại dùng profile mặc định.
   */
  bool clearMotorMotionProfile(uint8_t id);

  /**
   * @brief Dừng mềm một motor.
   *
   * Target được đặt về 0 nhưng current command sẽ giảm dần theo deceleration.
   */
  bool softStop(uint8_t id);

  /**
   * @brief Dừng ngay một motor.
   *
   * Bỏ qua ramp, đặt current/target về 0 và ép gửi frame tốc độ 0 ngay.
   */
  bool stop(uint8_t id);

  /**
   * @brief Dừng khẩn cấp toàn bộ motor đã đăng ký.
   *
   * Hàm ép gửi tốc độ 0 ngay cho tất cả motor và xóa trạng thái ramp/phanh.
   * Đây là dừng bằng phần mềm qua UART, không thay thế E-Stop phần cứng trong
   * các hệ thống có yêu cầu an toàn máy.
   */
  void emergencyStop();

  /**
   * @brief Hãm ngược chủ động một motor rồi tự động đưa tốc độ về 0.
   * @param id Địa chỉ motor.
   * @param strength Biên độ lệnh hãm 1..255.
   * @param pulseMs Thời gian giữ xung hãm [ms].
   *
   * Hàm không dùng delay(). update() sẽ theo dõi thời gian và gửi lệnh 0 khi
   * hết pulseMs.
   *
   * @warning Hãm ngược có thể tạo dòng điện lớn. Cần tune trên phần cứng thật.
   */
  bool activeBrake(uint8_t id,
                   uint8_t strength = 60,
                   uint16_t pulseMs = 30);

  /**
   * @brief Alias dễ đọc cho activeBrake(), nhấn mạnh mục đích phanh cứng.
   */
  bool hardBrake(uint8_t id,
                 uint8_t strength = 60,
                 uint16_t pulseMs = 30);

  /**
   * @brief Hãm ngược toàn bộ motor đang chạy rồi đưa về 0.
   */
  void hardBrakeAll(uint8_t strength = 60, uint16_t pulseMs = 30);

  /**
   * @brief Đặt khoảng thời gian tối thiểu giữa hai lần TX thay đổi [ms].
   *
   * Mục tiêu là tránh flood UART khi loop() chạy rất nhanh.
   */
  void setMinTxIntervalMs(uint16_t intervalMs);

  /**
   * @brief Đặt chu kỳ gửi lại lệnh hiện tại [ms].
   *
   * Dù tốc độ không đổi, frame sẽ được refresh định kỳ để phía driver vẫn nhận
   * được lệnh mới. Đặt 0 để tắt refresh định kỳ.
   */
  void setRefreshPeriodMs(uint16_t periodMs);

  /**
   * @brief Đặt mức thay đổi tối thiểu để coi command là đáng gửi ngay.
   * @param threshold Ngưỡng theo đơn vị command, tối thiểu 1.
   */
  void setTransmitThreshold(uint8_t threshold);

  /**
   * @brief Cấu hình watchdog lệnh điều khiển.
   * @param timeoutMs Thời gian tối đa không nhận lệnh mới [ms]; 0 = tắt.
   * @param action Cách dừng khi timeout.
   *
   * Mỗi lần gọi setMotor(), setTarget(), softStop(), stop() hoặc hàm phanh,
   * watchdog được xem là đã nhận lệnh hợp lệ mới từ ứng dụng.
   */
  void setCommandTimeout(
      uint32_t timeoutMs,
      TungLamPIDTimeoutAction action =
          TungLamPIDTimeoutAction::ImmediateStop);

  /** @return true nếu watchdog đã kích hoạt ở chu kỳ hiện tại. */
  bool commandTimedOut() const;

  /**
   * @brief Hàm dịch vụ không block của thư viện.
   *
   * Cần gọi liên tục trong loop(). Hàm thực hiện:
   * - tính dt bằng micros();
   * - cập nhật ramp tăng/giảm tốc;
   * - xử lý zero-crossing khi đảo chiều;
   * - kết thúc xung hardBrake();
   * - kiểm tra watchdog;
   * - quyết định thời điểm truyền UART.
   */
  void update();

  /** @brief Đọc command thực tế hiện đang áp dụng cho motor. */
  int16_t currentCommand(uint8_t id) const;

  /** @brief Đọc command mục tiêu mà motor đang tiến tới. */
  int16_t targetCommand(uint8_t id) const;

  /** @brief Trả về số motor hiện đã đăng ký trong bus. */
  uint8_t registeredCount() const;

 private:
  /**
   * @brief Trạng thái nội bộ của một motor.
   *
   * Toàn bộ dữ liệu được giữ trong mảng tĩnh để phù hợp hệ nhúng và tránh
   * fragmentation heap khi chạy lâu.
   */
  struct MotorState {
    bool active;              ///< Slot này đã được sử dụng hay chưa.
    uint8_t id;               ///< Địa chỉ UART 1..127.
    float current;            ///< Command hiện tại dạng float để ramp mượt.
    int16_t target;           ///< Command mục tiêu -255..255.

    float acceleration;       ///< Độ dốc tăng tốc riêng của motor.
    float deceleration;       ///< Độ dốc giảm tốc riêng của motor.
    bool customProfile;       ///< true nếu motor đang dùng profile riêng.

    int16_t lastSent;         ///< Command gần nhất đã truyền thành frame UART.
    bool sentOnce;            ///< Đã từng gửi frame cho motor này hay chưa.
    uint32_t lastTxMs;        ///< Mốc thời gian TX gần nhất.

    bool directionBit;        ///< DIR gần nhất; giữ nguyên khi speed = 0.

    bool braking;             ///< Motor đang trong pha hãm ngược.
    int16_t brakeCommand;     ///< Lệnh ngược chiều đang dùng để hãm.
    uint32_t brakeUntilMs;    ///< Mốc thời gian kết thúc xung hãm.
  };

  Stream* serial_;  ///< Stream UART đang được dùng để truyền frame.
  MotorState motors_[TUNGLAM_PID_MAX_MOTORS];

  float defaultAcceleration_;  ///< Tăng tốc mặc định [command/s].
  float defaultDeceleration_;  ///< Giảm tốc mặc định [command/s].

  uint16_t minTxIntervalMs_;   ///< Khoảng TX tối thiểu khi command thay đổi.
  uint16_t refreshPeriodMs_;   ///< Chu kỳ gửi lại command dù không đổi.
  uint8_t transmitThreshold_;  ///< Ngưỡng thay đổi command để phát TX mới.

  uint32_t lastUpdateUs_;      ///< Mốc micros() của lần update trước.
  bool updateClockPrimed_;     ///< Đã khởi tạo mốc thời gian update hay chưa.

  uint32_t commandTimeoutMs_;              ///< Thời gian watchdog [ms].
  TungLamPIDTimeoutAction timeoutAction_;  ///< Hành động khi watchdog timeout.
  uint32_t lastApplicationCommandMs_;      ///< Lần cuối ứng dụng gửi lệnh mới.
  bool watchdogPrimed_;                    ///< Watchdog đã có mốc tham chiếu.
  bool timeoutTriggered_;                  ///< Watchdog hiện đã kích hoạt.

  MotorState* findMotor(uint8_t id);
  const MotorState* findMotor(uint8_t id) const;
  MotorState* ensureMotor(uint8_t id);

  static bool validId(uint8_t id);
  static int16_t clampCommand(int16_t value);
  static int16_t quantize(float value);
  static float approach(float current, float target, float maxDelta);
  static bool signsOpposite(float a, float b);
  static bool deadlineReached(uint32_t now, uint32_t deadline);

  float stepMotor(MotorState& motor, float dtSeconds);
  bool transmit(MotorState& motor, int16_t command, bool force);
  bool writeFrame(MotorState& motor, int16_t command);
  void noteApplicationCommand();
  void stopAllInternal(bool immediate);
};

/**
 * @brief Lớp điều khiển đế robot 4 bánh Mecanum dùng TungLamPIDBus.
 *
 * Lớp này chỉ làm nhiệm vụ kinematics/mixer và ánh xạ 4 bánh. Giao thức UART,
 * motion profile, watchdog và TX scheduler vẫn do TungLamPIDBus xử lý.
 */
class TungLamPID4WD {
 public:
  explicit TungLamPID4WD(TungLamPIDBus& bus);

  /**
   * @brief Gán ID cho 4 motor theo đúng thứ tự chuẩn Tung Lâm Automation.
   * @param m1FrontLeft M1 - bánh trước trái.
   * @param m2RearLeft M2 - bánh sau trái.
   * @param m3RearRight M3 - bánh sau phải.
   * @param m4FrontRight M4 - bánh trước phải.
   *
   * Bốn ID phải nằm trong 1..127 và không được trùng nhau.
   */
  bool setMotorIDs(uint8_t m1FrontLeft,
                   uint8_t m2RearLeft,
                   uint8_t m3RearRight,
                   uint8_t m4FrontRight);

  /**
   * @brief Đảo dấu logic của một bánh mà không thay đổi công thức động học.
   * @param wheel Số bánh 1..4.
   * @param inverted true nếu cần đảo chiều lắp motor.
   */
  bool setWheelInverted(uint8_t wheel, bool inverted);

  /**
   * @brief Gán cùng một profile tăng/giảm tốc cho cả 4 motor của đế.
   */
  bool setMotionProfile(float acceleration, float deceleration);

  /**
   * @brief Đặt trực tiếp target riêng cho 4 bánh.
   *
   * Giá trị -255..255 và vẫn đi qua motion profile của TungLamPIDBus.
   */
  bool setWheels(int16_t m1, int16_t m2, int16_t m3, int16_t m4);

  /**
   * @brief Điều khiển Mecanum-X bằng vector chuyển động thân xe.
   * @param vx Thành phần tiến/lùi, -255..255.
   * @param vy Thành phần ngang trái/phải, -255..255.
   * @param wz Thành phần quay, -255..255.
   *
   * Khi tổng vector vượt 255, bốn bánh được scale đồng đều để giữ đúng tỉ lệ
   * vx/vy/wz thay vì cắt riêng từng bánh gây méo hướng chuyển động.
   */
  bool drive(int16_t vx, int16_t vy, int16_t wz);

  /** @brief Chạy thẳng tiến. */
  bool forward(uint8_t speed);

  /** @brief Chạy thẳng lùi. */
  bool backward(uint8_t speed);

  /** @brief Đi ngang trái. */
  bool strafeLeft(uint8_t speed);

  /** @brief Đi ngang phải. */
  bool strafeRight(uint8_t speed);

  /** @brief Quay trái / ngược chiều kim đồng hồ. */
  bool rotateLeft(uint8_t speed);

  /** @brief Quay phải / cùng chiều kim đồng hồ. */
  bool rotateRight(uint8_t speed);

  /** @brief Giảm tốc mềm cả 4 bánh về 0 theo deceleration đã cấu hình. */
  void softStop();

  /** @brief Dừng ngay cả 4 bánh bằng lệnh speed = 0. */
  void stop();

  /**
   * @brief Hãm ngược chủ động cả 4 bánh.
   * @warning Cần tune strength và pulseMs trên robot thật.
   */
  void hardBrake(uint8_t strength = 60, uint16_t pulseMs = 30);

  /** @brief Gọi hàm dịch vụ update() của bus dùng chung. */
  void update();

 private:
  TungLamPIDBus& bus_;  ///< Bus UART dùng chung của 4 motor.
  uint8_t ids_[4];      ///< ID theo thứ tự M1, M2, M3, M4.
  bool inverted_[4];    ///< Cờ đảo chiều logic riêng từng bánh.
  bool configured_;     ///< true sau khi setMotorIDs() hợp lệ.

  int16_t applyInversion(uint8_t wheelIndex, int16_t command) const;
  static int16_t clampBody(int16_t value);
  static void normalize4(int32_t& m1,
                         int32_t& m2,
                         int32_t& m3,
                         int32_t& m4);
};

#endif  // TUNGLAM_DRIVERPID_UART_H
