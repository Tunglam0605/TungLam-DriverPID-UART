/*
  TungLam DriverPID UART
  Copyright (c) 2026 Nguyen Khac Tung Lam
  Tung Lam Automation

  UART protocol:
    Byte 0 = [DIR bit7][ADDRESS bit6..0]
    Byte 1 = SPEED 0..255
    Byte 2 = 0xFF

  Design:
  - signed command API: -255..255
  - non-blocking, time-based acceleration/deceleration
  - fixed memory, no dynamic allocation
  - reusable for one motor, mechanisms, or a 4WD Mecanum base
*/

#ifndef TUNGLAM_DRIVERPID_UART_H
#define TUNGLAM_DRIVERPID_UART_H

#include <Arduino.h>

#ifndef TUNGLAM_PID_MAX_MOTORS
#define TUNGLAM_PID_MAX_MOTORS 16
#endif

enum class TungLamPIDTimeoutAction : uint8_t {
  SoftStop = 0,
  ImmediateStop = 1
};

class TungLamPIDBus {
 public:
  TungLamPIDBus();

  // Attach an already configured Arduino Stream.
  void attach(Stream& serial);

  // Convenience overload for HardwareSerial / SoftwareSerial-like objects
  // that provide begin(baud).
  template <typename TSerial>
  void begin(TSerial& serial, uint32_t baud) {
    serial.begin(baud);
    attach(serial);
  }

  bool attached() const;

  // Register an address (1..127). Registration is also automatic when a
  // command/profile function is called for a new address.
  bool registerMotor(uint8_t id);
  bool isRegistered(uint8_t id) const;

  // Immediate command. Bypasses the ramp and transmits now.
  bool setMotor(uint8_t id, int16_t signedSpeed);

  // Ramped command. Call update() continuously.
  bool setTarget(uint8_t id, int16_t signedSpeed);

  // Default profile for new motors and motors that do not have a custom one.
  // Unit: command units per second, where full scale is 255.
  bool setDefaultMotionProfile(float acceleration, float deceleration);

  // Per-motor profile override.
  bool setMotorMotionProfile(uint8_t id,
                             float acceleration,
                             float deceleration);

  // Remove a per-motor override and follow the current default profile.
  bool clearMotorMotionProfile(uint8_t id);

  // Ramped stop.
  bool softStop(uint8_t id);

  // Immediate speed=0 command.
  bool stop(uint8_t id);

  // Immediate speed=0 for every registered motor.
  void emergencyStop();

  // Active reverse braking: send opposite torque for pulseMs then send zero.
  // This is NOT a native documented brake opcode. Tune carefully on hardware.
  bool activeBrake(uint8_t id,
                   uint8_t strength = 60,
                   uint16_t pulseMs = 30);

  // Alias with the intent made explicit.
  bool hardBrake(uint8_t id,
                 uint8_t strength = 60,
                 uint16_t pulseMs = 30);

  // Apply active reverse braking to all currently moving registered motors.
  void hardBrakeAll(uint8_t strength = 60, uint16_t pulseMs = 30);

  // UART scheduling.
  void setMinTxIntervalMs(uint16_t intervalMs);
  void setRefreshPeriodMs(uint16_t periodMs);
  void setTransmitThreshold(uint8_t threshold);

  // Application command watchdog. Set timeoutMs=0 to disable.
  void setCommandTimeout(uint32_t timeoutMs,
                         TungLamPIDTimeoutAction action =
                             TungLamPIDTimeoutAction::ImmediateStop);
  bool commandTimedOut() const;

  // Non-blocking service routine. Call continuously from loop().
  void update();

  // Introspection.
  int16_t currentCommand(uint8_t id) const;
  int16_t targetCommand(uint8_t id) const;
  uint8_t registeredCount() const;

 private:
  struct MotorState {
    bool active;
    uint8_t id;
    float current;
    int16_t target;

    float acceleration;
    float deceleration;
    bool customProfile;

    int16_t lastSent;
    bool sentOnce;
    uint32_t lastTxMs;

    bool directionBit;

    bool braking;
    int16_t brakeCommand;
    uint32_t brakeUntilMs;
  };

  Stream* serial_;
  MotorState motors_[TUNGLAM_PID_MAX_MOTORS];

  float defaultAcceleration_;
  float defaultDeceleration_;

  uint16_t minTxIntervalMs_;
  uint16_t refreshPeriodMs_;
  uint8_t transmitThreshold_;

  uint32_t lastUpdateUs_;
  bool updateClockPrimed_;

  uint32_t commandTimeoutMs_;
  TungLamPIDTimeoutAction timeoutAction_;
  uint32_t lastApplicationCommandMs_;
  bool watchdogPrimed_;
  bool timeoutTriggered_;

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

class TungLamPID4WD {
 public:
  explicit TungLamPID4WD(TungLamPIDBus& bus);

  // Wheel convention:
  // M1 = Front Left, M2 = Rear Left, M3 = Rear Right, M4 = Front Right.
  bool setMotorIDs(uint8_t m1FrontLeft,
                   uint8_t m2RearLeft,
                   uint8_t m3RearRight,
                   uint8_t m4FrontRight);

  // Reverse one wheel's logical sign without changing kinematics.
  bool setWheelInverted(uint8_t wheel, bool inverted);

  // Apply the same motion profile to all four base motors.
  bool setMotionProfile(float acceleration, float deceleration);

  // Direct wheel targets, -255..255. These use the configured ramp.
  bool setWheels(int16_t m1, int16_t m2, int16_t m3, int16_t m4);

  // Mecanum-X normalized body command:
  // +vx forward, +vy left, +wz CCW/left.
  bool drive(int16_t vx, int16_t vy, int16_t wz);

  bool forward(uint8_t speed);
  bool backward(uint8_t speed);
  bool strafeLeft(uint8_t speed);
  bool strafeRight(uint8_t speed);
  bool rotateLeft(uint8_t speed);
  bool rotateRight(uint8_t speed);

  // Ramped base stop.
  void softStop();

  // Immediate speed=0 on the four base motors.
  void stop();

  // Active reverse pulse on the four base motors.
  void hardBrake(uint8_t strength = 60, uint16_t pulseMs = 30);

  // Service the shared bus.
  void update();

 private:
  TungLamPIDBus& bus_;
  uint8_t ids_[4];
  bool inverted_[4];
  bool configured_;

  int16_t applyInversion(uint8_t wheelIndex, int16_t command) const;
  static int16_t clampBody(int16_t value);
  static void normalize4(int32_t& m1, int32_t& m2, int32_t& m3, int32_t& m4);
};

#endif  // TUNGLAM_DRIVERPID_UART_H
