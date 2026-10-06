#include "TungLam_DriverPID_UART.h"

#include <math.h>

namespace {
constexpr float kMaxUpdateDtSeconds = 0.050f;
}

TungLamPIDBus::TungLamPIDBus()
    : serial_(nullptr),
      defaultAcceleration_(400.0f),
      defaultDeceleration_(800.0f),
      minTxIntervalMs_(5),
      refreshPeriodMs_(100),
      transmitThreshold_(1),
      lastUpdateUs_(0),
      updateClockPrimed_(false),
      commandTimeoutMs_(0),
      timeoutAction_(TungLamPIDTimeoutAction::ImmediateStop),
      lastApplicationCommandMs_(0),
      watchdogPrimed_(false),
      timeoutTriggered_(false) {
  for (uint8_t i = 0; i < TUNGLAM_PID_MAX_MOTORS; ++i) {
    motors_[i].active = false;
    motors_[i].id = 0;
    motors_[i].current = 0.0f;
    motors_[i].target = 0;
    motors_[i].acceleration = defaultAcceleration_;
    motors_[i].deceleration = defaultDeceleration_;
    motors_[i].customProfile = false;
    motors_[i].lastSent = 0;
    motors_[i].sentOnce = false;
    motors_[i].lastTxMs = 0;
    motors_[i].directionBit = true;
    motors_[i].braking = false;
    motors_[i].brakeCommand = 0;
    motors_[i].brakeUntilMs = 0;
  }
}

void TungLamPIDBus::attach(Stream& serial) {
  serial_ = &serial;
  lastUpdateUs_ = micros();
  updateClockPrimed_ = true;
}

bool TungLamPIDBus::attached() const {
  return serial_ != nullptr;
}

bool TungLamPIDBus::validId(uint8_t id) {
  return id >= 1 && id <= 127;
}

int16_t TungLamPIDBus::clampCommand(int16_t value) {
  if (value > 255) return 255;
  if (value < -255) return -255;
  return value;
}

int16_t TungLamPIDBus::quantize(float value) {
  if (value > 255.0f) value = 255.0f;
  if (value < -255.0f) value = -255.0f;
  return static_cast<int16_t>(lroundf(value));
}

float TungLamPIDBus::approach(float current,
                              float target,
                              float maxDelta) {
  if (maxDelta <= 0.0f) return target;
  if (current < target) {
    const float next = current + maxDelta;
    return next > target ? target : next;
  }
  if (current > target) {
    const float next = current - maxDelta;
    return next < target ? target : next;
  }
  return target;
}

bool TungLamPIDBus::signsOpposite(float a, float b) {
  return (a > 0.0f && b < 0.0f) || (a < 0.0f && b > 0.0f);
}

bool TungLamPIDBus::deadlineReached(uint32_t now, uint32_t deadline) {
  return static_cast<int32_t>(now - deadline) >= 0;
}

TungLamPIDBus::MotorState* TungLamPIDBus::findMotor(uint8_t id) {
  for (uint8_t i = 0; i < TUNGLAM_PID_MAX_MOTORS; ++i) {
    if (motors_[i].active && motors_[i].id == id) return &motors_[i];
  }
  return nullptr;
}

const TungLamPIDBus::MotorState* TungLamPIDBus::findMotor(uint8_t id) const {
  for (uint8_t i = 0; i < TUNGLAM_PID_MAX_MOTORS; ++i) {
    if (motors_[i].active && motors_[i].id == id) return &motors_[i];
  }
  return nullptr;
}

TungLamPIDBus::MotorState* TungLamPIDBus::ensureMotor(uint8_t id) {
  if (!validId(id)) return nullptr;

  MotorState* existing = findMotor(id);
  if (existing != nullptr) return existing;

  for (uint8_t i = 0; i < TUNGLAM_PID_MAX_MOTORS; ++i) {
    if (!motors_[i].active) {
      MotorState& motor = motors_[i];
      motor.active = true;
      motor.id = id;
      motor.current = 0.0f;
      motor.target = 0;
      motor.acceleration = defaultAcceleration_;
      motor.deceleration = defaultDeceleration_;
      motor.customProfile = false;
      motor.lastSent = 0;
      motor.sentOnce = false;
      motor.lastTxMs = 0;
      motor.directionBit = true;
      motor.braking = false;
      motor.brakeCommand = 0;
      motor.brakeUntilMs = 0;
      return &motor;
    }
  }

  return nullptr;
}

bool TungLamPIDBus::registerMotor(uint8_t id) {
  return ensureMotor(id) != nullptr;
}

bool TungLamPIDBus::isRegistered(uint8_t id) const {
  return findMotor(id) != nullptr;
}

void TungLamPIDBus::noteApplicationCommand() {
  lastApplicationCommandMs_ = millis();
  watchdogPrimed_ = true;
  timeoutTriggered_ = false;
}

bool TungLamPIDBus::setMotor(uint8_t id, int16_t signedSpeed) {
  MotorState* motor = ensureMotor(id);
  if (motor == nullptr) return false;

  const int16_t command = clampCommand(signedSpeed);
  motor->braking = false;
  motor->target = command;
  motor->current = static_cast<float>(command);
  noteApplicationCommand();
  return transmit(*motor, command, true);
}

bool TungLamPIDBus::setTarget(uint8_t id, int16_t signedSpeed) {
  MotorState* motor = ensureMotor(id);
  if (motor == nullptr) return false;

  motor->braking = false;
  motor->target = clampCommand(signedSpeed);
  noteApplicationCommand();
  return true;
}

bool TungLamPIDBus::setDefaultMotionProfile(float acceleration,
                                             float deceleration) {
  if (acceleration <= 0.0f || deceleration <= 0.0f) return false;

  defaultAcceleration_ = acceleration;
  defaultDeceleration_ = deceleration;

  for (uint8_t i = 0; i < TUNGLAM_PID_MAX_MOTORS; ++i) {
    MotorState& motor = motors_[i];
    if (motor.active && !motor.customProfile) {
      motor.acceleration = acceleration;
      motor.deceleration = deceleration;
    }
  }

  return true;
}

bool TungLamPIDBus::setMotorMotionProfile(uint8_t id,
                                          float acceleration,
                                          float deceleration) {
  if (acceleration <= 0.0f || deceleration <= 0.0f) return false;

  MotorState* motor = ensureMotor(id);
  if (motor == nullptr) return false;

  motor->acceleration = acceleration;
  motor->deceleration = deceleration;
  motor->customProfile = true;
  return true;
}

bool TungLamPIDBus::clearMotorMotionProfile(uint8_t id) {
  MotorState* motor = ensureMotor(id);
  if (motor == nullptr) return false;

  motor->acceleration = defaultAcceleration_;
  motor->deceleration = defaultDeceleration_;
  motor->customProfile = false;
  return true;
}

bool TungLamPIDBus::softStop(uint8_t id) {
  MotorState* motor = ensureMotor(id);
  if (motor == nullptr) return false;

  motor->braking = false;
  motor->target = 0;
  noteApplicationCommand();
  return true;
}

bool TungLamPIDBus::stop(uint8_t id) {
  MotorState* motor = ensureMotor(id);
  if (motor == nullptr) return false;

  motor->braking = false;
  motor->target = 0;
  motor->current = 0.0f;
  noteApplicationCommand();
  return transmit(*motor, 0, true);
}

void TungLamPIDBus::stopAllInternal(bool immediate) {
  for (uint8_t i = 0; i < TUNGLAM_PID_MAX_MOTORS; ++i) {
    MotorState& motor = motors_[i];
    if (!motor.active) continue;

    motor.braking = false;
    motor.target = 0;

    if (immediate) {
      motor.current = 0.0f;
      transmit(motor, 0, true);
    }
  }
}

void TungLamPIDBus::emergencyStop() {
  noteApplicationCommand();
  stopAllInternal(true);
}

bool TungLamPIDBus::activeBrake(uint8_t id,
                                uint8_t strength,
                                uint16_t pulseMs) {
  MotorState* motor = ensureMotor(id);
  if (motor == nullptr || strength == 0 || pulseMs == 0) return false;

  int16_t reference = quantize(motor->current);
  if (reference == 0) reference = motor->target;

  if (reference == 0) {
    motor->target = 0;
    motor->current = 0.0f;
    noteApplicationCommand();
    return transmit(*motor, 0, true);
  }

  const int16_t brake =
      reference > 0 ? -static_cast<int16_t>(strength)
                    : static_cast<int16_t>(strength);

  motor->target = 0;
  motor->current = static_cast<float>(brake);
  motor->braking = true;
  motor->brakeCommand = brake;
  motor->brakeUntilMs = millis() + pulseMs;

  noteApplicationCommand();
  return transmit(*motor, brake, true);
}

bool TungLamPIDBus::hardBrake(uint8_t id,
                              uint8_t strength,
                              uint16_t pulseMs) {
  return activeBrake(id, strength, pulseMs);
}

void TungLamPIDBus::hardBrakeAll(uint8_t strength, uint16_t pulseMs) {
  noteApplicationCommand();

  for (uint8_t i = 0; i < TUNGLAM_PID_MAX_MOTORS; ++i) {
    MotorState& motor = motors_[i];
    if (!motor.active) continue;

    int16_t reference = quantize(motor.current);
    if (reference == 0) reference = motor.target;
    if (reference == 0) {
      motor.target = 0;
      motor.current = 0.0f;
      transmit(motor, 0, true);
      continue;
    }

    const int16_t brake =
        reference > 0 ? -static_cast<int16_t>(strength)
                      : static_cast<int16_t>(strength);

    motor.target = 0;
    motor.current = static_cast<float>(brake);
    motor.braking = true;
    motor.brakeCommand = brake;
    motor.brakeUntilMs = millis() + pulseMs;
    transmit(motor, brake, true);
  }
}

void TungLamPIDBus::setMinTxIntervalMs(uint16_t intervalMs) {
  minTxIntervalMs_ = intervalMs;
}

void TungLamPIDBus::setRefreshPeriodMs(uint16_t periodMs) {
  refreshPeriodMs_ = periodMs;
}

void TungLamPIDBus::setTransmitThreshold(uint8_t threshold) {
  transmitThreshold_ = threshold == 0 ? 1 : threshold;
}

void TungLamPIDBus::setCommandTimeout(uint32_t timeoutMs,
                                      TungLamPIDTimeoutAction action) {
  commandTimeoutMs_ = timeoutMs;
  timeoutAction_ = action;
  watchdogPrimed_ = false;
  timeoutTriggered_ = false;
}

bool TungLamPIDBus::commandTimedOut() const {
  return timeoutTriggered_;
}

float TungLamPIDBus::stepMotor(MotorState& motor, float dtSeconds) {
  const float target = static_cast<float>(motor.target);

  if (motor.current == target) return motor.current;

  // Never jump directly through zero during a direction reversal.
  if (signsOpposite(motor.current, target)) {
    const float maxDelta = motor.deceleration * dtSeconds;
    motor.current = approach(motor.current, 0.0f, maxDelta);

    if (fabsf(motor.current) < 0.5f) motor.current = 0.0f;
    return motor.current;
  }

  const float currentMagnitude = fabsf(motor.current);
  const float targetMagnitude = fabsf(target);
  const bool accelerating = targetMagnitude > currentMagnitude;
  const float rate = accelerating ? motor.acceleration : motor.deceleration;

  motor.current = approach(motor.current, target, rate * dtSeconds);

  if (fabsf(motor.current - target) < 0.5f) {
    motor.current = target;
  }

  return motor.current;
}

bool TungLamPIDBus::writeFrame(MotorState& motor, int16_t command) {
  if (serial_ == nullptr) return false;

  command = clampCommand(command);

  if (command > 0) {
    motor.directionBit = true;
  } else if (command < 0) {
    motor.directionBit = false;
  }

  const uint8_t speed =
      static_cast<uint8_t>(command < 0 ? -command : command);

  uint8_t frame[3];
  frame[0] = static_cast<uint8_t>((motor.directionBit ? 0x80 : 0x00) |
                                  (motor.id & 0x7F));
  frame[1] = speed;
  frame[2] = 0xFF;

  const size_t written = serial_->write(frame, sizeof(frame));
  return written == sizeof(frame);
}

bool TungLamPIDBus::transmit(MotorState& motor,
                             int16_t command,
                             bool force) {
  const uint32_t now = millis();
  const uint32_t elapsed = now - motor.lastTxMs;
  const int16_t delta = command - motor.lastSent;
  const uint16_t absDelta =
      static_cast<uint16_t>(delta < 0 ? -delta : delta);

  const bool changed = !motor.sentOnce || absDelta >= transmitThreshold_;
  const bool refreshDue =
      refreshPeriodMs_ > 0 && (!motor.sentOnce || elapsed >= refreshPeriodMs_);

  if (!force) {
    if (!changed && !refreshDue) return true;
    if (changed && !refreshDue && elapsed < minTxIntervalMs_) return true;
  }

  if (!writeFrame(motor, command)) return false;

  motor.lastSent = command;
  motor.lastTxMs = now;
  motor.sentOnce = true;
  return true;
}

void TungLamPIDBus::update() {
  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();

  if (!updateClockPrimed_) {
    lastUpdateUs_ = nowUs;
    updateClockPrimed_ = true;
  }

  float dt = static_cast<float>(nowUs - lastUpdateUs_) * 0.000001f;
  lastUpdateUs_ = nowUs;
  if (dt > kMaxUpdateDtSeconds) dt = kMaxUpdateDtSeconds;

  if (commandTimeoutMs_ > 0 && watchdogPrimed_ && !timeoutTriggered_ &&
      (nowMs - lastApplicationCommandMs_) >= commandTimeoutMs_) {
    timeoutTriggered_ = true;
    stopAllInternal(timeoutAction_ == TungLamPIDTimeoutAction::ImmediateStop);
  }

  for (uint8_t i = 0; i < TUNGLAM_PID_MAX_MOTORS; ++i) {
    MotorState& motor = motors_[i];
    if (!motor.active) continue;

    if (motor.braking) {
      if (deadlineReached(nowMs, motor.brakeUntilMs)) {
        motor.braking = false;
        motor.current = 0.0f;
        motor.target = 0;
        transmit(motor, 0, true);
      } else {
        transmit(motor, motor.brakeCommand, false);
      }
      continue;
    }

    stepMotor(motor, dt);
    transmit(motor, quantize(motor.current), false);
  }
}

int16_t TungLamPIDBus::currentCommand(uint8_t id) const {
  const MotorState* motor = findMotor(id);
  return motor == nullptr ? 0 : quantize(motor->current);
}

int16_t TungLamPIDBus::targetCommand(uint8_t id) const {
  const MotorState* motor = findMotor(id);
  return motor == nullptr ? 0 : motor->target;
}

uint8_t TungLamPIDBus::registeredCount() const {
  uint8_t count = 0;
  for (uint8_t i = 0; i < TUNGLAM_PID_MAX_MOTORS; ++i) {
    if (motors_[i].active) ++count;
  }
  return count;
}

// ============================================================================
// 4WD MECANUM FACADE
// ============================================================================

TungLamPID4WD::TungLamPID4WD(TungLamPIDBus& bus)
    : bus_(bus), configured_(false) {
  for (uint8_t i = 0; i < 4; ++i) {
    ids_[i] = 0;
    inverted_[i] = false;
  }
}

bool TungLamPID4WD::setMotorIDs(uint8_t m1FrontLeft,
                                uint8_t m2RearLeft,
                                uint8_t m3RearRight,
                                uint8_t m4FrontRight) {
  const uint8_t ids[4] = {
      m1FrontLeft, m2RearLeft, m3RearRight, m4FrontRight};

  for (uint8_t i = 0; i < 4; ++i) {
    if (ids[i] < 1 || ids[i] > 127) {
      configured_ = false;
      return false;
    }
  }

  // Reject duplicate addresses in the same base.
  for (uint8_t i = 0; i < 4; ++i) {
    for (uint8_t j = i + 1; j < 4; ++j) {
      if (ids[i] == ids[j]) {
        configured_ = false;
        return false;
      }
    }
  }

  for (uint8_t i = 0; i < 4; ++i) {
    ids_[i] = ids[i];
    if (!bus_.registerMotor(ids_[i])) {
      configured_ = false;
      return false;
    }
  }

  configured_ = true;
  return true;
}

bool TungLamPID4WD::setWheelInverted(uint8_t wheel, bool inverted) {
  if (wheel < 1 || wheel > 4) return false;
  inverted_[wheel - 1] = inverted;
  return true;
}

bool TungLamPID4WD::setMotionProfile(float acceleration,
                                     float deceleration) {
  if (!configured_) return false;

  bool ok = true;
  for (uint8_t i = 0; i < 4; ++i) {
    ok = bus_.setMotorMotionProfile(ids_[i], acceleration, deceleration) && ok;
  }
  return ok;
}

int16_t TungLamPID4WD::applyInversion(uint8_t wheelIndex,
                                      int16_t command) const {
  return inverted_[wheelIndex] ? -command : command;
}

int16_t TungLamPID4WD::clampBody(int16_t value) {
  if (value > 255) return 255;
  if (value < -255) return -255;
  return value;
}

void TungLamPID4WD::normalize4(int32_t& m1,
                               int32_t& m2,
                               int32_t& m3,
                               int32_t& m4) {
  int32_t maxMagnitude = abs(m1);
  if (abs(m2) > maxMagnitude) maxMagnitude = abs(m2);
  if (abs(m3) > maxMagnitude) maxMagnitude = abs(m3);
  if (abs(m4) > maxMagnitude) maxMagnitude = abs(m4);

  if (maxMagnitude <= 255) return;

  const float scale = 255.0f / static_cast<float>(maxMagnitude);
  m1 = static_cast<int32_t>(lroundf(static_cast<float>(m1) * scale));
  m2 = static_cast<int32_t>(lroundf(static_cast<float>(m2) * scale));
  m3 = static_cast<int32_t>(lroundf(static_cast<float>(m3) * scale));
  m4 = static_cast<int32_t>(lroundf(static_cast<float>(m4) * scale));
}

bool TungLamPID4WD::setWheels(int16_t m1,
                              int16_t m2,
                              int16_t m3,
                              int16_t m4) {
  if (!configured_) return false;

  const int16_t commands[4] = {
      clampBody(m1), clampBody(m2), clampBody(m3), clampBody(m4)};

  bool ok = true;
  for (uint8_t i = 0; i < 4; ++i) {
    ok = bus_.setTarget(ids_[i], applyInversion(i, commands[i])) && ok;
  }
  return ok;
}

bool TungLamPID4WD::drive(int16_t vx, int16_t vy, int16_t wz) {
  vx = clampBody(vx);
  vy = clampBody(vy);
  wz = clampBody(wz);

  // Tung Lam Automation / V5 wheel convention:
  // [M1,M2,M3,M4] = [FL,RL,RR,FR]
  // +vx -> [+,+,+,+]
  // +vy -> [-,+,-,+]
  // +wz -> [-,-,+,+]
  int32_t m1 = static_cast<int32_t>(vx) - vy - wz;
  int32_t m2 = static_cast<int32_t>(vx) + vy - wz;
  int32_t m3 = static_cast<int32_t>(vx) - vy + wz;
  int32_t m4 = static_cast<int32_t>(vx) + vy + wz;

  normalize4(m1, m2, m3, m4);

  return setWheels(static_cast<int16_t>(m1),
                   static_cast<int16_t>(m2),
                   static_cast<int16_t>(m3),
                   static_cast<int16_t>(m4));
}

bool TungLamPID4WD::forward(uint8_t speed) {
  return drive(speed, 0, 0);
}

bool TungLamPID4WD::backward(uint8_t speed) {
  return drive(-static_cast<int16_t>(speed), 0, 0);
}

bool TungLamPID4WD::strafeLeft(uint8_t speed) {
  return drive(0, speed, 0);
}

bool TungLamPID4WD::strafeRight(uint8_t speed) {
  return drive(0, -static_cast<int16_t>(speed), 0);
}

bool TungLamPID4WD::rotateLeft(uint8_t speed) {
  return drive(0, 0, speed);
}

bool TungLamPID4WD::rotateRight(uint8_t speed) {
  return drive(0, 0, -static_cast<int16_t>(speed));
}

void TungLamPID4WD::softStop() {
  if (!configured_) return;
  for (uint8_t i = 0; i < 4; ++i) bus_.softStop(ids_[i]);
}

void TungLamPID4WD::stop() {
  if (!configured_) return;
  for (uint8_t i = 0; i < 4; ++i) bus_.stop(ids_[i]);
}

void TungLamPID4WD::hardBrake(uint8_t strength, uint16_t pulseMs) {
  if (!configured_) return;
  for (uint8_t i = 0; i < 4; ++i) {
    bus_.activeBrake(ids_[i], strength, pulseMs);
  }
}

void TungLamPID4WD::update() {
  bus_.update();
}
