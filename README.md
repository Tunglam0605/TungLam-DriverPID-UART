# TungLam DriverPID UART

Arduino library for controlling **TheGioiChip Smart PID motor drivers** over UART.

Designed from the UART motor-control pattern previously used in TungLamvsRoBo_2025/Robot0x01, but refactored into reusable layers so the same library can control:

- one motor for a mechanism,
- several independent mechanisms,
- a 4-wheel Mecanum/Omni-style base,
- future higher-level robot controllers.

## Design goals

- Simple signed-speed API: `-255..255`
- HardwareSerial or SoftwareSerial-compatible transport
- Time-based acceleration/deceleration independent of `loop()` frequency
- Non-blocking `update()`
- Per-motor motion profiles
- Soft stop, immediate stop, watchdog, and active reverse braking
- UART TX suppression + periodic refresh
- Fixed-size storage, no dynamic allocation
- 4WD Mecanum facade using Tung Lam Automation wheel convention

## UART frame

The driver command used by the proven Robot0x01 code is three bytes:

```text
Byte 0: [DIR:1][ADDRESS:7]
Byte 1: SPEED (0..255)
Byte 2: 0xFF
```

Positive signed speed maps to `DIR=1`, negative signed speed maps to `DIR=0`.

## Quick start

```cpp
#include <TungLam_DriverPID_UART.h>

TungLamPIDBus pid;

void setup() {
  pid.begin(Serial2, 115200);
  pid.setDefaultMotionProfile(400.0f, 800.0f);
}

void loop() {
  pid.setTarget(1, 180);
  pid.update();
}
```

### Immediate command

```cpp
pid.setMotor(1, 180);   // send immediately
pid.setMotor(1, -180);  // reverse
pid.stop(1);            // speed = 0 immediately
```

### Smooth command

```cpp
pid.setMotorMotionProfile(1, 350.0f, 700.0f);
pid.setTarget(1, 220);

void loop() {
  pid.update();
}
```

## 4WD Mecanum

Wheel order is intentionally aligned with TungLam-OmniMecanum-4WD:

```text
M1 = Front Left
M2 = Rear Left
M3 = Rear Right
M4 = Front Right
```

Body convention:

```text
+vx = forward
+vy = left
+wz = counter-clockwise / left rotation
```

Example:

```cpp
TungLamPIDBus pid;
TungLamPID4WD robot(pid);

void setup() {
  pid.begin(Serial2, 115200);
  robot.setMotorIDs(1, 2, 3, 4);
  robot.setMotionProfile(450.0f, 900.0f);
}

void loop() {
  robot.drive(180, 0, 60);
  robot.update();
}
```

## Stop semantics

- `softStop()`: target becomes zero; deceleration profile is respected.
- `stop()`: speed becomes zero immediately and a zero command is transmitted.
- `hardBrake()`: opt-in active reverse pulse, then zero. This is **not a documented native brake opcode**; it deliberately commands reverse torque and must be tuned on real hardware.
- `emergencyStop()`: all registered motors are forced to zero immediately.

> Active reverse braking can create high current and mechanical shock. Start with a low brake strength and short pulse.

## UART scheduling

The library does not spam the UART every `loop()` iteration.

It supports:

- minimum TX interval,
- change threshold,
- periodic command refresh.

Defaults are conservative and can be changed:

```cpp
pid.setMinTxIntervalMs(5);
pid.setRefreshPeriodMs(100);
pid.setTransmitThreshold(1);
```

## Watchdog

```cpp
pid.setCommandTimeout(300, TungLamPIDTimeoutAction::ImmediateStop);
```

If the application stops issuing commands, the library can stop the registered motors.

## Examples

- `01_SingleMotor_Basic`
- `02_SingleMotor_Ramp`
- `03_Mecanum_4WD`
- `04_ActiveBrake`

## Compatibility

The core only depends on Arduino `Stream` and fixed-size C++ storage. It is intended to be portable across Arduino-compatible boards with a UART implementation.

For a serial class that implements `begin(baud)`:

```cpp
pid.begin(Serial2, 115200);
```

For an already-configured `Stream`:

```cpp
mySerial.begin(115200);
pid.attach(mySerial);
```

## License

MIT License.

Copyright (c) 2026 Nguyen Khac Tung Lam — Tung Lam Automation.
