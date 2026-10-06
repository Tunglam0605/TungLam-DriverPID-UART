# Changelog

## 0.1.0 - 2026-10-06

Initial public release.

- Reusable Driver PID UART bus abstraction.
- 3-byte frame compatible with the proven Robot0x01 implementation.
- Signed speed API (-255..255).
- HardwareSerial / SoftwareSerial-like begin helper plus generic Stream attach.
- Time-based per-motor acceleration and deceleration.
- Safe zero-crossing during direction reversal.
- Soft stop and immediate stop.
- Command watchdog.
- UART change suppression and periodic refresh.
- Non-blocking active reverse braking.
- 4WD Mecanum facade using M1=FL, M2=RL, M3=RR, M4=FR.
