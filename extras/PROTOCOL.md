# UART Protocol

## Frame

Each motor command is three bytes:

| Byte | Meaning |
|---|---|
| 0 | bit7 = direction, bit6..0 = motor address |
| 1 | speed magnitude 0..255 |
| 2 | fixed terminator 0xFF |

The library uses motor IDs 1..127.

## Signed command mapping

The public API uses signed commands:

| API value | Direction bit | Speed byte |
|---:|---:|---:|
| +180 | 1 | 180 |
| -180 | 0 | 180 |
| 0 | previous direction | 0 |

Keeping the previous direction while speed is zero matches the behavior of the
older Robot0x01 code, where direction and speed were stored independently.

## Example

For motor address 3 at +200:

```text
Byte0 = 0b10000011 = 0x83
Byte1 = 0xC8
Byte2 = 0xFF
```

For motor address 3 at -200:

```text
Byte0 = 0b00000011 = 0x03
Byte1 = 0xC8
Byte2 = 0xFF
```

## TX policy

`setTarget()` updates the desired command. `update()` advances the motion
profile according to elapsed time and transmits only when necessary.

Three controls limit bus traffic:

- `setMinTxIntervalMs()`
- `setTransmitThreshold()`
- `setRefreshPeriodMs()`

## Active reverse braking

`activeBrake()` / `hardBrake()` are library-side behaviors, not an extra
UART opcode. The function briefly commands the opposite direction, then sends
zero.

Because this can produce high current, tune strength and pulse duration on the
actual motor, supply, and driver.
