# Migration from Robot0x01

The original project used:

```cpp
DataTX[i][0] = (direction << 7) | (address & 0x7F);
DataTX[i][1] = speed;
DataTX[i][2] = 0xFF;
Serial2.write(DataTX[i], 3);
```

and multiple motion-specific ramp functions such as:

```text
TangtocUart()
TangtocUart2()
TangtocUart3()
TangtocUart4()
TangtocUart5()
StopUart()
```

The new library separates responsibilities:

```text
Application / PS2
      |
      v
Mecanum mixer or mechanism command
      |
      v
Per-motor MotionProfile
      |
      v
DriverPID UART bus
      |
      v
3-byte frames
```

## Old -> new

```cpp
// Old
TienTx();
Prepare_Data();

// New
robot.forward(180);
robot.update();
```

```cpp
// Old
StopUart();

// New, ramped
robot.softStop();

// New, immediate
robot.stop();
```

The new ramp is time-based, so acceleration no longer depends on how fast
`loop()` happens to run.
