# RC5 IR Protocol with Magnitude Support (Project 2 Rev 2)

This project uses a simplified RC5 command protocol for the controller IR transmitter, with joystick magnitude encoded for variable-speed movement control.

## Physical Layer

- Carrier frequency: `38 kHz`
- IR LED pin on controller: `RB6`
- Modulation:
  - `MARK` = 38 kHz carrier burst
  - `SPACE` = IR LED off

## Frame Format (14 bits, MSB first)

`S1 S2 T A1 A0 C5 C4 C3 C2 C1 C0 M2 M1 M0`

- `S1`, `S2`: start bits, always `1`
- `T`: toggle bit (flips each transmitted command)
- `A1..A0`: 2-bit address (`IR_RC5_ADDRESS`, currently `0x00`)
- `C5..C0`: 6-bit command (0-63, see Commands section below)
- `M2..M0`: 3-bit magnitude (0-7, representing joystick intensity)
  - `0` = no movement / neutral
  - `7` = full joystick deflection
  - Scaled from 10-bit ADC joystick position (0-1023)

## Timing

- Half bit: `889 us`
- Full bit: `1778 us`
- Manchester coding:
  - Logical `1` = `MARK(889 us)` then `SPACE(889 us)`
  - Logical `0` = `SPACE(889 us)` then `MARK(889 us)`
- Full frame: ~24.9 ms (14 bits x 1778 us)

## Commands (6-bit encoding, 0-63)

### Direction Commands (use magnitude)
- `IR_CMD_FORWARD` (0x1): Move forward at magnitude 0-7
- `IR_CMD_BACKWARD` (0x2): Move backward at magnitude 0-7
- `IR_CMD_LEFT` (0x3): Turn left at magnitude 0-7
- `IR_CMD_RIGHT` (0x4): Turn right at magnitude 0-7

### Special Commands (magnitude usually 0)
- `IR_CMD_STOP` (0x0): Stop all motion
- `IR_CMD_ROTATE_180` (0x5): Rotate 180 degrees
- `IR_CMD_PATH_1` (0x6): Execute predefined path 1
- `IR_CMD_PATH_2` (0x7): Execute predefined path 2
- `IR_CMD_PATH_3` (0x8): Execute predefined path 3
- `IR_CMD_AUTO_START` (0x9): Start autonomous mode
- `IR_CMD_AUTO_STOP` (0xA): Stop autonomous mode
- `IR_CMD_CUSTOM_1` (0xB): Execute custom path 1
- `IR_CMD_CUSTOM_2` (0xC): Execute custom path 2
- `IR_CMD_CUSTOM_3` (0xD): Execute custom path 3
- `IR_CMD_CUSTOM_4` (0xE): Execute custom path 4

## Magnitude Scaling

The joystick provides a 10-bit ADC value (0-1023) for both X and Y axes. Magnitude is computed as:

```c
// Maximum distance from center (512) to edge (0 or 1023) is 512 units
max_offset = max(abs(x - 512), abs(y - 512));
magnitude = (max_offset * 7) / 512;
// Result: 0-7
```

This means:
- Center position (near center): magnitude ~0
- Moderate deflection: magnitude ~3-4
- Full deflection: magnitude 7

## Robot Side Integration

The robot receiver should:
1. Decode the frame to extract command and magnitude
2. For direction commands (FORWARD, BACKWARD, LEFT, RIGHT):
   - Use magnitude to set motor speed (0 = stopped, 7 = full speed)
3. For special commands (PATH_*, AUTO_*, etc.):
   - Ignore magnitude or use as a modifier (for example, a path speed factor)

## Integration Notes

- Encoder is implemented in `ir_rc5.c` / `ir_rc5.h`
- Joystick reading in `joystick.c` / `joystick.h` includes magnitude computation
- Controller sends commands from `controller.c` through `IrSendCommand(cmd, magnitude)`
- Each command is transmitted twice (`IR_TX_REPEATS`) for basic robustness
- E-stop mode repeatedly transmits `IR_CMD_STOP` with magnitude 0

## Example Transmitter Usage

```c
// Send forward command at about 75% intensity
uint8_t magnitude = 5;  // ~71% of 7
Joystick_State js;
Joystick_Read(&js);      // Updates js.magnitude (0-7)

IrSendCommand(ctx, IR_CMD_FORWARD, js.magnitude);
```

## Example Receiver Usage (pseudocode)

```c
if (decoded_frame.command == IR_CMD_FORWARD) {
    // Map 0-7 magnitude to motor speed (for example, 0-255)
    uint8_t motor_speed = (decoded_frame.magnitude * 255) / 7;
    SetMotorSpeed(FORWARD, motor_speed);
} else if (decoded_frame.command == IR_CMD_AUTO_START) {
    // Special commands ignore magnitude
    StartAutonomousMode();
}
```
