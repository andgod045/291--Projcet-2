# Recommended Joystick Control Scheme (VRX, VRY, SW)

> Note: The authoritative March 2026 controller/robot agreement for path select/run/edit behavior is documented in `PATH_EDITING_SYSTEM.md`.

This control scheme uses only the joystick axes (`vrx`, `vry`) and switch (`sw`). It is intentionally simple for the operator, while pushing path execution details to the robot side.

## 1) Command Set Coverage

All required commands are supported:

- `STOP` (regular stop)
- `TURN_LEFT`
- `TURN_RIGHT`
- `MOVE_FORWARD`
- `MOVE_BACKWARD`
- `ROTATE_180` (gesture: rapid up->down or down->up)
- Path select/config + auto mode start/stop
- `E_STOP` (emergency stop, separate from regular stop)
- Custom path define/upload/select

## 2) Operator Input Mapping

Assume joystick sampled every ~50 ms.

| Input                      | MANUAL                                  | AUTO_SELECT                                                 | CUSTOM_EDIT             | AUTO_RUN                | E_STOP                    |
|----------------------------|-----------------------------------------|-------------------------------------------------------------|-------------------------|-------------------------|---------------------------|
| Stick UP                   | Direction = (0,7)                       | No-op                                                       | Set step = `FORWARD`    | No-op                   | Ignore                    |
| Stick DOWN                 | Direction = (0,-7)                      | No-op                                                       | Set step = `BACKWARD`   | No-op                   | Ignore                    |
| Stick LEFT                 | Direction = (-7,0)                      | Select previous path/slot                                   | Set step = `LEFT`       | No-op                   | Ignore                    |
| Stick RIGHT                | Direction = (7,0)                       | Select next path/slot                                       | Set step = `RIGHT`      | No-op                   | Ignore                    |
| Stick CENTER               | `STOP` (regular stop)                   | No-op                                                       | No-op                   | No-op                   | Required for clear        |
| Rapid UP->DOWN OR DOWN->UP | No-op                                   | No-op                                                       | Set step = `ROTATE_180` | No-op                   | Ignore                    |
| SW short press             | `ROTATE_180`                            | Cancel -> `MANUAL`                                          | Begin editing next step | No-op                   | Ignore                    |
| SW double press            | Enter `AUTO_SELECT` (send `STOP` first) | Confirm selection: run predefined/custom OR enter edit mode | Save path -> `MANUAL`   | `AUTO_STOP` -> `MANUAL` | Ignore                    |
| SW long press (~1.2 s)     | Enter `E_STOP`                          | Enter `E_STOP`                                              | Enter `E_STOP`          | Enter `E_STOP`          | Attempt clear (see below) |

### Double Press Timing Rule

- A double press is two valid short presses with **<= 500 ms** between releases.
- If no second press arrives within 500 ms, the first press is treated as a normal single short press.

### Regular Stop vs Emergency Stop

- **Regular stop (`STOP`)**: normal motion halt (triggered by `CENTER`, and also before mode changes).
- **Emergency stop (`E_STOP`)**: safety-latched state, overrides everything, must be explicitly cleared.
- Protocol mapping: regular stop uses `IR_MISC_STOP`; emergency latch uses `IR_MISC_ESTOP`; clear uses `IR_MISC_ESTOP_CLEAR`.

## 3) Rotate 180 Gesture Definition

Use a gesture detector:

- In `MANUAL`: sends `ROTATE_180`.
- In `CUSTOM_EDIT`: assigns current path step as `ROTATE_180`.

Rules:

- Trigger source: direction goes to `UP` then `DOWN`, or `DOWN` then `UP`.
- Maximum interval between opposite directions: **<= 300 ms** (<= 6 samples at 50 ms/sample).
- Require neutral re-arm: after trigger, joystick must return to `CENTER` before next gesture.
- Add cooldown: **~1 s** lockout after trigger to avoid accidental retrigger.

## 4) Predefined and Custom Paths

### 4.1 Predefined Path Selection

Remote behavior:

- Enter `AUTO_SELECT` with `SW short` from `MANUAL`.
- `LEFT/RIGHT`: cycle `Path 1`, `Path 2`, `Path 3`, then custom slots (`Custom 1..N`).
- `SW double` on selected slot:
  - predefined slot -> start auto run
  - custom slot -> enter `CUSTOM_EDIT`
- `SW short` cancels selection and returns to `MANUAL`.

Robot behavior (recommended handoff):

- Robot stores intersection actions for each path.
- Remote sends only path ID and start/stop commands.

Required predefined paths:

| Intersection | Path 1 | Path 2 | Path 3 |
|---|---|---|---|
| 1 | Forward | Left | Right |
| 2 | Left | Right | Forward |
| 3 | Left | Left | Right |
| 4 | Forward | Right | Left |
| 5 | Right | Forward | Right |
| 6 | Left | Forward | Left |
| 7 | Right | Stop | Forward |
| 8 | Stop | N/A | Stop |

### 4.2 Custom Path Definition (No Timing Required)

Because the robot is line-following and consumes one direction at each intersection, each custom path is a simple ordered list of per-intersection actions:

- Allowed step values: `FORWARD`, `BACKWARD`, `LEFT`, `RIGHT`, `ROTATE_180`, `STOP`
- No timing fields are used.
- `STOP` can terminate a path early.

Recommended custom path editing flow (joystick-only):

- In `AUTO_SELECT`, choose a custom slot then `SW double` to enter `CUSTOM_EDIT`.
- In `CUSTOM_EDIT`:
  - Direction input assigns the current step (`UP=FORWARD`, `DOWN=BACKWARD`, `LEFT`, `RIGHT`, `Rapid U<->D=ROTATE_180`, `CENTER=FORWARD`)
  - `SW short`: save current step and move to next step index
  - `SW double`: set current step to `STOP`, save path, return to `MANUAL`

Storage recommendation:

- Store custom paths on the robot (not controller) to keep controller memory simple.
- Robot exposes a fixed number of custom slots (for example `Custom 1..Custom 4`).
