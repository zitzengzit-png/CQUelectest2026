# MATLAB Wireless Tune UI

`WirelessTuneUI.m` is a single-file MATLAB dashboard for the robot's USART6 wireless link.

## Run

```matlab
cd matlab
WirelessTuneUI
```

Tested against MATLAB R2025b. `ble`, `serialport`, and `uifigure` are available.

## Transport modes

| Mode | Meaning |
|---|---|
| `BLE` | MATLAB connects directly to BT27 as a BLE Central |
| `Serial` | MATLAB uses a USB BLE-Central virtual COM port; it does **not** connect to the BT27 UART pins directly |
| `Demo` | Generates synthetic 86 B telemetry for UI testing |

For BLE:

- scan for `BT27`;
- connect to service `FFE0`;
- notify characteristic `FFE1`;
- write characteristic `FFE2` (fallback `FFE1`);
- disconnect the phone before connecting MATLAB.

## UI functions

- Live speed target/actual plot;
- Pitch/Yaw plot;
- left/right PWM plot;
- editable PID parameter table (local values);
- target mode/target1/target2 controls;
- `Send Selected` sends up to 8 selected parameters;
- `Send Target` sends target only;
- `Hold ATT 0/0` sends ATTITUDE(0,0);
- `Save Log` writes `.mat` or `.csv`.

The car firmware currently returns only parameter `status/version/seq/id`, not the current parameter values. The table is therefore a **local editing table**, not a readback of the car.

## Protocol

Telemetry and tuning frame definitions are in
[`prj/elet2026/tasks/WIRELESS_TUNING.md`](../prj/elet2026/tasks/WIRELESS_TUNING.md).

Summary:

```text
Telemetry: 86 B, AA 55 | len=80 | ver=2 | LCD_Data_t | CRC16
Tune:      64 B, A5 5A | ver=1 | type=1 | seq | mode | count |
                 target1 | target2 | 8 x {id,float32} | reserved | CRC16
```

## Safety

- Enter DEBUG on the car with KEY0 first.
- Keep the robot stationary or supported while tuning.
- Send one command at a time and wait for `param_status/param_version`.
- The car queues are latest-only; rapid writes can drop intermediate commands.
- MATLAB/Windows BLE is not a hard-real-time control path. Do not close the 250 Hz loop in MATLAB.
