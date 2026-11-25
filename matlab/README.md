# MATLAB Wireless Tune UI

`WirelessTuneUI.m` is a single-file MATLAB dashboard for the robot's USART6 wireless link.
UI labels and messages are Chinese. Parameter names/units, protocol enums, and CSV
column headers remain English so they continue to match the firmware definitions.

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
- `Save Log` writes `.mat` or `.csv`;
- `Auto-save attitude CSV` writes continuous telemetry to `matlab/logs/attitude_YYYYMMDD_HHMMSS.csv`;
- `Excitation...` opens a target excitation window.

The car firmware currently returns only parameter `status/version/seq/id`, not the current parameter values. The table is therefore a **local editing table**, not a readback of the car.

## Target excitation

`Excitation...` can send a 10 Hz target sequence through the existing 64 B target command:

- modes: `ATTITUDE`, `SPEED`, `POSITION`;
- signals: `PRBS`, `Sine`, `Square`, `Step`;
- amplitude, offset, frequency, duration;
- `Stop` sends a zero target in the same mode.

This is a **closed-loop target excitation**, not an open-loop motor PWM test. It can be used to observe/identify the closed-loop response. For a true plant transfer function, add a firmware test mode with bounded PRBS/relay directly on the relevant PWM channel and use a test stand.

Safety:

- only enter DEBUG on the car with KEY0;
- keep the robot stationary/supported;
- attitude/position excitation can move the robot;
- verify the amplitude limits before starting.

## Identification notes

- 50 Hz × 10 s = 500 samples;
- suitable for a low-order yaw-rate/speed model only if the excitation is sufficient;
- pitch/balance identification should use a test stand;
- do not auto-apply gains from one short closed-loop record;
- recommend displaying model FIT and stability margins before applying any gains.


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
