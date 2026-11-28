# MATLAB Wireless Tune UI

`WirelessTuneUI.m` is a single-file MATLAB dashboard for the robot's USART6 wireless link.
UI labels and messages are Chinese. Parameter names/units, protocol enums, and CSV
column headers remain English so they continue to match the firmware definitions.

> **控制仿真**（建模 / 自动调参 / 可视化）见 **[`SIMULATION.md`](SIMULATION.md)**；
> 相关脚本：`balance_model.m`、`autotune_pid.m`、`vis_*.m`、`bm_draw_car.m`、
> `read_robot_config.m`（参数直接读 `prj/elet2026/tasks/robot_config.h`）。

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

## Balance model (`balance_model.m`)

Self-contained parameterised model + controller design + numerical verification
for the GA25-370 / 76 mm wheel balancer.

**Since 2026-12 the model is driven by the firmware**: `read_robot_config.m`
parses `../prj/elet2026/tasks/robot_config.h`, so all loop gains, limits, loop
rates (inner 250 Hz / middle 50 Hz / outer 10 Hz), wheel/CPR geometry and the
new `PID_ATT_FF_SPEED` / `PID_DELAY_COMP_S` come from the code. `sim_balance`
enforces the firmware rates: control updated at 4 ms with zero-order hold,
encoder speed at 50 Hz (20 ms window ending 2 ms ago, quantised to 7.75 mm/s),
10 ms IMU transport delay, speed feedforward and pitch prediction. The `[2c]`
table prints "code / design / used" for every loop.

Every other input carries a source tag: `DATASHEET` (GA25-370BMQ manual V1.0),
`MEASURED` (this car) or `ESTIMATE` (mass / inertia / deadzone / yaw damping).

**Plant friction (since 2026-12).** The wheel/gearbox is modelled with a Coulomb
+ smooth-stiction term whose magnitude equals the measured deadband
(`tau_f = MOTOR_DEADBAND_PWM * Kt`, i.e. 170 counts -> 0.013 N.m),
`tau = Kt*u - droop*v - tau_f*tanh(v/0.010)`. This replaces the earlier
"actuator dead zone" (`|u| < dead -> torque = 0`), which is *not* friction and
over-estimates the limit cycle. `P.plant = 'fric'` (default) / `'zone'` (legacy,
kept only for the comparison table).

**Friction feedforward (since 2026-12).** The firmware adds
`pwm += MOTOR_DEADBAND_PWM * tanh(pwm / MOTOR_DEADBAND_SMOOTH_EPS)` before the
direction decision, and `sim_balance` reproduces exactly that (`P.dead_comp`,
`P.dead_smooth`, `P.dead_eps`, all read from `robot_config.h`). `[5b]` compares
`zone` / `fric` / `fric+sign` / `fric+tanh` and sweeps `eps` (5/10/20/40).



```matlab
cd matlab
balance_model          % interactive: prints tables and draws 2 figures
% or headless:
% matlab -batch "cd('.../matlab'); balance_model"
```

Console sections:

1. plant: `v_nl`, stall torque, `a_max`, `M`, `Ieff`, `l_eff`, `w0`;
2. pitch PD design: `Kp`/`Kd`, closed-loop poles, gain crossover, PM, exact
   delay margin, and the PM left at 10/20 ms delay;
3. **motor back-EMF**: the closed loop is *unstable* without a speed
   feedforward (a right-half-plane pole at ~+4.6 /s); the required feedforward is
   `k_ff = (tau_stall/v_nl)/Kt` PWM counts per m/s (~6207 at 12 V), i.e.
   `common += k_ff * current_speed`;
4. max stable `Kp` vs loop delay (bisection on the exact phase-margin condition);
5. battery voltage / body-inertia sensitivity;
6. time-domain simulation (1 ms step) with PWM saturation, motor droop, loop
   delay, plant friction, the speed feedforward and the friction feedforward:
   release 5°, 0.3 m/s push, 0.3 m position step, 0.4 m/s speed step; includes a
   `kp_speed` sweep (8 / 12 / 19.5) and a delay-compensation test
   (`theta_used = theta + rate*T`);
6b. deadband / friction compensation table (`[5b]`): `zone` vs `fric` plant,
   compensation off vs on — steady-state position error and the residual
   pitch/speed limit cycle, plus an `eps` sweep;
6c. **requirement 1 stand-still test (`[10]`)**: `HOLD = POSITION(0,0)` with five
   excitations (3°/5° release, 0.20 m/s push, 1.5 N / 3.0 N for 0.15 s), checked
   against the 15 cm / 3 s rule, plus `kp_pos` and `max_pitch` sweeps;
7. yaw-rate PI design (first-order plant, `tau_yaw` unknown -> 3 cases) and a
   45 deg/s step simulation;
8. arc differential demand at R=0.40 m / 0.40 m/s vs `PID_ATT_MAX_DIFF_PWM`;
9. PASS/FAIL verdict table.

Notes:

- `Jp`, the masses, `u_dead` and `tau_yaw` are estimates. The free-fall test
  (5 deg -> 20 deg, `t = 2.063/w0`) and a full-duty acceleration run pin them
  down; put the measured values into `params()`.
- The script writes no files. Interactive runs only draw figures.



## Motion visualisation (`vis_balance.m`)

Run `balance_model` first (it writes `logs/balance_model.mat`), then
`vis_balance`. It writes two files into `matlab/logs/` (git-ignored):

- `balance_strobe.png` — 9-snapshot strobe of a **0.25 m position step** (side
  view: wheel, struts, top plate, board/battery block, CoG dot and CoG trace)
  plus the pitch / forward-speed traces;
- `balance_anim.gif` — side-view animation of a **0 → 0.40 m/s speed step**
  (wheels rolling with `phi = -x/R`, body tilt, CoG trace, camera follows the car).

The drawing uses only the model parameters (`R`, `l`, plate/board), so it stays
consistent with `balance_model.m` if you change `params()`. `logs/` is
git-ignored; re-run the two scripts to regenerate.


## Attitude-only disturbance test (`vis_attitude.m`)

`balance_model` also runs an **inner-loop-only** stand with environmental
disturbances (section `[9]`): a horizontal force at the CoG (1.5 N and 3.0 N for
150 ms, plus a steady 0.4 N wind/slope) and sensor noise (0.15 deg / 1 deg/s),
on top of the 10 ms loop delay, the pitch prediction and the 4% deadzone. It
prints max tilt / settle / peak PWM / drift for **attitude-only** and for
**HOLD/POSITION(0,0)** side by side.

`vis_attitude.m` (run after `balance_model`) writes `logs/attitude_dist.png`:
strobe of the upright-but-sliding car, the pitch of both modes with the push
shaded, and the forward position.

Key result: the attitude loop alone holds the *angle* (≈3 deg for a 1.5 N push)
but cannot stop the *motion* (≈158 cm drift); a 3 N push or a steady 0.4 N force
makes it fall, while HOLD/POSITION survives the 3 N push only when the peak stays
below ~2 N (37 cm standing error under a steady 0.4 N). HOLD/POSITION is required
for "stand in place".

## Requirement 1 stand-still test (`vis_stand.m`)

`balance_model` section `[10]` runs `HOLD = POSITION(0,0)` with five excitations
and checks the rule "≤15 cm in 3 s": 3°/5° release, 0.20 m/s initial push, and a
1.5 N / 3.0 N horizontal force for 0.15 s. It also sweeps `kp_pos`
(0.7/1.0/1.5/2.0) and `PID_SPEED_MAX_PITCH_DEG` (10/15/20).

`vis_stand.m` (run after `balance_model`) writes `logs/stand_still.png`: strobe of
the car coming back, pitch (with the push shaded) and forward position against the
±15 cm requirement band.

Current numbers (12 V, Td = 10 ms, friction feedforward on):

| excitation | max tilt | max |x| | back < 2 cm | verdict |
|---|---|---|---|---|---|
| release 3° | 3.00° | 1.7 cm | 0.00 s | PASS |
| release 5° | 5.00° | 2.8 cm | 0.39 s | PASS |
| push 0.20 m/s | 4.51° | 5.2 cm | 2.08 s | PASS |
| push 1.5 N / 0.15 s | 9.85° | 13.0 cm | 2.97 s | PASS (2 cm margin) |
| push 3.0 N / 0.15 s | — | — | — | FAIL (falls) |

Note: requirement 1 is "no external force", so the 3°/5° rows are the actual rule
check (2.8 cm); the push rows are only a robustness margin.

## Slope / wind bias: where should the integral go? (`[10b]`, `[10c]`)

A constant 0.4 N horizontal force (≈2° floor) is a pure `P` killer: with the
current `kp_pos = 0.7` the car creeps **39.6 cm in 6 s** and keeps going (the loop
has no equilibrium: zero lean ⇒ it accelerates; a lean needs a speed error). Two
candidate fixes are simulated (both are **design studies — not implemented in the
firmware yet**, `P.ki_pos` / `P.ki_speed`):

* position-loop integral (`[10b]`): removes the bias error but costs **22–45 %
  overshoot** on a 0.30 m step → rejected;
* speed-loop integral (`[10c]`, `th_ref = kp_speed·e_v + ki_speed·∫e_v dt`, with
  the integral clamped to ±`max_pitch` and frozen when saturated): bias error
  drops 39.6 → **0.4 cm**, step overshoot stays **1.5 %**, and the 1.5 N push
  response is unchanged (13.0 cm).

| kp_pos | ki_speed | 0.4 N: x@6 s | step rise | overshoot | 1.5 N push |
|---|---|---|---|---|---|
| 0.70 | 0 | 39.6 cm | 2.55 s | 0.0 % | 12.9 cm |
| 1.20 | 0 | 24.7 cm | 1.06 s | 1.1 % | 13.1 cm |
| 1.20 | 2 | 10.6 cm | 0.99 s | 1.5 % | 13.0 cm |
| 1.20 | 4 | 8.2 cm | 0.93 s | 1.6 % | 13.1 cm |

Longer run (12 s) with `kp_pos = 1.2`: `ki_speed = 4` gives **x = −0.7 cm**
(8 → 0.4 cm, 16 → −0.4 cm, all with ≤1.6 % overshoot).

**Recommendation**: `PID_POS_KP_DISTANCE = 1.2`, plus a speed-loop integral
`PID_SPEED_KI ≈ 4 deg per (m·s)` with the clamp `±PID_SPEED_MAX_PITCH_DEG` and
conditional integration. Both are single knobs; the firmware part is ~15 lines in
`SpeedController_Update()` + 2 macros.

## PID auto-tuning (`autotune_pid.m`)

`balance_model` exposes its internals so the tuner reuses exactly the same plant
and simulator (no duplicated code):

```matlab
api = balance_model('api');   % -> struct of handles: params / derive / sim / delay_margin / ternary
```

`autotune_pid` optimises four firmware-relevant gains by **coordinate pattern
search in log2 space** (step 2^0.5 → 2^0.04, then a ±3/±6 % refinement pass) —
no toolbox required:

| variable | firmware macro | unit |
|---|---|---|
| `kp_pitch` | `PID_ATT_KP_PITCH` | PWM/deg |
| `kd_pitch` | `PID_ATT_KD_PITCH` | PWM/(deg/s) |
| `kp_speed` | `PID_SPEED_KP` | deg/(m/s) |
| `kp_pos` | `PID_POS_KP_DISTANCE` | 1/s |

The cost is a weighted sum of five simulated cases, each normalised by a target
value (penalties for falling and for hitting PWM saturation; each case capped so
one term cannot dominate):

| case | what it measures |
|---|---|
| stand | 5° release → max tilt / max \|x\| / settle / final x |
| step | 0.30 m position step → overshoot / rise / steady error |
| speed | 0 → 0.40 m/s speed step → overshoot / rise / final speed |
| push | 1.5 N for 0.15 s → max tilt / max \|x\| / recovery |
| bias | 0.4 N constant (≈3° slope) → standing offset |

Tuning runs without sensor noise (repeatable cost); the reported table re-runs
both parameter sets **with** 0.15° / 1°/s rms noise and also checks the phase
margin at the 10 ms loop delay.

```matlab
cd matlab
autotune_pid              % 200 evals / 420 s budget
autotune_pid(400, 600)
```

Result (12 V, Td = 10 ms, 107 evals, J 41.6 → 10.1, 4 s of wall time):

| | baseline (firmware today) | tuned |
|---|---|---|
| `kp_pitch` | 162.5 | **149.0** |
| `kd_pitch` | 8.66 | **10.30** |
| `kp_speed` | 12.0 | 12.0 |
| `kp_pos` | 0.70 | **1.44** |
| step rise | 2.37 s | **0.76 s** |
| step overshoot | 0.0 % | 3.1 % |
| step error @4 s | −1.11 cm | 0.21 cm |
| stand settle | 0.51 s | 0.43 s |
| speed-step overshoot | 6.45 % | 3.86 % |
| 1.5 N push max \|x\| | 13.04 cm | 13.71 cm |
| 0.4 N slope offset @5 s | 39.2 cm | 20.7 cm |
| PM at 10 ms | 40.7° | **47.4°** |

The result is written to `matlab/logs/tuned_pid.txt` as a ready-to-paste
`#define` block. **Validate on hardware before trusting it** — the simulator
still assumes the sign chain is correct, 12 V, 10 ms delay and a measured
deadband of 170.

### Which optimiser, and why (measured on this machine)

The script has four back-ends; all share the *same* cost function and bounds and
optimise in log2 space, so the numbers below are directly comparable
(12 V, Td = 10 ms, 300-eval budget):

| method | toolbox required | J (start 41.58) | evals | wall time | kp / kd / kps / kpp |
|---|---|---|---|---|---|
| `'pattern'` (default) | **none** | **10.15** | 107 | 3.9 s | 149.0 / 10.30 / 12.0 / 1.44 |
| `'patternsearch'` | Global Optimization | 10.30 | 300 | 7.8 s | 162.5 / 9.24 / 12.0 / 1.40 |
| `'fminsearch'` | none (base MATLAB) | 29.10 | 300 | 11.4 s | 235.4 / 9.12 / 11.4 / 0.71 |
| `'fmincon'` (sqp) | Optimization | 37.73 | 166 | 7.9 s | *no move from x0* |

Why the derivative-based SQP fails here: the cost is **piecewise/penalised**
(is-fall = 100, per-case cap 25, PWM-saturation bonus, plus the 250/50/10 Hz
sampled switching), so finite-difference gradients are meaningless and it exits
at the start point. Nelder-Mead (`fminsearch`) wanders for 300 evals and is still
30 % above. MATLAB's own `patternsearch` works well (10.30) but stopped in a
slightly worse local basin than the coordinate search (10.15).

Both derivative-free results are valid, hardware-plausible sets — they differ by
1.5 % in cost and by ~9 % in `kp_pitch`. Pick per preference:

```matlab
autotune_pid                    % coordinate pattern search (default, portable)
autotune_pid('patternsearch')   % Global Optimization Toolbox
autotune_pid('fmincon', 400, 600)
autotune_pid('fminsearch', 300, 240)
```

## Visualising the tuned set (`vis_tuned.m`)

`vis_tuned` reads `logs/tuned_pid.txt` (it does **not** hard-code the gains),
re-runs four cases with those gains and draws them side by side with the current
firmware values:

```matlab
autotune_pid        % produce logs/tuned_pid.txt
vis_tuned           % regenerate + draw
```

Outputs (git-ignored, `matlab/logs/`):

| file | content |
|---|---|
| `tuned_strobe.png` | 9-panel strobe of the 0.30 m step (wheel roll, CoG path) + pitch/speed |
| `tuned_compare.png` | grey = firmware today, colour = tuned: step pitch, step x, speed step, 1.5 N push (with the ±15 cm band) |
| `tuned_anim.gif` | side-view animation of the tuned 0.30 m step (50 ms/frame) |
| `balance_model_tuned.mat` | the simulations behind the figures |

Console summary of the run behind those figures:

```
 step : rise 2.55 -> 0.75 s, overshoot 0.0 -> 3.2 %, err -1.55 -> 0.14 cm
 speed: overshoot 6.8 -> 2.7 %, v_end 0.394 -> 0.404 m/s
 stand: max|x| 2.81 -> 3.12 cm, settle 0.34 -> 0.43 s (no noise)
 push : max|x| 12.84 -> 13.74 cm, x@4s 1.20 -> 0.10 cm (with noise)
```

Trade-off worth knowing: the tuned set is **much faster** (step rise 3.4×,
recovery 12×) at the cost of a **larger peak lean** during the step (8.0° vs 4.2°,
i.e. it leans harder to accelerate) and 3.2 % overshoot. If the car looks too
aggressive on the bench, lower `kp_pos` to ~1.2 or weight overshoot higher in the
cost function and re-tune.





