# SimpleFOC BLDC Motor Control for STM32 Blue Pill

Field-oriented control (FOC) firmware for BLDC motors using SimpleFOC library on STM32 Blue Pill + SimpleFOC Mini driver board.

## Current Firmware Features

1. Open‑loop (sensorless) velocity control using SimpleFOC `BLDCMotor` + `BLDCDriver3PWM`.
2. Automatic motor pole pair estimation sweep (encoder required).
3. Encoder wiring / signal integrity test streaming angle & velocity.
4. Closed‑loop FOC velocity control (encoder based) with PID + LPF tuning interface.
5. 3‑phase PWM on TIM1 pins: PA8 / PA9 / PA10 (high‑frequency SinePWM).
6. Driver enable (EN / nSLEEP) on PB12 (low during init for safety).
7. Debug serial on `Serial2` (USART2: PA2=TX, PA3=RX) at 115200 baud.
8. Interactive CLI commands for each mode (velocity set, streaming toggle, zero reference, etc.).
9. Streaming telemetry over `Serial2` as CSV (tgt,vel) for use with plotting tools like SerialPlot.

## Hardware Summary

| Component                | Notes |
|--------------------------|-------|
| MCU Board                | STM32 Blue Pill (now configured as F103C8, 64KB flash) |
| Motor Driver             | SimpleFOC Mini v1.0 (DRV8313) |
| Motor                    | Sensorless BLDC (example: 24V, 50KV) |
| Motor Supply (VM)        | 12V (adjust per motor & driver limits) |
| Logic Supply             | 3.3V (shared between Blue Pill & driver logic) |
| Programmer               | ST-Link (SWD) |
| Serial Debug Adapter     | USB‑TTL (3.3V) on PA2/PA3 |
| Optional Encoder         | MT6701 (ABZ incremental mode) |

> If your physical MCU is the smaller C6 (32KB flash), the full SimpleFOC library may not fit. The project is set to `bluepill_f103c8` for extra flash headroom.

## Wiring Guide

Essential connections (grounds common):

| Blue Pill Pin | SimpleFOC Mini | Purpose |
|---------------|----------------|---------|
| PA8           | IN1 / PWM1     | Phase A PWM |
| PA9           | IN2 / PWM2     | Phase B PWM |
| PA10          | IN3 / PWM3     | Phase C PWM |
| PB12          | EN / nSLEEP    | Driver enable (active HIGH) |
| 3.3V          | 3.3V / VCC     | Logic power (if board expects it) |
| GND           | GND            | Common ground |
| PA2 (TX2)     | USB‑TTL RX     | Debug serial out |
| PA3 (RX2)     | USB‑TTL TX     | Debug serial in |
| VM (driver)   | 12V supply +   | Motor power (separate from 3.3V) |

Motor phases → OUT_A / OUT_B / OUT_C (order arbitrary; swap any two to invert direction if needed).

Optional (not used yet):
- nFAULT → free GPIO with pull‑up for diagnostics.
- Add a sensor (magnetic / encoder) later to move from open-loop to full FOC.

### MT6701 Encoder (ABZ Mode) Wiring

| MT6701 | Blue Pill | Notes |
|--------|-----------|-------|
| VCC    | 3.3V      | Power (keep leads short, decouple with 100nF) |
| GND    | GND       | Common ground |
| A      | PA0       | TIM2 CH1 (hardware quadrature capable) |
| B      | PA1       | TIM2 CH2 |
| Z (Index) | PB4 (optional) | Not used by SimpleFOC core yet in this test; can observe manually |

Outputs are push‑pull by default → no pull‑ups required. If you configured open‑drain mode, add ~10k pull‑ups to 3.3V on A/B/Z.

Cable tips: twist (A,GND) and (B,GND) pairs or use shield if noisy. Keep distance from high current phase wires.

## Firmware Mode Selection

Switch modes by editing the macros at the very top of `src/main.cpp` (exactly one must be `1`):

```c
#define MODE_OPEN_LOOP 0            // sensorless open-loop velocity demo
#define MODE_POLEPAIR_TEST 0        // automatic estimation of motor pole pairs using encoder
#define MODE_ENCODER_TEST 0         // encoder wiring / angle+velocity streaming
#define MODE_VELOCITY_CLOSED_LOOP 1 // closed-loop FOC velocity control (encoder)
```

### Summary of Modes
| Mode | Purpose | Requires Encoder | Typical Use |
|------|---------|------------------|-------------|
| `MODE_OPEN_LOOP` | Simple sensorless spin & sanity test | No | Quick functional test |
| `MODE_POLEPAIR_TEST` | Estimate mechanical pole pairs | Yes | When motor pole pairs unknown |
| `MODE_ENCODER_TEST` | Validate wiring & CPR, observe angle/velocity | Yes | First bring-up of sensor |
| `MODE_VELOCITY_CLOSED_LOOP` | Full FOC velocity loop (PID) | Yes | Actual application / performance |

> After determining pole pairs and confirming encoder CPR/direction, use the closed‑loop mode for best torque, low‑speed smoothness, and efficiency.

## Open-Loop Behavior & Tuning

Open-loop (no sensor) approximates rotation and may:
- Cog or stall at very low speeds.
- Require a minimum target velocity to start.
- Produce less torque and efficiency than closed-loop FOC.

Key parameters in `src/main.cpp`:
| Parameter | Purpose | Default |
|-----------|---------|---------|
| `MOTOR_POLE_PAIRS` | Set to your motor's pole pairs (magnets/2) | 7 |
| `SUPPLY_VOLTAGE` | Motor supply (for voltage mapping) | 12.0 |
| `VOLTAGE_LIMIT` | Limits applied phase voltage | 4.0 |
| `target_velocity` | Initial open-loop speed (rad/s) | 10.0 |

Adjust `VOLTAGE_LIMIT` upward gradually if the motor fails to start (watch heat). Increase `target_velocity` for easier startup (e.g. 20–40 rad/s), then reduce.

Rough conversion: rpm ≈ rad/s * 60 / (2π)

### Serial Command Interface

Open a monitor at 115200 baud on `Serial2` (PA2/PA3 wiring). Send:

```
v 30      # set target velocity to 30 rad/s (~286 rpm)
v -15     # reverse direction
```

You will see periodic status lines:
```
vel target(rad/s): 10.00  est mech rpm ~95.5
```

## Build & Upload

Prerequisites:
- PlatformIO (CLI or IDE)
- ST-Link connected (SWDIO, SWCLK, GND, 3.3V ref, NRST optional)

Commands:
```bash
# Build
platformio run

# Upload
platformio run --target upload

# Serial monitor (adjust port)
platformio device monitor
```

## `platformio.ini` Notes

Important entries:
```
board = bluepill_f103c8
lib_deps = askuric/Simple FOC@^2.3.5
build_flags = -Os -flto
```
Optimization flags help keep the binary within flash limits.

## Pole Pair Detection Mode (Automatic Estimation)

Use this mode to automatically estimate the motor's mechanical pole pairs using a connected incremental encoder. The firmware sweeps a known electrical angle while applying a small q‑axis voltage and measures the resulting mechanical span from the encoder. Estimated pole pairs ≈ (electrical angle swept) / (mechanical angle moved).

### When to Use
Run this once (or a couple of times) when you don't know the motor's pole pair count, before setting `MOTOR_POLE_PAIRS` for open‑loop tuning or moving to closed‑loop FOC. Requires the encoder already wired (A=PA0, B=PA1) and functioning.

### Enabling the Mode
At the top of `src/main.cpp` set only this macro to 1:
```c++
#define MODE_POLEPAIR_TEST 1
#define MODE_OPEN_LOOP 0
#define MODE_ENCODER_TEST 0
```

### Wiring Prerequisites
- Encoder A → PA0, B → PA1 (quadrature)
- Driver EN → PB12, phase PWMs → PA8/PA9/PA10 (same as other modes)
- Motor phases connected; rotor free to rotate (not mechanically loaded)

### Key Parameters (in code)
| Parameter | Meaning | Default |
|-----------|---------|---------|
| `DETECT_VOLTAGE` | Sine voltage magnitude applied during sweep | 2.5 V |
| `MAX_ELECTRICAL_REV` | Max electrical revolutions to sweep before stopping | 14 |
| `STEP_E_ANGLE` | Electrical angle step size (rad) | 0.02 |
| `SETTLE_US` | Microseconds to wait after each step | 2000 |

Increase `DETECT_VOLTAGE` carefully (small increments) if the rotor does not move or stalls (watch temperature & current). Lower it if the motion is abrupt.

### Running & Output
1. Build & flash.
2. Open Serial2 (115200). You'll see:
	- Intro & safety notes
	- Periodic progress lines: `Progress: e_rev=<electrical_revs> mech_span(rad)=<accumulated_mech_angle>` every ~0.5 s
3. The test stops early once enough mechanical span (> ~1.2 revs) is collected, or after the configured electrical span.
4. Final lines:
```
Electrical span used (rad): <value>
Mechanical span (rad): <value>
Estimated pole pairs: <float>
Rounded pole pairs: <integer>
```

Use the rounded integer as your `MOTOR_POLE_PAIRS` value elsewhere in the project. If consecutive runs disagree by more than ±1, increase sweep span (`MAX_ELECTRICAL_REV`) or improve mechanical freedom / increase `DETECT_VOLTAGE` slightly.

### Failure / Edge Cases
| Message | Cause | Action |
|---------|-------|--------|
| `[FAIL] Mechanical span too small` | Rotor hardly moved | Raise `DETECT_VOLTAGE`, ensure no load |
| Very noisy estimate | Mechanical backlash / stiction | Increase voltage slightly; ensure shaft turns smoothly |
| Estimate off by exactly 2× | Misinterpreting pole count (remember pole pairs = magnet count / 2) | Re-run and confirm; count magnets if possible |

### After Detection
1. Copy the rounded pole pair number.
2. Switch back to open‑loop or encoder test mode by toggling macros.
3. Set `MOTOR_POLE_PAIRS` (open‑loop) or use it when configuring closed‑loop FOC.

> Tip: Keep a note of the final pole pair count in this README (e.g. "My motor: 7 pole pairs").

## Encoder Test Mode (MT6701 ABZ)

Before enabling closed-loop FOC, you can verify encoder wiring with a dedicated test build included in `src/main.cpp`.

1. Set the macro at the top of `main.cpp`:
	```c++
	#define MODE_ENCODER_TEST 1   // enable encoder test
	```
	Set back to `0` to return to the motor open-loop demo.

2. Adjust the placeholder:
	```c++
	static const uint32_t ENCODER_CPR = 8192; // set to your configured resolution
	```
	MT6701 common incremental resolutions: 1024, 2048, 4096, 8192 CPR (after quadrature). Use the actual quadrature edge count (i.e. 4 * PPR).

3. Flash the firmware and open the Serial2 monitor (PA2/PA3) at 115200 baud.

4. Output columns (tab separated):
	```
	angle_rad    velocity_rad_s    rawApproxCounts
	```

5. Commands:
	- `z` : capture current angle as zero (software offset)
	- `r` : toggle fast streaming (~500 Hz) / slow (20 Hz)

6. Validation steps:
	- Rotate shaft slowly forward → angle increases smoothly.
	- Reverse direction → angle decreases (or invert later using `encoder.direction = Direction::CCW;`).
	- One full revolution ≈ `ENCODER_CPR` change in `rawApproxCounts` (after zeroing).
	- Velocity near zero while stationary; proportional to speed when moving.

If angle direction is inverted you can either swap A/B lines or (preferred) set direction in code when moving to closed-loop.

> This test mode is only for signal validation. For real operation switch to `MODE_VELOCITY_CLOSED_LOOP` once CPR & direction are confirmed.

## Closed-Loop Velocity FOC Mode

Enabled when `MODE_VELOCITY_CLOSED_LOOP` = 1. Provides field‑oriented control with an incremental encoder (AB). Adjust these constants near the mode block if needed:

| Constant | Meaning | Example |
|----------|---------|---------|
| `MOTOR_POLE_PAIRS` | Mechanical pole pairs (magnets/2) | 7 |
| `ENCODER_CPR` | Quadrature counts per mechanical revolution | 8192 |
| `SUPPLY_VOLTAGE` | Driver supply (for voltage mapping) | 12.0 V |
| `VOLTAGE_LIMIT` | Max q‑axis voltage demand (limits current/heat) | 6.0 V |
| `target_velocity` | Initial commanded velocity (rad/s) | 10.0 |

### CLI Commands (Serial2 115200)
```
v <rad_s>   set target velocity (ex: v 25)
s           stop (target = 0)
p           one-shot status line
r           toggle 5 Hz telemetry streaming
z           capture current mechanical angle as zero reference
```

Status line fields (streaming):

- CSV header: tgt,vel (printed once when streaming starts)
- CSV row: tgt,vel

Example:

```
10.000,9.872
10.000,9.913
```

Tip: I removed the bundled `foc_plot.py`; use SerialPlot (https://hackaday.io/project/166357-serialplot) or similar to visualize the `tgt` and `vel` columns in real time.

### PID & Filter Tuning
Parameters (inside the closed-loop section in code):
| Symbol | Typical Start | Effect |
|--------|---------------|--------|
| `PID_velocity.P` | 0.4–1.0 | Proportional response (stiffness) |
| `PID_velocity.I` | 5–25 | Removes steady-state error; too high → oscillation / windup |
| `PID_velocity.D` | 0 | Derivative damping (often not needed) |
| `PID_velocity.output_ramp` | 500–2000 | Limits rate of voltage command change |
| `PID_velocity.limit` | = `VOLTAGE_LIMIT` | Safety clamp of controller output |
| `LPF_velocity.Tf` | 0.02–0.08 s | Larger smooths noise, adds lag |

#### PID & Velocity parameter reference

Below is a concise reference table for the closed-loop velocity controller parameters and their recommended ranges.

| Parameter           | What it controls        | Unit         | Typical Range | Extreme Limits | Notes                                       |
| ------------------- | ----------------------- | ------------ | ------------- | -------------- | ------------------------------------------- |
| **P**               | Proportional gain       | V/(rad/s)    | 0.2 → 0.8     | 0 → ~2        | Higher = faster, but oscillates if too high |
| **I**               | Integral gain           | V/(rad/s·s)  | 1 → 15        | 0 → ~100      | Too high → windup, sluggish overshoot       |
| **D**               | Derivative gain         | V·s/rad      | 0 → 0.01      | 0 → 1          | Avoid unless overshoot is persistent        |
| **output_ramp**    | Voltage slope limit     | rad/s² equiv | 400 → 700     | 0 → 10000      | Lower = smoother, higher = snappier         |
| **limit**           | Max PID output voltage  | V            | 6 → 9         | 0 → supply     | Clamp for safety                            |
| **Tf**              | Velocity LPF time const | s            | 0.02 → 0.05   | 0 → 0.2        | Lower = faster but noisier                  |
| **velocity_limit** | Max target speed        | rad/s        | 40 → 60       | 0 → ∞          | Set based on motor KV × V                   |


Tuning procedure (quick):
1. Start with moderate `P` (0.5). `I=10`, `D=0`.
2. Increase `P` until you see slight overshoot / buzz, then back off ~20%.
3. Increase `I` so steady velocity error < 2–3% (watch for drift while stopped).
4. Add `LPF_velocity.Tf` if velocity readout is noisy → raise gradually (0.02 → 0.05 → 0.08).
5. Only add derivative if oscillations persist and can’t be fixed by reducing `P` / `I`.

If the motor spins the wrong direction compared to command, you can:
* Swap any two motor phase wires (electrically inverts torque), or
* In code, invert sensor direction before `initFOC()` (e.g. `encoder.direction = Direction::CCW;`).

### Common Closed-Loop Issues
| Symptom | Likely Cause | Fix |
|---------|--------------|-----|
| Hunts / oscillates | P or I too high | Reduce P first, then I |
| Slow to reach target | P too low or ramp limit small | Raise P or `output_ramp` |
| Drifts when target=0 | I windup / bias | Lower I, add small deadband via code, or zero offset |
| High pitched noise | PWM frequency resonance | Accept or adjust mechanical mounting; driver set to 25 kHz already |
| Jumps at enable | Improper alignment / wrong pole pairs | Re-run pole pair detection, verify CPR |

### Logging Improvements (Optional Ideas)
You can add a higher‑rate binary or CSV stream or simple min/max tracking inside the 5 Hz telemetry block. For deeper analysis, temporarily raise streaming frequency and capture with a logic/USB serial tool.

---

## Safety & Power Tips

1. Always connect GND first; disconnect VM last.
2. Keep fingers and loose items clear of the spinning motor.
3. If the motor vibrates without spinning: increase `target_velocity` or `VOLTAGE_LIMIT` slightly.
4. If it gets hot quickly: lower `VOLTAGE_LIMIT` and re-test.
5. For better performance, add a position sensor (AS5600, AS5048A, hall, or encoder) and switch to full FOC (`motion_control = velocity` or `angle`).

## Notes on Further Extensions

Potential next steps (not yet implemented in this repo):
1. Angle (position) control mode with soft motion profiling.
2. Torque / current (Iq) control (needs current sensing hardware & driver support).
3. Automatic PID auto‑tune utility & runtime parameter save to flash.
4. Index (Z) pulse handling for sub‑count absolute referencing.
5. Lightweight binary telemetry protocol for higher sample rates.

## Troubleshooting Quick List

| Symptom | Possible Cause | Fix |
|---------|----------------|-----|
| No serial output | Wrong pins / port | Use PA2/PA3 @ 115200; select correct /dev/tty* |
| Motor twitches only | Too low voltage / speed | Raise `VOLTAGE_LIMIT` or `target_velocity` |
| Upload fails (OpenOCD) | ST-Link connection | Re-seat SWD wires; try slower adapter speed |
| Direction opposite | Phase order | Swap any two motor phase wires |
| Overcurrent / heat | Excess voltage limit | Lower `VOLTAGE_LIMIT` |