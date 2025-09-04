# STM32 Blue Pill + SimpleFOC Mini (Sensorless Open-Loop) Project

Arduino/PlatformIO project driving a sensorless BLDC motor using a SimpleFOC Mini v1.0 (DRV8313 based) and an STM32 Blue Pill.

## Current Firmware Features

1. Open‑loop (sensorless) velocity control using SimpleFOC `BLDCMotor` + `BLDCDriver3PWM`.
2. 3‑phase PWM on TIM1 pins: PA8 / PA9 / PA10.
3. Driver enable (EN / nSLEEP) on PB12 (kept low during init, then enabled).
4. Debug serial on `Serial2` (USART2: PA2=TX, PA3=RX) at 115200 baud.
5. Runtime velocity command over serial: `v <rad_per_s>` (e.g. `v 25`).
6. Encoder wiring verification mode (high‑rate angle / velocity streaming).
7. Automatic motor pole pair estimation sweep (requires encoder) → prints estimated & rounded pole pairs.

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

## open-loop Behavior & Tuning

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

## Serial Command Interface

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

### Transition to Closed-Loop FOC
After encoder verification:
1. Set `MODE_ENCODER_TEST` back to 0.
2. Instantiate and init the encoder in the motor section.
3. Call `motor.linkSensor(&encoder);` then `motor.initFOC();`.
4. Switch controller type (e.g. `motor.controller = MotionControlType::velocity;`).
5. Use `loopFOC();` + `move(target);` in `loop()`.

Ask if you want a prepared closed-loop example.

## Pole Pair Detection Mode (Automatic Estimation)

Use this mode to automatically estimate the motor's mechanical pole pairs using a connected incremental encoder. The firmware sweeps a known electrical angle while applying a small q‑axis voltage and measures the resulting mechanical span from the encoder. Estimated pole pairs ≈ (electrical angle swept) / (mechanical angle moved).

### When to Use
Run this once (or a couple of times) when you don't know the motor's pole pair count, before setting `MOTOR_POLE_PAIRS` for open‑loop tuning or moving to closed‑loop FOC. Requires the encoder already wired (A=PA0, B=PA1) and functioning.

### Enabling the Mode
At the top of `src/main.cpp` set only this macro to 1:
```c++
#define MODE_POLEPAIR_TEST 1
#define MODE_ENCODER_TEST 0
#define MODE_OPEN_LOOP 0
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

---

## Safety & Power Tips

1. Always connect GND first; disconnect VM last.
2. Keep fingers and loose items clear of the spinning motor.
3. If the motor vibrates without spinning: increase `target_velocity` or `VOLTAGE_LIMIT` slightly.
4. If it gets hot quickly: lower `VOLTAGE_LIMIT` and re-test.
5. For better performance, add a position sensor (AS5600, AS5048A, hall, or encoder) and switch to full FOC (`motion_control = velocity` or `angle`).

## Moving to Closed-Loop Later

Add a magnetic or encoder sensor and replace:
```
motor.controller = MotionControlType::velocity_openloop;
```
with (example):
```
motor.controller = MotionControlType::velocity;
motor.sensor = &mySensor;
motor.initFOC();
```

## Troubleshooting Quick List

| Symptom | Possible Cause | Fix |
|---------|----------------|-----|
| No serial output | Wrong pins / port | Use PA2/PA3 @ 115200; select correct /dev/tty* |
| Motor twitches only | Too low voltage / speed | Raise `VOLTAGE_LIMIT` or `target_velocity` |
| Upload fails (OpenOCD) | ST-Link connection | Re-seat SWD wires; try slower adapter speed |
| Direction opposite | Phase order | Swap any two motor phase wires |
| Overcurrent / heat | Excess voltage limit | Lower `VOLTAGE_LIMIT` |

## License

Educational / personal use. Adjust as needed.

---
Feel free to extend this with sensors, current sensing, or multi-motor control.
