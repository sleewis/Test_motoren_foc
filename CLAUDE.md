# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Test sketch for two GM4108H-120T BLDC motors on a MKS-ESP32FOC V2.0 board.  
Goal: verify motors and encoders, and tune FOC current-control gains via Serial.

## Build & Upload

```bash
# Compile
arduino-cli compile --fqbn esp32:esp32:esp32 Test_motoren_foc/

# Upload (replace COMx with the actual port)
arduino-cli upload --fqbn esp32:esp32:esp32 --port COMx Test_motoren_foc/

# Serial monitor
arduino-cli monitor --port COMx --config baudrate=115200
```

## Architecture

### Dual-core FreeRTOS (Test_motoren_foc.ino)

| Task | Core | Rate | Responsibility |
|------|------|------|---------------|
| `fastTask` | 1 | 500 Hz (2 ms) | Motor control loop — reads commands, calls `loopOpenLoop()` or `loopFOC()`, writes telemetry |
| `slowTask` | 0 | ~10 Hz | Serial command parsing + telemetry print |

Both tasks share `gState` (a `SharedState` struct) protected by `xMutex`.  
`fastTask` never blocks on the mutex (uses `xSemaphoreTake(xMutex, 0)`); if it can't acquire it, it skips that cycle to keep timing tight.

### Motor class (Motor.h / Motor.cpp)

One `Motor` instance per BLDC motor. Key methods:

- `begin()` — configures PWM (20 kHz, 10-bit) and enable pin.
- `alignStart()` / `alignFinish()` — two-step alignment; both motors are started in parallel in `fastTask` so the 2 s wait happens only once.
- `loopOpenLoop(voltage)` — sinusoidal commutation with no current feedback; useful for direction/encoder sanity checks.
- `loopFOC(iqTarget)` — full FOC chain: Clarke → Park → dual PI (Id=0, Iq=iqTarget) → inverse Park → inverse Clarke → PWM.
- `commonUpdate()` — shared first step for both loop methods: read AS5600 encoder, compute velocity (low-pass filtered), read phase currents, overcurrent check.
- `setFocGains(kp, ki)` — live-tunable; resets integrators on change.

### Signal chain (FOC mode)

```
Phase currents (ADC) ──Clarke──> (Iα,Iβ) ──Park(θ_e)──> (Id,Iq)
                                                              │
Target: Id=0, Iq=iqTarget ──────────────────> PI controllers
                                                              │
                                          (Vd,Vq) ──inv. Park──> (Vα,Vβ)
                                                                      │
                                                        inv. Clarke ──> PWM (U,V,W)
```

### Pin assignments

| Signal | Motor 1 | Motor 2 |
|--------|---------|---------|
| PWM U/V/W | 33, 32, 25 | 26, 27, 14 |
| Enable | 12 | 12 |
| Current IA/IB | 39, 36 | 34, 35 |
| I²C (AS5600) | Wire0 GPIO 19/18 | Wire1 GPIO 23/5 |

Motor 2 is mounted mirrored — sign is inverted in `fastTask` (`loopFOC(-cmd2)`).

### Key constants (Motor.h)

| Constant | Value | Meaning |
|----------|-------|---------|
| `VOLTAGE_LIMIT` | 7.0 V | Max phase voltage (≈ Vbus/2 for 4S LiPo) |
| `ALIGN_VOLTAGE` | 3.0 V | Voltage applied during alignment |
| `MOTOR_DT` | 0.002 s | Fixed timestep for PI integrators |
| `MAX_CURRENT_A` | 6.0 A | Overcurrent trip threshold |
| `VELOCITY_ALPHA` | 0.80 | Low-pass coefficient for velocity |
| `_focKp` / `_focKi` | 0.5 / 50.0 | Default FOC PI gains |

## Serial Commands (115200 baud)

```
m1 <V/A>   set motor 1 setpoint
m2 <V/A>   set motor 2 setpoint
ol         switch to open-loop (voltage [V])
foc        switch to FOC (torque current [A])
kp <val>   set FOC Kp [V/A] for both motors
ki <val>   set FOC Ki [V/(A·s)] for both motors
stop       set both setpoints to 0
info       print current settings
```
