# STM32 PID Motor Speed Control — Hardware Reference

## Overview

A register-level STM32F103C8 (Blue Pill) project that runs PID speed control on a DC motor with encoder. Built to mirror the Python simulation — user sees P-only vs PI vs PID in real hardware.

## Project Location

`/home/pi/stm32-pid-motor/`

## Pin Mapping

| STM32 Pin | Function | Connects To |
|-----------|----------|-------------|
| PA0 (TIM2_CH1) | PWM output (1kHz) | L298N ENA (enable/speed) |
| PB0 | Direction IN1 | L298N IN1 |
| PB1 | Direction IN2 | L298N IN2 |
| PB6 (TIM4_CH1) | Encoder Channel A | Encoder OUT1 |
| PB7 (TIM4_CH2) | Encoder Channel B | Encoder OUT2 |
| PA9 (USART1_TX) | Debug output (115200) | USB-UART RX |
| PA10 (USART1_RX) | Debug input | USB-UART TX |

## Build & Flash

```bash
cd /home/pi/stm32-pid-motor/
make          # compiles pid-motor.elf + pid-motor.bin
make flash    # st-flash write to 0x08000000
```

## Program Behaviour

Auto-cycles through 3 phases, each ~5 seconds:

| Phase | Kp | Ki | Kd | Effect |
|-------|----|----|----|--------|
| P-only | 2 | 1 | 0 | Fast response, steady-state error visible |
| PI | 2 | 3 | 0 | Error removed, possible overshoot |
| PID | 3 | 4 | 0.3 | Smooth, fast, minimal overshoot |
| (reset) | — | — | — | Returns to P-only after ~15s |

Serial output at 115200 baud, 5Hz update rate:
```
SP=500 RPM=312 PWM=623 err=188 I=15
SP=500 RPM=498 PWM=502 err=2 I=32
SP=500 RPM=502 PWM=499 err=-2 I=33
```

Columns: Setpoint, Measured RPM, PWM duty, Error, Integral accumulator.

## Code Structure (single file: `main.c`)

- **Vector table** — standard STM32F103, TIM4 ISR at position 30
- **UART** — register-level, putc/puts/putint for debug
- **PWM init** — TIM2, PSC=71, ARR=999 → 1kHz PWM, 0.1% resolution
- **Encoder init** — TIM4 encoder mode 3 (both edges), with overflow ISR for 16-bit wrapping
- **PID** — float-based, clamping anti-windup, configurable limits
- **SysTick** — 1ms tick used as control loop timebase
- **Control loop** — 50Hz (20ms period), PID update → PWM set → encoder read → serial print

## Key Constants

- PWM frequency: 1kHz (72MHz / 72 / 1000)
- Control loop: 50Hz
- Encoder range: 16-bit signed, handled in ISR
- PID dt: ~20ms (controlled by SysTick polling)

## Anti-Windup Implementation

In `pid_update()`:

```c
// Clamp output
if(output > pid->out_max) {
    output = pid->out_max;
    // Back off integral if it's making it worse
    if(i_term > 0) pid->integral -= error * dt;
}
if(output < pid->out_min) {
    output = pid->out_min;
    if(i_term < 0) pid->integral -= error * dt;
}
```

## Available Motor Control Functions

Defined but not used in auto-cycle — available for manual control:

- `motor_forward()` — PB0=1, PB1=0
- `motor_reverse()` — PB0=0, PB1=1
- `motor_brake()` — both high
- `motor_coast()` — both low

## Wiring Diagram (text)

```
STM32F103C8                    L298N                   DC Motor
┌──────────────┐              ┌────────┐             ┌────────┐
│ PA0 ──PWM──→│              │ ENA    │             │        │
│ PB0 ───────→│ IN1 ───────→│ IN1    ├──Motor──────→│  M1    │
│ PB1 ───────→│ IN2 ───────→│ IN2    │             │  +ENC  │
│ PB6 ←──CHA──│              │        │             │        │
│ PB7 ←──CHB──│              │ 5V/GND │             └────────┘
│ PA9 ──TX───→│ USB-UART     └────────┘
│ PA10←──RX───│
└──────────────┘

Power: L298N 12V → motor, 5V → STM32 (via regulator)
Ground: L298N GND ↔ STM32 GND (COMMON!)
```

## Tips

1. **Common ground** — L298N GND MUST connect to STM32 GND, or encoder readings will be garbage
2. **Encoder pull-ups** — Some encoders need external 10kΩ pull-up on PB6/PB7 if internal pull isn't enough
3. **Power sequencing** — Power L298N first, then STM32. The back-EMF from motor braking can brown-out the STM32 if shared rail is weak
4. **Tuning tip for first-order plants** — The DC motor is a first-order plant (no natural oscillation). Skip Ziegler-Nichols and do manual PI tuning: Kp to get response speed, Ki to kill steady error, Kd is optional (motor has inertia that acts as natural damping)
