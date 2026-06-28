---
name: pid-control
title: PID Control — Learn, Simulate, Interview-Ready
description: Hands-on PID control training using a Python simulation toolkit on Raspberry Pi. Covers P/I/D theory, manual tuning, anti-windup, Ziegler-Nichols, and interview simulation. No hardware needed.
category: embedded
triggers:
  - user asks to learn PID / control theory / feedback control
  - user wants to practice PID tuning
  - embedded interview prep for control systems
  - ROS2 motor control / drone / balancing bot PID topics
  - how to learn BLDC / FOC / field-oriented control
  - brushless DC motor driver development / gimbal motor control
  - sensorless or sensored FOC (encoder-based)
  - Clarke / Park transform questions in interviews
---

## Overview

This skill uses the script at `/home/pi/pid_sim.py` to provide hands-on PID training. The tool has 4 modes:

| Mode | What it does |
|------|-------------|
| **1. Guided Tuning Walkthrough** | Step-by-step Kp→Ki→Kd tuning with charts |
| **2. Anti-Windup Comparison** | Standard vs clamping vs incremental PID |
| **3. Ziegler-Nichols Auto-Tune** | Classic 1942 method, auto-finds Ku/Tu |
| **4. Interview Simulation** | Practice tuning unknown plants interactively |

## Quick Start

### Generate All Charts (passive learning)

```bash
/home/pi/pid_env/bin/python -c "
import matplotlib
matplotlib.use('Agg')
import sys; sys.path.insert(0, '/home/pi')
import importlib.util
spec = importlib.util.spec_from_file_location('pid', '/home/pi/pid_sim.py')
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
mod.mode_tuning_walkthrough()
mod.mode_antiwindup()
mod.mode_ziegler_nichols()
"
```

Output: `/home/pi/pid_tuning_walkthrough.png`, `pid_antiwindup.png`, `pid_ziegler_nichols.png`

### Interactive Tuning via Chat (Telegram/CLI)

**⚠️ Mode 4 (interview_sim) uses `input()` — does NOT work in chat.** Use the helper script instead:

```bash
# List available plants
/home/pi/pid_env/bin/python /home/pi/pid_tune.py list

# Tune a plant with specific gains
/home/pi/pid_env/bin/python /home/pi/pid_tune.py 0 5 5 0
```

**Chat-based workflow** (telegram/cli without PTY):
1. Show user the plant + default result (Kp=2 Ki=0 Kd=0)
2. Let them pick a strategy: raise Kp first, or add Ki, or add Kd
3. Run `pid-tune.py <idx> <Kp> <Ki> <Kd>`, show metrics + chart
4. Repeat until PASS, then move to next plant

The helper script is also at `scripts/pid-tune.py` within this skill (copy to /home/pi if needed).

## Environment

- **Python venv**: `/home/pi/pid_env/` (has matplotlib + numpy)
- **Script**: `/home/pi/pid_sim.py`
- **Output directory**: `/home/pi/` (charts saved as PNG)
- **matplotlib**: must use `Agg` backend (headless Pi), already configured

## Worked Example: Fast Motor (tau=0.1)

From a real session — Plant 0 (first-order, tau=0.1, gain=1.0):

| Step | Kp | Ki | Kd | Overshoot | Settle | Steady Err | Result |
|------|----|----|----|-----------|--------|------------|--------|
| 1 | 2 | 0 | 0 | 0.0 | N/A | +26.68 | P-only, large steady error |
| 2 | 5 | 0 | 0 | 0.0 | N/A | +13.34 | Bigger Kp halves the gap |
| 3 | 5 | 1 | 0 | 0.0 | N/A | +6.42 | Ki starts pulling it in |
| 4 | 5 | 3 | 0 | 0.0 | 2.508s | +1.46 | Almost there |
| 5 | 5 | 5 | 0 | **0.0** | **1.527s** | **+0.35** | **PASS 🎉** |

**Pattern:** first-order plants need PI (P-only can't kill steady error). The fast motor (small tau) responds instantly — no Kd needed since there's no natural overshoot to damp.

## Moving to Hardware: STM32 PID Motor Control

This skill also includes a register-level STM32F103C8 project that mirrors the Python simulation on real hardware. See `references/stm32-pid-motor.md` for:

- **Project path**: `/home/pi/stm32-pid-motor/` (Makefile + main.c + linker.ld)
- **Build**: `make && make flash` (st-flash to 0x08000000)
- **Pinout**: PA0=PWM, PB0/PB1=Direction, PB6/PB7=Encoder (TIM4 encoder mode)
- **Behaviour**: Auto-cycles P-only → PI → PID every ~5 seconds, debug via UART 115200
- **Control loop**: 50Hz, float PID with clamping anti-windup

The hardware project demonstrates the exact same tuning concepts from the simulation — just with a real spinning motor.

## FOC (Field-Oriented Control) — Next Step After PID

Once you have PID on a DC brushed motor down, **FOC for BLDC motors** is the natural progression. BLDC motors are used in drones, gimbals, robots, and EVs — and are the core ask at high-paying embedded motor-control jobs (Insta360 gimbal, rock扫地机器人, etc.).

### Learning Path (see `references/foc-learning-path.md` for full detail)

| Phase | What | Time | Hardware cost |
|-------|------|------|---------------|
| **1. Python Simulation** | Clarke → Park → SVPWM → dual-loop PI (speed + current) | 1-2 weeks | ¥0 (Pi + existing venv) |
| **2. Hardware Integration** | 3-phase PWM + ADC current sensing + encoder rotor tracking | 3-4 weeks | ~¥50-80 (BLDC driver board + gimbal motor) |
| **3. Optimization** | Sensorless SMO observer, MTPA, field weakening | stretch | Same hardware |

The Python sim (`sim_foc.py`) lives in the same project directory as the STM32 PID code (`/home/pi/stm32-pid-motor/sim_foc.py`). It implements the full FOC chain and plots speed/torque/current/voltage traces. Run it immediately:

```bash
source /home/pi/pid_env/bin/activate
cd /home/pi/stm32-pid-motor
python3 sim_foc.py
# Output: foc_sim_results.png
```

### Interview-Ready FOC Talking Points

From the simulation alone you can credibly discuss:

- **Clarke transform**: Ia,Ib,Ic → Iα,Iβ — projects 3-phase into 2 orthogonal axes
- **Park transform**: Iα,Iβ → Id,Iq — rotates into the rotor frame so I_q produces torque and I_d produces flux (independent control)
- **Id=0 control**: Surface-mount PMSM has max torque per amp when Id=0
- **SVPWM**: Space Vector PWM uses the full DC bus (~15% more voltage than SPWM via zero-sequence injection)
- **Dual-loop architecture**: Inner current loop (10kHz, faster) + outer speed loop (1kHz)
- **Back-EMF saturation**: At high speeds, V_bemf eats into the voltage budget, limiting current controller authority — this is why field weakening (negative Id) exists
- **PI tuning for current loops**: Different from speed loops — current loops need much higher bandwidth (Kp ~5-10, Ki ~100-500 vs speed Kp/Ki ~0.5/5)

### Hardware Upgrade from STM32 PID

Your existing `/home/pi/stm32-pid-motor/` project plus:
- **MG310 gimbal motor** (with encoder, ~¥30-40)
- **L6234 or DRV8313** 3-phase driver board (~¥30-50), OR **SimpleFOC Mini** (~¥50)
- Wire up: 3 PWM outputs (TIM2 CH1/CH2/CH3), 2 ADC channels (current sensing), encoder pins

Minimal code changes from your existing `main.c`:
1. Single PWM → 3-phase complementary PWM with dead-time
2. Single ADC → dual simultaneous sampling (triggered by PWM center alignment)
3. One PI → two PIs (Id loop + Iq loop)
4. Clarke/Park transforms between ADC readings and PI inputs

## Key Concepts

### P (Proportional)
- Output = Kp × error
- Fast response, but **steady-state error** remains
- Too high → oscillation; too low → sluggish

### I (Integral)
- Output = Ki × ∫error dt
- **Eliminates steady-state error**
- Too high → overshoot + oscillation (integral windup)

### D (Derivative)
- Output = Kd × d(error)/dt
- **Predicts and dampens overshoot**
- Too high → noisy/jittery, slower response

### Manual Tuning Method (interview-ready)
1. Ki=Kd=0, increase Kp until sustained oscillation → record Ku, Tu
2. Add Ki to kill steady-state error
3. Add Kd to damp overshoot
4. Iterate: Kp↑=faster↔more OS, Ki↑=kills error↔rings, Kd↑=smoother↔noise-sensitive

### Anti-Windup (critical for real systems)
When output saturates (e.g. PWM at 100%), the integral keeps accumulating. Fixes:
1. **Clamping** — freeze integral when saturated
2. **Back-calculation** — feed back (clipped - raw) to integral
3. **Incremental PID** — output delta, naturally bounded

### Ziegler-Nichols Closed-Loop Method
```
P:   Kp = 0.50 × Ku
PI:  Kp = 0.45 × Ku,  Ki = 0.54 × Ku / Tu
PID: Kp = 0.60 × Ku,  Ki = 1.20 × Ku / Tu,  Kd = 0.075 × Ku × Tu
```
**Caveat**: ZN is aggressive. Reduce Kp by 10-20% for margin in practice.

## Plant Models

| Plant | Behavior | Real-world analog |
|-------|----------|-------------------|
| **FirstOrderPlant** | Asymptotic approach, no natural overshoot | DC motor speed, heater, RC circuit |
| **SecondOrderPlant** | Underdamped — overshoots naturally | Drone attitude, servo, suspension |
| **IntegratingPlant** | Linear accumulation, no self-regulation | Liquid level, position without friction |

## Linked Files

- `references/interview-questions.md` — common PID interview Q&A and tuning cheat sheet
- `references/stm32-pid-motor.md` — STM32F103C8 hardware PID project (wiring, build, code reference)
- `references/foc-learning-path.md` — FOC theory, Python simulation walkthrough, hardware upgrade path from PID to BLDC, interview Q&A
- `scripts/pid-tune.py` — chat-friendly tuning helper (no interactive prompts)

## Pitfalls

| Pitfall | Why | Fix |
|---------|-----|-----|
| **Mode 4 hangs in chat** | Uses `input()` — no PTY in Telegram | Use `scripts/pid-tune.py` instead (all args from CLI) |
| **Charts have garbled Chinese** | Pi headless font = DejaVu Sans (no CJK) | Use English labels in `pid_sim.py` |
| **Random noise between runs** | `simulate()` adds `np.random.normal(0, noise_std)` | Noise seed differs each run — normal, just re-run |
| **First-order plant won't oscillate** | Ziegler-Nichols requires sustained oscillation | Use SecondOrderPlant for ZN demo; for 1st-order just manual-tune PI |
| **Integral won't converge** | Anti-windup clamps I when output saturates | Lower Kp so P term doesn't saturate alone, or increase Ki
