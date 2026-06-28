# FOC (Field-Oriented Control) Learning Path

From DC brushed PID → BLDC FOC. Companion to `sim_foc.py` at `/home/pi/stm32-pid-motor/sim_foc.py`.

---

## Phase 1: Python Simulation (¥0, 1-2 weeks)

### What to run

```bash
source /home/pi/pid_env/bin/activate
cd /home/pi/stm32-pid-motor
python3 sim_foc.py
# Output: foc_sim_results.png
```

### What the simulation does

```
电机模型(dq) → invPark → invClarke → SVPWM(va,vb,vc)
                                        ↑
                              PI电流环(Id/Iq PI 10kHz)
                                        ↑
                              Clarke→Park(从相电流测量Id/Iq)
                                        ↑
                              PI速度环(speed PI 1kHz)
                                        ↑
                              Target Speed (RPM)
```

| Block | What it does | Interview angle |
|-------|-------------|-----------------|
| **Clarke** | Ia,Ib,Ic → Iα,Iβ (3-to-2 phase, amplitude-preserving) | "Reduces 3 variables to 2 orthogonal axes for simpler control" |
| **Park** | Iα,Iβ → Id,Iq (rotates into rotor frame) | "Converts sinusoidal AC to DC-like signals — allows PI control of torque (Iq) and flux (Id) independently" |
| **SVPWM** | vα,vβ → 3-phase duty cycles (zero-sequence injection) | "Uses DC bus 15% more efficiently than SPWM; at Vdc=12V, max phase voltage = Vdc/√3 ≈ 6.93V vs SPWM's Vdc/2 = 6V" |
| **Id=0 control** | Forces d-axis current to zero | "For SPMSM, all torque comes from Iq; Id=0 is MTPA (maximum torque per ampere)" |
| **Dual-loop** | Inner current (10kHz) + outer speed (1kHz) | "Current loop bandwidth must be 5-10x faster than speed loop for stability; typical ratio = 10:1" |

### Debugging experiments to try

1. **Change speed PI gains** — `pi_speed = PIController(Kp=0.5, Ki=5.0, limit=5.0)`
   - Kp=2.0 → overshoot, oscillation around target
   - Ki=0 → steady-state error never eliminated (P-only speed loop)
   - Kp=0.1 → sluggish, takes >500ms to reach target

2. **Change current PI gains** — `pi_iq = PIController(Kp=5.0, Ki=200.0, limit=6.0)`
   - Kp=1.0 → Iq tracks reference slowly, audible whine from motor (simulated)
   - Ki too high → Iq oscillates around reference, torque ripple

3. **Change load torque** — `T_load = 0.005` → Iq jumps from ~0.15A to ~0.19A to compensate
   - Larger loads (0.01 Nm) → if Iq_ref hits the 5.0A limit → speed drops → integral windup in speed PI

4. **Change target speed** — `speed_rpm=2000` → may not reach it if back-EMF > Vdc/2

### Key insight discovered during tuning

**The Vdc/2 limit is a hard ceiling.** With Vdc=12V, the PI output clips at ±6V. Back-EMF = ωₑ × flux = (ωₘ × pole_pairs) × flux. If back-EMF exceeds 6V at the target speed, the motor physically cannot reach that RPM regardless of tuning. This is why:
- Higher Vdc → higher max speed (e.g. 24V bus for gimbals)
- Lower flux linkage → higher max speed but less torque per amp
- Field weakening (Id < 0) reduces effective flux → extends speed range at cost of efficiency

---

## Phase 2: Hardware Integration (¥50-80, 3-4 weeks)

### Shopping list

| Item | Price | Notes |
|------|-------|-------|
| MG310 / GB37-520 gimbal BLDC motor | ¥30-40 | Built-in encoder (ABI + index), ~7 pole pairs |
| L6234 driver board or DRV8313 eval | ¥30-50 | 3-phase, 2.8A, has current sense outputs |
| Or SimpleFOC Mini shield | ¥50 | All-in-one, runs STM32 or Arduino |
| Jumper wires + 12V power supply | ¥10-20 | Already probably have these |

### Wiring to existing STM32F103 (Blue Pill) project

```
STM32F103                      L6234 Driver
───────────                    ────────────
PA0 (TIM2_CH1) ──────────────→ IN1 (U-phase PWM)
PA1 (TIM2_CH2) ──────────────→ IN2 (V-phase PWM)
PA2 (TIM2_CH3) ──────────────→ IN3 (W-phase PWM)
PA4 (ADC1_IN4) ←────────────── IS1 (phase A current)
PA5 (ADC1_IN5) ←────────────── IS2 (phase B current)
PB6 (TIM4_CH1) ←────────────── Encoder A
PB7 (TIM4_CH2) ←────────────── Encoder B
                             L6234 ← 12V DC supply
                                        ↑
                                   BLDC Gimbal Motor
                                   (3 wires: U/V/W)
```

### Code changes from existing `main.c`

| Existing (DC brushed) | New (BLDC FOC) | Complexity |
|----------------------|----------------|------------|
| 1 PWM output (TIM2_CH1) | 3 complementary PWM (TIM2 CH1/CH2/CH3) + dead-time | ★★★ |
| H-bridge direction GPIOs | No direction GPIOs — invert PWM or swap phases | ★ |
| No current sense | ADC dual-sampling triggered by PWM center-aligned update | ★★★★ |
| 1 PI controller (speed) | 3 PI controllers (Id + Iq current, speed outer) | ★★ |
| No coordinate transforms | Clarke → Park (forward) + invPark → invClarke (reverse) | ★★★ |
| Encoder = speed only | Encoder = rotor electrical angle (absolute + interpolated) | ★★ |

### Key gotchas

1. **Dead-time insertion**: Never turn on both high-side and low-side MOSFETs in the same half-bridge. Insert 1-3µs deadtime between complementary signals. STM32 TIM BRK/BDTR register handles this.

2. **ADC sampling window**: Sample current in the middle of the PWM ON period when all phases are conducting. Use TIM2_TRGO to trigger ADC at center-aligned PWM counter = ARR/2.

3. **Rotor alignment**: Before FOC runs, do a DC alignment (apply Id=constant, Iq=0 for 500ms) — rotor locks to the d-axis. This establishes the encoder offset. Without this, theta_e is wrong and the motor jitters or runs away.

4. **Current sensor offset**: Even with no current flowing, ADC reads some offset. Calibrate by averaging 100+ samples at motor-off, subtract from live readings.

---

## Phase 3: Advanced Topics (Stretch)

### Sensorless FOC (Sliding Mode Observer)

Instead of an encoder, estimate rotor position from back-EMF. The SMO uses a switching function on the αβ current error:

```
z = -K × sign(i_hat - i_meas)   →   ê ≈ LPF(z)   →   θ_hat = atan2(ê_β, ê_α)
```

Problems: noisy at low speed (back-EMF too small), needs startup sequence (align → open-loop accelerate → switch to SMO).

### MTPA (Maximum Torque Per Ampere)

For interior PMSM (Ld ≠ Lq, e.g. Prius motor), Id=0 is NOT optimal. MTPA injects negative Id to exploit the reluctance torque:
```
Id_opt = -flux/(2*(Lq-Ld)) - sqrt(...)
```
Makes the motor more efficient at mid-to-high torque.

### Field Weakening

Above the base speed, Vq hits the voltage limit. Inject negative Id to cancel part of the PM flux, reducing back-EMF:
```
|V|² = Vd² + Vq² ≤ (Vdc/√3)²
Id_weak = -flux/Ld + (1/Ld)*sqrt((Vmax/ωₑ)² - (Lq·Iq)²)
```

---

## Interview Questions You Can Now Answer

| Question | What to say |
|----------|-------------|
| "What's the difference between Clarke and Park?" | Clarke: ABC → αβ (stationary 2-phase). Park: αβ → dq (rotating, synced to rotor). Park needs rotor angle; Clarke doesn't. |
| "Why Id=0?" | For SPMSM, torque = 1.5·pole_pairs·flux·Iq. Id doesn't contribute to torque but adds copper loss. Minimizing Id = maximizing torque per amp. |
| "How do you tune FOC current loops?" | Start with low Kp (~1), no Ki. Raise Kp until Iq tracks with ~1A overshoot. Add Ki (~100) to eliminate steady error. Bandwidth target: 1/10th of PWM frequency. |
| "What limits max motor speed in FOC?" | DC bus voltage. Back-EMF grows with speed. When V_bemf ≈ Vdc/2, the PI saturates. Solution: higher Vdc, or field weakening (negative Id). |
| "How do you start FOC on a stopped motor?" | 1) DC alignment (Id=constant for 500ms, rotor to d-axis). 2) Record encoder offset. 3) Close Id loop, slowly ramp Iq from 0. 4) Switch to speed mode once motor is spinning. |
| "What's SVPWM?" | Space Vector PWM — generates the stator voltage vector by switching between 6 active vectors + 2 null vectors. Uses zero-sequence injection to center-modulate, giving 15% more voltage than sinusoidal PWM. |

---

## Files

| File | Location |
|------|----------|
| FOC simulation | `/home/pi/stm32-pid-motor/sim_foc.py` |
| Output chart | `/home/pi/stm32-pid-motor/foc_sim_results.png` |
| Existing PID project | `/home/pi/stm32-pid-motor/main.c` |
| Python venv | `/home/pi/pid_env/` |
