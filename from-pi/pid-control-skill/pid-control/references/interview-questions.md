# PID Interview Q&A

## Fundamental Questions

### Q: What is steady-state error and how do you eliminate it?
**A**: The residual difference between setpoint and actual output after the system stabilizes. P-only controllers have inherent steady-state error because the error must be non-zero to produce output. Adding the I term (integrating over time) eliminates it — Ki continuously accumulates the error until output is sufficient.

### Q: How do Kp, Ki, Kd affect system response?
**A**:
- **Kp up**: Faster rise, more overshoot, less stable
- **Ki up**: Eliminates error faster, but causes more overshoot and oscillation
- **Kd up**: Dampens overshoot, smoother response, but slower to react and noise-sensitive

### Q: What is integral windup and how do you prevent it?
**A**: When the control output saturates (e.g. PWM hits 100%) but the integral term keeps accumulating error because the actual value is still below setpoint. When the output finally desaturates, the bloated integral causes massive overshoot. **Fixes:**
1. **Clamping** (conditional integration) — freeze I accumulation when saturated
2. **Back-calculation** — feed (clipped − raw output) back to the integrator
3. **Incremental PID** — output = last_out + delta, naturally bounded

### Q: Explain the Ziegler-Nichols tuning method.
**A**: Closed-loop method from 1942:
1. Set Ki=Kd=0, increase Kp until the system oscillates at constant amplitude
2. Record ultimate gain Ku and oscillation period Tu
3. Apply formula: P: Kp=0.5Ku; PI: Kp=0.45Ku, Ki=0.54Ku/Tu; PID: Kp=0.6Ku, Ki=1.2Ku/Tu, Kd=0.075Ku×Tu
4. Fine-tune from there (ZN is aggressive — reduce Kp 10-20% for margin)

### Q: What's the difference between positional and incremental PID?
**A**: 
- **Positional (standard)**: output = Kp×e + Ki×∫e + Kd×de/dt. Has an integrator that accumulates → windup risk.
- **Incremental**: output = last_output + Δoutput. Δoutput only depends on recent errors. Naturally anti-windup since the output is directly clipped.

### Q: How would you tune PID for a slow thermal system vs a fast motor?
**A**:
- **Thermal (large tau)**: Use modest Kp (system is inherently stable), generous Ki to slowly eliminate error, small or no Kd (thermal dynamics are slow, derivative sees mostly noise)
- **Fast motor (small tau)**: Higher Kp for quick response, careful Ki to avoid overshoot, Kd helps with braking but must be filtered due to sensor noise

### Q: What's the derivative kick problem and how to fix it?
**A**: When setpoint changes abruptly, de/dt spikes → huge D term → motor jerks. Fix: put derivative on **measurement only** (not on error): `Kd × (0 − d(measurement)/dt)` instead of `Kd × d(error)/dt`.

## Embedded-Specific Questions

### Q: How do you implement PID on a microcontroller?
**A**: Fixed-point arithmetic if no FPU. Timer interrupt at fixed frequency (e.g. 100Hz-1kHz). Read sensor, compute error, update integral/derivative, clamp output, write to PWM register. Watch for overflow in integral accumulator — use saturation arithmetic.

### Q: What sampling rate should you choose?
**A**: At least 10× the system's closed-loop bandwidth. Rule of thumb: 10× faster than the plant's dominant time constant. Too slow → aliasing/unstable. Too fast → noise amplification + unnecessary CPU load.

### Q: How do you handle sensor noise in the D term?
**A**:
1. Low-pass filter on the measurement before feeding into derivative
2. Use measurement-derivative instead of error-derivative (no kick either)
3. Make Kd small for noisy sensors
4. In extreme cases: skip D entirely and use PI + feedforward
