#!/usr/bin/env python3
"""
PID Interview Training Tool — no hardware, job-ready in an afternoon
Covers: manual tuning, anti-windup, metrics, and common failure modes
"""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# ═══════════════════════════════════════════════════════════════════════
# 1. PID implementations — standard, anti-windup (clamping), incremental
# ═══════════════════════════════════════════════════════════════════════

class PID:
    """Standard PID. Problem: integral windup when output hits limits."""
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.last_error = 0.0
        self.integral = 0.0

    def update(self, measurement, dt=0.01):
        error = self.setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        self.last_error = error
        out = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return np.clip(out, 0, 100)


class PID_AntiWindup:
    """Clamping anti-windup: freeze integral when output is saturated."""
    def __init__(self, Kp, Ki, Kd, setpoint=0, out_min=0, out_max=100):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.out_min, self.out_max = out_min, out_max
        self.last_error = 0.0
        self.integral = 0.0
        self.last_out = 0.0

    def update(self, measurement, dt=0.01):
        error = self.setpoint - measurement
        p_term = self.Kp * error
        d_term = self.Kd * (error - self.last_error) / dt if dt > 0 else 0
        self.last_error = error

        # Only integrate if output is not saturated (conditional integration)
        if self.out_min < self.last_out < self.out_max:
            self.integral += error * dt
        # Also integrate if it would pull the output back toward valid range
        elif (self.last_out <= self.out_min and error > 0) or \
             (self.last_out >= self.out_max and error < 0):
            self.integral += error * dt

        i_term = self.Ki * self.integral
        out = p_term + i_term + d_term
        self.last_out = float(np.clip(out, self.out_min, self.out_max))
        return self.last_out


class PID_Incremental:
    """Incremental PID: output = last_out + delta. Naturally anti-windup."""
    def __init__(self, Kp, Ki, Kd, setpoint=0, out_min=0, out_max=100):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.out_min, self.out_max = out_min, out_max
        self.last_error = 0.0
        self.prev_error = 0.0
        self.last_out = 0.0

    def update(self, measurement, dt=0.01):
        error = self.setpoint - measurement
        delta = (self.Kp * (error - self.last_error) +
                 self.Ki * error * dt +
                 self.Kd * (error - 2 * self.last_error + self.prev_error) / dt)
        self.prev_error = self.last_error
        self.last_error = error
        self.last_out = float(np.clip(self.last_out + delta, self.out_min, self.out_max))
        return self.last_out


# ═══════════════════════════════════════════════════════════════════════
# 2. Plant models
# ═══════════════════════════════════════════════════════════════════════

class FirstOrderPlant:
    """tau * dy/dt + y = K * u   (DC motor, heater, simple tank)"""
    def __init__(self, tau=0.5, gain=1.0, dead_time_samples=0):
        self.tau = tau
        self.gain = gain
        self.y = 0.0
        self._dead_buf = [0.0] * dead_time_samples

    def step(self, u, dt=0.01):
        self._dead_buf.append(u)
        delayed = self._dead_buf.pop(0)
        self.y += (self.gain * delayed - self.y) / self.tau * dt
        return self.y


class SecondOrderPlant:
    """Underdamped second-order: overshoots naturally (drone, servo)"""
    def __init__(self, wn=5.0, zeta=0.3, gain=1.0):
        self.wn = wn          # natural frequency
        self.zeta = zeta      # damping ratio
        self.gain = gain
        self.y = 0.0
        self.dy = 0.0

    def step(self, u, dt=0.01):
        ddy = self.gain * self.wn**2 * u - 2 * self.zeta * self.wn * self.dy - self.wn**2 * self.y
        self.dy += ddy * dt
        self.y += self.dy * dt
        return self.y


class IntegratingPlant:
    """dy/dt = K * u   (liquid level, position without friction)"""
    def __init__(self, gain=0.5):
        self.gain = gain
        self.y = 0.0

    def step(self, u, dt=0.01):
        self.y += self.gain * u * dt
        return self.y


# ═══════════════════════════════════════════════════════════════════════
# 3. Simulation runner
# ═══════════════════════════════════════════════════════════════════════

def simulate(pid_class, pid_kwargs, plant, dt=0.005, T=5.0,
             setpoint=80, disturbance_t=None, noise_std=0.3):
    """Unified sim runner. Returns time, measurement, control arrays."""
    pid = pid_class(setpoint=setpoint, **pid_kwargs)
    steps = int(T / dt)
    t = np.linspace(0, T, steps)
    y_arr = np.zeros(steps)
    u_arr = np.zeros(steps)
    plant.y = 0.0  # reset
    if hasattr(plant, 'dy'):
        plant.dy = 0.0

    for i in range(steps):
        noise = np.random.normal(0, noise_std)
        u = pid.update(plant.y + noise, dt)

        # Inject disturbance
        if disturbance_t and any(abs(t[i] - td) < dt for td in disturbance_t):
            u += 30  # load disturbance

        val = plant.step(u, dt)
        y_arr[i], u_arr[i] = val, u
    return t, y_arr, u_arr


def compute_metrics(t, y, setpoint):
    """Rise time, settling time, overshoot, steady-state error, IAE."""
    steady_band = int(len(y) * 0.2)
    steady_val = float(np.mean(y[-steady_band:])) if steady_band else float(y[-1])
    ss_err = setpoint - steady_val
    overshoot = max(0.0, float(np.max(y) - setpoint))

    # Rise time: 10% -> 90% of setpoint
    lo, hi = setpoint * 0.1, setpoint * 0.9
    rise_start = np.argmax(y >= lo) if np.any(y >= lo) else 0
    rise_end = np.argmax(y >= hi) if np.any(y >= hi) else -1
    rise_t = float(t[rise_end] - t[rise_start]) if rise_end > 0 else float("inf")

    # Settling time: enter and stay within 5%
    settle_mask = np.abs(y - setpoint) <= setpoint * 0.05
    # Find the point where it enters and never leaves
    settle_t = float("inf")
    for i in range(len(t) - steady_band):
        if np.all(settle_mask[i:i + steady_band // 2]):
            settle_t = float(t[i])
            break

    # IAE = Integral of Absolute Error
    iae = float(np.trapezoid(np.abs(setpoint - y), t))

    return {
        "steady": steady_val, "ss_err": ss_err, "overshoot": overshoot,
        "rise_time": rise_t, "settle_time": settle_t, "IAE": iae,
    }


# ═══════════════════════════════════════════════════════════════════════
# 4. Modes
# ═══════════════════════════════════════════════════════════════════════

def mode_tuning_walkthrough():
    """Guided manual tuning: the step-by-step method interviewers expect."""
    print("\n" + "=" * 60)
    print("GUIDED TUNING WALKTHROUGH")
    print("The method every embedded interviewer wants to hear")
    print("=" * 60)

    plant = FirstOrderPlant(tau=0.5, gain=1.0)
    setpoint = 80
    print("\nPlant: first-order, tau=0.5s (slow heater/DC motor)")
    print("Goal: setpoint=80,  tune to minimal overshoot & fast settle\n")

    # Build a 2x2 grid of 4 subplots showing the tuning progression
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle("Manual PID Tuning — Step by Step", fontsize=13, fontweight="bold")
    ax_flat = axes.flatten()

    steps_data = []

    # Step 1: Ki=0, Kd=0, increase Kp until oscillation
    kp, ki, kd = 3.0, 0.0, 0.0
    t, y, u = simulate(PID_AntiWindup, dict(Kp=kp, Ki=ki, Kd=kd), plant, setpoint=setpoint)
    m = compute_metrics(t, y, setpoint)
    steps_data.append((ax_flat[0], t, y, kp, ki, kd, m,
                       "Step 1: P-only, Kp=3", "Visible steady error (gap to 80)"))

    # Step 2: Add Ki to kill steady error
    kp, ki, kd = 3.0, 2.0, 0.0
    t, y, u = simulate(PID_AntiWindup, dict(Kp=kp, Ki=ki, Kd=kd), plant, setpoint=setpoint)
    m = compute_metrics(t, y, setpoint)
    steps_data.append((ax_flat[1], t, y, kp, ki, kd, m,
                       "Step 2: Add Ki=2 (PI)", "Error -> 0, but slight overshoot"))

    # Step 3: Add Kd to damp the overshoot
    kp, ki, kd = 5.0, 3.0, 0.5
    t, y, u = simulate(PID_AntiWindup, dict(Kp=kp, Ki=ki, Kd=kd), plant, setpoint=setpoint)
    m = compute_metrics(t, y, setpoint)
    steps_data.append((ax_flat[2], t, y, kp, ki, kd, m,
                       "Step 3: Add Kd=0.5 (PID)", "D damps overshoot, smooth landing"))

    # Step 4: Fine-tuned
    kp, ki, kd = 4.0, 2.5, 0.6
    t, y, u = simulate(PID_AntiWindup, dict(Kp=kp, Ki=ki, Kd=kd), plant, setpoint=setpoint)
    m = compute_metrics(t, y, setpoint)
    steps_data.append((ax_flat[3], t, y, kp, ki, kd, m,
                       "Step 4: Fine-tune (PID)", "Fast rise, no overshoot, zero error"))

    for ax, t, y, kp, ki, kd, m, title, desc in steps_data:
        ax.plot(t, y, color="#3498db", linewidth=1.5)
        ax.axhline(setpoint, color="gray", ls="--", alpha=0.4)
        ax.set_title(title, fontsize=10, fontweight="bold")
        ax.set_ylabel(f"Kp={kp} Ki={ki} Kd={kd}")
        ax.set_ylim(-5, 120)
        ax.grid(True, alpha=0.2)
        ax.text(0.98, 0.88,
                f"Err={m['ss_err']:+.1f}  OS={m['overshoot']:.1f}  "
                f"Settle={'%.2fs' % m['settle_time'] if np.isfinite(m['settle_time']) else 'N/A'}",
                transform=ax.transAxes, fontsize=8, ha="right",
                bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))
        ax.text(0.02, 0.88, desc, transform=ax.transAxes, fontsize=7, va="top",
                color="#7f8c8d")

    axes[1, 0].set_xlabel("Time (s)")
    axes[1, 1].set_xlabel("Time (s)")
    plt.tight_layout()
    plt.savefig("/home/pi/pid_tuning_walkthrough.png", dpi=120)
    plt.close()
    print("Chart saved: /home/pi/pid_tuning_walkthrough.png")

    # Print the interview-ready summary
    print("""
┌─ INTERVIEW CHEAT SHEET ─────────────────────────────────────┐
│ Manual PID Tuning Method (what they want to hear):          │
│                                                             │
│ 1. Set Ki=Kd=0. Increase Kp until it oscillates steadily.  │
│    -> This is your "ultimate gain" Ku.                      │
│                                                             │
│ 2. Measure the oscillation period Tu (peak-to-peak).        │
│    -> Ziegler-Nichols: Kp=0.6*Ku, Ki=2*Kp/Tu, Kd=Kp*Tu/8  │
│                                                             │
│ 3. Add Ki to eliminate steady-state error (the gap).        │
│    -> If it oscillates, Ki is too high.                     │
│                                                             │
│ 4. Add Kd to damp overshoot and smooth the response.        │
│    -> If noisy/jittery, Kd is too high.                     │
│                                                             │
│ 5. Iterate: Kp up = faster but more overshoot               │
│             Ki up = kills error faster but rings            │
│             Kd up = smoother but slower + noise-sensitive   │
└─────────────────────────────────────────────────────────────┘
""")


def mode_antiwindup():
    """Show the difference between standard PID and anti-windup PID."""
    print("\n" + "=" * 60)
    print("ANTI-WINDUP COMPARISON")
    print("Standard PID vs Clamping vs Incremental")
    print("=" * 60)

    # Use a plant that needs large output initially -> saturation
    plant = FirstOrderPlant(tau=1.0, gain=0.3)  # slow + low gain = needs big output
    setpoint = 80

    fig, axes = plt.subplots(2, 3, figsize=(14, 7))
    fig.suptitle("Anti-Windup: Standard PID vs Clamping vs Incremental",
                 fontsize=12, fontweight="bold")

    configs = [
        ("Standard PID\n(windup!)", PID, dict(Kp=4, Ki=3, Kd=0.3)),
        ("Clamping Anti-Windup\n(conditional integration)", PID_AntiWindup, dict(Kp=4, Ki=3, Kd=0.3)),
        ("Incremental PID\n(natural anti-windup)", PID_Incremental, dict(Kp=4, Ki=3, Kd=0.3)),
    ]

    for col, (label, pid_cls, kwargs) in enumerate(configs):
        t, y, u = simulate(pid_cls, kwargs, plant, setpoint=setpoint)
        m = compute_metrics(t, y, setpoint)

        axes[0, col].plot(t, y, color="#3498db", linewidth=1.5)
        axes[0, col].axhline(setpoint, color="gray", ls="--", alpha=0.4)
        axes[0, col].set_title(label, fontsize=9, fontweight="bold")
        axes[0, col].set_ylabel("Output")
        axes[0, col].set_ylim(-5, 130)
        axes[0, col].grid(True, alpha=0.2)
        axes[0, col].text(0.98, 0.92,
                          f"Settle={'%.2fs' % m['settle_time'] if np.isfinite(m['settle_time']) else 'N/A'}\n"
                          f"OS={m['overshoot']:.1f}",
                          transform=axes[0, col].transAxes, fontsize=8, ha="right",
                          bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))

        axes[1, col].plot(t, u, color="#e74c3c", linewidth=1.2)
        axes[1, col].axhline(100, color="gray", ls=":", alpha=0.4, label="Saturation (100)")
        axes[1, col].axhline(0, color="gray", ls=":", alpha=0.4)
        axes[1, col].set_ylabel("Control")
        axes[1, col].set_xlabel("Time (s)")
        axes[1, col].set_ylim(-5, 110)
        axes[1, col].grid(True, alpha=0.2)
        axes[1, col].legend(fontsize=7)

    plt.tight_layout()
    plt.savefig("/home/pi/pid_antiwindup.png", dpi=120)
    plt.close()
    print("Chart saved: /home/pi/pid_antiwindup.png")
    print("""
┌─ WHY THIS MATTERS IN INTERVIEWS ────────────────────────────┐
│ "What is integral windup and how do you prevent it?"       │
│                                                             │
│ Windup: When output saturates (e.g. PWM at 100%), the      │
│ integral term keeps accumulating error. When the setpoint   │
│ is finally reached, the integral is so huge it causes a     │
│ massive overshoot before unwinding.                         │
│                                                             │
│ 3 common fixes (name at least ONE in an interview):        │
│                                                             │
│ 1. Clamping — freeze integral when output is saturated     │
│ 2. Back-calculation — feed (clipped - raw) back to integral│
│ 3. Incremental PID — output delta instead of absolute,     │
│    naturally bounded. No integral accumulator to blow up.  │
└─────────────────────────────────────────────────────────────┘
""")


def mode_interview_sim():
    """Practice: tune an unknown plant — the interview scenario."""
    print("\n" + "=" * 60)
    print("INTERVIEW SIMULATION — Tune an unknown plant")
    print("You get the step response, you adjust Kp/Ki/Kd")
    print("Goal: overshoot < 5, settle_time < 2s, |ss_err| < 1")
    print("=" * 60)

    plants = [
        ("Fast motor (tau=0.1)", FirstOrderPlant(tau=0.1, gain=1.0)),
        ("Slow heater (tau=1.0, deadtime=0.02s)", FirstOrderPlant(tau=1.0, gain=1.0, dead_time_samples=4)),
        ("Underdamped drone (wn=8, zeta=0.2)", SecondOrderPlant(wn=8, zeta=0.2)),
        ("Integrating tank", IntegratingPlant(gain=0.5)),
    ]

    for name, plant in plants:
        print(f"\n{'─' * 50}")
        print(f"Plant: {name}")
        print("Try to tune it. Enter Kp Ki Kd, or 'skip' / 'q' to move on.")
        print(f"{'─' * 50}")

        kp, ki, kd = 2.0, 0.0, 0.0
        while True:
            t, y, u = simulate(PID_AntiWindup, dict(Kp=kp, Ki=ki, Kd=kd), plant, setpoint=80)
            m = compute_metrics(t, y, 80)
            ok = (m["overshoot"] < 5 and m["settle_time"] < 2.0 and abs(m["ss_err"]) < 1)

            status = "PASS" if ok else "---"
            print(f"[{status}] Kp={kp:.2f} Ki={ki:.2f} Kd={kd:.2f}  "
                  f"OS={m['overshoot']:.1f}  Settle={m['settle_time']:.3f}s  "
                  f"Err={m['ss_err']:+.2f}  IAE={m['IAE']:.1f}")

            if ok:
                print("  >>> Tuned successfully! <<<")
                # Plot the successful tune
                fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 4))
                ax1.plot(t, y, color="#2ecc71", linewidth=1.5)
                ax1.axhline(80, color="gray", ls="--")
                ax1.set_title(f"{name} — Kp={kp} Ki={ki} Kd={kd} [PASS]")
                ax1.set_ylabel("Output")
                ax1.grid(True, alpha=0.2)
                ax2.plot(t, u, color="#e74c3c", linewidth=1)
                ax2.set_ylabel("Control")
                ax2.set_xlabel("Time (s)")
                ax2.grid(True, alpha=0.2)
                plt.tight_layout()
                path = f"/home/pi/pid_interview_{name.replace(' ', '_').replace('(', '').replace(')', '').replace('=', '_')}.png"
                plt.savefig(path, dpi=100)
                plt.close()
                print(f"  Chart: {path}")
                break

            try:
                user = input("Kp Ki Kd > ").strip()
            except EOFError:
                print("\n  (non-interactive, skipping remaining plants)")
                break
            if user.lower() in ("q", "skip"):
                print("  Skipped.\n")
                break
            if not user:
                continue
            try:
                parts = user.split()
                kp, ki, kd = float(parts[0]), float(parts[1]), float(parts[2])
            except (ValueError, IndexError):
                print("  Format: Kp Ki Kd")


def mode_ziegler_nichols():
    """Auto-tune using Ziegler-Nichols closed-loop method."""
    print("\n" + "=" * 60)
    print("ZIEGLER-NICHOLS AUTO-TUNING")
    print("The classic method from 1942, still used today")
    print("=" * 60)

    plant = FirstOrderPlant(tau=0.5, gain=1.0)
    setpoint = 80

    # Step 1: Find ultimate gain Ku by increasing Kp until sustained oscillation
    # ZN requires a plant that can sustain oscillation.
    # First-order plants don't oscillate with P-only — use a second-order underdamped plant.
    plant_zn = SecondOrderPlant(wn=3.0, zeta=0.1)
    print("\nStep 1: Finding ultimate gain Ku (increase Kp until steady oscillation)...")
    print("  (using second-order underdamped plant — first-order won't oscillate with P-only)")
    Ku, Tu = None, None
    for kp in np.arange(0.2, 30, 0.3):
        t, y, _ = simulate(PID, dict(Kp=kp, Ki=0, Kd=0), plant_zn,
                           setpoint=setpoint, T=20.0, noise_std=0.05)
        # Detect peaks in the second half
        half = len(y) // 2
        peaks = []
        for i in range(1, len(y) - half - 1):
            idx = half + i
            if y[idx] > y[idx - 1] and y[idx] > y[idx + 1] and y[idx] > setpoint * 0.5:
                peaks.append((t[idx], y[idx]))

        if len(peaks) >= 4:
            amplitudes = [abs(p[1] - setpoint) for p in peaks[-4:]]
            if np.std(amplitudes) < max(amplitudes) * 0.35:
                Ku = kp
                periods = [peaks[i + 1][0] - peaks[i][0] for i in range(len(peaks) - 1)]
                Tu = np.mean(periods) if periods else 1.0
                break

    if Ku is None:
        print("Could not find Ku. Oscillation not detected.")
        return

    print(f"  Ku = {Ku:.2f},  Tu = {Tu:.3f}s")

    # Step 2: Calculate gains from ZN formulas
    print("\nStep 2: Apply Ziegler-Nichols formulas...")

    zn_sets = {
        "P":     (0.50 * Ku, 0.0, 0.0),
        "PI":    (0.45 * Ku, 0.54 * Ku / Tu, 0.0),
        "PID":   (0.60 * Ku, 1.20 * Ku / Tu, 0.075 * Ku * Tu),
        "PID (some overshoot)": (0.33 * Ku, 0.66 * Ku / Tu, 0.11 * Ku * Tu),
        "PID (no overshoot)":   (0.20 * Ku, 0.40 * Ku / Tu, 0.066 * Ku * Tu),
    }

    fig, axes = plt.subplots(len(zn_sets), 1, figsize=(10, 10), sharex=True)
    fig.suptitle(f"Ziegler-Nichols Tuning (Ku={Ku:.2f}, Tu={Tu:.3f}s)",
                 fontsize=12, fontweight="bold")

    for ax, (label, (kp, ki, kd)) in zip(axes, zn_sets.items()):
        t, y, _ = simulate(PID_AntiWindup, dict(Kp=kp, Ki=ki, Kd=kd), plant, setpoint=setpoint)
        m = compute_metrics(t, y, setpoint)
        ax.plot(t, y, color="#3498db", linewidth=1.4)
        ax.axhline(setpoint, color="gray", ls="--", alpha=0.4)
        ax.set_ylabel(f"{label}\nKp={kp:.2f} Ki={ki:.2f} Kd={kd:.3f}", fontsize=8)
        ax.set_ylim(-5, 130)
        ax.grid(True, alpha=0.2)
        ax.text(0.98, 0.88,
                f"OS={m['overshoot']:.1f}  Settle={'%.2fs' % m['settle_time'] if np.isfinite(m['settle_time']) else 'N/A'}  Err={m['ss_err']:+.2f}",
                transform=ax.transAxes, fontsize=8, ha="right",
                bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))

    axes[-1].set_xlabel("Time (s)")
    plt.tight_layout()
    plt.savefig("/home/pi/pid_ziegler_nichols.png", dpi=120)
    plt.close()
    print("Chart saved: /home/pi/pid_ziegler_nichols.png")
    print("""
┌─ ZIEGLER-NICHOLS CHEAT SHEET ───────────────────────────────┐
│ Closed-loop method:                                         │
│  1. Set Ki=Kd=0, increase Kp until steady oscillation       │
│     -> Record Ku (critical gain) and Tu (period)            │
│  2. Apply formula:                                          │
│     P:   Kp=0.5*Ku                                         │
│     PI:  Kp=0.45*Ku, Ki=0.54*Ku/Tu                         │
│     PID: Kp=0.6*Ku, Ki=1.2*Ku/Tu, Kd=0.075*Ku*Tu          │
│  3. Fine-tune manually from this starting point             │
│                                                             │
│ Interview tip: ZN gives aggressive starting values.         │
│ Real practice: reduce Kp by 10-20% after ZN for margin.     │
└─────────────────────────────────────────────────────────────┘
""")


# ═══════════════════════════════════════════════════════════════════════
# Main menu
# ═══════════════════════════════════════════════════════════════════════

def main():
    print("""
╔══════════════════════════════════════════════════════════════╗
║         PID INTERVIEW TRAINING TOOL                          ║
║         No hardware needed. Just run the modes.              ║
╠══════════════════════════════════════════════════════════════╣
║  1  Guided Tuning Walkthrough  — the 4-step method          ║
║  2  Anti-Windup Comparison     — why real PID needs limits  ║
║  3  Ziegler-Nichols Auto-Tune  — classic 1942 method        ║
║  4  Interview Simulation       — tune an unknown plant      ║
║  5  Run ALL modes              — full training session      ║
║  q  Quit                                                    ║
╚══════════════════════════════════════════════════════════════╝
""")

    while True:
        try:
            choice = input("Select mode [1-5/q] > ").strip()
        except EOFError:
            print("\nDone.")
            break
        if choice == "1":
            mode_tuning_walkthrough()
        elif choice == "2":
            mode_antiwindup()
        elif choice == "3":
            mode_ziegler_nichols()
        elif choice == "4":
            mode_interview_sim()
        elif choice == "5":
            mode_tuning_walkthrough()
            mode_antiwindup()
            mode_ziegler_nichols()
            mode_interview_sim()
        elif choice.lower() == "q":
            print("Good luck with the interview!")
            break
        else:
            print("Invalid choice. Enter 1-5 or q.")


if __name__ == "__main__":
    main()
