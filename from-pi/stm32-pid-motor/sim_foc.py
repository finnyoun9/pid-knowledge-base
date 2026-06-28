#!/usr/bin/env python3
"""
FOC (Field-Oriented Control) simulation for BLDC motor.
Clarke → Park → PI current loops → Inverse Park → Inverse Clarke → SVPWM

Run: source /home/pi/pid_env/bin/activate && python3 sim_foc.py
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ═══════════════════════════════════════════════════════════════
# 1. FOC Core Transforms
# ═══════════════════════════════════════════════════════════════

def clarke_transform(ia, ib, ic):
    """3-phase (ABC) → 2-phase stationary (αβ)"""
    i_alpha = ia
    i_beta  = (ia + 2*ib) / np.sqrt(3)  # 120° shift
    return i_alpha, i_beta

def park_transform(i_alpha, i_beta, theta):
    """Stationary (αβ) → Rotating (dq)"""
    sin_t = np.sin(theta)
    cos_t = np.cos(theta)
    id =  i_alpha * cos_t + i_beta * sin_t
    iq = -i_alpha * sin_t + i_beta * cos_t
    return id, iq

def inv_park_transform(vd, vq, theta):
    """Rotating (dq) → Stationary (αβ)"""
    sin_t = np.sin(theta)
    cos_t = np.cos(theta)
    v_alpha = vd * cos_t - vq * sin_t
    v_beta  = vd * sin_t + vq * cos_t
    return v_alpha, v_beta

def inv_clarke_transform(v_alpha, v_beta):
    """(αβ) → 3-phase duty cycles (va, vb, vc)"""
    va = v_alpha
    vb = -0.5 * v_alpha + np.sqrt(3)/2 * v_beta
    vc = -0.5 * v_alpha - np.sqrt(3)/2 * v_beta
    return va, vb, vc

# ═══════════════════════════════════════════════════════════════
# 2. Space Vector PWM (SVPWM)
# ═══════════════════════════════════════════════════════════════

def svpwm(va, vb, vc, vdc=12.0):
    """
    Space Vector PWM using min/max injection (centered SVM).
    Converts (va, vb, vc) reference voltages to duty cycles.
    """
    vmax = max(va, vb, vc)
    vmin = min(va, vb, vc)
    v_offset = (vmax + vmin) / 2.0  # zero-sequence injection
    da = (va - v_offset) / vdc + 0.5
    db = (vb - v_offset) / vdc + 0.5
    dc = (vc - v_offset) / vdc + 0.5
    return da, db, dc

# ═══════════════════════════════════════════════════════════════
# 3. PI Controller (same structure as your C code)
# ═══════════════════════════════════════════════════════════════

class PIController:
    def __init__(self, Kp, Ki, limit=12.0):
        self.Kp = Kp
        self.Ki = Ki
        self.limit = limit
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def update(self, error, dt):
        self.integral += error * dt
        # Clamp integral anti-windup
        self.integral = np.clip(self.integral, -self.limit/self.Ki, self.limit/self.Ki)
        output = self.Kp * error + self.Ki * self.integral
        return np.clip(output, -self.limit, self.limit)

# ═══════════════════════════════════════════════════════════════
# 4. Simple BLDC Motor Model (for simulation)
# ═══════════════════════════════════════════════════════════════

class BLDCMotor:
    """
    Simplified BLDC motor dq-model.
    Parameters typical for a small gimbal BLDC.
    """
    def __init__(self, Rs=0.5, Ld=0.001, Lq=0.001, flux=0.005,
                 J=0.0005, B=0.00005, pole_pairs=7):
        self.Rs = Rs           # Stator resistance (Ω)
        self.Ld = Ld           # d-axis inductance (H)
        self.Lq = Lq           # q-axis inductance (H)
        self.flux = flux       # PM flux linkage (Wb) — REDUCED for low back-EMF
        self.J = J             # Rotor inertia (kg·m²)
        self.B = B             # Viscous damping
        self.pole_pairs = pp = pole_pairs

        # State
        self.id = 0.0
        self.iq = 0.0
        self.omega_e = 0.0     # Electrical angular velocity (rad/s)
        self.theta_e = 0.0     # Electrical angle (rad)
        self.omega_m = 0.0     # Mechanical angular velocity (rad/s)

    def step(self, vd, vq, dt, T_load=0.0):
        """Step the motor model by dt seconds."""
        pp = self.pole_pairs
        omega_e = self.omega_e

        # dq current dynamics
        did_dt = (vd - self.Rs * self.id + omega_e * self.Lq * self.iq) / self.Ld
        diq_dt = (vq - self.Rs * self.iq - omega_e * (self.Ld * self.id + self.flux)) / self.Lq

        self.id += did_dt * dt
        self.iq += diq_dt * dt

        # Electromagnetic torque: Te = 1.5 * pp * (flux*iq + (Ld-Lq)*id*iq)
        Te = 1.5 * pp * self.flux * self.iq  # (Ld≈Lq for surface-mount PMSM)

        # Mechanical dynamics
        domega_m = (Te - self.B * self.omega_m - T_load) / self.J
        self.omega_m += domega_m * dt
        self.omega_e = self.omega_m * pp
        self.theta_e += self.omega_e * dt
        self.theta_e = np.fmod(self.theta_e, 2 * np.pi)

# ═══════════════════════════════════════════════════════════════
# 5. Main FOC Simulation Loop
# ═══════════════════════════════════════════════════════════════

def print_section(title):
    """Print a highlighted section header."""
    print(f"\n{'═'*55}")
    print(f"  {title}")
    print(f"{'═'*55}")

def run_foc_simulation(dt=0.0001, runtime=0.5, speed_rpm=1000):
    """Run FOC current + speed control loop."""

    print_section("FOC Simulation: Setup")
    motor = BLDCMotor()
    vdc = 12.0

    # PI controllers
    pi_id = PIController(Kp=5.0, Ki=200.0, limit=vdc/2)       # d-axis current loop
    pi_iq = PIController(Kp=5.0, Ki=200.0, limit=vdc/2)       # q-axis current loop
    pi_speed = PIController(Kp=0.5, Ki=5.0, limit=5.0)        # speed loop (outer)

    id_ref = 0.0  # Id=0 control (maximum torque per ampere)
    speed_ref = speed_rpm * 2 * np.pi / 60  # RPM → rad/s (mechanical)

    steps = int(runtime / dt)
    log_every = 5  # log every 500µs

    # Data logging
    data = {
        't': [], 'id': [], 'iq': [], 'id_ref': [],
        'iq_ref': [], 'speed': [], 'vd': [], 'vq': [],
        'Te': [], 'theta': [],
    }

    print(f"  Target speed: {speed_rpm} RPM ({speed_ref:.1f} rad/s)")
    print(f"  Motor: Rs={motor.Rs}Ω, Ld={motor.Ld*1e3:.1f}mH, flux={motor.flux*1e3:.1f}mWb")
    print(f"  Vdc={vdc}V, dt={dt*1e6:.0f}µs, {steps} steps ({runtime*1e3:.0f}ms)")
    print(f"  PI gains: Speed(Kp={pi_speed.Kp}, Ki={pi_speed.Ki})")
    print(f"            Current(Kp={pi_id.Kp}, Ki={pi_id.Ki})")

    # Load step at 250ms
    load_time = 0.25

    print_section("FOC Simulation: Running")
    for i in range(steps):
        t = i * dt

        # --- 1. Measure phase currents (reconstruct from dq state) ---
        i_alpha, i_beta = inv_park_transform(motor.id, motor.iq, motor.theta_e)
        ia, ib, ic = inv_clarke_transform(i_alpha, i_beta)

        # --- 2. Clarke → Park (measurement in rotor frame) ---
        i_alpha_m, i_beta_m = clarke_transform(ia, ib, ic)
        id_meas, iq_meas = park_transform(i_alpha_m, i_beta_m, motor.theta_e)

        # --- 3. Speed PI (outer loop, 1kHz = every 10 current steps) ---
        if i % 10 == 0:
            speed_err = speed_ref - motor.omega_m
            iq_ref = pi_speed.update(speed_err, dt * 10)

        # --- 4. Current PI (inner loop, 10kHz) ---
        vd = pi_id.update(id_ref - id_meas, dt)
        vq = pi_iq.update(iq_ref - iq_meas, dt)

        # --- 5. Inverse Park + Inverse Clarke + SVPWM ---
        v_alpha, v_beta = inv_park_transform(vd, vq, motor.theta_e)
        va, vb, vc = inv_clarke_transform(v_alpha, v_beta)
        da, db, dc = svpwm(va, vb, vc, vdc)

        # --- 6. Apply to motor ---
        T_load = 0.005 if t > load_time else 0.0  # small load at 250ms
        motor.step(vd, vq, dt, T_load)

        # --- 7. Log ---
        if i % log_every == 0:
            data['t'].append(t * 1000)  # ms
            data['id'].append(id_meas)
            data['iq'].append(iq_meas)
            data['id_ref'].append(id_ref)
            data['iq_ref'].append(iq_ref)
            data['speed'].append(motor.omega_m * 60 / (2 * np.pi))  # RPM
            data['vd'].append(vd)
            data['vq'].append(vq)
            data['Te'].append(1.5 * motor.pole_pairs * motor.flux * motor.iq)

        # Progress bar
        if i % (steps // 10) == 0:
            print(f"  {i*100//steps:>3}% | speed={motor.omega_m*60/(2*np.pi):.0f} RPM, "
                  f"Id={motor.id:.3f}A, Iq={motor.iq:.3f}A")

    # ═══════════════════════════════════════════════════════════
    # 6. Results
    # ═══════════════════════════════════════════════════════════
    final_rpm = motor.omega_m * 60 / (2 * np.pi)
    print(f"\n  Completed! Final: {final_rpm:.0f} RPM (target {speed_rpm})")
    print(f"  Id={motor.id:.4f}A, Iq={motor.iq:.4f}A")
    print(f"  Torque={1.5 * motor.pole_pairs * motor.flux * motor.iq:.5f} Nm")

    # ═══════════════════════════════════════════════════════════
    # 7. Plots
    # ═══════════════════════════════════════════════════════════
    fig, axs = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    t = np.array(data['t'])

    # Speed
    axs[0].plot(t, data['speed'], 'b-', label='Speed')
    axs[0].axhline(y=speed_rpm, color='r', ls='--', alpha=0.6, label=f'Target {speed_rpm} RPM')
    axs[0].axvline(x=load_time*1000, color='gray', ls=':', alpha=0.6, label='Load step')
    axs[0].set_ylabel('Speed (RPM)')
    axs[0].legend(fontsize=9)
    axs[0].grid(True, alpha=0.3)

    # dq currents
    axs[1].plot(t, data['iq'], 'r-', label='Iq (torque current)', linewidth=1)
    axs[1].plot(t, data['iq_ref'], 'r--', alpha=0.5, label='Iq reference', linewidth=1)
    axs[1].plot(t, data['id'], 'b-', label='Id (flux current)', linewidth=1)
    axs[1].axhline(y=0, color='gray', ls=':', alpha=0.3)
    axs[1].set_ylabel('Current (A)')
    axs[1].legend(fontsize=9)
    axs[1].grid(True, alpha=0.3)

    # dq voltages
    axs[2].plot(t, data['vd'], 'g-', label='Vd', linewidth=1)
    axs[2].plot(t, data['vq'], 'm-', label='Vq', linewidth=1)
    axs[2].axhline(y=6, color='gray', ls=':', alpha=0.3, label='Vdc/2')
    axs[2].axhline(y=-6, color='gray', ls=':', alpha=0.3)
    axs[2].set_ylabel('Voltage (V)')
    axs[2].legend(fontsize=9)
    axs[2].grid(True, alpha=0.3)

    # Torque
    axs[3].plot(t, data['Te'], 'orange', label='Electromagnetic torque', linewidth=1)
    axs[3].axvline(x=load_time*1000, color='gray', ls=':', alpha=0.6)
    axs[3].set_ylabel('Torque (Nm)')
    axs[3].set_xlabel('Time (ms)')
    axs[3].legend(fontsize=9)
    axs[3].grid(True, alpha=0.3)

    speed_pct = final_rpm / speed_rpm * 100
    plt.suptitle(f'FOC Simulation — Speed Control (reached {final_rpm:.0f}/{speed_rpm} RPM = {speed_pct:.0f}%)')
    plt.tight_layout()
    plt.savefig('/home/pi/stm32-pid-motor/foc_sim_results.png', dpi=150)
    print(f"\n  Plot saved to foc_sim_results.png")


if __name__ == '__main__':
    run_foc_simulation(dt=0.0001, runtime=1.0, speed_rpm=1000)
