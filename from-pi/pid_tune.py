#!/usr/bin/env python3
"""
PID tuning helper — run a plant with Kp Ki Kd from command line.
Usage: /home/pi/pid_env/bin/python pid_tune.py <plant_idx> <Kp> <Ki> <Kd>
   or: /home/pi/pid_env/bin/python pid_tune.py list
"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import sys, os

sys.path.insert(0, '/home/pi')
import importlib.util
spec = importlib.util.spec_from_file_location('pid', '/home/pi/pid_sim.py')
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

plants = [
    ("Fast motor (tau=0.1)",          mod.FirstOrderPlant(tau=0.1, gain=1.0)),
    ("Slow heater (tau=1.0, deadt=0.02s)", mod.FirstOrderPlant(tau=1.0, gain=1.0, dead_time_samples=4)),
    ("Underdamped drone (wn=8, zeta=0.2)", mod.SecondOrderPlant(wn=8, zeta=0.2)),
    ("Integrating tank",              mod.IntegratingPlant(gain=0.5)),
]

if len(sys.argv) == 2 and sys.argv[1] == "list":
    print("Available plants:")
    for i, (name, _) in enumerate(plants):
        print(f"  {i}: {name}")
    sys.exit(0)

if len(sys.argv) < 4:
    print("Usage: pid_tune.py <plant_idx> <Kp> <Ki> <Kd>")
    print("   or: pid_tune.py list")
    sys.exit(1)

idx = int(sys.argv[1])
kp, ki, kd = float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])

name, plant = plants[idx]
setpoint = 80
t, y, u = mod.simulate(mod.PID_AntiWindup, dict(Kp=kp, Ki=ki, Kd=kd), plant, setpoint=setpoint)
m = mod.compute_metrics(t, y, setpoint)

ok = (m["overshoot"] < 5 and m["settle_time"] < 2.0 and abs(m["ss_err"]) < 1)
status = "✅ PASS" if ok else "❌ FAIL"

print(f"\n{'='*50}")
print(f"Plant: {name}")
print(f"Tuned: Kp={kp:.2f}  Ki={ki:.2f}  Kd={kd:.2f}")
print(f"{'='*50}")
print(f"  Status:     {status}")
print(f"  Overshoot:  {m['overshoot']:.2f}  (goal: <5)")
print(f"  Settle:     {m['settle_time']:.3f}s  (goal: <2.0s)" if np.isfinite(m['settle_time']) else f"  Settle:     N/A  (goal: <2.0s)")
print(f"  Steady err: {m['ss_err']:+.2f}  (goal: |err|<1)")
print(f"  IAE:        {m['IAE']:.1f}")
print(f"  Rise time:  {m['rise_time']:.3f}s")
print()

# Save chart
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 4))
ax1.plot(t, y, color="#2ecc71" if ok else "#e74c3c", linewidth=1.5)
ax1.axhline(80, color="gray", ls="--", alpha=0.5)
ax1.set_title(f"{name} — Kp={kp} Ki={ki} Kd={kd}  [{'PASS' if ok else 'FAIL'}]")
ax1.set_ylabel("Output")
ax1.grid(True, alpha=0.2)

ax2.plot(t, u, color="#3498db", linewidth=1)
ax2.axhline(100, color="gray", ls=":", alpha=0.4, label="PWM max")
ax2.set_ylabel("Control (PWM %)")
ax2.set_xlabel("Time (s)")
ax2.grid(True, alpha=0.2)
ax2.legend(fontsize=8)

plt.tight_layout()
path = f"/home/pi/pid_tune_plant{idx}.png"
plt.savefig(path, dpi=100)
plt.close()
print(f"Chart saved: {path}")
