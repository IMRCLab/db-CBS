import yaml
import numpy as np
import matplotlib.pyplot as plt

yaml_file = "/home/akmarak-laptop/IMRC/mrmp-benchmark/planners/dbCBS/db-CBS/new_format_motions/integrator2_2d_v0/test/my_motions.bin.im.bin.sp.bin.yaml"

vmax = 0.5      # speed limit
amax = 2.0      # acceleration limit

with open(yaml_file, "r") as f:
    motions = yaml.safe_load(f)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=False)

for motion in motions:

    states = np.asarray(motion["states"])
    actions = np.asarray(motion["actions"])

    # ||v||
    speed = np.linalg.norm(states[:, 2:4], axis=1)

    # ||a||
    accel = np.linalg.norm(actions[:, 0:2], axis=1)

    ax1.plot(speed, alpha=0.4)
    ax2.plot(accel, alpha=0.4)

ax1.axhline(vmax, color='r', linestyle='--', linewidth=2,
            label=f"vmax={vmax}")
ax2.axhline(amax, color='r', linestyle='--', linewidth=2,
            label=f"amax={amax}")

ax1.set_ylabel(r"$||v||$")
ax2.set_ylabel(r"$||a||$")
ax2.set_xlabel("Trajectory step")

ax1.grid(True)
ax2.grid(True)

ax1.legend()
ax2.legend()

plt.tight_layout()
plt.show()