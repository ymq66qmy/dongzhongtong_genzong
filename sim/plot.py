"""Usage: python sim/plot.py trace.csv sim/tracking.png"""
import csv
import sys

import matplotlib.pyplot as plt

with open(sys.argv[1], newline="") as stream:
    rows = list(csv.DictReader(stream))

def values(key):
    return [float(row[key]) for row in rows]

time = values("time_s")
states = ["POINT", "NEAR", "SWEEP", "REFINE", "VERIFY_PEAK", "CAL_AIR", "TRACK", "RECOVER"]
fig, axes = plt.subplots(3, 1, figsize=(10, 6.8), sharex=True, constrained_layout=True)
for ax in axes:
    ax.axvline(45, color="#337595", linestyle="--", linewidth=1)
    ax.axvspan(145, 183, color="#eeb95b", alpha=0.25)
    ax.grid(alpha=0.2)
axes[0].plot(time, values("rssi"), color="#135c69", linewidth=0.8)
axes[0].axhline(-116, color="#ad7533", linestyle=":", linewidth=0.8, label="RSSI-only detection")
axes[0].axhline(-88, color="#888888", linestyle="--", linewidth=0.8, label="Lock threshold")
axes[0].axhline(-96, color="#999999", linestyle=":", linewidth=0.8, label="Release threshold")
axes[0].set_ylabel("Beacon level")
axes[0].legend(loc="lower left", ncol=2, fontsize=8)
axes[1].step(time, [states.index(row["mode"]) for row in rows], where="post", color="#2259a5")
axes[1].set_yticks(range(len(states)), states)
axes[2].plot(time, values("error_deg"), color="#135c69", linewidth=0.9, label="Ground")
axes[2].plot(time, values("air_error_deg"), color="#a75b25", linewidth=0.9, label="Air")
axes[2].legend(loc="upper right", ncol=2, fontsize=8)
axes[2].set_yscale("symlog", linthresh=3)
axes[2].set_ylabel("Two-axis error (deg)")
axes[2].set_xlabel("Time (s)")
axes[2].set_xlim(0, 285)
fig.suptitle("Ground acquisition, takeoff at 45 s and beacon outage at 145–183 s")
fig.savefig(sys.argv[2], dpi=150)

