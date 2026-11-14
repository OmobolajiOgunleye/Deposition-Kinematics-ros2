"""Regenerate docs/joint_estimation.png (needs matplotlib).

Replays the same estimator class the ROS node uses on the same trajectory, offline,
so no ROS installation is needed:   python tools/make_figure.py
"""
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from deposition_kinematics_ros2.estimation import JointKalmanBank  # noqa: E402
from deposition_kinematics_ros2.trajectory import build_arm, build_path, build_trajectory  # noqa: E402

SIGMA, Q, SPEED, LAYERS = 0.002, 0.1, 25.0, 3
traj = build_trajectory(build_arm(3), build_path("circle", LAYERS), speed=SPEED, tool_mode="tangent")
rng = np.random.default_rng(42)
noisy = traj.q + rng.normal(0.0, SIGMA, traj.q.shape)
raw = np.vstack([np.zeros((1, 3)), np.diff(noisy, axis=0) * traj.rate_hz])
bank = JointKalmanBank(SIGMA, Q)
kf = np.zeros_like(traj.q)
for i, t in enumerate(traj.t):
    for j in range(3):
        kf[i, j] = bank.update(f"joint{j + 1}", float(t), float(noisy[i, j]))[1]

rmse_raw = np.sqrt(np.mean((raw - traj.qdot) ** 2))
rmse_kf = np.sqrt(np.mean((kf - traj.qdot) ** 2))

fig, ax = plt.subplots(2, 1, figsize=(10, 6.5), gridspec_kw={"height_ratios": [2, 1]})
j = 0
ax[0].plot(traj.t, raw[:, j], color="#bbbbbb", lw=0.8, label="finite difference of noisy angle")
ax[0].plot(traj.t, kf[:, j], color="#d9531e", lw=1.4, label="Kalman estimate")
ax[0].plot(traj.t, traj.qdot[:, j], color="#1f3b73", lw=1.4, ls="--", label="true velocity")
ax[0].set_ylabel("joint 1 velocity (rad/s)")
ax[0].set_title(f"Joint-velocity estimation from a noisy encoder (sigma = {SIGMA} rad, circle wall, 3R arm, {SPEED:g} mm/s)")
ax[0].legend(loc="lower right")
ax[0].set_xlabel("time (s)")
ax[0].grid(alpha=0.3)

ax[1].bar(["finite difference", "Kalman filter"], [rmse_raw, rmse_kf], color=["#bbbbbb", "#d9531e"])
for x, v in enumerate([rmse_raw, rmse_kf]):
    ax[1].text(x, v, f"{v:.3f}", ha="center", va="bottom")
ax[1].set_ylabel("velocity RMSE, all joints (rad/s)")
ax[1].set_ylim(0, rmse_raw * 1.18)
ax[1].grid(alpha=0.3, axis="y")
fig.tight_layout()
out = os.path.join(os.path.dirname(__file__), "..", "docs", "joint_estimation.png")
fig.savefig(out, dpi=130)
print(f"wrote {out}: velocity RMSE raw {rmse_raw:.4f} -> Kalman {rmse_kf:.4f} rad/s")
