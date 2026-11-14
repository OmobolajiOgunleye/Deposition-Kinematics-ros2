"""Tests for the per-joint Kalman filter bank (no ROS needed)."""
import numpy as np
import pytest

from deposition_kinematics_ros2.estimation import JointKalmanBank, RmseTracker
from deposition_kinematics_ros2.trajectory import build_arm, build_path, build_trajectory


def _run(traj, sigma, q, seed=0):
    rng = np.random.default_rng(seed)
    noisy = traj.q + rng.normal(0.0, sigma, traj.q.shape)
    bank = JointKalmanBank(sigma, q)
    vel = np.zeros_like(traj.q)
    ang = np.zeros_like(traj.q)
    for i, t in enumerate(traj.t):
        for j in range(traj.q.shape[1]):
            ang[i, j], vel[i, j] = bank.update(f"joint{j + 1}", float(t), float(noisy[i, j]))
    raw = np.vstack([np.zeros((1, traj.q.shape[1])), np.diff(noisy, axis=0) * traj.rate_hz])
    return noisy, ang, vel, raw


@pytest.mark.parametrize("kind,dof,tool", [("circle", 3, "tangent"), ("spiral", 2, "fixed")])
def test_kalman_velocity_beats_finite_difference(kind, dof, tool):
    traj = build_trajectory(build_arm(dof), build_path(kind, 2), speed=25.0, tool_mode=tool)
    _, ang, vel, raw = _run(traj, sigma=0.002, q=0.1)
    skip = 50  # ignore the start-up transient
    rmse_raw = np.sqrt(np.mean((raw[skip:] - traj.qdot[skip:]) ** 2))
    rmse_kf = np.sqrt(np.mean((vel[skip:] - traj.qdot[skip:]) ** 2))
    assert rmse_kf < 0.6 * rmse_raw


def test_angle_estimate_is_better_than_the_noisy_angle():
    traj = build_trajectory(build_arm(2), build_path("spiral", 2), speed=25.0, tool_mode="fixed")
    noisy, ang, _, _ = _run(traj, sigma=0.002, q=0.1)
    assert np.sqrt(np.mean((ang - traj.q) ** 2)) < np.sqrt(np.mean((noisy - traj.q) ** 2))


def test_restarts_after_a_long_gap_and_ignores_old_stamps():
    bank = JointKalmanBank(0.01, 0.1, max_gap_s=1.0)
    bank.update("joint1", 0.0, 0.0)
    a, v = bank.update("joint1", 0.02, 0.0)
    a2, v2 = bank.update("joint1", 0.01, 5.0)  # older stamp: ignored
    assert (a2, v2) == (a, v)
    a3, v3 = bank.update("joint1", 10.0, 3.0)  # long gap: restart at the new reading
    assert a3 == pytest.approx(3.0) and v3 == 0.0


def test_invalid_noise_parameters_raise():
    with pytest.raises(ValueError):
        JointKalmanBank(0.0, 0.1)
    with pytest.raises(ValueError):
        JointKalmanBank(0.01, -1.0)


def test_rmse_tracker_summary():
    tr = RmseTracker()
    assert tr.summary() == {"n": 0}
    tr.add(1.0, 0.5, 0.0, 0.2, 0.1, 0.0)
    s = tr.summary()
    assert s["n"] == 1 and s["velocity_rmse_raw"] == pytest.approx(1.0) and s["velocity_rmse_filtered"] == pytest.approx(0.5)
