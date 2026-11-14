"""
estimation.py

A bank of independent two-state Kalman filters (angle, angular velocity), one per
joint, built on ``PolisherKalmanFilter`` from the Polisher-State-Estimator project.
No ROS code here, so it can be tested and reused outside ROS.

Each joint reports only a noisy angle (like an incremental encoder); the filter
infers a smooth angle and the joint velocity. The time step is taken from the
message time stamps on every update, so irregular timing is handled.
"""
from __future__ import annotations

import math

from .kalman_filter import PolisherKalmanFilter

__all__ = ["JointKalmanBank", "RmseTracker"]


class JointKalmanBank:
    """Per-joint Linear Kalman Filters driven by time-stamped angle measurements.

    Parameters
    ----------
    angle_noise_std : standard deviation of the angle measurement noise, rad (R = std^2)
    process_noise_q : process-noise intensity q, (rad/s^2)^2 * s. Larger = follows
        accelerations faster but passes more sensor noise.
    max_gap_s : if two samples are further apart than this, the filter restarts
    """

    def __init__(self, angle_noise_std: float = 0.002, process_noise_q: float = 0.1, max_gap_s: float = 1.0):
        if angle_noise_std <= 0.0 or process_noise_q <= 0.0:
            raise ValueError("angle_noise_std and process_noise_q must be > 0")
        self.r = float(angle_noise_std) ** 2
        self.q = float(process_noise_q)
        self.max_gap_s = float(max_gap_s)
        self._filters: dict[str, PolisherKalmanFilter] = {}
        self._last_t: dict[str, float] = {}

    def update(self, name: str, t: float, angle: float):
        """Feed one measurement. Returns ``(angle_estimate, velocity_estimate)``."""
        kf = self._filters.get(name)
        last = self._last_t.get(name)
        if kf is not None and last is not None:
            if t <= last:  # out-of-order or duplicate stamp: keep the current estimate
                return kf.angle, kf.velocity
            if (t - last) <= self.max_gap_s:
                est = kf.step(t - last, angle)
                self._last_t[name] = t
                return est
        # first sample, or a long gap: (re)start the filter at this measurement
        kf = PolisherKalmanFilter(self.q, self.r, initial_angle=angle, initial_velocity=0.0)
        self._filters[name] = kf
        self._last_t[name] = t
        return kf.angle, kf.velocity

    def reset(self) -> None:
        self._filters.clear()
        self._last_t.clear()


class RmseTracker:
    """Running RMSE of raw finite-difference velocity and filtered velocity vs the truth."""

    def __init__(self):
        self.n = 0
        self._raw = 0.0
        self._filt = 0.0
        self._ang_raw = 0.0
        self._ang_filt = 0.0

    def add(self, raw_vel: float, filt_vel: float, true_vel: float,
            raw_angle: float, filt_angle: float, true_angle: float) -> None:
        self._raw += (raw_vel - true_vel) ** 2
        self._filt += (filt_vel - true_vel) ** 2
        self._ang_raw += (raw_angle - true_angle) ** 2
        self._ang_filt += (filt_angle - true_angle) ** 2
        self.n += 1

    def summary(self) -> dict:
        if self.n == 0:
            return {"n": 0}
        f = lambda s: math.sqrt(s / self.n)
        return {
            "n": self.n,
            "velocity_rmse_raw": f(self._raw),
            "velocity_rmse_filtered": f(self._filt),
            "angle_rmse_raw": f(self._ang_raw),
            "angle_rmse_filtered": f(self._ang_filt),
        }
