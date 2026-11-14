"""
trajectory.py

Turns a layered deposition toolpath into a smooth, fixed-rate joint trajectory
that a ROS 2 node can play back. It contains no ROS code, so it is unit-tested
without a ROS installation.

The trajectory is built in segments:

    home -> approach -> layer 0 -> layer change -> layer 1 -> ... -> return home

* During a layer the arm follows the toolpath at the commanded deposition speed
  (joint angles come from ``deposition.solve_toolpath``, i.e. the inverse
  kinematics from the Deposition-Kinematics project).
* Between layers the Z axis lifts by one layer height while the joints move to
  the start of the next layer with a smooth (cubic ease) profile. In the original
  toolkit the 3R wrist unwinds a full turn here; this keeps the joint states
  continuous, so a state estimator never sees a 2*pi jump.

Units: millimetres and radians (the ROS nodes convert to metres).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .deposition import solve_toolpath
from .kinematics import PlanarArm
from .toolpaths import Toolpath, circle_layers, spiral_layers

__all__ = [
    "ARM_2R",
    "ARM_3R",
    "PART_CENTER",
    "Trajectory",
    "build_arm",
    "build_path",
    "build_trajectory",
]

# Same demo geometry as the Deposition-Kinematics project (mm). The URDF files in
# urdf/ use the same link lengths. Change both together.
ARM_2R = [200.0, 150.0]
ARM_3R = [200.0, 150.0, 60.0]
PART_CENTER = (240.0, 0.0)


@dataclass
class Trajectory:
    """A fixed-rate joint trajectory.

    Attributes
    ----------
    t : (M,) time stamps, s
    q : (M, n) joint angles, rad
    qdot : (M, n) joint velocities, rad/s
    z : (M,) height of the Z axis, mm
    zdot : (M,) Z velocity, mm/s
    depositing : (M,) True while the tool follows a layer toolpath
    layer : (M,) layer index (0-based; during a layer change it is the layer that starts next)
    rate_hz : sampling rate, Hz
    """

    t: np.ndarray
    q: np.ndarray
    qdot: np.ndarray
    z: np.ndarray
    zdot: np.ndarray
    depositing: np.ndarray
    layer: np.ndarray
    rate_hz: float
    n_layers: int

    @property
    def duration(self) -> float:
        return float(self.t[-1])

    @property
    def n_joints(self) -> int:
        return int(self.q.shape[1])

    def at(self, t: float):
        """Interpolated state at time ``t`` (clipped to the trajectory).

        Returns ``(q, qdot, z, zdot, depositing, layer)``.
        """
        t = float(np.clip(t, 0.0, self.t[-1]))
        q = np.array([np.interp(t, self.t, self.q[:, j]) for j in range(self.n_joints)])
        qd = np.array([np.interp(t, self.t, self.qdot[:, j]) for j in range(self.n_joints)])
        z = float(np.interp(t, self.t, self.z))
        zd = float(np.interp(t, self.t, self.zdot))
        i = int(min(np.searchsorted(self.t, t, side="right") - 1, len(self.t) - 1))
        i = max(i, 0)
        return q, qd, z, zd, bool(self.depositing[i]), int(self.layer[i])


def build_arm(dof: int) -> PlanarArm:
    """The demo arm: 2R (200, 150 mm) or 3R (200, 150, 60 mm)."""
    if dof == 2:
        return PlanarArm(ARM_2R)
    if dof == 3:
        return PlanarArm(ARM_3R)
    raise ValueError("dof must be 2 or 3")


def build_path(kind: str, n_layers: int = 0) -> Toolpath:
    """Demo toolpaths from the Deposition-Kinematics project.

    ``n_layers = 0`` keeps the project default (16 for the circle wall, 5 for the spiral fill).
    """
    if kind == "circle":
        return circle_layers(
            center=PART_CENTER, radius=60.0, n_layers=n_layers or 16, layer_height=2.5, ds=2.0
        )
    if kind == "spiral":
        return spiral_layers(
            center=PART_CENTER, r_out=60.0, r_in=6.0, bead_pitch=4.0,
            n_layers=n_layers or 5, layer_height=2.5, ds=2.0,
        )
    raise ValueError("path must be 'circle' or 'spiral'")


def _ease(u: np.ndarray) -> np.ndarray:
    """Cubic ease-in/ease-out: zero velocity at both ends."""
    return 3.0 * u ** 2 - 2.0 * u ** 3


def build_trajectory(
    arm: PlanarArm,
    path: Toolpath,
    speed: float = 25.0,
    rate_hz: float = 50.0,
    tool_mode: str = "fixed",
    fixed_phi: float = 0.0,
    elbow: str = "up",
    lift_speed: float = 5.0,
    max_transition_joint_speed: float = 2.0,
    min_transition_s: float = 0.3,
) -> Trajectory:
    """Build the full joint trajectory for ``path`` (see the module docstring).

    Parameters
    ----------
    speed : deposition travel speed along the layer, mm/s
    rate_hz : sampling rate of the returned trajectory
    lift_speed : Z-axis speed during a layer change, mm/s
    max_transition_joint_speed : joint speed budget for moves that are not deposition, rad/s
    min_transition_s : shortest allowed non-deposition move, s
    """
    if speed <= 0 or rate_hz <= 0 or lift_speed <= 0 or max_transition_joint_speed <= 0:
        raise ValueError("speed, rate_hz, lift_speed and max_transition_joint_speed must be > 0")

    res = solve_toolpath(arm, path, speed=speed, tool_mode=tool_mode, fixed_phi=fixed_phi, elbow=elbow)
    layers = np.unique(path.layer)

    def move_duration(qa, za, qb, zb) -> float:
        dq = float(np.max(np.abs(qb - qa)))
        dz = abs(zb - za)
        return max(min_transition_s, dq / max_transition_joint_speed, dz / lift_speed)

    # Each segment is (kind, t_start, t_end, payload). kind: "dep" or "move".
    segments = []
    q_home = np.zeros(arm.n)
    z_home = float(path.xyz[0, 2])

    first = np.flatnonzero(path.layer == layers[0])
    t0 = 0.0
    dur = move_duration(q_home, z_home, res.q[first[0]], path.xyz[first[0], 2])
    segments.append(("move", t0, t0 + dur, (q_home, z_home, res.q[first[0]], float(path.xyz[first[0], 2]), int(layers[0]))))
    t0 += dur

    for li, k in enumerate(layers):
        idx = np.flatnonzero(path.layer == k)
        xy = path.xyz[idx, :2]
        s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))])
        t_knots = t0 + s / speed
        segments.append(("dep", t_knots[0], t_knots[-1], (t_knots, res.q[idx], float(path.xyz[idx[0], 2]), int(k))))
        t0 = float(t_knots[-1])
        if li + 1 < len(layers):
            nxt = np.flatnonzero(path.layer == layers[li + 1])[0]
            qa, za = res.q[idx[-1]], float(path.xyz[idx[-1], 2])
            qb, zb = res.q[nxt], float(path.xyz[nxt, 2])
            dur = move_duration(qa, za, qb, zb)
            segments.append(("move", t0, t0 + dur, (qa, za, qb, zb, int(layers[li + 1]))))
            t0 += dur

    last = np.flatnonzero(path.layer == layers[-1])[-1]
    qa, za = res.q[last], float(path.xyz[last, 2])
    dur = move_duration(qa, za, q_home, z_home)
    segments.append(("move", t0, t0 + dur, (qa, za, q_home, z_home, int(layers[-1]))))
    t_end = t0 + dur

    n = int(np.floor(t_end * rate_hz)) + 1
    tt = np.arange(n) / rate_hz
    if t_end - tt[-1] > 1e-9:
        tt = np.append(tt, t_end)
    M = len(tt)
    q = np.zeros((M, arm.n))
    z = np.zeros(M)
    dep = np.zeros(M, dtype=bool)
    lay = np.zeros(M, dtype=int)

    for si, (kind, ts, te, payload) in enumerate(segments):
        last_seg = si == len(segments) - 1
        mask = (tt >= ts) & ((tt <= te) if last_seg else (tt < te))
        if not mask.any():
            continue
        if kind == "dep":
            t_knots, q_knots, z_layer, k = payload
            for j in range(arm.n):
                q[mask, j] = np.interp(tt[mask], t_knots, q_knots[:, j])
            z[mask] = z_layer
            dep[mask] = True
            lay[mask] = k
        else:
            qa, za, qb, zb, k = payload
            u = _ease((tt[mask] - ts) / (te - ts))
            q[mask] = qa[None, :] + u[:, None] * (qb - qa)[None, :]
            z[mask] = za + u * (zb - za)
            lay[mask] = k

    qdot = np.gradient(q, tt, axis=0)
    zdot = np.gradient(z, tt)
    # The trajectory starts and ends at rest (cubic ease), but a one-sided finite difference
    # at the two end samples would report a small spurious speed. Force it to zero.
    qdot[0] = qdot[-1] = 0.0
    zdot[0] = zdot[-1] = 0.0
    return Trajectory(
        t=tt, q=q, qdot=qdot, z=z, zdot=zdot, depositing=dep, layer=lay,
        rate_hz=float(rate_hz), n_layers=int(len(layers)),
    )
