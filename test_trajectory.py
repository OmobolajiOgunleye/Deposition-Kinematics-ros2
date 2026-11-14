"""Tests for the trajectory builder (no ROS needed)."""
import numpy as np
import pytest

from deposition_kinematics_ros2.trajectory import build_arm, build_path, build_trajectory

CASES = [("circle", 3, "tangent"), ("circle", 2, "fixed"), ("spiral", 2, "fixed"), ("spiral", 3, "fixed")]


@pytest.fixture(scope="module", params=CASES, ids=lambda c: f"{c[0]}-{c[1]}R-{c[2]}")
def case(request):
    kind, dof, tool = request.param
    arm = build_arm(dof)
    path = build_path(kind, 3)
    traj = build_trajectory(arm, path, speed=25.0, rate_hz=50.0, tool_mode=tool)
    return arm, path, traj


def test_starts_and_ends_at_home(case):
    _, _, traj = case
    assert np.allclose(traj.q[0], 0.0)
    assert np.allclose(traj.q[-1], 0.0)
    assert traj.z[0] == pytest.approx(0.0)
    assert traj.z[-1] == pytest.approx(0.0)


def test_starts_and_ends_at_rest(case):
    _, _, traj = case
    assert np.allclose(traj.qdot[0], 0.0) and np.allclose(traj.qdot[-1], 0.0)
    assert traj.zdot[0] == 0.0 and traj.zdot[-1] == 0.0


def test_time_is_uniform_and_increasing(case):
    _, _, traj = case
    dt = np.diff(traj.t)
    assert np.all(dt > 0)
    assert np.allclose(dt[:-1], 1.0 / 50.0)


def test_joint_motion_is_continuous(case):
    """No jumps, in particular no 2*pi wrist jump between layers."""
    _, _, traj = case
    step = np.abs(np.diff(traj.q, axis=0)).max()
    assert step * traj.rate_hz < 3.2  # rad/s: 2 rad/s budget x 1.5 cubic-ease peak, plus margin


def test_tool_follows_the_toolpath_while_depositing(case):
    arm, path, traj = case
    idx = np.flatnonzero(traj.depositing)[::7]
    for i in idx:
        x, y, _ = arm.fk(traj.q[i])
        layer_pts = path.xyz[path.layer == traj.layer[i], :2]
        d = np.min(np.linalg.norm(layer_pts - np.array([x, y]), axis=1))
        assert d < 1.01  # points are 2 mm apart, so the nearest one is within about 1 mm


def test_deposition_speed_is_the_commanded_speed(case):
    arm, _, traj = case
    idx = np.flatnonzero(traj.depositing[1:] & traj.depositing[:-1] & (traj.layer[1:] == traj.layer[:-1]))[::11]
    speeds = []
    for i in idx:
        a = arm.fk(traj.q[i])[:2]
        b = arm.fk(traj.q[i + 1])[:2]
        speeds.append(np.linalg.norm(b - a) * traj.rate_hz)
    assert np.median(speeds) == pytest.approx(25.0, rel=0.03)


def test_z_axis_steps_one_layer_height_per_layer(case):
    _, path, traj = case
    zs = [traj.z[traj.depositing & (traj.layer == k)][0] for k in range(traj.n_layers)]
    assert np.allclose(np.diff(zs), path.layer_height)
    for k in range(traj.n_layers):
        zk = traj.z[traj.depositing & (traj.layer == k)]
        assert np.allclose(zk, zk[0])  # Z does not move while depositing


def test_at_interpolates_and_clips(case):
    _, _, traj = case
    q, qd, z, zd, dep, layer = traj.at(-5.0)
    assert np.allclose(q, traj.q[0])
    q, qd, z, zd, dep, layer = traj.at(traj.duration + 5.0)
    assert np.allclose(q, traj.q[-1])
    mid = traj.t[len(traj.t) // 2]
    q, *_ = traj.at(mid)
    assert np.allclose(q, traj.q[len(traj.t) // 2], atol=1e-9)


def test_bad_arguments_raise():
    with pytest.raises(ValueError):
        build_arm(4)
    with pytest.raises(ValueError):
        build_path("triangle")
    with pytest.raises(ValueError):
        build_trajectory(build_arm(2), build_path("circle", 2), speed=0.0)
