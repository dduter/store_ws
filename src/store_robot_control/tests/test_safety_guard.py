from pathlib import Path

import pytest

from store_robot_control.core.safety_guard import SafetyGuard
from conftest import TEST_WORKSPACE


@pytest.fixture
def guard():
    return SafetyGuard(TEST_WORKSPACE)


@pytest.mark.parametrize(("pose", "code"), [
    ([180, 0, 15, 0], ""),
    ([720, 0, 400, 0], ""),
    ([200, 0, 14.99, 0], "Z_MIN_VIOLATION"),
    ([179, 0, 15, 0], "OUT_OF_REACH"),
    ([300, 421, 15, 0], "OUT_OF_REACH"),
    ([300, 0, 401, 0], "OUT_OF_REACH"),
    ([720, 420, 15, 0], "OUT_OF_REACH"),
    ([300, 0, 15, float("nan")], "INVALID_POSE"),
    ([300, 0, 15], "INVALID_POSE"),
])
def test_pose(guard, pose, code):
    result = guard.validate_pose(pose)
    assert result.ok == (not code)
    assert result.error_code == code


def test_j6_bounds(guard):
    for edge in (-150.0, 150.0):
        assert guard.check_j6_limit(edge)
        assert not guard.check_j6_limit(edge * 1.001)
        assert guard.clamp_j6_angle(edge * 2) == edge


def test_transit_height(guard):
    assert guard.safe_transit_z(15) == 260
    assert guard.safe_transit_z(250) == 330
    assert guard.safe_transit_z(390) == 400


def test_emergency_stop(guard):
    assert not guard.is_emergency_stopped()
    guard.set_emergency_stop(True)
    assert guard.is_emergency_stopped()
    guard.set_emergency_stop(False)
    assert not guard.is_emergency_stopped()
