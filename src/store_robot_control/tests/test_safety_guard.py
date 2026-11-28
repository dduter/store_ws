import pytest

from store_robot_control.core.safety_guard import SafetyGuard
from conftest import REPO_CONFIG, TEST_WORKSPACE


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


def test_zone_minimums(zoned_workspace):
    guard = SafetyGuard(zoned_workspace)
    assert guard.z_min_at(350.0, 0.0) == -18.0
    assert guard.validate_pose([350.0, 0.0, -18.0, 0.0]).ok
    assert guard.validate_pose([350.0, 0.0, -18.1, 0.0]).error_code == "Z_MIN_VIOLATION"
    assert guard.z_min_at(230.9, -486.3) == -115.0
    assert guard.validate_pose([230.9, -486.3, -90.6, 0.0]).ok
    assert guard.validate_pose([230.9, -486.3, -115.1, 0.0]).error_code == "Z_MIN_VIOLATION"


def test_overlapping_zones_use_stricter_minimum_at_inclusive_boundary(zoned_workspace):
    zoned_workspace["safety"]["z_min_zones"].append({
        "x_range": [200.0, 250.0], "y_range": [-400.0, -320.0], "z_min_mm": -25.0,
    })
    guard = SafetyGuard(zoned_workspace)
    assert guard.z_min_at(250.0, -320.0) == -25.0
    assert guard.z_min_at(251.0, -320.0) == -115.0


@pytest.mark.parametrize(("key", "value"), [
    ("x_range", [1.0]),
    ("x_range", [2.0, 1.0]),
    ("x_range", [False, 1.0]),
    ("y_range", [0.0, float("inf")]),
    ("y_range", ["0", "1"]),
    ("z_min_mm", float("nan")),
    ("z_min_mm", 401.0),
    ("z_min_mm", None),
])
def test_invalid_zone_fields(zoned_workspace, key, value):
    zoned_workspace["safety"]["z_min_zones"][0][key] = value
    with pytest.raises(ValueError, match=key):
        SafetyGuard(zoned_workspace)


@pytest.mark.parametrize("zones", [None, {}, ["zone"], [{}]])
def test_invalid_zone_structure(zoned_workspace, zones):
    zoned_workspace["safety"]["z_min_zones"] = zones
    with pytest.raises(ValueError, match="z_min_zones"):
        SafetyGuard(zoned_workspace)


def test_real_workspace_config_loads():
    guard = SafetyGuard(REPO_CONFIG / "workspace_zones.yaml")
    assert guard.validate_pose([350.0, 0.0, 40.0, 0.0]).ok
