from pathlib import Path

import pytest

from store_robot_control.core.reorient_motion import ReorientPlanningError, plan_reorient
from store_robot_control.core.safety_guard import SafetyGuard
from conftest import TEST_WORKSPACE


@pytest.fixture
def guard():
    return SafetyGuard(TEST_WORKSPACE)


@pytest.mark.parametrize(("start", "end", "rz"), [(-150, 30, 190), (150, -30, -170)])
def test_flip_both_directions(guard, start, end, rz):
    waypoints = plan_reorient([300, 0, 10, 10], start, guard)
    assert [point.stage for point in waypoints] == [
        "approach", "descend", "grasp", "lift", "flip_180", "place", "release", "retreat",
    ]
    assert [point.pose[2] for point in waypoints] == [260, 15, 15, 220, 220, 15, 15, 260]
    assert [point.j6_deg for point in waypoints] == [start] * 4 + [end] * 4
    assert waypoints[-1].pose[3] == rz


@pytest.mark.parametrize("angle", [0, -30, 30, -151, 151])
def test_no_safe_flip(guard, angle):
    with pytest.raises(ReorientPlanningError) as exc:
        plan_reorient([300, 0, 40, 0], angle, guard)
    assert exc.value.error_code == "J6_LIMIT_VIOLATION"


@pytest.mark.parametrize("bad_pose", [
    [300, 0, float("nan"), 0],
    [300, 0, float("-inf"), 0],
])
def test_non_finite_pose_rejected_before_clamping(guard, bad_pose):
    with pytest.raises(ReorientPlanningError) as exc:
        plan_reorient(bad_pose, -150, guard)
    assert exc.value.error_code == "INVALID_POSE"
