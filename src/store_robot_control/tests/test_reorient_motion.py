from pathlib import Path
import ast

import pytest

from store_robot_control.core.reorient_motion import ReorientPlanningError, plan_reorient, plan_two_stage_flip
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


@pytest.mark.parametrize("rz", [0.0, -90.0, 90.0, 45.0, -45.0])
@pytest.mark.parametrize("target_z", [15.0, 170.0, 310.0])
def test_two_stage_flip(guard, rz, target_z):
    points = plan_two_stage_flip([350, 20, target_z, rz], guard)
    assert [point.stage for point in points] == [
        "approach", "descend", "grasp", "lift_stage1", "tilt_90", "place_standing",
        "release_stage1", "retreat_stage1", "reapproach_stage2", "descend_stage2",
        "grasp_stage2", "lift_stage2", "lay_down_180", "place_flipped",
        "release_flipped", "retreat",
    ]
    for point in points:
        assert guard.validate_pose(point.pose).ok
        assert guard.check_j6_limit(point.j6_deg)
        assert -150 < point.j6_deg < 150
        assert len(point.orientation_zyz) == 3
    assert points[3].pose[2] == max(260, target_z + 90)
    assert points[5].pose[2] == target_z + 20
    assert points[3].pose[2] > points[5].pose[2] > points[1].pose[2]
    assert points[13].pose[:3] == [350, 20, target_z]
    assert points[4].orientation_zyz == points[12].orientation_zyz == [-rz, 90, 0]
    assert [point.stage for point in points if point.check_grip] == ["grasp", "grasp_stage2"]
    assert [point.stage for point in points if point.action == "release"] == [
        "release_stage1", "release_flipped",
    ]


@pytest.mark.parametrize("target_z", [311.0, 390.0, 400.0])
def test_two_stage_flip_rejects_insufficient_lift_headroom(guard, target_z):
    with pytest.raises(ReorientPlanningError) as exc:
        plan_two_stage_flip([350, 20, target_z, 0.0], guard)
    assert exc.value.error_code == "OUT_OF_REACH"


def test_two_stage_flip_reapproach_falls_back_at_boundary(guard):
    points = plan_two_stage_flip([350, 420, 40, 0], guard)
    assert points[8].pose[:2] == [350, 420]


@pytest.mark.parametrize(("pose", "code"), [
    ([350, 0, float("nan"), 0], "INVALID_POSE"),
    ([350, 0, 40, float("inf")], "INVALID_POSE"),
    ([350, 0, 401, 0], "OUT_OF_REACH"),
    ([350, 0, 14, 0], "Z_MIN_VIOLATION"),
])
def test_two_stage_flip_rejects_invalid_pose(guard, pose, code):
    with pytest.raises(ReorientPlanningError) as exc:
        plan_two_stage_flip(pose, guard)
    assert exc.value.error_code == code


def test_core_has_no_ros_imports():
    core = Path(__file__).resolve().parents[1] / "store_robot_control" / "core"
    forbidden = {"rclpy", "store_interfaces"}
    for path in core.rglob("*.py"):
        for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
            if isinstance(node, ast.Import):
                assert all(alias.name.split(".")[0] not in forbidden for alias in node.names), path
            elif isinstance(node, ast.ImportFrom) and node.module:
                assert node.module.split(".")[0] not in forbidden, path
