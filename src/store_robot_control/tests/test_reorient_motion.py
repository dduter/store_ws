from pathlib import Path
import ast

import pytest

from store_robot_control.core.reorient_motion import (
    ReorientPlanningError, plan_edge_pivot_flip, plan_gentle_flip, plan_reorient, plan_two_stage_flip,
)
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
    assert points[0].orientation_zyz == [0.0, 180.0, rz]
    assert points[3].orientation_zyz == [0.0, 180.0, 0.0]
    assert points[4].orientation_zyz == points[12].orientation_zyz == [90.0, 90.0, 90.0]
    assert [point.stage for point in points if point.check_grip] == ["grasp", "grasp_stage2"]
    assert [point.stage for point in points if point.action == "release"] == [
        "release_stage1", "release_flipped",
    ]


def test_two_stage_flip_negative_y_tilts_toward_negative_y(guard):
    points = plan_two_stage_flip([350, -100, 40, 30], guard)
    assert points[4].orientation_zyz == points[12].orientation_zyz == [-90.0, 90.0, -90.0]
    assert points[8].pose[:2] == [350, -135]


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


@pytest.mark.parametrize("j5, delta", [(90.0, -40.0), (-90.0, 40.0), (0.0, -40.0)])
def test_gentle_flip(guard, j5, delta):
    points = plan_gentle_flip([350.0, 20.0, 200.0, 25.0], j5, 30.0, guard)
    assert [point.stage for point in points] == [
        "approach", "descend", "grasp", "lift", "gentle_tilt_j5",
        "place_flip", "release_flip", "lift_after_flip", "restore_j5", "retreat",
    ]
    assert points[0].pose[2] == 280
    assert points[1].pose[2] == points[2].pose[2] == 200
    assert points[3].pose[2] == 290
    assert points[4].pose[2] > 290
    assert points[5].pose[2] == points[6].pose[2] == 255
    assert points[7].pose[2] == points[4].pose[2]
    assert points[8].pose[2] == 290
    assert points[9].pose[2] == 280
    assert points[2].check_grip
    assert points[4].action == points[8].action == "move_joint_j5"
    assert points[4].j5_delta_deg == -points[8].j5_delta_deg == delta
    assert points[6].action == "release"
    assert all(point.j6_deg == 30 and point.orientation_zyz is None for point in points)


@pytest.mark.parametrize("x,z,j5,j6,kwargs,code", [
    (350, 10, 90, 0, {}, "Z_MIN_VIOLATION"),
    (350, 320, 90, 0, {}, "OUT_OF_REACH"),
    (350, 260, 90, 0, {}, "OUT_OF_REACH"),   # lift_z(350) + J5 원호 상승(~56mm) > z_max(400)
    (600, 40, 90, 0, {}, "OUT_OF_REACH"),    # x(600) + J5 원호 전방(~154mm) > x_max(720)
    (350, 40, 126, 0, {}, "OUT_OF_REACH"),
    (350, 40, 90, 151, {}, "J6_LIMIT_VIOLATION"),
    (350, 40, float("nan"), 0, {}, "INVALID_POSE"),
    (350, 40, 90, 0, {"tilt_deg": 56}, "INVALID_POSE"),
    (350, 40, 90, 0, {"flip_lift_delta_mm": 0}, "INVALID_POSE"),
])
def test_gentle_flip_rejects_invalid_plan(guard, x, z, j5, j6, kwargs, code):
    with pytest.raises(ReorientPlanningError) as exc:
        plan_gentle_flip([x, 20, z, 0], j5, j6, guard, **kwargs)
    assert exc.value.error_code == code


@pytest.mark.parametrize(("j5", "delta"), [(90.0, -35.0), (-90.0, 35.0)])
def test_edge_pivot_flip(guard, j5, delta):
    points = plan_edge_pivot_flip([332.5, 0.0, 40.0, 25.0], j5, 30.0, guard)
    assert [point.stage for point in points] == [
        "approach", "descend", "grasp", "pivot_lift", "pivot_tilt_j5",
        "drag_over", "release_flip", "lift_after_flip", "restore_j5", "retreat",
    ]
    assert points[0].pose == [297.5, 0.0, 260.0, 25.0]
    assert points[1].pose == points[2].pose == [297.5, 0.0, 40.0, 25.0]
    assert points[2].action == "grasp" and points[2].check_grip
    assert points[3].pose == points[8].pose == [315.0, 0.0, 110.0, 25.0]
    assert points[4].pose == pytest.approx([522.061630, 0.0, 175.286122, 25.0], abs=1e-3)
    assert points[4].action == points[8].action == "move_joint_j5"
    assert (points[4].j5_delta_deg, points[8].j5_delta_deg) == (delta, -delta)
    assert points[5].pose == points[6].pose == pytest.approx([577.061630, 0.0, 65.0, 25.0], abs=1e-3)
    assert points[6].action == "release"
    assert points[7].pose == pytest.approx([577.061630, 0.0, 175.286122, 25.0], abs=1e-3)
    assert points[9].pose == [332.5, 0.0, 260.0, 25.0]
    assert all(point.j6_deg == 30.0 for point in points)


@pytest.mark.parametrize("j5", [54.0, -54.0, 30.0, -30.0, 126.0])
def test_edge_pivot_flip_rejects_unsafe_j5(guard, j5):
    with pytest.raises(ReorientPlanningError) as exc:
        plan_edge_pivot_flip([332.5, 0.0, 40.0, 0.0], j5, 0.0, guard)
    assert exc.value.error_code == "OUT_OF_REACH"


@pytest.mark.parametrize("x", [500.0, 550.0])
def test_edge_pivot_flip_rejects_workspace_boundary(guard, x):
    with pytest.raises(ReorientPlanningError) as exc:
        plan_edge_pivot_flip([x, 0.0, 40.0, 0.0], 90.0, 0.0, guard)
    assert exc.value.error_code == "OUT_OF_REACH"


@pytest.mark.parametrize("kwargs", [{"drag_forward_mm": float("inf")}, {"min_j5_abs_deg": 14.0}])
def test_edge_pivot_flip_rejects_invalid_settings(guard, kwargs):
    with pytest.raises(ReorientPlanningError) as exc:
        plan_edge_pivot_flip([332.5, 0.0, 40.0, 0.0], 90.0, 0.0, guard, **kwargs)
    assert exc.value.error_code == "INVALID_POSE"


def test_edge_pivot_flip_rejects_j6_limit(guard):
    with pytest.raises(ReorientPlanningError) as exc:
        plan_edge_pivot_flip([332.5, 0.0, 40.0, 0.0], 90.0, 151.0, guard)
    assert exc.value.error_code == "J6_LIMIT_VIOLATION"
