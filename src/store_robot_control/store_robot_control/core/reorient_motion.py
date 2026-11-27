from dataclasses import dataclass
from math import cos, hypot, isfinite, radians, sin
from typing import Optional

from .safety_guard import SafetyGuard


@dataclass
class MotionWaypoint:
    stage: str
    pose: list[float]
    j6_deg: float
    orientation_zyz: Optional[list[float]] = None
    action: str = "move_linear"
    check_grip: bool = False
    j5_delta_deg: float = 0.0


class ReorientPlanningError(ValueError):
    def __init__(self, error_code, message):
        super().__init__(message)
        self.error_code = error_code


def plan_reorient(target_pose, current_j6_deg: float, safety_guard: SafetyGuard):
    try:
        x, y, target_z, rz = map(float, target_pose)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ReorientPlanningError("INVALID_POSE", "유한한 좌표 4개가 필요합니다.") from exc
    if not all(map(isfinite, (x, y, target_z, rz, float(current_j6_deg)))):
        raise ReorientPlanningError("INVALID_POSE", "좌표와 J6 각도는 유한해야 합니다.")
    guard = safety_guard
    z = max(guard.z_min_mm, target_z)
    lift_z = guard.poses["reorient_lift_z_mm"]
    if not guard.check_j6_limit(current_j6_deg):
        raise ReorientPlanningError("J6_LIMIT_VIOLATION", "현재 J6 각도가 허용 범위를 벗어났습니다.")
    for delta in (180.0, -180.0):
        final_j6 = current_j6_deg + delta
        if guard.j6_min_deg < final_j6 < guard.j6_max_deg:
            break
    else:
        raise ReorientPlanningError("J6_LIMIT_VIOLATION", "허용 범위 내에서 J6를 180도 회전할 수 없습니다.")
    if lift_z < z:
        raise ReorientPlanningError("OUT_OF_REACH", "회전 높이가 배치 높이보다 낮습니다.")
    waypoints = [
        MotionWaypoint("approach", [x, y, guard.safe_transit_z(z), rz], current_j6_deg),
        MotionWaypoint("descend", [x, y, z, rz], current_j6_deg),
        MotionWaypoint("grasp", [x, y, z, rz], current_j6_deg, action="grasp", check_grip=True),
        MotionWaypoint("lift", [x, y, lift_z, rz], current_j6_deg),
        MotionWaypoint("flip_180", [x, y, lift_z, rz + delta], final_j6, action="move_joint_j6"),
        MotionWaypoint("place", [x, y, z, rz + delta], final_j6),
        MotionWaypoint("release", [x, y, z, rz + delta], final_j6, action="release"),
        MotionWaypoint("retreat", [x, y, guard.safe_transit_z(z), rz + delta], final_j6),
    ]
    for waypoint in waypoints:
        check = guard.validate_pose(waypoint.pose)
        if not check.ok:
            raise ReorientPlanningError(check.error_code, check.message)
    return waypoints


def plan_two_stage_flip(target_pose, safety_guard: SafetyGuard, flip_lift_delta_mm=90.0,
                        standing_offset_z_mm=20.0, reapproach_offset_mm=35.0,
                        tilt_ry_deg=90.0):
    guard = safety_guard
    check = guard.validate_pose(target_pose)
    if not check.ok:
        raise ReorientPlanningError(check.error_code, check.message)
    x, y, target_z, rz = check.clamped_pose
    if not -90.0 <= rz <= 90.0:
        raise ReorientPlanningError("INVALID_POSE", "재정렬에 필요한 rz 범위는 ±90도입니다.")
    if not all(map(isfinite, (flip_lift_delta_mm, standing_offset_z_mm,
                              reapproach_offset_mm, tilt_ry_deg))):
        raise ReorientPlanningError("INVALID_POSE", "재정렬 오프셋과 기울기는 유한해야 합니다.")
    if flip_lift_delta_mm <= 0 or standing_offset_z_mm < 0:
        raise ReorientPlanningError("INVALID_POSE", "반전 상승 높이는 양수, 직립 오프셋은 0 이상이어야 합니다.")
    z_grasp = max(guard.z_min_mm, target_z)
    transit_z = guard.safe_transit_z(z_grasp)
    lift_z = max(z_grasp + flip_lift_delta_mm, guard.safe_transit_z_mm)
    z_stand = max(guard.z_min_mm, z_grasp + standing_offset_z_mm)
    if lift_z > guard.bounds["z_max"] or transit_z > guard.bounds["z_max"]:
        raise ReorientPlanningError(
            "OUT_OF_REACH",
            f"반전 상승 높이({lift_z:.1f}mm)가 상한({guard.bounds['z_max']:.1f}mm)을 초과합니다.",
        )
    if lift_z <= z_grasp or lift_z <= z_stand:
        raise ReorientPlanningError(
            "OUT_OF_REACH", "반전과 재파지에 필요한 상승 여유를 확보할 수 없습니다.",
        )

    # 실물 M0609 + RG2(240mm)는 전방(+X) 90° 기울기 시 J3(>160°) 또는 J5(>135°) 한계에 걸립니다.
    # 들어올릴 때 하향 기준(0, 180, 0)으로 정렬한 뒤, 로봇 중심 방향으로 손목이 접히도록
    # 측면(±Y, ZYZ=(±90, 90, ±90)) 90° 기울기를 두 번 누적해 180° 반전을 수행합니다.
    side = 1.0 if y >= 0.0 else -1.0
    top_down_pick = [0.0, 180.0, rz]
    top_down_flip = [0.0, 180.0, 0.0]
    tilted = [side * 90.0, tilt_ry_deg, side * 90.0]
    x_re = x
    y_re = y + side * reapproach_offset_mm
    if not guard.validate_pose([x_re, y_re, lift_z, 0.0]).ok:
        x_re, y_re = x, y

    def waypoint(stage, px, py, z, orientation, action="move_linear"):
        # J6는 하향 기준 자세를 기준으로 추정하며, 실행 중에는 실제 관절도 확인합니다.
        j6 = rz if orientation is top_down_pick else 0.0
        pose = [px, py, z, orientation[2]]
        check = guard.validate_pose(pose)
        if not check.ok:
            raise ReorientPlanningError(check.error_code, check.message)
        if not guard.check_j6_limit(j6):
            raise ReorientPlanningError("J6_LIMIT_VIOLATION", "예상 J6 각도가 허용 범위를 벗어났습니다.")
        return MotionWaypoint(stage, pose, j6, orientation.copy(), action, action == "grasp")

    return [
        waypoint("approach", x, y, transit_z, top_down_pick),
        waypoint("descend", x, y, z_grasp, top_down_pick),
        waypoint("grasp", x, y, z_grasp, top_down_pick, "grasp"),
        waypoint("lift_stage1", x, y, lift_z, top_down_flip),
        waypoint("tilt_90", x, y, lift_z, tilted),
        waypoint("place_standing", x, y, z_stand, tilted),
        waypoint("release_stage1", x, y, z_stand, tilted, "release"),
        waypoint("retreat_stage1", x, y, lift_z, tilted),
        waypoint("reapproach_stage2", x_re, y_re, lift_z, top_down_flip),
        waypoint("descend_stage2", x_re, y_re, z_stand, top_down_flip),
        waypoint("grasp_stage2", x_re, y_re, z_stand, top_down_flip, "grasp"),
        waypoint("lift_stage2", x_re, y_re, lift_z, top_down_flip),
        waypoint("lay_down_180", x, y, lift_z, tilted),
        waypoint("place_flipped", x, y, z_grasp, tilted),
        waypoint("release_flipped", x, y, z_grasp, tilted, "release"),
        waypoint("retreat", x, y, transit_z, tilted),
    ]


def plan_gentle_flip(target_pose, current_j5_deg: float, current_j6_deg: float,
                     safety_guard: SafetyGuard, tilt_deg: float = 40.0,
                     flip_lift_delta_mm: float = 90.0, max_j5_abs_deg: float = 125.0,
                     release_clearance_mm: float = 55.0, tool_length_mm: float = 361.0):
    guard = safety_guard
    check = guard.validate_pose(target_pose)
    if not check.ok:
        raise ReorientPlanningError(check.error_code, check.message)
    values = (
        current_j5_deg, current_j6_deg, tilt_deg, flip_lift_delta_mm,
        max_j5_abs_deg, release_clearance_mm, tool_length_mm,
    )
    if (not all(map(isfinite, values)) or not 10.0 <= tilt_deg <= 55.0
            or flip_lift_delta_mm <= 0 or release_clearance_mm < 0 or tool_length_mm < 0):
        raise ReorientPlanningError("INVALID_POSE", "재정렬 각도 또는 상승 높이 설정이 유효하지 않습니다.")
    if not guard.check_j6_limit(current_j6_deg):
        raise ReorientPlanningError("J6_LIMIT_VIOLATION", "J6 각도가 허용 범위를 벗어났습니다.")

    x, y, target_z, rz = check.clamped_pose
    z_grasp = max(guard.z_min_mm, target_z)
    transit_z = guard.safe_transit_z(z_grasp)
    lift_z = max(z_grasp + flip_lift_delta_mm, guard.safe_transit_z_mm)
    if max(lift_z, transit_z) > guard.bounds["z_max"] or lift_z <= z_grasp:
        raise ReorientPlanningError("OUT_OF_REACH", "재정렬에 필요한 상승 여유를 확보할 수 없습니다.")

    delta_j5 = -tilt_deg if current_j5_deg >= 0.0 else tilt_deg
    tilted_j5 = current_j5_deg + delta_j5
    if max(abs(current_j5_deg), abs(tilted_j5)) > max_j5_abs_deg:
        raise ReorientPlanningError("OUT_OF_REACH", "J5 안전 각도 범위를 초과합니다.")

    # J5가 0° 방향으로 펴지며 생기는 TCP 원호 궤적(방사 외측 + 상승)을 작업 범위 내에서 검증합니다.
    radial = hypot(x, y)
    ux, uy = (x / radial, y / radial) if radial > 1e-6 else (1.0, 0.0)
    swept_x, swept_y, swept_z = x, y, lift_z
    for fraction in (0.5, 1.0):
        angle = radians(tilt_deg * fraction)
        swept_x = x + ux * tool_length_mm * sin(angle)
        swept_y = y + uy * tool_length_mm * sin(angle)
        swept_z = lift_z + tool_length_mm * (1.0 - cos(angle))
        arc_check = guard.validate_pose([swept_x, swept_y, swept_z, rz])
        if not arc_check.ok:
            raise ReorientPlanningError(
                arc_check.error_code, "J5 회전 중 TCP 궤적이 작업 범위를 벗어납니다.",
            )

    # 기울인 자세에서 공중 낙하를 막기 위해 지지면 여유 높이까지 내려온 뒤 그리퍼를 엽니다.
    place_z = min(swept_z, max(guard.z_min_mm + 45.0, z_grasp + release_clearance_mm))

    waypoints = [
        MotionWaypoint("approach", [x, y, transit_z, rz], current_j6_deg),
        MotionWaypoint("descend", [x, y, z_grasp, rz], current_j6_deg),
        MotionWaypoint("grasp", [x, y, z_grasp, rz], current_j6_deg, action="grasp", check_grip=True),
        MotionWaypoint("lift", [x, y, lift_z, rz], current_j6_deg),
        MotionWaypoint("gentle_tilt_j5", [swept_x, swept_y, swept_z, rz], current_j6_deg,
                       action="move_joint_j5", j5_delta_deg=delta_j5),
        MotionWaypoint("place_flip", [swept_x, swept_y, place_z, rz], current_j6_deg),
        MotionWaypoint("release_flip", [swept_x, swept_y, place_z, rz], current_j6_deg, action="release"),
        MotionWaypoint("lift_after_flip", [swept_x, swept_y, swept_z, rz], current_j6_deg),
        MotionWaypoint("restore_j5", [x, y, lift_z, rz], current_j6_deg,
                       action="move_joint_j5", j5_delta_deg=-delta_j5),
        MotionWaypoint("retreat", [x, y, transit_z, rz], current_j6_deg),
    ]
    for waypoint in waypoints:
        check = guard.validate_pose(waypoint.pose)
        if not check.ok:
            raise ReorientPlanningError(check.error_code, check.message)
        if not guard.check_j6_limit(waypoint.j6_deg):
            raise ReorientPlanningError("J6_LIMIT_VIOLATION", "예상 J6 각도가 허용 범위를 벗어났습니다.")
    return waypoints


def plan_edge_pivot_flip(target_pose, current_j5_deg: float, current_j6_deg: float,
                         safety_guard: SafetyGuard, edge_offset_mm: float = 35.0,
                         pivot_lift_mm: float = 70.0, pivot_tilt_deg: float = 50.0,
                         drag_forward_mm: float = 55.0, release_z_offset_mm: float = 25.0,
                         min_j5_abs_deg: float = 20.0, max_j5_abs_deg: float = 125.0,
                         tool_length_mm: float = 361.0):
    guard = safety_guard
    check = guard.validate_pose(target_pose)
    if not check.ok:
        raise ReorientPlanningError(check.error_code, check.message)
    values = (
        current_j5_deg, current_j6_deg, edge_offset_mm, pivot_lift_mm, pivot_tilt_deg,
        drag_forward_mm, release_z_offset_mm, min_j5_abs_deg, max_j5_abs_deg, tool_length_mm,
    )
    if (not all(map(isfinite, values)) or edge_offset_mm <= 0 or pivot_lift_mm <= 0
            or not 20.0 <= pivot_tilt_deg <= 75.0 or drag_forward_mm <= 0
            or release_z_offset_mm < 0 or min_j5_abs_deg < 15.0 or tool_length_mm < 0):
        raise ReorientPlanningError("INVALID_POSE", "가장자리 반전 설정이 유효하지 않습니다.")
    if not guard.check_j6_limit(current_j6_deg):
        raise ReorientPlanningError("J6_LIMIT_VIOLATION", "J6 각도가 허용 범위를 벗어났습니다.")

    x, y, target_z, rz = check.clamped_pose
    radial = hypot(x, y)
    ux, uy = (x / radial, y / radial) if radial > 1e-6 else (1.0, 0.0)
    pick_x, pick_y = x - ux * edge_offset_mm, y - uy * edge_offset_mm
    lift_x, lift_y = x - ux * edge_offset_mm / 2, y - uy * edge_offset_mm / 2
    z_grasp = max(guard.z_min_mm, target_z)
    transit_z = guard.safe_transit_z(z_grasp)
    lift_z = z_grasp + pivot_lift_mm
    if not z_grasp < lift_z <= guard.bounds["z_max"]:
        raise ReorientPlanningError("OUT_OF_REACH", "피벗 상승 높이가 작업 범위를 벗어납니다.")

    delta_j5 = -pivot_tilt_deg if current_j5_deg >= 0.0 else pivot_tilt_deg
    tilted_j5 = current_j5_deg + delta_j5
    if (abs(current_j5_deg) > max_j5_abs_deg
            or not min_j5_abs_deg <= abs(tilted_j5) <= max_j5_abs_deg
            or current_j5_deg * tilted_j5 <= 0):
        raise ReorientPlanningError("OUT_OF_REACH", "J5 특이점 또는 안전 각도 범위를 벗어납니다.")

    for fraction in (0.5, 1.0):
        angle = radians(pivot_tilt_deg * fraction)
        swept_x = lift_x + ux * tool_length_mm * sin(angle)
        swept_y = lift_y + uy * tool_length_mm * sin(angle)
        swept_z = lift_z + tool_length_mm * (1.0 - cos(angle))
        arc_check = guard.validate_pose([swept_x, swept_y, swept_z, rz])
        if not arc_check.ok:
            raise ReorientPlanningError(
                arc_check.error_code, "J5 회전 중 TCP 궤적이 작업 범위를 벗어납니다.",
            )

    drag_x = swept_x + ux * drag_forward_mm
    drag_y = swept_y + uy * drag_forward_mm
    release_z = min(swept_z, max(guard.z_min_mm + 40.0, z_grasp + release_z_offset_mm))

    def waypoint(stage, px, py, pz, action="move_linear", j5_delta=0.0):
        pose = [px, py, pz, rz]
        check = guard.validate_pose(pose)
        if not check.ok:
            raise ReorientPlanningError(check.error_code, check.message)
        if not guard.check_j6_limit(current_j6_deg):
            raise ReorientPlanningError("J6_LIMIT_VIOLATION", "예상 J6 각도가 허용 범위를 벗어났습니다.")
        return MotionWaypoint(stage, pose, current_j6_deg, action=action,
                              check_grip=action == "grasp", j5_delta_deg=j5_delta)

    return [
        waypoint("approach", pick_x, pick_y, transit_z),
        waypoint("descend", pick_x, pick_y, z_grasp),
        waypoint("grasp", pick_x, pick_y, z_grasp, "grasp"),
        waypoint("pivot_lift", lift_x, lift_y, lift_z),
        waypoint("pivot_tilt_j5", swept_x, swept_y, swept_z, "move_joint_j5", delta_j5),
        waypoint("drag_over", drag_x, drag_y, release_z),
        waypoint("release_flip", drag_x, drag_y, release_z, "release"),
        waypoint("lift_after_flip", drag_x, drag_y, swept_z),
        waypoint("restore_j5", lift_x, lift_y, lift_z, "move_joint_j5", -delta_j5),
        waypoint("retreat", x, y, transit_z),
    ]
