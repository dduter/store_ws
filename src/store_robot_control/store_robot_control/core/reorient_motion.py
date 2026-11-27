from dataclasses import dataclass
from math import isfinite
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
