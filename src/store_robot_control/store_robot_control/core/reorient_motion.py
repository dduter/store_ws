from dataclasses import dataclass
from math import isfinite

from .safety_guard import SafetyGuard


@dataclass
class MotionWaypoint:
    stage: str
    pose: list[float]
    j6_deg: float


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
        MotionWaypoint("grasp", [x, y, z, rz], current_j6_deg),
        MotionWaypoint("lift", [x, y, lift_z, rz], current_j6_deg),
        MotionWaypoint("flip_180", [x, y, lift_z, rz + delta], final_j6),
        MotionWaypoint("place", [x, y, z, rz + delta], final_j6),
        MotionWaypoint("release", [x, y, z, rz + delta], final_j6),
        MotionWaypoint("retreat", [x, y, guard.safe_transit_z(z), rz + delta], final_j6),
    ]
    for waypoint in waypoints:
        check = guard.validate_pose(waypoint.pose)
        if not check.ok:
            raise ReorientPlanningError(check.error_code, check.message)
    return waypoints
