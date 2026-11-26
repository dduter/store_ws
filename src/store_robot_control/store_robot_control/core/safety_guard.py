from collections.abc import Mapping
from dataclasses import dataclass
from math import hypot, isfinite
from pathlib import Path
from threading import Event
from typing import Optional

import yaml


def load_settings(source):
    if isinstance(source, Mapping):
        return source
    with Path(source).open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


@dataclass
class SafetyCheckResult:
    ok: bool
    error_code: str = ""
    message: str = ""
    clamped_pose: Optional[list[float]] = None


class SafetyGuard:
    def __init__(self, source):
        config = load_settings(source)
        self.settings = config["safety"]
        self.poses = config["poses"]
        self.bounds = self.settings["workspace_bounds_mm"]
        self.max_radial_reach_mm = self.settings.get(
            "max_radial_reach_mm", self.bounds.get("max_radial_reach_mm"),
        )
        self.z_min_mm = self.settings["z_min_mm"]
        self.safe_transit_z_mm = self.settings["safe_transit_z_mm"]
        self.j6_min_deg = max(-150.0, self.settings["j6_min_deg"])
        self.j6_max_deg = min(150.0, self.settings["j6_max_deg"])
        self._emergency_stop = Event()

    def validate_pose(self, pose):
        try:
            x, y, z, rz = map(float, pose)
        except (TypeError, ValueError, OverflowError):
            return SafetyCheckResult(False, "INVALID_POSE", "유한한 좌표 4개가 필요합니다.")
        if not all(map(isfinite, (x, y, z, rz))):
            return SafetyCheckResult(False, "INVALID_POSE", "좌표는 유한해야 합니다.")
        if z < self.z_min_mm:
            return SafetyCheckResult(False, "Z_MIN_VIOLATION", "최소 높이보다 낮습니다.")
        bounds = self.bounds
        if not (
            bounds["x_min"] <= x <= bounds["x_max"]
            and bounds["y_min"] <= y <= bounds["y_max"]
            and z <= bounds["z_max"]
            and hypot(x, y) <= self.max_radial_reach_mm
        ):
            return SafetyCheckResult(False, "OUT_OF_REACH", "작업 범위를 벗어났습니다.")
        return SafetyCheckResult(True, clamped_pose=[x, y, z, rz])

    def check_j6_limit(self, j6_deg):
        return self.j6_min_deg <= j6_deg <= self.j6_max_deg

    def clamp_j6_angle(self, j6_deg):
        if not isfinite(j6_deg):
            raise ValueError("J6 각도는 유한해야 합니다.")
        return max(self.j6_min_deg, min(self.j6_max_deg, j6_deg))

    def safe_transit_z(self, pose_z):
        return min(
            self.bounds["z_max"],
            max(self.safe_transit_z_mm, pose_z + self.settings["approach_offset_z_mm"]),
        )

    def set_emergency_stop(self, active):
        if active:
            self._emergency_stop.set()
        else:
            self._emergency_stop.clear()

    def is_emergency_stopped(self):
        return self._emergency_stop.is_set()
