from collections.abc import Mapping
from dataclasses import dataclass
from math import hypot, isfinite
from numbers import Real
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
        zones = self.settings.get("z_min_zones", [])
        if not isinstance(zones, (list, tuple)):
            raise ValueError("z_min_zones는 구역 목록이어야 합니다.")
        self._z_min_zones = tuple(self._parse_z_min_zone(zone) for zone in zones)
        self.safe_transit_z_mm = self.settings["safe_transit_z_mm"]
        self.j6_min_deg = max(-150.0, self.settings["j6_min_deg"])
        self.j6_max_deg = min(150.0, self.settings["j6_max_deg"])
        self._emergency_stop = Event()

    def _parse_z_min_zone(self, zone):
        def finite_number(value):
            if not isinstance(value, Real) or isinstance(value, bool):
                return False
            try:
                return isfinite(value)
            except (TypeError, OverflowError, ValueError):
                return False

        if not isinstance(zone, Mapping):
            raise ValueError("z_min_zones의 각 구역은 매핑이어야 합니다.")
        ranges = []
        for axis in ("x_range", "y_range"):
            limits = zone.get(axis)
            if (not isinstance(limits, (list, tuple)) or len(limits) != 2
                    or not all(map(finite_number, limits)) or limits[0] > limits[1]):
                raise ValueError(f"z_min_zones의 {axis}에는 오름차순의 유한한 값 두 개가 필요합니다.")
            ranges.append(tuple(limits))
        z_min = zone.get("z_min_mm")
        if not finite_number(z_min) or z_min > self.bounds["z_max"]:
            raise ValueError("구역의 z_min_mm은 z_max 이하의 유한한 숫자여야 합니다.")
        return (*ranges, float(z_min))

    def z_min_at(self, x: float, y: float) -> float:
        return max(
            (z_min for (x_lo, x_hi), (y_lo, y_hi), z_min in self._z_min_zones
             if x_lo <= x <= x_hi and y_lo <= y <= y_hi),
            default=self.z_min_mm,
        )

    def validate_pose(self, pose):
        try:
            x, y, z, rz = map(float, pose)
        except (TypeError, ValueError, OverflowError):
            return SafetyCheckResult(False, "INVALID_POSE", "유한한 좌표 4개가 필요합니다.")
        if not all(map(isfinite, (x, y, z, rz))):
            return SafetyCheckResult(False, "INVALID_POSE", "좌표는 유한해야 합니다.")
        if z < self.z_min_at(x, y):
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
