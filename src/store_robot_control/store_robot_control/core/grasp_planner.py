from dataclasses import dataclass
from math import isfinite

from .safety_guard import load_settings


@dataclass
class GraspPlan:
    category: str
    grasp_width_mm: float
    open_width_mm: float
    grasp_force_n: float
    grasp_depth_offset_mm: float
    reorient_mode: str
    rg2_width_reg: int
    rg2_open_width_reg: int
    rg2_force_reg: int
    vertical_edge_offset_mm: float | None = None
    vertical_corner_x_offset_mm: float = 0.0
    vertical_slide_in_y_mm: float = 0.0
    vertical_entry_axis: str = "y"
    vertical_entry_yaw_deg: float = 0.0
    vertical_back_tilt_deg: float | None = None
    vertical_fwd_tilt_deg: float | None = None
    vertical_sweep_mm: float | None = None
    vertical_tilt_sign: int = -1
    vertical_fwd_tilt_sign: int | None = None
    vertical_j6_offset_deg: float = 0.0
    vertical_lift_delta_mm: float | None = None
    vertical_mid_z_delta_mm: float | None = None
    vertical_release_z_delta_mm: float | None = None
    pick_offset_x_mm: float = 0.0
    pick_offset_y_mm: float = 0.0
    pick_yaw_offset_deg: float = 0.0
    pick_transit_z_mm: float | None = None
    place_z_offset_mm: float = 0.0


class GraspPlanner:
    def __init__(self, source):
        config = load_settings(source)
        self.defaults = config["defaults"]
        self.categories = config["categories"]
        self.products = config.get("products") or {}
        self.aliases = config.get("aliases") or {}
        self.limits = config["rg2_limits"]

    def plan_grasp(self, product_category: str, width_override_mm: float = 0.0,
                   force_override_n: float = 0.0) -> GraspPlan:
        resolved = self.aliases.get(product_category, product_category)
        if resolved in self.products:
            product_cfg = self.products[resolved]
            base_cat = product_cfg.get("category", "")
            category = resolved
            settings = self.defaults | self.categories.get(base_cat, {}) | product_cfg
        elif resolved in self.categories:
            category = resolved
            settings = self.defaults | self.categories[resolved]
        else:
            category = "defaults"
            settings = dict(self.defaults)
        width = width_override_mm or settings["grasp_width_mm"]
        force = force_override_n or settings["grasp_force_n"]
        if not isfinite(width) or not isfinite(force):
            raise ValueError("그리퍼 폭과 힘은 유한해야 합니다.")
        limits = self.limits
        width = max(limits["min_width_mm"], min(limits["max_width_mm"], width))
        margin_open = width + settings.get("open_margin_mm", 20.0)
        configured_open = settings.get("open_width_mm", margin_open)
        target_open = margin_open if width_override_mm else max(configured_open, margin_open)
        open_width = max(width, min(limits["max_width_mm"], target_open))
        force = max(limits["min_force_n"], min(limits["max_force_n"], force))
        edge_offset = settings.get("vertical_edge_offset_mm")
        if edge_offset is not None:
            error = "vertical_edge_offset_mm은 유한한 양수여야 합니다."
            try:
                edge_offset = float(edge_offset)
            except (TypeError, ValueError, OverflowError) as exc:
                raise ValueError(error) from exc
            if not isfinite(edge_offset) or edge_offset <= 0:
                raise ValueError(error)
        corner_x_offset = settings.get("vertical_corner_x_offset_mm", 0.0)
        try:
            corner_x_offset = float(corner_x_offset)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("vertical_corner_x_offset_mm은 유한한 실수여야 합니다.") from exc
        if not isfinite(corner_x_offset):
            raise ValueError("vertical_corner_x_offset_mm은 유한한 실수여야 합니다.")
        slide_in_y = settings.get("vertical_slide_in_y_mm", 0.0)
        try:
            slide_in_y = float(slide_in_y)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("vertical_slide_in_y_mm은 0 이상의 유한한 실수여야 합니다.") from exc
        if not isfinite(slide_in_y) or slide_in_y < 0:
            raise ValueError("vertical_slide_in_y_mm은 0 이상의 유한한 실수여야 합니다.")
        entry_axis = str(settings.get("vertical_entry_axis", "y")).lower()
        if entry_axis not in ("x", "y"):
            raise ValueError("vertical_entry_axis는 'x' 또는 'y'여야 합니다.")
        entry_yaw = settings.get("vertical_entry_yaw_deg", 0.0)
        try:
            entry_yaw = float(entry_yaw)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("vertical_entry_yaw_deg은 유한한 실수여야 합니다.") from exc
        if not isfinite(entry_yaw) or abs(entry_yaw) > 90.0:
            raise ValueError("vertical_entry_yaw_deg은 [-90, 90] 범위여야 합니다.")
        tilt_sign = settings.get("vertical_tilt_sign", -1)
        if tilt_sign not in (-1, 1):
            raise ValueError("vertical_tilt_sign은 -1 또는 1이어야 합니다.")
        fwd_tilt_sign = settings.get("vertical_fwd_tilt_sign")
        if fwd_tilt_sign is not None and fwd_tilt_sign not in (-1, 1):
            raise ValueError("vertical_fwd_tilt_sign은 -1 또는 1이어야 합니다.")
        j6_offset = settings.get("vertical_j6_offset_deg", 0.0)
        try:
            j6_offset = float(j6_offset)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("vertical_j6_offset_deg은 유한한 실수여야 합니다.") from exc
        if not isfinite(j6_offset) or abs(j6_offset) > 120.0:
            raise ValueError("vertical_j6_offset_deg은 [-120, 120] 범위여야 합니다.")

        def optional_pos(key):
            val = settings.get(key)
            if val is None:
                return None
            try:
                val = float(val)
            except (TypeError, ValueError, OverflowError) as exc:
                raise ValueError(f"{key}은 유한한 양수여야 합니다.") from exc
            if not isfinite(val) or val <= 0:
                raise ValueError(f"{key}은 유한한 양수여야 합니다.")
            return val

        lift_delta = optional_pos("vertical_lift_delta_mm")
        mid_z_delta = optional_pos("vertical_mid_z_delta_mm")
        release_z_delta = optional_pos("vertical_release_z_delta_mm")
        back_tilt = settings.get("vertical_back_tilt_deg")
        if back_tilt is not None:
            try:
                back_tilt = float(back_tilt)
            except (TypeError, ValueError, OverflowError) as exc:
                raise ValueError("vertical_back_tilt_deg은 0 이상의 유한한 실수여야 합니다.") from exc
            if not isfinite(back_tilt) or back_tilt < 0:
                raise ValueError("vertical_back_tilt_deg은 0 이상의 유한한 실수여야 합니다.")
        fwd_tilt = optional_pos("vertical_fwd_tilt_deg")
        sweep_mm = optional_pos("vertical_sweep_mm")
        pick_transit_z = optional_pos("pick_transit_z_mm")

        def finite_float(key, default=0.0):
            val = settings.get(key, default)
            try:
                val = float(val)
            except (TypeError, ValueError, OverflowError) as exc:
                raise ValueError(f"{key}은 유한한 실수여야 합니다.") from exc
            if not isfinite(val):
                raise ValueError(f"{key}은 유한한 실수여야 합니다.")
            return val

        pick_ox = finite_float("pick_offset_x_mm")
        pick_oy = finite_float("pick_offset_y_mm")
        pick_yaw = finite_float("pick_yaw_offset_deg")
        if abs(pick_yaw) > 120.0:
            raise ValueError("pick_yaw_offset_deg은 [-120, 120] 범위여야 합니다.")
        place_z_off = finite_float("place_z_offset_mm")
        if place_z_off < 0.0:
            raise ValueError("place_z_offset_mm은 0 이상의 유한한 실수여야 합니다.")
        return GraspPlan(
            category, width, open_width, force, settings["grasp_depth_offset_mm"],
            settings["reorient_mode"], int(round(width * 10)), int(round(open_width * 10)),
            int(round(force * 10)), vertical_edge_offset_mm=edge_offset,
            vertical_corner_x_offset_mm=corner_x_offset,
            vertical_slide_in_y_mm=slide_in_y,
            vertical_entry_axis=entry_axis,
            vertical_entry_yaw_deg=entry_yaw,
            vertical_back_tilt_deg=back_tilt,
            vertical_fwd_tilt_deg=fwd_tilt,
            vertical_sweep_mm=sweep_mm,
            vertical_tilt_sign=int(tilt_sign),
            vertical_fwd_tilt_sign=None if fwd_tilt_sign is None else int(fwd_tilt_sign),
            vertical_j6_offset_deg=j6_offset,
            vertical_lift_delta_mm=lift_delta,
            vertical_mid_z_delta_mm=mid_z_delta,
            vertical_release_z_delta_mm=release_z_delta,
            pick_offset_x_mm=pick_ox,
            pick_offset_y_mm=pick_oy,
            pick_yaw_offset_deg=pick_yaw,
            pick_transit_z_mm=pick_transit_z,
            place_z_offset_mm=place_z_off,
        )
