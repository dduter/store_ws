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


class GraspPlanner:
    def __init__(self, source):
        config = load_settings(source)
        self.defaults = config["defaults"]
        self.categories = config["categories"]
        self.products = config.get("products") or {}
        self.limits = config["rg2_limits"]

    def plan_grasp(self, product_category: str, width_override_mm: float = 0.0,
                   force_override_n: float = 0.0) -> GraspPlan:
        if product_category in self.products:
            product_cfg = self.products[product_category]
            base_cat = product_cfg.get("category", "")
            category = product_category
            settings = self.defaults | self.categories.get(base_cat, {}) | product_cfg
        elif product_category in self.categories:
            category = product_category
            settings = self.defaults | self.categories[product_category]
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
        return GraspPlan(
            category, width, open_width, force, settings["grasp_depth_offset_mm"],
            settings["reorient_mode"], int(round(width * 10)), int(round(open_width * 10)),
            int(round(force * 10)),
        )
