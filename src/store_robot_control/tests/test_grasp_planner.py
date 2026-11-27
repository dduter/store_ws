from pathlib import Path

import pytest

from store_robot_control.core.grasp_planner import GraspPlanner


@pytest.fixture
def planner():
    config = Path(__file__).resolve().parents[3] / "config"
    return GraspPlanner(config / "products_catalog.yaml")


@pytest.mark.parametrize(("category", "expected"), [
    ("paper_box", (20, 95, 20, 18)),
    ("snack_bag", (15, 95, 12, 15)),
    ("candy_chocolate", (10, 75, 15, 12)),
])
def test_catalog_grasp(planner, category, expected):
    plan = planner.plan_grasp(category)
    assert plan.category == category
    assert plan.reorient_mode == "gentle_flip"
    assert (
        plan.grasp_width_mm,
        plan.open_width_mm,
        plan.grasp_force_n,
        plan.grasp_depth_offset_mm,
    ) == expected
    assert (plan.rg2_width_reg, plan.rg2_open_width_reg, plan.rg2_force_reg) == (
        expected[0] * 10,
        expected[1] * 10,
        expected[2] * 10,
    )


@pytest.mark.parametrize("category", ["", "unknown"])
def test_default_grasp(planner, category):
    assert planner.plan_grasp(category) == planner.plan_grasp("defaults")
    assert planner.plan_grasp(category).grasp_width_mm == 20
    assert planner.plan_grasp(category).open_width_mm == 90
    assert planner.plan_grasp(category).reorient_mode == "gentle_flip"


def test_overrides(planner):
    plan = planner.plan_grasp("snack_bag", 52.5, 12.3)
    assert (plan.rg2_width_reg, plan.rg2_open_width_reg, plan.rg2_force_reg) == (525, 725, 123)


def test_per_product_override_falls_back_to_category():
    planner = GraspPlanner({
        "defaults": {
            "grasp_width_mm": 20.0,
            "open_width_mm": 90.0,
            "open_margin_mm": 20.0,
            "grasp_force_n": 18.0,
            "grasp_depth_offset_mm": 15.0,
            "reorient_mode": "flip_180_in_place",
        },
        "rg2_limits": {"min_width_mm": 0.0, "max_width_mm": 110.0, "min_force_n": 3.0, "max_force_n": 40.0},
        "categories": {
            "paper_box": {"grasp_width_mm": 20.0, "open_width_mm": 95.0, "grasp_force_n": 20.0, "grasp_depth_offset_mm": 18.0},
        },
        "products": {
            "choco_pie": {"category": "paper_box", "grasp_force_n": 14.0},
        },
    })
    plan = planner.plan_grasp("choco_pie")
    assert plan.category == "choco_pie"
    assert (plan.grasp_width_mm, plan.open_width_mm, plan.grasp_force_n, plan.grasp_depth_offset_mm) == (20.0, 95.0, 14.0, 18.0)


@pytest.mark.parametrize(("override", "expected"), [
    ((200, 80), (110, 110, 40)),
    ((-1, -1), (0, 20, 3)),
])
def test_rg2_clamping(planner, override, expected):
    plan = planner.plan_grasp("paper_box", *override)
    assert (plan.grasp_width_mm, plan.open_width_mm, plan.grasp_force_n) == expected


@pytest.mark.parametrize("category", ["cigarette_mock"])
def test_special_categories_disable_reorient(planner, category):
    assert planner.plan_grasp(category).reorient_mode == "none"
