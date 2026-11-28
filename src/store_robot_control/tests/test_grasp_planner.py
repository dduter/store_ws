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
    assert plan.vertical_edge_offset_mm is None
    assert plan.reorient_mode == "edge_pivot_flip"
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
    assert planner.plan_grasp(category).vertical_edge_offset_mm is None
    assert planner.plan_grasp(category).reorient_mode == "edge_pivot_flip"


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
            "choco_pie": {"category": "paper_box", "grasp_force_n": 14.0, "vertical_edge_offset_mm": "42.5"},
        },
    })
    plan = planner.plan_grasp("choco_pie")
    assert plan.category == "choco_pie"
    assert plan.vertical_edge_offset_mm == 42.5
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


def test_measured_product_overrides(planner):
    pepero = planner.plan_grasp("pepero_almond")
    assert pepero.category == "pepero_almond"
    assert (pepero.open_width_mm, pepero.grasp_width_mm, pepero.grasp_force_n) == (108.0, 84.0, 3.0)
    assert pepero.reorient_mode == "back_tilt_vertical_flip"
    assert pepero.vertical_edge_offset_mm == 65.0
    assert (
        pepero.pick_offset_x_mm,
        pepero.pick_offset_y_mm,
        pepero.pick_yaw_offset_deg,
        pepero.pick_transit_z_mm,
        pepero.place_z_offset_mm,
    ) == (0.0, 0.0, 0.0, None, 0.0)

    postick = planner.plan_grasp("postick")
    assert postick.category == "postick"
    assert (postick.open_width_mm, postick.grasp_width_mm, postick.grasp_force_n) == (70.0, 0.0, 18.0)
    assert postick.reorient_mode == "back_tilt_vertical_flip"
    assert postick.vertical_entry_axis == "x"
    assert postick.vertical_entry_yaw_deg == -45.0
    assert postick.vertical_edge_offset_mm == 114.0
    assert postick.vertical_corner_x_offset_mm == 75.0
    assert postick.vertical_back_tilt_deg == 50.0
    assert postick.vertical_fwd_tilt_deg == 60.0
    assert postick.vertical_sweep_mm == 180.0
    assert postick.vertical_tilt_sign == -1
    assert postick.vertical_fwd_tilt_sign == -1
    assert postick.vertical_lift_delta_mm == 110.0
    assert postick.vertical_mid_z_delta_mm == 275.0
    assert postick.vertical_release_z_delta_mm == 125.0
    assert (
        postick.pick_offset_x_mm,
        postick.pick_offset_y_mm,
        postick.pick_yaw_offset_deg,
        postick.pick_transit_z_mm,
        postick.place_z_offset_mm,
    ) == (75.0, -114.0, -45.0, 260.0, 95.0)

    # 확정 YOLO 클래스명 별칭 호환 확인
    assert planner.plan_grasp("pepero_almond_front") == pepero
    assert planner.plan_grasp("postick_front") == postick
    drink = planner.plan_grasp("drink_front")
    assert drink.category == "drink"
    assert drink.reorient_mode == "in_place_yaw_180"
    assert planner.plan_grasp("ciga") == planner.plan_grasp("cigarette_mock")


@pytest.mark.parametrize("offset", [0, -1, float("nan"), float("inf"), "invalid"])
def test_invalid_vertical_edge_offset(planner, monkeypatch, offset):
    monkeypatch.setitem(planner.products["pepero_almond"], "vertical_edge_offset_mm", offset)
    with pytest.raises(ValueError, match="vertical_edge_offset_mm"):
        planner.plan_grasp("pepero_almond")


@pytest.mark.parametrize(("field", "value"), [
    ("vertical_entry_axis", "z"),
    ("vertical_entry_yaw_deg", -91.0),
    ("vertical_entry_yaw_deg", 91.0),
    ("vertical_tilt_sign", 0),
    ("vertical_fwd_tilt_sign", 2),
    ("vertical_slide_in_y_mm", -1.0),
    ("vertical_back_tilt_deg", -1.0),
])
def test_invalid_vertical_flip_setting(planner, monkeypatch, field, value):
    monkeypatch.setitem(planner.products["postick"], field, value)
    with pytest.raises(ValueError, match=field):
        planner.plan_grasp("postick_front")


@pytest.mark.parametrize(("field", "value"), [
    ("pick_yaw_offset_deg", 125.0),
    ("place_z_offset_mm", -1.0),
    ("pick_transit_z_mm", 0.0),
    ("pick_transit_z_mm", -1.0),
])
def test_invalid_transfer_offset(planner, monkeypatch, field, value):
    monkeypatch.setitem(planner.products["postick"], field, value)
    with pytest.raises(ValueError, match=field):
        planner.plan_grasp("postick_front")


def test_zero_vertical_back_tilt_is_allowed(planner, monkeypatch):
    monkeypatch.setitem(planner.products["postick"], "vertical_back_tilt_deg", 0.0)
    assert planner.plan_grasp("postick_front").vertical_back_tilt_deg == 0.0


def test_product_back_safe_fallback(planner):
    plan = planner.plan_grasp("product_back")
    assert (plan.open_width_mm, plan.grasp_width_mm, plan.grasp_force_n) == (108.0, 84.0, 3.0)
    assert plan.reorient_mode == "back_tilt_vertical_flip"
    assert plan.vertical_edge_offset_mm == 65.0
