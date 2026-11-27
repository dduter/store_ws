from store_robot_control.core.grasp_planner import clamp_grasp_params
from store_robot_control.core.reorient_motion import can_retry_reorient
from store_robot_control.core.safety_guard import clamp_z_min, is_j6_within_limit
from store_robot_control.core.teleop_math import apply_deadzone, debounce_hold_ready


def test_safety_guard():
    assert clamp_z_min(5.0, 15.0) == 15.0
    assert is_j6_within_limit(149.0) is True
    assert is_j6_within_limit(151.0) is False


def test_grasp_and_reorient():
    assert clamp_grasp_params(120.0, 2.0) == (110.0, 5.0)
    assert can_retry_reorient(0, 1) is True
    assert can_retry_reorient(1, 1) is False


def test_teleop_math():
    assert apply_deadzone(10.0, 15.0) == 0.0
    assert apply_deadzone(20.0, 15.0) == 20.0
    assert debounce_hold_ready(0.49, 0.5) is False
    assert debounce_hold_ready(0.50, 0.5) is True
