import pytest

from conftest import REPO_CONFIG
from store_robot_control.core.teleop_math import (
    DeadzoneFilter, EmaLowPassFilter, GripperGestureDebouncer, TeleopPipeline, TeleopWatchdog,
)


def test_radial_deadzone():
    deadzone = DeadzoneFilter(5)
    assert deadzone.apply(1, -2) == (0, 0)
    assert deadzone.apply(3, -4) == (0, 0)
    assert deadzone.apply(6, -8) == pytest.approx((3, -4))


def test_ema_smoothing_convergence_and_reset():
    ema = EmaLowPassFilter(0.5)
    assert ema.update((0, 0, 0)) == [0, 0, 0]
    assert ema.update([2, 4, 8]) == [1, 2, 4]
    for _ in range(30):
        value = ema.update([2, 4, 8])
    assert value == pytest.approx([2, 4, 8])
    ema.reset()
    assert ema.update([1]) == [1]
    assert EmaLowPassFilter(1).update([4]) == [4]


@pytest.mark.parametrize("alpha", [-0.1, 0, 1.01, float("nan")])
def test_ema_alpha_validation(alpha):
    with pytest.raises(ValueError):
        EmaLowPassFilter(alpha)


def test_gripper_debounce():
    gripper = GripperGestureDebouncer()
    assert not gripper.update(True, 0)
    assert not gripper.update(True, 0.49)
    assert gripper.update(True, 0.50)
    assert gripper.update(False, 0.60)
    assert gripper.update(True, 0.70)
    assert gripper.update(False, 1.0)
    assert gripper.update(False, 1.49)
    assert not gripper.update(False, 1.50)


def test_watchdog():
    watchdog = TeleopWatchdog()
    assert watchdog.is_expired(1)
    watchdog.feed(1)
    assert not watchdog.is_expired(1.49)
    assert not watchdog.is_expired(1.50)
    assert watchdog.is_expired(1.51)
    watchdog.reset()
    assert watchdog.is_expired(1)


def test_pipeline_from_control_config():
    pipeline = TeleopPipeline(REPO_CONFIG / "control_params.yaml")
    assert pipeline.update_motion(15, 0) == [0, 0]
    assert pipeline.update_motion(25, 0) == pytest.approx([3, 0])
    assert pipeline.gripper.debounce_sec == 0.5
    assert pipeline.watchdog.timeout_sec == 0.5
    assert TeleopPipeline({"teleop": {"ema_alpha": 1}}).ema.alpha == 1
