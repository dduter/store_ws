import json
from datetime import datetime, timedelta

from store_robot_control.core.blackbox_log import BlackboxLogger
from store_robot_control.core.grasp_planner import GraspPlan


def test_jsonl_append(tmp_path):
    logger = BlackboxLogger(tmp_path / "data" / "logs")
    plan = GraspPlan("defaults", 55, 75, 20, 15, "flip_180_in_place", 550, 750, 200)
    entry = dict(task_id="goal-1", action_type="pick_place", product_category="",
                 goal={"target_pose": [300, 0, 45, 0]}, grasp_plan=plan,
                 success=True, stopped_for_safety=False, error_code="",
                 final_pose=[400, 0, 45, 0], duration_ms=10)
    logger.log(**entry)
    logger.log(**(entry | {"task_id": "goal-2", "success": False, "stopped_for_safety": True,
                           "error_code": "CANCELED", "goal": {"target_pose": [float("nan")]}}))
    completed, stopped = map(json.loads, logger.path.read_text().splitlines())
    assert completed["task_id"] == "goal-1"
    assert completed["grasp_plan"]["rg2_width_reg"] == 550
    assert completed["final_pose"] == [400, 0, 45, 0]
    assert datetime.fromisoformat(completed["timestamp_utc"]).utcoffset() == timedelta(0)
    assert datetime.fromisoformat(completed["timestamp_local"]).utcoffset() is not None
    assert stopped["goal"]["target_pose"] == ["nan"]
    assert stopped["stopped_for_safety"] and stopped["error_code"] == "CANCELED"
