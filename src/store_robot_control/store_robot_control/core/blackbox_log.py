import json
from dataclasses import asdict, is_dataclass
from datetime import datetime, timezone
from pathlib import Path
from threading import Lock


def _json_safe(value):
    if isinstance(value, float):
        return value if float("-inf") < value < float("inf") else str(value)
    if is_dataclass(value):
        return _json_safe(asdict(value))
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


class BlackboxLogger:
    def __init__(self, log_dir):
        self.path = Path(log_dir) / "robot_blackbox.jsonl"
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = Lock()

    def log(self, *, task_id, action_type, product_category, goal, grasp_plan,
            success, stopped_for_safety, error_code, final_pose, duration_ms):
        now = datetime.now(timezone.utc)
        entry = {
            "timestamp_utc": now.isoformat(),
            "timestamp_local": now.astimezone().isoformat(),
            "task_id": task_id,
            "action_type": action_type,
            "product_category": product_category,
            "goal": goal,
            "grasp_plan": grasp_plan,
            "success": success,
            "stopped_for_safety": stopped_for_safety,
            "error_code": error_code,
            "final_pose": final_pose,
            "duration_ms": duration_ms,
        }
        line = json.dumps(_json_safe(entry), ensure_ascii=False, allow_nan=False)
        with self._lock, self.path.open("a", encoding="utf-8") as stream:
            stream.write(line + "\n")
