from copy import deepcopy
from pathlib import Path

import yaml

REPO_CONFIG = Path(__file__).resolve().parents[3] / "config"

TEST_WORKSPACE = {
    "robot": {
        "robot_id": "dsr01",
        "robot_model": "m0609",
        "tool_name": "Tool_Weight",
        "tcp_name": "GripperDA_v1",
    },
    "safety": {
        "z_min_mm": 15.0,
        "safe_transit_z_mm": 260.0,
        "approach_offset_z_mm": 80.0,
        "j6_min_deg": -150.0,
        "j6_max_deg": 150.0,
        "workspace_bounds_mm": {
            "x_min": 180.0,
            "x_max": 720.0,
            "y_min": -420.0,
            "y_max": 420.0,
            "z_max": 400.0,
        },
        "max_radial_reach_mm": 760.0,
    },
    "poses": {
        "home_joint_deg": [0.0, 0.0, 90.0, 0.0, 90.0, 0.0],
        "scan_pose_joint_deg": [0.0, 10.0, 85.0, 0.0, 85.0, 0.0],
        "home_cartesian": [380.0, 0.0, 380.0, 0.0, 180.0, 0.0],
        "scan_pose_cartesian": [400.0, 0.0, 420.0, 0.0, 180.0, 0.0],
        "employee_handover_pose": [480.0, -220.0, 220.0, 0.0],
        "reorient_lift_z_mm": 220.0,
    },
    "zones": {
        "display_area": {"x_range": [250.0, 580.0], "y_range": [-200.0, 200.0]},
        "inbound_box": {"x_range": [250.0, 450.0], "y_range": [200.0, 380.0]},
        "buffer_slots": [[300.0, -280.0, 40.0, 0.0]],
    },
}


def make_test_config_dir(tmp_path: Path) -> Path:
    cfg = tmp_path / "config"
    cfg.mkdir(exist_ok=True)
    for name in ("products_catalog.yaml", "control_params.yaml"):
        (cfg / name).write_bytes((REPO_CONFIG / name).read_bytes())
    with (cfg / "workspace_zones.yaml").open("w", encoding="utf-8") as stream:
        yaml.safe_dump(deepcopy(TEST_WORKSPACE), stream, sort_keys=False)
    return cfg
