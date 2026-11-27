import argparse
from math import isfinite
from pathlib import Path
from shutil import copy2
from stat import S_IMODE
from tempfile import NamedTemporaryFile

import rclpy
import yaml
from rclpy.utilities import remove_ros_args

from ..core.safety_guard import load_settings
from ..hardware.dsr_interface import ensure_tool_and_tcp, load_dsr_api, top_down_effective_rz
from .mock_robot import find_config_dir


def _ensure_tool_tcp(api, robot):
    active_tool, active_tcp = ensure_tool_and_tcp(api, robot["tool_name"], robot["tcp_name"])
    print(f"Active Tool={active_tool}, TCP={active_tcp}")


def _read_coordinates(api):
    pose, _ = api.get_current_posx(ref=api.DR_BASE)
    joints = api.get_current_posj()
    if pose is None or isinstance(joints, (int, float)):
        raise RuntimeError("Doosan 제어기에서 좌표를 읽지 못했습니다.")
    pose, joints = list(map(float, pose)), list(map(float, joints))
    if len(pose) != 6 or len(joints) != 6 or not all(map(isfinite, pose + joints)):
        raise ValueError("Cartesian 및 Joint 자세는 각각 유한한 값 6개여야 합니다.")
    print(f"Cartesian (mm, deg) {pose}")
    effective_rz = top_down_effective_rz(pose)
    if effective_rz is None:
        print(f"4D [x, y, z, rz]    (비하향 자세 ry={pose[4]:.2f}deg — 4D 복사 불가)")
    else:
        print(
            f"4D [x, y, z, rz]    "
            f"{[round(pose[0], 2), round(pose[1], 2), round(pose[2], 2), round(effective_rz, 2)]}"
        )
    print(f"Joint (deg) {joints}")
    return pose, joints


def _save_workspace(path, workspace):
    with NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent, delete=False) as stream:
        temporary = Path(stream.name)
    try:
        with temporary.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(workspace, stream, sort_keys=False, allow_unicode=True)
        temporary.chmod(S_IMODE(path.stat().st_mode))
        copy2(path, path.with_suffix(".yaml.bak"))
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)
    print(f"저장 완료 ({path})")


def _update_workspace(choice, workspace, pose, joints):
    poses = workspace["poses"]
    if choice in {"1", "2"}:
        name = "home" if choice == "1" else "scan_pose"
        poses[f"{name}_joint_deg"] = joints
        poses[f"{name}_cartesian"] = pose
    elif choice == "3":
        margin = float(input("안전 여유 높이 (mm, 기본 10)> ").strip() or "10")
        limit = pose[2] + margin
        safety = workspace["safety"]
        if not isfinite(margin) or margin < 0 or limit > safety["safe_transit_z_mm"]:
            raise ValueError("안전 여유는 유한한 0 이상의 값이며, 결과는 안전 이동 높이 이하여야 합니다.")
        if input(f"z_min_mm을 {limit:.3f} mm로 저장하시겠습니까? [y/N]> ").strip().lower() != "y":
            return False
        safety["z_min_mm"] = limit
    elif choice in {"4", "5"}:
        effective_rz = top_down_effective_rz(pose)
        if effective_rz is None:
            raise ValueError(f"4D 좌표 저장은 하향 자세(ry≈180°)에서만 가능합니다 (현재 ry={pose[4]:.2f}°).")
        pose_4d = [*pose[:3], effective_rz]
        if choice == "4":
            poses["employee_handover_pose"] = pose_4d
        else:
            slots = workspace["zones"].setdefault("buffer_slots", [])
            entry = input(f"교체할 슬롯 번호 (1~{len(slots)}, Enter는 추가)> ").strip()
            if not entry:
                slots.append(pose_4d)
            else:
                index = int(entry) - 1
                if not 0 <= index < len(slots):
                    raise ValueError("기존 버퍼 슬롯 번호를 입력하세요.")
                slots[index] = pose_4d
    return True


def _teach(api, path):
    while True:
        print("\n1. home  2. scan_pose  3. z_min_mm  4. 직원 전달 자세  5. 버퍼 슬롯  q. 종료")
        choice = input("저장할 항목> ").strip().lower()
        if choice == "q":
            return
        if choice not in {"1", "2", "3", "4", "5"}:
            print("메뉴에 있는 항목을 선택하세요.")
            continue
        try:
            if api.get_robot_mode() != 0:
                print("직접 교시 모드 (robot_mode=0)에서 저장하세요.")
                continue
            workspace = load_settings(path)
            pose, joints = _read_coordinates(api)
            if _update_workspace(choice, workspace, pose, joints):
                _save_workspace(path, workspace)
        except ValueError as exc:
            print(f"입력을 확인하세요 ({exc})")


def main(args=None):
    parser = argparse.ArgumentParser(description="직접 교시 모드에서 Doosan 작업 좌표를 기록합니다.")
    parser.add_argument("--config-dir", type=Path, default=find_config_dir())
    parser.add_argument("--print-only", action="store_true", help="현재 자세를 한 번 출력합니다.")
    # ROS 인수 분리는 실행 파일 이름을 포함한 argv를 기준으로 동작합니다.
    options = parser.parse_args(remove_ros_args(args=None if args is None else ["teach_zones", *args])[1:])
    path = options.config_dir / "workspace_zones.yaml"
    workspace = load_settings(path)
    robot = workspace["robot"]
    rclpy.init(args=args)
    node = None
    try:
        node = rclpy.create_node(
            "store_dsr_client", namespace=robot["robot_id"], use_global_arguments=False,
        )
        api = load_dsr_api(robot["robot_id"], robot["robot_model"], node)
        _ensure_tool_tcp(api, robot)
        if options.print_only:
            _read_coordinates(api)
        else:
            print("로봇을 직접 교시 모드 (robot_mode=0)로 두고 원하는 자세로 이동하세요.")
            _teach(api, path)
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
