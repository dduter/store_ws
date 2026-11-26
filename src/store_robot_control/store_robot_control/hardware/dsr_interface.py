from math import isfinite


def load_dsr_api(robot_id, robot_model, dsr_node):
    if dsr_node is None or dsr_node.executor is not None:
        raise ValueError("Doosan API에는 실행기에 등록되지 않은 전용 노드가 필요합니다.")

    import DR_init

    DR_init.__dsr__id = robot_id
    DR_init.__dsr__model = robot_model
    DR_init.__dsr__node = dsr_node

    import DSR_ROBOT2

    if DSR_ROBOT2.g_node is not dsr_node:
        raise RuntimeError("Doosan API가 이미 다른 노드에 연결되어 있습니다.")
    return DSR_ROBOT2


class DoosanRobotInterface:
    def __init__(self, robot_id, robot_model, tool_name, tcp_name, home_cartesian,
                 home_joint_deg, dry_run=False, dsr_node=None):
        self._home_cartesian = list(map(float, home_cartesian))
        self._pose = self._home_cartesian.copy()
        self._joints = list(map(float, home_joint_deg))
        self._dry_run = dry_run
        if not dry_run:
            self._api = load_dsr_api(robot_id, robot_model, dsr_node)

            from DR_common2 import posj, posx

            self._posj, self._posx = posj, posx
            self._ensure_tool_and_tcp(tool_name, tcp_name)

    def _ensure_tool_and_tcp(self, tool_name, tcp_name):
        current_tool = self._api.get_tool() if hasattr(self._api, "get_tool") else None
        current_tcp = self._api.get_tcp() if hasattr(self._api, "get_tcp") else None
        if current_tool == tool_name and current_tcp == tcp_name:
            return
        # 두산 제어기는 MANUAL 모드(0)에서만 set_tool / set_tcp를 허용합니다.
        previous_mode = self._api.get_robot_mode() if hasattr(self._api, "get_robot_mode") else None
        switched_manual = False
        try:
            if previous_mode is not None and previous_mode != 0 and hasattr(self._api, "set_robot_mode"):
                self._check_result(self._api.set_robot_mode(0), "set_robot_mode(0)")
                switched_manual = True
            if current_tool != tool_name:
                self._check_result(self._api.set_tool(tool_name), "set_tool")
            if current_tcp != tcp_name:
                self._check_result(self._api.set_tcp(tcp_name), "set_tcp")
        finally:
            if switched_manual:
                self._check_result(
                    self._api.set_robot_mode(previous_mode), f"set_robot_mode({previous_mode})",
                )

    @staticmethod
    def _check_result(result, operation):
        if result != 0:
            raise RuntimeError(f"Doosan {operation} 명령 실패 (반환값 {result})")

    @staticmethod
    def _six_values(values, operation):
        if values is None or isinstance(values, (int, float)):
            raise RuntimeError(f"Doosan {operation}에서 좌표를 읽지 못했습니다.")
        result = list(map(float, values))
        if len(result) != 6 or not all(map(isfinite, result)):
            raise ValueError(f"{operation}에는 유한한 좌표 6개가 필요합니다.")
        return result

    def get_current_pose_6d(self) -> list[float]:
        if self._dry_run:
            return self._pose.copy()
        response = self._api.get_current_posx(ref=self._api.DR_BASE)
        if not isinstance(response, tuple) or len(response) != 2:
            raise RuntimeError("Doosan get_current_posx에서 좌표를 읽지 못했습니다.")
        return self._six_values(response[0], "get_current_posx")

    def get_current_pose_4d(self) -> list[float]:
        pose = self.get_current_pose_6d()
        return [*pose[:3], pose[5]]

    def get_current_joints(self) -> list[float]:
        if self._dry_run:
            return self._joints.copy()
        return self._six_values(self._api.get_current_posj(), "get_current_posj")

    def move_joint(self, joints_deg: list[float], vel: float, acc: float):
        joints = self._six_values(joints_deg, "movej")
        if self._dry_run:
            self._pose[5] += joints[5] - self._joints[5]
            self._joints = joints
            return
        self._check_result(self._api.movej(
            self._posj(joints), vel=vel, acc=acc, mod=self._api.DR_MV_MOD_ABS,
        ), "movej")

    def move_linear_4d(self, pose_4d: list[float], vel: float, acc: float,
                       rx: float | None = None, ry: float | None = None):
        x, y, z, rz = pose_4d
        # ry≈180° ZYZ 짐벌락에서는 (rx=0, ry=180, rz)가 기준 하향 자세로 유일하게 표현됩니다.
        pose = self._six_values([
            x, y, z,
            0.0 if rx is None else rx,
            180.0 if ry is None else ry,
            rz,
        ], "movel")
        if self._dry_run:
            cur_eff = (
                self._pose[5] - self._pose[3]
                if abs(abs(self._pose[4]) - 180.0) < 5.0
                else self._pose[5]
            )
            tgt_eff = pose[5] - pose[3] if abs(abs(pose[4]) - 180.0) < 5.0 else pose[5]
            delta = (tgt_eff - cur_eff + 180.0) % 360.0 - 180.0
            self._joints[5] += delta
            self._pose = pose
            return
        self._check_result(self._api.movel(
            self._posx(pose), vel=vel, acc=acc, ref=self._api.DR_BASE,
            mod=self._api.DR_MV_MOD_ABS,
        ), "movel")

    def set_simulated_reference_pose(self, pose):
        if self._dry_run:
            self._pose = self._six_values(pose, "가상 기준 자세")
