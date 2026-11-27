from math import acos, cos, degrees, dist, isfinite, radians, sin


class MotionUnreachableError(RuntimeError):
    pass


def _zyz_matrix(rx_deg: float, ry_deg: float, rz_deg: float):
    ca, sa = cos(radians(rx_deg)), sin(radians(rx_deg))
    cb, sb = cos(radians(ry_deg)), sin(radians(ry_deg))
    cc, sc = cos(radians(rz_deg)), sin(radians(rz_deg))
    # Doosan ZYZ 오일러 각 회전행렬: R = Rz(rx) @ Ry(ry) @ Rz(rz)
    return (
        (ca * cb * cc - sa * sc, -ca * cb * sc - sa * cc, ca * sb),
        (sa * cb * cc + ca * sc, -sa * cb * sc + ca * cc, sa * sb),
        (-sb * cc, sb * sc, cb),
    )


def zyz_orientation_error_deg(actual_zyz, target_zyz) -> float:
    r1 = _zyz_matrix(*actual_zyz)
    r2 = _zyz_matrix(*target_zyz)
    trace = sum(r1[i][j] * r2[i][j] for i in range(3) for j in range(3))
    return degrees(acos(max(-1.0, min(1.0, (trace - 1.0) * 0.5))))


def top_down_effective_rz(pose_6d) -> float | None:
    if abs(abs(pose_6d[4]) - 180.0) >= 5.0:
        return None
    return (pose_6d[5] - pose_6d[3] + 180.0) % 360.0 - 180.0


def ensure_tool_and_tcp(api, tool_name: str, tcp_name: str):
    def check(result, operation):
        if result != 0:
            raise RuntimeError(f"Doosan {operation} 명령 실패 (반환값 {result})")

    current_tool = api.get_tool() if hasattr(api, "get_tool") else None
    current_tcp = api.get_tcp() if hasattr(api, "get_tcp") else None
    if current_tool != tool_name or current_tcp != tcp_name:
        # 두산 제어기는 MANUAL 모드(0)에서만 set_tool / set_tcp를 허용합니다.
        previous_mode = api.get_robot_mode() if hasattr(api, "get_robot_mode") else None
        switched_manual = False
        try:
            if previous_mode is not None and previous_mode != 0 and hasattr(api, "set_robot_mode"):
                check(api.set_robot_mode(0), "set_robot_mode(0)")
                switched_manual = True
            if current_tool != tool_name:
                check(api.set_tool(tool_name), "set_tool")
            if current_tcp != tcp_name:
                check(api.set_tcp(tcp_name), "set_tcp")
        finally:
            if switched_manual:
                check(api.set_robot_mode(previous_mode), f"set_robot_mode({previous_mode})")

    active_tool = api.get_tool() if hasattr(api, "get_tool") else tool_name
    active_tcp = api.get_tcp() if hasattr(api, "get_tcp") else tcp_name
    if active_tool != tool_name or active_tcp != tcp_name:
        raise RuntimeError(
            f"Doosan Tool/TCP 활성화 확인 실패 (요청: {tool_name}/{tcp_name}, 현재: {active_tool}/{active_tcp})"
        )
    return active_tool, active_tcp


def linear_j6_delta(current, target):
    # 하향 ZYZ 자세에서는 rz - rx가 공구의 유효 회전각입니다.
    current_rz = current[5] - current[3] if abs(abs(current[4]) - 180.0) < 5.0 else current[5]
    target_rz = target[5] - target[3] if abs(abs(target[4]) - 180.0) < 5.0 else target[5]
    return (target_rz - current_rz + 180.0) % 360.0 - 180.0


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
                 home_joint_deg, dry_run=False, dsr_node=None,
                 arrival_tolerance_mm: float = 15.0,
                 arrival_angle_tolerance_deg: float = 10.0):
        self._arrival_tolerance_mm = float(arrival_tolerance_mm)
        if not isfinite(self._arrival_tolerance_mm) or self._arrival_tolerance_mm < 0:
            raise ValueError("도달 허용 오차는 유한한 음이 아닌 값이어야 합니다.")
        self._arrival_angle_tolerance_deg = float(arrival_angle_tolerance_deg)
        if not isfinite(self._arrival_angle_tolerance_deg) or self._arrival_angle_tolerance_deg < 0:
            raise ValueError("자세 도달 허용 오차는 유한한 음이 아닌 값이어야 합니다.")
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
        ensure_tool_and_tcp(self._api, tool_name, tcp_name)

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

    def predict_linear_j6(self, pose):
        joints = self.get_current_joints()
        if self._dry_run:
            return joints[5], joints[5] + linear_j6_delta(self._pose, pose)
        # 현재 관절 해 공간의 역기구학 결과로 목표 J6를 확인합니다.
        solution = self._api.get_solution_space(self._posj(joints))
        if not isinstance(solution, int) or not 0 <= solution <= 7:
            raise RuntimeError("Doosan 관절 해 공간을 읽지 못했습니다.")
        target_joints = self._api.ikin(self._posx(pose), solution, ref=self._api.DR_BASE)
        try:
            return joints[5], self._six_values(target_joints, "ikin")[5]
        except (RuntimeError, ValueError, TypeError) as exc:
            raise MotionUnreachableError("목표 자세의 역기구학 해를 구하지 못했습니다.") from exc

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
        self.move_linear_6d([
            x, y, z,
            0.0 if rx is None else rx,
            180.0 if ry is None else ry,
            rz,
        ], vel, acc)

    def move_linear_6d(self, pose_6d: list[float], vel: float, acc: float):
        pose = self._six_values(pose_6d, "movel")
        if self._dry_run:
            self._joints[5] += linear_j6_delta(self._pose, pose)
            self._pose = pose
            return
        self._check_result(self._api.movel(
            self._posx(pose), vel=vel, acc=acc, ref=self._api.DR_BASE,
            mod=self._api.DR_MV_MOD_ABS,
        ), "movel")
        actual = self.get_current_pose_6d()
        distance = dist(actual[:3], pose[:3])
        if distance > self._arrival_tolerance_mm:
            raise MotionUnreachableError(
                f"목표 좌표 미도달 (오차 {distance:.1f}mm > {self._arrival_tolerance_mm:.1f}mm)"
            )
        angle_error = zyz_orientation_error_deg(actual[3:], pose[3:])
        if angle_error > self._arrival_angle_tolerance_deg:
            raise MotionUnreachableError(
                f"목표 자세 미도달 (회전 오차 {angle_error:.1f}deg > {self._arrival_angle_tolerance_deg:.1f}deg)"
            )

    def set_simulated_reference_pose(self, pose):
        if self._dry_run:
            self._pose = self._six_values(pose, "가상 기준 자세")
