from inspect import signature
from math import isfinite
from time import monotonic, sleep


class RG2Gripper:
    UNIT_ID = 65
    WIDTH_REGISTER = 267
    STATUS_REGISTER = 268
    MOVE_COMMAND = 16
    LIMITS = {"rg2": (110.0, 40.0), "rg6": (160.0, 120.0)}

    def __init__(self, ip, port, gripper_type="rg2", wait_after_move_sec=0.6, dry_run=False):
        self._max_width, self._max_force = self.LIMITS[gripper_type.lower()]
        if not isfinite(wait_after_move_sec) or wait_after_move_sec < 0:
            raise ValueError("그리퍼 대기 시간은 유한한 0 이상의 수여야 합니다.")
        self._ip, self._port = ip, port
        self._delay = wait_after_move_sec
        self._dry_run = dry_run
        self._client = None
        self._width_mm = self._max_width
        self._force_n = 0.0
        self._closing = False
        self.simulate_grip_fail = False

    def connect(self):
        if self._dry_run or self._client is not None:
            return True

        from pymodbus.client import ModbusTcpClient

        client = ModbusTcpClient(self._ip, port=self._port, timeout=1.0)
        try:
            if not client.connect():
                raise ConnectionError("그리퍼 Modbus 연결에 실패했습니다.")
        except Exception:
            client.close()
            raise
        self._client = client
        return True

    def close(self):
        if self._client is not None:
            client, self._client = self._client, None
            client.close()

    def _request(self, method_name, **kwargs):
        if self._client is None:
            raise ConnectionError("그리퍼 Modbus 연결이 필요합니다.")
        method = getattr(self._client, method_name)
        unit_key = "device_id" if "device_id" in signature(method).parameters else "slave"
        response = method(**kwargs, **{unit_key: self.UNIT_ID})
        if response.isError():
            raise ConnectionError(f"그리퍼 Modbus 요청 실패 ({method_name}): {response}")
        return response

    def _read_register(self, address):
        return self._request("read_holding_registers", address=address, count=1).registers[0]

    def _move(self, width_mm, force_n, *, closing):
        if not isfinite(width_mm) or not 0 <= width_mm <= self._max_width:
            raise ValueError("그리퍼 폭이 허용 범위를 벗어났습니다.")
        if not isfinite(force_n) or not 3 <= force_n <= self._max_force:
            raise ValueError("그리퍼 힘이 허용 범위를 벗어났습니다.")
        width_reg = round(width_mm * 10)
        force_reg = round(force_n * 10)
        if self._dry_run:
            self._width_mm = width_reg / 10.0
            self._force_n = force_reg / 10.0
            self._closing = closing
            return
        self._request("write_registers", address=0, values=[force_reg, width_reg, self.MOVE_COMMAND])
        sleep(self._delay)
        if not self.wait_until_idle():
            raise TimeoutError("그리퍼가 제한 시간 내에 정지하지 않았습니다.")

    def move_mm(self, width_mm, force_n):
        self._move(width_mm, force_n, closing=True)

    def open_mm(self, open_width_mm, force_n):
        self._move(open_width_mm, force_n, closing=False)

    def get_width_mm(self) -> float:
        if self._dry_run:
            return self._width_mm
        return self._read_register(self.WIDTH_REGISTER) / 10.0

    def get_status_bits(self) -> dict:
        if self._dry_run:
            status = 2 if self._closing and not self.simulate_grip_fail else 0
        else:
            status = self._read_register(self.STATUS_REGISTER)
        return {"busy": bool(status & 1), "grip_detected": bool(status & 2)}

    def is_grip_detected(self) -> bool:
        return self.get_status_bits()["grip_detected"]

    def wait_until_idle(self, timeout_sec=3.0) -> bool:
        deadline = monotonic() + timeout_sec
        while self.get_status_bits()["busy"]:
            remaining = deadline - monotonic()
            if remaining <= 0:
                return False
            sleep(min(0.05, remaining))
        return True
