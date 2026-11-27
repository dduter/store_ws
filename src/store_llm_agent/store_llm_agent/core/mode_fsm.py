"""매장 운영 모드(CUSTOMER / EMPLOYEE / REMOTE) 상태 머신 및 명령 권한 가드레일."""

from __future__ import annotations

import time
from typing import Final

MODE_CUSTOMER: Final[str] = "CUSTOMER"
MODE_EMPLOYEE: Final[str] = "EMPLOYEE"
MODE_REMOTE: Final[str] = "REMOTE"

_CUSTOMER_ALLOWED_INTENTS: Final[frozenset[str]] = frozenset({"order", "query_stock", "clarify"})
_EMPLOYEE_ALLOWED_INTENTS: Final[frozenset[str]] = frozenset({
    "order",
    "query_stock",
    "inspect",
    "dispose",
    "inbound",
    "clarify",
})


class ModeFSM:
    """조끼 인식 디바운싱 기반 자동 모드 전환과 역할별 명령 권한을 관리한다."""

    def __init__(
        self,
        required_vest_frames: int = 5,
        vest_conf_threshold: float = 0.7,
        employee_timeout_sec: float = 5.0,
    ) -> None:
        self.required_vest_frames = required_vest_frames
        self.vest_conf_threshold = vest_conf_threshold
        self.employee_timeout_sec = employee_timeout_sec

        self._mode: str = MODE_CUSTOMER
        self._consecutive_vest_frames: int = 0
        self._last_vest_seen_time: float | None = None
        self._task_active: bool = False

    @property
    def mode(self) -> str:
        return self._mode

    @property
    def task_active(self) -> bool:
        return self._task_active

    def set_task_active(self, active: bool, now: float | None = None) -> None:
        """현재 로봇/작업 수행 중 여부를 설정한다. 작업 종료 시 타임아웃 기준 시각을 갱신한다."""
        ts = time.monotonic() if now is None else now
        self._task_active = active
        if not active and self._mode == MODE_EMPLOYEE:
            self._last_vest_seen_time = ts

    def update_vest_detection(
        self,
        detected: bool,
        confidence: float = 1.0,
        now: float | None = None,
    ) -> str:
        """웹캠 조끼 감지 프레임 결과를 반영하여 모드를 전이한다."""
        ts = time.monotonic() if now is None else now

        if self._mode == MODE_REMOTE:
            return self._mode

        is_valid_vest = detected and confidence >= self.vest_conf_threshold
        if is_valid_vest:
            self._consecutive_vest_frames += 1
            self._last_vest_seen_time = ts
            if self._consecutive_vest_frames >= self.required_vest_frames:
                self._mode = MODE_EMPLOYEE
            return self._mode

        # 미감지 또는 신뢰도 미달
        self._consecutive_vest_frames = 0
        if (
            self._mode == MODE_EMPLOYEE
            and not self._task_active
            and self._last_vest_seen_time is not None
            and (ts - self._last_vest_seen_time) >= self.employee_timeout_sec
        ):
            self._mode = MODE_CUSTOMER
            self._last_vest_seen_time = None

        return self._mode

    def set_employee_mode(self, enabled: bool) -> str:
        """CLI/테스트용으로 직원 모드 여부를 직접 전환한다."""
        self._mode = MODE_EMPLOYEE if enabled else MODE_CUSTOMER
        self._consecutive_vest_frames = self.required_vest_frames if enabled else 0
        self._last_vest_seen_time = time.monotonic() if enabled else None
        return self._mode

    def enter_remote(self) -> str:
        """관리자 인증 후 REMOTE 모드로 전환한다."""
        self._mode = MODE_REMOTE
        self._consecutive_vest_frames = 0
        self._task_active = False
        return self._mode

    def exit_remote(self) -> str:
        """REMOTE 모드를 종료하고 기본 CUSTOMER 모드로 복귀한다."""
        self._mode = MODE_CUSTOMER
        self._consecutive_vest_frames = 0
        self._last_vest_seen_time = None
        self._task_active = False
        return self._mode

    def check_permission(self, intent: str) -> tuple[bool, str]:
        """현재 모드에서 주어진 intent 실행이 허용되는지 검사한다."""
        if self._mode == MODE_REMOTE:
            return False, "REMOTE 모드에서는 자동 매장 명령을 실행할 수 없습니다."

        if intent == "remote":
            return (
                False,
                "원격 제어('remote')는 관리자 키 인증(/store/verify_remote_key)을 통해서만 전환할 수 있습니다.",
            )

        if self._mode == MODE_CUSTOMER:
            if intent in _CUSTOMER_ALLOWED_INTENTS:
                return True, "OK"
            return (
                False,
                f"CUSTOMER 모드에서는 '{intent}' 명령이 차단됩니다. (직원 조끼 인증 필요)",
            )

        if self._mode == MODE_EMPLOYEE:
            if intent in _EMPLOYEE_ALLOWED_INTENTS:
                return True, "OK"
            return False, f"EMPLOYEE 모드에서 지원하지 않는 intent입니다: '{intent}'"

        return False, f"알 수 없는 모드 상태입니다: '{self._mode}'"
