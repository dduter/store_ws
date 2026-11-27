from math import hypot, isfinite

from .safety_guard import load_settings


class DeadzoneFilter:
    def __init__(self, deadzone_px: float):
        if not isfinite(deadzone_px) or deadzone_px < 0:
            raise ValueError("데드존 반경은 유한한 음이 아닌 값이어야 합니다.")
        self.deadzone_px = deadzone_px

    def apply(self, dx: float, dy: float) -> tuple[float, float]:
        radius = hypot(dx, dy)
        if radius <= self.deadzone_px:
            return 0.0, 0.0
        scale = (radius - self.deadzone_px) / radius
        return dx * scale, dy * scale


class EmaLowPassFilter:
    def __init__(self, alpha: float):
        if not 0.0 < alpha <= 1.0:
            raise ValueError("EMA alpha는 0보다 크고 1 이하여야 합니다.")
        self.alpha = alpha
        self.reset()

    def update(self, sample: list[float] | tuple[float, ...]) -> list[float]:
        if self._value is None:
            self._value = list(sample)
        else:
            self._value = [
                previous + self.alpha * (current - previous)
                for previous, current in zip(self._value, sample, strict=True)
            ]
        return self._value.copy()

    def reset(self):
        self._value = None


class GripperGestureDebouncer:
    def __init__(self, debounce_sec: float = 0.5):
        if not isfinite(debounce_sec) or debounce_sec < 0:
            raise ValueError("디바운스 시간은 유한한 음이 아닌 값이어야 합니다.")
        self.debounce_sec = debounce_sec
        self.stable_closed = False
        self._pending_since = None

    def update(self, raw_closed: bool, now_sec: float) -> bool:
        if raw_closed == self.stable_closed:
            self._pending_since = None
        else:
            if self._pending_since is None:
                self._pending_since = now_sec
            if now_sec >= self._pending_since + self.debounce_sec:
                self.stable_closed = raw_closed
                self._pending_since = None
        return self.stable_closed


class TeleopWatchdog:
    def __init__(self, timeout_sec: float = 0.5):
        if not isfinite(timeout_sec) or timeout_sec < 0:
            raise ValueError("워치독 제한 시간은 유한한 음이 아닌 값이어야 합니다.")
        self.timeout_sec = timeout_sec
        self.reset()

    def feed(self, now_sec: float):
        self.last_heartbeat_sec = now_sec

    def is_expired(self, now_sec: float) -> bool:
        return (
            self.last_heartbeat_sec is None
            or now_sec > self.last_heartbeat_sec + self.timeout_sec
        )

    def reset(self):
        self.last_heartbeat_sec = None


class TeleopPipeline:
    """제어 설정에서 원격 조작 필터와 워치독을 구성합니다."""

    def __init__(self, source=None):
        config = {} if source is None else load_settings(source)
        settings = config.get("teleop", config)
        self.deadzone = DeadzoneFilter(settings.get("deadzone_px", 15.0))
        self.ema = EmaLowPassFilter(settings.get("ema_alpha", 0.3))
        self.gripper = GripperGestureDebouncer(settings.get("gripper_debounce_sec", 0.5))
        self.watchdog = TeleopWatchdog(settings.get("watchdog_timeout_sec", 0.5))

    def update_motion(self, dx: float, dy: float) -> list[float]:
        return self.ema.update(self.deadzone.apply(dx, dy))
