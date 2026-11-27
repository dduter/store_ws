def ema_filter(prev: float, curr: float, alpha: float = 0.25) -> float:
    return alpha * curr + (1.0 - alpha) * prev


def apply_deadzone(delta: float, deadzone: float = 15.0) -> float:
    return 0.0 if abs(delta) < deadzone else delta


def debounce_hold_ready(hold_duration_sec: float, required_sec: float = 0.5) -> bool:
    return hold_duration_sec >= required_sec
