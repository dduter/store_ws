from enum import Enum


class StoreMode(str, Enum):
    CUSTOMER = 'CUSTOMER'
    EMPLOYEE = 'EMPLOYEE'
    REMOTE = 'REMOTE'


EMPLOYEE_ONLY_INTENTS = {'inspect', 'dispose', 'give_expired_box', 'inbound'}


class ModeFSM:
    """Manages store mode transitions and intent authorization."""

    def __init__(self, initial_mode: StoreMode = StoreMode.CUSTOMER) -> None:
        self.mode = initial_mode

    def update_from_vest(self, detected: bool, consecutive_frames: int, required_frames: int = 5) -> StoreMode:
        if detected and consecutive_frames >= required_frames and self.mode != StoreMode.REMOTE:
            self.mode = StoreMode.EMPLOYEE
        return self.mode

    def is_intent_allowed(self, intent: str) -> bool:
        if self.mode == StoreMode.CUSTOMER and intent in EMPLOYEE_ONLY_INTENTS:
            return False
        return True
