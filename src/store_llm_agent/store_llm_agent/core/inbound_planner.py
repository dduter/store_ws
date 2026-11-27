from typing import Any, Dict, List


def plan_lifo_restore(moved_stack: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Return temporary moved items in LIFO reverse order for restoration."""
    return list(reversed(moved_stack))
