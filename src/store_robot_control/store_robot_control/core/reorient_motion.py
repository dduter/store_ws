def can_retry_reorient(attempts: int, max_attempts: int = 1) -> bool:
    """Allow 180-degree reorientation at most max_attempts times."""
    return attempts < max_attempts
