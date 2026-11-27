def clamp_grasp_params(width_mm: float, force_n: float) -> tuple[float, float]:
    """Clamp RG2 gripper width (0-110mm) and force (5-40N)."""
    w = max(0.0, min(110.0, width_mm))
    f = max(5.0, min(40.0, force_n))
    return w, f
