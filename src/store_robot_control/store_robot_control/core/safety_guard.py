def clamp_z_min(z_mm: float, z_min_mm: float = 15.0) -> float:
    return max(z_mm, z_min_mm)


def is_j6_within_limit(j6_deg: float, j6_min: float = -150.0, j6_max: float = 150.0) -> bool:
    """Protect Eye-in-Hand D435i USB cable by enforcing J6 limits."""
    return j6_min <= j6_deg <= j6_max
