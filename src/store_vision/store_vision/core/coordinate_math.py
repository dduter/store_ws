import numpy as np


def pixel_to_camera_3d(
    u: float,
    v: float,
    depth_mm: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> np.ndarray:
    """Project 2D pixel + aligned depth (mm) into 3D camera coordinates (mm)."""
    if depth_mm <= 0.0:
        raise ValueError("depth_mm must be positive")
    x = (u - cx) * depth_mm / fx
    y = (v - cy) * depth_mm / fy
    return np.array([x, y, depth_mm, 1.0], dtype=np.float64)


def camera_to_base_3d(
    p_cam_h: np.ndarray,
    t_base2gripper: np.ndarray,
    t_gripper2camera: np.ndarray,
) -> np.ndarray:
    """Transform homogeneous 3D point from camera frame to robot base frame."""
    p_base_h = t_base2gripper @ t_gripper2camera @ p_cam_h
    return p_base_h[:3]


def compute_grasp_z(
    z_surface_mm: float,
    grasp_depth_offset_mm: float,
    z_min_mm: float = 15.0,
) -> float:
    """Apply grasp depth offset from top surface while enforcing Z_MIN safety clamp."""
    return max(z_surface_mm - grasp_depth_offset_mm, z_min_mm)
