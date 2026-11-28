import numpy as np
from scipy.spatial.transform import Rotation


def pixel_to_camera_3d(
    u: float,
    v: float,
    depth_mm: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> np.ndarray:
    """픽셀과 정렬 깊이(mm)를 카메라 기준 3차원 좌표(mm)로 역투영한다."""
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
    """카메라 기준 동차 좌표를 로봇 베이스 기준으로 변환한다."""
    p_base_h = t_base2gripper @ t_gripper2camera @ p_cam_h
    return p_base_h[:3]


def compute_grasp_z(
    z_surface_mm: float,
    grasp_depth_offset_mm: float,
    z_min_mm: float = 15.0,
) -> float:
    """표면 높이에서 파지 깊이를 빼고 최소 허용 높이를 적용한다."""
    return max(z_surface_mm - grasp_depth_offset_mm, z_min_mm)


def pose_6d_to_matrix(pose_6d) -> np.ndarray:
    """Doosan 베이스 기준 자세(mm, 내재 ZYZ 각도)를 동차 변환으로 변환한다."""
    pose = np.asarray(pose_6d, dtype=np.float64)
    if pose.shape != (6,) or not np.isfinite(pose).all():
        raise ValueError('자세에는 유한한 값 6개가 필요합니다.')
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = Rotation.from_euler('ZYZ', pose[3:], degrees=True).as_matrix()
    matrix[:3, 3] = pose[:3]
    return matrix


def normalize_rz_deg(angle_deg: float) -> float:
    """평행 그리퍼의 각도를 (-90, 90] 범위로 정규화한다."""
    angle = float(angle_deg)
    if not np.isfinite(angle):
        raise ValueError('회전 각도는 유한해야 합니다.')
    return 90.0 - ((90.0 - angle) % 180.0)


def camera_yaw_to_base_rz(
    yaw_deg: float,
    t_base2gripper: np.ndarray,
    t_gripper2camera: np.ndarray,
) -> float:
    """카메라 이미지 평면의 파지 각도(deg)를 로봇 베이스 XY 평면의 rz (-90, 90]로 변환한다."""
    rad = np.deg2rad(float(yaw_deg))
    if not np.isfinite(rad):
        raise ValueError('회전 각도는 유한해야 합니다.')
    r_base2cam = (t_base2gripper @ t_gripper2camera)[:3, :3]
    axis_base = r_base2cam @ np.array([np.cos(rad), np.sin(rad), 0.0], dtype=np.float64)
    if np.hypot(axis_base[0], axis_base[1]) < 1e-6:
        return normalize_rz_deg(yaw_deg)
    return round(normalize_rz_deg(float(np.rad2deg(np.arctan2(axis_base[1], axis_base[0])))), 2)


def is_valid_se3_matrix(matrix: np.ndarray) -> bool:
    """4x4 동차 변환 행렬(SE(3))의 직교성·행렬식·하단 행을 검증한다."""
    arr = np.asarray(matrix, dtype=np.float64)
    if arr.shape != (4, 4) or not np.isfinite(arr).all():
        return False
    if not np.allclose(arr[3], [0.0, 0.0, 0.0, 1.0], atol=1e-3):
        return False
    rot = arr[:3, :3]
    return bool(
        np.allclose(rot.T @ rot, np.eye(3), atol=1e-2)
        and np.isclose(np.linalg.det(rot), 1.0, atol=1e-2)
    )
