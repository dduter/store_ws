from datetime import date
import numpy as np

from store_vision.core.coordinate_math import camera_to_base_3d, compute_grasp_z, pixel_to_camera_3d
from store_vision.core.depth_layer import assign_depth_layers
from store_vision.core.vision_vlm import is_expired


def test_pixel_to_base_identity():
    p_cam = pixel_to_camera_3d(320.0, 240.0, 400.0, fx=600.0, fy=600.0, cx=320.0, cy=240.0)
    assert np.allclose(p_cam, [0.0, 0.0, 400.0, 1.0])
    p_base = camera_to_base_3d(p_cam, np.eye(4), np.eye(4))
    assert np.allclose(p_base, [0.0, 0.0, 400.0])


def test_compute_grasp_z_clamp():
    assert compute_grasp_z(50.0, 12.0, z_min_mm=15.0) == 38.0
    assert compute_grasp_z(20.0, 15.0, z_min_mm=15.0) == 15.0


def test_assign_depth_layers():
    dets = [{'id': 'top', 'depth_mm': 300.0}, {'id': 'bottom', 'depth_mm': 350.0}]
    layered = assign_depth_layers(dets, layer_gap_mm=25.0)
    assert layered[0]['depth_layer'] == 1
    assert layered[1]['depth_layer'] == 2


def test_expiration_check():
    ref = date(2026, 11, 26)
    assert is_expired('2026.11.20', today=ref) is True
    assert is_expired('2026.12.01', today=ref) is False
    assert is_expired('unreadable', today=ref) is None
