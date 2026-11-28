from datetime import date
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from store_vision.core.catalog_utils import (
    find_config_dir,
    load_catalog_index,
    load_default_containers,
    load_scan_pose_6d,
    lookup_catalog,
)
from store_vision.core.coordinate_math import (
    camera_to_base_3d,
    compute_grasp_z,
    normalize_rz_deg,
    pixel_to_camera_3d,
    pose_6d_to_matrix,
)
from store_vision.core.depth_layer import assign_depth_layers
from store_vision.core.vision_vlm import (
    classify_inspection_result,
    inspect_crop_with_openai,
    inspect_crops_parallel,
    is_expired,
    parse_expiration_date,
    parse_vlm_json,
)


CONFIG = Path(__file__).resolve().parents[3] / 'config'
TODAY = date(2026, 11, 26)


def test_pose_identity():
    result = pose_6d_to_matrix([0] * 6)
    assert result.dtype == np.float64
    np.testing.assert_array_equal(result, np.eye(4))


def test_pose_top_down():
    result = pose_6d_to_matrix([100, 200, 300, 0, 180, 0])
    np.testing.assert_allclose(result, [
        [-1, 0, 0, 100],
        [0, 1, 0, 200],
        [0, 0, -1, 300],
        [0, 0, 0, 1],
    ], atol=1e-12)


def test_pose_intrinsic_zyz():
    a, b, c = np.deg2rad([37, -52, 113])
    ra = [[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]]
    rb = [[np.cos(b), 0, np.sin(b)], [0, 1, 0], [-np.sin(b), 0, np.cos(b)]]
    rc = [[np.cos(c), -np.sin(c), 0], [np.sin(c), np.cos(c), 0], [0, 0, 1]]
    result = pose_6d_to_matrix([14, -27, 76, 37, -52, 113])
    np.testing.assert_allclose(result[:3, :3], np.asarray(ra) @ rb @ rc)
    np.testing.assert_array_equal(result[:3, 3], [14, -27, 76])
    np.testing.assert_array_equal(result[3], [0, 0, 0, 1])


@pytest.mark.parametrize('pose', [[0] * 5, [0] * 7, [0] * 5 + [np.nan], [0] * 5 + [np.inf]])
def test_pose_invalid(pose):
    with pytest.raises(ValueError):
        pose_6d_to_matrix(pose)


def test_pixel_to_camera_and_base():
    point = pixel_to_camera_3d(350, 220, 400, fx=600, fy=400, cx=320, cy=240)
    np.testing.assert_allclose(point, [20, -20, 400, 1])
    np.testing.assert_allclose(camera_to_base_3d(point, np.eye(4), np.eye(4)), [20, -20, 400])
    base = pose_6d_to_matrix([100, 200, 500, 0, 180, 0])
    camera = pose_6d_to_matrix([5, 6, 7, 0, 0, 0])
    np.testing.assert_allclose(camera_to_base_3d(point, base, camera), [75, 186, 93])


def test_pixel_to_camera_requires_positive_depth():
    with pytest.raises(ValueError):
        pixel_to_camera_3d(320, 240, 0, 600, 600, 320, 240)


@pytest.mark.parametrize(('angle', 'expected'), [
    (0, 0), (90, 90), (-90, 90), (270, 90), (-270, 90), (100, -80), (-100, 80),
    (751.5, 31.5), (-89.999, -89.999),
])
def test_normalize_rz(angle, expected):
    assert normalize_rz_deg(angle) == pytest.approx(expected)


def test_compute_grasp_z_clamp():
    assert compute_grasp_z(50.0, 12.0, z_min_mm=15.0) == 38.0
    assert compute_grasp_z(20.0, 15.0, z_min_mm=15.0) == 15.0


@pytest.mark.parametrize(('depths', 'expected'), [
    ([325, 350, 342], [1, 1, 1]),
    ([350], [1]),
    ([300, 350, 325, 324.9], [2, 1, 1, 2]),
    ([0, -1, np.nan, np.inf, 350], [1, 1, 1, 1, 1]),
    ([0, -1], [1, 1]),
    ([], []),
])
def test_assign_depth_layers(depths, expected):
    detections = [{'depth_mm': depth} for depth in depths]
    assert [row['depth_layer'] for row in assign_depth_layers(detections)] == expected
    assert all('depth_layer' not in row for row in detections)


@pytest.mark.parametrize('raw', ['2026.11.26', '26.11.26', '2026-11-26', ' 2026/11/26 '])
def test_parse_expiration_date(raw):
    assert parse_expiration_date(raw) == TODAY


@pytest.mark.parametrize('raw', ['', 'unreadable', '2026.02.29', '2026.11/26', '2026.13.01'])
def test_invalid_expiration_date(raw):
    assert parse_expiration_date(raw) is None


def test_expiration_check():
    assert is_expired('2026.11.20', today=TODAY) is True
    assert is_expired('2026.11.26', today=TODAY) is False
    assert is_expired('2026.12.01', today=TODAY) is False
    assert is_expired('unreadable', today=TODAY) is None


@pytest.mark.parametrize(('raw', 'marker', 'backside', 'expected'), [
    ('26.11.25', True, True, ('NeedsReorientation', '')),
    ('26.11.25', True, False, ('MarkerContaminated', '검정 마카 오염')),
    (' 26.11.25 ', False, False, ('Expired', '유통기한 만료 (26.11.25)')),
    ('26.11.26', False, False, ('Normal', '')),
    ('', False, False, ('Unreadable', '')),
])
def test_classify_inspection(raw, marker, backside, expected):
    assert classify_inspection_result(raw, marker, is_backside=backside, today=TODAY) == expected


@pytest.mark.parametrize('wrapper', ['{}', '```json\n{}\n```', '```\n{}\n```'])
def test_parse_vlm_json(wrapper):
    payload = '{"expiration_raw": " 26.11.26 ", "marker_contaminated": true, "product_hint": "포스틱"}'
    assert parse_vlm_json(wrapper.format(payload)) == {
        'expiration_raw': '26.11.26', 'marker_contaminated': True, 'product_hint': '포스틱',
    }


@pytest.mark.parametrize('raw', [
    'bad JSON', '[1, 2]', 'null', '{"expiration_raw": 23, "marker_contaminated": "false", "product_hint": []}',
])
def test_parse_vlm_json_defaults(raw):
    assert parse_vlm_json(raw) == {'expiration_raw': '', 'marker_contaminated': False, 'product_hint': ''}


def test_inspect_crops_parallel_preserves_order():
    assert inspect_crops_parallel([3, 1, 2, 0], lambda value: {'crop': value}, max_workers=2) == [
        {'crop': 3}, {'crop': 1}, {'crop': 2}, {'crop': 0},
    ]
    assert inspect_crops_parallel([], lambda value: value) == []


def test_inspect_crop_with_mock_client():
    def create(**request):
        assert request['model'] == 'gpt-4o'
        assert request['timeout'] == 12.0
        image_url = request['messages'][0]['content'][1]['image_url']['url']
        assert image_url.startswith('data:image/jpeg;base64,')
        message = SimpleNamespace(content='{"expiration_raw":"26.11.25","product_hint":"포스틱"}')
        return SimpleNamespace(choices=[SimpleNamespace(message=message)])

    client = SimpleNamespace(chat=SimpleNamespace(completions=SimpleNamespace(create=create)))
    crop = np.zeros((8, 8, 3), dtype=np.uint8)
    assert inspect_crop_with_openai(crop, client=client, today=TODAY) == {
        'expiration_raw': '26.11.25', 'product_hint': '포스틱', 'status': 'Expired',
        'marker_contaminated': False, 'disposal_reason': '유통기한 만료 (26.11.25)',
    }

    def timeout(**_):
        raise TimeoutError

    client.chat.completions.create = timeout
    assert inspect_crop_with_openai(crop, client=client) == {
        'expiration_raw': '', 'product_hint': '', 'status': 'Unreadable',
        'marker_contaminated': False, 'disposal_reason': '',
    }


@pytest.mark.parametrize(('key', 'canonical', 'category', 'width'), [
    ('pepero_almond_front', 'pepero_almond_front', 'paper_box', 84.0),
    ('pepero', 'pepero_almond_front', 'paper_box', 84.0),
    ('PEPERO', 'pepero_almond_front', 'paper_box', 84.0),
    ('postick_front', 'postick_front', 'snack_bag', 0.0),
    ('drink_front', 'drink_front', 'bottle', 45.0),
    ('product_back', 'product_back', 'product_back', 84.0),
    ('ciga', 'ciga', 'ciga', 15.0),
    ('cigarette_mock', 'ciga', 'ciga', 15.0),
])
def test_real_catalog(key, canonical, category, width):
    item = lookup_catalog(load_catalog_index(CONFIG / 'products_catalog.yaml'), key)
    assert item['yolo_class'] == canonical
    assert item['category'] == category
    assert item['grasp_width_mm'] == width
    assert item.get('name') or item.get('display_name')


def test_catalog_front_fallback_and_unknown():
    index = {'sample_front': {'name': '상품'}}
    assert lookup_catalog(index, 'SAMPLE') == {'name': '상품'}
    assert lookup_catalog(index, 'unknown') == {}


def test_config_directory(monkeypatch, tmp_path):
    monkeypatch.delenv('STORE_CONFIG_DIR', raising=False)
    assert find_config_dir() == CONFIG
    monkeypatch.setenv('STORE_CONFIG_DIR', str(tmp_path))
    assert find_config_dir() == tmp_path


def test_real_workspace_zones():
    zones = CONFIG / 'workspace_zones.yaml'
    assert load_scan_pose_6d(zones) == pytest.approx(
        [295.733734, 18.080362, 402.806458, 60.482437, -177.727463, 57.507275],
    )
    assert load_default_containers(zones) == [
        dict(container_type='checkout', drop_pose_base=[-69.8, -477.4, -40.0], valid=True, confidence=1.0),
        dict(container_type='disposal', drop_pose_base=[520.0, -430.0, -10.0], valid=True, confidence=1.0),
    ]


def test_workspace_overrides_and_missing_file(tmp_path):
    zones = tmp_path / 'zones.yaml'
    assert load_scan_pose_6d(zones) == [295.73, 18.08, 402.81, 60.48, -177.73, 57.51]
    zones.write_text(
        'poses:\n  scan_pose_cartesian: [1, 2, 3, 4, 5, 6]\n'
        'zones:\n  containers:\n    checkout: {drop_center: [10, 20, 30, 40]}\n'
        '    disposal: [11, 22, 33]\n',
    )
    assert load_scan_pose_6d(zones) == [1, 2, 3, 4, 5, 6]
    assert [entry['drop_pose_base'] for entry in load_default_containers(zones)] == [
        [10, 20, 30], [11, 22, 33],
    ]


def test_camera_yaw_to_base_rz_and_se3_validation():
    from pathlib import Path
    from store_vision.core.coordinate_math import (
        camera_yaw_to_base_rz,
        is_valid_se3_matrix,
        pose_6d_to_matrix,
    )

    assert is_valid_se3_matrix(np.eye(4)) is True
    assert is_valid_se3_matrix(np.zeros((4, 4))) is False
    real_cal = Path(__file__).resolve().parents[3] / 'config' / 'calibration' / 'T_gripper2camera.npy'
    if real_cal.exists():
        assert is_valid_se3_matrix(np.load(real_cal)) is True

    # 90도 Z 회전된 카메라에서 이미지 0도 축은 베이스 90도 축이 됩니다.
    t_b2g = pose_6d_to_matrix([0.0, 0.0, 400.0, 90.0, 0.0, 0.0])
    assert camera_yaw_to_base_rz(0.0, t_b2g, np.eye(4)) == 90.0
    assert camera_yaw_to_base_rz(15.0, np.eye(4), np.eye(4)) == 15.0


def test_assign_depth_layers_side_by_side_vs_stacked():
    # 키가 다른 단품이 옆에 나란히 놓여 있을 때(XY 거리 250mm > 120mm)는 둘 다 1층
    side_by_side = [
        {'id': 'bottle', 'depth_mm': 220.0, 'xy': (300.0, 0.0)},
        {'id': 'pepero', 'depth_mm': 400.0, 'xy': (300.0, 250.0)},
    ]
    res = assign_depth_layers(side_by_side, layer_gap_mm=25.0)
    assert [r['depth_layer'] for r in res] == [1, 1]

    # 인접 위치(XY 거리 20mm <= 120mm)에 쌓여 있을 때는 위가 2층, 아래가 1층
    stacked = [
        {'id': 'top', 'depth_mm': 300.0, 'xy': (200.0, -450.0)},
        {'id': 'bottom', 'depth_mm': 350.0, 'xy': (210.0, -460.0)},
    ]
    res2 = assign_depth_layers(stacked, layer_gap_mm=25.0)
    assert [r['depth_layer'] for r in res2] == [2, 1]
