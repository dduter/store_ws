import os
from math import isfinite
from pathlib import Path

import yaml


def find_config_dir() -> Path:
    if config_dir := os.environ.get('STORE_CONFIG_DIR'):
        return Path(config_dir).expanduser().resolve()
    cwd = Path.cwd()
    for parent in (*Path(__file__).resolve().parents, cwd, *cwd.parents):
        config = parent / 'config'
        if (config / 'products_catalog.yaml').is_file():
            return config
    return Path.home() / 'store_ws' / 'config'


def _load_yaml(path: str | Path) -> dict:
    try:
        data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    except FileNotFoundError:
        return {}
    return data if isinstance(data, dict) else {}


def load_catalog_index(catalog_path: str | Path) -> dict[str, dict]:
    catalog = _load_yaml(catalog_path)
    categories = catalog.get('categories') or {}
    defaults = catalog.get('defaults') or {}
    index = {}
    for key, category in categories.items():
        index[key] = dict(defaults, **category)
        index[key].setdefault('category', key)
        index[key].setdefault('yolo_class', key)

    for key, product in (catalog.get('products') or {}).items():
        info = defaults | categories.get(product.get('category'), {}) | product
        info.setdefault('yolo_class', key)
        for alias in (key, info['yolo_class'], *(product.get('aliases') or [])):
            index[alias] = info
    for alias, key in (catalog.get('aliases') or {}).items():
        if key in index:
            index.setdefault(alias, index[key])
    for key, info in list(index.items()):
        index.setdefault(key.lower(), info)
    return index


def lookup_catalog(index: dict[str, dict], yolo_cls: str) -> dict:
    cls = yolo_cls.strip()
    for key in (cls, cls.lower(), f'{cls}_front', f'{cls.lower()}_front'):
        if key in index:
            return index[key]
    return {}


def _coordinates(values, size: int) -> list[float] | None:
    if not isinstance(values, (list, tuple)) or len(values) != size:
        return None
    try:
        coords = [float(value) for value in values]
    except (TypeError, ValueError, OverflowError):
        return None
    return coords if all(map(isfinite, coords)) else None


def load_scan_pose_6d(zones_path: str | Path) -> list[float]:
    poses = _load_yaml(zones_path).get('poses') or {}
    return _coordinates(poses.get('scan_pose_cartesian'), 6) or [
        295.73, 18.08, 402.81, 60.48, -177.73, 57.51,
    ]


def load_default_containers(zones_path: str | Path) -> list[dict]:
    zones = _load_yaml(zones_path).get('zones') or {}
    containers = zones.get('containers') or {}
    defaults = {'checkout': [-69.8, -477.4, -40.0], 'disposal': [520.0, -430.0, -10.0]}
    result = []
    for name, default in defaults.items():
        values = containers.get(name)
        if isinstance(values, dict):
            values = values.get('drop_pose_base', values.get('drop_center', values.get('drop_safe')))
        pose = _coordinates(values, 3)
        if pose is None and (pose_4d := _coordinates(values, 4)):
            pose = pose_4d[:3]
        result.append(dict(container_type=name, drop_pose_base=pose or default, valid=True, confidence=1.0))
    return result
