from typing import Any, Dict, List


def assign_depth_layers(
    detections: List[Dict[str, Any]],
    layer_gap_mm: float = 25.0,
) -> List[Dict[str, Any]]:
    """Separate detections into top (layer=1) and lower (layer=2) by camera depth."""
    valid_depths = [d['depth_mm'] for d in detections if d.get('depth_mm', 0.0) > 0.0]
    if not valid_depths:
        return detections

    min_depth = min(valid_depths)
    result = []
    for det in detections:
        item = dict(det)
        depth = item.get('depth_mm', 0.0)
        item['depth_layer'] = 1 if depth <= (min_depth + layer_gap_mm) else 2
        result.append(item)
    return result
