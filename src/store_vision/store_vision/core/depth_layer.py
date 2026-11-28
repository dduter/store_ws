from math import hypot, isfinite


def assign_depth_layers(
    detections: list[dict],
    layer_gap_mm: float = 25.0,
    overlap_radius_mm: float = 120.0,
) -> list[dict]:
    """카메라에서 더 먼 하층 표면을 1층, 인접 하층 위에 적층된 상층을 2층으로 분류한다.

    검출 항목에 베이스 좌표 ``xy``(mm)가 있으면 인접 반경(``overlap_radius_mm``) 내의
    하층 상품과 비교해 나란히 놓인 키가 다른 단품의 오탐 적층 판정을 방지한다.
    """
    depths = [det.get('depth_mm', 0.0) for det in detections]
    valid = [isfinite(d) and d > 0 for d in depths]
    has_xy = any(isinstance(det.get('xy'), (list, tuple)) and len(det['xy']) >= 2 for det in detections)

    if not has_xy:
        max_depth = max((d for d, ok in zip(depths, valid) if ok), default=0.0)
        return [
            dict(det, depth_layer=2 if ok and max_depth - depth > layer_gap_mm else 1)
            for det, depth, ok in zip(detections, depths, valid)
        ]

    result = []
    for i, (det, depth, ok) in enumerate(zip(detections, depths, valid)):
        layer = 1
        xy = det.get('xy')
        if ok and isinstance(xy, (list, tuple)) and len(xy) >= 2:
            for j, (other, other_depth, other_ok) in enumerate(zip(detections, depths, valid)):
                if i == j or not other_ok:
                    continue
                oxy = other.get('xy')
                if not (isinstance(oxy, (list, tuple)) and len(oxy) >= 2):
                    continue
                if hypot(xy[0] - oxy[0], xy[1] - oxy[1]) <= overlap_radius_mm and other_depth - depth > layer_gap_mm:
                    layer = 2
                    break
        result.append(dict(det, depth_layer=layer))
    return result
