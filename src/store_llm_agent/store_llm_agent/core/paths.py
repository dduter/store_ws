"""store_ws 워크스페이스 루트 및 설정 파일 경로 탐색 유틸리티."""

from __future__ import annotations

import os
from pathlib import Path


def _is_store_ws_root(path: Path) -> bool:
    """config/workspace_zones.yaml 또는 store_ontology.ttl 존재 여부로 루트를 판별한다."""
    return (
        (path / "config" / "workspace_zones.yaml").is_file()
        or (path / "config" / "ontology" / "store_ontology.ttl").is_file()
    )


def find_workspace_root(start_file: str | Path | None = None) -> Path:
    """환경변수, 소스 트리, 현재 작업 디렉토리 순으로 store_ws 루트를 탐색한다."""
    env_root = os.environ.get("STORE_WS_ROOT")
    if env_root:
        candidate = Path(env_root).expanduser().resolve()
        if _is_store_ws_root(candidate):
            return candidate

    search_roots: list[Path] = []
    if start_file is not None:
        search_roots.append(Path(start_file).resolve())
    search_roots.extend([
        Path.cwd().resolve(),
        Path.home() / "store_ws",
        Path.home() / "ws_cobot_pjt" / "store_ws",
    ])

    for root in search_roots:
        for candidate in (root, *root.parents):
            if _is_store_ws_root(candidate):
                return candidate

    if start_file is not None:
        parents = Path(start_file).resolve().parents
        if len(parents) > 4:
            return parents[4]
    return Path.cwd().resolve()
