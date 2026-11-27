"""24시 무인편의점 LLM 및 온톨로지 에이전트 패키지."""

from __future__ import annotations

from pathlib import Path
import sys


def _ensure_venv_site_packages() -> None:
    """ros2 run/launch가 시스템 python3로 실행될 때도 .venv 의존성(rdflib, networkx, openai 등)을 로드한다."""
    candidates = [
        Path.home() / "ws_cobot_pjt" / "store_ws" / ".venv",
        Path.home() / "ws_cobot_pjt" / "ws_dsr" / ".venv",
    ]
    ver = f"python{sys.version_info.major}.{sys.version_info.minor}"
    for venv_dir in candidates:
        site_dir = venv_dir / "lib" / ver / "site-packages"
        if site_dir.is_dir():
            s = str(site_dir)
            if s not in sys.path:
                sys.path.append(s)


_ensure_venv_site_packages()
