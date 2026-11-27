import json
from pathlib import Path
from typing import Any, Dict


def append_blackbox_event(log_path: str, event: Dict[str, Any]) -> None:
    path = Path(log_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('a', encoding='utf-8') as f:
        f.write(json.dumps(event, ensure_ascii=False) + '\n')
