from pathlib import Path
from typing import List


class MockCamera:
    """Offline snapshot provider for development without physical D435i."""

    def __init__(self, snapshots_dir: str) -> None:
        self.snapshots_dir = Path(snapshots_dir)

    def list_images(self) -> List[Path]:
        if not self.snapshots_dir.exists():
            return []
        exts = {'.png', '.jpg', '.jpeg'}
        return sorted(p for p in self.snapshots_dir.iterdir() if p.suffix.lower() in exts)
