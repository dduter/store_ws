"""Merge 5 Roboflow YOLOv8 datasets into a clean, balanced 5-class dataset."""

from __future__ import annotations

import ast
import random
import re
import shutil
from collections import Counter
from pathlib import Path

DATASETS_ROOT = Path("/home/ms-04/datasets")
OUTPUT_DIR = DATASETS_ROOT / "store_5classes"

TARGET_CLASSES = [
    "postick_front",
    "pepero_almond_front",
    "drink_front",
    "product_back",
    "vest",
]
TARGET_ID = {name: idx for idx, name in enumerate(TARGET_CLASSES)}

# Map raw Roboflow class names to the 5 canonical target classes.
# Any class not in this mapping (e.g., Gamjakkang, Ogamja, etc.) is dropped.
# (후순위: ciga 추후 추가)
CLASS_MAP = {
    "postick": "postick_front",
    "Postick": "postick_front",
    "pepero_almonds": "pepero_almond_front",
    "pepero_almond": "pepero_almond_front",
    "alloe": "drink_front",
    "drink_front": "drink_front",
    "Backside": "product_back",
    "product_back": "product_back",
    "vest": "vest",
    "Vest": "vest",
}

SOURCES = [
    {
        "dir": DATASETS_ROOT / "postick_front.v1i.yolov8",
        "prefix": "postick",
        "max_per_split": None,
    },
    {
        "dir": DATASETS_ROOT / "pepero_almond_front.v2i.yolov8",
        "prefix": "pepero",
        "max_per_split": {"train": 350, "valid": 60, "test": 30},
    },
    {
        "dir": DATASETS_ROOT / "aloe_drink_front.v1i.yolov8",
        "prefix": "aloe",
        "max_per_split": None,
    },
    {
        "dir": DATASETS_ROOT / "product_back.v1i.yolov8",
        "prefix": "back",
        "max_per_split": None,
    },
    {
        "dir": DATASETS_ROOT / "vest.v1i.yolov8",
        "prefix": "vest",
        "max_per_split": None,
        "optional": True,
    },
]


def load_names(data_yaml: Path) -> list[str]:
    text = data_yaml.read_text(encoding="utf-8")
    m = re.search(r"^names:\s*(\[.*?\])\s*$", text, re.MULTILINE)
    if m:
        return list(ast.literal_eval(m.group(1)))
    raise ValueError(f"Could not parse names from {data_yaml}")


def merge_datasets(seed: int = 42) -> None:
    rng = random.Random(seed)
    if OUTPUT_DIR.exists():
        shutil.rmtree(OUTPUT_DIR)

    split_image_counts: dict[str, int] = Counter()
    split_box_counts: dict[str, Counter] = {s: Counter() for s in ("train", "valid", "test")}

    for split in ("train", "valid", "test"):
        (OUTPUT_DIR / split / "images").mkdir(parents=True, exist_ok=True)
        (OUTPUT_DIR / split / "labels").mkdir(parents=True, exist_ok=True)

    for src in SOURCES:
        src_dir: Path = src["dir"]
        if not src_dir.exists():
            if src.get("optional"):
                print(f"[WARN] Optional dataset directory not found (skipping): {src_dir}")
                continue
            raise FileNotFoundError(f"Missing dataset directory: {src_dir}")
        raw_names = load_names(src_dir / "data.yaml")
        raw_id_to_target: dict[int, int] = {}
        for idx, raw_name in enumerate(raw_names):
            mapped = CLASS_MAP.get(raw_name)
            if mapped is not None:
                raw_id_to_target[idx] = TARGET_ID[mapped]

        for split in ("train", "valid", "test"):
            img_dir = src_dir / split / "images"
            lbl_dir = src_dir / split / "labels"
            if not img_dir.exists():
                continue

            valid_pairs: list[tuple[Path, list[str]]] = []
            for img_path in sorted(img_dir.iterdir()):
                if not img_path.is_file():
                    continue
                lbl_path = lbl_dir / f"{img_path.stem}.txt"
                if not lbl_path.exists():
                    continue
                new_lines: list[str] = []
                for line in lbl_path.read_text(encoding="utf-8").splitlines():
                    parts = line.strip().split()
                    if len(parts) < 5:
                        continue
                    raw_cls = int(parts[0])
                    if raw_cls in raw_id_to_target:
                        new_cls = raw_id_to_target[raw_cls]
                        new_lines.append(f"{new_cls} {' '.join(parts[1:])}")
                if new_lines:
                    valid_pairs.append((img_path, new_lines))

            max_map = src["max_per_split"]
            if max_map and split in max_map and len(valid_pairs) > max_map[split]:
                valid_pairs = rng.sample(valid_pairs, max_map[split])

            for img_path, new_lines in valid_pairs:
                new_stem = f"{src['prefix']}_{img_path.stem}"
                dst_img = OUTPUT_DIR / split / "images" / f"{new_stem}{img_path.suffix}"
                dst_lbl = OUTPUT_DIR / split / "labels" / f"{new_stem}.txt"
                shutil.copy2(img_path, dst_img)
                dst_lbl.write_text("\n".join(new_lines) + "\n", encoding="utf-8")
                split_image_counts[split] += 1
                for nl in new_lines:
                    cls_idx = int(nl.split()[0])
                    split_box_counts[split][TARGET_CLASSES[cls_idx]] += 1

    yaml_text = (
        f"path: {OUTPUT_DIR}\n"
        "train: train/images\n"
        "val: valid/images\n"
        "test: test/images\n\n"
        f"nc: {len(TARGET_CLASSES)}\n"
        f"names: {TARGET_CLASSES}\n"
    )
    (OUTPUT_DIR / "data.yaml").write_text(yaml_text, encoding="utf-8")

    print(f"Merged dataset saved to: {OUTPUT_DIR}")
    for split in ("train", "valid", "test"):
        print(f"  {split}: {split_image_counts[split]} images | boxes={dict(split_box_counts[split])}")


if __name__ == "__main__":
    merge_datasets()
