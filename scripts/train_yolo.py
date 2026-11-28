"""Reusable YOLOv8 training script for store_ws product models."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

from ultralytics import YOLO

DEFAULT_DATA = "/home/ms-04/datasets/store_5classes/data.yaml"
DEFAULT_PROJECT = "/home/ms-04/store_ws/runs"
DEPLOY_WEIGHTS = Path("/home/ms-04/store_ws/config/weights/products_yolov8n.pt")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train YOLOv8 product detection model.")
    parser.add_argument("--data", default=DEFAULT_DATA, help="Path to data.yaml")
    parser.add_argument("--model", default="yolov8n.pt", help="Base model or checkpoint")
    parser.add_argument("--epochs", type=int, default=50, help="Number of training epochs")
    parser.add_argument("--imgsz", type=int, default=640, help="Input image size")
    parser.add_argument("--batch", type=int, default=16, help="Batch size")
    parser.add_argument("--device", default="0", help="CUDA device id or 'cpu'")
    parser.add_argument("--project", default=DEFAULT_PROJECT, help="Runs output directory")
    parser.add_argument("--name", default="products_train", help="Experiment folder name")
    parser.add_argument(
        "--deploy",
        action="store_true",
        help="Copy trained best.pt to config/weights/products_yolov8n.pt",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    data_path = Path(args.data)
    if not data_path.exists():
        raise FileNotFoundError(f"data.yaml not found: {data_path}")

    model = YOLO(args.model)
    results = model.train(
        data=str(data_path),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        project=args.project,
        name=args.name,
        exist_ok=True,
    )

    best_pt = Path(results.save_dir) / "weights" / "best.pt"
    print(f"\n[train_yolo] Training finished. best.pt -> {best_pt}")

    if args.deploy and best_pt.exists():
        DEPLOY_WEIGHTS.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(best_pt, DEPLOY_WEIGHTS)
        print(f"[train_yolo] Deployed to -> {DEPLOY_WEIGHTS}")


if __name__ == "__main__":
    main()
