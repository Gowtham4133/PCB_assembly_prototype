"""
=============================================================
  TRAIN YOLOv8 ON SYNTHETIC PCB DATASET
  Run AFTER generate_synthetic_dataset.py
=============================================================
  Output: runs/detect/pcb_yolo/weights/best.pt
=============================================================
"""

from ultralytics import YOLO
from pathlib import Path
import yaml, sys

# ─── CONFIG ──────────────────────────────────────────────────
DATA_YAML  = "dataset/data.yaml"
BASE_MODEL = "yolov8n.pt"           # nano – fast training
EPOCHS     = 80
IMG_SIZE   = 640
BATCH      = 8                      # lower if GPU memory is limited
PROJECT    = "runs/detect"
NAME       = "pcb_yolo"
# ─────────────────────────────────────────────────────────────

def check_dataset():
    p = Path(DATA_YAML)
    if not p.exists():
        print(f"[ERROR] {DATA_YAML} not found.")
        print("        Run:  python generate_synthetic_dataset.py  first.")
        sys.exit(1)
    with open(p) as f:
        cfg = yaml.safe_load(f)
    train_path = Path(cfg["train"])
    n = len(list(train_path.glob("*.jpg"))) if train_path.exists() else 0
    print(f"  Dataset  : {DATA_YAML}")
    print(f"  Classes  : {cfg['names']}")
    print(f"  Train imgs: {n}")


def train():
    print("═" * 55)
    print("  YOLOv8 PCB Training")
    print("═" * 55)
    check_dataset()

    model = YOLO(BASE_MODEL)
    results = model.train(
        data    = DATA_YAML,
        epochs  = EPOCHS,
        imgsz   = IMG_SIZE,
        batch   = BATCH,
        project = PROJECT,
        name    = NAME,
        patience= 20,           # early stopping
        save    = True,
        plots   = True,
        verbose = True,
    )

    best = Path(PROJECT) / NAME / "weights" / "best.pt"
    print("\n" + "═" * 55)
    if best.exists():
        print(f"  ✓  best.pt  →  {best.resolve()}")
        print(f"\n  Run next: python pcb_assembly_app.py --model {best}")
    else:
        print("  Training finished – check runs/detect/ for weights.")
    print("═" * 55)


if __name__ == "__main__":
    train()
