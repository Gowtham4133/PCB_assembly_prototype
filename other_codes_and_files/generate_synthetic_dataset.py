"""
=============================================================
  PCB SYNTHETIC DATASET GENERATOR
  Generates schematic-style PCB symbol images (R, C, L/LED)
  with YOLO-format annotations for YOLOv8 training.
=============================================================
  Classes: 0=C (Capacitor), 1=L (LED), 2=R (Resistor)
  Output : dataset/train|valid|test / images & labels
           dataset/data.yaml
=============================================================
"""

import os, math, random, shutil
from pathlib import Path
from PIL import Image, ImageDraw, ImageFilter
import yaml

# ─── CONFIG ──────────────────────────────────────────────────
IMG_SIZE   = 640
CLASSES    = ['C', 'L', 'R']          # must match data.yaml
N_TRAIN    = 600
N_VAL      = 120
N_TEST     = 60
SYMBOLS_PER_IMAGE = (2, 6)            # min/max symbols per image
SYMBOL_SIZE_RANGE = (45, 90)          # px
LEAD_THICKNESS    = 2

DATASET_DIR = Path("dataset")
SPLITS = {"train": N_TRAIN, "valid": N_VAL, "test": N_TEST}

SEED = 42
random.seed(SEED)

# ─── DRAWING HELPERS ─────────────────────────────────────────

def _rotate_pts(pts, cx, cy, angle):
    ca, sa = math.cos(angle), math.sin(angle)
    out = []
    for (px, py) in pts:
        rx = cx + (px - cx) * ca - (py - cy) * sa
        ry = cy + (px - cx) * sa + (py - cy) * ca
        out.append((rx, ry))
    return out


def draw_resistor(draw, cx, cy, size, angle=0):
    """IEC resistor: rectangle body + two leads."""
    bw, bh = size * 0.36, size * 0.60
    lead    = size * 0.22

    body = [
        (cx - bw/2, cy - bh/2),
        (cx + bw/2, cy - bh/2),
        (cx + bw/2, cy + bh/2),
        (cx - bw/2, cy + bh/2),
    ]
    lead_top  = [(cx, cy - bh/2 - lead), (cx, cy - bh/2)]
    lead_bot  = [(cx, cy + bh/2),         (cx, cy + bh/2 + lead)]

    body     = _rotate_pts(body,     cx, cy, angle)
    lead_top = _rotate_pts(lead_top, cx, cy, angle)
    lead_bot = _rotate_pts(lead_bot, cx, cy, angle)

    draw.polygon(body, outline='black', fill='white')
    draw.line(lead_top, fill='black', width=LEAD_THICKNESS)
    draw.line(lead_bot, fill='black', width=LEAD_THICKNESS)

    all_pts = body + lead_top + lead_bot
    xs = [p[0] for p in all_pts]; ys = [p[1] for p in all_pts]
    return min(xs), min(ys), max(xs), max(ys)


def draw_capacitor(draw, cx, cy, size):
    """Top-view through-hole cap: circle + inner square + dot."""
    r   = size * 0.42
    sq  = size * 0.10
    dot = max(2, size * 0.04)

    # outer circle
    draw.ellipse([cx-r, cy-r, cx+r, cy+r], outline='black', fill='white', width=2)
    # inner square (pin 1 marker)
    draw.rectangle([cx-sq, cy-sq*0.5, cx+sq, cy+sq*0.5],
                   outline='black', fill='white')
    # dot (pin 2)
    draw.ellipse([cx-dot, cy+sq*1.2-dot, cx+dot, cy+sq*1.2+dot], fill='black')

    return cx-r, cy-r, cx+r, cy+r


def draw_led(draw, cx, cy, size, angle=0):
    """LED schematic: rectangle body + diode triangle + bar + leads."""
    bw, bh  = size * 0.36, size * 0.60
    lead    = size * 0.22
    th, tb  = bh * 0.30, bh * 0.10  # triangle height/base half-width

    body = [
        (cx - bw/2, cy - bh/2),
        (cx + bw/2, cy - bh/2),
        (cx + bw/2, cy + bh/2),
        (cx - bw/2, cy + bh/2),
    ]
    lead_top = [(cx, cy - bh/2 - lead), (cx, cy - bh/2)]
    lead_bot = [(cx, cy + bh/2),         (cx, cy + bh/2 + lead)]
    triangle = [(cx, cy - th/2),         (cx - tb, cy + th/2), (cx + tb, cy + th/2)]
    bar      = [(cx - tb, cy + th/2 + bh*0.04), (cx + tb, cy + th/2 + bh*0.04)]

    body     = _rotate_pts(body,     cx, cy, angle)
    lead_top = _rotate_pts(lead_top, cx, cy, angle)
    lead_bot = _rotate_pts(lead_bot, cx, cy, angle)
    triangle = _rotate_pts(triangle, cx, cy, angle)
    bar      = _rotate_pts(bar,      cx, cy, angle)

    draw.polygon(body, outline='black', fill='white')
    draw.line(lead_top, fill='black', width=LEAD_THICKNESS)
    draw.line(lead_bot, fill='black', width=LEAD_THICKNESS)
    draw.polygon(triangle, outline='black', fill='black')
    draw.line(bar, fill='black', width=LEAD_THICKNESS + 1)

    all_pts = body + lead_top + lead_bot
    xs = [p[0] for p in all_pts]; ys = [p[1] for p in all_pts]
    return min(xs), min(ys), max(xs), max(ys)


# ─── BACKGROUND GENERATORS ───────────────────────────────────

def make_background():
    """Alternates between white, light-grey noise, and PCB-green."""
    style = random.choice(['white', 'noise', 'pcb'])
    if style == 'white':
        img = Image.new('RGB', (IMG_SIZE, IMG_SIZE), (255, 255, 255))
    elif style == 'noise':
        arr = [random.randint(230, 255) for _ in range(IMG_SIZE * IMG_SIZE * 3)]
        img = Image.frombytes('RGB', (IMG_SIZE, IMG_SIZE), bytes(arr))
        img = img.filter(ImageFilter.GaussianBlur(0.8))
    else:  # pcb green
        g   = random.randint(140, 170)
        arr = [random.randint(max(0,g-8), min(255,g+8)) if i%3==1
               else random.randint(20, 50) for i in range(IMG_SIZE * IMG_SIZE * 3)]
        img = Image.frombytes('RGB', (IMG_SIZE, IMG_SIZE), bytes(arr))
        img = img.filter(ImageFilter.GaussianBlur(0.5))
    return img


# ─── BBOX OVERLAP CHECK ──────────────────────────────────────

def _overlaps(box, placed, margin=12):
    x1, y1, x2, y2 = box
    x1 -= margin; y1 -= margin; x2 += margin; y2 += margin
    for (bx1, by1, bx2, by2) in placed:
        if x1 < bx2 and x2 > bx1 and y1 < by2 and y2 > by1:
            return True
    return False


# ─── SINGLE IMAGE GENERATOR ──────────────────────────────────

def generate_image():
    """Returns (PIL Image, list of (class_id, x_c, y_c, w, h) normalised)."""
    img  = make_background()
    draw = ImageDraw.Draw(img)

    n       = random.randint(*SYMBOLS_PER_IMAGE)
    labels  = []
    placed  = []            # bounding boxes already placed
    margin  = 60            # keep symbols away from edges

    for _ in range(n):
        cls    = random.randint(0, 2)       # 0=C 1=L 2=R
        size   = random.randint(*SYMBOL_SIZE_RANGE)
        angle  = random.choice([0, math.pi/2]) if cls != 0 else 0

        # try up to 20 positions
        for attempt in range(20):
            cx = random.randint(margin + size, IMG_SIZE - margin - size)
            cy = random.randint(margin + size, IMG_SIZE - margin - size)

            # tentative draw on a temp image to get bbox
            tmp  = Image.new('RGB', (IMG_SIZE, IMG_SIZE), (255, 255, 255))
            tmpd = ImageDraw.Draw(tmp)

            if   cls == 0: box = draw_capacitor(tmpd, cx, cy, size)
            elif cls == 1: box = draw_led(tmpd, cx, cy, size, angle)
            else:          box = draw_resistor(tmpd, cx, cy, size, angle)

            # clip check
            bx1, by1, bx2, by2 = box
            if bx1 < 2 or by1 < 2 or bx2 > IMG_SIZE-2 or by2 > IMG_SIZE-2:
                continue
            if _overlaps(box, placed):
                continue

            # commit draw
            if   cls == 0: draw_capacitor(draw, cx, cy, size)
            elif cls == 1: draw_led(draw, cx, cy, size, angle)
            else:          draw_resistor(draw, cx, cy, size, angle)

            placed.append((bx1, by1, bx2, by2))

            # YOLO normalised label
            xc = ((bx1 + bx2) / 2) / IMG_SIZE
            yc = ((by1 + by2) / 2) / IMG_SIZE
            bw = (bx2 - bx1)        / IMG_SIZE
            bh = (by2 - by1)        / IMG_SIZE
            labels.append((cls, xc, yc, bw, bh))
            break

    return img, labels


# ─── DATASET BUILDER ─────────────────────────────────────────

def build_dataset():
    print("═" * 55)
    print("  PCB Synthetic Dataset Generator")
    print("═" * 55)

    # clean previous
    if DATASET_DIR.exists():
        shutil.rmtree(DATASET_DIR)

    idx = 0
    for split, count in SPLITS.items():
        img_dir = DATASET_DIR / split / "images"
        lbl_dir = DATASET_DIR / split / "labels"
        img_dir.mkdir(parents=True, exist_ok=True)
        lbl_dir.mkdir(parents=True, exist_ok=True)

        print(f"  Generating {count:>4} images for [{split}] ...", end="", flush=True)
        for i in range(count):
            img, labels = generate_image()
            stem = f"pcb_{idx:05d}"
            img.save(img_dir / f"{stem}.jpg", quality=92)

            with open(lbl_dir / f"{stem}.txt", "w") as f:
                for (c, x, y, w, h) in labels:
                    f.write(f"{c} {x:.6f} {y:.6f} {w:.6f} {h:.6f}\n")
            idx += 1
        print(f" done ✓  ({idx} total)")

    # write data.yaml
    cfg = {
        "train": str((DATASET_DIR / "train" / "images").resolve()),
        "val":   str((DATASET_DIR / "valid" / "images").resolve()),
        "test":  str((DATASET_DIR / "test"  / "images").resolve()),
        "nc":    len(CLASSES),
        "names": CLASSES,
    }
    yaml_path = DATASET_DIR / "data.yaml"
    with open(yaml_path, "w") as f:
        yaml.dump(cfg, f, default_flow_style=False)

    print(f"\n  data.yaml saved → {yaml_path.resolve()}")
    print(f"  Total images    : {idx}")
    print(f"  Classes         : {CLASSES}")
    print("═" * 55)
    print("  Run next: python train_yolov8.py")
    print("═" * 55)


if __name__ == "__main__":
    build_dataset()
