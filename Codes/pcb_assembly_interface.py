"""
=============================================================
  PCB ASSEMBLY APP  -  v11 Final
=============================================================
  Workspace  : 200 x 200 mm
  X + Y map  : pixel 0 -> 1mm,  pixel 640 -> 201mm
  Feeder Y   : R=50mm  C=100mm  L=120mm  (Arduino side)

  WORKFLOW:
    Home -> Y to feeder_Y (pick) -> PICK ->
    Y to cy_px->mm, X to cx_px->mm (place) -> PLACE -> Home

  TABLE: ID | Class | Conf | CX px | CY px | X mm | Y mm
=============================================================
"""

import argparse, json, threading, time
from pathlib import Path
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from PIL import Image, ImageDraw, ImageTk

try:
    from ultralytics import YOLO as _YOLO
    YOLO_OK = True
except ImportError:
    YOLO_OK = False
    print("[WARN] ultralytics not installed - pip install ultralytics")

try:
    import serial, serial.tools.list_ports
    SERIAL_OK = True
except ImportError:
    SERIAL_OK = False
    print("[WARN] pyserial not installed - pip install pyserial")

# ─── CONSTANTS ────────────────────────────────────────────────
CLASS_COLORS  = {0: "#00BFFF", 1: "#FF6B35", 2: "#7CFC00"}
CLASS_NAMES   = {0: "C (Capacitor)", 1: "L (LED)", 2: "R (Resistor)"}
DEFAULT_CONF  = 0.25
DEFAULT_PORT  = "COM3"
BAUD_RATE     = 115200
DISPLAY_SIZE  = (560, 560)
DEFAULT_MODEL = r"C:\Users\ELCOT\AppData\Local\Programs\Python\Python310\1.pcb_detect\best.pt"

# Workspace constants (must match Arduino WS_MM and OFFSET)
IMG_W   = 640
IMG_H   = 640
WS_MM   = 200.0
OFFSET  = 1.0

def px_to_mm(px, img_dim=640):
    """pixel -> physical mm. Same formula as Arduino pixToSteps()."""
    return round(OFFSET + (px / img_dim) * WS_MM, 2)


# ─── DETECTION ───────────────────────────────────────────────

def run_detection(model, image_path, conf=DEFAULT_CONF):
    results    = model(str(image_path), conf=conf)[0]
    detections = []
    for box in results.boxes:
        cls   = int(box.cls[0])
        score = float(box.conf[0])
        x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
        h, w  = results.orig_shape
        cx_px = round((x1 + x2) / 2)
        cy_px = round((y1 + y2) / 2)
        detections.append({
            "class_id"  : cls,
            "class_name": CLASS_NAMES.get(cls, f"cls{cls}"),
            "conf"      : round(score, 3),
            "x1": round(x1), "y1": round(y1),
            "x2": round(x2), "y2": round(y2),
            "cx_px" : cx_px,
            "cy_px" : cy_px,
            "cx_mm" : px_to_mm(cx_px, w),
            "cy_mm" : px_to_mm(cy_px, h),
        })
    return detections


def annotate_image(image_path, detections):
    img  = Image.open(image_path).convert("RGB")
    draw = ImageDraw.Draw(img)
    for d in detections:
        color = CLASS_COLORS.get(d["class_id"], "#FF0000")
        draw.rectangle([d["x1"], d["y1"], d["x2"], d["y2"]],
                       outline=color, width=3)
        label = (f"{d['class_name']} {d['conf']:.0%} "
                 f"X:{d['cx_mm']}mm Y:{d['cy_mm']}mm")
        lx, ly = d["x1"] + 4, d["y1"] - 20
        draw.rectangle([lx-2, ly-2, lx + len(label)*7, ly+14],
                       fill=color)
        draw.text((lx, ly), label, fill="black")
    return img


# ─── PAYLOAD ─────────────────────────────────────────────────

def build_payload(detections):
    """Send cx_px and cy_px. Arduino converts both to mm internally."""
    components = []
    for i, d in enumerate(detections):
        components.append({
            "id"       : i,
            "component": d["class_name"].split()[0],   # "C", "L", "R"
            "cx_px"    : d["cx_px"],
            "cy_px"    : d["cy_px"],
            "cx_mm"    : d["cx_mm"],   # reference only
            "cy_mm"    : d["cy_mm"],   # reference only
        })
    return json.dumps({"components": components,
                       "count": len(components)}) + "\n"


# ─── SERIAL ──────────────────────────────────────────────────

def send_to_esp(port, baud, payload, status_cb):
    """
    Connect WITHOUT resetting ESP8266 (dtr=False, rts=False).
    Wait for PCB-ROBOT-READY, send payload, wait for ACK-DONE.
    """
    if not SERIAL_OK:
        status_cb("ERROR: pyserial not installed")
        return
    try:
        ser = serial.Serial()
        ser.port     = port
        ser.baudrate = baud
        ser.timeout  = 0.1
        ser.dtr      = False   # KEY: no ESP reset on connect
        ser.rts      = False
        ser.open()

        status_cb(f"Connected {port} (no reset). Waiting for READY...")

        deadline = time.time() + 180
        ready    = False
        while time.time() < deadline:
            if ser.in_waiting:
                try:
                    line = ser.readline().decode("utf-8",
                                                 errors="ignore").strip()
                except Exception:
                    line = ""
                if line:
                    status_cb(f"ESP: {line}")
                if "PCB-ROBOT-READY" in line:
                    ready = True
                    break
            time.sleep(0.05)

        if not ready:
            status_cb("TIMEOUT: READY not received. Check ESP power.")
            ser.close()
            return

        ser.reset_input_buffer()
        ser.write(payload.encode("utf-8"))
        ser.flush()
        status_cb("Sent. Waiting for ACK-DONE...")

        deadline2 = time.time() + 300
        while time.time() < deadline2:
            if ser.in_waiting:
                try:
                    line = ser.readline().decode("utf-8",
                                                 errors="ignore").strip()
                except Exception:
                    line = ""
                if line:
                    status_cb(f"ESP: {line}")
                if "ACK-DONE" in line:
                    status_cb("Job complete! ACK-DONE received.")
                    break
                if "JSON-ERR" in line:
                    status_cb(f"ESP error: {line}")
                    break
                if "BUSY" in line:
                    status_cb("ESP busy - wait for current job.")
                    break
            else:
                time.sleep(0.02)
        else:
            status_cb("Timeout waiting for ACK-DONE")

        ser.close()

    except serial.SerialException as e:
        status_cb(f"Serial error: {e}")
    except Exception as e:
        status_cb(f"Error: {e}")


# ─── GUI ─────────────────────────────────────────────────────

class PCBApp(tk.Tk):
    def __init__(self, model_path=None):
        super().__init__()
        self.title("PCB Assembly v11 - 200x200mm")
        self.resizable(True, True)
        self.configure(bg="#1C1C2E")

        self._model      = None
        self._model_path = model_path or DEFAULT_MODEL
        self._detections = []
        self._image_path = None
        self._photo      = None

        self._build_ui()
        self._load_model_async()

    def _build_ui(self):
        # Top bar
        bar = tk.Frame(self, bg="#12122A", pady=8)
        bar.pack(fill="x")
        tk.Label(bar, text="PCB Vision Assembly v11",
                 bg="#12122A", fg="#7DF9FF",
                 font=("Courier", 15, "bold")).pack(side="left", padx=14)

        mf = tk.Frame(bar, bg="#12122A")
        mf.pack(side="left", padx=8)
        tk.Label(mf, text="Model:", bg="#12122A", fg="#AAA",
                 font=("Courier", 9)).pack(side="left")
        self.model_var = tk.StringVar(
            value=Path(self._model_path).name
            if self._model_path else "Not loaded")
        tk.Label(mf, textvariable=self.model_var,
                 bg="#12122A", fg="#7DF9FF",
                 font=("Courier", 9)).pack(side="left", padx=4)
        tk.Button(mf, text="Browse...", command=self._browse_model,
                  bg="#2A2A4A", fg="white", font=("Courier", 8),
                  relief="flat", padx=5).pack(side="left")

        tk.Label(bar, text="200x200mm | R=50 C=100 L=120mm | SPM=200",
                 bg="#12122A", fg="#FF6B35",
                 font=("Courier", 8)).pack(side="right", padx=14)

        # Content
        content = tk.Frame(self, bg="#1C1C2E")
        content.pack(fill="both", expand=True, padx=8, pady=6)

        # Left: canvas
        left = tk.Frame(content, bg="#1C1C2E")
        left.pack(side="left", fill="both", expand=True)

        self.canvas = tk.Canvas(left, width=DISPLAY_SIZE[0],
                                height=DISPLAY_SIZE[1],
                                bg="#0D0D1A", highlightthickness=1,
                                highlightbackground="#333")
        self.canvas.pack()
        self.canvas.create_text(DISPLAY_SIZE[0]//2, DISPLAY_SIZE[1]//2,
                                text="Select an image to begin",
                                fill="#444", font=("Courier", 13))

        btn_row = tk.Frame(left, bg="#1C1C2E")
        btn_row.pack(fill="x", pady=5)
        self._btn("Open Image",    self._open_image,
                  btn_row).pack(side="left", padx=4)
        self._btn("Run Detection", self._detect,
                  btn_row).pack(side="left", padx=4)

        self.conf_var = tk.DoubleVar(value=DEFAULT_CONF)
        tk.Label(btn_row, text="Conf:", bg="#1C1C2E", fg="#AAA",
                 font=("Courier", 9)).pack(side="left", padx=(10, 2))
        tk.Scale(btn_row, variable=self.conf_var,
                 from_=0.05, to=0.95, resolution=0.05,
                 orient="horizontal", length=90,
                 bg="#1C1C2E", fg="#7DF9FF",
                 highlightthickness=0,
                 troughcolor="#2A2A4A").pack(side="left")

        # Right: table + ESP
        right = tk.Frame(content, bg="#1C1C2E", width=400)
        right.pack(side="right", fill="both", padx=(8, 0))

        tk.Label(right, text="Detected Components",
                 bg="#1C1C2E", fg="#7DF9FF",
                 font=("Courier", 11, "bold")).pack(anchor="w")

        cols = ("ID", "Class", "Conf", "CX px", "CY px", "X mm", "Y mm")
        self.tree = ttk.Treeview(right, columns=cols,
                                 show="headings", height=13)
        sty = ttk.Style()
        sty.theme_use("clam")
        sty.configure("Treeview",
                      background="#12122A", foreground="#EEE",
                      fieldbackground="#12122A", rowheight=22,
                      font=("Courier", 8))
        sty.configure("Treeview.Heading",
                      background="#2A2A4A", foreground="#7DF9FF",
                      font=("Courier", 8, "bold"))
        col_w = {"ID": 28, "Class": 100, "Conf": 46,
                 "CX px": 54, "CY px": 54, "X mm": 56, "Y mm": 56}
        for c in cols:
            self.tree.heading(c, text=c)
            self.tree.column(c, width=col_w[c], anchor="center")
        self.tree.pack(fill="both", expand=True, pady=3)

        # ESP panel
        esp = tk.LabelFrame(right, text=" ESP8266 Serial ",
                            bg="#1C1C2E", fg="#FF6B35",
                            font=("Courier", 9), bd=1, relief="groove")
        esp.pack(fill="x", pady=4)

        pf = tk.Frame(esp, bg="#1C1C2E")
        pf.pack(fill="x", padx=4, pady=3)
        tk.Label(pf, text="Port:", bg="#1C1C2E", fg="#AAA",
                 font=("Courier", 9)).pack(side="left")
        self.port_var = tk.StringVar(value=DEFAULT_PORT)
        ports = ([p.device for p in serial.tools.list_ports.comports()]
                 if SERIAL_OK else [DEFAULT_PORT])
        if not ports:
            ports = [DEFAULT_PORT]
        self.port_combo = ttk.Combobox(pf, textvariable=self.port_var,
                                       values=ports, width=9,
                                       font=("Courier", 9))
        self.port_combo.pack(side="left", padx=4)
        self._btn("R", self._refresh_ports, pf,
                  width=2).pack(side="left")

        self._btn("Send to ESP8266", self._send_esp,
                  esp).pack(fill="x", padx=4, pady=(0, 3))

        self.payload_text = tk.Text(
            esp, height=4, bg="#0D0D1A", fg="#7DF9FF",
            font=("Courier", 8), state="disabled",
            relief="flat", wrap="none")
        self.payload_text.pack(fill="x", padx=4, pady=(0, 5))

        self.status_var = tk.StringVar(
            value="Ready | 200x200mm | SPM=200 | R=50 C=100 L=120mm")
        tk.Label(self, textvariable=self.status_var,
                 bg="#0D0D1A", fg="#7DF9FF", font=("Courier", 9),
                 anchor="w", padx=10, pady=3).pack(fill="x", side="bottom")

    def _btn(self, text, cmd, parent=None, **kw):
        if parent is None:
            parent = self
        return tk.Button(parent, text=text, command=cmd,
                         bg="#2A2A4A", fg="#7DF9FF",
                         font=("Courier", 10, "bold"),
                         relief="flat", padx=10, pady=4,
                         activebackground="#3A3A6A",
                         activeforeground="white", **kw)

    def _load_model_async(self):
        if not YOLO_OK or not self._model_path:
            return
        def _load():
            self._status("Loading model...")
            try:
                self._model = _YOLO(self._model_path)
                self._status(
                    f"Model loaded ({Path(self._model_path).name})")
                self.model_var.set(Path(self._model_path).name)
            except Exception as e:
                self._status(f"Model load error: {e}")
        threading.Thread(target=_load, daemon=True).start()

    def _browse_model(self):
        init = (str(Path(self._model_path).parent)
                if self._model_path else "C:\\")
        path = filedialog.askopenfilename(
            title="Select best.pt", initialdir=init,
            filetypes=[("PyTorch weights", "*.pt"),
                       ("All files", "*.*")])
        if path:
            self._model_path = path
            self._load_model_async()

    def _open_image(self):
        path = filedialog.askopenfilename(
            title="Select PCB Image",
            filetypes=[("Images", "*.jpg *.jpeg *.png *.bmp"),
                       ("All", "*.*")])
        if not path:
            return
        self._image_path = path
        self._detections = []
        self._update_canvas(Image.open(path).convert("RGB"))
        self._status(f"Image loaded: {Path(path).name}")

    def _update_canvas(self, pil_img):
        pil_img.thumbnail(DISPLAY_SIZE, Image.LANCZOS)
        self._photo = ImageTk.PhotoImage(pil_img)
        self.canvas.delete("all")
        self.canvas.create_image(DISPLAY_SIZE[0]//2, DISPLAY_SIZE[1]//2,
                                 image=self._photo, anchor="center")

    def _detect(self):
        if not self._image_path:
            messagebox.showwarning("No Image", "Open an image first.")
            return
        if not YOLO_OK:
            messagebox.showerror("Missing", "ultralytics not installed.")
            return
        if self._model is None:
            messagebox.showwarning("No Model",
                f"Model not loaded.\nPath:\n{self._model_path}")
            return
        self._status("Running detection...")
        def _run():
            try:
                dets = run_detection(self._model, self._image_path,
                                     self.conf_var.get())
                ann  = annotate_image(self._image_path, dets)
                self.after(0, lambda: self._show_results(dets, ann))
            except Exception as e:
                self.after(0,
                    lambda: self._status(f"Detection error: {e}"))
        threading.Thread(target=_run, daemon=True).start()

    def _show_results(self, detections, annotated_img):
        self._detections = detections
        self._update_canvas(annotated_img)

        for row in self.tree.get_children():
            self.tree.delete(row)
        for i, d in enumerate(detections):
            tag = ("C" if d["class_id"] == 0
                   else ("L" if d["class_id"] == 1 else "R"))
            self.tree.insert("", "end", iid=str(i),
                             values=(i,
                                     d["class_name"],
                                     f"{d['conf']:.0%}",
                                     d["cx_px"],
                                     d["cy_px"],
                                     f"{d['cx_mm']}mm",
                                     f"{d['cy_mm']}mm"),
                             tags=(tag,))
        self.tree.tag_configure("C", foreground="#00BFFF")
        self.tree.tag_configure("L", foreground="#FF6B35")
        self.tree.tag_configure("R", foreground="#7CFC00")

        payload = build_payload(detections)
        self.payload_text.config(state="normal")
        self.payload_text.delete("1.0", "end")
        self.payload_text.insert("end", payload)
        self.payload_text.config(state="disabled")

        c = sum(1 for d in detections if d["class_id"] == 0)
        l = sum(1 for d in detections if d["class_id"] == 1)
        r = sum(1 for d in detections if d["class_id"] == 2)
        self._status(
            f"Detected {len(detections)}  C:{c}  L:{l}  R:{r}")

    def _refresh_ports(self):
        if SERIAL_OK:
            ports = [p.device
                     for p in serial.tools.list_ports.comports()]
            self.port_combo["values"] = ports or [DEFAULT_PORT]
            self._status("Ports refreshed")

    def _send_esp(self):
        if not self._detections:
            messagebox.showwarning("No Data", "Run detection first.")
            return
        payload = build_payload(self._detections)
        port    = self.port_var.get()
        self.payload_text.config(state="normal")
        self.payload_text.delete("1.0", "end")
        self.payload_text.insert("end", payload)
        self.payload_text.config(state="disabled")
        self._status(f"Connecting to {port}...")
        threading.Thread(
            target=send_to_esp,
            args=(port, BAUD_RATE, payload, self._status),
            daemon=True
        ).start()

    def _status(self, msg):
        self.after(0, lambda: self.status_var.set(msg))


# ─── ENTRY POINT ─────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--port",  default=DEFAULT_PORT)
    args = parser.parse_args()
    DEFAULT_PORT = args.port
    app = PCBApp(model_path=args.model)
    app.mainloop()
