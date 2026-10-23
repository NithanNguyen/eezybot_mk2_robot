#!/usr/bin/env python3
"""
colorcal.py - Hieu chuan va danh gia phan loai mau nap chai (xanh la / do / trang)
bang module GY-33 (TCS34725, che do MCU_IIC) tren firmware env "robot" cua repo nay.

Phuong phap: r = R/(R+G+B), g = G/(R+G+B) tu RGBC tho; tam lop gan nhat (Euclid);
khoang cach > d_max -> khong xac dinh (vut bo); |C - C_empty| < C_margin -> khong co nap.

Lenh (chay tu thu muc goc repo):
  python3 tools/colorcal/colorcal.py wizard     # toan bo quy trinh (khuyen dung)
  python3 tools/colorcal/colorcal.py ports | check | tune | train | push | live | report
  python3 tools/colorcal/colorcal.py collect --set train --n 30
  python3 tools/colorcal/colorcal.py collect --set test --n 20 --labels green,red,white,empty,other
  python3 tools/colorcal/colorcal.py evaluate --device
Tuy chon chung: --port, --ws <thu muc lam viec>, --sim (thiet bi MO PHONG), --yes, -v
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

# ----------------------------------------------------------------------------- hang so
CLASSES = ("green", "red", "white")                 # thu tu = thu tu tam lop trong firmware
ALL_LABELS = CLASSES + ("empty", "other")          # "other": vat la, ky vong bi loai bo
PRED_LABELS = ("green", "red", "white", "unknown", "no_cap")
EXPECTED = {"green": "green", "red": "red", "white": "white", "empty": "no_cap", "other": "unknown"}
FW2PRED = {"GREEN": "green", "RED": "red", "WHITE": "white", "UNKNOWN": "unknown", "NO_CAP": "no_cap"}
VI = {"green": "xanh lá", "red": "đỏ", "white": "trắng", "empty": "trạm trống",
      "other": "vật lạ", "unknown": "không xác định", "no_cap": "không có nắp"}
HOWTO = {
    "green": "Đặt nắp XANH LÁ vào trạm",
    "red": "Đặt nắp ĐỎ vào trạm",
    "white": "Đặt nắp TRẮNG vào trạm",
    "empty": "Để trạm TRỐNG (không có nắp)",
    "other": "Đặt một vật KHÔNG thuộc 3 màu (vd. nắp xanh dương) - để kiểm tra chức năng loại bỏ",
}
# GY-33: LED muc 0-10 (so nho = sang hon), mac dinh 3 (tai lieu hang tr.4-6).
LED_DEFAULT, LED_MAX = 3, 10
DEFAULT_CFG = {"led": LED_DEFAULT, "navg": 5, "settle_ms": 300}
READ_INTERVAL_MS = 120          # = ColorClassifier::READ_INTERVAL_MS
RAW_MAX = 65535                 # = colormath::RAW_MAX
PLATEAU_GAIN = 1.02             # sang hon 1 muc ma C tang < 2% -> co the bao hoa (tune)
DEFAULT_K_SIGMA = 3.0           # d_max = max_k(mean_k + K*std_k)
MIN_SAMPLES = 5
CFG_KEYS = ("led", "navg", "settle_ms")
CSV_FIELDS = ["time", "set", "label", "R", "G", "B", "C", "sat"] + list(CFG_KEYS)

ROOT = Path(__file__).resolve().parents[2]          # goc repo
FW_HEADER = ROOT / "include" / "color_model.h"


def meas_time_s(cfg):
    return (cfg["settle_ms"] + (cfg["navg"] - 1) * READ_INTERVAL_MS) / 1000.0


def cfg_str(cfg):
    return "LED mức {}, navg={}, chờ {} ms (≈ {:.2f} s/phép đo)".format(
        cfg["led"], cfg["navg"], cfg["settle_ms"], meas_time_s(cfg))


# ----------------------------------------------------------------------------- tien ich
class Ctx:
    def __init__(self, args):
        self.args = args
        self.ws = Path(args.ws).resolve() if args.ws else ROOT
        self.data = self.ws / "data" / "color"
        self.reports = self.ws / "reports" / "color"
        self.data.mkdir(parents=True, exist_ok=True)
        self.reports.mkdir(parents=True, exist_ok=True)
        self.yes = bool(args.yes)
        self.sim = bool(args.sim)
        self._dev = None

    @property
    def config_path(self):
        return self.data / "config.json"

    @property
    def model_path(self):
        return self.data / "model.json"

    def csv_path(self, set_name):
        return self.data / "{}.csv".format(set_name)

    def header_path(self):
        # Model mo phong khong duoc ghi vao firmware that.
        return (self.reports / "color_model.h") if self.sim else FW_HEADER

    def ask(self, prompt, default=""):
        if self.yes:
            print(prompt + " [tự động]")
            return default
        try:
            return input(prompt)
        except EOFError:
            return default

    def device(self):
        if self._dev is None:
            self._dev = Device(port=self.args.port, sim=self.sim, verbose=self.args.verbose)
        return self._dev

    def close(self):
        if self._dev is not None:
            self._dev.close()
            self._dev = None


def die(msg, code=1):
    print("\n[x] " + msg, file=sys.stderr)
    sys.exit(code)


def load_json(path):
    with open(str(path), encoding="utf-8") as f:
        return json.load(f)


def save_json(path, obj):
    with open(str(path), "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=2, ensure_ascii=False)


def load_config(ctx):
    if ctx.config_path.exists():
        cfg = load_json(ctx.config_path)
        return {k: int(cfg[k]) for k in CFG_KEYS}
    return dict(DEFAULT_CFG)


# ----------------------------------------------------------------------------- thiet bi
class DeviceError(RuntimeError):
    pass


class Device:
    """Giao tiep voi firmware (that hoac mo phong) theo giao thuc trong src/robot/color_console.h."""

    def __init__(self, port=None, baud=115200, sim=False, verbose=False):
        self.verbose = verbose
        self.sim = sim
        if sim:
            self.ser = SimSerial()
            print("[i] Dùng THIẾT BỊ MÔ PHỎNG (--sim). Số liệu là giả lập, chỉ để chạy thử phần mềm.")
        else:
            self.ser = self._open_serial(port, baud)
        self._handshake()

    @staticmethod
    def _open_serial(port, baud):
        try:
            import serial  # pyserial
        except ImportError:
            die("Thiếu pyserial: pip3 install -r tools/colorcal/requirements.txt")
        if not port:
            port = auto_port()
        ser = serial.Serial()
        ser.port, ser.baudrate, ser.timeout = port, baud, 0.2
        ser.dtr = False          # DTR/RTS = False truoc open(): khong vao bootloader
        ser.rts = False
        try:
            ser.exclusive = True
        except (AttributeError, ValueError):
            pass
        try:
            ser.open()
        except serial.SerialException as e:
            die("Không mở được {}: {}\n    -> Đóng Serial Monitor; kiểm tra quyền "
                "(sudo usermod -aG dialout $USER rồi đăng xuất).".format(port, e))
        print("[i] Đã mở {} @ {}".format(port, baud))
        return ser

    def _handshake(self):
        time.sleep(0 if self.sim else 0.3)
        self.ser.reset_input_buffer()
        for _ in range(6):
            try:
                self.cmd("PING", "OK PONG", timeout=1.0)
                return
            except DeviceError:
                time.sleep(0 if self.sim else 0.5)
        die("ESP32 không trả lời PING. Kiểm tra: đã nạp firmware (pio run -e robot -t upload), "
            "đúng cổng, baud 115200; thử nhấn nút EN.")

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def cmd(self, line, prefix, timeout=5.0):
        if self.verbose:
            print("  >> " + line)
        self.ser.write((line + "\n").encode())
        deadline = time.time() + timeout
        while time.time() < deadline:
            raw = self.ser.readline()
            if not raw:
                if self.sim:
                    break
                continue
            resp = raw.decode("utf-8", errors="replace").strip()
            if self.verbose:
                print("  << " + resp)
            if not resp:
                continue
            if resp.startswith("#"):
                if "ERR" in resp or "LED" in resp:
                    print("  [thiết bị] " + resp[1:].strip())
                continue
            if resp.startswith("ERR"):
                raise DeviceError(resp)
            if resp.startswith(prefix):
                return resp
        raise DeviceError("Hết thời gian chờ phản hồi '{}' cho lệnh '{}'".format(prefix, line))

    def info(self):
        out = {}
        for part in self.cmd("INFO", "INFO").split(",")[1:]:
            if "=" in part:
                k, v = part.split("=", 1)
                out[k] = v
        return out

    def set_led(self, level):
        self.cmd("LED {}".format(int(level)), "OK LED")

    def measure(self, navg, settle_ms):
        timeout = (settle_ms + navg * READ_INTERVAL_MS) / 1000.0 + 3.0
        f = self.cmd("MEAS {} {}".format(int(navg), int(settle_ms)), "MEAS,", timeout=timeout).split(",")
        return {"R": float(f[1]), "G": float(f[2]), "B": float(f[3]), "C": float(f[4]),
                "sat": int(f[5]), "led": int(f[6])}

    @staticmethod
    def _parse_cls(resp):
        f = resp.split(",")
        return {"label": FW2PRED.get(f[1], f[1].lower()), "nearest": FW2PRED.get(f[2], f[2].lower()),
                "dist": float(f[3]), "r": float(f[4]), "g": float(f[5]),
                "R": float(f[6]), "G": float(f[7]), "B": float(f[8]), "C": float(f[9]), "sat": int(f[10])}

    def classify(self):
        return self._parse_cls(self.cmd("CLS", "CLS,", timeout=10.0))

    def classify_raw(self, R, G, B, C, sat):
        line = "CLSRAW {:.3f} {:.3f} {:.3f} {:.3f} {}".format(R, G, B, C, int(sat))
        return self._parse_cls(self.cmd(line, "CLS,"))

    def model_get(self):
        f = self.cmd("MODEL GET", "MODEL,").split(",")
        if f[1] == "none":
            return None
        v = [float(x) for x in f[2:]]
        return {"source": f[1], "led": int(v[0]), "navg": int(v[1]), "settle_ms": int(v[2]),
                "centroids": [[v[3], v[4]], [v[5], v[6]], [v[7], v[8]]],
                "dmax": v[9], "c_empty": v[10], "c_margin": v[11]}

    def model_set(self, m):
        self.cmd("MODEL SET " + " ".join(model_values_str(m)), "OK MODEL", timeout=5.0)

    def set_sim_object(self, label):
        if self.sim:
            self.ser.obj = label


def model_values(m):
    return [m["led"], m["navg"], m["settle_ms"]] + [x for c in m["centroids"] for x in c] + \
           [m["dmax"], m["c_empty"], m["c_margin"]]


def model_values_str(m):
    """12 gia tri theo thu tu cua MODEL SET (3 so nguyen + 9 so thuc)."""
    return [str(int(x)) if i < 3 else "{:.8g}".format(x) for i, x in enumerate(model_values(m))]


def auto_port():
    try:
        from serial.tools import list_ports
    except ImportError:
        die("Thiếu pyserial: pip3 install -r tools/colorcal/requirements.txt")
    ports = [p for p in list_ports.comports() if p.device.startswith(("/dev/ttyUSB", "/dev/ttyACM", "COM"))]
    if len(ports) == 1:
        return ports[0].device
    if not ports:
        die("Không thấy cổng serial nào. Cắm ESP32 rồi chạy lại, hoặc chỉ định --port.")
    die("Có nhiều cổng: {} - chỉ định bằng --port.".format(", ".join(p.device for p in ports)))


# ----------------------------------------------------------------------------- thiet bi mo phong
class SimSerial:
    """Mo phong firmware + GY-33 de chay thu quy trinh khi chua co phan cung.
    Cac he so la GIA DINH (khong phai do that)."""

    BASE = {"green": (2600, 5200, 3400), "red": (7000, 1900, 1700), "white": (8200, 8600, 7400),
            "empty": (700, 750, 680), "other": (1500, 2600, 6200)}

    def __init__(self, seed=1234):
        self.rng = np.random.RandomState(seed)
        self.led = LED_DEFAULT
        self.model = None
        self.obj = "empty"
        self.out = []

    def reset_input_buffer(self):
        self.out = []

    def close(self):
        pass

    def readline(self):
        return (self.out.pop(0) + "\n").encode() if self.out else b""

    def _measure(self, navg):
        k = (LED_MAX + 1 - self.led) / 8.0               # gia dinh: do sang LED theo muc
        place = self.rng.normal(1.0, 0.06)
        chroma = 1.0 + self.rng.normal(0, 0.015, 3)
        acc, sat = np.zeros(4), 0
        for _ in range(navg):
            rgb = np.array(self.BASE[self.obj]) * k * place * chroma * (1 + self.rng.normal(0, 0.004, 3))
            v = np.clip(np.r_[rgb, rgb.sum() * 1.03], 0, RAW_MAX).astype(int)
            acc += v
            sat |= int((v >= RAW_MAX).any())
        acc /= navg
        return acc[0], acc[1], acc[2], acc[3], sat

    def _cls_line(self, R, G, B, C, sat):
        r = classify_one(self.model, R, G, B, C, sat)
        inv = {v: k for k, v in FW2PRED.items()}
        return "CLS,{},{},{:.6f},{:.6f},{:.6f},{:.3f},{:.3f},{:.3f},{:.3f},{}".format(
            inv[r["label"]], inv[r["nearest"]], r["dist"], r["r"], r["g"], R, G, B, C, sat)

    def write(self, data):
        t = data.decode().strip().split()
        if not t:
            return
        c, emit = t[0].upper(), self.out.append
        if c == "PING":
            emit("OK PONG")
        elif c == "INFO":
            emit("INFO,fw=sim,sensor=1,led={},model={}".format(self.led, "nvs" if self.model else "none"))
        elif c == "LED":
            self.led = int(t[1])
            emit("OK LED,{}".format(self.led))
        elif c == "MEAS":
            R, G, B, C, sat = self._measure(int(t[1]))
            emit("MEAS,{:.3f},{:.3f},{:.3f},{:.3f},{},{}".format(R, G, B, C, sat, self.led))
        elif c in ("CLS", "CLSRAW"):
            if not self.model:
                emit("ERR NO_MODEL")
            elif c == "CLS":
                self.led = self.model["led"]
                emit(self._cls_line(*self._measure(self.model["navg"])))
            else:
                emit(self._cls_line(float(t[1]), float(t[2]), float(t[3]), float(t[4]), int(t[5])))
        elif c == "MODEL":
            sub = t[1].upper()
            if sub == "GET":
                emit("MODEL,none" if not self.model else
                     "MODEL,nvs," + ",".join(str(v) for v in model_values(self.model)))
            elif sub == "SET":
                v = t[2:]
                self.model = {"led": int(v[0]), "navg": int(v[1]), "settle_ms": int(v[2]),
                              "centroids": [[float(v[3]), float(v[4])], [float(v[5]), float(v[6])],
                                            [float(v[7]), float(v[8])]],
                              "dmax": float(v[9]), "c_empty": float(v[10]), "c_margin": float(v[11])}
                self.led = self.model["led"]
                emit("OK MODEL,nvs")
            elif sub == "CLEAR":
                self.model = None
                emit("OK MODEL,cleared")
        else:
            emit("ERR UNKNOWN_CMD")


# ----------------------------------------------------------------------------- thuat toan
F32 = np.float32


def classify_one(model, R, G, B, C, sat):
    """Ban Python cua colormath::classify(), tinh bang float32 giong ESP32."""
    R, G, B, C = F32(R), F32(G), F32(B), F32(C)
    s = R + G + B
    rn = R / s if s > 0 else F32(0)
    gn = G / s if s > 0 else F32(0)
    best, dbest = 0, F32(1e30)
    for k, (cx, cy) in enumerate(model["centroids"]):
        dr, dg = rn - F32(cx), gn - F32(cy)
        d = F32(math.sqrt(F32(dr * dr + dg * dg)))
        if d < dbest:
            best, dbest = k, d
    if abs(C - F32(model["c_empty"])) < F32(model["c_margin"]):
        label = "no_cap"
    elif sat or s < 1:
        label = "unknown"
    elif dbest > F32(model["dmax"]):
        label = "unknown"
    else:
        label = CLASSES[best]
    return {"label": label, "nearest": CLASSES[best], "dist": float(dbest), "r": float(rn), "g": float(gn)}


def read_rows(path):
    if not path.exists():
        return []
    rows = []
    with open(str(path), newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            for k in ("R", "G", "B", "C"):
                r[k] = float(r[k])
            for k in ("sat",) + CFG_KEYS:
                r[k] = int(r[k])
            rows.append(r)
    return rows


def chroma_of(rows):
    X = np.array([[r["R"], r["G"], r["B"]] for r in rows])
    s = X.sum(axis=1)
    return np.c_[X[:, 0] / s, X[:, 1] / s]


def check_cfg_consistency(rows, what):
    keys = {tuple(r[k] for k in CFG_KEYS) for r in rows}
    if len(keys) != 1:
        die("{} có nhiều cấu hình đo khác nhau {} - mọi mẫu phải dùng cùng LED/navg/chờ. "
            "Xoá file và thu lại.".format(what, sorted(keys)))
    return dict(zip(CFG_KEYS, keys.pop()))


def fit_model(rows, k_sigma):
    """Tam lop, d_max, nguong kenh C tu tap huan luyen. Tra ve (model, thong ke)."""
    stats = {"classes": {}}
    cents = []
    for lab in CLASSES:
        sub = [r for r in rows if r["label"] == lab]
        X = chroma_of(sub)
        c = X.mean(axis=0)
        d = np.linalg.norm(X - c, axis=1)
        Cv = np.array([r["C"] for r in sub])
        cents.append(c)
        stats["classes"][lab] = {"n": len(sub), "centroid": c.tolist(), "d_mean": float(d.mean()),
                                 "d_std": float(d.std(ddof=1)), "d_max": float(d.max()),
                                 "c_rel_std": float(Cv.std(ddof=1) / Cv.mean())}
    dmax = max(s["d_mean"] + k_sigma * s["d_std"] for s in stats["classes"].values())
    cents = np.array(cents)
    pair = {"{}-{}".format(CLASSES[i], CLASSES[j]): float(np.linalg.norm(cents[i] - cents[j]))
            for i in range(3) for j in range(i + 1, 3)}
    stats["centroid_dist"] = pair
    stats["dmin"] = min(pair.values())
    stats["sep_ratio"] = dmax / (stats["dmin"] / 2.0)    # < 1: vung chap nhan khong chong lan

    Ce = np.array([r["C"] for r in rows if r["label"] == "empty"])
    Cc = np.array([r["C"] for r in rows if r["label"] in CLASSES])
    c_empty = float(Ce.mean())
    e_spread = float(np.abs(Ce - c_empty).max())
    cap_gap = float(np.abs(Cc - c_empty).min())
    stats["c"] = {"empty_mean": c_empty, "empty_spread": e_spread, "cap_min_gap": cap_gap}
    if cap_gap <= e_spread:
        die("Kênh C KHÔNG tách được 'có nắp' và 'trạm trống' (nắp gần nhất cách C_empty {:.0f}, "
            "trạm trống dao động tới {:.0f}). Đổi nền trạm sang màu tối rồi thu lại nhãn 'empty'."
            .format(cap_gap, e_spread))
    model = {"centroids": cents.tolist(), "dmax": float(dmax), "k_sigma": k_sigma,
             "c_empty": c_empty, "c_margin": (e_spread + cap_gap) / 2.0}
    return model, stats


def confusion(true_labels, pred_labels):
    rows_present = [l for l in ALL_LABELS if l in set(true_labels)]
    M = np.zeros((len(rows_present), len(PRED_LABELS)), dtype=int)
    for t, p in zip(true_labels, pred_labels):
        M[rows_present.index(t), PRED_LABELS.index(p)] += 1
    return rows_present, M


def metrics(true_labels, pred_labels):
    out = {"n": len(true_labels), "per_label": {}}
    out["correct"] = sum(EXPECTED[t] == p for t, p in zip(true_labels, pred_labels))
    out["accuracy"] = out["correct"] / max(1, len(true_labels))
    # loi nguy hiem: nap bi bo vao hop mau khac
    out["wrong_color"] = sum(t in CLASSES and p in CLASSES and p != t for t, p in zip(true_labels, pred_labels))
    for lab in ALL_LABELS:
        ps = [p for t, p in zip(true_labels, pred_labels) if t == lab]
        if not ps:
            continue
        out["per_label"][lab] = {
            "n": len(ps),
            "correct": sum(p == EXPECTED[lab] for p in ps),
            "rejected": sum(p == "unknown" for p in ps) if lab != "other" else 0,
            "no_cap": sum(p == "no_cap" for p in ps) if lab != "empty" else 0,
            "wrong_color": sum(p in CLASSES and p != lab for p in ps),
        }
    return out


def wilson_ci(k, n, z=1.96):
    """Khoang tin cay 95% Wilson cho ti le k/n."""
    if n == 0:
        return (0.0, 1.0)
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return (max(0.0, c - h), min(1.0, c + h))


# ----------------------------------------------------------------------------- bieu do
COL = {"green": "#2e7d32", "red": "#c62828", "white": "#8a8f98", "empty": "#c0c4cc", "other": "#6a3d9a"}
EDGE = {"white": "#1b1f24", "empty": "#5b6470"}
MARK = {"green": "o", "red": "s", "white": "^", "empty": "v", "other": "D"}
INK, MUTED, GRID = "#1b1f24", "#5b6470", "#d6dae0"


def _mpl():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 10, "axes.edgecolor": MUTED, "axes.labelcolor": INK,
                         "xtick.color": MUTED, "ytick.color": MUTED, "axes.titlesize": 11})
    return plt


def _style(ax):
    ax.grid(True, color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)


def _scatter(ax, x, y, lab, **kw):
    ax.scatter(x, y, s=26, marker=MARK[lab], facecolor=COL[lab], edgecolor=EDGE.get(lab, "white"),
               linewidth=0.6, zorder=3, **kw)


def plot_chroma(model, rows, path, title):
    plt = _mpl()
    from matplotlib.patches import Circle
    fig, ax = plt.subplots(figsize=(6.6, 5.4))
    for lab in CLASSES + ("other",):
        sub = [r for r in rows if r["label"] == lab and r["sat"] == 0]
        if sub:
            X = chroma_of(sub)
            _scatter(ax, X[:, 0], X[:, 1], lab, label="{} (n={})".format(VI[lab], len(sub)))
    for lab, (cx, cy) in zip(CLASSES, model["centroids"]):
        ax.add_patch(Circle((cx, cy), model["dmax"], fill=False, linestyle="--", linewidth=1.0,
                            edgecolor=MUTED, zorder=2))
        ax.scatter([cx], [cy], marker="X", s=80, color=INK, edgecolor="white", linewidth=0.8, zorder=4)
        ax.annotate("tâm {}".format(VI[lab]), (cx, cy), xytext=(8, 8), textcoords="offset points",
                    color=INK, fontsize=9)
    ax.plot([], [], linestyle="--", color=MUTED, label="bán kính d_max = {:.4f}".format(model["dmax"]))
    ax.set_xlabel("r = R / (R+G+B)")
    ax.set_ylabel("g = G / (R+G+B)")
    ax.set_title(title, color=INK)
    ax.set_aspect("equal", adjustable="datalim")
    _style(ax)
    ax.legend(frameon=False, fontsize=9, loc="best")
    fig.tight_layout()
    fig.savefig(str(path), dpi=160)
    plt.close(fig)


def plot_c_channel(model, rows, path, title):
    plt = _mpl()
    labels = [l for l in ALL_LABELS if any(r["label"] == l for r in rows)]
    fig, ax = plt.subplots(figsize=(6.6, 4.2))
    rng = np.random.RandomState(0)
    lo, hi = model["c_empty"] - model["c_margin"], model["c_empty"] + model["c_margin"]
    ax.axhspan(max(lo, 1e-3), hi, color=GRID, alpha=0.6, zorder=1)
    for i, lab in enumerate(labels):
        c = np.array([r["C"] for r in rows if r["label"] == lab])
        _scatter(ax, i + rng.uniform(-0.18, 0.18, len(c)), c, lab)
    ax.text(len(labels) - 0.5, hi, "vùng 'không có nắp'\n|C − C_empty| < C_margin", color=MUTED,
            fontsize=8, va="bottom", ha="right")
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels([VI[l] for l in labels])
    ax.set_ylabel("Kênh C (số đếm thô, trung bình)")
    ax.set_title(title, color=INK)
    _style(ax)
    fig.tight_layout()
    fig.savefig(str(path), dpi=160)
    plt.close(fig)


def plot_confusion(row_labels, M, path, title):
    plt = _mpl()
    fig, ax = plt.subplots(figsize=(6.6, 0.55 * len(row_labels) + 2.0))
    ax.imshow(M, cmap="Blues", aspect="auto", vmin=0, vmax=max(1, M.max()))
    for i in range(M.shape[0]):
        for j in range(M.shape[1]):
            v = M[i, j]
            ax.text(j, i, str(v), ha="center", va="center", fontsize=10,
                    color="white" if v > 0.55 * M.max() else (INK if v else MUTED))
    ax.set_xticks(range(len(PRED_LABELS)))
    ax.set_xticklabels([VI[p] for p in PRED_LABELS])
    ax.set_yticks(range(len(row_labels)))
    ax.set_yticklabels([VI[l] for l in row_labels])
    ax.set_xlabel("Dự đoán")
    ax.set_ylabel("Thực tế")
    ax.set_title(title, color=INK)
    for s in ax.spines.values():
        s.set_visible(False)
    fig.tight_layout()
    fig.savefig(str(path), dpi=160)
    plt.close(fig)


# ----------------------------------------------------------------------------- lenh: ports / check
def cmd_ports(ctx):
    try:
        from serial.tools import list_ports
    except ImportError:
        die("Thiếu pyserial: pip3 install -r tools/colorcal/requirements.txt")
    for p in list_ports.comports():
        print("{:<16} {}".format(p.device, p.description))


def cmd_check(ctx):
    dev = ctx.device()
    info = dev.info()
    print("[i] Firmware: " + ", ".join("{}={}".format(k, v) for k, v in info.items()))
    if info.get("sensor") != "1":
        die("ESP32 không thấy GY-33 (0x5A). Kiểm tra dây DR=GPIO21, CT=GPIO22, VCC=3V3, GND "
            "và jumper S0-G (chế độ MCU_IIC).")
    cfg = load_config(ctx)
    m = dev.measure(cfg["navg"], cfg["settle_ms"])
    print("[i] Đo thử: R={:.0f} G={:.0f} B={:.0f} C={:.0f} sat={} (LED mức {})".format(
        m["R"], m["G"], m["B"], m["C"], m["sat"], m["led"]))
    mdl = dev.model_get()
    print("[i] Model trên ESP32: " + ("chưa có" if mdl is None else "có (nguồn: {})".format(mdl["source"])))
    print("[✓] Kết nối OK.")


# ----------------------------------------------------------------------------- lenh: tune
def cmd_tune(ctx):
    """Quet muc LED voi nap trang (vat sang nhat), chon muc LED.
    GY-33 khong cho doi thoi gian tich phan/do loi va tai lieu khong neu muc bao hoa,
    nen dau hieu bao hoa duoc suy ra tu viec C ngung tang khi LED sang hon."""
    dev = ctx.device()
    a = ctx.args
    dev.set_sim_object("white")
    ctx.ask("\n{} rồi nhấn Enter... ".format(HOWTO["white"]))
    rows = {}
    for lv in range(LED_MAX, -1, -1):                   # toi -> sang
        dev.set_led(lv)
        m = dev.measure(3, 400)
        rows[lv] = m
        print("  LED {:>2}: R={:>6.0f} G={:>6.0f} B={:>6.0f} C={:>6.0f}{}".format(
            lv, m["R"], m["G"], m["B"], m["C"], "  (bão hòa 65535)" if m["sat"] else ""))
    # muc lv bi nghi bao hoa neu da cham 65535, hoac sang hon muc lv+1 ma C tang < 2%
    suspect = {lv for lv in rows if rows[lv]["sat"] or
               (lv < LED_MAX and rows[lv]["C"] < PLATEAU_GAIN * rows[lv + 1]["C"])}
    if a.led is not None:
        led = a.led
    elif LED_DEFAULT not in suspect:
        led = LED_DEFAULT
    else:
        ok = [lv for lv in rows if lv not in suspect and lv > LED_DEFAULT]
        if not ok:
            die("Mọi mức LED đều có dấu hiệu bão hòa. Tăng khoảng cách cảm biến - nắp rồi chạy lại.")
        led = min(ok)                                   # sang nhat trong cac muc khong nghi bao hoa
        print("[!] Mức mặc định {} có dấu hiệu bão hòa -> chọn mức {}.".format(LED_DEFAULT, led))
    if suspect:
        print("[i] Mức nghi bão hòa (C tăng < {:.0%} khi sáng hơn): {}".format(PLATEAU_GAIN - 1, sorted(suspect)))
    dev.set_led(led)
    cfg = {"led": led, "navg": a.navg, "settle_ms": a.settle}
    save_json(ctx.config_path, dict(cfg, tuned_at=dt.datetime.now().isoformat(timespec="seconds"),
                                    scan_white={str(k): v for k, v in rows.items()}, simulated=ctx.sim))
    print("[✓] Cấu hình: {} -> {}".format(cfg_str(cfg), ctx.config_path))
    return cfg


# ----------------------------------------------------------------------------- lenh: collect
def _append_row(path, row):
    new = not path.exists()
    with open(str(path), "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        if new:
            w.writeheader()
        w.writerow(row)


def _pop_last_row(path):
    lines = path.read_text(encoding="utf-8").splitlines(True)
    if len(lines) <= 1:
        return False
    path.write_text("".join(lines[:-1]), encoding="utf-8")
    return True


def cmd_collect(ctx, set_name=None, labels=None, n=None):
    a = ctx.args
    set_name = set_name or a.set
    labels = labels or [l.strip() for l in a.labels.split(",") if l.strip()]
    n = n or a.n
    for l in labels:
        if l not in ALL_LABELS:
            die("Nhãn '{}' không hợp lệ. Dùng: {}".format(l, ", ".join(ALL_LABELS)))
    if not ctx.config_path.exists():
        print("[!] Chưa chạy 'tune' -> dùng cấu hình mặc định.")
    cfg = load_config(ctx)
    dev = ctx.device()
    dev.set_led(cfg["led"])
    path = ctx.csv_path(set_name)
    every = getattr(a, "every", 0.0) or 0.0
    print("\n=== THU DỮ LIỆU '{}' -> {} ===\nCấu hình: {}".format(set_name, path, cfg_str(cfg)))
    print("Mỗi mẫu: nhấc nắp ra rồi đặt lại (xoay một góc khác) để dữ liệu chứa sai lệch vị trí thật.")
    for lab in labels:
        dev.set_sim_object(lab)
        if ctx.ask("\n--- '{}' ({}): {} mẫu. {}. Enter = bắt đầu, s = bỏ qua: ".format(
                lab, VI[lab], n, HOWTO[lab])).strip().lower() == "s":
            continue
        k, n_sat = 0, 0
        while k < n:
            if every > 0 and not ctx.yes:
                for t in range(int(math.ceil(every)), 0, -1):
                    print("\r  [{} {}/{}] đặt lại nắp... {} s ".format(lab, k + 1, n, t), end="", flush=True)
                    time.sleep(1)
                print()
            else:
                r = ctx.ask("  [{} {}/{}] Đặt lại rồi Enter (u = xoá mẫu trước, s = nhãn sau, q = thoát): "
                            .format(lab, k + 1, n)).strip().lower()
                if r == "q":
                    print("[i] Dừng. Dữ liệu đã lưu tới mẫu trước.")
                    return
                if r == "s":
                    break
                if r == "u":
                    if k > 0 and _pop_last_row(path):
                        k -= 1
                        print("  [i] Đã xoá mẫu vừa lấy.")
                    continue
            m = dev.measure(cfg["navg"], cfg["settle_ms"])
            if m["sat"]:
                n_sat += 1
                print("  [!] Bão hòa (65535) - mẫu bị bỏ.")
                if n_sat >= 3:
                    die("Bão hòa liên tục: chạy lại 'tune' (hoặc --led lớn hơn = tối hơn).")
                continue
            n_sat = 0
            row = {"time": dt.datetime.now().isoformat(timespec="seconds"), "set": set_name, "label": lab,
                   "R": "{:.3f}".format(m["R"]), "G": "{:.3f}".format(m["G"]), "B": "{:.3f}".format(m["B"]),
                   "C": "{:.3f}".format(m["C"]), "sat": m["sat"]}
            row.update(cfg)
            _append_row(path, row)
            k += 1
            s = m["R"] + m["G"] + m["B"]
            print("  [✓] {:>2}/{}  C={:>7.0f}  r={:.4f}  g={:.4f}".format(
                k, n, m["C"], m["R"] / s if s else 0, m["G"] / s if s else 0))
    print("[✓] Xong. {} có {} mẫu.".format(path.name, len(read_rows(path))))


# ----------------------------------------------------------------------------- lenh: train
def cmd_train(ctx):
    a = ctx.args
    path = ctx.csv_path("train")
    rows = [r for r in read_rows(path) if r["sat"] == 0 and r["label"] in CLASSES + ("empty",)]
    if not rows:
        die("Chưa có dữ liệu {} - chạy 'collect --set train'.".format(path))
    for lab in CLASSES + ("empty",):
        n = sum(r["label"] == lab for r in rows)
        if n < MIN_SAMPLES:
            die("Nhãn '{}' chỉ có {} mẫu (cần >= {}; khuyến nghị 30).".format(lab, n, MIN_SAMPLES))
    cfg = check_cfg_consistency(rows, "train.csv")
    model, st = fit_model(rows, a.k_sigma)
    model.update(cfg)
    model.update({"version": 2, "sensor": "GY-33 MCU_IIC", "classes": list(CLASSES),
                  "created": dt.datetime.now().isoformat(timespec="seconds"),
                  "simulated": ctx.sim, "train_stats": st})

    # Leave-one-out (uoc luong lac quan; so lieu chinh thuc la tap test)
    truth, pred = [], []
    for i, r in enumerate(rows):
        others = rows[:i] + rows[i + 1:]
        m2 = dict(model)
        if r["label"] in CLASSES:
            cents = [list(c) for c in model["centroids"]]
            cents[CLASSES.index(r["label"])] = chroma_of(
                [o for o in others if o["label"] == r["label"]]).mean(axis=0).tolist()
            m2["centroids"] = cents
        else:
            m2["c_empty"] = float(np.mean([o["C"] for o in others if o["label"] == "empty"]))
        truth.append(r["label"])
        pred.append(classify_one(m2, r["R"], r["G"], r["B"], r["C"], r["sat"])["label"])
    labs, M = confusion(truth, pred)
    model["loo"] = {"rows": labs, "matrix": M.tolist(), "metrics": metrics(truth, pred)}
    save_json(ctx.model_path, model)
    write_header(ctx, model)

    plot_chroma(model, rows, ctx.reports / "train_chroma.png", "Không gian sắc độ (r, g) - tập huấn luyện")
    plot_c_channel(model, rows, ctx.reports / "train_c_channel.png", "Kênh C theo nhãn - tập huấn luyện")
    plot_confusion(labs, M, ctx.reports / "train_loo_confusion.png", "Ma trận nhầm lẫn leave-one-out (huấn luyện)")

    print("\n=== KẾT QUẢ HUẤN LUYỆN ===\nCấu hình: " + cfg_str(cfg))
    for lab, c in zip(CLASSES, model["centroids"]):
        s = st["classes"][lab]
        print("  {:<6} n={:<3} tâm=({:.4f}, {:.4f})  d̄={:.4f}  σ_d={:.4f}  d_lớn_nhất={:.4f}".format(
            lab, s["n"], c[0], c[1], s["d_mean"], s["d_std"], s["d_max"]))
        if s["n"] >= 10 and s["c_rel_std"] < 0.002:
            print("  [!] Kênh C của '{}' gần như không đổi giữa các lần đặt nắp - có thể bão hòa; "
                  "thử 'tune' hoặc --led lớn hơn.".format(lab))
    print("  d_max = {:.4f} (K = {:g})   khoảng cách tâm nhỏ nhất = {:.4f}   d_max/(D_min/2) = {:.2f}".format(
        model["dmax"], a.k_sigma, st["dmin"], st["sep_ratio"]))
    if st["sep_ratio"] >= 1.0:
        print("  [!] Vùng chấp nhận của các màu chồng lấn (tỉ số >= 1). Kiểm tra lại nắp/nguồn sáng.")
    print("  C_empty = {:.1f}, C_margin = {:.1f}".format(model["c_empty"], model["c_margin"]))
    lm = model["loo"]["metrics"]
    print("  Leave-one-out: đúng {}/{} ({:.1%}), sai màu {}".format(lm["correct"], lm["n"], lm["accuracy"],
                                                                 lm["wrong_color"]))
    print("[✓] Lưu {} và {}".format(ctx.model_path, ctx.header_path()))
    write_report(ctx)
    return model


def write_header(ctx, m):
    path = ctx.header_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    cents = ", ".join("{{{:.7f}f, {:.7f}f}}".format(c[0], c[1]) for c in m["centroids"])
    path.write_text(
        "// Sinh tu dong boi tools/colorcal/colorcal.py train luc {created}. KHONG sua tay.\n"
        "// Thu tu tam lop: GREEN, RED, WHITE. {sim}\n"
        "#pragma once\n"
        "#define COLOR_MODEL_DEFINED 1\n"
        "#define CM_LED {led}\n"
        "#define CM_NAVG {navg}\n"
        "#define CM_SETTLE_MS {settle}\n"
        "#define CM_CENT {{{cents}}}\n"
        "#define CM_DMAX {dmax:.7f}f\n"
        "#define CM_C_EMPTY {ce:.3f}f\n"
        "#define CM_C_MARGIN {cm:.3f}f\n".format(
            created=m["created"], sim="(MO PHONG - khong dung cho phan cung)" if m.get("simulated") else "",
            led=m["led"], navg=m["navg"], settle=m["settle_ms"], cents=cents,
            dmax=m["dmax"], ce=m["c_empty"], cm=m["c_margin"]), encoding="utf-8")


# ----------------------------------------------------------------------------- lenh: push
def cmd_push(ctx):
    if not ctx.model_path.exists():
        die("Chưa có model - chạy 'train'.")
    m = load_json(ctx.model_path)
    if m.get("simulated") and not ctx.sim:
        die("model.json là model MÔ PHỎNG - không nạp vào ESP32 thật.")
    dev = ctx.device()
    dev.model_set(m)
    back = dev.model_get()
    bad = [(i, r, g) for i, (r, g) in enumerate(zip(model_values(m), model_values(back)))
           if abs(r - g) > 1e-5 * max(1.0, abs(r))]
    if bad:
        die("Đọc lại model khác giá trị đã gửi: {}".format(bad))
    print("[✓] Đã nạp model vào NVS của ESP32 và đọc lại khớp.")


# ----------------------------------------------------------------------------- lenh: evaluate
def cmd_evaluate(ctx):
    a = ctx.args
    if not ctx.model_path.exists():
        die("Chưa có model - chạy 'train'.")
    model = load_json(ctx.model_path)
    path = ctx.csv_path(a.set)
    rows = read_rows(path)
    if not rows:
        die("Chưa có dữ liệu {} - chạy 'collect --set {}'.".format(path, a.set))
    cfg = check_cfg_consistency(rows, path.name)
    if any(cfg[k] != model[k] for k in CFG_KEYS):
        die("Cấu hình đo của {} ({}) khác model ({}).".format(path.name, cfg_str(cfg), cfg_str(model)))

    preds = [classify_one(model, r["R"], r["G"], r["B"], r["C"], r["sat"]) for r in rows]
    truth, plabs = [r["label"] for r in rows], [p["label"] for p in preds]
    labs, M = confusion(truth, plabs)
    met = metrics(truth, plabs)
    res = {"set": a.set, "created": dt.datetime.now().isoformat(timespec="seconds"),
           "rows": labs, "matrix": M.tolist(), "metrics": met,
           "accuracy_ci95": wilson_ci(met["correct"], met["n"]), "simulated": ctx.sim}

    if a.device:
        dev = ctx.device()
        mism = []
        for r, p in zip(rows, preds):
            d = dev.classify_raw(r["R"], r["G"], r["B"], r["C"], r["sat"])
            if d["label"] != p["label"]:
                mism.append({"row": r, "python": p["label"], "esp32": d["label"]})
        res["parity"] = {"checked": len(rows), "mismatch": len(mism), "examples": mism[:10]}

    save_json(ctx.reports / "eval_{}.json".format(a.set), res)
    plot_confusion(labs, M, ctx.reports / "{}_confusion.png".format(a.set), "Ma trận nhầm lẫn - tập {}".format(a.set))
    plot_chroma(model, rows, ctx.reports / "{}_chroma.png".format(a.set), "Không gian sắc độ (r, g) - tập {}".format(a.set))

    print("\n=== ĐÁNH GIÁ TẬP '{}' ({} mẫu) ===".format(a.set, met["n"]))
    w = 16
    print("{:<18}".format("thực tế \\ dự đoán") + "".join("{:>{w}}".format(VI[p], w=w) for p in PRED_LABELS))
    for lab, row in zip(labs, M):
        print("{:<18}".format(VI[lab]) + "".join("{:>{w}}".format(v, w=w) for v in row))
    lo, hi = res["accuracy_ci95"]
    print("Đúng {}/{} = {:.1%} (KTC 95% Wilson: {:.1%} - {:.1%}); nắp bị phân SAI MÀU: {}".format(
        met["correct"], met["n"], met["accuracy"], lo, hi, met["wrong_color"]))
    if "parity" in res:
        print("Tương đương ESP32 <-> Python: {} khác biệt / {} mẫu".format(
            res["parity"]["mismatch"], res["parity"]["checked"]))
    write_report(ctx)
    return res


# ----------------------------------------------------------------------------- lenh: live
def cmd_live(ctx):
    dev = ctx.device()
    if dev.model_get() is None:
        die("ESP32 chưa có model - chạy 'push'.")
    print("Phân loại liên tục (Ctrl+C để dừng)...")
    try:
        while True:
            r = dev.classify()
            print("  {:<15} gần nhất={:<6} d={:.4f}  r={:.4f} g={:.4f}  C={:.0f}{}".format(
                VI[r["label"]], r["nearest"], r["dist"], r["r"], r["g"], r["C"],
                "   -> khu vực VỨT BỎ" if r["label"] == "unknown" else ""))
            if ctx.sim:
                break
            time.sleep(ctx.args.period)
    except KeyboardInterrupt:
        print()


# ----------------------------------------------------------------------------- bao cao
def _md_table(header, rows):
    out = ["| " + " | ".join(header) + " |", "|" + "|".join("---" for _ in header) + "|"]
    return "\n".join(out + ["| " + " | ".join(str(c) for c in r) + " |" for r in rows])


def write_report(ctx):
    if not ctx.model_path.exists():
        return
    m = load_json(ctx.model_path)
    st = m["train_stats"]
    L = ["# Báo cáo hiệu chuẩn phân loại màu nắp chai (GY-33)\n"]
    if m.get("simulated"):
        L.append("> **CẢNH BÁO: dữ liệu MÔ PHỎNG (--sim). Không dùng số liệu này trong báo cáo.**\n")
    L.append("Tạo lúc {} bởi `tools/colorcal/colorcal.py`.\n".format(dt.datetime.now().isoformat(timespec="seconds")))
    L.append("## 1. Mô hình (tập huấn luyện)\n\nCấu hình đo: {}\n".format(cfg_str(m)))
    rows = [[VI[lab], st["classes"][lab]["n"], "{:.4f}".format(c[0]), "{:.4f}".format(c[1]),
             "{:.4f}".format(st["classes"][lab]["d_mean"]), "{:.4f}".format(st["classes"][lab]["d_std"]),
             "{:.4f}".format(st["classes"][lab]["d_max"])] for lab, c in zip(CLASSES, m["centroids"])]
    L.append(_md_table(["Lớp", "n", "r̄", "ḡ", "d̄", "σ_d", "d lớn nhất"], rows) + "\n")
    L.append("- d_max = max_k(d̄_k + {:g}·σ_d,k) = **{:.4f}**; d_max / (D_min/2) = {:.2f} ({})".format(
        m["k_sigma"], m["dmax"], st["sep_ratio"], "không chồng lấn" if st["sep_ratio"] < 1 else "CHỒNG LẤN"))
    c = st["c"]
    L.append("- Kênh C: C_empty = {:.1f}, trạm trống dao động ±{:.1f}, nắp gần nhất cách {:.1f} → C_margin = {:.1f}"
             .format(c["empty_mean"], c["empty_spread"], c["cap_min_gap"], m["c_margin"]))
    lm = m["loo"]["metrics"]
    L.append("- Leave-one-out: đúng {}/{} ({:.1%}), sai màu {}.\n".format(
        lm["correct"], lm["n"], lm["accuracy"], lm["wrong_color"]))
    L.append("![sắc độ](train_chroma.png)\n\n![kênh C](train_c_channel.png)\n")

    ev_path = ctx.reports / "eval_test.json"
    ev = load_json(ev_path) if ev_path.exists() else None
    if ev:
        met = ev["metrics"]
        lo, hi = ev["accuracy_ci95"]
        L.append("## 2. Đánh giá trên tập kiểm tra độc lập\n")
        L.append(_md_table(["Thực tế \\ Dự đoán"] + [VI[p] for p in PRED_LABELS],
                           [[VI[l]] + r for l, r in zip(ev["rows"], ev["matrix"])]) + "\n")
        L.append("- Đúng **{}/{} = {:.1%}** (KTC 95% Wilson: {:.1%} – {:.1%}); nắp bị phân **sai màu**: {}".format(
            met["correct"], met["n"], met["accuracy"], lo, hi, met["wrong_color"]))
        if "parity" in ev:
            L.append("- ESP32 và Python cho cùng nhãn ở {}/{} mẫu.".format(
                ev["parity"]["checked"] - ev["parity"]["mismatch"], ev["parity"]["checked"]))
        L.append("\n![ma trận nhầm lẫn](test_confusion.png)\n")

    n = st["classes"]
    para = ("Cảm biến GY-33 hoạt động ở chế độ MCU_IIC; ESP32 đọc các giá trị RGBC thô 16 bit (thanh ghi "
            "0x00–0x07) với LED của module ở mức {led}. Mỗi phép đo chờ {settle} ms sau khi nắp được đặt, rồi lấy "
            "trung bình {navg} lần đọc cách nhau {iv} ms. Giá trị R, G, B được chuẩn hóa thành tọa độ sắc độ "
            "r = R/(R+G+B), g = G/(R+G+B). Tâm của ba lớp được tính từ {ng}, {nr} và {nw} mẫu huấn luyện (xanh lá, "
            "đỏ, trắng); nắp được gán vào lớp có tâm gần nhất theo khoảng cách Euclid. Ngưỡng loại bỏ "
            "d_max = {dmax:.4f} được xác định bằng max_k(d̄_k + {k:g}σ_k); nắp có khoảng cách lớn hơn d_max được đưa "
            "vào khu vực vứt bỏ và báo lỗi qua Serial. Trạm được xem là không có nắp khi |C − {ce:.0f}| < {cm:.0f}.").format(
        led=m["led"], settle=m["settle_ms"], navg=m["navg"], iv=READ_INTERVAL_MS, ng=n["green"]["n"],
        nr=n["red"]["n"], nw=n["white"]["n"], dmax=m["dmax"], k=m["k_sigma"], ce=m["c_empty"], cm=m["c_margin"])
    if ev:
        met = ev["metrics"]
        lo, hi = ev["accuracy_ci95"]
        para += (" Trên tập kiểm tra độc lập gồm {n} mẫu, hệ thống phân loại đúng {c} mẫu ({acc:.1%}, khoảng tin "
                 "cậy 95% Wilson {lo:.1%} – {hi:.1%}), với {w} trường hợp nắp bị phân sai màu.").format(
            n=met["n"], c=met["correct"], acc=met["accuracy"], lo=lo, hi=hi, w=met["wrong_color"])
    L.append("## {}. Đoạn mô tả gợi ý cho báo cáo\n\n{}\n".format(3 if ev else 2, para))
    (ctx.reports / "report.md").write_text("\n".join(L), encoding="utf-8")
    print("[✓] Cập nhật {}".format(ctx.reports / "report.md"))


def cmd_report(ctx):
    if not ctx.model_path.exists():
        die("Chưa có model - chạy 'train'.")
    write_report(ctx)


# ----------------------------------------------------------------------------- lenh: wizard
def _existing(ctx, path, what):
    """'keep' | 'append' | 'overwrite' cho file du lieu da co."""
    if not path.exists():
        return "overwrite"
    r = ctx.ask("\n{} đã có {} mẫu. [k] dùng luôn / [a] thu thêm / [o] xoá và thu lại (mặc định k): "
                .format(what, len(read_rows(path))), default="k").strip().lower()
    if r == "o":
        path.unlink()
        return "overwrite"
    return "append" if r == "a" else "keep"


def cmd_wizard(ctx):
    a = ctx.args
    print("=" * 70 + "\nQUY TRÌNH HIỆU CHUẨN PHÂN LOẠI MÀU NẮP (7 bước)\n" + "=" * 70)
    print("\n[1/7] Kiểm tra kết nối")
    cmd_check(ctx)

    print("\n[2/7] Chọn mức LED (quét với nắp trắng)")
    redo = True
    if ctx.config_path.exists():
        redo = ctx.ask("Đã có {} ({}). [k] dùng luôn / [t] quét lại (mặc định k): ".format(
            ctx.config_path.name, cfg_str(load_config(ctx))), default="k").strip().lower() == "t"
    if redo:
        old = load_config(ctx) if ctx.config_path.exists() else None
        new = cmd_tune(ctx)
        if old and old != new:
            for s in ("train", "test"):
                p = ctx.csv_path(s)
                if p.exists():
                    p.rename(p.with_suffix(".csv.old"))
                    print("[!] Cấu hình đổi -> {} đổi tên thành .old".format(p.name))

    print("\n[3/7] Thu dữ liệu HUẤN LUYỆN")
    if _existing(ctx, ctx.csv_path("train"), "train.csv") != "keep":
        cmd_collect(ctx, "train", list(CLASSES) + ["empty"], a.n_train)

    print("\n[4/7] Huấn luyện")
    cmd_train(ctx)

    print("\n[5/7] Nạp model vào ESP32")
    cmd_push(ctx)

    print("\n[6/7] Thu dữ liệu KIỂM TRA (độc lập, thu SAU khi huấn luyện)")
    if _existing(ctx, ctx.csv_path("test"), "test.csv") != "keep":
        labels = list(CLASSES) + ["empty"]
        if ctx.ask("Thêm nhãn 'other' (vật lạ) để kiểm tra chức năng vứt bỏ? [Y/n]: ",
                   default="y").strip().lower() != "n":
            labels.append("other")
        cmd_collect(ctx, "test", labels, a.n_test)

    print("\n[7/7] Đánh giá + kiểm tra tương đương firmware")
    a.set, a.device = "test", True
    cmd_evaluate(ctx)
    print("\n[✓] HOÀN TẤT. Kết quả: {}\n    Model đã nằm trong NVS của ESP32; header: {}".format(
        ctx.reports / "report.md", ctx.header_path()))


# ----------------------------------------------------------------------------- main
def build_parser():
    p = argparse.ArgumentParser(description="Hiệu chuẩn & đánh giá phân loại màu nắp chai (GY-33 + ESP32).")
    p.add_argument("--port", help="cổng serial, vd. /dev/ttyUSB0 (mặc định: tự dò nếu chỉ có 1 cổng)")
    p.add_argument("--ws", help="thư mục làm việc chứa data/color và reports/color (mặc định: gốc repo)")
    p.add_argument("--sim", action="store_true", help="dùng thiết bị MÔ PHỎNG (chạy thử phần mềm)")
    p.add_argument("--yes", action="store_true", help="không hỏi (dùng cho --sim)")
    p.add_argument("-v", "--verbose", action="store_true", help="in mọi dòng giao tiếp serial")
    sp = p.add_subparsers(dest="cmd")
    sp.required = True
    sp.add_parser("ports")
    sp.add_parser("check")

    def tune_opts(q):
        q.add_argument("--led", type=int, choices=range(LED_MAX + 1), metavar="0-10",
                       help="ép mức LED thay vì tự chọn (số nhỏ = sáng hơn)")
        q.add_argument("--navg", type=int, default=DEFAULT_CFG["navg"])
        q.add_argument("--settle", type=int, default=DEFAULT_CFG["settle_ms"], help="chờ (ms) trước lần đọc đầu")

    tune_opts(sp.add_parser("tune"))
    c = sp.add_parser("collect")
    c.add_argument("--set", default="train", help="train | test")
    c.add_argument("--labels", default="green,red,white,empty", help="vd. green,red,white,empty,other")
    c.add_argument("--n", type=int, default=30, help="số mẫu mỗi nhãn")
    c.add_argument("--every", type=float, default=0.0, help="tự lấy mẫu mỗi N giây thay vì nhấn Enter")
    tr = sp.add_parser("train")
    tr.add_argument("--k-sigma", type=float, default=DEFAULT_K_SIGMA, help="K trong d_max = max(d̄ + Kσ)")
    sp.add_parser("push")
    e = sp.add_parser("evaluate")
    e.add_argument("--set", default="test")
    e.add_argument("--device", action="store_true", help="phân loại lại từng mẫu trên ESP32 để so với Python")
    sp.add_parser("live").add_argument("--period", type=float, default=0.3)
    sp.add_parser("report")
    w = sp.add_parser("wizard")
    w.add_argument("--n-train", type=int, default=30)
    w.add_argument("--n-test", type=int, default=20)
    w.add_argument("--k-sigma", type=float, default=DEFAULT_K_SIGMA)
    w.add_argument("--every", type=float, default=0.0)
    tune_opts(w)
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    ctx = Ctx(args)
    cmds = {"ports": cmd_ports, "check": cmd_check, "tune": cmd_tune, "collect": cmd_collect,
            "train": cmd_train, "push": cmd_push, "evaluate": cmd_evaluate, "live": cmd_live,
            "report": cmd_report, "wizard": cmd_wizard}
    try:
        cmds[args.cmd](ctx)
    except DeviceError as e:
        die("Lỗi thiết bị: {}".format(e))
    except KeyboardInterrupt:
        print("\n[i] Đã dừng.")
    finally:
        ctx.close()


if __name__ == "__main__":
    main()
