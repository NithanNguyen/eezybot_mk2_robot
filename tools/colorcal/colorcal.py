#!/usr/bin/env python3
"""
colorcal.py - Tu dong hoa quy trinh hieu chuan va danh gia phan loai mau nap chai
(TCS34725 + ESP32 NodeMCU-32S, firmware env "robot" cua repo nay).

Phuong phap: chuan hoa sac do r = R/(R+G+B), g = G/(R+G+B); phan loai theo tam lop
gan nhat (Euclid); loai bo khi khoang cach > d_max; khong co nap khi
|C - C_empty| < C_margin.

Lenh (chay tu thu muc goc cua repo):
  python3 tools/colorcal/colorcal.py wizard            # chay TOAN BO quy trinh theo thu tu (khuyen dung)
  python3 tools/colorcal/colorcal.py ports             # liet ke cong serial
  python3 tools/colorcal/colorcal.py check             # kiem tra ket noi + cam bien
  python3 tools/colorcal/colorcal.py tune              # tu chon thoi gian tich phan + do loi
  python3 tools/colorcal/colorcal.py collect --set train --n 30
  python3 tools/colorcal/colorcal.py train             # tam lop, d_max, nguong C, color_model.h, bieu do
  python3 tools/colorcal/colorcal.py push              # nap model vao NVS cua ESP32
  python3 tools/colorcal/colorcal.py collect --set test --n 20 --labels green,red,yellow,empty,other
  python3 tools/colorcal/colorcal.py evaluate --device # ma tran nham lan + kiem tra tuong duong firmware
  python3 tools/colorcal/colorcal.py live              # phan loai lien tuc tren thiet bi
  python3 tools/colorcal/colorcal.py report            # tao lai reports/report.md

Tuy chon chung: --port /dev/ttyUSB0, --ws <thu muc lam viec> (mac dinh: goc repo -> data/color, reports/color),
  --sim (thiet bi MO PHONG, chi de chay thu phan mem - so lieu KHONG phai do that),
  --yes (khong hoi; dung cho --sim).
Yeu cau: Python >= 3.8, numpy, matplotlib, pyserial (xem tools/colorcal/requirements.txt).
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

# ----------------------------------------------------------------------------- hang so
CLASSES = ("green", "red", "yellow")               # thu tu = thu tu tam lop trong firmware
CAP_LABELS = CLASSES
ALL_LABELS = CLASSES + ("empty", "other")         # "other": vat la, ky vong bi loai bo
PRED_LABELS = ("green", "red", "yellow", "unknown", "no_cap")
EXPECTED = {"green": "green", "red": "red", "yellow": "yellow", "empty": "no_cap", "other": "unknown"}
FW2PRED = {"GREEN": "green", "RED": "red", "YELLOW": "yellow", "UNKNOWN": "unknown", "NO_CAP": "no_cap"}
VI = {"green": "xanh lá", "red": "đỏ", "yellow": "vàng", "empty": "trạm trống",
      "other": "vật lạ", "unknown": "không xác định", "no_cap": "không có nắp"}
HOWTO = {
    "green": "Đặt nắp XANH LÁ vào trạm",
    "red": "Đặt nắp ĐỎ vào trạm",
    "yellow": "Đặt nắp VÀNG vào trạm",
    "empty": "Để trạm TRỐNG (không có nắp)",
    "other": "Đặt một vật KHÔNG thuộc 3 màu (vd. nắp xanh dương/trắng) - để kiểm tra chức năng loại bỏ",
}
GAIN_X = (1, 4, 16, 60)
DEFAULT_CFG = {"atime": 0xEB, "gain": 1, "navg": 5, "nflush": 3}   # 50.4 ms, 4x
TUNE_ATIMES = (0xF6, 0xEB, 0xD6, 0xC0)            # 24 / 50.4 / 100.8 / 153.6 ms (Adafruit_TCS34725.h)
TUNE_HEADROOM = 0.60       # C lon nhat <= 60% muc bao hoa (de du cho sai lech vi tri/LED)
DEFAULT_K_SIGMA = 3.0      # d_max = max_k(mean_k + K*std_k)
MIN_SAMPLES = 5
CSV_FIELDS = ["time", "set", "label", "R", "G", "B", "C", "sat", "atime", "gain", "navg", "nflush"]

ROOT = Path(__file__).resolve().parents[2]          # goc repo
FW_HEADER = ROOT / "include" / "color_model.h"


def it_ms(atime):
    return (256 - int(atime)) * 2.4


def saturation_level(atime):
    """Giong colormath::saturationLevel() (theo logic DN40 trong thu vien Adafruit)."""
    cycles = 256 - int(atime)
    if cycles > 63:
        return 65535
    s = 1024 * cycles
    return s - s // 4


def cfg_str(cfg):
    return "ATIME=0x{:02X} ({:.1f} ms), gain {}x, navg={}, nflush={}".format(
        cfg["atime"], it_ms(cfg["atime"]), GAIN_X[cfg["gain"]], cfg["navg"], cfg["nflush"])


# ----------------------------------------------------------------------------- tien ich
class Ctx:
    def __init__(self, args):
        self.args = args
        self.ws = Path(args.ws).resolve() if args.ws else ROOT
        self.data = self.ws / "data" / "color"
        self.reports = self.ws / "reports" / "color"
        self.data.mkdir(parents=True, exist_ok=True)
        self.reports.mkdir(parents=True, exist_ok=True)
        self.yes = bool(getattr(args, "yes", False))
        self.sim = bool(getattr(args, "sim", False))
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
        # Model mo phong khong duoc ghi de vao firmware that.
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


def load_config(ctx, required=False):
    if ctx.config_path.exists():
        cfg = load_json(ctx.config_path)
        return {k: int(cfg[k]) for k in ("atime", "gain", "navg", "nflush")}
    if required:
        die("Chưa có {} - chạy 'tune' trước.".format(ctx.config_path))
    print("[!] Chưa có data/color/config.json (chưa chạy 'tune') -> dùng cấu hình mặc định: "
          + cfg_str(DEFAULT_CFG))
    return dict(DEFAULT_CFG)


# ----------------------------------------------------------------------------- thiet bi
class DeviceError(RuntimeError):
    pass


class Device:
    """Giao tiep voi firmware (that hoac mo phong) theo giao thuc dong lenh trong src/robot/color_console.h."""

    def __init__(self, port=None, baud=115200, sim=False, verbose=False):
        self.verbose = verbose
        self.sim = sim
        if sim:
            self.ser = SimSerial()
            print("[i] Dùng THIẾT BỊ MÔ PHỎNG (--sim). Số liệu là giả lập, chỉ để chạy thử phần mềm.")
        else:
            self.ser = self._open_serial(port, baud)
        self._handshake()

    # -- mo cong: giong lab1/tools/hw1_logger.py: DTR/RTS = False truoc khi open()
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
        ser.dtr = False
        ser.rts = False
        try:
            ser.exclusive = True
        except (AttributeError, ValueError):
            pass
        try:
            ser.open()
        except serial.SerialException as e:
            die("Không mở được {}: {}\n    -> Đóng Serial Monitor của PlatformIO / chương trình khác "
                "đang dùng cổng; kiểm tra quyền (sudo usermod -aG dialout $USER rồi đăng xuất).".format(port, e))
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
        die("ESP32 không trả lời PING. Kiểm tra: đã nạp firmware chưa (pio run -e robot -t upload), đúng cổng, "
            "baud 115200; thử nhấn nút EN.")

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def _readline(self):
        raw = self.ser.readline()
        if not raw:
            return None
        return raw.decode("utf-8", errors="replace").strip()

    def cmd(self, line, prefix, timeout=5.0):
        if self.verbose:
            print("  >> " + line)
        self.ser.write((line + "\n").encode())
        deadline = time.time() + timeout
        while time.time() < deadline:
            resp = self._readline()
            if resp is None:
                if self.sim:
                    break
                continue
            if self.verbose:
                print("  << " + resp)
            if not resp:
                continue
            if resp.startswith("#"):
                if "ERR" in resp:
                    print("  [thiết bị] " + resp[1:].strip())
                continue
            if resp.startswith("ERR"):
                raise DeviceError(resp)
            if resp.startswith(prefix):
                return resp
        raise DeviceError("Hết thời gian chờ phản hồi '{}' cho lệnh '{}'".format(prefix, line))

    # -- lenh cap cao
    def info(self):
        resp = self.cmd("INFO", "INFO")
        out = {}
        for part in resp.split(",")[1:]:
            if "=" in part:
                k, v = part.split("=", 1)
                out[k] = v
        return out

    def set_cfg(self, atime, gain):
        resp = self.cmd("CFG {} {}".format(int(atime), int(gain)), "OK CFG")
        f = resp.split(",")
        return {"atime": int(f[1]), "gain": int(f[2]), "sat": int(f[3])}

    def measure(self, navg, nflush, atime):
        timeout = (navg + nflush + 2) * it_ms(atime) / 1000.0 + 3.0
        f = self.cmd("MEAS {} {}".format(int(navg), int(nflush)), "MEAS,", timeout=timeout).split(",")
        return {"R": float(f[1]), "G": float(f[2]), "B": float(f[3]), "C": float(f[4]),
                "sat": int(f[5]), "atime": int(f[6]), "gain": int(f[7])}

    @staticmethod
    def _parse_cls(resp):
        f = resp.split(",")
        return {"label": FW2PRED.get(f[1], f[1].lower()), "nearest": FW2PRED.get(f[2], f[2].lower()),
                "dist": float(f[3]), "r": float(f[4]), "g": float(f[5]),
                "R": float(f[6]), "G": float(f[7]), "B": float(f[8]), "C": float(f[9]), "sat": int(f[10])}

    def classify(self, timeout=10.0):
        return self._parse_cls(self.cmd("CLS", "CLS,", timeout=timeout))

    def classify_raw(self, R, G, B, C, sat):
        line = "CLSRAW {:.3f} {:.3f} {:.3f} {:.3f} {}".format(R, G, B, C, int(sat))
        return self._parse_cls(self.cmd(line, "CLS,"))

    def model_get(self):
        f = self.cmd("MODEL GET", "MODEL,").split(",")
        if f[1] == "none":
            return None
        v = [float(x) for x in f[2:]]
        return {"source": f[1], "atime": int(v[0]), "gain": int(v[1]), "navg": int(v[2]),
                "nflush": int(v[3]), "centroids": [[v[4], v[5]], [v[6], v[7]], [v[8], v[9]]],
                "dmax": v[10], "c_empty": v[11], "c_margin": v[12]}

    def model_set(self, m):
        vals = [m["atime"], m["gain"], m["navg"], m["nflush"]]
        vals += [x for c in m["centroids"] for x in c]
        vals += [m["dmax"], m["c_empty"], m["c_margin"]]
        line = "MODEL SET " + " ".join(
            str(int(x)) if i < 4 else "{:.8g}".format(x) for i, x in enumerate(vals))
        self.cmd(line, "OK MODEL", timeout=5.0)

    def set_sim_object(self, label):
        if self.sim:
            self.ser.obj = label


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
    """Mo phong firmware de chay thu toan bo quy trinh khi chua co phan cung.
    Mo hinh vat ly don gian, cac he so la GIA DINH (khong phai do that)."""

    # dem / ms o do loi 1x (R, G, B) - gia dinh
    RATES = {"green": (18.0, 38.0, 26.0), "red": (52.0, 14.0, 12.0), "yellow": (60.0, 52.0, 18.0),
             "empty": (6.0, 6.0, 6.0), "other": (12.0, 20.0, 48.0)}

    def __init__(self, seed=1234):
        self.rng = np.random.RandomState(seed)
        self.atime, self.gain = DEFAULT_CFG["atime"], DEFAULT_CFG["gain"]
        self.model = None
        self.obj = "empty"
        self.out = []

    def reset_input_buffer(self):
        self.out = []

    def close(self):
        pass

    def readline(self):
        return (self.out.pop(0) + "\n").encode() if self.out else b""

    def _emit(self, s):
        self.out.append(s)

    def _read_once(self, place, chroma):
        k = GAIN_X[self.gain] * it_ms(self.atime)
        rates = np.array(self.RATES[self.obj]) * chroma * place
        rgb = rates * k * (1.0 + self.rng.normal(0, 0.004, 3))
        c = rgb.sum() * 1.03 * (1.0 + self.rng.normal(0, 0.004))
        hw_max = min(65535, 1024 * (256 - self.atime))
        vals = np.clip(np.r_[rgb, c], 0, hw_max).astype(int)
        return vals

    def _measure(self, navg):
        place = self.rng.normal(1.0, 0.06)              # sai lech vi tri / do cao nap
        chroma = 1.0 + self.rng.normal(0, 0.015, 3)     # sai lech mau giua cac lan dat
        sat_lvl = saturation_level(self.atime)
        acc = np.zeros(4)
        sat = 0
        for _ in range(navg):
            v = self._read_once(place, chroma)
            acc += v
            sat |= int(v[3] >= sat_lvl)
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
        c = t[0].upper()
        if c == "PING":
            self._emit("OK PONG")
        elif c == "INFO":
            self._emit("INFO,fw=sim,sensor=1,atime={},gain={},sat={},it_ms={:.1f},model={}".format(
                self.atime, self.gain, saturation_level(self.atime), it_ms(self.atime),
                "nvs" if self.model else "none"))
        elif c == "CFG":
            self.atime, self.gain = int(t[1], 0), int(t[2], 0)
            self._emit("OK CFG,{},{},{},{:.1f}".format(self.atime, self.gain,
                                                         saturation_level(self.atime), it_ms(self.atime)))
        elif c == "MEAS":
            R, G, B, C, sat = self._measure(int(t[1]))
            self._emit("MEAS,{:.3f},{:.3f},{:.3f},{:.3f},{},{},{}".format(R, G, B, C, sat, self.atime, self.gain))
        elif c == "CLS":
            if not self.model:
                self._emit("ERR NO_MODEL")
                return
            self.atime, self.gain = self.model["atime"], self.model["gain"]
            self._emit(self._cls_line(*self._measure(self.model["navg"])))
        elif c == "CLSRAW":
            if not self.model:
                self._emit("ERR NO_MODEL")
                return
            self._emit(self._cls_line(float(t[1]), float(t[2]), float(t[3]), float(t[4]), int(t[5])))
        elif c == "MODEL":
            sub = t[1].upper()
            if sub == "GET":
                if not self.model:
                    self._emit("MODEL,none")
                else:
                    m = self.model
                    vals = [m["atime"], m["gain"], m["navg"], m["nflush"]] + \
                        [x for cc in m["centroids"] for x in cc] + [m["dmax"], m["c_empty"], m["c_margin"]]
                    self._emit("MODEL,nvs," + ",".join(str(v) for v in vals))
            elif sub == "SET":
                v = t[2:]
                self.model = {"atime": int(v[0]), "gain": int(v[1]), "navg": int(v[2]), "nflush": int(v[3]),
                              "centroids": [[float(v[4]), float(v[5])], [float(v[6]), float(v[7])],
                                            [float(v[8]), float(v[9])]],
                              "dmax": float(v[10]), "c_empty": float(v[11]), "c_margin": float(v[12])}
                self.atime, self.gain = self.model["atime"], self.model["gain"]
                self._emit("OK MODEL,nvs")
            elif sub == "CLEAR":
                self.model = None
                self._emit("OK MODEL,cleared")
        elif c == "LED":
            self._emit("OK LED " + t[1])
        else:
            self._emit("ERR UNKNOWN_CMD")


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
    with open(str(path), newline="", encoding="utf-8") as f:
        rows = []
        for r in csv.DictReader(f):
            for k in ("R", "G", "B", "C"):
                r[k] = float(r[k])
            for k in ("sat", "atime", "gain", "navg", "nflush"):
                r[k] = int(r[k])
            rows.append(r)
        return rows


def chroma_of(rows):
    R = np.array([r["R"] for r in rows])
    G = np.array([r["G"] for r in rows])
    B = np.array([r["B"] for r in rows])
    s = R + G + B
    return np.c_[R / s, G / s]


def check_cfg_consistency(rows, what):
    keys = {(r["atime"], r["gain"], r["navg"], r["nflush"]) for r in rows}
    if len(keys) != 1:
        die("{} có nhiều cấu hình đo khác nhau {} - mọi mẫu phải dùng cùng ATIME/gain/navg/nflush. "
            "Xoá file và thu lại sau khi 'tune'.".format(what, sorted(keys)))
    a, g, n, f = keys.pop()
    return {"atime": a, "gain": g, "navg": n, "nflush": f}


def fit_model(rows, k_sigma):
    """Tinh tam lop, d_max, nguong kenh C tu tap huan luyen. Tra ve (model, thong ke)."""
    stats = {"classes": {}}
    cents = []
    for lab in CLASSES:
        sub = [r for r in rows if r["label"] == lab]
        X = chroma_of(sub)
        c = X.mean(axis=0)
        d = np.linalg.norm(X - c, axis=1)
        cents.append(c)
        stats["classes"][lab] = {"n": len(sub), "centroid": c.tolist(), "std_rg": X.std(axis=0, ddof=1).tolist(),
                                 "d_mean": float(d.mean()), "d_std": float(d.std(ddof=1)), "d_max": float(d.max())}
    dmax = max(s["d_mean"] + k_sigma * s["d_std"] for s in stats["classes"].values())
    cents = np.array(cents)
    pair = {}
    for i in range(3):
        for j in range(i + 1, 3):
            pair["{}-{}".format(CLASSES[i], CLASSES[j])] = float(np.linalg.norm(cents[i] - cents[j]))
    dmin = min(pair.values())
    stats["centroid_dist"] = pair
    stats["dmin"] = dmin
    stats["sep_ratio"] = dmax / (dmin / 2.0)     # < 1: cac vung chap nhan khong chong lan

    Ce = np.array([r["C"] for r in rows if r["label"] == "empty"])
    Cc = np.array([r["C"] for r in rows if r["label"] in CAP_LABELS])
    c_empty = float(Ce.mean())
    e_spread = float(np.abs(Ce - c_empty).max())
    cap_gap = float(np.abs(Cc - c_empty).min())
    stats["c"] = {"empty_mean": c_empty, "empty_spread": e_spread, "cap_min_gap": cap_gap,
                  "empty_std": float(Ce.std(ddof=1)) if len(Ce) > 1 else 0.0}
    if cap_gap <= e_spread:
        die("Kênh C KHÔNG tách được 'có nắp' và 'trạm trống' (nắp gần nhất cách C_empty {:.0f}, "
            "trạm trống dao động tới {:.0f}). Hãy đổi nền trạm sang màu tối/khác hẳn nắp rồi thu lại "
            "nhãn 'empty'.".format(cap_gap, e_spread))
    c_margin = (e_spread + cap_gap) / 2.0
    model = {"centroids": cents.tolist(), "dmax": float(dmax), "k_sigma": k_sigma,
             "c_empty": c_empty, "c_margin": float(c_margin)}
    return model, stats


def confusion(true_labels, pred_labels):
    rows_present = [l for l in ALL_LABELS if l in set(true_labels)]
    M = np.zeros((len(rows_present), len(PRED_LABELS)), dtype=int)
    for t, p in zip(true_labels, pred_labels):
        M[rows_present.index(t), PRED_LABELS.index(p)] += 1
    return rows_present, M


def metrics(true_labels, pred_labels):
    out = {"n": len(true_labels), "per_label": {}}
    correct = 0
    wrong_color = 0
    for t, p in zip(true_labels, pred_labels):
        correct += int(EXPECTED[t] == p)
        wrong_color += int(t in CAP_LABELS and p in CAP_LABELS and p != t)
    out["accuracy"] = correct / max(1, len(true_labels))
    out["correct"] = correct
    out["wrong_color"] = wrong_color     # loi nguy hiem: nap bi bo vao hop mau khac
    for lab in ALL_LABELS:
        idx = [i for i, t in enumerate(true_labels) if t == lab]
        if not idx:
            continue
        ps = [pred_labels[i] for i in idx]
        out["per_label"][lab] = {
            "n": len(idx),
            "correct": sum(p == EXPECTED[lab] for p in ps),
            "rejected": sum(p == "unknown" for p in ps) if lab != "other" else 0,
            "no_cap": sum(p == "no_cap" for p in ps) if lab != "empty" else 0,
            "wrong_color": sum(p in CAP_LABELS and p != lab for p in ps) if lab in CAP_LABELS else
            sum(p in CAP_LABELS for p in ps),
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
COL = {"green": "#2e7d32", "red": "#c62828", "yellow": "#d4a106", "empty": "#7d8590", "other": "#6a3d9a"}
MARK = {"green": "o", "red": "s", "yellow": "^", "empty": "v", "other": "D"}
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


def plot_chroma(model, rows, path, title):
    plt = _mpl()
    from matplotlib.patches import Circle
    fig, ax = plt.subplots(figsize=(6.6, 5.4))
    for lab in CLASSES + ("other",):
        sub = [r for r in rows if r["label"] == lab and r["sat"] == 0]
        if not sub:
            continue
        X = chroma_of(sub)
        ax.scatter(X[:, 0], X[:, 1], s=26, marker=MARK[lab], facecolor=COL[lab], edgecolor="white",
                   linewidth=0.6, label="{} (n={})".format(VI[lab], len(sub)), zorder=3)
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
        ax.scatter(i + rng.uniform(-0.18, 0.18, len(c)), c, s=18, marker=MARK[lab], facecolor=COL[lab],
                   edgecolor="white", linewidth=0.5, zorder=3)
    ax.text(len(labels) - 0.5, hi, "  vùng 'không có nắp'\n  |C − C_empty| < C_margin", color=MUTED,
            fontsize=8, va="bottom", ha="right")
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels([VI[l] for l in labels])
    allc = np.array([r["C"] for r in rows])
    if allc.min() > 0 and allc.max() / allc.min() > 20:
        ax.set_yscale("log")
    ax.set_ylabel("Kênh C (số đếm, trung bình navg lần đọc)")
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
    ports = list(list_ports.comports())
    if not ports:
        print("Không thấy cổng serial nào.")
    for p in ports:
        print("{:<16} {}".format(p.device, p.description))


def cmd_check(ctx):
    dev = ctx.device()
    info = dev.info()
    print("[i] Firmware: " + ", ".join("{}={}".format(k, v) for k, v in info.items()))
    if info.get("sensor") != "1":
        die("ESP32 không thấy TCS34725 (0x29). Kiểm tra dây SDA=GPIO21, SCL=GPIO22, VIN=3V3, GND.")
    cfg = load_config(ctx)
    dev.set_cfg(cfg["atime"], cfg["gain"])
    m = dev.measure(cfg["navg"], cfg["nflush"], cfg["atime"])
    print("[i] Đo thử ({}): R={:.0f} G={:.0f} B={:.0f} C={:.0f} sat={}".format(
        cfg_str(cfg), m["R"], m["G"], m["B"], m["C"], m["sat"]))
    mdl = dev.model_get()
    print("[i] Model trên ESP32: " + ("chưa có" if mdl is None else "có (nguồn: {})".format(mdl["source"])))
    print("[✓] Kết nối OK.")


# ----------------------------------------------------------------------------- lenh: tune
def cmd_tune(ctx):
    """Chon (ATIME, gain) cho so dem lon nhat ma nap sang nhat van <= 60% muc bao hoa."""
    dev = ctx.device()
    a = ctx.args
    cands = [(at, g) for at in TUNE_ATIMES for g in range(4)]
    table = {c: {} for c in cands}
    print("\n=== TỰ CHỌN THỜI GIAN TÍCH PHÂN VÀ ĐỘ LỢI ===")
    print("Lần lượt đặt từng màu nắp; mỗi màu đo {} cấu hình (~5-15 s).".format(len(cands)))
    for lab in CLASSES:
        dev.set_sim_object(lab)
        ctx.ask("\n{} rồi nhấn Enter... ".format(HOWTO[lab]))
        for at, g in cands:
            dev.set_cfg(at, g)
            m = dev.measure(3, 2, at)
            table[(at, g)][lab] = (m["C"], m["sat"])
        print("  [✓] đã đo {}".format(VI[lab]))

    feasible = []
    print("\n{:>10} {:>5} {:>9}  {}".format("IT (ms)", "gain", "bão hòa", "C xanh / đỏ / vàng"))
    for (at, g) in cands:
        sat_lvl = saturation_level(at)
        cs = [table[(at, g)][l][0] for l in CLASSES]
        any_sat = any(table[(at, g)][l][1] for l in CLASSES)
        ok = (not any_sat) and max(cs) <= TUNE_HEADROOM * sat_lvl
        print("{:>10.1f} {:>4}x {:>9} {} {}".format(it_ms(at), GAIN_X[g], sat_lvl,
                                                  " / ".join("{:>6.0f}".format(c) for c in cs),
                                                  "" if ok else "  (loại: vượt {:.0%} bão hòa)".format(TUNE_HEADROOM)))
        if ok:
            feasible.append((min(cs), -it_ms(at), at, g))
    # Uu tien so dem cua nap toi nhat lon nhat (do phan giai tot nhat); bang nhau thi chon thoi gian ngan.
    best = max(feasible)[2:] if feasible else None
    if best is None:
        die("Không có cấu hình nào dưới {:.0%} mức bão hòa - LED quá sáng/nắp quá gần. "
            "Tăng khoảng cách cảm biến - nắp rồi chạy lại.".format(TUNE_HEADROOM))
    cfg = {"atime": best[0], "gain": best[1], "navg": a.navg, "nflush": a.nflush}
    save_json(ctx.config_path, dict(cfg, tuned_at=dt.datetime.now().isoformat(timespec="seconds"),
                                    headroom=TUNE_HEADROOM, rule="max min(C) s.t. max(C) <= headroom*sat",
                                    simulated=ctx.sim))
    dev.set_cfg(cfg["atime"], cfg["gain"])
    print("\n[✓] Chọn: {}  -> lưu {}".format(cfg_str(cfg), ctx.config_path))
    print("    Thời gian một phép đo ≈ {:.2f} s".format((cfg["navg"] + cfg["nflush"]) * it_ms(cfg["atime"]) / 1000))
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
    with open(str(path), encoding="utf-8") as f:
        lines = f.readlines()
    if len(lines) <= 1:
        return False
    with open(str(path), "w", encoding="utf-8") as f:
        f.writelines(lines[:-1])
    return True


def cmd_collect(ctx, set_name=None, labels=None, n=None):
    a = ctx.args
    set_name = set_name or a.set
    labels = labels or [l.strip() for l in a.labels.split(",") if l.strip()]
    n = n or a.n
    for l in labels:
        if l not in ALL_LABELS:
            die("Nhãn '{}' không hợp lệ. Dùng: {}".format(l, ", ".join(ALL_LABELS)))
    cfg = load_config(ctx)
    dev = ctx.device()
    dev.set_cfg(cfg["atime"], cfg["gain"])
    path = ctx.csv_path(set_name)
    every = getattr(a, "every", 0.0) or 0.0
    print("\n=== THU DỮ LIỆU '{}' -> {} ===\nCấu hình: {}".format(set_name, path, cfg_str(cfg)))
    print("Mỗi mẫu: nhấc nắp ra rồi đặt lại (xoay một góc khác) để dữ liệu chứa sai lệch vị trí thật.")
    for lab in labels:
        dev.set_sim_object(lab)
        resp = ctx.ask("\n--- Nhãn '{}' ({}): cần {} mẫu. {}. Enter = bắt đầu, s = bỏ qua: ".format(
            lab, VI[lab], n, HOWTO[lab]))
        if resp.strip().lower() == "s":
            continue
        k, n_sat = 0, 0
        while k < n:
            if every > 0 and not ctx.yes:
                for t in range(int(math.ceil(every)), 0, -1):
                    print("\r  [{} {}/{}] đặt lại nắp... {} s ".format(lab, k + 1, n, t), end="", flush=True)
                    time.sleep(1)
                print()
            else:
                r = ctx.ask("  [{} {}/{}] Đặt lại rồi Enter (u = xoá mẫu trước, s = sang nhãn sau, q = thoát): "
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
            m = dev.measure(cfg["navg"], cfg["nflush"], cfg["atime"])
            if m["sat"]:
                n_sat += 1
                print("  [!] Bão hòa - mẫu bị bỏ.")
                if n_sat >= 3:
                    die("Bão hòa liên tục: chạy lại 'tune' (cảm biến quá sáng ở cấu hình hiện tại).")
                continue
            n_sat = 0
            row = {"time": dt.datetime.now().isoformat(timespec="seconds"), "set": set_name, "label": lab,
                   "R": "{:.3f}".format(m["R"]), "G": "{:.3f}".format(m["G"]), "B": "{:.3f}".format(m["B"]),
                   "C": "{:.3f}".format(m["C"]), "sat": m["sat"], "atime": cfg["atime"], "gain": cfg["gain"],
                   "navg": cfg["navg"], "nflush": cfg["nflush"]}
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
    rows = [r for r in read_rows(path) if r["sat"] == 0]
    if not rows:
        die("Chưa có dữ liệu {} - chạy 'collect --set train'.".format(path))
    rows = [r for r in rows if r["label"] in CLASSES + ("empty",)]
    for lab in CLASSES + ("empty",):
        n = sum(r["label"] == lab for r in rows)
        if n < MIN_SAMPLES:
            die("Nhãn '{}' chỉ có {} mẫu (cần >= {}; khuyến nghị 30).".format(lab, n, MIN_SAMPLES))
    cfg = check_cfg_consistency(rows, "train.csv")
    model, st = fit_model(rows, a.k_sigma)
    model.update(cfg)
    model.update({"version": 1, "classes": list(CLASSES),
                  "created": dt.datetime.now().isoformat(timespec="seconds"),
                  "simulated": ctx.sim, "train_stats": st})

    # Leave-one-out tren tap huan luyen (uoc luong lac quan; so lieu chinh thuc la tap test)
    truth, pred = [], []
    for i, r in enumerate(rows):
        others = rows[:i] + rows[i + 1:]
        m2 = dict(model)
        if r["label"] in CLASSES:
            cents = [list(c) for c in model["centroids"]]
            k = CLASSES.index(r["label"])
            cents[k] = chroma_of([o for o in others if o["label"] == r["label"]]).mean(axis=0).tolist()
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
        print("  {:<7} n={:<3} tâm=({:.4f}, {:.4f})  d̄={:.4f}  σ_d={:.4f}  d_max_lớp={:.4f}".format(
            lab, s["n"], c[0], c[1], s["d_mean"], s["d_std"], s["d_max"]))
    print("  d_max = {:.4f} (K = {:g})   khoảng cách tâm nhỏ nhất = {:.4f}   tỉ số d_max/(D_min/2) = {:.2f}".format(
        model["dmax"], a.k_sigma, st["dmin"], st["sep_ratio"]))
    if st["sep_ratio"] >= 1.0:
        print("  [!] Các vùng chấp nhận chồng lấn (tỉ số >= 1): hai màu quá gần nhau trong (r, g). "
              "Kiểm tra lại nắp/nguồn sáng; có thể giảm K.")
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
    txt = (
        "// Sinh tu dong boi tools/colorcal/colorcal.py train luc {created}. KHONG sua tay.\n"
        "// Thu tu tam lop: GREEN, RED, YELLOW. {sim}\n"
        "#pragma once\n"
        "#define COLOR_MODEL_DEFINED 1\n"
        "#define CM_ATIME 0x{atime:02X}\n"
        "#define CM_GAIN {gain}\n"
        "#define CM_NAVG {navg}\n"
        "#define CM_NFLUSH {nflush}\n"
        "#define CM_CENT {{{cents}}}\n"
        "#define CM_DMAX {dmax:.7f}f\n"
        "#define CM_C_EMPTY {ce:.3f}f\n"
        "#define CM_C_MARGIN {cm:.3f}f\n"
    ).format(created=m["created"], sim="(MO PHONG - khong dung cho phan cung)" if m.get("simulated") else "",
             atime=m["atime"], gain=m["gain"], navg=m["navg"], nflush=m["nflush"], cents=cents,
             dmax=m["dmax"], ce=m["c_empty"], cm=m["c_margin"])
    path.write_text(txt, encoding="utf-8")


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
    ref = [m["atime"], m["gain"], m["navg"], m["nflush"]] + [x for c in m["centroids"] for x in c] + \
          [m["dmax"], m["c_empty"], m["c_margin"]]
    got = [back["atime"], back["gain"], back["navg"], back["nflush"]] + \
          [x for c in back["centroids"] for x in c] + [back["dmax"], back["c_empty"], back["c_margin"]]
    bad = [(i, r, g) for i, (r, g) in enumerate(zip(ref, got)) if abs(r - g) > 1e-5 * max(1.0, abs(r))]
    if bad:
        die("Đọc lại model khác giá trị đã gửi: {}".format(bad))
    print("[✓] Đã nạp model vào NVS của ESP32 và đọc lại khớp (nguồn: {}).".format(back["source"]))


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
    if any(cfg[k] != model[k] for k in cfg):
        die("Cấu hình đo của {} ({}) khác model ({}).".format(path.name, cfg_str(cfg), cfg_str(model)))

    preds = [classify_one(model, r["R"], r["G"], r["B"], r["C"], r["sat"]) for r in rows]
    truth = [r["label"] for r in rows]
    plabs = [p["label"] for p in preds]
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
                mism.append({"row": r, "python": p["label"], "esp32": d["label"],
                             "dist_py": p["dist"], "dist_esp": d["dist"]})
        res["parity"] = {"checked": len(rows), "mismatch": len(mism), "examples": mism[:10]}

    save_json(ctx.reports / "eval_{}.json".format(a.set), res)
    plot_confusion(labs, M, ctx.reports / "{}_confusion.png".format(a.set),
                   "Ma trận nhầm lẫn - tập {}".format(a.set))
    plot_chroma(model, rows, ctx.reports / "{}_chroma.png".format(a.set),
                "Không gian sắc độ (r, g) - tập {}".format(a.set))

    print("\n=== ĐÁNH GIÁ TẬP '{}' ({} mẫu) ===".format(a.set, met["n"]))
    print(format_confusion_text(labs, M))
    lo, hi = res["accuracy_ci95"]
    print("Đúng {}/{} = {:.1%} (KTC 95% Wilson: {:.1%} - {:.1%}); nắp bị phân SAI MÀU: {}".format(
        met["correct"], met["n"], met["accuracy"], lo, hi, met["wrong_color"]))
    if "parity" in res:
        pr = res["parity"]
        print("Tương đương ESP32 <-> Python: {} khác biệt / {} mẫu".format(pr["mismatch"], pr["checked"]))
        for e in pr["examples"]:
            print("   khác: python={} esp32={} dist={:.6f}/{:.6f}".format(e["python"], e["esp32"],
                                                                         e["dist_py"], e["dist_esp"]))
    write_report(ctx)
    return res


def format_confusion_text(labs, M):
    w = 16
    head = "{:<{w}}".format("thực tế \\ dự đoán", w=18) + "".join("{:>{w}}".format(VI[p], w=w) for p in PRED_LABELS)
    lines = [head]
    for lab, row in zip(labs, M):
        lines.append("{:<18}".format(VI[lab]) + "".join("{:>{w}}".format(v, w=w) for v in row))
    return "\n".join(lines)


# ----------------------------------------------------------------------------- lenh: live
def cmd_live(ctx):
    dev = ctx.device()
    if dev.model_get() is None:
        die("ESP32 chưa có model - chạy 'push'.")
    print("Phân loại liên tục (Ctrl+C để dừng)...")
    try:
        while True:
            r = dev.classify()
            msg = "  {:<15} gần nhất={:<7} d={:.4f}  r={:.4f} g={:.4f}  C={:.0f}".format(
                VI[r["label"]], r["nearest"], r["dist"], r["r"], r["g"], r["C"])
            if r["label"] == "unknown":
                msg += "   -> khu vực VỨT BỎ"
            print(msg)
            if ctx.sim:
                break
            time.sleep(ctx.args.period)
    except KeyboardInterrupt:
        print()


# ----------------------------------------------------------------------------- bao cao
def _md_table(header, rows):
    out = ["| " + " | ".join(header) + " |", "|" + "|".join("---" for _ in header) + "|"]
    out += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    return "\n".join(out)


def write_report(ctx):
    if not ctx.model_path.exists():
        return
    m = load_json(ctx.model_path)
    st = m["train_stats"]
    L = []
    L.append("# Báo cáo hiệu chuẩn phân loại màu nắp chai (TCS34725)\n")
    if m.get("simulated"):
        L.append("> **CẢNH BÁO: dữ liệu MÔ PHỎNG (--sim). Không dùng số liệu này trong báo cáo.**\n")
    L.append("Tạo lúc {}. Sinh tự động bởi `tools/colorcal/colorcal.py`.\n".format(dt.datetime.now().isoformat(timespec="seconds")))
    L.append("## 1. Cấu hình đo\n")
    L.append("- {}\n- Mức bão hòa kênh C: {}\n- Thời gian một phép đo ≈ {:.2f} s\n".format(
        cfg_str(m), saturation_level(m["atime"]), (m["navg"] + m["nflush"]) * it_ms(m["atime"]) / 1000))
    L.append("## 2. Mô hình (tập huấn luyện)\n")
    rows = []
    for lab, c in zip(CLASSES, m["centroids"]):
        s = st["classes"][lab]
        rows.append([VI[lab], s["n"], "{:.4f}".format(c[0]), "{:.4f}".format(c[1]), "{:.4f}".format(s["d_mean"]),
                     "{:.4f}".format(s["d_std"]), "{:.4f}".format(s["d_max"])])
    L.append(_md_table(["Lớp", "n", "r̄", "ḡ", "d̄ (tới tâm)", "σ_d", "d lớn nhất"], rows) + "\n")
    L.append("- d_max = max_k(d̄_k + {:g}·σ_d,k) = **{:.4f}**".format(m["k_sigma"], m["dmax"]))
    L.append("- Khoảng cách giữa các tâm: " + ", ".join("{} = {:.4f}".format(k, v) for k, v in st["centroid_dist"].items()))
    L.append("- Tỉ số d_max / (D_min/2) = {:.2f} ({})".format(
        st["sep_ratio"], "vùng chấp nhận không chồng lấn" if st["sep_ratio"] < 1 else "CHỒNG LẤN - cần xem lại"))
    c = st["c"]
    L.append("- Kênh C: C_empty = {:.1f}, trạm trống dao động tối đa ±{:.1f}, nắp gần nhất cách {:.1f} "
             "→ C_margin = {:.1f}\n".format(c["empty_mean"], c["empty_spread"], c["cap_min_gap"], m["c_margin"]))
    lm = m["loo"]["metrics"]
    L.append("Leave-one-out trên tập huấn luyện: đúng {}/{} ({:.1%}), sai màu {}.\n".format(
        lm["correct"], lm["n"], lm["accuracy"], lm["wrong_color"]))
    L.append("![sắc độ](train_chroma.png)\n\n![kênh C](train_c_channel.png)\n")

    ev_path = ctx.reports / "eval_test.json"
    ev = load_json(ev_path) if ev_path.exists() else None
    if ev:
        met = ev["metrics"]
        L.append("## 3. Đánh giá trên tập kiểm tra độc lập\n")
        L.append(_md_table(["Thực tế \\ Dự đoán"] + [VI[p] for p in PRED_LABELS],
                           [[VI[l]] + r for l, r in zip(ev["rows"], ev["matrix"])]) + "\n")
        lo, hi = ev["accuracy_ci95"]
        L.append("- Đúng **{}/{} = {:.1%}** (khoảng tin cậy 95% Wilson: {:.1%} – {:.1%})".format(
            met["correct"], met["n"], met["accuracy"], lo, hi))
        L.append("- Nắp bị phân **sai màu** (vào hộp khác): {}".format(met["wrong_color"]))
        prow = []
        for lab, s in met["per_label"].items():
            prow.append([VI[lab], s["n"], s["correct"], s["rejected"], s["no_cap"], s["wrong_color"]])
        L.append("\n" + _md_table(["Nhãn", "n", "đúng", "bị loại (vứt bỏ)", "nhầm 'không nắp'", "sai màu"], prow))
        if "parity" in ev:
            pr = ev["parity"]
            L.append("\n- Kiểm tra tương đương firmware: ESP32 và Python cho cùng nhãn ở {}/{} mẫu.".format(
                pr["checked"] - pr["mismatch"], pr["checked"]))
        L.append("\n![ma trận nhầm lẫn](test_confusion.png)\n\n![sắc độ test](test_chroma.png)\n")

    L.append("## {}. Đoạn mô tả gợi ý cho báo cáo\n".format(4 if ev else 3))
    n = st["classes"]
    para = ("Mỗi phép đo lấy trung bình {navg} lần đọc RGBC sau khi bỏ {nflush} lần đọc đầu kể từ lúc bật LED, "
            "với thời gian tích phân {it:.1f} ms và độ lợi {g}×. Các giá trị R, G, B được chuẩn hóa thành tọa độ "
            "sắc độ r = R/(R+G+B), g = G/(R+G+B). Tâm của ba lớp được tính từ {ng}, {nr} và {ny} mẫu huấn luyện "
            "(xanh lá, đỏ, vàng); nắp được gán vào lớp có tâm gần nhất theo khoảng cách Euclid. Ngưỡng loại bỏ "
            "d_max = {dmax:.4f} được xác định bằng max_k(d̄_k + {k:g}σ_k) trên khoảng cách của mẫu huấn luyện tới "
            "tâm lớp của chính nó; mẫu có khoảng cách lớn hơn d_max được đưa vào khu vực vứt bỏ và báo lỗi qua "
            "Serial. Trạm được xem là không có nắp khi |C − {ce:.0f}| < {cm:.0f}.").format(
        navg=m["navg"], nflush=m["nflush"], it=it_ms(m["atime"]), g=GAIN_X[m["gain"]], ng=n["green"]["n"],
        nr=n["red"]["n"], ny=n["yellow"]["n"], dmax=m["dmax"], k=m["k_sigma"], ce=m["c_empty"], cm=m["c_margin"])
    if ev:
        met = ev["metrics"]
        lo, hi = ev["accuracy_ci95"]
        para += (" Trên tập kiểm tra độc lập gồm {n} mẫu, hệ thống phân loại đúng {c} mẫu ({acc:.1%}, khoảng tin "
                 "cậy 95% Wilson {lo:.1%} – {hi:.1%}), với {w} trường hợp nắp bị phân sai màu.").format(
            n=met["n"], c=met["correct"], acc=met["accuracy"], lo=lo, hi=hi, w=met["wrong_color"])
    L.append(para + "\n")
    (ctx.reports / "report.md").write_text("\n".join(L), encoding="utf-8")
    print("[✓] Cập nhật {}".format(ctx.reports / "report.md"))


def cmd_report(ctx):
    if not ctx.model_path.exists():
        die("Chưa có model - chạy 'train'.")
    write_report(ctx)


# ----------------------------------------------------------------------------- lenh: wizard
def _existing(ctx, path, what):
    """Tra ve 'keep' | 'overwrite' | 'append' cho file du lieu da co."""
    if not path.exists():
        return "overwrite"
    n = len(read_rows(path))
    r = ctx.ask("\n{} đã có {} mẫu. [k] dùng luôn / [a] thu thêm / [o] xoá và thu lại (mặc định k): "
                .format(what, n), default="k").strip().lower()
    if r == "o":
        path.unlink()
        return "overwrite"
    return "append" if r == "a" else "keep"


def cmd_wizard(ctx):
    a = ctx.args
    print("=" * 70 + "\nQUY TRÌNH HIỆU CHUẨN PHÂN LOẠI MÀU NẮP (7 bước)\n" + "=" * 70)
    print("\n[1/7] Kiểm tra kết nối")
    cmd_check(ctx)

    print("\n[2/7] Chọn thời gian tích phân + độ lợi")
    if ctx.config_path.exists():
        r = ctx.ask("Đã có {} ({}). [k] dùng luôn / [t] tune lại (mặc định k): ".format(
            ctx.config_path.name, cfg_str(load_config(ctx))), default="k").strip().lower()
        if r == "t":
            if ctx.csv_path("train").exists() or ctx.csv_path("test").exists():
                print("[!] Đổi cấu hình -> dữ liệu cũ không dùng được nữa (sẽ bị đổi tên .old).")
                for s in ("train", "test"):
                    p = ctx.csv_path(s)
                    if p.exists():
                        p.rename(p.with_suffix(".csv.old"))
            cmd_tune(ctx)
    else:
        cmd_tune(ctx)

    print("\n[3/7] Thu dữ liệu HUẤN LUYỆN")
    mode = _existing(ctx, ctx.csv_path("train"), "train.csv")
    if mode != "keep":
        cmd_collect(ctx, "train", list(CLASSES) + ["empty"], a.n_train)

    print("\n[4/7] Huấn luyện")
    cmd_train(ctx)

    print("\n[5/7] Nạp model vào ESP32")
    cmd_push(ctx)

    print("\n[6/7] Thu dữ liệu KIỂM TRA (độc lập, thu SAU khi huấn luyện)")
    mode = _existing(ctx, ctx.csv_path("test"), "test.csv")
    if mode != "keep":
        labels = list(CLASSES) + ["empty"]
        r = ctx.ask("Có muốn thêm nhãn 'other' (vật lạ) để kiểm tra chức năng vứt bỏ? [Y/n]: ", default="y")
        if r.strip().lower() != "n":
            labels.append("other")
        cmd_collect(ctx, "test", labels, a.n_test)

    print("\n[7/7] Đánh giá + kiểm tra tương đương firmware")
    a.set, a.device = "test", True
    cmd_evaluate(ctx)
    print("\n[✓] HOÀN TẤT. Kết quả: {}\n    Model đã nằm trong NVS của ESP32; header: {}".format(
        ctx.reports / "report.md", ctx.header_path()))


# ----------------------------------------------------------------------------- main
def build_parser():
    p = argparse.ArgumentParser(description="Hiệu chuẩn & đánh giá phân loại màu nắp chai (TCS34725 + ESP32).")
    p.add_argument("--port", help="cổng serial, vd. /dev/ttyUSB0 (mặc định: tự dò nếu chỉ có 1 cổng)")
    p.add_argument("--ws", help="thư mục làm việc chứa data/color và reports/color (mặc định: gốc repo)")
    p.add_argument("--sim", action="store_true", help="dùng thiết bị MÔ PHỎNG (chạy thử phần mềm)")
    p.add_argument("--yes", action="store_true", help="không hỏi (dùng cho --sim)")
    p.add_argument("-v", "--verbose", action="store_true", help="in mọi dòng giao tiếp serial")
    sp = p.add_subparsers(dest="cmd")
    sp.required = True
    sp.add_parser("ports")
    sp.add_parser("check")
    t = sp.add_parser("tune")
    t.add_argument("--navg", type=int, default=DEFAULT_CFG["navg"])
    t.add_argument("--nflush", type=int, default=DEFAULT_CFG["nflush"])
    c = sp.add_parser("collect")
    c.add_argument("--set", default="train", help="train | test (tên file CSV trong data/)")
    c.add_argument("--labels", default="green,red,yellow,empty", help="vd. green,red,yellow,empty,other")
    c.add_argument("--n", type=int, default=30, help="số mẫu mỗi nhãn")
    c.add_argument("--every", type=float, default=0.0,
                   help="tự lấy mẫu mỗi N giây (đếm ngược để đặt lại nắp) thay vì nhấn Enter")
    tr = sp.add_parser("train")
    tr.add_argument("--k-sigma", type=float, default=DEFAULT_K_SIGMA, help="K trong d_max = max(d̄ + Kσ)")
    sp.add_parser("push")
    e = sp.add_parser("evaluate")
    e.add_argument("--set", default="test")
    e.add_argument("--device", action="store_true", help="kiểm tra tương đương: phân loại lại từng mẫu trên ESP32")
    lv = sp.add_parser("live")
    lv.add_argument("--period", type=float, default=0.3)
    sp.add_parser("report")
    w = sp.add_parser("wizard")
    w.add_argument("--n-train", type=int, default=30)
    w.add_argument("--n-test", type=int, default=20)
    w.add_argument("--navg", type=int, default=DEFAULT_CFG["navg"])
    w.add_argument("--nflush", type=int, default=DEFAULT_CFG["nflush"])
    w.add_argument("--k-sigma", type=float, default=DEFAULT_K_SIGMA)
    w.add_argument("--every", type=float, default=0.0)
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
