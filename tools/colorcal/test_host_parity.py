#!/usr/bin/env python3
"""
Kiem tra tuong duong thuat toan C++ (lib/ColorClassifier/ColorMath.h) va Python
(colorcal.classify_one) tren PC, KHONG can ESP32. Can g++.

  python3 tools/colorcal/test_host_parity.py                       # dung data/color/model.json + train.csv, test.csv
  python3 tools/colorcal/test_host_parity.py --ws <thu muc> --random 20000
"""
import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import colorcal as cc  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
SRC = Path(__file__).resolve().parent / "host_parity.cpp"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ws", default=str(ROOT))
    ap.add_argument("--random", type=int, default=5000, help="so mau ngau nhien them (gom ca sat bien d_max)")
    a = ap.parse_args()
    ws = Path(a.ws)
    model = cc.load_json(ws / "data" / "color" / "model.json")
    rows = []
    for s in ("train", "test"):
        rows += [(r["R"], r["G"], r["B"], r["C"], r["sat"]) for r in cc.read_rows(ws / "data" / "color" / "{}.csv".format(s))]

    # Mau ngau nhien: quanh moi tam lop o khoang cach ~ d_max (truong hop kho nhat) + C quanh nguong.
    rng = np.random.RandomState(7)
    for _ in range(a.random):
        cx, cy = model["centroids"][rng.randint(3)]
        ang, rad = rng.uniform(0, 2 * np.pi), model["dmax"] * rng.uniform(0.9, 1.1)
        r, g = cx + rad * np.cos(ang), cy + rad * np.sin(ang)
        S = rng.uniform(500, 60000)
        C = model["c_empty"] + rng.choice([-1, 1]) * model["c_margin"] * rng.uniform(0.9, 1.1) \
            if rng.rand() < 0.2 else S * 1.03
        rows.append((round(r * S, 3), round(g * S, 3), round((1 - r - g) * S, 3), round(C, 3), int(rng.rand() < 0.02)))

    with tempfile.TemporaryDirectory() as td:
        exe = Path(td) / "host_parity"
        subprocess.check_call(["g++", "-std=c++11", "-O2", "-Wall", "-o", str(exe), str(SRC)])
        # Gui model theo dung dinh dang colorcal gui cho ESP32 (MODEL SET)
        stdin = " ".join(cc.model_values_str(model)) + "\n"
        stdin += "\n".join("{:.3f} {:.3f} {:.3f} {:.3f} {}".format(*r) for r in rows) + "\n"
        out = subprocess.run([str(exe)], input=stdin.encode(), stdout=subprocess.PIPE, check=True).stdout.decode()

    m32 = dict(model)
    m32["centroids"] = [[float("{:.8g}".format(x)) for x in c] for c in model["centroids"]]
    for k in ("dmax", "c_empty", "c_margin"):
        m32[k] = float("{:.8g}".format(model[k]))
    bad = 0
    for line, r in zip(out.strip().splitlines(), rows):
        lab_cpp, d_cpp = line.split()
        p = cc.classify_one(m32, *r)
        if cc.FW2PRED[lab_cpp] != p["label"]:
            bad += 1
            if bad <= 10:
                print("KHAC: row={} cpp={} ({}) py={} ({:.7f})".format(r, lab_cpp, d_cpp, p["label"], p["dist"]))
    print("So sanh {} mau: {} khac biet.".format(len(rows), bad))
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
