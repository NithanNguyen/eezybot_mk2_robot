#!/usr/bin/env python3
"""
Kiem tra chuyen dong cua firmware (lib/Motion/ArmMotion.h) so voi mo phong eezybot_mk2_sim/run_sort.py, TUNG NHIP
LENH 20 ms, tren ca chu trinh phan loai. Can g++, numpy va repo eezybot_mk2_sim (results/sort/log.csv, caps.csv).

  python3 tools/motion/test_motion_parity.py                       # ArmMotion.h + lop Arduino (arm.cpp) tren PC
  python3 tools/motion/test_motion_parity.py --trace esp32.txt     # so trace da ghi tu ESP32 (chay kho)
  python3 tools/motion/test_motion_parity.py --port /dev/ttyUSB0   # tu gui lenh cho ESP32, ghi trace roi so

Tham chieu: results/sort/log.csv (run_sort.py --foreign 2, seed 0: 11 chu trinh gom ca 4 dich do / xanh / trang /
vut bo). Moi chu trinh = cac nhip tu pha "toi tram" toi het "ve Home"; dich lay tu caps.csv (cot dest).
So sanh moi nhip: ten pha, goc 3 servo, goc lenh kep; so nhip moi chu trinh phai bang nhau.
Dung sai |d goc| <= 0.005 do. Can cu: log.csv ghi 4 chu so thap phan (+-5e-5 do); IK float lech <= 4e-7 rad
(tools/kinematics, x2 qua ti so truyen de = 5e-5 do); he so gian k giai tich lech mo phong <= 3e-5 -> T lech
<= 4e-5 s -> goc lech <= 1.35 rad/s x 4e-5 s = 3e-3 do. 0.005 do nho hon 1 buoc LEDC 16 bit (~0.0275 do).
Kiem them tren trace: van toc dau cong tac (FK, sai phan 20 ms) <= 150 mm/s nhu summary.txt cua mo phong.
Che do mac dinh chay hai nguon:
  [1] host_motion.cpp : chi lib/Motion/ArmMotion.h, ca 11 chu trinh theo thu tu log.csv
  [2] host_arm.cpp    : src/robot/arm.cpp that (task 20 ms, hang doi, lenh "ARM ...", in CMD / ARM DONE) voi stub
                        FreeRTOS trong tools/motion/stub (thread + mutex C++), 4 dich; trace qua dung duong in Serial
"""
import argparse
import csv
import math
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SRC = Path(__file__).resolve().parent / "host_motion.cpp"
SRC_ARM = Path(__file__).resolve().parent / "host_arm.cpp"
TOL_DEG = 0.005
SORT_PHASES = {"toi tram", "ha xuong", "dong kep", "nhac len", "toi ngan 1", "toi ngan 2", "toi ngan 3",
               "toi vung vut bo", "ha xuong vung vut bo", "mo kep", "ve Home"}
DEST_FW = {"red": "RED", "green": "GREEN", "white": "WHITE", "reject": "REJECT"}


def load_reference(sim):
    """[(dest, [(pha, s0, s1, s2, kep), ...]), ...] theo thu tu chu trinh trong log.csv."""
    res = Path(sim) / "results" / "sort"
    with open(res / "caps.csv") as f:
        dests = [DEST_FW[r["dest"]] for r in csv.DictReader(f)]
    cycles, cur = [], None
    with open(res / "log.csv") as f:
        for r in csv.DictReader(f):
            ph = r["pha"]
            if ph == "toi tram" and (cur is None or cur[-1][0] != "toi tram"):
                cur = []
                cycles.append(cur)
            if cur is not None and ph in SORT_PHASES:
                cur.append((ph, float(r["servo0_deg"]), float(r["servo1_deg"]), float(r["servo2_deg"]),
                            float(r["kep_lenh_deg"])))
            elif cur is not None:
                cur = None
    if len(cycles) != len(dests):
        sys.exit("log.csv co {} chu trinh nhung caps.csv co {} nap".format(len(cycles), len(dests)))
    return list(zip(dests, cycles))


def parse_trace(text):
    """Tach trace (host hoac ESP32) thanh [(dest, status, [(pha, s0, s1, s2, kep)], extra), ...]."""
    out, cur = [], []
    for line in text.splitlines():
        line = line.strip()
        if line.startswith("CMD,"):
            f = line.split(",")
            cur.append((f[2], float(f[3]), float(f[4]), float(f[5]), float(f[6])))
        elif line.startswith("ARM DONE"):
            f = line.split()
            kv = dict(x.split("=", 1) for x in f[3:] if "=" in x)
            out.append((f[2], kv.get("status", "?"), cur, kv))
            cur = []
    return out


def tool_speed_max(rows, K):
    """Van toc dau cong tac lon nhat (mm/s) tu goc servo lenh, sai phan 20 ms (giong summary.txt)."""
    K.set_tool("cap")
    ps = np.array([K.fk(*K.from_servo(np.array(r[1:4]))) for r in rows])
    return float(np.linalg.norm(np.diff(ps, axis=0), axis=1).max()) / 0.02 if len(ps) > 1 else 0.0


def compare(ref, got, K):
    """So tung chu trinh trong got voi chu trinh dau tien cung dich trong ref."""
    by_dest = {}
    for d, cyc in ref:
        by_dest.setdefault(d, []).append(cyc)
    bad, worst, vmax, used = 0, 0.0, 0.0, {}
    for d, status, rows, _ in got:
        if d == "HOME":
            continue                                       # ve Home luc khoi dong: khong co tham chieu
        k = used.get(d, 0)
        cands = by_dest.get(d, [])
        if not cands:
            print("KHAC: khong co chu trinh tham chieu cho {}".format(d)); bad += 1; continue
        refc = cands[min(k, len(cands) - 1)]
        used[d] = k + 1
        if status != "OK":
            print("KHAC: {} ket thuc voi status={}".format(d, status)); bad += 1
        if len(rows) != len(refc):
            print("KHAC: {} co {} nhip, mo phong {} nhip".format(d, len(rows), len(refc))); bad += 1
        nph = sum(1 for a, b in zip(rows, refc) if a[0] != b[0])
        if nph:
            print("KHAC: {} lech ten pha o {} nhip".format(d, nph)); bad += 1
        m = min(len(rows), len(refc))
        if m:
            e = np.abs(np.array([r[1:] for r in rows[:m]]) - np.array([r[1:] for r in refc[:m]])).max()
            worst = max(worst, e)
            if e > TOL_DEG:
                i = int(np.argmax(np.abs(np.array([r[1:] for r in rows[:m]]) - np.array([r[1:] for r in refc[:m]])).max(1)))
                print("KHAC: {} lech goc {:.4f} do tai nhip {} (pha {})".format(d, e, i + 1, rows[i][0])); bad += 1
        v = tool_speed_max(rows, K)
        vmax = max(vmax, v)
        if v > 150.0 + 0.05:
            print("KHAC: {} van toc dau cong tac {:.2f} mm/s > 150".format(d, v)); bad += 1
    return bad, worst, vmax


def run_host(dests):
    with tempfile.TemporaryDirectory() as td:
        exe = Path(td) / "host_motion"
        subprocess.check_call(["g++", "-std=c++11", "-O2", "-Wall", "-Wextra", "-Wdouble-promotion", "-Werror",
                               "-o", str(exe), str(SRC)])
        return subprocess.run([str(exe)], input=("\n".join("SORT " + d for d in dests) + "\n").encode(),
                              stdout=subprocess.PIPE, check=True).stdout.decode()


def run_glue(dests):
    with tempfile.TemporaryDirectory() as td:
        exe = Path(td) / "host_arm"
        here = Path(__file__).resolve().parent
        subprocess.check_call(["g++", "-std=gnu++17", "-O2", "-pthread", "-Wall", "-Wextra", "-Werror",
                               "-I" + str(here / "stub"), "-I" + str(ROOT / "lib" / "Motion"), "-o", str(exe),
                               str(SRC_ARM), str(ROOT / "src" / "robot" / "arm.cpp")])
        cmds = ["ARM TRACE 1"] + ["ARM SORT " + d for d in dests] + ["ARM TRACE 0", "ARM TIMING"]
        return subprocess.run([str(exe)], input=("\n".join(cmds) + "\n").encode(), stdout=subprocess.PIPE,
                              check=True, timeout=300).stdout.decode()


def run_port(port, dests, save):
    import serial  # pyserial (tools/colorcal/requirements.txt)
    lines = []
    with serial.Serial(port, 115200, timeout=0.5) as s:
        time.sleep(2.5)                       # mo cong lam ESP32 reset: cho "# READY"
        s.reset_input_buffer()

        def send(cmd):
            s.write((cmd + "\n").encode())

        def wait_idle(timeout=15.0):
            t0 = time.time()
            while time.time() - t0 < timeout:
                send("ARM STATUS")
                t1 = time.time()
                while time.time() - t1 < 1.0:
                    ln = s.readline().decode(errors="replace").strip()
                    if ln.startswith("ARM STATUS"):
                        if "busy=0" in ln:
                            return
                        break
                time.sleep(0.2)
            sys.exit("ESP32 van ban sau {} s (khong co 'ARM STATUS,busy=0')".format(timeout))

        send("ARM TRACE 1")
        for d in dests:
            wait_idle()
            send("ARM SORT " + d)
            t0 = time.time()
            while time.time() - t0 < 60:
                ln = s.readline().decode(errors="replace").strip()
                if ln:
                    lines.append(ln)
                if ln.startswith("ERR"):
                    print("ESP32: " + ln)
                if ln.startswith("ARM DONE") or ln.startswith("ERR"):
                    break
            else:
                sys.exit("Het thoi gian cho 'ARM DONE' cho {}".format(d))
        send("ARM TRACE 0")
        send("ARM TIMING")
        t0 = time.time()
        while time.time() - t0 < 3:
            ln = s.readline().decode(errors="replace").strip()
            if ln:
                lines.append(ln)
            if ln.startswith("ARM TIMING"):
                break
    text = "\n".join(lines) + "\n"
    if save:
        Path(save).write_text(text)
        print("Da ghi trace: {}".format(save))
    return text


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", default=str(ROOT.parent / "eezybot_mk2_sim"), help="duong dan repo eezybot_mk2_sim")
    ap.add_argument("--trace", help="file trace da ghi tu ESP32 (dong CMD,... va ARM DONE ...)")
    ap.add_argument("--port", help="cong Serial cua ESP32: tu gui ARM SORT cho 4 dich va ghi trace")
    ap.add_argument("--save", default="esp32_trace.txt", help="noi luu trace khi dung --port")
    a = ap.parse_args()
    sys.path.insert(0, a.sim)
    try:
        import mk2_kinematics as K
    except ImportError as e:
        sys.exit("Khong import duoc mk2_kinematics tu {} ({}). Dung --sim.".format(a.sim, e))

    ref = load_reference(a.sim)
    four = ["RED", "GREEN", "WHITE", "REJECT"]
    if a.port:
        sources = [("ESP32 ({})".format(a.port), lambda: run_port(a.port, four, a.save))]
    elif a.trace:
        sources = [("trace {}".format(a.trace), lambda: Path(a.trace).read_text(errors="replace"))]
    else:
        sources = [("[1] ArmMotion.h tren PC", lambda: run_host([d for d, _ in ref])),
                   ("[2] arm.cpp + stub FreeRTOS tren PC", lambda: run_glue(four))]
    print("Tham chieu: {} chu trinh trong {}/results/sort/log.csv.".format(len(ref), a.sim))
    total_bad = 0
    for src, fn in sources:
        text = fn()
        got = [g for g in parse_trace(text) if g[0] != "HOME"]   # ve Home luc khoi dong: khong co tham chieu
        if not got:
            print("{}: KHONG tim thay chu trinh nao (dong 'ARM DONE ...')".format(src))
            total_bad += 1
            continue
        bad, worst, vmax = compare(ref, got, K)
        n_ticks = sum(len(r) for _, _, r, _ in got)
        cmax = max(float(kv.get("cmax_us", "nan")) for _, _, _, kv in got)
        drop = max(int(kv.get("dropped", "0")) for _, _, _, kv in got)
        if drop:
            print("KHAC: mat {} dong trace (hang doi day)".format(drop)); bad += 1
        print("{}: ".format(src) + ", ".join("{} {} nhip".format(d, len(r)) for d, _, r, _ in got))
        print("    lech goc lon nhat {:.4f} do (dung sai {}), van toc dau cong tac lon nhat {:.2f} mm/s, "
              "thoi gian tinh 1 nhip lon nhat {:.0f} us".format(worst, TOL_DEG, vmax, cmax))
        for ln in text.splitlines():
            if ln.startswith("ARM TIMING"):
                print("    " + ln)
        print("    {} chu trinh ({} nhip): {} khac biet.".format(len(got), n_ticks, bad))
        total_bad += bad
    print("Tong: {} khac biet.".format(total_bad))
    sys.exit(1 if total_bad else 0)


if __name__ == "__main__":
    main()
