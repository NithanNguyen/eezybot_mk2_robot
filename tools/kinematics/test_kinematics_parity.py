#!/usr/bin/env python3
"""
Kiem tra dong hoc cua firmware (lib/Kinematics/Mk2Kinematics.h, float) so voi eezybot_mk2_sim/mk2_kinematics.py
(double) tren PC, KHONG can ESP32. Can g++, numpy va repo eezybot_mk2_sim.

  python3 tools/kinematics/test_kinematics_parity.py                  # sim o ../eezybot_mk2_sim
  python3 tools/kinematics/test_kinematics_parity.py --sim <duong dan> --random 20000

Buoc 0: cac hang so trong include/arm_config.h ma dong hoc dung phai trung params.py (neu khong: chay lai
        eezybot_mk2_sim/tools/export_arm_config.py).
So sanh (dau vao lam tron float32 truoc khi dua cho CA HAI ben):
  FK  (gripper, cap)  : vi tri, |dp| <= 5e-4 mm
  IK  (gripper, cap)  : trang thai (OK / NEAR_AXIS / OUT_OF_REACH / gioi han nao) va goc, |dq| <= 2e-5 rad
  LIM                 : trang thai gioi han
  SV / FS             : anh xa goc servo, |d| <= 1e-4 do / 2e-6 rad
  JAC                 : Jacobian, |dJ| <= 1e-5 * max|J|
Dung sai: float32 co eps = 2^-24 ~ 6e-8; vi tri ~300 mm qua ~10 phep tinh -> ~1e-4 mm, goc ~2 rad -> ~1e-6 rad.
Ca BIEN (ket qua so sanh nguong co the lat vi lam tron float): bien do khoang cach toi nguong < 2e-3 do,
|cos(gamma)| - 1 trong 2e-6, hoac x^2 + y^2 - DY^2 trong 1e-3 mm^2. Ca bien khong so trang thai nhung van dem.
"""
import argparse
import math
import re
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SRC = Path(__file__).resolve().parent / "host_kinematics.cpp"
CFG = ROOT / "include" / "arm_config.h"
TOL_P, TOL_Q, TOL_SV, TOL_FS, TOL_J = 5e-4, 2e-5, 1e-4, 2e-6, 1e-5
BAND_DEG, BAND_C, BAND_RHO = 2e-3, 2e-6, 1e-3
TOOLS = ("gripper", "cap")


def f32(x):
    return float(np.float32(x))


def read_cfg():
    """Hang so float va Point3 trong arm_config.h."""
    txt = CFG.read_text()
    vals = {m.group(1): float(m.group(2)) for m in re.finditer(r"constexpr float (\w+) = (-?[\d.e+-]+)f;", txt)}
    for m in re.finditer(r"constexpr float (\w+)\[3\] = \{([^}]*)\};", txt):
        vals[m.group(1)] = [float(v.strip().rstrip("f")) for v in m.group(2).split(",")]
    pts = {m.group(1): [float(v.strip().rstrip("f")) for v in m.group(2).split(",")]
           for m in re.finditer(r"constexpr Point3 (\w+) = \{([^}]*)\};", txt)}
    return vals, pts


def check_cfg(P, vals):
    want = {"L_MAIN_ARM": P.L_UPPER, "L_HORIZ_ARM": P.L_FORE, "SHOULDER_HEIGHT": P.H_SHOULDER,
            "SHOULDER_OFFSET_X": P.SHOULDER_OFFSET_X, "BASE_GEAR_RATIO": P.BASE_GEAR_RATIO,
            "TOOL_GRIPPER_DX": P.TOOL_OFFSETS["gripper"][0], "TOOL_GRIPPER_DZ": P.TOOL_OFFSETS["gripper"][1],
            "TOOL_CAP_DX": P.TOOL_OFFSETS["cap"][0], "TOOL_CAP_DZ": P.TOOL_OFFSETS["cap"][1], "TOOL_DY": P.TOOL_OFFSET_Y,
            "THETA1_MIN": P.THETA1_LIM[0], "THETA1_MAX": P.THETA1_LIM[1],
            "THETA2_MIN": P.THETA2_LIM[0], "THETA2_MAX": P.THETA2_LIM[1],
            "PHI_MIN": P.PHI_LIM[0], "PHI_MAX": P.PHI_LIM[1], "ELBOW_MIN": P.REL_LIM[0], "ELBOW_MAX": P.REL_LIM[1],
            "CTRL_BASE_MIN": P.CTRL_BASE_DEG[0], "CTRL_BASE_MAX": P.CTRL_BASE_DEG[1],
            "CTRL_SHOULDER_MIN": P.CTRL_SHOULDER_DEG[0], "CTRL_SHOULDER_MAX": P.CTRL_SHOULDER_DEG[1],
            "CTRL_ELBOW_MIN": P.CTRL_ELBOW_DEG[0], "CTRL_ELBOW_MAX": P.CTRL_ELBOW_DEG[1],
            "SERVO_MIN_DEG": P.SERVO_RANGE[0], "SERVO_MAX_DEG": P.SERVO_RANGE[1],
            "SERVO_OFFSET_DEG": list(P.SERVO_OFFSET), "SERVO_SIGN": list(P.SERVO_SIGN)}
    bad = []
    for k, v in want.items():
        got = vals.get(k)
        if got is None or not np.allclose(got, v, rtol=1e-6, atol=1e-6):
            bad.append("{}: arm_config.h {} != params.py {}".format(k, got, v))
    return bad, len(want)


def py_status(K, fn, *a):
    """Chay ham Python, doi IKError / thong bao gioi han thanh ten trang thai cua Mk2Kinematics.h."""
    try:
        return "OK", fn(*a)
    except K.IKError as e:
        return msg_status(str(e)), None


def msg_status(m):
    for key, st in (("qua gan truc de", "NEAR_AXIS"), ("ngoai tam voi", "OUT_OF_REACH"), ("phi-th2=", "LIM_ELBOW"),
                    ("th1=", "LIM_THETA1"), ("th2=", "LIM_THETA2"), ("phi=", "LIM_PHI"),
                    ("CTRL_BASE_DEG", "LIM_CTRL_BASE"), ("CTRL_SHOULDER_DEG", "LIM_CTRL_SHOULDER"),
                    ("CTRL_ELBOW_DEG", "LIM_CTRL_ELBOW"), ("servo0=", "LIM_SERVO0"), ("servo1=", "LIM_SERVO1"),
                    ("servo2=", "LIM_SERVO2")):
        if key in m:
            return st
    raise ValueError("thong bao loi la: " + m)


def lim_status(K, q):
    ok, msg = K.within_limits(q)
    return "OK" if ok else msg_status(msg)


def margin_deg(P, K, q):
    """Khoang cach nho nhat (do) toi moi nguong ma within_limits() kiem tra."""
    th1, th2, phi = (math.degrees(v) for v in q)
    rel = phi - th2
    s = K.to_servo(q)
    pairs = [(th1, P.THETA1_LIM), (th2, P.THETA2_LIM), (rel, P.REL_LIM), (phi, P.PHI_LIM),
             (P.BASE_GEAR_RATIO * th1, P.CTRL_BASE_DEG), (th2 - 90.0, P.CTRL_SHOULDER_DEG), (phi, P.CTRL_ELBOW_DEG)]
    pairs += [(v, P.SERVO_RANGE) for v in s]
    return min(min(abs(v - lo), abs(hi - v)) for v, (lo, hi) in pairs)


def ik_boundary(P, K, p):
    x, y, z = p
    rho2 = x * x + y * y - K.DY * K.DY
    if abs(rho2) < BAND_RHO:
        return True
    if rho2 <= 0:
        return False
    rr = math.sqrt(rho2)
    r, zz = rr - K.DX - K.OX, z - K.H - K.DZ
    c = (r * r + zz * zz - K.L2 ** 2 - K.L3 ** 2) / (2 * K.L2 * K.L3)
    if abs(abs(c) - 1.0) < BAND_C:
        return True
    if abs(c) > 1:
        return False
    q = K.ik(p, check_limits=False)
    return margin_deg(P, K, q) < BAND_DEG


def run_cpp(lines):
    with tempfile.TemporaryDirectory() as td:
        exe = Path(td) / "host_kinematics"
        subprocess.check_call(["g++", "-std=c++11", "-O2", "-Wall", "-Wextra", "-Wdouble-promotion", "-Werror",
                               "-o", str(exe), str(SRC)])
        out = subprocess.run([str(exe)], input=("\n".join(lines) + "\n").encode(), stdout=subprocess.PIPE,
                             check=True).stdout.decode()
    return out.splitlines()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", default=str(ROOT.parent / "eezybot_mk2_sim"), help="duong dan repo eezybot_mk2_sim")
    ap.add_argument("--random", type=int, default=20000, help="so mau ngau nhien moi loai")
    a = ap.parse_args()
    sys.path.insert(0, a.sim)
    try:
        import params as P
        import mk2_kinematics as K
    except ImportError as e:
        sys.exit("Khong import duoc params / mk2_kinematics tu {} ({}). Dung --sim.".format(a.sim, e))

    vals, pts = read_cfg()
    cfg_bad, n_cfg = check_cfg(P, vals)
    if cfg_bad:
        print("\n".join(cfg_bad))
        sys.exit("include/arm_config.h khong khop params.py -> chay eezybot_mk2_sim/tools/export_arm_config.py")

    rng = np.random.RandomState(5)
    N = a.random
    # goc khop: rong hon gioi han de co ca trong va ngoai gioi han
    Q = np.c_[np.radians(rng.uniform(-60, 60, N)), np.radians(rng.uniform(30, 135, N)), np.radians(rng.uniform(-80, 30, N))]
    Q = np.vectorize(f32)(Q)
    SVIN = np.vectorize(f32)(rng.uniform(-10, 190, (N, 3)))
    # diem IK: (1) FK cua goc ngau nhien, (2) hop ngau nhien (nhieu diem ngoai tam), (3) sat truc de, (4) diem chu trinh
    ik_pts = []
    for i in range(N):
        tl = TOOLS[i % 2]
        K.set_tool(tl)
        ik_pts.append((tl, K.fk(*Q[i])))
        ik_pts.append((tl, rng.uniform([-50, -300, -50], [420, 300, 350])))
    for i in range(200):
        ik_pts.append((TOOLS[i % 2], np.r_[rng.uniform(-0.6, 0.6, 2), rng.uniform(0, 300)]))
    for nm, p in pts.items():
        if nm.startswith("P_"):
            ik_pts.append(("cap", np.array(p)))
    ik_pts = [(t, np.vectorize(f32)(np.asarray(p, float))) for t, p in ik_pts]

    lines = []
    for q in Q:
        for ti in (0, 1):
            lines.append("FK {} {!r} {!r} {!r}".format(ti, *q))
            lines.append("JAC {} {!r} {!r} {!r}".format(ti, *q))
        lines.append("LIM {!r} {!r} {!r}".format(*q))
        lines.append("SV {!r} {!r} {!r}".format(*q))
    for s in SVIN:
        lines.append("FS {!r} {!r} {!r}".format(*s))
    for tl, p in ik_pts:
        lines.append("IK {} {!r} {!r} {!r}".format(TOOLS.index(tl), *p))
    out = iter(run_cpp(lines))

    err = dict(fk=0.0, jac=0.0, sv=0.0, fs=0.0, ik_q=0.0, ik_p=0.0)
    bad, nbound, counts, report = 0, dict(lim=0, ik=0), {}, []

    def fail(msg):
        nonlocal bad
        bad += 1
        if bad <= 10:
            print("KHAC: " + msg)

    for q in Q:
        for tl in TOOLS:
            K.set_tool(tl)
            p_cpp = np.array([float(v) for v in next(out).split()])
            e = np.abs(p_cpp - K.fk(*q)).max(); err["fk"] = max(err["fk"], e)
            if e > TOL_P:
                fail("FK {} q={} |dp|={:.2e}".format(tl, q, e))
            J_cpp = np.array([float(v) for v in next(out).split()]).reshape(3, 3)
            J_py = K.jacobian(*q)
            e = np.abs(J_cpp - J_py).max() / np.abs(J_py).max(); err["jac"] = max(err["jac"], e)
            if e > TOL_J:
                fail("JAC {} q={} rel={:.2e}".format(tl, q, e))
        st_cpp = next(out).strip()
        if margin_deg(P, K, q) < BAND_DEG:
            nbound["lim"] += 1
        elif st_cpp != lim_status(K, q):
            fail("LIM q={} cpp={} py={}".format(q, st_cpp, lim_status(K, q)))
        counts["LIM " + st_cpp] = counts.get("LIM " + st_cpp, 0) + 1
        s_cpp = np.array([float(v) for v in next(out).split()])
        e = np.abs(s_cpp - K.to_servo(q)).max(); err["sv"] = max(err["sv"], e)
        if e > TOL_SV:
            fail("SV q={} |d|={:.2e} do".format(q, e))
    for s in SVIN:
        q_cpp = np.array([float(v) for v in next(out).split()])
        e = np.abs(q_cpp - np.array(K.from_servo(s))).max(); err["fs"] = max(err["fs"], e)
        if e > TOL_FS:
            fail("FS s={} |d|={:.2e} rad".format(s, e))
    for tl, p in ik_pts:
        K.set_tool(tl)
        f = next(out).split()
        st_cpp, q_cpp = f[0], np.array([float(v) for v in f[1:]])
        st_py, q_py = py_status(K, K.ik, p)
        counts["IK " + st_cpp] = counts.get("IK " + st_cpp, 0) + 1
        if ik_boundary(P, K, p):
            nbound["ik"] += 1
            continue
        if st_cpp != st_py:
            fail("IK {} p={} cpp={} py={}".format(tl, p, st_cpp, st_py))
            continue
        if st_py == "OK":
            e = np.abs(q_cpp - np.array(q_py)).max(); err["ik_q"] = max(err["ik_q"], e)
            ep = np.abs(K.fk(*q_cpp) - p).max(); err["ik_p"] = max(err["ik_p"], ep)
            if e > TOL_Q:
                fail("IK {} p={} |dq|={:.2e} rad".format(tl, p, e))

    # cac diem chu trinh trong arm_config.h phai giai duoc (dau cong tac 'cap')
    K.set_tool("cap")
    for nm, p in pts.items():
        if nm.startswith("P_") and py_status(K, K.ik, np.array(p))[0] != "OK":
            fail("diem chu trinh {} = {} khong giai duoc IK".format(nm, p))

    n_ik = len(ik_pts)
    print("arm_config.h khop params.py ({} hang so dong hoc).".format(n_cfg))
    print("Mau: FK/JAC {} x 2 dau cong tac, LIM/SV {}, FS {}, IK {} (gom {} diem chu trinh).".format(
        N, N, N, n_ik, sum(1 for k in pts if k.startswith("P_"))))
    print("Trang thai: " + ", ".join("{} {}".format(k, v) for k, v in sorted(counts.items())))
    print("Ca bien bo qua so trang thai: LIM {}, IK {}".format(nbound["lim"], nbound["ik"]))
    print("Sai so lon nhat: FK {:.1e} mm, IK goc {:.1e} rad, IK -> FK {:.1e} mm, JAC {:.1e} (tuong doi), "
          "SV {:.1e} do, FS {:.1e} rad".format(err["fk"], err["ik_q"], err["ik_p"], err["jac"], err["sv"], err["fs"]))
    print("So sanh {} lenh: {} khac biet.".format(len(lines), bad))
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
