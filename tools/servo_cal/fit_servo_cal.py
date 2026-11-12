#!/usr/bin/env python3
"""
Tinh hieu chuan servo tu so do bang thuoc (buoc 6, docs/hieu_chuan_servo.md). Can numpy, scipy va eezybot_mk2_sim.

  python3 tools/servo_cal/fit_servo_cal.py do_dac.csv              # sim o ../eezybot_mk2_sim
  python3 tools/servo_cal/fit_servo_cal.py --selftest              # kiem tra script bang du lieu gia co nhieu

CSV (mau: tools/servo_cal/measurements_template.csv), moi dong mot lan do:
  joint   : base | shoulder | elbow | gripper
  us      : do rong xung dang phat (us, doc tu lenh STATUS / AT cua firmware calib)
  method  : cach do -> goc khop q (do)
    chord   (base)    : value = day cung (mm) giua vet danh dau o Home va vet hien tai, DUONG neu vet lech sang TRAI
                        khi dung sau robot nhin ra truoc (+y); radius = khoang cach truc de -> vet (mm).
                        q = theta1 = 2 asin(value / (2 radius))
    cos     (shoulder): value = khoang cach ngang (mm) tu truc vai O toi day doi tha tu truc khuyu E, DUONG neu E
                        o phia TRUOC O (huong ra tram). q = theta2 = acos(value / L_UPPER)
    sin     (elbow)   : value = do cao tam truc co tay W tru do cao tam truc khuyu E (mm). q = phi = asin(value / L_FORE)
    gap     (gripper) : value = khe nho nhat giua hai mat kep (mm). q = goc banh rang tu luc dong, nghich dao
                        gripper.gap_at() cua mo phong (hinh hoc do tu gripper that)
    deg     (moi khop): value = goc khop do truc tiep (do): theta1, theta2, phi hoac goc banh rang kep
  radius  : chi dung cho chord
Mo hinh moi khop: us = c + k q (hoi quy tuyen tinh, can >= 3 muc xung khac nhau). In ra:
  xung o Home (theta1 = 0, theta2 = 90, phi = 0), k (us / do khop, co dau = chieu quay), sai so du (do),
  xung tai cac gioi han khop cua params.py (canh bao neu ra ngoai 500..2500 us),
  va doan params.py de dan vao eezybot_mk2_sim/params.py.
"""
import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
JOINTS = ("base", "shoulder", "elbow", "gripper")
HOME_Q = {"base": 0.0, "shoulder": 90.0, "elbow": 0.0}
METHODS = {"base": ("chord", "deg"), "shoulder": ("cos", "deg"), "elbow": ("sin", "deg"), "gripper": ("gap", "deg")}
PULSE_HARD = (500.0, 2500.0)


def to_angle(P, GR, joint, method, value, radius):
    if method not in METHODS[joint]:
        raise ValueError("khop {} khong dung cach do '{}' (chon {})".format(joint, method, "/".join(METHODS[joint])))
    if method == "deg":
        return value
    if method == "chord":
        if not radius or radius <= 0:
            raise ValueError("chord can radius > 0")
        if abs(value) > 2 * radius:
            raise ValueError("day cung {} mm lon hon duong kinh {} mm".format(value, 2 * radius))
        return math.copysign(math.degrees(2 * math.asin(abs(value) / (2 * radius))), value)
    if method == "cos":
        if abs(value) > P.L_UPPER:
            raise ValueError("|value| {} > L_UPPER {}".format(value, P.L_UPPER))
        return math.degrees(math.acos(value / P.L_UPPER))
    if method == "sin":
        if abs(value) > P.L_FORE:
            raise ValueError("|value| {} > L_FORE {}".format(value, P.L_FORE))
        return math.degrees(math.asin(value / P.L_FORE))
    if method == "gap":
        from scipy.optimize import brentq
        lo, hi = GR.gap_at(0.0), GR.gap_at(85.0)
        if not lo <= value <= hi:
            raise ValueError("khe {} mm ngoai {:.2f}..{:.2f} mm (goc 0..85 do)".format(value, lo, hi))
        return brentq(lambda a: GR.gap_at(a) - value, 0.0, 85.0, xtol=1e-6)
    raise ValueError(method)


def fit(rows):
    """rows: [(us, q)] -> (c, k, rms_deg, n_levels)."""
    u = np.array([r[0] for r in rows], float)
    q = np.array([r[1] for r in rows], float)
    nlev = len(np.unique(np.round(u)))
    if nlev < 2:
        return None
    k, c = np.polyfit(q, u, 1)
    rms = float(np.sqrt(np.mean(((u - c) / k - q) ** 2)))
    return c, k, rms, nlev


def joint_limits(P):
    """Khoang goc khop hop le (do) theo params.py, giao cac gioi han co hoc va tam lenh."""
    g = P.BASE_GEAR_RATIO
    th1 = (max(P.THETA1_LIM[0], P.CTRL_BASE_DEG[0] / g), min(P.THETA1_LIM[1], P.CTRL_BASE_DEG[1] / g))
    th2 = (max(P.THETA2_LIM[0], P.CTRL_SHOULDER_DEG[0] + 90), min(P.THETA2_LIM[1], P.CTRL_SHOULDER_DEG[1] + 90))
    phi = (max(P.PHI_LIM[0], P.CTRL_ELBOW_DEG[0]), min(P.PHI_LIM[1], P.CTRL_ELBOW_DEG[1]))
    return {"base": th1, "shoulder": th2, "elbow": phi, "gripper": (0.0, P.GRIP_TRAVEL_DEG)}


def analyse(P, GR, records):
    by = {j: [] for j in JOINTS}
    for r in records:
        by[r["joint"]].append((r["us"], to_angle(P, GR, r["joint"], r["method"], r["value"], r.get("radius"))))
    res, ok = {}, True
    lims = joint_limits(P)
    print("Khop      n  muc   xung Home   k (us/do khop)  du (do)   xung tai gioi han khop")
    for j in JOINTS:
        rows = by[j]
        if not rows:
            print("{:9s} -- chua co so do".format(j))
            continue
        f = fit(rows)
        if f is None:
            print("{:9s} {:2d}  chi 1 muc xung: can >= 3 muc khac nhau".format(j, len(rows)))
            ok = False
            continue
        c, k, rms, nlev = f
        ref = HOME_Q.get(j, 0.0)
        u_home = c + k * ref
        lo, hi = lims[j]
        u_lo, u_hi = c + k * lo, c + k * hi
        warn = []
        if nlev < 3:
            warn.append("chi {} muc xung (nen >= 3)".format(nlev))
        if rms > 1.0:
            warn.append("du {:.2f} do > 1 do: do lai".format(rms))
        if not (PULSE_HARD[0] <= min(u_lo, u_hi) and max(u_lo, u_hi) <= PULSE_HARD[1]):
            warn.append("gioi han khop can xung ngoai 500..2500 us")
        if not (PULSE_HARD[0] <= u_home <= PULSE_HARD[1]):
            warn.append("xung Home ngoai 500..2500 us")
        ok &= not warn
        print("{:9s} {:2d}  {:3d}   {:8.1f}   {:+10.3f}     {:6.2f}   q={:.1f}..{:.1f} do -> {:.0f}..{:.0f} us{}".format(
            j, len(rows), nlev, u_home, k, rms, lo, hi, u_lo, u_hi, ("   CANH BAO: " + "; ".join(warn)) if warn else ""))
        res[j] = dict(c=c, k=k, rms=rms, u_home=u_home, n=len(rows))
    if "gripper" in res:
        g = res["gripper"]
        print("Kep: dong (0 do) {:.0f} us, giu nap ({:g} do) {:.0f} us, cham nap ({:g} do) {:.0f} us, mo ({:g} do) {:.0f} us".format(
            g["c"], P.GRIP_HOLD_DEG, g["c"] + g["k"] * P.GRIP_HOLD_DEG, P.CAP_GRIP_DEG, g["c"] + g["k"] * P.CAP_GRIP_DEG,
            P.GRIP_OPEN_DEG, g["c"] + g["k"] * P.GRIP_OPEN_DEG))
    if all(j in res for j in ("base", "shoulder", "elbow")):
        print("\nDoan dan vao eezybot_mk2_sim/params.py (muc 4):")
        print("SERVO_PULSE_HOME_US = ({:.1f}, {:.1f}, {:.1f})        # [DO-THAT] xung tai Home: de, vai, khuyu".format(
            *(res[j]["u_home"] for j in ("base", "shoulder", "elbow"))))
        print("SERVO_PULSE_PER_DEG = ({:+.4f}, {:+.4f}, {:+.4f})    # [DO-THAT] us / do KHOP (theta1, theta2, phi), co dau".format(
            *(res[j]["k"] for j in ("base", "shoulder", "elbow"))))
    if "gripper" in res:
        print("GRIP_PULSE_CLOSED_US = {:.1f}                       # [DO-THAT] xung khi goc banh rang kep = 0 (dong)".format(
            res["gripper"]["c"]))
        print("GRIP_PULSE_PER_DEG = {:+.4f}                       # [DO-THAT] us / do banh rang kep, co dau".format(
            res["gripper"]["k"]))
    return res, ok


def read_csv(path):
    out = []
    with open(path, newline="") as f:
        for i, r in enumerate(csv.DictReader(row for row in f if row.strip() and not row.lstrip().startswith("#"))):
            j = r["joint"].strip().lower()
            if j not in JOINTS:
                raise ValueError("dong {}: joint '{}' khong hop le".format(i + 2, j))
            rad = (r.get("radius") or "").strip()
            out.append(dict(joint=j, us=float(r["us"]), method=r["method"].strip().lower(), value=float(r["value"]),
                            radius=float(rad) if rad else None))
    return out


def selftest(P, GR):
    """Du lieu gia: gia su anh xa that, sinh so do bang thuoc lam tron 1 mm (0.5 mm voi khe kep), fit lai, so sanh."""
    rng = np.random.default_rng(1)
    # (xung tai Home -- voi kep: tai goc 0 = dong --, k us/do khop)
    true = {"base": (1512.0, -22.6), "shoulder": (1488.0, 10.9), "elbow": (1530.0, -11.3), "gripper": (1020.0, 10.6)}
    R = 280.0
    recs = []
    for us in (1200, 1350, 1500, 1650, 1800):
        th1 = (us - true["base"][0]) / true["base"][1]
        chord = math.copysign(2 * R * math.sin(math.radians(abs(th1)) / 2), th1)
        recs.append(dict(joint="base", us=us, method="chord", value=round(chord + rng.normal(0, 0.5)), radius=R))
    for us in (1300, 1400, 1500, 1600, 1700):
        th2 = 90 + (us - true["shoulder"][0]) / true["shoulder"][1]
        recs.append(dict(joint="shoulder", us=us, method="cos", value=round(P.L_UPPER * math.cos(math.radians(th2)) + rng.normal(0, 0.5)), radius=None))
    for us in (1400, 1500, 1600, 1700, 1800):
        phi = (us - true["elbow"][0]) / true["elbow"][1]
        recs.append(dict(joint="elbow", us=us, method="sin", value=round(P.L_FORE * math.sin(math.radians(phi)) + rng.normal(0, 0.5)), radius=None))
    for us in (1100, 1200, 1300, 1400, 1500):
        a = (us - true["gripper"][0]) / true["gripper"][1]
        recs.append(dict(joint="gripper", us=us, method="gap", value=round(2 * (GR.gap_at(a) + rng.normal(0, 0.25))) / 2, radius=None))
    res, _ = analyse(P, GR, recs)
    bad = 0
    print("\nSo voi anh xa gia dinh:")
    for j, (c0, k0) in true.items():
        du, dk = res[j]["u_home"] - c0, (res[j]["k"] - k0) / k0
        # sai so cho phep: thuoc 1 mm -> ~0.4 do tai Home -> ~5 us; do doc ~3 %
        flag = abs(du) <= 12 and abs(dk) <= 0.03
        bad += not flag
        print("  {:9s} xung Home lech {:+6.1f} us, k lech {:+6.2%}  {}".format(j, du, dk, "OK" if flag else "LECH"))
    print("SELFTEST {}".format("DAT" if not bad else "KHONG DAT"))
    return bad == 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="?", help="file so do")
    ap.add_argument("--sim", default=str(ROOT.parent / "eezybot_mk2_sim"), help="duong dan repo eezybot_mk2_sim")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    sys.path.insert(0, a.sim)
    try:
        import params as P
        import gripper as GR
    except ImportError as e:
        sys.exit("Khong import duoc params / gripper tu {} ({}). Dung --sim; can numpy, scipy, trimesh.".format(a.sim, e))
    if a.selftest:
        sys.exit(0 if selftest(P, GR) else 1)
    if not a.csv:
        ap.error("can file CSV hoac --selftest")
    _, ok = analyse(P, GR, read_csv(a.csv))
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
