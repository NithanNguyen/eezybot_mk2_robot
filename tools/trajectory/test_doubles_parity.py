#!/usr/bin/env python3
"""
Kiem tra bo sinh double-S cua firmware (lib/Trajectory/DoubleS1D.h, float) so voi Ruckig tren PC, KHONG can ESP32.
Can g++, numpy, ruckig va repo eezybot_mk2_sim (lay trajectories.ruckig_1d — cung ham mo phong dung).

  python3 tools/trajectory/test_doubles_parity.py                     # sim o ../eezybot_mk2_sim
  python3 tools/trajectory/test_doubles_parity.py --sim <duong dan> --random 3000

So tung mau tai t_k = k*dt (k = 1..n, giong vong update cua Ruckig):
  vi tri s(t)  : voi trajectories.ruckig_1d()         (dung ham mo phong dung cho duong thang)
  v(t), a(t)   : voi vong Ruckig 1 chieu cung tham so  (ruckig_1d chi tra vi tri)
  thoi gian T va so mau n.
  jerk(t)      : gia toc Ruckig tuyen tinh tung khuc nen jerk tham chieu = do doc gia toc hai ben t_k
                 (lech 1e-5 s); bo qua mau sat moc pha (doc trai != doc phai). Lay toi 200 mau deu moi ca.
Bo ca:
  [duong thang] quang duong thuc te cua chu trinh voi 150 mm/s, 1500 mm/s^2, 30000 mm/s^3
  [khop sigma ] sigma 0 -> 1 voi gioi han (JV, JA, JJ)/|dq| va gian thoi gian k (nhu run_sort.joint_move)
  [ngau nhien ] h, v, a, j, dt rai log, phu ca 4 truong hop (dat/khong dat vmax, amax)
  [bien       ] h = 0, h am, gioi han khong hop le
Dung sai = lam tron float32 cua phep tinh + luong tu thoi gian. Thoi gian t = k*dt va T - t (pha giam toc) la
float nen chi chinh xac toi dt_q = 4 ulp(T) ~ 4 T 2^-23; trong khoang do a doi J dt_q, v doi alim dt_q, s doi vlim dt_q:
  |ds| <= 2e-6 |h| + vlim dt_q + 1e-6     |dv| <= 2e-5 vlim + alim dt_q + 1e-6
  |da| <= 2e-4 alim + J dt_q + 1e-5       |dT| <= 2e-6 T + 1e-7
Voi chuyen dong that (T <= vai giay, moi doan tinh thoi gian tu luc bat dau doan) dt_q <= 1e-6 s nen thanh phan
luong tu thoi gian nho; script in rieng sai so lon nhat cua tung nhom ca.
"""
import argparse
import math
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SRC = Path(__file__).resolve().parent / "host_doubles.cpp"
TOL_S, TOL_V, TOL_A, TOL_T = 2e-6, 2e-5, 2e-4, 2e-6


def ruckig_va(h, v, a, j, dt):
    """Vong Ruckig 1 chieu (giong trajectories.ruckig_1d) nhung tra ca van toc, gia toc, thoi gian T va doi
    tuong Trajectory (de lay jerk tham chieu)."""
    from ruckig import InputParameter, OutputParameter, Result, Ruckig, Trajectory
    inp = InputParameter(1)
    inp.current_position = [0.0]; inp.current_velocity = [0.0]; inp.current_acceleration = [0.0]
    inp.target_position = [h]; inp.target_velocity = [0.0]; inp.target_acceleration = [0.0]
    inp.max_velocity = [v]; inp.max_acceleration = [a]; inp.max_jerk = [j]
    traj = Trajectory(1)
    Ruckig(1).calculate(inp, traj)
    otg, out = Ruckig(1, dt), OutputParameter(1)
    vel, acc, res = [], [], Result.Working
    while res == Result.Working:
        res = otg.update(inp, out)
        vel.append(out.new_velocity[0]); acc.append(out.new_acceleration[0])
        out.pass_to_input(inp)
    if res != Result.Finished:
        raise RuntimeError("Ruckig loi: {}".format(res))
    return traj.duration, np.array(vel), np.array(acc), traj


def f32(x):
    """Gia tri ma C++ doc duoc bang scanf("%f") — Ruckig cung nhan dung so nay de so sanh cong bang."""
    return float(np.float32(x))


def build_cases(n_random):
    cases = []
    V, A, J = 150.0, 1500.0, 30000.0
    # [duong thang] ha/nhac 40 mm, ha vung vut bo 33 mm, cac doan dai cua chu trinh (mm)
    for L in (40.0, 33.0, 132.5, 142.6, 144.5, 146.0, 162.9, 199.7):
        cases.append(("duong thang", L, V, A, J, 0.02))
    # [khop sigma] |dq| lon nhat cua cac doan khop trong chu trinh: 0.52..0.88 rad (sim: mk2_kinematics.ik + sort_scene.task_points);
    # quet rong hon 0.05..2 rad va gian thoi gian k = 1..1.3 (run_sort.joint_move)
    JV, JA, JJ = V / 280.0, A / 280.0, J / 280.0
    rng = np.random.RandomState(11)
    for dq in (0.05, 0.3, 0.7, 1.0, 1.6, 2.0):
        for k in (1.0, 1.1, 1.3):
            cases.append(("khop sigma", 1.0, JV / dq / k, JA / dq / k ** 2, JJ / dq / k ** 3, 0.02))
    # [ngau nhien]
    for _ in range(n_random):
        h = 10 ** rng.uniform(-3, math.log10(500)) * rng.choice([-1, 1])
        v, a, j = 10 ** rng.uniform(0, 2.7), 10 ** rng.uniform(1, 4), 10 ** rng.uniform(2, 6)
        dt = float(rng.choice([0.02, 0.005, 0.001]))
        cases.append(("ngau nhien", h, v, a, j, dt))
    return [(g, f32(h), f32(v), f32(a), f32(j), f32(dt)) for g, h, v, a, j, dt in cases]


def run_cpp(rows):
    with tempfile.TemporaryDirectory() as td:
        exe = Path(td) / "host_doubles"
        # -Wdouble-promotion: bao loi neu DoubleS1D.h vo tinh tinh bang double (cham tren ESP32)
        subprocess.check_call(["g++", "-std=c++11", "-O2", "-Wall", "-Wextra", "-Wdouble-promotion", "-Werror",
                               "-o", str(exe), str(SRC)])
        stdin = "\n".join("{!r} {!r} {!r} {!r} {!r}".format(*r) for r in rows) + "\n"
        out = subprocess.run([str(exe)], input=stdin.encode(), stdout=subprocess.PIPE, check=True).stdout.decode()
    res, lines, i = [], out.splitlines(), 0
    while i < len(lines):
        tag, ok, T, n, vlim, alim = lines[i].split()
        assert tag == "CASE"
        n = int(n)
        S = np.array([[float(x) for x in l.split()] for l in lines[i + 1:i + 1 + n]])
        res.append(dict(ok=int(ok), T=float(T), vlim=float(vlim), alim=float(alim), S=S))
        i += 1 + n
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", default=str(ROOT.parent / "eezybot_mk2_sim"), help="duong dan repo eezybot_mk2_sim")
    ap.add_argument("--random", type=int, default=2000, help="so ca ngau nhien")
    a = ap.parse_args()
    sys.path.insert(0, a.sim)
    try:
        import trajectories as TR
    except ImportError as e:
        sys.exit("Khong import duoc trajectories.py tu {} ({}). Dung --sim.".format(a.sim, e))

    cases = build_cases(a.random)
    edge = [("bien h=0", 0.0, 150.0, 1500.0, 30000.0, 0.02), ("bien v<=0", 10.0, 0.0, 1500.0, 30000.0, 0.02),
            ("bien a<=0", 10.0, 150.0, -1.0, 30000.0, 0.02)]
    rows = [c[1:] for c in cases] + [c[1:] for c in edge]
    out = run_cpp(rows)

    bad, worst, regimes, n_samples, n_boundary, n_jerk = 0, {}, {}, 0, 0, 0
    for (grp, h, v, acc, j, dt), r in zip(cases, out):
        s_ref = TR.ruckig_1d(h, v, acc, j, dt) if h >= 0 else -TR.ruckig_1d(-h, v, acc, j, dt)
        T_ref, v_ref, a_ref, traj = ruckig_va(h, v, acc, j, dt)
        S = r["S"]
        msgs = []
        if not r["ok"]:
            msgs.append("plan() tra false")
        if len(S) != len(s_ref):
            # chi chap nhan lech 1 mau khi T gan dung boi so cua dt (lam tron float/double khac nhau)
            if abs(len(S) - len(s_ref)) == 1 and abs(T_ref / dt - round(T_ref / dt)) < 1e-4:
                n_boundary += 1
            else:
                msgs.append("so mau {} != Ruckig {}".format(len(S), len(s_ref)))
        m = min(len(S), len(s_ref))
        dtq = 4.0 * float(np.spacing(np.float32(max(r["T"], T_ref))))   # luong tu thoi gian float
        vl, al = abs(r["vlim"]), abs(r["alim"])
        es = np.abs(S[:m, 0] - s_ref[:m]).max() / (abs(h) + 1e-12)
        ev = np.abs(S[:m, 1] - v_ref[:m]).max() / (abs(r["vlim"]) + 1e-12)
        ea = np.abs(S[:m, 2] - a_ref[:m]).max() / (abs(r["alim"]) + 1e-12)
        eT = abs(r["T"] - T_ref) / (T_ref + 1e-12)
        if np.abs(S[:m, 0] - s_ref[:m]).max() > TOL_S * abs(h) + vl * dtq + 1e-6:
            msgs.append("ds/h = {:.2e}".format(es))
        if np.abs(S[:m, 1] - v_ref[:m]).max() > TOL_V * vl + al * dtq + 1e-6:
            msgs.append("dv/vlim = {:.2e}".format(ev))
        if np.abs(S[:m, 2] - a_ref[:m]).max() > TOL_A * al + j * dtq + 1e-5:
            msgs.append("da/alim = {:.2e}".format(ea))
        if abs(r["T"] - T_ref) > TOL_T * T_ref + 1e-7:
            msgs.append("dT/T = {:.2e}".format(eT))
        # jerk: do doc gia toc Ruckig hai ben t_k (tuyen tinh tung khuc -> chinh xac khi cung pha)
        dl = 1e-5
        for kk in np.unique(np.linspace(0, m - 1, min(m, 200)).astype(int)):
            t = (kk + 1) * dt
            if t + dl >= T_ref or t - dl <= 0:
                continue
            a0, al_, ar_ = (traj.at_time(x)[2][0] for x in (t, t - dl, t + dl))
            sl, sr = (a0 - al_) / dl, (ar_ - a0) / dl
            if abs(sl - sr) > 1e-3 * j:
                continue                                  # sat moc pha
            n_jerk += 1
            if abs(S[kk, 3] - sr) > 2e-3 * j:
                msgs.append("jerk {:.6g} != Ruckig {:.6g} tai t={:.6g}".format(S[kk, 3], sr, t))
                break
        if abs(S[-1, 0] - h) > TOL_S * abs(h) + 1e-6 or S[-1, 1] != 0.0 or S[-1, 2] != 0.0:
            msgs.append("mau cuoi khong phai dich nghi")
        w = worst.setdefault(grp, dict(s=0.0, v=0.0, a=0.0, T=0.0, Tmax=0.0))
        for k_, e in (("s", es), ("v", ev), ("a", ea), ("T", eT), ("Tmax", T_ref)):
            w[k_] = max(w[k_], e)
        reg = ("dat vmax" if abs(abs(r["vlim"]) - v) <= 1e-4 * v else "khong dat vmax") + ", " + \
              ("dat amax" if abs(abs(r["alim"]) - acc) <= 1e-4 * acc else "khong dat amax")
        regimes[reg] = regimes.get(reg, 0) + 1
        n_samples += m
        if msgs:
            bad += 1
            if bad <= 10:
                print("KHAC [{}] h={:.6g} v={:.6g} a={:.6g} j={:.6g} dt={:g}: {}".format(
                    grp, h, v, acc, j, dt, "; ".join(msgs)))

    # ca bien: h = 0 -> T = 0, 1 mau = 0; gioi han khong hop le -> ok = 0, T = 0
    r0, rv, ra = out[len(cases):]
    if not (r0["ok"] == 1 and r0["T"] == 0.0 and np.all(r0["S"] == 0.0)):
        bad += 1; print("KHAC [bien h=0]: ", r0)
    for nm, r in (("v<=0", rv), ("a<=0", ra)):
        if not (r["ok"] == 0 and r["T"] == 0.0 and np.all(r["S"] == 0.0)):
            bad += 1; print("KHAC [bien {}]: ".format(nm), r)

    groups = {}
    for c in cases:
        groups[c[0]] = groups.get(c[0], 0) + 1
    print("Ca: " + ", ".join("{} {}".format(k, v) for k, v in groups.items()) + ", bien 3")
    print("Truong hop: " + ", ".join("{} {}".format(k, v) for k, v in sorted(regimes.items())))
    print("Sai so lon nhat (tuong doi) theo nhom:")
    for g, w in worst.items():
        print("  {:12s} T <= {:7.2f} s: s/h {:.1e}, v/vlim {:.1e}, a/alim {:.1e}, T {:.1e}".format(
            g, w["Tmax"], w["s"], w["v"], w["a"], w["T"]))
    print("Jerk: {} mau so voi do doc gia toc Ruckig.".format(n_jerk))
    print("So sanh {} ca ({} mau, {} ca lech 1 mau do T sat boi so dt): {} khac biet.".format(
        len(cases) + len(edge), n_samples, n_boundary, bad))
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
