// DoubleS1D.h - bien dang double-S 1 chieu (jerk bi chan) nghi -> nghi, tinh giai tich bang float
// (khong phu thuoc Arduino).
//
// Dung chung cho ca hai kieu chuyen dong cua tay may (mot bo sinh, hai anh xa):
//   - duong thang : p(t) = p0 + s(t)/L * (p1 - p0), roi IK      (plan(L, V_MAX, A_MAX, J_MAX))
//   - khop        : q(t) = q0 + sigma(t) * (q1 - q0), sigma 0->1 (plan(1, v, a, j) voi gioi han da quy doi)
//
// Thuat toan: L. Biagiotti, C. Melchiorri, "Trajectory Planning for Automatic Machines and Robots",
// Springer 2008, muc 3.4 (double S), truong hop v0 = v1 = 0, a0 = a1 = 0. Khi do bien dang doi xung:
// pha giam toc la anh guong cua pha tang toc (Td = Ta, Tj1 = Tj2 = Tj). Day cung la nghiem toi uu thoi gian
// ma Ruckig tra ve cho bai toan 1 chieu nghi -> nghi; kiem chung tren PC: tools/trajectory/test_doubles_parity.py.
//
//   pha tang toc [0, Ta]  : jerk +J trong Tj, gia toc giu alim, jerk -J trong Tj
//   pha deu      [Ta, Ta+Tv]: van toc vlim
//   pha giam toc [Ta+Tv, T] : guong cua pha tang toc, T = 2 Ta + Tv
//
// Moi phep tinh deu o float (ESP32 co FPU don chinh xac; double chay bang phan mem).
#pragma once
#include <math.h>

class DoubleS1D {
 public:
  struct Sample {
    float pos, vel, acc;
  };

  // Lap ke hoach cho quang duong co dau h (don vi tuy y) voi gioi han vmax, amax, jmax > 0.
  // Tra ve false neu gioi han khong hop le (khi do duration() = 0 va moi mau = 0).
  bool plan(float h, float vmax, float amax, float jmax) {
    *this = DoubleS1D();
    // !(x <= 3.4e38f) dung cho ca NaN va vo cung, khong can isfinite (macro/std:: khac nhau giua cac toolchain)
    if (!(vmax > 0.0f) || !(amax > 0.0f) || !(jmax > 0.0f) || !(fabsf(h) <= 3.4e38f)) return false;
    if (!(vmax <= 3.4e38f) || !(amax <= 3.4e38f) || !(jmax <= 3.4e38f)) return false;
    _sign = h < 0.0f ? -1.0f : 1.0f;
    _h = fabsf(h);
    _j = jmax;
    if (_h == 0.0f) return true;

    // Truong hop 1: dat vmax. Neu vmax*jmax >= amax^2 thi gia toc dat amax, nguoc lai gia toc chi la tam giac.
    float tj, ta;
    if (vmax * jmax >= amax * amax) {
      tj = amax / jmax;
      ta = tj + vmax / amax;
    } else {
      tj = sqrtf(vmax / jmax);
      ta = 2.0f * tj;
    }
    float tv = _h / vmax - ta;
    if (tv < 0.0f) {
      // Truong hop 2: khong dat vmax (Tv = 0). Thu voi gia toc dat amax.
      tv = 0.0f;
      tj = amax / jmax;
      const float delta = amax * amax * amax * amax / (jmax * jmax) + 4.0f * amax * _h;
      ta = (amax * amax / jmax + sqrtf(delta)) / (2.0f * amax);
      if (ta < 2.0f * tj) {
        // Quang qua ngan de dat amax: gia toc tam giac, h = 2 J Tj^3.
        tj = cbrtf(_h / (2.0f * jmax));
        ta = 2.0f * tj;
      }
    }
    _tj = tj;
    _ta = ta;
    _tv = tv;
    _alim = jmax * tj;
    _vlim = (ta - tj) * _alim;
    _T = 2.0f * ta + tv;
    return true;
  }

  float duration() const { return _T; }   // s
  float vlim() const { return _sign * _vlim; }
  float alim() const { return _sign * _alim; }
  float distance() const { return _sign * _h; }

  // Mau tai thoi diem t (s). t < 0 -> trang thai dau, t > T -> trang thai cuoi.
  Sample at(float t) const {
    Sample s = {0.0f, 0.0f, 0.0f};
    if (_T <= 0.0f) return s;
    if (t >= _T) {
      s.pos = _sign * _h;
      return s;
    }
    if (t <= 0.0f) return s;
    if (t < _ta) {
      s = accelPhase(t);
    } else if (t < _ta + _tv) {
      s.pos = 0.5f * _vlim * _ta + _vlim * (t - _ta);
      s.vel = _vlim;
      s.acc = 0.0f;
    } else {
      // pha giam toc = guong cua pha tang toc qua thoi diem T
      const Sample m = accelPhase(_T - t);
      s.pos = _h - m.pos;
      s.vel = m.vel;
      s.acc = -m.acc;
    }
    s.pos *= _sign;
    s.vel *= _sign;
    s.acc *= _sign;
    return s;
  }

  float pos(float t) const { return at(t).pos; }

 private:
  // Pha tang toc, 0 <= t <= Ta, quang duong khong dau.
  Sample accelPhase(float t) const {
    Sample s;
    if (t < _tj) {                       // jerk +J
      s.pos = _j * t * t * t / 6.0f;
      s.vel = 0.5f * _j * t * t;
      s.acc = _j * t;
    } else if (t < _ta - _tj) {          // gia toc khong doi alim
      s.pos = _alim / 6.0f * (3.0f * t * t - 3.0f * _tj * t + _tj * _tj);
      s.vel = _alim * (t - 0.5f * _tj);
      s.acc = _alim;
    } else {                             // jerk -J
      const float r = _ta - t;
      s.pos = 0.5f * _vlim * _ta - _vlim * r + _j * r * r * r / 6.0f;
      s.vel = _vlim - 0.5f * _j * r * r;
      s.acc = _j * r;
    }
    return s;
  }

  float _h = 0.0f, _sign = 1.0f, _j = 0.0f;
  float _tj = 0.0f, _ta = 0.0f, _tv = 0.0f, _T = 0.0f;
  float _alim = 0.0f, _vlim = 0.0f;
};
