// ArmMotion.h - lap ke hoach va tuan tu chuyen dong cua chu trinh phan loai, tinh bang float
// (khong phu thuoc Arduino, khong cap phat dong).
//
// Ban C++ cua phan chuyen dong trong eezybot_mk2_sim/run_sort.py: cung hai kieu doan, cung thu tu buoc, cung so
// nhip 20 ms. Kiem chung tren PC tung nhip lenh voi results/sort/log.csv: tools/motion/test_motion_parity.py.
//
//   JointMove : q(t) = q0 + sigma(t) (q1 - q0), sigma = DoubleS1D 0 -> 1 voi gioi han khop JOINT_*_MAX / max|dq|
//               (= Ruckig Synchronization.Phase voi cung gioi han cho 3 khop). Sau do tinh GIAI TICH dinh v, a, j
//               cua dau cong tac doc duong di va gian thoi gian deu k = max(1, v/V, sqrt(a/A), cbrt(j/J))
//               (run_sort.joint_move tinh dinh bang dao ham so; hai cach lech k <= 3e-5).
//   LineMove  : p(t) = p0 + s(t)/L (p1 - p0), s = DoubleS1D 0 -> L voi V_MAX, A_MAX, J_MAX, roi IK.
//   Mau thu k (k = 1..n) lay tai t = k * dt, n = max(1, ceil(T / dt)): mau cuoi = dich (giong vong Ruckig).
#pragma once
#include <math.h>
#include <stdint.h>
#include "../../include/arm_config.h"
#include "../Kinematics/Mk2Kinematics.h"
#include "../Trajectory/DoubleS1D.h"

namespace motion {

using mk2kin::Joint;
using mk2kin::Status;
using mk2kin::Tool;
using mk2kin::Vec3;

static const float DT = armcfg::CONTROL_PERIOD_S;

// So nhip cua mot bien dang thoi gian T.
inline int ticksFor(float T) {
  const int n = (int)ceilf(T / DT);
  return n < 1 ? 1 : n;
}

// So nhip cua mot lan giu lenh (run_sort.hold): max(1, round(seconds / dt)).
inline int holdTicks(float seconds) {
  const int n = (int)lroundf(seconds / DT);
  return n < 1 ? 1 : n;
}

// ---------------------------------------------------------------------------------------------- JointMove
struct ToolPeaks {
  float v, a, j;  // dinh van toc, gia toc, jerk cua dau cong tac (mm/s, mm/s^2, mm/s^3) khi k = 1
  float k;        // he so gian thoi gian da ap dung
};

class JointMove {
 public:
  // q0 == q1 van hop le: 1 nhip tai dich (giong Ruckig).
  void plan(const Joint& q0, const Joint& q1, const Tool& tool) {
    _q0 = q0;
    _dq.th1 = q1.th1 - q0.th1;
    _dq.th2 = q1.th2 - q0.th2;
    _dq.phi = q1.phi - q0.phi;
    _q1 = q1;
    const float m = fmaxf(fabsf(_dq.th1), fmaxf(fabsf(_dq.th2), fabsf(_dq.phi)));
    _peaks.v = _peaks.a = _peaks.j = 0.0f;
    _peaks.k = 1.0f;
    if (m <= 0.0f) {
      _prof.plan(0.0f, 1.0f, 1.0f, 1.0f);
      _n = 1;
      return;
    }
    const float v = armcfg::JOINT_V_MAX / m, a = armcfg::JOINT_A_MAX / m, j = armcfg::JOINT_J_MAX / m;
    _prof.plan(1.0f, v, a, j);
    toolPeaks(tool);
    const float k = fmaxf(1.0f, fmaxf(_peaks.v / armcfg::V_MAX,
                                      fmaxf(sqrtf(_peaks.a / armcfg::A_MAX), cbrtf(_peaks.j / armcfg::J_MAX))));
    _peaks.k = k;
    if (k > 1.0f) _prof.plan(1.0f, v / k, a / (k * k), j / (k * k * k));
    _n = ticksFor(_prof.duration());
  }

  int ticks() const { return _n; }
  float duration() const { return _prof.duration(); }
  const ToolPeaks& peaks() const { return _peaks; }

  // Mau thu i (1..n).
  Joint at(int i) const {
    if (i >= _n) return _q1;
    const float s = _prof.at((float)i * DT).pos;
    Joint q = {_q0.th1 + s * _dq.th1, _q0.th2 + s * _dq.th2, _q0.phi + s * _dq.phi};
    return q;
  }

 private:
  // Dinh v, a, j cua dau cong tac tren duong q0 + sigma dq (bien dang hien tai, k = 1).
  // p(sigma) quay quanh truc dung goc th1 nen viet trong he quay: w = (r + i dy) e^{i th1}, z(sigma).
  // Dao ham theo sigma (dq hang so):
  //   r'   = -L2 sin(th2) d2 - L3 sin(phi) d3          z'   =  L2 cos(th2) d2 + L3 cos(phi) d3
  //   r''  = -L2 cos(th2) d2^2 - L3 cos(phi) d3^2      z''  = -L2 sin(th2) d2^2 - L3 sin(phi) d3^2
  //   r''' =  L2 sin(th2) d2^3 + L3 sin(phi) d3^3      z''' = -L2 cos(th2) d2^3 - L3 cos(phi) d3^3
  //   u = r + i dy:  W1 = u' + i d1 u,  W2 = u'' + 2i d1 u' - d1^2 u,  W3 = u''' + 3i d1 u'' - 3 d1^2 u' - i d1^3 u
  //   (|e^{i th1}| = 1 nen chuan cua vector khong doi trong he quay). Theo thoi gian (sigma', sigma'', sigma'''):
  //   v = P1 s',  a = P2 s'^2 + P1 s'',  j = P3 s'^3 + 3 P2 s' s'' + P1 s'''.
  // Lay mau luoi 1000 doan + hai ben moi moc pha (jerk gian doan tai do).
  void toolPeaks(const Tool& tool) {
    const float T = _prof.duration();
    const float bnd[6] = {_prof.tj(), _prof.ta() - _prof.tj(), _prof.ta(), _prof.ta() + _prof.tv(),
                          T - _prof.ta() + _prof.tj(), T - _prof.tj()};
    const int NG = 1000;
    for (int i = 0; i <= NG + 12; i++) {
      float t;
      if (i <= NG) {
        t = T * (float)i / (float)NG;
      } else {
        const int b = (i - NG - 1) / 2;
        t = bnd[b] + (((i - NG - 1) & 1) ? 1e-6f : -1e-6f) * T;
        if (t < 0.0f) t = 0.0f;
        if (t > T) t = T;
      }
      evalPeak(t, tool);
    }
  }

  void evalPeak(float t, const Tool& tool) {
    using namespace armcfg;
    const DoubleS1D::Sample s = _prof.at(t);
    const float th2 = _q0.th2 + s.pos * _dq.th2, phi = _q0.phi + s.pos * _dq.phi;
    const float d1 = _dq.th1, d2 = _dq.th2, d3 = _dq.phi;
    const float c2 = cosf(th2), s2 = sinf(th2), c3 = cosf(phi), s3 = sinf(phi);
    const float r = SHOULDER_OFFSET_X + L_MAIN_ARM * c2 + L_HORIZ_ARM * c3 + tool.dx, dy = tool.dy;
    const float r1 = -L_MAIN_ARM * s2 * d2 - L_HORIZ_ARM * s3 * d3;
    const float r2 = -L_MAIN_ARM * c2 * d2 * d2 - L_HORIZ_ARM * c3 * d3 * d3;
    const float r3 = L_MAIN_ARM * s2 * d2 * d2 * d2 + L_HORIZ_ARM * s3 * d3 * d3 * d3;
    const float z1 = L_MAIN_ARM * c2 * d2 + L_HORIZ_ARM * c3 * d3;
    const float z2 = -L_MAIN_ARM * s2 * d2 * d2 - L_HORIZ_ARM * s3 * d3 * d3;
    const float z3 = -L_MAIN_ARM * c2 * d2 * d2 * d2 - L_HORIZ_ARM * c3 * d3 * d3 * d3;
    // u = r + i dy; u', u'', u''' la so thuc (dy hang so)
    const float W1x = r1 - d1 * dy, W1y = d1 * r;
    const float W2x = r2 - d1 * d1 * r, W2y = 2.0f * d1 * r1 - d1 * d1 * dy;
    const float W3x = r3 - 3.0f * d1 * d1 * r1 + d1 * d1 * d1 * dy, W3y = 3.0f * d1 * r2 - d1 * d1 * d1 * r;
    const float sv = s.vel, sa = s.acc, sj = s.jrk;
    const float vx = W1x * sv, vy = W1y * sv, vz = z1 * sv;
    const float ax = W2x * sv * sv + W1x * sa, ay = W2y * sv * sv + W1y * sa, az = z2 * sv * sv + z1 * sa;
    const float sv3 = sv * sv * sv, k3 = 3.0f * sv * sa;
    const float jx = W3x * sv3 + k3 * W2x + W1x * sj, jy = W3y * sv3 + k3 * W2y + W1y * sj,
                jz = z3 * sv3 + k3 * z2 + z1 * sj;
    _peaks.v = fmaxf(_peaks.v, sqrtf(vx * vx + vy * vy + vz * vz));
    _peaks.a = fmaxf(_peaks.a, sqrtf(ax * ax + ay * ay + az * az));
    _peaks.j = fmaxf(_peaks.j, sqrtf(jx * jx + jy * jy + jz * jz));
  }

  Joint _q0 = {0, 0, 0}, _q1 = {0, 0, 0}, _dq = {0, 0, 0};
  DoubleS1D _prof;
  ToolPeaks _peaks = {0, 0, 0, 1};
  int _n = 1;
};

// ---------------------------------------------------------------------------------------------- LineMove
class LineMove {
 public:
  void plan(const Vec3& p0, const Vec3& p1, const Tool& tool) {
    _p0 = p0;
    _d.x = p1.x - p0.x;
    _d.y = p1.y - p0.y;
    _d.z = p1.z - p0.z;
    _tool = tool;
    _L = sqrtf(_d.x * _d.x + _d.y * _d.y + _d.z * _d.z);
    _prof.plan(_L, armcfg::V_MAX, armcfg::A_MAX, armcfg::J_MAX);
    _n = ticksFor(_prof.duration());
  }

  int ticks() const { return _n; }
  float duration() const { return _prof.duration(); }

  // Mau thu i (1..n): IK cua diem tren doan thang. Tra ve trang thai IK (kem kiem tra gioi han).
  Status at(int i, Joint& q) const {
    const float u = (_L > 0.0f) ? _prof.at((float)i * DT).pos / _L : 1.0f;
    const Vec3 p = {_p0.x + _d.x * u, _p0.y + _d.y * u, _p0.z + _d.z * u};
    return mk2kin::ik(p, _tool, q);
  }

 private:
  Vec3 _p0 = {0, 0, 0}, _d = {0, 0, 0};
  Tool _tool = {0, 0, 0};
  float _L = 0.0f;
  DoubleS1D _prof;
  int _n = 1;
};

// ---------------------------------------------------------------------------------------------- Sequencer
// Ten pha trung run_sort.py (cot "pha" cua log.csv) de so sanh truc tiep.
enum Phase : uint8_t {
  PH_IDLE = 0,
  PH_TO_STATION,       // "toi tram"
  PH_DESCEND,          // "ha xuong"
  PH_GRIP_CLOSE,       // "dong kep"
  PH_LIFT,             // "nhac len"
  PH_TO_BIN1,          // "toi ngan 1" (do)
  PH_TO_BIN2,          // "toi ngan 2" (xanh la)
  PH_TO_BIN3,          // "toi ngan 3" (trang)
  PH_TO_REJECT,        // "toi vung vut bo"
  PH_DESCEND_REJECT,   // "ha xuong vung vut bo"
  PH_GRIP_OPEN,        // "mo kep"
  PH_HOME,             // "ve Home"
};

inline const char* phaseName(Phase p) {
  switch (p) {
    case PH_IDLE: return "nghi";
    case PH_TO_STATION: return "toi tram";
    case PH_DESCEND: return "ha xuong";
    case PH_GRIP_CLOSE: return "dong kep";
    case PH_LIFT: return "nhac len";
    case PH_TO_BIN1: return "toi ngan 1";
    case PH_TO_BIN2: return "toi ngan 2";
    case PH_TO_BIN3: return "toi ngan 3";
    case PH_TO_REJECT: return "toi vung vut bo";
    case PH_DESCEND_REJECT: return "ha xuong vung vut bo";
    case PH_GRIP_OPEN: return "mo kep";
    case PH_HOME: return "ve Home";
  }
  return "?";
}

enum class Dest : uint8_t { RED = 0, GREEN = 1, WHITE = 2, REJECT = 3 };

// Lenh cua mot nhip 20 ms.
struct Command {
  Joint q;            // goc khop (rad)
  float servoDeg[3];  // goc lenh 3 servo MG996R (do), mk2kin::toServo
  float gripDeg;      // goc lenh kep tinh tu luc dong (do) = GRIP_*_DEG * GRIPPER_GEAR_RATIO
  Phase phase;
};

inline Joint homeJoint() {
  const Joint q = {armcfg::HOME_THETA1 * mk2kin::DEG2RAD, armcfg::HOME_THETA2 * mk2kin::DEG2RAD,
                   armcfg::HOME_PHI * mk2kin::DEG2RAD};
  return q;
}
inline float gripOpenDeg() { return armcfg::GRIP_OPEN_DEG * armcfg::GRIPPER_GEAR_RATIO; }
inline float gripHoldDeg() { return armcfg::GRIP_HOLD_DEG * armcfg::GRIPPER_GEAR_RATIO; }

class Sequencer {
 public:
  // Trang thai khoi dong: tay o q, kep o gripDeg (khong phat lenh).
  void reset(const Joint& q, float gripDeg) {
    _q = q;
    _grip = gripDeg;
    _nSteps = _cur = 0;
    _active = false;
    _err = mk2kin::OK;
  }

  bool active() const { return _active; }
  Status error() const { return _err; }
  const Joint& joint() const { return _q; }
  float grip() const { return _grip; }
  // Dinh dau cong tac va he so k cua doan khop gan nhat (de in ra / kiem thu).
  const ToolPeaks& lastPeaks() const { return _joint.peaks(); }
  float lastDuration() const { return _lastT; }

  // Chu trinh gap tai tram -> tha -> ve Home (run_sort.py, buoc 4-6). Tra ve false neu dang chay.
  bool startSort(Dest d) {
    if (_active) return false;
    using namespace armcfg;
    _nSteps = _cur = 0;
    pushJointToPoint(P_PRE_GRASP, PH_TO_STATION);
    pushLine(P_PRE_GRASP, P_GRASP, PH_DESCEND);
    pushHold(0.1f, -1.0f, PH_DESCEND);
    pushHold(GRIP_MOVE_S, gripHoldDeg(), PH_GRIP_CLOSE);
    pushLine(P_GRASP, P_PRE_GRASP, PH_LIFT);
    if (d == Dest::REJECT) {
      pushJointToPoint(P_REJECT_PRE, PH_TO_REJECT);
      pushLine(P_REJECT_PRE, P_DROP_REJECT, PH_DESCEND_REJECT);
    } else if (d == Dest::RED) {
      pushJointToPoint(P_DROP_RED, PH_TO_BIN1);
    } else if (d == Dest::GREEN) {
      pushJointToPoint(P_DROP_GREEN, PH_TO_BIN2);
    } else {
      pushJointToPoint(P_DROP_WHITE, PH_TO_BIN3);
    }
    pushHold(DROP_WAIT_S, gripOpenDeg(), PH_GRIP_OPEN);
    if (d == Dest::REJECT) pushLine(P_DROP_REJECT, P_REJECT_PRE, PH_LIFT);
    pushJointToQ(homeJoint(), PH_HOME);
    return begin();
  }

  // Ve Home tu tu the hien tai roi mo kep (khoi dong: run_sort "khoi dong" + "mo kep").
  bool startHome() {
    if (_active) return false;
    _nSteps = _cur = 0;
    pushJointToQ(homeJoint(), PH_HOME);
    pushHold(armcfg::GRIP_MOVE_S, gripOpenDeg(), PH_GRIP_OPEN);
    return begin();
  }

  // Mot nhip: ghi lenh vao out va tra ve true neu con dang chay o nhip nay.
  // Loi IK / gioi han -> dung chu trinh, giu tu the cuoi, error() != OK.
  bool tick(Command& out) {
    if (!_active) {
      fill(out, PH_IDLE);
      return false;
    }
    Step& st = _steps[_cur];
    if (_i == 0) {
      if (!beginStep(st)) {
        _active = false;
        fill(out, PH_IDLE);
        return false;
      }
    }
    _i++;
    if (st.kind == S_JOINT) {
      const Joint q = _joint.at(_i);
      const Status s = mk2kin::withinLimits(q);
      if (s != mk2kin::OK) return fail(s, out);
      _q = q;
    } else if (st.kind == S_LINE) {
      Joint q;
      const Status s = _line.at(_i, q);
      if (s != mk2kin::OK) return fail(s, out);
      _q = q;
    }
    fill(out, st.phase);
    if (_i >= _n) {
      _i = 0;
      if (++_cur >= _nSteps) _active = false;
    }
    return true;
  }

 private:
  enum Kind : uint8_t { S_JOINT, S_LINE, S_HOLD };
  struct Step {
    Kind kind;
    Phase phase;
    bool toPoint;  // S_JOINT: dich la diem (IK luc bat dau buoc) hay goc khop
    Vec3 a, b;     // S_LINE: a -> b; S_JOINT + toPoint: dich a
    Joint q;       // S_JOINT + !toPoint: dich
    float holdS;   // S_HOLD
    float gripDeg; // S_HOLD: >= 0 thi dat lenh kep moi khi bat dau buoc
  };
  static const int MAX_STEPS = 12;

  void pushJointToPoint(const armcfg::Point3& p, Phase ph) {
    Step& s = next(S_JOINT, ph);
    s.toPoint = true;
    s.a = toVec(p);
  }
  void pushJointToQ(const Joint& q, Phase ph) {
    Step& s = next(S_JOINT, ph);
    s.toPoint = false;
    s.q = q;
  }
  void pushLine(const armcfg::Point3& a, const armcfg::Point3& b, Phase ph) {
    Step& s = next(S_LINE, ph);
    s.a = toVec(a);
    s.b = toVec(b);
  }
  void pushHold(float seconds, float gripDeg, Phase ph) {
    Step& s = next(S_HOLD, ph);
    s.holdS = seconds;
    s.gripDeg = gripDeg;
  }
  Step& next(Kind k, Phase ph) {
    Step& s = _steps[_nSteps < MAX_STEPS ? _nSteps++ : MAX_STEPS - 1];
    s = Step();
    s.kind = k;
    s.phase = ph;
    s.gripDeg = -1.0f;
    return s;
  }
  static Vec3 toVec(const armcfg::Point3& p) {
    const Vec3 v = {p.x, p.y, p.z};
    return v;
  }

  bool begin() {
    _cur = 0;
    _i = 0;
    _err = mk2kin::OK;
    _active = _nSteps > 0;
    return _active;
  }

  // Lap ke hoach cho buoc hien tai. False neu dich khong giai duoc IK.
  bool beginStep(const Step& st) {
    if (st.kind == S_JOINT) {
      Joint target = st.q;
      if (st.toPoint) {
        const Status s = mk2kin::ik(st.a, mk2kin::TOOL_CAP, target);
        if (s != mk2kin::OK) {
          _err = s;
          return false;
        }
      }
      _joint.plan(_q, target, mk2kin::TOOL_CAP);
      _n = _joint.ticks();
      _lastT = _joint.duration();
    } else if (st.kind == S_LINE) {
      _line.plan(st.a, st.b, mk2kin::TOOL_CAP);
      _n = _line.ticks();
      _lastT = _line.duration();
    } else {
      if (st.gripDeg >= 0.0f) _grip = st.gripDeg;
      _n = holdTicks(st.holdS);
      _lastT = (float)_n * DT;
    }
    return true;
  }

  bool fail(Status s, Command& out) {
    _err = s;
    _active = false;
    fill(out, PH_IDLE);
    return false;
  }

  void fill(Command& out, Phase ph) const {
    out.q = _q;
    mk2kin::toServo(_q, out.servoDeg);
    out.gripDeg = _grip;
    out.phase = ph;
  }

  Step _steps[MAX_STEPS];
  int _nSteps = 0, _cur = 0, _i = 0, _n = 0;
  bool _active = false;
  Status _err = mk2kin::OK;
  Joint _q = {0, 0, 0};
  float _grip = 0.0f, _lastT = 0.0f;
  JointMove _joint;
  LineMove _line;
};

}  // namespace motion
