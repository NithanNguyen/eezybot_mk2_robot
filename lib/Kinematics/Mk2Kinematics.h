// Mk2Kinematics.h - dong hoc thuan/nguoc giai tich, kiem tra gioi han, Jacobian va anh xa goc servo cua
// EEZYbotARM Mk2, tinh bang float (khong phu thuoc Arduino).
//
// Ban C++ cua eezybot_mk2_sim/mk2_kinematics.py (cung cong thuc, cung thu tu kiem tra gioi han). Moi tham so lay
// tu include/arm_config.h, file nay SINH TU params.py nen firmware va mo phong dung chung mot bo so.
// Kiem chung tren PC: tools/kinematics/test_kinematics_parity.py.
//
// Bien khop (rad):
//   th1 : goc de quanh truc dung (0 = huong +x)
//   th2 : goc TUYET DOI cua canh tay chinh O->E so voi phuong ngang
//   phi : goc TUYET DOI cua tay ngang E->W so voi phuong ngang (servo khuyu dat truc tiep phi nho hinh binh hanh)
// Dau cong tac giu huong co dinh nen vi tri p = (x, y, z) (mm) chi phu thuoc (th1, th2, phi).
#pragma once
#include <math.h>
#include "../../include/arm_config.h"

namespace mk2kin {

struct Vec3 {
  float x, y, z;
};

struct Joint {
  float th1, th2, phi;  // rad
};

// Dau cong tac so voi tam co tay W: (dx, dz) trong mat phang canh tay, dy lech ngang.
struct Tool {
  float dx, dz, dy;
};
static const Tool TOOL_GRIPPER = {armcfg::TOOL_GRIPPER_DX, armcfg::TOOL_GRIPPER_DZ, armcfg::TOOL_DY};
static const Tool TOOL_CAP = {armcfg::TOOL_CAP_DX, armcfg::TOOL_CAP_DZ, armcfg::TOOL_DY};  // tam nap khi kep

// Ket qua IK / kiem tra gioi han. Thu tu kiem tra giong mk2_kinematics.within_limits().
enum Status : int {
  OK = 0,
  NEAR_AXIS,          // x^2 + y^2 <= DY^2: diem qua gan truc de
  OUT_OF_REACH,       // |cos(gamma)| > 1: ngoai tam voi
  LIM_THETA1,         // th1 ngoai [THETA1_MIN, THETA1_MAX]
  LIM_THETA2,         // th2 ngoai [THETA2_MIN, THETA2_MAX]
  LIM_ELBOW,          // phi - th2 ngoai [ELBOW_MIN, ELBOW_MAX]
  LIM_PHI,            // phi ngoai [PHI_MIN, PHI_MAX]
  LIM_CTRL_BASE,      // BASE_GEAR_RATIO * th1 ngoai tam lenh de
  LIM_CTRL_SHOULDER,  // th2 - 90 ngoai tam lenh vai
  LIM_CTRL_ELBOW,     // phi ngoai tam lenh khuyu
  LIM_SERVO0,         // goc servo ngoai [SERVO_MIN_DEG, SERVO_MAX_DEG]
  LIM_SERVO1,
  LIM_SERVO2,
};

inline const char* statusName(Status s) {
  switch (s) {
    case OK: return "OK";
    case NEAR_AXIS: return "NEAR_AXIS";
    case OUT_OF_REACH: return "OUT_OF_REACH";
    case LIM_THETA1: return "LIM_THETA1";
    case LIM_THETA2: return "LIM_THETA2";
    case LIM_ELBOW: return "LIM_ELBOW";
    case LIM_PHI: return "LIM_PHI";
    case LIM_CTRL_BASE: return "LIM_CTRL_BASE";
    case LIM_CTRL_SHOULDER: return "LIM_CTRL_SHOULDER";
    case LIM_CTRL_ELBOW: return "LIM_CTRL_ELBOW";
    case LIM_SERVO0: return "LIM_SERVO0";
    case LIM_SERVO1: return "LIM_SERVO1";
    case LIM_SERVO2: return "LIM_SERVO2";
  }
  return "?";
}

static const float RAD2DEG = 57.29577951f;
static const float DEG2RAD = 0.01745329252f;

// ---------------------------------------------------------------- FK
// Vi tri dau cong tac (mm).
inline Vec3 fk(const Joint& q, const Tool& t) {
  using namespace armcfg;
  const float r = SHOULDER_OFFSET_X + L_MAIN_ARM * cosf(q.th2) + L_HORIZ_ARM * cosf(q.phi) + t.dx;
  const float z = SHOULDER_HEIGHT + L_MAIN_ARM * sinf(q.th2) + L_HORIZ_ARM * sinf(q.phi) + t.dz;
  const float c1 = cosf(q.th1), s1 = sinf(q.th1);
  Vec3 p = {r * c1 - t.dy * s1, r * s1 + t.dy * c1, z};
  return p;
}

// ---------------------------------------------------------------- anh xa servo
// (th1, th2, phi) rad -> goc lenh servo (do): servo = OFFSET + SIGN * (goc - goc_home), de qua ti so truyen.
inline void toServo(const Joint& q, float deg[3]) {
  using namespace armcfg;
  deg[0] = SERVO_OFFSET_DEG[0] + SERVO_SIGN[0] * BASE_GEAR_RATIO * (q.th1 * RAD2DEG);
  deg[1] = SERVO_OFFSET_DEG[1] + SERVO_SIGN[1] * (q.th2 * RAD2DEG - 90.0f);
  deg[2] = SERVO_OFFSET_DEG[2] + SERVO_SIGN[2] * (q.phi * RAD2DEG);
}

inline Joint fromServo(const float deg[3]) {
  using namespace armcfg;
  Joint q;
  q.th1 = (deg[0] - SERVO_OFFSET_DEG[0]) / (SERVO_SIGN[0] * BASE_GEAR_RATIO) * DEG2RAD;
  q.th2 = (90.0f + (deg[1] - SERVO_OFFSET_DEG[1]) / SERVO_SIGN[1]) * DEG2RAD;
  q.phi = (deg[2] - SERVO_OFFSET_DEG[2]) / SERVO_SIGN[2] * DEG2RAD;
  return q;
}

// ---------------------------------------------------------------- gioi han
inline Status withinLimits(const Joint& q) {
  using namespace armcfg;
  const float th1 = q.th1 * RAD2DEG, th2 = q.th2 * RAD2DEG, phi = q.phi * RAD2DEG;
  const float rel = phi - th2;
  if (!(THETA1_MIN <= th1 && th1 <= THETA1_MAX)) return LIM_THETA1;
  if (!(THETA2_MIN <= th2 && th2 <= THETA2_MAX)) return LIM_THETA2;
  if (!(ELBOW_MIN <= rel && rel <= ELBOW_MAX)) return LIM_ELBOW;
  if (!(PHI_MIN <= phi && phi <= PHI_MAX)) return LIM_PHI;
  const float base = BASE_GEAR_RATIO * th1, sh = th2 - 90.0f;
  if (!(CTRL_BASE_MIN <= base && base <= CTRL_BASE_MAX)) return LIM_CTRL_BASE;
  if (!(CTRL_SHOULDER_MIN <= sh && sh <= CTRL_SHOULDER_MAX)) return LIM_CTRL_SHOULDER;
  if (!(CTRL_ELBOW_MIN <= phi && phi <= CTRL_ELBOW_MAX)) return LIM_CTRL_ELBOW;
  float s[3];
  toServo(q, s);
  for (int i = 0; i < 3; i++)
    if (!(SERVO_MIN_DEG <= s[i] && s[i] <= SERVO_MAX_DEG)) return (Status)(LIM_SERVO0 + i);
  return OK;
}

// ---------------------------------------------------------------- IK
// IK giai tich, chon nghiem khuyu-tren (phi - th2 < 0), dung cau hinh that cua Mk2. q chi duoc ghi khi
// ket qua hinh hoc ton tai (moi trang thai tru NEAR_AXIS, OUT_OF_REACH), ke ca khi vi pham gioi han.
inline Status ik(const Vec3& p, const Tool& t, Joint& q, bool checkLimits = true) {
  using namespace armcfg;
  const float rho2 = p.x * p.x + p.y * p.y;
  if (rho2 <= t.dy * t.dy) return NEAR_AXIS;
  const float rr = sqrtf(rho2 - t.dy * t.dy);          // khoang cach trong mat phang canh tay
  const float th1 = atan2f(p.y, p.x) - atan2f(t.dy, rr);
  const float r = rr - t.dx - SHOULDER_OFFSET_X;
  const float zz = p.z - SHOULDER_HEIGHT - t.dz;
  const float d2 = r * r + zz * zz;
  const float c = (d2 - L_MAIN_ARM * L_MAIN_ARM - L_HORIZ_ARM * L_HORIZ_ARM) / (2.0f * L_MAIN_ARM * L_HORIZ_ARM);
  if (c > 1.0f || c < -1.0f) return OUT_OF_REACH;
  const float rel = -acosf(c);                           // phi - th2 < 0: khuyu tren
  const float th2 = atan2f(zz, r) - atan2f(L_HORIZ_ARM * sinf(rel), L_MAIN_ARM + L_HORIZ_ARM * cosf(rel));
  q.th1 = th1;
  q.th2 = th2;
  q.phi = th2 + rel;
  return checkLimits ? withinLimits(q) : OK;
}

// ---------------------------------------------------------------- Jacobian
// J = dp/dq (mm/rad), hang = (x, y, z), cot = (th1, th2, phi). Dung de doi van toc khop -> van toc dau cong tac.
inline void jacobian(const Joint& q, const Tool& t, float J[3][3]) {
  using namespace armcfg;
  const float r = SHOULDER_OFFSET_X + L_MAIN_ARM * cosf(q.th2) + L_HORIZ_ARM * cosf(q.phi) + t.dx;
  const float c1 = cosf(q.th1), s1 = sinf(q.th1);
  const float dr2 = -L_MAIN_ARM * sinf(q.th2), dr3 = -L_HORIZ_ARM * sinf(q.phi);
  const float dz2 = L_MAIN_ARM * cosf(q.th2), dz3 = L_HORIZ_ARM * cosf(q.phi);
  J[0][0] = -r * s1 - t.dy * c1; J[0][1] = dr2 * c1; J[0][2] = dr3 * c1;
  J[1][0] = r * c1 - t.dy * s1;  J[1][1] = dr2 * s1; J[1][2] = dr3 * s1;
  J[2][0] = 0.0f;                J[2][1] = dz2;      J[2][2] = dz3;
}

}  // namespace mk2kin
