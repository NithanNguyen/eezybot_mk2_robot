// Bien dich Mk2Kinematics.h tren PC (khong can ESP32). Dung boi tools/kinematics/test_kinematics_parity.py
// de so sanh voi eezybot_mk2_sim/mk2_kinematics.py. Moi phep tinh dong hoc o float.
// stdin, moi dong mot lenh (tool: 0 = gripper, 1 = cap):
//   FK  tool th1 th2 phi      -> "x y z"
//   IK  tool x y z            -> "<STATUS> th1 th2 phi"   (goc = 0 khi NEAR_AXIS / OUT_OF_REACH)
//   LIM th1 th2 phi           -> "<STATUS>"
//   SV  th1 th2 phi           -> "s0 s1 s2"   (goc servo, do)
//   FS  s0 s1 s2              -> "th1 th2 phi"
//   JAC tool th1 th2 phi      -> 9 so, theo hang
#include <cstdio>
#include <cstring>
#include <math.h>
#include "../host_compat/arduino_macro_trap.h"  // sau header he thong, truoc lib/: bat ten trung macro Arduino-ESP32
#include "../../lib/Kinematics/Mk2Kinematics.h"

using namespace mk2kin;

static const Tool& tool(int i) { return i ? TOOL_CAP : TOOL_GRIPPER; }

int main() {
  char cmd[8];
  while (std::scanf("%7s", cmd) == 1) {
    int ti = 0;
    float a, b, c;
    if (!std::strcmp(cmd, "FK") && std::scanf("%d %f %f %f", &ti, &a, &b, &c) == 4) {
      const Joint q = {a, b, c};
      const Vec3 p = fk(q, tool(ti));
      std::printf("%.9g %.9g %.9g\n", (double)p.x, (double)p.y, (double)p.z);
    } else if (!std::strcmp(cmd, "IK") && std::scanf("%d %f %f %f", &ti, &a, &b, &c) == 4) {
      const Vec3 p = {a, b, c};
      Joint q = {0.0f, 0.0f, 0.0f};
      const Status s = ik(p, tool(ti), q);
      std::printf("%s %.9g %.9g %.9g\n", statusName(s), (double)q.th1, (double)q.th2, (double)q.phi);
    } else if (!std::strcmp(cmd, "LIM") && std::scanf("%f %f %f", &a, &b, &c) == 3) {
      const Joint q = {a, b, c};
      std::printf("%s\n", statusName(withinLimits(q)));
    } else if (!std::strcmp(cmd, "SV") && std::scanf("%f %f %f", &a, &b, &c) == 3) {
      const Joint q = {a, b, c};
      float s[3];
      toServo(q, s);
      std::printf("%.9g %.9g %.9g\n", (double)s[0], (double)s[1], (double)s[2]);
    } else if (!std::strcmp(cmd, "FS") && std::scanf("%f %f %f", &a, &b, &c) == 3) {
      const float s[3] = {a, b, c};
      const Joint q = fromServo(s);
      std::printf("%.9g %.9g %.9g\n", (double)q.th1, (double)q.th2, (double)q.phi);
    } else if (!std::strcmp(cmd, "JAC") && std::scanf("%d %f %f %f", &ti, &a, &b, &c) == 4) {
      const Joint q = {a, b, c};
      float J[3][3];
      jacobian(q, tool(ti), J);
      for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) std::printf("%.9g%c", (double)J[i][j], (i == 2 && j == 2) ? '\n' : ' ');
    } else {
      std::fprintf(stderr, "lenh khong hop le: %s\n", cmd);
      return 2;
    }
  }
  return 0;
}
