// Bien dich DoubleS1D.h tren PC va lay mau tung bien dang (khong can ESP32).
// Dung boi tools/trajectory/test_doubles_parity.py de so sanh voi Ruckig (eezybot_mk2_sim/trajectories.ruckig_1d).
// stdin : moi dong "h vmax amax jmax dt"
// stdout: moi ca 1 dong "CASE <ok> <T> <n> <vlim> <alim>" roi n dong mau tai t_k = k*dt, k = 1..n,
//         n = ceil(T/dt) (mau cuoi = dich, giong vong update cua Ruckig). Moi phep tinh cua bo sinh o float.
//         Dong mau: "pos vel acc jrk".
#include <cmath>
#include <cstdio>
#include "../../lib/Trajectory/DoubleS1D.h"

int main() {
  float h, v, a, j, dt;
  while (std::scanf("%f %f %f %f %f", &h, &v, &a, &j, &dt) == 5) {
    DoubleS1D p;
    const bool ok = p.plan(h, v, a, j);
    const float T = p.duration();
    long n = (long)std::ceil((double)T / (double)dt);
    if (n < 1) n = 1;
    std::printf("CASE %d %.9g %ld %.9g %.9g\n", ok ? 1 : 0, (double)T, n, (double)p.vlim(), (double)p.alim());
    for (long k = 1; k <= n; k++) {
      const DoubleS1D::Sample s = p.at((float)k * dt);
      std::printf("%.9g %.9g %.9g %.9g\n", (double)s.pos, (double)s.vel, (double)s.acc, (double)s.jrk);
    }
  }
  return 0;
}
