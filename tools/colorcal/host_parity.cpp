// Bien dich ColorMath.h tren PC va phan loai tung dong CSV (khong can ESP32).
// Dung boi tools/colorcal/test_host_parity.py de so sanh voi ban Python.
// stdin: dong 1 = 13 gia tri model (atime gain navg nflush c00 c01 c10 c11 c20 c21 dmax cEmpty cMargin)
//        cac dong sau = "R G B C sat"
// stdout: "<LABEL> <dist>" moi dong.
#include <cstdio>
#include "../../lib/ColorClassifier/ColorMath.h"

int main() {
  colormath::Model m = {};
  unsigned at, g, na, nf;
  if (std::scanf("%u %u %u %u", &at, &g, &na, &nf) != 4) return 1;
  m.magic = colormath::MODEL_MAGIC;
  m.atime = (uint8_t)at; m.gain = (uint8_t)g; m.navg = (uint8_t)na; m.nflush = (uint8_t)nf;
  for (int k = 0; k < colormath::NUM_CLASSES; k++)
    if (std::scanf("%f %f", &m.cent[k][0], &m.cent[k][1]) != 2) return 1;
  if (std::scanf("%f %f %f", &m.dmax, &m.cEmpty, &m.cMargin) != 3) return 1;
  float R, G, B, C;
  int sat;
  while (std::scanf("%f %f %f %f %d", &R, &G, &B, &C, &sat) == 5) {
    colormath::Result r = colormath::classify(m, R, G, B, C, sat != 0);
    std::printf("%s %.7f\n", colormath::labelName(r.label), r.dist);
  }
  return 0;
}
