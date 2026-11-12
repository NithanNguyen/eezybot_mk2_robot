// Bien dich ArmMotion.h tren PC va chay chu trinh (khong can ESP32). Dung boi tools/motion/test_motion_parity.py
// de so tung nhip lenh voi eezybot_mk2_sim/results/sort/log.csv.
// Trang thai dau: Home, kep mo (giong run_sort.py sau buoc "khoi dong").
// stdin : moi dong "SORT <RED|GREEN|WHITE|REJECT>" hoac "HOME"
// stdout: moi nhip "CMD,<nhip>,<pha>,<s0>,<s1>,<s2>,<kep>" (do), cuoi moi lenh
//         "ARM DONE <ten> status=<STATUS> ticks=<n> cmax_us=<us>"   (giong dinh dang firmware in ra)
#include <chrono>
#include <cstdio>
#include <cstring>
#include <math.h>
#include <stdint.h>
#include "../host_compat/arduino_macro_trap.h"  // sau header he thong, truoc lib/: bat ten trung macro Arduino-ESP32
#include "../../lib/Motion/ArmMotion.h"

int main() {
  motion::Sequencer seq;
  seq.reset(motion::homeJoint(), motion::gripOpenDeg());
  char what[16];
  long tick = 0;
  while (std::scanf("%15s", what) == 1) {
    bool ok;
    char name[16] = "HOME";
    if (!std::strcmp(what, "SORT")) {
      if (std::scanf("%15s", name) != 1) return 1;
      motion::Dest d;
      if (!std::strcmp(name, "RED")) d = motion::Dest::RED;
      else if (!std::strcmp(name, "GREEN")) d = motion::Dest::GREEN;
      else if (!std::strcmp(name, "WHITE")) d = motion::Dest::WHITE;
      else if (!std::strcmp(name, "REJECT")) d = motion::Dest::REJECT;
      else return 1;
      ok = seq.startSort(d);
    } else if (!std::strcmp(what, "HOME")) {
      ok = seq.startHome();
    } else {
      return 1;
    }
    if (!ok) return 1;
    long n = 0;
    double cmax = 0.0;
    motion::Command c;
    for (;;) {
      const auto t0 = std::chrono::steady_clock::now();
      const bool act = seq.tick(c);
      const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
      if (us > cmax) cmax = us;
      if (!act) break;
      n++;
      std::printf("CMD,%ld,%s,%.4f,%.4f,%.4f,%.4f\n", ++tick, motion::phaseName(c.phase), (double)c.servoDeg[0],
                  (double)c.servoDeg[1], (double)c.servoDeg[2], (double)c.gripDeg);
    }
    std::printf("ARM DONE %s status=%s ticks=%ld cmax_us=%.1f\n", name, mk2kin::statusName(seq.error()), n, cmax);
  }
  return 0;
}
