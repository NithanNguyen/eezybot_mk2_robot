// ColorMath.h - phan loai mau nap chai tu du lieu RGBC tho (khong phu thuoc Arduino).
//
// Thuat toan (giong het ban Python trong tools/colorcal/colorcal.py):
//   1. |C - C_empty| < C_margin              -> NO_CAP   (tram trong, tiep tuc cho)
//   2. bao hoa hoac R+G+B ~ 0                -> UNKNOWN
//   3. r = R/(R+G+B), g = G/(R+G+B)          (toa do sac do)
//   4. tam lop gan nhat (Euclid trong mat phang (r, g))
//   5. khoang cach > d_max                   -> UNKNOWN  (khu vuc "vut bo")
//      nguoc lai                             -> nhan cua tam lop gan nhat
#pragma once
#include <stdint.h>
#include <math.h>

namespace colormath {

enum Label : int8_t { GREEN = 0, RED = 1, WHITE = 2, UNKNOWN = 3, NO_CAP = 4 };
static const int NUM_CLASSES = 3;
static const uint32_t MODEL_MAGIC = 0x434D3032UL;  // "CM02" (GY-33)
// GY-33 tra ve so 16 bit; tai lieu khong neu muc bao hoa that cua cau hinh ben trong,
// nen chi coi gia tri 65535 la bao hoa chac chan.
static const uint16_t RAW_MAX = 65535;

struct Model {
  uint32_t magic;               // MODEL_MAGIC neu hop le
  uint8_t led;                  // muc LED GY-33 (0..10) khi thu du lieu
  uint8_t navg;                 // so lan doc lay trung binh
  uint16_t settleMs;            // cho truoc lan doc dau
  float cent[NUM_CLASSES][2];   // tam lop (r, g): GREEN, RED, WHITE
  float dmax;                   // nguong loai bo
  float cEmpty;                 // C trung binh khi tram trong
  float cMargin;                // |C - cEmpty| < cMargin -> khong co nap
};

struct Result {
  Label label;
  Label nearest;
  float dist;
  float rn, gn;
  bool saturated;
};

inline const char* labelName(Label l) {
  switch (l) {
    case GREEN:   return "GREEN";
    case RED:     return "RED";
    case WHITE:   return "WHITE";
    case UNKNOWN: return "UNKNOWN";
    case NO_CAP:  return "NO_CAP";
  }
  return "?";
}

inline bool modelValid(const Model& m) {
  return m.magic == MODEL_MAGIC && m.navg > 0 && m.dmax > 0.0f && m.led <= 10;
}

inline Result classify(const Model& m, float R, float G, float B, float C, bool saturated) {
  Result res;
  res.saturated = saturated;
  float s = R + G + B;
  res.rn = (s > 0.0f) ? R / s : 0.0f;
  res.gn = (s > 0.0f) ? G / s : 0.0f;

  int best = 0;
  float dBest = 1e30f;
  for (int k = 0; k < NUM_CLASSES; k++) {
    float dr = res.rn - m.cent[k][0];
    float dg = res.gn - m.cent[k][1];
    float d = sqrtf(dr * dr + dg * dg);
    if (d < dBest) { dBest = d; best = k; }
  }
  res.nearest = (Label)best;
  res.dist = dBest;

  if (fabsf(C - m.cEmpty) < m.cMargin)  res.label = NO_CAP;
  else if (saturated || s < 1.0f)        res.label = UNKNOWN;
  else if (dBest > m.dmax)               res.label = UNKNOWN;
  else                                   res.label = (Label)best;
  return res;
}

}  // namespace colormath
