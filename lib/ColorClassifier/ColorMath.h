// ColorMath.h - phan loai mau nap chai tu du lieu RGBC (khong phu thuoc Arduino).
//
// Thuat toan (giong het ban Python trong tools/colorcal.py):
//   1. Neu |C - C_empty| < C_margin            -> NO_CAP   (tram trong, tiep tuc cho)
//   2. Neu bao hoa hoac R+G+B ~ 0              -> UNKNOWN  (khong tin duoc phep do)
//   3. r = R/(R+G+B), g = G/(R+G+B)            (toa do sac do, bo anh huong cuong do)
//   4. Tim tam lop gan nhat (khoang cach Euclid trong mat phang (r, g))
//   5. Neu khoang cach > d_max                 -> UNKNOWN  (dua vao khu vuc "vut bo")
//      nguoc lai                               -> nhan cua tam lop gan nhat
//
// File nay chi dung <stdint.h>/<math.h> nen bien dich duoc ca tren PC
// (dung trong bai kiem tra tuong duong firmware <-> Python).
#pragma once
#include <stdint.h>
#include <math.h>

namespace colormath {

enum Label : int8_t { GREEN = 0, RED = 1, YELLOW = 2, UNKNOWN = 3, NO_CAP = 4 };
static const int NUM_CLASSES = 3;
static const uint32_t MODEL_MAGIC = 0x434D3031UL;  // "CM01"

struct Model {
  uint32_t magic;               // MODEL_MAGIC neu hop le
  uint8_t atime;                // thanh ghi ATIME cua TCS34725
  uint8_t gain;                 // 0..3 = 1x, 4x, 16x, 60x
  uint8_t navg;                 // so lan doc lay trung binh
  uint8_t nflush;               // so lan doc bo di sau khi bat LED
  float cent[NUM_CLASSES][2];   // tam lop (r, g): GREEN, RED, YELLOW
  float dmax;                   // nguong loai bo
  float cEmpty;                 // C trung binh khi tram trong
  float cMargin;                // |C - cEmpty| < cMargin -> khong co nap
};

struct Result {
  Label label;       // ket qua cuoi cung
  Label nearest;     // lop co tam gan nhat (de chan doan)
  float dist;        // khoang cach toi tam gan nhat
  float rn, gn;      // toa do sac do
  bool saturated;
};

// Muc bao hoa cua kenh C, theo cung logic voi
// Adafruit_TCS34725::calculateColorTemperature_dn40():
//  - 1024 dem / chu ky 2.4 ms, toi da 65535
//  - khi so chu ky <= 63 (ATIME >= 0xC1) lay 75% de tranh bao hoa do gon song.
inline uint32_t saturationLevel(uint8_t atime) {
  uint32_t cycles = 256u - (uint32_t)atime;
  if (cycles > 63u) return 65535u;
  uint32_t sat = 1024u * cycles;
  sat -= sat / 4u;
  return sat;
}

inline float integrationMs(uint8_t atime) { return (256.0f - (float)atime) * 2.4f; }

inline const char* labelName(Label l) {
  switch (l) {
    case GREEN:   return "GREEN";
    case RED:     return "RED";
    case YELLOW:  return "YELLOW";
    case UNKNOWN: return "UNKNOWN";
    case NO_CAP:  return "NO_CAP";
  }
  return "?";
}

inline bool modelValid(const Model& m) {
  return m.magic == MODEL_MAGIC && m.navg > 0 && m.dmax > 0.0f && m.gain <= 3;
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

  if (fabsf(C - m.cEmpty) < m.cMargin) {
    res.label = NO_CAP;
  } else if (saturated || s < 1.0f) {
    res.label = UNKNOWN;
  } else if (dBest > m.dmax) {
    res.label = UNKNOWN;
  } else {
    res.label = (Label)best;
  }
  return res;
}

}  // namespace colormath
