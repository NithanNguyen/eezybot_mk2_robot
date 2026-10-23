// color_console.h - lenh Serial cho cam bien mau (dung boi tools/colorcal/colorcal.py).
//
//   PING                         -> OK PONG
//   INFO                         -> INFO,fw=..,sensor=..,led=..,model=..
//   LED <0-10>                   -> OK LED,<muc>        (ghi thanh ghi 0x10 cua GY-33)
//   MEAS <navg> <settle_ms>      -> MEAS,R,G,B,C,sat,led
//   CLS                          -> CLS,label,nearest,dist,r,g,R,G,B,C,sat   (do theo model)
//   CLSRAW <R> <G> <B> <C> <sat> -> CLS,...   (phan loai so lieu cho truoc)
//   MODEL GET                    -> MODEL,src,led,navg,settle,c00,c01,c10,c11,c20,c21,dmax,cEmpty,cMargin
//   MODEL SET <12 gia tri>       -> OK MODEL,nvs
//   MODEL CLEAR                  -> OK MODEL,cleared
//   RUN <ms>                     -> phan loai lien tuc; RUN 0 de dung
// Loi: "ERR <ly do>". Dong bat dau bang '#' la thong bao.
#pragma once
#include "ColorClassifier.h"

namespace color_console {

void begin(ColorClassifier& cc, const char* fwVersion);
bool handle(char* tok[], int n);     // true neu da xu ly lenh
void update();                       // goi trong loop(): phuc vu RUN
// Phan loai mot lan, in dong CLS; neu UNKNOWN in them "# ERR UNKNOWN_COLOR ...".
colormath::Result classifyAndPrint();

}  // namespace color_console
