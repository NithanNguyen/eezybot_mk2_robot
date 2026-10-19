// color_console.h - cac lenh Serial cho cam bien mau (giao thuc dung boi tools/colorcal/colorcal.py).
//
//   PING                         -> OK PONG
//   INFO                         -> INFO,fw=..,sensor=..,atime=..,gain=..,sat=..,it_ms=..,model=..
//   LED 0|1                      -> OK LED <0|1>
//   CFG <atime> <gain>           -> OK CFG,<atime>,<gain>,<sat>,<it_ms>
//   MEAS <navg> <nflush>         -> MEAS,R,G,B,C,sat,atime,gain
//   CLS                          -> CLS,label,nearest,dist,r,g,R,G,B,C,sat   (do theo model)
//   CLSRAW <R> <G> <B> <C> <sat> -> CLS,...   (phan loai so lieu cho truoc, dung kiem tra)
//   MODEL GET                    -> MODEL,src,atime,gain,navg,nflush,c00,c01,c10,c11,c20,c21,dmax,cEmpty,cMargin
//   MODEL SET <13 gia tri>       -> OK MODEL,nvs        (luu vao NVS)
//   MODEL CLEAR                  -> OK MODEL,cleared
//   RUN <ms>                     -> phan loai lien tuc moi <ms>; RUN 0 de dung
// Loi: "ERR <ly do>". Dong bat dau bang '#' la thong bao, PC bo qua.
#pragma once
#include "ColorClassifier.h"

namespace color_console {

void begin(ColorClassifier& cc, const char* fwVersion);
// Tra ve true neu tok[0] la lenh cua module nay (da xu ly va tra loi).
bool handle(char* tok[], int n);
void update();   // goi trong loop(): phuc vu lenh RUN
// Phan loai mot lan, in dong CLS; neu UNKNOWN in them "# ERR UNKNOWN_COLOR ...".
colormath::Result classifyAndPrint();

}  // namespace color_console
