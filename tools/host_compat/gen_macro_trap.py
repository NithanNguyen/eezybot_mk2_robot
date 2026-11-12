#!/usr/bin/env python3
"""Sinh tools/host_compat/arduino_macro_trap.h tu header that cua Arduino-ESP32.

  python3 tools/host_compat/gen_macro_trap.py <thu muc cores/esp32> [<commit>]
  vd. ~/.platformio/packages/framework-arduinoespressif32/cores/esp32

Moi macro (dang doi tuong hoac dang ham) duoc dinh nghia lai thanh hang so 0x7A11, giong newlib <ctype.h>
(#define _L 02). Host test include file nay SAU cac header he thong va TRUOC lib/: neu code trong lib/ khai bao
mot ten trung macro (bien, ham, tham so) thi PC bao loi cu phap giong ESP32. Them 8 macro cua newlib <ctype.h>
(_U _L _N _S _P _C _X _B) vi Arduino.h -> WCharacter.h -> <ctype.h>. Bo qua cac macro trong USED_ON_PURPOSE
(vd. Serial -> doi tuong cong noi tiep), vi firmware dung chung dung nghia.
"""
import pathlib
import re
import sys

NEWLIB_CTYPE = ["_U", "_L", "_N", "_S", "_P", "_C", "_X", "_B"]
# Macro tro toi doi tuong that ma firmware DUNG co chu y (khong phai trung ten): khong bay.
USED_ON_PURPOSE = {"Serial"}
# Macro phien ban / cau hinh dung trong #if de chon nhanh code (vd. ServoPwm chon API LEDC theo
# ESP_ARDUINO_VERSION_MAJOR): khong phai ten co the trung, va bay chung se doi nhanh duoc bien dich tren PC.
CONFIG_PREFIXES = ("ESP_ARDUINO_VERSION", "ESP_IDF_VERSION", "CONFIG_")


def main():
    core = pathlib.Path(sys.argv[1])
    commit = sys.argv[2] if len(sys.argv) > 2 else "?"
    obj, fn = set(), set()
    for f in sorted(core.glob("*.h")):
        for line in f.read_text(errors="replace").splitlines():
            m = re.match(r"\s*#\s*define\s+([A-Za-z_]\w*)(\()?", line)
            if m:
                (fn if m.group(2) else obj).add(m.group(1))
    guard = lambda n: n.startswith("__") or n.endswith(("_h", "_H", "_H_")) or n.startswith(CONFIG_PREFIXES)
    obj = {n for n in obj if not guard(n)} - set(NEWLIB_CTYPE) - USED_ON_PURPOSE
    fn = {n for n in fn if not guard(n)} - obj - USED_ON_PURPOSE
    out = ["// SINH TU DONG boi tools/host_compat/gen_macro_trap.py tu Arduino-ESP32 cores/esp32 (commit {}).".format(commit),
           "// KHONG SUA TAY. Xem docstring cua gen_macro_trap.py.",
           "#pragma once"]
    out += ["#define {} 0x7A11".format(n) for n in NEWLIB_CTYPE]
    out += ["#define {} 0x7A11".format(n) for n in sorted(obj)]
    out += ["#define {}(...) 0x7A11".format(n) for n in sorted(fn)]
    dst = pathlib.Path(__file__).resolve().parent / "arduino_macro_trap.h"
    dst.write_text("\n".join(out) + "\n")
    print("{}: {} macro (newlib ctype {}, doi tuong {}, ham {})".format(dst, len(NEWLIB_CTYPE) + len(obj) + len(fn),
                                                                     len(NEWLIB_CTYPE), len(obj), len(fn)))


if __name__ == "__main__":
    main()
