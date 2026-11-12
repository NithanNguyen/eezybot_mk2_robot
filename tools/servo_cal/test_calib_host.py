#!/usr/bin/env python3
"""Bien dich va chay tools/servo_cal/host_calib_test.cpp (logic firmware hieu chuan tren PC, can g++)."""
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
# Chay hai lan: API LEDC cua Arduino-ESP32 core 2.x (ledcSetup/ledcAttachPin) va 3.x (ledcAttach).
rc = 0
with tempfile.TemporaryDirectory() as td:
    for major in (2, 3):
        exe = Path(td) / "host_calib_test_{}".format(major)
        subprocess.check_call(["g++", "-std=gnu++17", "-O1", "-Wall", "-Wextra", "-Werror",
                               "-DESP_ARDUINO_VERSION_MAJOR={}".format(major), "-I" + str(HERE / "stub"),
                               "-I" + str(ROOT / "include"), "-I" + str(ROOT / "lib" / "ServoPwm"), "-o", str(exe),
                               str(HERE / "host_calib_test.cpp")])
        print("== Arduino-ESP32 core {}.x".format(major))
        rc |= subprocess.run([str(exe)]).returncode
sys.exit(rc)
