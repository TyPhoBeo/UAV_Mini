# -*- coding: utf-8 -*-
"""
test_takeoff_gate_offline.py -- cong CMD_TAKEOFF: vi sao lenh bi tu choi, va
lieu ly do co DI RA DUOC toi GUI khong.

BOI CANH (loi that da xay ra): SENSOR_BARO_ENABLED=0 + ToF driver_ok=0 ->
CMD_TAKEOFF cham `break` o nhanh "khong co nguon correction nao" va CHI bao
bang ESP_LOGW. ESP_LOGW khong di qua UDP. Trong khi do command_parser.c tra loi
"TAKEOFF started" dua tren DU DOAN chi xet FSM state. Nhin tu GUI: bam TAKEOFF,
thay bao thanh cong, khong co gi xay ra, khong co ly do.

Bai test nay bao ve:
  (A) Thu tu + ket qua cua tung dieu kien trong cong.
  (B) Bat bien QUAN TRONG NHAT: bi tu choi thi PHAI co ma reject khac 0
      (khong duong nao im lang), va reject_seq phai tang.
  (C) Enum trong telemetry.h khop bang ten trong GUI.
"""

import io
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TELEM_H = os.path.join(HERE, "..", "components", "flight_core", "include",
                       "flight_core", "telemetry.h")
GUI_PY = os.path.join(HERE, "..", "tools", "uav_udp_console.py")

fails = []


def check(name, got, want):
    if got == want:
        print("  PASS  " + name)
    else:
        print("  FAIL  %s: got=%r want=%r" % (name, got, want))
        fails.append(name)


# =============================================================================
# Ban chep cong CMD_TAKEOFF (flight_core.c case CMD_TAKEOFF)
# =============================================================================
REJ_NONE, REJ_STATE, REJ_NO_CORRECTION, REJ_ALT_EST_INVALID = 0, 1, 2, 3

FSM_DISARMED, FSM_ARMED, FSM_BENCH_RAMP = "DISARMED", "ARMED", "BENCH_RAMP"
FSM_TAKING_OFF, FSM_HOLDING = "TAKING_OFF", "HOLDING"


class Core(object):
    def __init__(self, state=FSM_ARMED, baro_ok=False, tof_ok=False,
                 tof_ground_ref_valid=False, alt_est_valid=True):
        self.state = state
        self.baro_ok = baro_ok
        self.tof_ok = tof_ok
        self.tof_ground_ref_valid = tof_ground_ref_valid
        self.alt_est_valid = alt_est_valid
        self.reject = REJ_NONE
        self.reject_seq = 0
        self.takeoff_started = False

    def cmd_takeoff(self):
        # Xoa ly do lan truoc NGAY DAU (khop firmware) -- neu khong, ly do cu
        # dinh lai va GUI hien sai sau khi nguoi dung da sua xong.
        self.reject = REJ_NONE

        if self.state != FSM_ARMED:
            self.reject = REJ_STATE
            self.reject_seq += 1
            return

        has_tof = self.tof_ok and self.tof_ground_ref_valid
        has_baro = self.baro_ok
        if not has_tof and not has_baro:
            self.reject = REJ_NO_CORRECTION
            self.reject_seq += 1
            return

        if not self.alt_est_valid:
            self.reject = REJ_ALT_EST_INVALID
            self.reject_seq += 1
            return

        self.takeoff_started = True
        self.state = FSM_TAKING_OFF


print("== (A) Tung dieu kien cua cong ==")

# T1 -- CHINH XAC cau hinh dang loi cua nguoi dung.
c = Core(state=FSM_ARMED, baro_ok=False, tof_ok=False)
c.cmd_takeoff()
check("T1 baro TAT + ToF hong -> NO_CORRECTION", c.reject, REJ_NO_CORRECTION)
check("T1 khong cat canh", c.takeoff_started, False)
check("T1 FSM khong doi", c.state, FSM_ARMED)

# T1b -- ToF driver OK nhung ground_ref chua chot (chua ARM thanh cong lan nao,
# hoac luc ARM khong co mau ToF hop le). VAN la khong co nguon correction.
c = Core(baro_ok=False, tof_ok=True, tof_ground_ref_valid=False)
c.cmd_takeoff()
check("T1b ToF driver OK nhung thieu ground_ref -> van NO_CORRECTION",
      c.reject, REJ_NO_CORRECTION)

# T2 -- ToF day du, baro tat: PHAI bay duoc (khong hard-code doi rieng baro).
c = Core(baro_ok=False, tof_ok=True, tof_ground_ref_valid=True)
c.cmd_takeoff()
check("T2 ToF-only -> CAT CANH duoc", c.takeoff_started, True)
check("T2 khong co reject", c.reject, REJ_NONE)

# T3 -- baro bat, ToF hong: van bay duoc.
c = Core(baro_ok=True, tof_ok=False)
c.cmd_takeoff()
check("T3 baro-only -> CAT CANH duoc", c.takeoff_started, True)

# T4 -- sai state.
for st in (FSM_DISARMED, FSM_BENCH_RAMP, FSM_HOLDING, FSM_TAKING_OFF):
    c = Core(state=st, baro_ok=True)
    c.cmd_takeoff()
    check("T4 state=%s -> REJECT_STATE" % st, c.reject, REJ_STATE)

# T5 -- estimator khong hop le, du CO nguon correction.
c = Core(baro_ok=True, alt_est_valid=False)
c.cmd_takeoff()
check("T5 estimator invalid -> ALT_EST_INVALID", c.reject, REJ_ALT_EST_INVALID)

# T6 -- THU TU: thieu correction duoc bao TRUOC estimator invalid. Quan trong vi
# ly do dau tien moi la ly do nguoi dung phai sua; bao nham se dan ho di sai huong.
c = Core(baro_ok=False, tof_ok=False, alt_est_valid=False)
c.cmd_takeoff()
check("T6 hong ca hai -> bao NO_CORRECTION truoc", c.reject, REJ_NO_CORRECTION)


print("\n== (B) Bat bien: KHONG duong nao im lang ==")

# Moi cach that bai deu phai de lai ma != 0. Day chinh la thu bi thieu truoc day.
cases = [
    ("state sai", Core(state=FSM_DISARMED, baro_ok=True)),
    ("khong correction", Core(baro_ok=False, tof_ok=False)),
    ("estimator invalid", Core(baro_ok=True, alt_est_valid=False)),
]
for name, c in cases:
    c.cmd_takeoff()
    check("B: %s -> reject != 0" % name, c.reject != REJ_NONE, True)
    check("B: %s -> reject_seq tang" % name, c.reject_seq, 1)

# Thanh cong thi KHONG duoc tang seq (GUI dung seq de biet "vua bam lai va lai truot").
c = Core(baro_ok=True)
c.cmd_takeoff()
check("B: thanh cong -> reject_seq KHONG tang", c.reject_seq, 0)

# Sua xong roi bam lai -> ma reject phai ve 0, khong duoc dinh ly do cu.
c = Core(baro_ok=False, tof_ok=False)
c.cmd_takeoff()
assert c.reject == REJ_NO_CORRECTION
c.baro_ok = True
c.cmd_takeoff()
check("B: sua xong bam lai -> reject ve NONE", c.reject, REJ_NONE)
check("B: sua xong bam lai -> cat canh", c.takeoff_started, True)


print("\n== (C) Enum firmware khop bang ten GUI ==")

telem = io.open(TELEM_H, encoding="utf-8").read()
m = re.search(r"typedef enum \{(.*?)\} takeoff_reject_t;", telem, re.S)
check("telemetry.h co takeoff_reject_t", m is not None, True)
fw_names = re.findall(r"(TAKEOFF_REJECT_\w+)", m.group(1)) if m else []
check("firmware co dung 4 ma", len(fw_names), 4)
check("thu tu firmware", fw_names,
      ["TAKEOFF_REJECT_NONE", "TAKEOFF_REJECT_STATE",
       "TAKEOFF_REJECT_NO_CORRECTION", "TAKEOFF_REJECT_ALT_EST_INVALID"])

gui = io.open(GUI_PY, encoding="utf-8").read()
m2 = re.search(r"TAKEOFF_REJECT_NAMES = \{(.*?)\n\}", gui, re.S)
check("GUI co TAKEOFF_REJECT_NAMES", m2 is not None, True)
gui_keys = re.findall(r'^\s*"(\d+)":', m2.group(1), re.M) if m2 else []
check("GUI co du ma 0..3", gui_keys, ["0", "1", "2", "3"])

# Wire: TKOREJ phai co trong telemetry_format.c, neu khong thi ma reject
# khong bao gio ra khoi firmware -- dung cai loi dang sua.
fmt = io.open(os.path.join(HERE, "..", "src", "telemetry_format.c"),
              encoding="utf-8").read()
check("wire co TKOREJ=", "TKOREJ=%d" in fmt, True)
check("wire co TKORSEQ=", "TKORSEQ=%u" in fmt, True)
check("co ban ra telemetry", "takeoff_reject_seq" in fmt, True)

# command_parser KHONG duoc noi "started" (no khong biet du dieu kien de noi).
parser = io.open(os.path.join(HERE, "..", "src", "command_parser.c"),
                 encoding="utf-8").read()
check("parser KHONG con noi 'TAKEOFF started'", "TAKEOFF started" in parser, False)


print("")
if fails:
    print("KET QUA: %d FAIL -- %s" % (len(fails), ", ".join(fails)))
    sys.exit(1)
print("KET QUA: ALL PASS")
