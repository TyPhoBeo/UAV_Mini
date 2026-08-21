# -*- coding: utf-8 -*-
"""
test_trim_offline.py -- trim roll/pitch: hoan truc, ap dung, va luu NVS.

HAI LOI THAT da sua, va day la test giu chung khong quay lai:

  (1) Trim BIEN MAT trong suot mot lenh timed (CMD_MOVE / CMD_SET_YAW).
      Ban cu chi cong trim o nhanh setpoint; nhanh timed lay thang
      s_timed_*_deg. Nghia la trong 2 giay chay `move forward 30 2`, drone dat
      dung theo cai lech ma trim sinh ra de bu. Trim KHONG phai setpoint -- no
      la hang so bu lech co khi/CG, "lenh nao thang" khong ap dung cho no.

  (2) Trim chi song trong RAM -> mat sach sau moi lan cam lai dien. Nguoi dung
      do trim ca buoi roi mat het, va te hon la de quen roi cat canh voi khung
      lech.

Rui ro rieng cua duong NVS: flight_core.c HOAN TRUC giua quy uoc NGOAI (@TRIM
SET nhan) va bien noi bo. Luu mot dang roi nap dang kia = trim quay 90 do sau
reboot, kieu loi im lang kho lan nhat. Test bat dung cho do.
"""

import io
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CORE_C = os.path.join(HERE, "..", "components", "flight_core", "src", "flight_core.c")
CALIB_C = os.path.join(HERE, "..", "components", "flight_core", "src", "calibration.c")
CALIB_H = os.path.join(HERE, "..", "components", "flight_core", "include",
                       "flight_core", "calibration.h")

TRIM_MAX_DEG = 10.0
SP_TILT_MAX_DEG = 30.0   # khop tuning.h; chi dung lam tran clamp trong mo hinh

fails = []


def check(name, got, want):
    if got == want:
        print("  PASS  " + name)
    else:
        print("  FAIL  %s: got=%r want=%r" % (name, got, want))
        fails.append(name)


def clampf(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


# =============================================================================
# Ban chep phan trim cua flight_core.c
# =============================================================================
class Core(object):
    def __init__(self):
        # Bien NOI BO (da hoan truc so voi quy uoc ngoai).
        self.trim_roll_deg = 0.0
        self.trim_pitch_deg = 0.0
        self.sp_roll_deg = 0.0
        self.sp_pitch_deg = 0.0
        self.sp_yaw_rate_dps = 0.0
        self.timed_active = False
        self.timed_roll_deg = 0.0
        self.timed_pitch_deg = 0.0
        self.timed_yaw_rate_dps = 0.0
        self.nvs = None   # (roll, pitch) theo quy uoc NGOAI

    # case CMD_SET_TRIM
    def cmd_set_trim(self, roll_deg, pitch_deg):
        r = clampf(roll_deg, -TRIM_MAX_DEG, TRIM_MAX_DEG)
        p = clampf(pitch_deg, -TRIM_MAX_DEG, TRIM_MAX_DEG)
        self.trim_pitch_deg = r      # HOAN TRUC (khop firmware)
        self.trim_roll_deg = p
        self.nvs = (r, p)            # luu theo quy uoc NGOAI
        return r, p

    # flight_core_get_trim()
    def get_trim(self):
        return (self.trim_pitch_deg, self.trim_roll_deg)

    # Nap NVS luc boot
    def boot_load(self, nvs):
        self.nvs = nvs
        if nvs is not None:
            roll_out, pitch_out = nvs
            self.trim_pitch_deg = roll_out   # HOAN TRUC y het cmd_set_trim
            self.trim_roll_deg = pitch_out

    # buoc 9 cua stabilize_task -- target dua vao attitude_control
    def attitude_target(self):
        base_roll = self.timed_roll_deg if self.timed_active else self.sp_roll_deg
        base_pitch = self.timed_pitch_deg if self.timed_active else self.sp_pitch_deg
        return (
            clampf(base_roll + self.trim_roll_deg, -SP_TILT_MAX_DEG, SP_TILT_MAX_DEG),
            clampf(base_pitch + self.trim_pitch_deg, -SP_TILT_MAX_DEG, SP_TILT_MAX_DEG),
            self.timed_yaw_rate_dps if self.timed_active else self.sp_yaw_rate_dps,
        )


print("== (A) Hoan truc: SET roi GET phai ra dung so da dat ==")

c = Core()
c.cmd_set_trim(1.5, -2.5)
check("GET tra ve dung quy uoc NGOAI", c.get_trim(), (1.5, -2.5))
check("noi bo DA hoan truc (roll_out -> trim_pitch)", c.trim_pitch_deg, 1.5)
check("noi bo DA hoan truc (pitch_out -> trim_roll)", c.trim_roll_deg, -2.5)

check("clamp +/-10 deg (tren)", c.cmd_set_trim(99.0, 0.0)[0], 10.0)
check("clamp +/-10 deg (duoi)", c.cmd_set_trim(0.0, -99.0)[1], -10.0)


print("\n== (B) LOI 1: trim phai ap dung CA khi dang chay lenh timed ==")

c = Core()
c.cmd_set_trim(1.0, -2.0)     # ngoai: roll=+1, pitch=-2
# -> noi bo: trim_pitch=+1, trim_roll=-2

# Nhanh setpoint (khong co lenh timed)
c.timed_active = False
c.sp_roll_deg, c.sp_pitch_deg = 0.0, 0.0
tr, tp, _ = c.attitude_target()
check("SP: trim cong vao roll noi bo", tr, -2.0)
check("SP: trim cong vao pitch noi bo", tp, 1.0)

# Nhanh timed -- day chinh la cho ban cu DANH ROI trim
c.timed_active = True
c.timed_roll_deg, c.timed_pitch_deg = 5.0, 0.0
tr, tp, _ = c.attitude_target()
check("TIMED: trim VAN duoc cong vao roll", tr, 5.0 + (-2.0))
check("TIMED: trim VAN duoc cong vao pitch", tp, 0.0 + 1.0)


def attitude_target_OLD(core):
    """Ban CU -- de chung minh no that su danh roi trim (test co y nghia)."""
    if core.timed_active:
        return (core.timed_roll_deg, core.timed_pitch_deg, core.timed_yaw_rate_dps)
    return (clampf(core.sp_roll_deg + core.trim_roll_deg, -SP_TILT_MAX_DEG, SP_TILT_MAX_DEG),
            clampf(core.sp_pitch_deg + core.trim_pitch_deg, -SP_TILT_MAX_DEG, SP_TILT_MAX_DEG),
            core.sp_yaw_rate_dps)


check("ban CU that su danh roi trim khi timed (test co y nghia)",
      attitude_target_OLD(c)[0], 5.0)

# yaw rate KHONG duoc dinh trim (trim chi co roll/pitch)
c.timed_yaw_rate_dps = 33.0
check("yaw rate khong bi trim cham vao", c.attitude_target()[2], 33.0)

# clamp cuoi van ap dung sau khi cong trim
c2 = Core()
c2.cmd_set_trim(0.0, 10.0)    # noi bo trim_roll = +10
c2.timed_active = True
c2.timed_roll_deg = 25.0
check("clamp SP_TILT_MAX sau khi cong trim", c2.attitude_target()[0], 30.0)


print("\n== (C) LOI 2: trim song qua reboot, va KHONG bi xoay truc ==")

c = Core()
c.cmd_set_trim(1.5, -2.5)
saved = c.nvs

fresh = Core()
fresh.boot_load(saved)
check("sau reboot GET ra dung so cu", fresh.get_trim(), (1.5, -2.5))
check("sau reboot bien noi bo giong het truoc reboot",
      (fresh.trim_roll_deg, fresh.trim_pitch_deg),
      (c.trim_roll_deg, c.trim_pitch_deg))

# Hanh vi bay phai giong het truoc/sau reboot
c.timed_active = fresh.timed_active = False
check("sau reboot target bay giong het", fresh.attitude_target(), c.attitude_target())


def boot_load_WRONG(core, nvs):
    """Neu ai do quen hoan truc luc nap -- phai thay hau qua ro rang."""
    core.trim_roll_deg, core.trim_pitch_deg = nvs


wrong = Core()
boot_load_WRONG(wrong, saved)
check("quen hoan truc -> trim bi XOAY (test co y nghia)",
      wrong.get_trim() != (1.5, -2.5), True)

# Chua tung luu = 0/0, KHONG phai loi
blank = Core()
blank.boot_load(None)
check("chua tung luu -> trim 0/0", blank.get_trim(), (0.0, 0.0))
check("chua tung luu -> target khong bi lech", blank.attitude_target()[:2], (0.0, 0.0))


print("\n== (D) Duong NVS co that trong source ==")

core_src = io.open(CORE_C, encoding="utf-8").read()
calib_src = io.open(CALIB_C, encoding="utf-8").read()
calib_hdr = io.open(CALIB_H, encoding="utf-8").read()

check("calibration.h khai bao calibration_save_trim",
      "calibration_save_trim" in calib_hdr, True)
check("calibration_params_t co trim_valid", "trim_valid" in calib_hdr, True)
check("calibration.c cai dat save_trim",
      "esp_err_t calibration_save_trim(" in calib_src, True)
check("calibration.c co CRC cho trim", "CALIB_KEY_TRIM_CRC" in calib_src, True)
check("CMD_SET_TRIM co goi save",
      re.search(r"case CMD_SET_TRIM:.*?calibration_save_trim\(", core_src, re.S) is not None,
      True)
check("boot co nap trim tu NVS", "s_calib.trim_valid" in core_src, True)

# Bat bien chong hoi quy cho loi 1: KHONG duoc con nhanh timed gan thang
# target_roll_deg = s_timed_roll_deg (tuc bo qua trim).
check("khong con gan target_roll_deg = s_timed_roll_deg (bo qua trim)",
      re.search(r"target_roll_deg\s*=\s*s_timed_roll_deg", core_src) is None, True)
check("trim duoc cong vao base chung",
      "base_roll + s_trim_roll_deg" in core_src, True)


print("")
if fails:
    print("KET QUA: %d FAIL -- %s" % (len(fails), ", ".join(fails)))
    sys.exit(1)
print("KET QUA: ALL PASS")
