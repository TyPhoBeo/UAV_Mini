"""PHASE A -- FLYING chay TANG TRONG cua cascade (vz -> throttle), vz_target = 0.

VAN DE DUOC SUA
---------------
Ban truoc: vao FLYING la TAT CA HAI tang, throttle dong bang o
s_flying_throttle_latch. "Ga dung yen" KHONG dong nghia "do cao dung yen":
neu luc vao FLYING drone dang co vz != 0 (vua nha W/S, vua thoat guard, hay
chi la propwash) thi no TROI TU DO suot ca doan FLYING. Micro quad tut duoc
10-20cm trong 300ms -- va FLYING_TO_HOLD_SETTLE_MS chinh la 300ms.

TAI SAO CHI TAT TANG NGOAI
--------------------------
    tang NGOAI  alt -> vz_target : doc RANGE. Bay qua ban cao 0.75m thi range
                                   nhay 1.2 -> 0.45, tang nay ket luan "tut
                                   0.75m" va boc ga dung dung. PHAI TAT.
    tang TRONG  vz  -> throttle  : doc VAN TOC. Neu vz lay tu accel thuan thi
                                   no KHONG he biet co cai ban nao ben duoi.
                                   Van dung xuyen qua cu nhay. GIU CHAY.

A1 -- NGUON vz PHAI LA ACCEL-ONLY
---------------------------------
alt_estimator fuse ToF vao vz_ms qua HAI duong (xem alt_estimator.c cuoi ham):
    dv  = ALT_EST_TOF_VZ_GAIN * (tof_vz_lpf_ms - vz_ms)
    dv += clamp(ALT_EST_TOF_INNOV_VZ_GAIN * iz / tof_dt_s, -0.20, 0.20)
Bay qua ban -> iz nhay 0.75m -> duong thu hai bom mot van toc GIA vao vz_ms.
Dung vz_ms cho vong Vz trong FLYING = phan ung voi so gia = CON TE HON tat han.
Nen phai dung vz_accel_only_ms.
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CORE_C = (ROOT / "components/flight_core/src/flight_core.c").read_text(encoding="utf-8")
TUNING_H = (ROOT / "components/flight_core/include/flight_core/tuning.h").read_text(encoding="utf-8")
EST_C = (ROOT / "components/flight_core/src/alt_estimator.c").read_text(encoding="utf-8")

fails = []


def check(name, cond, detail=""):
    if cond:
        print("  PASS  %s" % name)
    else:
        print("  FAIL  %s%s" % (name, ("  -- " + detail) if detail else ""))
        fails.append(name)


def strip_c_comments(src):
    src = re.sub(r"/\*.*?\*/", "", src, flags=re.S)
    return re.sub(r"//[^\n]*", "", src)


CODE = strip_c_comments(CORE_C)
TUNE = strip_c_comments(TUNING_H)
EST = strip_c_comments(EST_C)


def const_of(src, name):
    m = re.search(r"#define\s+%s\s+([0-9.]+)f?" % re.escape(name), src)
    return float(m.group(1)) if m else None


print("== (1) Hang so Phase A ton tai va hop ly ==")
en = const_of(TUNE, "FLYING_VZ_HOLD_ENABLED")
check("FLYING_VZ_HOLD_ENABLED = 1", en == 1.0, "duoc = %r" % en)
accel_only = const_of(TUNE, "FLYING_VZ_USE_ACCEL_ONLY")
check("FLYING_VZ_USE_ACCEL_ONLY = 1 (A1)", accel_only == 1.0,
      "phai =1, xem docstring; duoc = %r" % accel_only)

ilim = const_of(TUNE, "FLYING_VZ_ILIMIT_DUTY")
hold_ilim = const_of(TUNE, "ALT_HOLD_VZ_ILIMIT")
check("FLYING_VZ_ILIMIT_DUTY co dinh nghia", ilim is not None)
if ilim is not None and hold_ilim is not None:
    check("FLYING_VZ_ILIMIT_DUTY <= ALT_HOLD_VZ_ILIMIT",
          ilim <= hold_ilim,
          "FLYING=%s > HOLD=%s: trong FLYING drone nghieng nen vz am nhe la "
          "BINH THUONG; cho I chay rong hon HOLDING se mang mot cuc bias ve "
          "khi tha can" % (ilim, hold_ilim))
    check("FLYING_VZ_ILIMIT_DUTY > 0 (khong phai freeze tra hinh)", ilim > 0.0)

print()
print("== (2) FLYING that su GOI cascade, khong con gan thang latch ==")
# lay rieng nhanh FLYING
m = re.search(r"if\s*\(flying_no_alt_pid\)\s*\{", CODE)
check("tim thay nhanh if (flying_no_alt_pid)", m is not None)
if m:
    seg = CODE[m.start():m.start() + 4000]
    check("nhanh FLYING co goi alt_hold_vz_cascade",
          "alt_hold_vz_cascade" in seg,
          "khong goi = van dang tha troi mo vong, dung loi Phase A sua")
    check("vz_target truyen vao la 0.0f",
          re.search(r"alt_hold_vz_cascade\s*\([^;]*?0\.0f", seg, re.S) is not None)
    check("dung vz_accel_only_ms (A1)",
          "vz_accel_only_ms" in seg,
          "dung vz_ms se an van toc GIA do ToF bom vao khi bay qua vat the")

print()
print("== (3) vz_accel_only_ms THAT SU khong dinh ToF ==")
# trong estimator, correction ToF chi duoc cong vao vz_ms
mfuse = re.search(r"e->vz_ms\s*\+=\s*dv;", EST)
check("tim thay cho fuse ToF vao vz_ms", mfuse is not None)
check("KHONG co dong nao cong dv vao vz_accel_only_ms",
      re.search(r"vz_accel_only_ms\s*\+=\s*dv", EST) is None,
      "neu co thi accel-only da bi nhiem ToF, A1 vo nghia")
# accel_only chi duoc tich phan tu az_corrected
check("vz_accel_only_ms tich phan tu az_corrected_ms2",
      re.search(r"vz_accel_only_ms\s*\+=\s*e->az_corrected_ms2\s*\*\s*dt", EST) is not None)

print()
print("== (4) Nap I bumpless o tick DAU, khong nap lai moi tick ==")
check("co co s_flying_vz_engaged", "s_flying_vz_engaged" in CODE)
check("nap I nam trong if (!s_flying_vz_engaged)",
      re.search(r"if\s*\(!s_flying_vz_engaged\)\s*\{[^}]*vz_integral", CODE, re.S) is not None,
      "nap lai moi tick = ghi de I lien tuc = vong Vz mat han phan tich phan")
check("co xoa co khi roi FLYING",
      re.search(r"s_fsm\.state\s*!=\s*FSM_FLYING\s*\)\s*s_flying_vz_engaged\s*=\s*false", CODE) is not None,
      "khong xoa = lan vao FLYING sau khong nap I -> buoc nhay ga")

print()
print("== (5) A4: ban giao FLYING -> HOLDING KHONG duoc reset I ==")
mexit = re.search(r"roi FLYING -> xoa latch ga", CORE_C)
check("tim thay khoi thoat FLYING", mexit is not None)
if mexit:
    seg = strip_c_comments(CORE_C[mexit.start():mexit.start() + 2500])
    check("khoi thoat KHONG gan vz_integral = 0",
          re.search(r"vz_integral\s*=\s*0", seg) is None,
          "reset I o day = ga tut mot cuc hover ngay tick dau HOLDING")

print()
if fails:
    print("KET QUA: %d FAIL -- %s" % (len(fails), ", ".join(fails)))
    sys.exit(1)
print("KET QUA: ALL PASS -- FLYING giu vz=0 bang tang trong, accel-only, bumpless")
