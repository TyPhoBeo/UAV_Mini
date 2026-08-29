"""PHASE E2/E3 -- expo can nghieng + bu cos(tilt) cho throttle.

E3 -- VI SAO PHAI BU
--------------------
Luc day 4 canh quat luon vuong goc voi THAN drone. Nghieng theta thi thanh phan
THANG DUNG chi con F*cos(theta):
    12 deg -> cos 0.978 -> mat 2.2% luc nang doc  (~22 duty tren nen ~1000)
    30 deg -> cos 0.866 -> mat 13.4%
Khong bu thi cu nghieng la tut. Vong Vz CO THE tu bu bang I-term, nhung mat vai
tram ms de hoc -- trong khoang do drone da tut roi. Bu cos la FEEDFORWARD.

⚠ TRAN BU LA BAT BUOC
1/cos phan ky khi theta -> 90deg. Mot lan attitude estimate loi (va no CO loi
luc va cham/rung manh) se cho he so khong lo -> FULL THROTTLE. Tran nay la thu
duy nhat dung giua mot sai so cam bien va ga toi da.

⚠ NGUON cos PHAI TUOI THEO NHIP DIEU KHIEN
tof_tilt_cos chi duoc ghi trong if(tof_new) -- tuc nhip ToF (~25Hz), va DUNG HAN
neu ToF chet. Bu ga phai theo kip attitude (250Hz), nen phai dung mot truong
rieng duoc ghi moi tick.
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


print("== (1) E3: hang so bu tilt ==")
en = const_of(TUNE, "TILT_COMP_ENABLED")
check("TILT_COMP_ENABLED = 1", en == 1.0)
mx = const_of(TUNE, "TILT_COMP_MAX_FACTOR")
check("TILT_COMP_MAX_FACTOR co dinh nghia", mx is not None)
if mx is not None:
    check("tran > 1.0 (co bu that)", mx > 1.0)
    check("tran <= 1.5 (khong cho phan ky)", mx <= 1.5,
          "tran %s qua rong: mot sai so attitude se thanh full throttle" % mx)
    # goc toi da con bu duoc
    import math
    deg = math.degrees(math.acos(min(1.0, 1.0 / mx)))
    tilt_max = const_of(strip_c_comments(CORE_C), "MOVE_MAX_TILT_DEG")
    if tilt_max:
        check("tran phu duoc goc bay thuc te (%.1f deg >= %.1f deg)" % (deg, tilt_max),
              deg >= tilt_max,
              "tran chi bu toi %.1f deg nhung bay toi %.1f deg -> vung tren khong bu"
              % (deg, tilt_max))

print()
print("== (2) E3: dung nguon cos TUOI, khong phai tof_tilt_cos ==")
mblk = re.search(r"#if TILT_COMP_ENABLED(.*?)#endif", CODE, re.S)
check("tim thay khoi bu tilt", mblk is not None)
if mblk:
    seg = mblk.group(1)
    check("dung s_alt_est.tilt_cos", "s_alt_est.tilt_cos" in seg)
    check("KHONG dung tof_tilt_cos", "tof_tilt_cos" not in seg,
          "tof_tilt_cos chi cap nhat o nhip ToF va DUNG HAN neu ToF chet")
    check("co clampf gioi han he so", "clampf" in seg)
    check("co kiem isfinite", "isfinite" in seg,
          "NaN tu quaternion hong se lam throttle thanh rac")
    check("chi bu khi throttle_cmd > 0",
          re.search(r"throttle_cmd\s*>\s*0", seg) is not None,
          "khong kiem = co nguy co 'hoi sinh' ga luc disarmed/landing cutoff")
    check("ket qua duoc clamp vao MOTOR_SAFE_MAX_DUTY",
          "MOTOR_SAFE_MAX_DUTY" in seg)

print()
print("== (3) tilt_cos duoc ghi MOI TICK, ngoai if(tof_new) ==")
check("estimator co ghi e->tilt_cos = rzz",
      re.search(r"e->tilt_cos\s*=\s*rzz", EST) is not None)
mfun = re.search(r"void alt_estimator_update\s*\(", EST)
mset = re.search(r"e->tilt_cos\s*=\s*rzz", EST)
if mfun and mset:
    body = EST[mfun.end():mset.start()]
    depth = body.count("{") - body.count("}")
    check("ghi o do sau ngoac = 1 (than ham, moi tick)",
          depth == 1,
          "depth=%d -- nam trong khoi con thi khong con tuoi moi tick" % depth)

print()
print("== (4) E2: expo ==")
e = const_of(TUNE, "MOVE_TILT_EXPO")
check("MOVE_TILT_EXPO co dinh nghia", e is not None)
if e is not None:
    check("expo trong [0, 1]", 0.0 <= e <= 1.0, "duoc %s" % e)
check("co ham move_expo()", "move_expo" in CODE)
mexp = re.search(r"static inline float move_expo\s*\([^)]*\)\s*\{(.*?)\}", CODE, re.S)
check("cong thuc e*x^3 + (1-e)*x", mexp is not None and
      re.search(r"x\s*\*\s*x\s*\*\s*x", mexp.group(1)) is not None if mexp else False)

print()
print("== (5) E2: expo CHI ap cho tilt, KHONG cho do cao/yaw ==")
mmove = re.search(r"case CMD_MOVE:\s*\{(.*?)\n            break;", CODE, re.S)
check("tim thay case CMD_MOVE", mmove is not None)
if mmove:
    seg = mmove.group(1)
    for d in ("MOVE_FORWARD", "MOVE_BACK", "MOVE_LEFT", "MOVE_RIGHT"):
        mline = re.search(r"case %s:[^;]*;" % d, seg)
        check("%s dung pct (co expo)" % d,
              mline is not None and re.search(r"\bpct\b", mline.group(0)) is not None
              and "pct_raw" not in mline.group(0))
    for d in ("MOVE_UP", "MOVE_DOWN"):
        mline = re.search(r"case %s:[^;]*;" % d, seg)
        check("%s dung pct_raw (buoc 10cm phai dung 10cm)" % d,
              mline is not None and "pct_raw" in mline.group(0))
    for d in ("MOVE_CW", "MOVE_CCW"):
        mline = re.search(r"case %s:[^;]*;" % d, seg)
        check("%s dung pct_raw (yaw tuyen tinh)" % d,
              mline is not None and "pct_raw" in mline.group(0))

print()
if fails:
    print("KET QUA: %d FAIL -- %s" % (len(fails), ", ".join(fails)))
    sys.exit(1)
print("KET QUA: ALL PASS -- bu cos co tran + nguon tuoi, expo chi cho tilt")
