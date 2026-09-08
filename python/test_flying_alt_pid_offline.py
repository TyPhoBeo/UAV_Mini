"""FLYING chay PID do cao GIONG HET HOLDING, va W/S = +/-0.10m o CA HAI state.

VAN DE DA SUA (do duoc khi bay thu)
-----------------------------------
Trong FLYING, nghieng de bay ngang thi drone LUON co xu huong TUT do cao.
Dung ve vat ly: thanh phan thang dung cua luc day giam theo cos(tilt). Nhung
o ban cu KHONG CO VONG NAO KEO LAI:

    tang NGOAI (alt -> vz_target)  : bi TAT trong FLYING
    s_flying_throttle_latch        : dung yen, khong biet drone dang tut

Ly do CU de tat tang ngoai la "ToF nhay khi bay qua vat the". Nhung viec do
gio da co LOP TERRAIN xu ly ngay trong estimator (terr_pending -> confirm ->
commit offset): alt_m da LIEN TUC xuyen qua bac dia hinh, nen tang ngoai
KHONG con nhin thay cu nhay nao de phan ung sai.

=> Tat tang ngoai nua la VUA THUA VUA CO HAI.

HOP DONG MOI
------------
    FLYING == HOLDING ve mat dieu khien do cao. Ca hai tang cascade deu chay.
    ToF van la nguon do cao chuan. Bu cos(tilt) lo phan feedforward.
    W/S = +/-0.10m vao alt target o CA HAI state (mot y nghia duy nhat).

Co bien dich FLYING_DISABLES_ALT_PID duoc GIU LAI (mac dinh 0) de quay ve
hanh vi cu ngay neu lop terrain to ra khong du tin khi bay thuc.
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CORE_C = (ROOT / "components/flight_core/src/flight_core.c").read_text(encoding="utf-8")
GUI_PY = (ROOT / "tools/uav_udp_console.py").read_text(encoding="utf-8")

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

print("== (1) FIRMWARE: FLYING khong con tat PID do cao ==")
m = re.search(r"#define\s+FLYING_DISABLES_ALT_PID\s+(\d)", CODE)
check("co FLYING_DISABLES_ALT_PID", m is not None)
check("mac dinh = 0", m is not None and m.group(1) == "0",
      "bat co nay = FLYING lai tut do cao khi nghieng")

# Khi co = 0, flying_no_alt_pid phai la hang so false -> ca nhanh la dead code.
check("nhanh #else dat flying_no_alt_pid = false",
      re.search(r"#else\s*\n\s*const bool flying_no_alt_pid = false;", CODE) is not None,
      "khong co nhanh nay thi tat co cung khong doi duoc hanh vi")
check("nhanh #if van giu duong cu (quay lai duoc)",
      re.search(r"#if FLYING_DISABLES_ALT_PID\s*\n\s*const bool flying_no_alt_pid = \(s_fsm\.state == FSM_FLYING\);",
                CODE) is not None)

print()
print("== (2) FIRMWARE: FLYING roi vao dung nhanh alt_hold_run() ==")
# Nhanh cuoi cua chuoi if/else phai la alt_hold_run() -- do la duong ma
# HOLDING di, va gio FLYING cung di.
check("con nhanh else goi alt_hold_run()",
      re.search(r"\}\s*else\s*\{\s*alt_hold_run\(", CODE) is not None,
      "khong con thi FLYING se roi vao mot nhanh khac, khong phai HOLD binh thuong")

print()
print("== (3) GUI: W/S = buoc do cao o CA '2' (HOLD) lan '5' (FLYING) ==")
mp = re.search(r"def _ws_press\(self, sign\):(.*?)\n    def ", GUI_PY, re.S)
check("tim thay _ws_press()", mp is not None)
if mp:
    body = mp.group(1)
    check("re nhanh theo ca hai mode '2' va '5'",
          re.search(r"_ws_mode\(\)\s+in\s+\(\s*[\"']2[\"']\s*,\s*[\"']5[\"']\s*\)", body) is not None,
          "chi bat '2' thi FLYING van di duong offset ga cu")
    # thu tu quan trong: nhanh alt_step phai RETURN truoc khi toi _send_thr_offset
    i_step = body.find("_alt_step(")
    i_ret = body.find("return", i_step) if i_step != -1 else -1
    i_thr = body.find("_send_thr_offset(")
    check("nhanh buoc do cao RETURN truoc nhanh offset ga",
          i_step != -1 and i_ret != -1 and i_thr != -1 and i_ret < i_thr,
          "khong return thi mot lan bam gui CA hai lenh")

print()
print("== (4) GUI: buoc do cao khop firmware ==")
mstep = re.search(r"WS_ALT_STEP_M\s*=\s*([0-9.]+)", GUI_PY)
check("WS_ALT_STEP_M co dinh nghia", mstep is not None)
if mstep:
    step = float(mstep.group(1))
    check("WS_ALT_STEP_M = 0.10m", abs(step - 0.10) < 1e-9, "duoc %s" % step)
    # firmware ap dung buoc nay cho '>' / '<'
    PARSER = (ROOT / "src/command_parser.c").read_text(encoding="utf-8")
    check("firmware dung cung buoc cho '>'/'<'",
          "0.10f" in PARSER or "0.1f" in PARSER,
          "GUI chi gui PHIM, so buoc do firmware quyet -- lech nhau la GUI hien sai")

print()
print("== (5) GUI: nhan nut khong con noi 'ga' o FLYING ==")
check("nhan W/S dung buoc cm cho ca hai mode",
      re.search(r"if amode in \(\s*[\"']2[\"']\s*,\s*[\"']5[\"']\s*\)", GUI_PY) is not None,
      "nhan cu noi 'ga +150' o FLYING se sai hoan toan so voi hanh vi that")

print()
if fails:
    print("KET QUA: %d FAIL -- %s" % (len(fails), ", ".join(fails)))
    sys.exit(1)
print("KET QUA: ALL PASS -- FLYING == HOLDING ve do cao, W/S = +/-0.10m ca hai")
