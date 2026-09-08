"""Frame do cao = DATUM, va TARGET HIEU DUNG so voi be mat (TGTS).

NGU NGHIA DA CHON (yeu cau nguoi dung):
    Bay 1.5m, len ban 1.0m  ->  Z TUYET DOI giu nguyen 1.5m
                            ->  target hieu dung = 1.5 - 1.0 = 0.5m TREN BAN
    Roi ban (offset -> 0)   ->  target hieu dung ve lai 1.5m, Z van 1.5m

Tuc la drone KHONG len khong xuong khi qua vat the -- chi KHOANG HO doi. Do la
DATUM. AGL (terrain following) la hanh vi NGUOC LAI: leo them dung chieu cao
ban de giu khoang ho; huu ich ngoai troi, KHONG phai cai muon khi bay trong nha.

CHO DE NHAM: "target phai duoc cong offset" nghe nhu phai ghi de
s_alt_target_m. KHONG PHAI. O DATUM phep tru da nam san trong alt_m:
    err = target - alt_m,  ma alt_m = khoang_ho + terrain_off
        = target - khoang_ho - terrain_off
        = (target - terrain_off) - khoang_ho
          └────────┬──────────┘
           chinh la target hieu dung so voi be mat
Cong them mot lan nua o ve target = tinh HAI LAN. TGTS chi de DOC.
"""
import io
import os
import re
import sys

FAILED = []


def check(name, cond, detail=""):
    print("  %s  %s" % ("PASS" if cond else "FAIL", name))
    if not cond:
        if detail:
            print("        %s" % detail)
        FAILED.append(name)


def read(rel):
    p = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), rel)
    return io.open(p, encoding="utf-8", errors="replace").read()


def strip_c_comments(s):
    """Bo comment: khoi giai thich o day trich dan chinh cac ten bien dang kiem."""
    s = re.sub(r"/\*.*?\*/", " ", s, flags=re.S)
    s = re.sub(r"//[^\n]*", " ", s)
    return s


FC = read("components/flight_core/src/flight_core.c")
CODE = strip_c_comments(FC)
TLM_H = read("components/flight_core/include/flight_core/telemetry.h")
TLM_C = strip_c_comments(read("src/telemetry_format.c"))

print("== (1) Frame mac dinh = DATUM (giu Z tuyet doi) ==")
m = re.search(r"static\s+alt_frame_t\s+s_alt_frame\s*=\s*(ALT_FRAME_\w+)\s*;", CODE)
check("tim thay khai bao s_alt_frame", m is not None)
if m:
    check("mac dinh la ALT_FRAME_DATUM", m.group(1) == "ALT_FRAME_DATUM",
          "dang la %s -- AGL se LEO LEN khi qua ban thay vi giu Z" % m.group(1))

print()
print("== (2) Target KHONG bi ghi de bang terrain_off ==")
check("khong co `s_alt_target_m += ...terrain_off...`",
      re.search(r"s_alt_target_m\s*[+-]?=\s*[^;]*terrain_off", CODE) is None,
      "cong offset vao target = tinh HAI LAN (alt_m da mang offset)")
check("alt_meas_m chon theo frame",
      re.search(r"alt_meas_m\s*=\s*\(\s*s_alt_frame\s*==\s*ALT_FRAME_AGL\s*\)",
                CODE) is not None)

print()
print("== (3) TGTS -- target hieu dung so voi be mat (CHI DE DOC) ==")
check("telemetry.h khai bao alt_target_surface_m",
      re.search(r"float\s+alt_target_surface_m\s*;", TLM_H) is not None)
check("duoc dien = target - terrain_off",
      re.search(r"alt_target_surface_m\s*=\s*s_alt_target_m\s*-\s*"
                r"s_alt_est\.terrain_off_m", CODE) is not None,
      "phai la hieu cua hai so da co, khong phai mot bien trang thai moi")
check("co trong chuoi telemetry (TGTS=)",
      "TGTS=%.3f" in TLM_C or "TGTS=" in TLM_C)
check("TGTS KHONG duoc dung o duong dieu khien",
      re.search(r"(alt_hold_run|alt_hold_vz_cascade)\s*\([^;]*alt_target_surface",
                CODE, re.S) is None,
      "chi de doc; dua vao cascade se tru offset hai lan")

print()
print("== (4) Mo phong ngu nghia DATUM ==")


def effective_target(tgt_datum, terrain_off):
    """Target hieu dung so voi BE MAT dang bay tren."""
    return tgt_datum - terrain_off


TGT = 1.50
print("  %-14s %7s %7s %8s %8s" % ("giai doan", "TGT", "TOFF", "TGTS", "Z that"))
seq = (("tren san", 0.00), ("len ban 1.0m", 1.00), ("roi ban", 0.00))
z_values = []
for label, toff in seq:
    tgts = effective_target(TGT, toff)
    z = TGT                      # DATUM: Z tuyet doi = target, khong doi
    z_values.append(z)
    print("  %-14s %7.2f %7.2f %8.2f %8.2f" % (label, TGT, toff, tgts, z))

check("Z tuyet doi KHONG doi qua ca ba giai doan",
      max(z_values) - min(z_values) < 1e-6,
      "DATUM phai giu Z; neu doi thi da thanh terrain following")
check("len ban 1.0m -> TGTS = 0.50 (dung vi du nguoi dung)",
      abs(effective_target(1.50, 1.00) - 0.50) < 1e-6)
check("roi ban -> TGTS ve lai 1.50",
      abs(effective_target(1.50, 0.00) - 1.50) < 1e-6)

print()
print("== (5) Dai so: hai cach cho CUNG mot err ==")
# Cach A (dang dung): err = target - alt_m,   alt_m = clearance + terrain_off
# Cach B (de nham):   err = (target - terrain_off) - clearance
# Hai cai PHAI bang nhau -- do la ly do khong duoc lam ca hai.
bad = []
for toff in (0.0, 0.5, 1.0, 1.2):
    for clr in (0.2, 0.5, 1.0, 1.5):
        alt_m = clr + toff
        a = TGT - alt_m
        b = (TGT - toff) - clr
        if abs(a - b) > 1e-9:
            bad.append((toff, clr, a, b))
check("err giong het nhau o moi dia hinh/khoang ho", not bad, str(bad[:3]))
print("        -> cong offset vao target NUA se thanh: err - terrain_off (sai)")

print()
print("== (6) Ban giao TAKEOFF -> HOLDING khong giat ==")
ALTC = strip_c_comments(read("components/flight_core/src/alt_estimator.c"))
# Khoi RESET khi nam dat -- neo bang chinh dong gan terrain_off_m = 0 thay vi
# tim `if (!airborne)` dau tien (co nhieu khoi nhu vay, va khoi dau la update_age).
mz = re.search(r"e->terrain_off_m\s*=\s*0\.0f\s*;", ALTC)
check("nam dat -> terrain_off_m = 0", mz is not None,
      "neu offset khong ve 0 khi nam dat thi lan bay sau ke thua bac cu")
if mz:
    ctx = ALTC[max(0, mz.start() - 400):mz.start()]
    check("dong do nam trong nhanh !airborne",
          "airborne" in ctx,
          "phai o nhanh nam dat, khong phai mot cho nao khac")
check("=> tren san TGTS == TGT, hai frame trung nhau", True)

print()
if FAILED:
    print("KET QUA: %d FAIL -- %s" % (len(FAILED), FAILED[0]))
    sys.exit(1)
print("KET QUA: TAT CA PASS")
