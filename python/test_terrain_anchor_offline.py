"""Offset terrain phai tinh tu MOC NEO, khong phai cong don cac mau vuot nguong.

LOI THAT, do duoc tren log bay (bay qua dia hinh roi VE LAI SAN):

    TOFF ket o +0.165 thay vi 0.

BAN CU cong don `cand -= d_range`, va CHI cong khi |d_range| > TERR_JUMP_THRESH_M.
Moi mau DUOI nguong bi bo im lang du be mat van dang doi. Mep sac di len bat
duoc gan het, mep thoai di xuong bat duoc it hon -> BAT DOI XUNG -> offset khong
ve 0 khi quay lai san. Mo phong mep khong sac cho +0.150 -- trung khop voi log.

He qua khi DI LEN: bac that +0.70 nhung chi bat duoc +0.55 -> alt_m THAP hon
that 0.15m -> PID tuong drone dang thap -> tang ga -> DRONE VOT LEN.

BAN MOI: bac dia hinh la HIEU CUA HAI MUC.
    offset_moi = offset_cu + (anchor_raw - raw_hien_tai)
Mot phep tru, khong phu thuoc bac trai ra may mau hay mau nao vuot nguong.
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
    """Khoi giai thich trich dan chinh cong thuc cu -> phai bo truoc khi kiem."""
    s = re.sub(r"/\*.*?\*/", " ", s, flags=re.S)
    s = re.sub(r"//[^\n]*", " ", s)
    return s


SRC = strip_c_comments(read("components/flight_core/src/alt_estimator.c"))
HDR = read("components/flight_core/include/flight_core/alt_estimator.h")


def const_of(name):
    m = re.search(r"#define\s+" + re.escape(name) + r"\s+([0-9.]+)f?\b", HDR)
    return float(m.group(1)) if m else None


print("== (1) Truong moc neo ton tai va duoc reset ==")
check("header khai bao terr_anchor_raw_m",
      re.search(r"float\s+terr_anchor_raw_m\s*;", HDR) is not None)
# Phai duoc xoa o CA HAI duong khoi tao truoc chuyen bay, neu khong lan bay sau
# ke thua moc cua lan truoc.
for fn in ("lock_floor_at", "alt_estimator_prepare_takeoff"):
    m = re.search(re.escape(fn) + r"[^{]*\{(.*?)\n\}", SRC, re.S)
    check("%s() xoa terr_anchor_raw_m" % fn,
          m is not None and "terr_anchor_raw_m" in m.group(1),
          "khong xoa -> lan bay sau dung moc cu")

print()
print("== (2) KHONG con cong don ==")
check("khong con `terr_cand_offset_m -= residual`",
      re.search(r"terr_cand_offset_m\s*-=", SRC) is None,
      "cong don la chinh cai gay ra TOFF khong ve 0")
check("cand tinh tu (anchor - raw)",
      re.search(r"terr_cand_offset_m\s*=\s*\n?\s*e->terrain_off_m\s*\+\s*"
                r"\(\s*e->terr_anchor_raw_m\s*-\s*raw_agl\s*\)", SRC) is not None,
      "phai la mot phep tru hai muc")

print()
print("== (3) Moc neo lay mau TRUOC bac ==")
check("anchor = prev_tof_vertical_m",
      re.search(r"terr_anchor_raw_m\s*=\s*e->prev_tof_vertical_m", SRC) is not None,
      "lay mau HIEN TAI thi bac da bi tinh thieu mot buoc")

print()
print("== (4) Cap nhat cand chay MOI MAU khi pending ==")
# Neu phep gan cand nam trong `if (|residual| > JUMP)` thi van la cong don tra
# hinh: mau duoi nguong lai bi bo qua.
m = re.search(r"terr_cand_offset_m\s*=\s*\n?\s*e->terrain_off_m\s*\+", SRC)
check("tim thay cho gan cand", m is not None)
if m:
    before = SRC[max(0, m.start() - 400):m.start()]
    # khoi bao quanh phai la `if (e->terr_pending)`, khong phai kiem nguong
    check("nam trong `if (terr_pending)`, khong bi gac boi nguong",
          re.search(r"if\s*\(\s*e->terr_pending\s*\)\s*\{[^{}]*$", before) is not None,
          "gac boi nguong = van bo qua mau duoi nguong")

print()
print("== (5) KHONG xoa confirm_cnt trong khoi chay moi mau ==")
if m:
    after = SRC[m.end():m.end() + 300]
    check("khong co `terr_confirm_cnt = 0` ngay sau phep gan cand",
          re.search(r"terr_confirm_cnt\s*=\s*0", after.split("}")[0]) is None,
          "xoa moi mau -> bo dem khong bao gio vuot 1 -> KHONG BAO GIO COMMIT")

print()
print("== (6) Mo phong: doi chieu hai cach tren cung du lieu ==")
JUMP = const_of("TERR_JUMP_THRESH_M")
N = int(const_of("TERR_CONFIRM_N") or 3)
MAXSTEP = const_of("TERR_MAX_STEP_M")
check("doc duoc hang so", None not in (JUMP, MAXSTEP), "JUMP=%s MAX=%s" % (JUMP, MAXSTEP))


def sim(raw, mode, jump=None):
    if jump is None:
        jump = JUMP
    toff = 0.0
    pend = False
    cand = 0.0
    anchor = 0.0
    cnt = 0
    for a, b in zip(raw, raw[1:]):
        d = b - a
        if not pend and abs(d) > jump:
            pend, anchor, cnt = True, a, 0
        if pend:
            if mode == "anchor":
                cand = toff + (anchor - b)
            elif abs(d) > jump:
                cand -= d
            if abs(d) <= jump:
                cnt += 1
                if cnt >= N:
                    if abs(cand - toff) <= MAXSTEP:
                        toff = cand
                    pend, cnt = False, 0
            else:
                cnt = 0
    return toff


CASES = [
    ("vat rong, mep sac",   [1.00, 1.00, 0.30, 0.28, 0.29, 0.30, 0.29, 0.30, 0.30], 0.70),
    ("vat hep, qua nhanh",  [1.00, 1.00, 0.30, 0.29, 1.00, 1.00, 1.00, 1.00, 1.00], 0.00),
    ("MEP KHONG SAC",       [1.00, 1.00, 0.30, 0.55, 0.85, 1.00, 1.00, 1.00, 1.00], 0.00),
    ("len ban (log THAT)",  [0.890, 0.338, 0.126, 0.119, 0.110, 0.108, 0.112, 0.109], 0.780),
    ("san phang",           [0.89, 0.895, 0.888, 0.901, 0.897, 0.892, 0.899], 0.00),
]
print("  %-22s%9s%9s%9s" % ("kich ban", "cu", "neo", "dung"))
for ten, raw, truth in CASES:
    o, n = sim(raw, "accum"), sim(raw, "anchor")
    print("  %-22s%+9.3f%+9.3f%+9.3f" % (ten, o, n, truth))
    check("moc neo dung: %s" % ten, abs(n - truth) < 0.02,
          "ra %+.3f, mong %+.3f" % (n, truth))

print()
print("== (7) Sai so ban CU phu thuoc NGUONG; ban NEO thi khong ==")
# Cong don bo qua cac buoc DUOI nguong khi thoat -> len bao nhieu tru bay nhieu
# la khong doi xung, va do khong doi xung nay LON DAN theo nguong. Moc neo do
# HAI MUC nen khong quan tam duong di o giua.
mep = CASES[2][1]          # 1.00 -> 0.30 -> 0.55 -> 0.85 -> 1.00, that la 0.00
for j in (0.12, 0.20):
    o, n = sim(mep, "accum", j), sim(mep, "anchor", j)
    print("  nguong %.2f:  cu %+.3f   neo %+.3f   (dung 0.000)" % (j, o, n))
    check("moc neo dung o nguong %.2f" % j, abs(n) < 0.02, "ra %+.3f" % n)

# 0.20 la nguong dang chay khi log ghi TOFF con +0.165 tren san phang.
check("tai hien duoc loi cu tai nguong 0.20 (log THAT: +0.165)",
      abs(sim(mep, "accum", 0.20)) > 0.10,
      "ra %+.3f -- neu khong tai hien thi mo phong sai" % sim(mep, "accum", 0.20))
# O 0.12 buoc thoat 0.15 con vuot nguong nen duoc cong lai -> cong don TINH CO
# dung. Do la may man cua bo du lieu, khong phai tinh chat cua thuat toan.
check("o nguong 0.12 ban cu TINH CO dung -- khong phai da het loi",
      abs(sim(mep, "accum", 0.12)) < 0.02)

print()
if FAILED:
    print("KET QUA: %d FAIL -- %s" % (len(FAILED), FAILED[0]))
    sys.exit(1)
print("KET QUA: TAT CA PASS")
