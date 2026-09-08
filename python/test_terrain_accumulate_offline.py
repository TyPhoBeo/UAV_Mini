"""BAC DIA HINH KHONG DEN TRONG MOT MAU -- ung vien offset phai CONG DON.

LOI THAT DA SUA (bat duoc tren log bay len ban)
-----------------------------------------------
    TOFV  0.890 -> 0.338 -> 0.126 -> 0.119 -> 0.110    (alt_m coast ~0.86)
    TRES  -0.552 o mau dau, roi tiep tuc am nho dan
    TPEND 0 -> 1          <- DETECT DUNG
    TTMO  0 -> 1          <- nhung HET HAN, khong commit
    FAULT 0 -> 1, MODE 5 -> 4 (LANDING giua chuyen)

Ban cu chot cand_offset MOT LAN o mau dau:
    cand_offset = terrain_off - residual = 0 - (-0.552) = +0.552
roi DONG BANG. Nhung range con truot them ~0.23m o cac mau sau, nen ung vien
loi thoi ngay:
    cand_z = 0.126 + 0.552 = 0.678   vs  alt_m 0.86   -> lech 0.18 > TOL 0.10
-> confirm_cnt reset MOI MAU -> khong bao gio dat TERR_CONFIRM_N
-> het TERR_PENDING_TIMEOUT_MS -> huy -> khong con correction -> soft fault.

Ban moi: moi mau con vuot nguong deu CONG THEM phan cua no vao ung vien. Khi
bac di qua het, residual ve ~0 -> cand ngung doi -> cand_z khop alt_m -> COMMIT.
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EST_C = (ROOT / "components/flight_core/src/alt_estimator.c").read_text(encoding="utf-8")
EST_H = (ROOT / "components/flight_core/include/flight_core/alt_estimator.h").read_text(encoding="utf-8")

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


CODE = strip_c_comments(EST_C)
HDR = strip_c_comments(EST_H)


def const_of(name, src=HDR):
    m = re.search(r"#define\s+%s\s+([0-9.]+)f?" % re.escape(name), src)
    return float(m.group(1)) if m else None


JUMP = const_of("TERR_JUMP_THRESH_M")
TOL = const_of("TERR_CONFIRM_TOL_M")
N = int(const_of("TERR_CONFIRM_N"))
MAXSTEP = const_of("TERR_MAX_STEP_M")
TMO = const_of("TERR_PENDING_TIMEOUT_MS")
DT = 0.040

print("== (1) CODE: cong don, khong ghi de ==")
# ⚠ YEU CAU, KHONG PHAI CACH VIET.
#
# Ban dau test nay doi dung `cand -= residual` (cong don). Cach do da BI THAY
# bang MOC NEO vi chinh no gay ra mot loi khac: `-=` chi chay khi
# |residual| > TERR_JUMP_THRESH_M, nen moi mau DUOI nguong bi bo im lang du be
# mat van dang doi -> bat doi xung -> TOFF khong ve 0 khi quay lai san
# (do duoc tren log: ket o +0.165). Xem test_terrain_anchor_offline.py.
#
# Yeu cau THAT su can bao ve o day van khong doi: ung vien phai BAM theo bac
# qua NHIEU MAU, khong duoc chot mot lan roi dong bang. Ca hai cach deu thoa;
# chi kiem dieu do, khong kiem dau `-=`.
check("ung vien KHONG bi chot mot lan roi dong bang",
      re.search(r"terr_cand_offset_m\s*=\s*e->terrain_off_m\s*-\s*residual", CODE) is None,
      "gan mot lan = ung vien loi thoi ngay khi bac con truot")
check("ung vien duoc cap nhat MOI MAU trong khi pending",
      re.search(r"if\s*\(\s*e->terr_pending\s*\)\s*\{[^{}]*terr_cand_offset_m\s*=",
                CODE, re.S) is not None,
      "phai nam trong khoi chay moi mau, khong bi gac boi nguong")
check("moc thoi gian dat o CANH LEN (trong if (!terr_pending))",
      re.search(r"if\s*\(!e->terr_pending\)\s*\{[^}]*terr_pending_since_us\s*=\s*now_us",
                CODE, re.S) is not None,
      "dat lai moi mau -> timeout khong bao gio het han")

print()
print("== (2) MO PHONG tren DU LIEU THAT tu log ==")


def sim(seq, alt_m, accumulate):
    """Tra (committed, offset, ms). Mo phong dung logic estimator."""
    terr = 0.0
    pend = False
    cand = 0.0
    cnt = 0
    t = 0.0
    t0 = None
    prev = seq[0]
    for v in seq[1:]:
        res = v - prev              # vz_accel ~ 0 khi dang treo
        if abs(res) > JUMP:
            if not pend:
                pend = True
                cand = terr
                t0 = t
            if accumulate:
                cand -= res
            else:
                cand = terr - res   # ban CU
            cnt = 0
        if pend:
            cand_z = v + cand
            cnt = cnt + 1 if abs(cand_z - alt_m) < TOL else 0
            if cnt >= N and abs(cand - terr) <= MAXSTEP:
                return (True, cand, t * 1000)
            if (t - t0) * 1000 > TMO:
                return (False, None, t * 1000)
        prev = v
        t += DT
    return (False, None, t * 1000)


REAL = [0.890, 0.338, 0.126, 0.119, 0.110, 0.110, 0.110, 0.110]
ALT = 0.86

ok_old, _, _ = sim(REAL, ALT, accumulate=False)
check("ban CU KHONG commit duoc (tai hien dung loi)", not ok_old,
      "neu ban cu cung commit thi test nay khong chung minh duoc gi")

ok_new, off_new, ms_new = sim(REAL, ALT, accumulate=True)
check("ban MOI COMMIT duoc", ok_new)
if ok_new:
    check("commit trong han (%.0fms < %.0fms)" % (ms_new, TMO), ms_new < TMO)
    check("offset hop ly (0.5..1.0m), duoc %.3f" % off_new, 0.5 <= off_new <= 1.0)

print()
print("== (3) KHONG bao gia tren nhieu ToF binh thuong ==")
import random
random.seed(7)
for amp in (0.01, 0.02, 0.03):
    noise = [0.90 + random.uniform(-amp, amp) for _ in range(15)]
    biggest = max(abs(noise[i + 1] - noise[i]) for i in range(len(noise) - 1))
    ok, _, _ = sim(noise, 0.90, accumulate=True)
    check("nhieu +/-%.0fcm (buoc max %.3f < nguong %.2f) -> KHONG commit"
          % (amp * 100, biggest, JUMP),
          not ok,
          "bao gia: nhieu binh thuong bi coi la bac dia hinh")

print()
print("== (4) Sanity van chan duoc bac vo ly ==")
# Bac gia 3m: cong don se vuot TERR_MAX_STEP_M -> khong duoc commit
absurd = [0.90, 0.90 - 1.6, 0.90 - 3.0] + [0.90 - 3.0] * 6
ok, off, _ = sim(absurd, 0.90, accumulate=True)
check("bac ~3m bi TU CHOI (TERR_MAX_STEP_M=%.1f)" % MAXSTEP, not ok,
      "commit duoc offset %s = ghi mot so vo ly vao trang thai ben vung" % off)

print()
if fails:
    print("KET QUA: %d FAIL -- %s" % (len(fails), ", ".join(fails)))
    sys.exit(1)
print("KET QUA: ALL PASS -- bac truot nhieu mau van commit, nhieu khong bao gia")
