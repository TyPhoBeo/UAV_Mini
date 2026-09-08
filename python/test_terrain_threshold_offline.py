"""Nguong phat hien terrain + khoa sau khi roi dat.

LOI: nang TERR_JUMP_THRESH_M 0.12 -> 0.20 lam terrain NGUNG hoat dong voi vat
the vua phai. Nguoi dung bay thu: "khong con phan offset khi bay qua dia hinh".

Nguyen nhan: toi lay bien tu dinh nhieu 0.076m -- nhung do TOAN BO tu pha LEO
ngay sau cat canh (range di tu ~0 len gia tri that). Do tach theo pha bay:

    hover                   dinh |d_range| = 0.015
    bay ngang + xoay 70deg  dinh          = 0.019..0.026
    leo ngay sau cat canh   dinh          = 0.076   <-- chi o day

Dung truong hop xau nhat cua MOT pha khong lien quan de dat nguong cho CA
chuyen bay. Va dung pha do thi drone o duoi 0.3m, KHONG THE o tren vat the nao.

SUA: khoa terrain TERR_ARM_AFTER_LIFTOFF_MS dau tien, roi ha nguong ve 0.12.
"""
import io, os, re, sys

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


HDR = read("components/flight_core/include/flight_core/alt_estimator.h")
SRC = re.sub(r"//[^\n]*", "", read("components/flight_core/src/alt_estimator.c"))


def const_of(n):
    m = re.search(r"#define\s+" + re.escape(n) + r"\s+([0-9.]+)f?\b", HDR)
    return float(m.group(1)) if m else None


T = const_of("TERR_JUMP_THRESH_M")
ARM = const_of("TERR_ARM_AFTER_LIFTOFF_MS")

print("== (1) Nguong ==")
check("TERR_JUMP_THRESH_M = 0.12", T == 0.12, "dang la %s" % T)

print()
print("== (2) Co khoa sau khi roi dat ==")
check("hang so TERR_ARM_AFTER_LIFTOFF_MS ton tai", ARM is not None and ARM > 0, str(ARM))
check("struct co airborne_since_us",
      re.search(r"int64_t\s+airborne_since_us\s*;", HDR) is not None)
check("moc duoc dat khi airborne, xoa khi nam dat",
      re.search(r"if\s*\(airborne\)\s*\{[^}]*airborne_since_us\s*=\s*now_us", SRC) is not None
      and re.search(r"else\s+e->airborne_since_us\s*=\s*0", SRC) is not None)
check("cong terrain doi terr_armed",
      re.search(r"airborne\s*&&\s*terr_armed\s*&&", SRC) is not None,
      "khong gac thi pha leo van co the bao dong gia")

# Moc phai duoc dat TRUOC khoi tof_new, neu khong terrain doc mot moc cu mot tick.
i_set = SRC.find("airborne_since_us = now_us")
i_use = SRC.find("terr_armed =")
check("moc duoc dat TRUOC cho terrain doc no",
      i_set != -1 and i_use != -1 and i_set < i_use,
      "set=%d use=%d" % (i_set, i_use))

print()
print("== (3) Nhieu THAT tung pha co vuot nguong khong? ==")


def dd(x):
    return [abs(b - a) for a, b in zip(x, x[1:])]


leo = [0.006, 0.008, 0.023, 0.099, 0.092, 0.110, 0.167, 0.184, 0.220, 0.235, 0.257]
hover = [0.894, 0.891, 0.893, 0.904, 0.889, 0.891, 0.899, 0.895, 0.893, 0.905, 0.912]
xoay = [0.649, 0.665, 0.659, 0.673, 0.680, 0.688, 0.689, 0.697, 0.716, 0.731, 0.737]

for ten, x, khoa in (("hover", hover, False), ("bay ngang + xoay", xoay, False)):
    peak = max(dd(x))
    print("  %-20s dinh %.3f  bien %.1fx" % (ten, peak, T / peak))
    check("%s: khong bao dong gia" % ten, peak < T,
          "dinh %.3f >= nguong %.3f" % (peak, T))

peak_leo = max(dd(leo))
print("  %-20s dinh %.3f  bien %.1fx  <- BI KHOA" % ("leo sau cat canh", peak_leo, T / peak_leo))
# Trung thuc: 0.076 < 0.12, nen tren LOG NAY pha leo khong he bao dong gia.
# Khoa 500ms la PHONG THU, khong phai dieu kien bat buoc: bien o pha leo mong
# hon han bay bang, ma luc do drone chac chan chua o tren vat the nao.
check("pha leo co bien mong hon bay bang it nhat 2 lan",
      peak_leo > 2.0 * max(max(dd(hover)), max(dd(xoay))),
      "neu bien nhu nhau thi khoa khong mua duoc gi")
check("bay bang co bien >= 4x (day moi la con so chong bao dong gia)",
      T / max(max(dd(hover)), max(dd(xoay))) >= 4.0)

print()
print("== (4) Ban THAT trong log van commit dung ==")
N = int(const_of("TERR_CONFIRM_N") or 3)
ban = [0.890, 0.338, 0.126, 0.119, 0.110, 0.108, 0.112]
toff, pend, anchor, cnt = 0.0, False, 0.0, 0
for a, b in zip(ban, ban[1:]):
    d = b - a
    if not pend and abs(d) > T:
        pend, anchor, cnt = True, a, 0
    if pend:
        cand = toff + (anchor - b)
        if abs(d) <= T:
            cnt += 1
            if cnt >= N:
                toff, pend, cnt = cand, False, 0
        else:
            cnt = 0
truth = ban[0] - ban[-1]
print("  TOFF = %.3f   that = %.3f   lech %.1f cm" % (toff, truth, abs(toff - truth) * 100))
check("ban 0.78m commit dung trong 2cm", abs(toff - truth) < 0.02)

print()
print("== (5) Vat the nho hon co bat duoc khong (bac trai 2 mau) ==")
for h, mong in ((0.20, False), (0.25, True), (0.30, True), (0.50, True)):
    got = (h / 2) > T
    print("  cao %.2fm -> buoc %.3f  %s" % (h, h / 2, "BAT DUOC" if got else "lot luoi"))
    check("vat %.2fm: %s" % (h, "phai bat duoc" if mong else "chap nhan lot"), got == mong)

print()
if FAILED:
    print("KET QUA: %d FAIL -- %s" % (len(FAILED), FAILED[0]))
    sys.exit(1)
print("KET QUA: TAT CA PASS")
