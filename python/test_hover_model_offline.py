"""Model ga hover theo dien ap pin + latch mot lan luc ARM.

TRANSCRIPTION cua hover_model.c. Hai muc dich:

  1. Kiem model KHOP voi 2 diem do bench-ramp that cua nguoi dung
     (4.2V -> 900 duty, 3.6V -> 1300..1400 duty). Day la TOAN BO co so
     thuc nghiem cua module -- model 2 tham so khong co bac tu do nao de
     tu phat hien minh sai, nen kiem no o day la lop bao ve DUY NHAT.

  2. Kiem KHONG CO HANG SO NAO BI NHAN DOI giua hover_model.h va tuning.h.
     Yeu cau tuong minh cua nguoi dung. Mot dai luong vat ly -> mot hang so.
"""
import io
import math
import re
import sys
import pathlib

REPO = pathlib.Path(__file__).resolve().parents[1]
FAILS = []


def check(name, cond, extra=""):
    print(("  OK   " if cond else "  FAIL ") + name + (f"   {extra}" if extra else ""))
    if not cond:
        FAILS.append(name)


def dnum(path, name):
    src = io.open(REPO / path, encoding="utf-8").read()
    m = re.search(r"^#define\s+%s\s+\(?([-+0-9.eE]+)f?\)?" % name, src, re.M)
    assert m, f"khong doc duoc {name} trong {path}"
    return float(m.group(1))


def dint(path, name):
    return int(dnum(path, name))


HM = "components/flight_core/include/flight_core/hover_model.h"
TUN = "components/flight_core/include/flight_core/tuning.h"
MOT = "components/flight_core/include/flight_core/drivers/motor_driver.h"

REF_DUTY = dnum(HM, "HOVER_MODEL_REF_DUTY")
V_REF    = dnum(HM, "HOVER_MODEL_V_REF")
EXP      = dnum(HM, "HOVER_MODEL_EXP")
MIN_DUTY = dnum(HM, "HOVER_MODEL_MIN_DUTY")
MIN_V    = dnum(HM, "HOVER_MODEL_MIN_LATCH_V")
NSAMP    = dint(HM, "HOVER_MODEL_SAMPLES")
MINSAMP  = dint(HM, "HOVER_MODEL_MIN_SAMPLES")

# MAX_DUTY la bieu thuc SUY RA, khong phai so -- tinh lai dung cong thuc.
SAFE_MAX  = dnum(MOT, "MOTOR_SAFE_MAX_DUTY")
COLL_FRAC = dnum(TUN, "ATT_MAX_COLLECTIVE_FRACTION")
MAX_DUTY  = SAFE_MAX * COLL_FRAC

PRIME_FRAC = dnum(TUN, "TAKEOFF_PRIME_HOVER_FRAC")
HOVER_NOM  = dnum(TUN, "ALT_HOLD_HOVER_NOMINAL")

print("Hang so:")
print(f"  model      : {REF_DUTY:.0f} * ({V_REF}/V)^{EXP}")
print(f"  clamp      : [{MIN_DUTY:.0f}, {MAX_DUTY:.0f}]  (MAX = {SAFE_MAX:.0f} * {COLL_FRAC})")
print(f"  latch min V: {MIN_V}")
print(f"  prime frac : {PRIME_FRAC}  -> prime khoi tao = {HOVER_NOM * PRIME_FRAC:.0f}")
print(f"  mau        : {NSAMP} (toi thieu {MINSAMP})")
print()


# ---------------- transcription cua hover_model.c ----------------
def hover_from_voltage(v):
    if not (v > 0.1):
        return MIN_DUTY
    d = REF_DUTY * ((V_REF / v) ** EXP)
    if not math.isfinite(d):
        return MIN_DUTY
    return min(max(d, MIN_DUTY), MAX_DUTY)


def prime_duty(hover):
    p = hover * PRIME_FRAC
    return int(0.0 if p < 0.0 else p)


class Ring:
    def __init__(self):
        self.v = [0.0] * NSAMP
        self.count = 0
        self.head = 0
        self.last_seq = 0

    def push(self, v, seq, valid):
        if not valid or seq == 0 or seq == self.last_seq:
            return
        self.last_seq = seq
        self.v[self.head] = v
        self.head = (self.head + 1) % NSAMP
        if self.count < NSAMP:
            self.count += 1

    def median(self):
        if self.count < MINSAMP:
            return None
        s = sorted(self.v[: self.count])
        n = len(s)
        return s[n // 2] if n % 2 == 1 else 0.5 * (s[n // 2 - 1] + s[n // 2])


# ================= TEST A — khop 2 diem do bench-ramp =================
print("TEST A: model khop so do bench-ramp THAT")
h42 = hover_from_voltage(4.2)
check("A1 4.2V -> ~900 duty (diem neo)", abs(h42 - 900.0) < 1.0, f"-> {h42:.1f}")

h36 = hover_from_voltage(3.6)
check("A2 3.6V -> nam trong dai do 1300..1400", 1300.0 <= h36 <= 1400.0, f"-> {h36:.1f}")

# Exponent suy nguoc tu 2 diem phai khop hang so dang dung.
exp_fit = math.log(1350.0 / 900.0) / math.log(4.2 / 3.6)
check("A3 exponent dang dung khop fit tu 2 diem (sai so <2%)",
      abs(EXP - exp_fit) / exp_fit < 0.02, f"dung={EXP} fit={exp_fit:.3f}")

check("A4 hover TANG khi pin TUT (dau cua model dung chieu)",
      hover_from_voltage(3.6) > hover_from_voltage(3.9) > hover_from_voltage(4.2))

# ================= TEST B — clamp + gia tri bien =================
print("\nTEST B: clamp va gia tri bien")
check("B1 pin cao bat thuong bi kep SAN", hover_from_voltage(6.0) == MIN_DUTY,
      f"-> {hover_from_voltage(6.0):.0f}")
check("B2 pin rat thap bi kep TRAN", hover_from_voltage(2.0) == MAX_DUTY,
      f"-> {hover_from_voltage(2.0):.0f}")
check("B3 vbat=0 khong chia-0, tra SAN", hover_from_voltage(0.0) == MIN_DUTY)
check("B4 vbat am tra SAN", hover_from_voltage(-1.0) == MIN_DUTY)

for v in [x / 100.0 for x in range(340, 421)]:
    d = hover_from_voltage(v)
    if not (MIN_DUTY <= d <= MAX_DUTY):
        check(f"B5 ra ngoai clamp tai {v}V", False, f"-> {d}")
        break
else:
    check("B5 moi V trong 3.4..4.2 deu nam trong clamp", True)

# NGUONG LATCH phai la diem model CHUA cham tran -- neu da cham tran o
# dung nguong thi nguong do vo nghia (ARM cho qua mot gia tri da bi kep).
h_min_latch = hover_from_voltage(MIN_V)
check("B6 tai nguong latch toi thieu, model CHUA cham tran",
      h_min_latch < MAX_DUTY - 1.0, f"{MIN_V}V -> {h_min_latch:.0f} (tran {MAX_DUTY:.0f})")

# Va ngay duoi nguong thi PHAI cham tran -- day la LY DO ton tai cua nguong.
h_below = hover_from_voltage(MIN_V - 0.2)
check("B7 ngay DUOI nguong latch thi model DA cham tran (nguong dat dung cho)",
      h_below >= MAX_DUTY - 1.0, f"{MIN_V - 0.2:.1f}V -> {h_below:.0f}")

# ================= TEST C — bat bien PRIME < hover =================
print("\nTEST C: bat bien PRIME phai THAP HON hover")
check("C1 ty le prime < 1.0 (prime KHONG duoc du suc nhac drone)",
      PRIME_FRAC < 1.0, f"PRIME_FRAC={PRIME_FRAC}")

worst = None
for v in [x / 100.0 for x in range(340, 421)]:
    h = hover_from_voltage(v)
    p = prime_duty(h)
    if p >= h:
        worst = (v, h, p)
        break
check("C2 prime < hover o MOI muc pin hop le", worst is None,
      "" if worst is None else f"tai {worst[0]}V: prime={worst[2]} >= hover={worst[1]:.0f}")

# Gia tri KHOI TAO trong tuning.h phai theo CUNG bat bien.
prime_init = int(HOVER_NOM * PRIME_FRAC)
check("C3 TAKEOFF_PRIME_DUTY khoi tao cung < ALT_HOLD_HOVER_NOMINAL",
      prime_init < HOVER_NOM, f"prime_init={prime_init} hover_nom={HOVER_NOM:.0f}")

# ================= TEST D — vong dem vbat =================
print("\nTEST D: vong dem vbat (trung vi, loc theo seq)")
r = Ring()
check("D1 rong -> tu choi latch", r.median() is None)

r.push(3.9, 1, True)
r.push(3.9, 2, True)
check("D2 duoi nguong so mau -> VAN tu choi", r.median() is None, f"count={r.count}")

r.push(3.9, 3, True)
check("D3 du nguong -> chap nhan", r.median() is not None, f"count={r.count}")

# Loc seq: cung mot mau doc lai 25 lan (250Hz / 10Hz) khong duoc dem 25 lan.
r2 = Ring()
for _ in range(25):
    r2.push(3.9, 7, True)
check("D4 cung seq lap 25 lan chi nap 1 o", r2.count == 1, f"count={r2.count}")

# Mau hong bi BO QUA hoan toan, khong xoa lich su cu.
r3 = Ring()
for i, v in enumerate([3.90, 3.88, 3.92, 3.89, 3.91], start=1):
    r3.push(v, i, True)
before = r3.median()
r3.push(0.0, 99, False)
check("D5 mau invalid khong lam doi trung vi", r3.median() == before,
      f"{before:.3f}")

# Trung vi CHONG mau lech, khac han trung binh.
r4 = Ring()
for i, v in enumerate([3.90, 3.90, 1.20, 3.90, 3.90], start=1):
    r4.push(v, i, True)
med = r4.median()
mean = sum([3.90, 3.90, 1.20, 3.90, 3.90]) / 5.0
check("D6 mot mau lech HAN khong keo duoc trung vi", abs(med - 3.90) < 1e-6,
      f"trung vi={med:.2f} (trung binh se la {mean:.2f})")
check("D7 trung binh SE bi keo -> ly do chon trung vi la co that",
      abs(mean - 3.90) > 0.5, f"trung binh={mean:.2f}")

# Hau qua thuc te cua D6/D7 tren ga hover:
check("D8 dung trung binh se chot hover sai HANG TRAM duty",
      hover_from_voltage(mean) - hover_from_voltage(med) > 100.0,
      f"trung vi->{hover_from_voltage(med):.0f} trung binh->{hover_from_voltage(mean):.0f}")

# Ring quay vong: chi giu NSAMP mau gan nhat.
r5 = Ring()
for i in range(1, NSAMP + 4):
    r5.push(3.0 + i * 0.1, i, True)
check("D9 ring bao hoa o NSAMP", r5.count == NSAMP, f"count={r5.count}")

# ================= TEST E — KHONG NHAN DOI HANG SO =================
print("\nTEST E: khong hang so nao bi nhan doi voi tuning.h")
hm_src = io.open(REPO / HM, encoding="utf-8").read()
tun_src = io.open(REPO / TUN, encoding="utf-8").read()

hm_names = set(re.findall(r"^#define\s+(\w+)", hm_src, re.M))
tun_names = set(re.findall(r"^#define\s+(\w+)", tun_src, re.M))
dup = hm_names & tun_names
check("E1 khong ten #define nao trung giua 2 file", not dup, f"trung: {sorted(dup)}")

# MAX_DUTY phai la BIEU THUC suy ra, khong duoc go tay so 1700.
# Define nay noi dong bang '\' nen phai gom het cac dong noi tiep truoc,
# roi moi cat comment -- khong thi body chi la moi dau '\'.
m = re.search(r"#define\s+HOVER_MODEL_MAX_DUTY((?:[^\n]*\\\n)*[^\n]*)", hm_src)
body_raw = m.group(1) if m else ""
body = re.sub(r"//.*", "", body_raw)          # bo comment cuoi dong
check("E2 HOVER_MODEL_MAX_DUTY suy ra tu MOTOR_SAFE_MAX_DUTY + ATT_MAX_COLLECTIVE_FRACTION",
      "MOTOR_SAFE_MAX_DUTY" in body and "ATT_MAX_COLLECTIVE_FRACTION" in body,
      repr(body.strip()))
check("E3 HOVER_MODEL_MAX_DUTY KHONG go tay 1700 trong BIEU THUC",
      "1700" not in body, repr(body.strip()))

# Ty le prime chi duoc dinh nghia MOT lan, va o tuning.h.
check("E4 ty le prime dinh nghia trong tuning.h", "TAKEOFF_PRIME_HOVER_FRAC" in tun_names)
check("E5 hover_model.h KHONG dinh nghia lai ty le prime",
      "TAKEOFF_PRIME_HOVER_FRAC" not in hm_names)
check("E6 hover_model.c DUNG hang so tu tuning.h",
      "TAKEOFF_PRIME_HOVER_FRAC" in io.open(
          REPO / "components/flight_core/src/hover_model.c", encoding="utf-8").read())

# TAKEOFF_PRIME_DUTY phai suy ra tu ty le do, khong go tay he so khac.
m = re.search(r"#define\s+TAKEOFF_PRIME_DUTY\s+(.+)", tun_src)
check("E7 TAKEOFF_PRIME_DUTY suy ra tu TAKEOFF_PRIME_HOVER_FRAC",
      m is not None and "TAKEOFF_PRIME_HOVER_FRAC" in m.group(1), m.group(1) if m else "")

print()
if FAILS:
    print(f"FAILED {len(FAILS)}: " + ", ".join(FAILS))
    sys.exit(1)
print("TAT CA PASS")
