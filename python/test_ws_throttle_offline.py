"""Test phim GIU W/S -> throttle offset momentary.

Yeu cau: GIU W -> throttle +100; NHA -> tra ve DUNG muc truoc do.
Khoa lai 2 thu de vo nhat:
  1. Mat goi "nha phim" (UDP) KHONG duoc de throttle dinh +100 vinh vien.
  2. Offset la TAM THOI — khong duoc cong don vao muc ga da chot.

TRANSCRIPTION cua nhanh FSM_BENCH_RAMP trong flight_core.c.
"""
import io
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

MOTOR_SAFE_MAX_DUTY = 2000
BENCH_OFFSET_STALE_US = 400_000

FAILS = []


def check(name, cond, extra=""):
    print(("  OK   " if cond else "  FAIL ") + name + (f"   {extra}" if extra else ""))
    if not cond:
        FAILS.append(name)


def clampi(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class Bench:
    """s_bench_throttle_duty + s_bench_throttle_offset (flight_core.c)."""

    def __init__(self):
        self.duty = 0
        self.offset = 0
        self.offset_update_us = 0

    def cmd_step(self, delta):          # CMD_BENCH_THROTTLE_STEP
        self.duty = clampi(self.duty + delta, 0, MOTOR_SAFE_MAX_DUTY)

    def cmd_offset(self, off, now_us):  # CMD_BENCH_THROTTLE_OFFSET
        self.offset = clampi(off, -MOTOR_SAFE_MAX_DUTY, MOTOR_SAFE_MAX_DUTY)
        self.offset_update_us = now_us

    def tick(self, now_us):             # nhanh FSM_BENCH_RAMP
        if self.offset != 0 and (now_us - self.offset_update_us) > BENCH_OFFSET_STALE_US:
            self.offset = 0
        return clampi(self.duty + self.offset, 0, MOTOR_SAFE_MAX_DUTY)


DT = 4000   # 250Hz

# ============================================================================
print("TEST 1: GIU W -> +100, NHA -> tra ve DUNG muc cu")
b = Bench()
now = 1_000_000
b.cmd_step(600)                       # ga da chot = 600
base = b.tick(now)
check("truoc khi giu: 600", base == 600, f"={base}")

b.cmd_offset(+100, now)               # nhan W
for _ in range(25):                   # giu 100ms, GUI keepalive moi 100ms
    now += DT
held = b.tick(now)
check("dang giu: 700", held == 700, f"={held}")

b.cmd_offset(0, now)                  # nha W
released = b.tick(now)
check("nha ra: tra ve DUNG 600", released == 600, f"={released}")
check("muc ga chot KHONG bi thay doi", b.duty == 600, f"duty={b.duty}")

# ============================================================================
print("\nTEST 2: giu LAU -> keepalive giu offset song, KHONG tu tat")
b = Bench()
now = 1_000_000
b.cmd_step(800)
b.cmd_offset(+100, now)
ok = True
for i in range(2500):                 # 10s
    now += DT
    if i % 25 == 0:                   # GUI keepalive 100ms
        b.cmd_offset(+100, now)
    if b.tick(now) != 900:
        ok = False
        break
check("giu 10s: van 900 suot", ok)

# ============================================================================
print("\nTEST 3 (AN TOAN): mat goi 'nha phim' -> watchdog tu ve 0")
b = Bench()
now = 1_000_000
b.cmd_step(800)
b.cmd_offset(+100, now)
check("dang giu: 900", b.tick(now) == 900)
# ground-station chet / goi nha bi mat: KHONG co lenh nao nua
trip_us = None
for _ in range(300):                  # 1.2s im lang
    now += DT
    v = b.tick(now)
    if v == 800 and trip_us is None:
        trip_us = now
check("watchdog tra ve 800 (KHONG dinh +100 vinh vien)", b.tick(now) == 800,
      f"={b.tick(now)}")
check("trip sau ~400ms", trip_us is not None, f"trip_us={trip_us}")

# ============================================================================
print("\nTEST 4: offset KHONG cong don (giu-nha nhieu lan)")
b = Bench()
now = 1_000_000
b.cmd_step(500)
for _ in range(5):
    b.cmd_offset(+100, now); now += DT * 10
    b.cmd_offset(0, now);    now += DT * 10
check("sau 5 lan giu-nha: van 500", b.tick(now) == 500, f"={b.tick(now)}")

# ============================================================================
print("\nTEST 5: S (giam) doi xung")
b = Bench()
now = 1_000_000
b.cmd_step(600)
b.cmd_offset(-100, now)
check("giu S: 500", b.tick(now) == 500, f"={b.tick(now)}")
b.cmd_offset(0, now)
check("nha S: tra ve 600", b.tick(now) == 600, f"={b.tick(now)}")

# ============================================================================
print("\nTEST 6: clamp san 0 — khong am")
b = Bench()
now = 1_000_000
b.cmd_step(50)
b.cmd_offset(-100, now)
check("50-100 -> clamp 0, khong am", b.tick(now) == 0, f"={b.tick(now)}")

# ============================================================================
# TEST 7..12 -- HOLDING/FLYING (nhanh MOI trong flight_core.c buoc 9)
# ============================================================================
# Khac han BENCH_RAMP: o day alt_hold DANG lai throttle bang mot vong PI, va
# W/S KHONG con cong duty nua -- no la LENH VAN TOC.
#
# VI SAO DOI (xem ALT_HOLD_WS_VZ_MS trong tuning.h): "+100 duty" la tham quyen
# KHONG CO DON VI -- cung mot phim cho ra toc do leo khac nhau tuy pin day/can,
# tuy khoi luong. Lenh Vz thi co don vi: giu W = leo 0.3 m/s, giong nhau o moi
# muc pin, vi thanh phan I cua vong Vz nuot chenh lech.
#
# Nhanh moi lam DONG THOI bon viec, cac test duoi khoa dung bon viec do:
#   (1) HOAN TAC I ma alt_hold_run() vua cong (sap chay LAI tang trong)
#       -- KHAC HAN ban cu von DONG BANG I suot luc giu phim
#   (2) suy vz_target tu DAU cua offset (do lon 100 KHONG duoc dung)
#   (3) geofence cat LENH theo chieu dang vi pham (chieu nguoc VAN cho di)
#   (4) neo target do cao theo Z hien tai (nha phim = giu tai cho)
ALT_HOLD_MIN_THROTTLE_DUTY = 400
ALT_MAX_M = 3.0
ALT_MIN_M = 0.0

# Doc THANG tu tuning.h -- khong go tay, de doi hang so ben C thi test theo kip.
_TUN = io.open(os.path.join(HERE, "..", "components", "flight_core", "include",
                             "flight_core", "tuning.h"), encoding="utf-8").read()


def _tnum(name):
    mm = re.search(r"^#define\s+%s\s+\(?([-+0-9.eE]+)f?\)?" % name, _TUN, re.M)
    assert mm, "khong doc duoc " + name
    return float(mm.group(1))


WS_VZ_MS = _tnum("ALT_HOLD_WS_VZ_MS")
VZ_LIMIT_MS = _tnum("ALT_HOLD_VZ_LIMIT_MS")
VZ_KP = _tnum("ALT_HOLD_VZ_KP")
VZ_KI = _tnum("ALT_HOLD_VZ_KI")
VZ_ILIMIT = _tnum("ALT_HOLD_VZ_ILIMIT")
HOVER = _tnum("ALT_HOLD_HOVER_NOMINAL")
print("  ALT_HOLD_WS_VZ_MS = %.2f m/s (tran vz_target = %.2f)" % (WS_VZ_MS, VZ_LIMIT_MS))


def clampf(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class Fly:
    """TRANSCRIPTION nhanh FSM_HOLDING/FSM_FLYING sau khi W/S doi sang Vz."""

    def __init__(self, alt=1.0, target=1.0, hold_driving=True, vz=0.0):
        self.alt = alt
        self.target = target
        self.vz = vz               # vz DO DUOC tu alt_estimator
        self.i = 50.0              # I da hoc duoc tu truoc
        self.offset = 0
        self.offset_update_us = 0
        self.hold_driving = hold_driving
        self.vz_target = 0.0       # telemetry VZTGT

    def cmd_offset(self, off, now_us):
        self.offset = clampi(off, -MOTOR_SAFE_MAX_DUTY, MOTOR_SAFE_MAX_DUTY)
        self.offset_update_us = now_us

    def _cascade(self, vz_target, dt):
        """alt_hold_vz_cascade() rut gon: PI tren sai so vz, co clamp I."""
        err = vz_target - self.vz
        self.i = clampf(self.i + VZ_KI * err * dt, -VZ_ILIMIT, VZ_ILIMIT)
        return clampi(int(HOVER + VZ_KP * err + self.i),
                      ALT_HOLD_MIN_THROTTLE_DUTY, MOTOR_SAFE_MAX_DUTY)

    def tick(self, now_us, base_duty=1250, dt=0.004):
        # buoc 4c: watchdog chay o MOI state
        if self.offset != 0 and (now_us - self.offset_update_us) > BENCH_OFFSET_STALE_US:
            self.offset = 0

        i_before = self.i
        # alt_hold_run(): gia lap "I nhich them moi tick" de test bat duoc (1)
        self.i += 7.0
        thr = base_duty
        self.vz_target = 0.0

        if self.offset != 0 and self.hold_driving:
            self.i = i_before                                     # (1) HOAN TAC
            ws_vz = WS_VZ_MS if self.offset > 0 else -WS_VZ_MS    # (2) chi lay DAU
            if ws_vz > 0.0 and self.alt >= ALT_MAX_M:             # (3) geofence
                ws_vz = 0.0
            if ws_vz < 0.0 and self.alt <= ALT_MIN_M:
                ws_vz = 0.0
            thr = self._cascade(ws_vz, dt)
            self.vz_target = ws_vz
            self.target = clampf(self.alt, ALT_MIN_M, ALT_MAX_M)  # (4)
        return thr


print("")
print("TEST 7: HOLDING -- giu W/S ra LENH VAN TOC, khong cong duty")
f = Fly()
now = 1_000_000
base = f.tick(now)
check("chua giu: throttle = alt_hold_run (1250)", base == 1250, "=%d" % base)

f = Fly()
f.cmd_offset(+100, now)
f.tick(now)
check("giu W -> vz_target = +%.2f m/s" % WS_VZ_MS,
      abs(f.vz_target - WS_VZ_MS) < 1e-6, "vz_target=%+.2f" % f.vz_target)

f = Fly()
f.cmd_offset(-100, now)
f.tick(now)
check("giu S -> vz_target = -%.2f m/s" % WS_VZ_MS,
      abs(f.vz_target + WS_VZ_MS) < 1e-6, "vz_target=%+.2f" % f.vz_target)

# DO LON tren day KHONG duoc dung -- chi co DAU. Day la bang chung truc tiep
# rang tham quyen phim khong con phu thuoc con so duty nao.
a, b = Fly(), Fly()
a.cmd_offset(+100, now); a.tick(now)
b.cmd_offset(+2000, now); b.tick(now)
check("do lon offset KHONG anh huong (chi lay DAU)",
      abs(a.vz_target - b.vz_target) < 1e-9,
      "100 -> %+.2f | 2000 -> %+.2f" % (a.vz_target, b.vz_target))

check("lenh phim KHONG vuot tran vz_target chung", WS_VZ_MS <= VZ_LIMIT_MS,
      "%.2f <= %.2f" % (WS_VZ_MS, VZ_LIMIT_MS))

print("")
print("TEST 8 (COT LOI): I PHAI CHAY -- chinh no hoc ga de giu dung van toc")
# Day la DAO NGUOC co y so voi ban ±duty cu (ban do phai DONG BANG I, neu khong
# I tu tru dan dung bang offset va phim het an sau vai giay). O che do Vz thi
# nguoc lai: dong bang I se chan mat thu duy nhat biet cach giu 0.3 m/s.
f = Fly(vz=0.0)                           # drone chua leo -> sai so vz = +0.3
now = 1_000_000
f.cmd_offset(+100, now)
i0 = f.i
for i in range(250):                      # giu 1s
    now += DT
    if i % 25 == 0:
        f.cmd_offset(+100, now)
    f.tick(now)
check("giu 1s: I BO LEN de dap sai so vz (khong bi dong bang)", f.i > i0 + 10,
      "i0=%.0f -> %.0f" % (i0, f.i))
# Va no phai bo len dung nhip Ki*err: 1s * 100 * 0.3 = +30
check("I bo len dung nhip Ki*err*t (~+%.0f sau 1s)" % (VZ_KI * WS_VZ_MS),
      abs((f.i - i0) - VZ_KI * WS_VZ_MS) < 5.0, "thuc te %+.1f" % (f.i - i0))

# Da dat duoc van toc yeu cau -> sai so = 0 -> I DUNG LAI (khong windup).
f3 = Fly(vz=WS_VZ_MS)
now = 1_000_000
f3.cmd_offset(+100, now)
i0 = f3.i
for i in range(250):
    now += DT
    if i % 25 == 0:
        f3.cmd_offset(+100, now)
    f3.tick(now)
check("dat dung van toc -> I DUNG LAI (khong windup)", abs(f3.i - i0) < 1e-6,
      "i0=%.0f -> %.0f" % (i0, f3.i))

print("")
print("TEST 9: nha phim -> giu NGAY do cao dang o, khong giat ve cho cu")
f = Fly(alt=1.0, target=1.0)
now = 1_000_000
f.cmd_offset(+100, now)
f.alt = 1.4                               # drone da leo len trong luc giu W
f.tick(now)
check("dang giu: target neo theo alt hien tai (1.4)", abs(f.target - 1.4) < 1e-6,
      "target=%.2f" % f.target)
f.cmd_offset(0, now)
f.tick(now)
check("nha ra: target VAN 1.4 (giu tai cho, khong ve 1.0)", abs(f.target - 1.4) < 1e-6,
      "target=%.2f" % f.target)
check("nha ra: vz_target ve 0", abs(f.vz_target) < 1e-9, "%+.2f" % f.vz_target)

print("")
print("TEST 10 (AN TOAN): geofence chan LENH, khong chi chan target")
# Ban ±duty cu cong thang vao duty nen tran do cao KHONG co duong nao tac dong
# trong luc dang giu phim -- clamp chi co tac dung SAU khi nha. Gio lenh la van
# toc nen chan duoc dung cho.
f = Fly(alt=3.5, target=1.0)               # da vuot tran 3.0
now = 1_000_000
f.cmd_offset(+100, now)
f.tick(now)
check("tren tran ma giu W -> lenh leo bi CAT ve 0",
      abs(f.vz_target) < 1e-9, "vz_target=%+.2f" % f.vz_target)
check("target van bi clamp ve tran 3.0", abs(f.target - ALT_MAX_M) < 1e-6,
      "target=%.2f" % f.target)
# Chieu NGUOC lai phai VAN di duoc -- luon phai thoat ra khoi bien.
f.cmd_offset(-100, now)
f.tick(now)
check("tren tran ma giu S -> VAN xuong duoc (thoat khoi bien)",
      abs(f.vz_target + WS_VZ_MS) < 1e-6, "vz_target=%+.2f" % f.vz_target)
# Doi xung o san.
f = Fly(alt=0.0, target=0.5)
f.cmd_offset(-100, now)
f.tick(now)
check("cham san ma giu S -> lenh ha bi CAT ve 0",
      abs(f.vz_target) < 1e-9, "vz_target=%+.2f" % f.vz_target)
f.cmd_offset(+100, now)
f.tick(now)
check("cham san ma giu W -> VAN len duoc", abs(f.vz_target - WS_VZ_MS) < 1e-6,
      "vz_target=%+.2f" % f.vz_target)

print("")
print("TEST 11 (AN TOAN): mat alt_hold (hold_driving=0) -> KHONG ra lenh Vz")
f = Fly(hold_driving=False)
now = 1_000_000
f.cmd_offset(+100, now)
thr = f.tick(now)
check("khong ghi de khi alt_hold khong lai throttle", thr == 1250, "=%d" % thr)
check("va vz_target giu 0", abs(f.vz_target) < 1e-9, "%+.2f" % f.vz_target)

print("")
print("TEST 12 (AN TOAN): watchdog stale cung chay o HOLDING")
f = Fly()
now = 1_000_000
f.cmd_offset(+100, now)
f.tick(now)
check("dang giu: co lenh Vz", abs(f.vz_target - WS_VZ_MS) < 1e-6)
for _ in range(300):                      # 1.2s im lang
    now += DT
thr = f.tick(now)
check("mat goi nha phim -> lenh Vz ve 0, khong dinh leo mai",
      abs(f.vz_target) < 1e-9, "vz_target=%+.2f" % f.vz_target)
check("va throttle tra ve cho alt_hold_run", thr == 1250, "=%d" % thr)

print()
if FAILS:
    print(f"FAIL: {len(FAILS)} test -> {FAILS}")
    sys.exit(1)
print("PASS: tat ca TEST 1..12 (W/S throttle offset: BENCH_RAMP + HOLDING/FLYING)")
