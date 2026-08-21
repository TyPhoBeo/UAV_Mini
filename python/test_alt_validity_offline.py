"""Test VALIDITY vs DEGRADED — TRANSCRIPTION khoi cuoi alt_estimator_update()
+ nhanh fault cua commander_evaluate().

Khoa lai thay doi: `valid` KHONG con phu thuoc baro. Tat baro (hoac baro rot vai
tram ms) KHONG duoc lam estimator bi tuyen bo CHET nua — IMU la nguon prediction
chinh. Nhung mat CA HAI nguon correction thi PHAI trip degraded, vi sai so Z
tang BAC HAI: 3s ~0.14m, 10s ~1.5m, 30s ~13.5m.
"""
import math
import sys

DEGRADED_MS = 3000        # ALT_EST_NO_CORRECTION_DEGRADED_MS
FAILS = []


def check(name, cond, extra=""):
    print(("  OK   " if cond else "  FAIL ") + name + (f"   {extra}" if extra else ""))
    if not cond:
        FAILS.append(name)


class Est:
    """Phan VALIDITY/DEGRADED cuoi alt_estimator_update()."""

    def __init__(self):
        self.alt_m = 0.0
        self.vz_ms = 0.0
        self.accel_bias_ms2 = 0.0
        self.valid = False
        self.degraded = True
        self.last_correction_us = 0
        self.no_correction_ms = 0

    def tick(self, now_us, inertial_enabled, got_correction=False):
        if got_correction:
            self.last_correction_us = now_us

        finite = (math.isfinite(self.alt_m) and math.isfinite(self.vz_ms)
                  and math.isfinite(self.accel_bias_ms2))
        if not finite:
            self.alt_m = 0.0
            self.vz_ms = 0.0
            self.accel_bias_ms2 = 0.0
            self.valid = False
        else:
            self.valid = True

        if not inertial_enabled:
            self.last_correction_us = now_us
            self.no_correction_ms = 0
            self.degraded = False
        elif self.last_correction_us == 0:
            self.last_correction_us = now_us
            self.no_correction_ms = 0
            self.degraded = False
        else:
            age = now_us - self.last_correction_us
            self.no_correction_ms = age // 1000
            self.degraded = age > DEGRADED_MS * 1000


# commander: airborne && degraded -> SOFT
def commander_soft(airborne, lost, degraded):
    if lost:
        return "lost"
    if airborne and degraded:
        return "degraded"
    return None


DT_US = 4000   # 250Hz

# ============================================================================
print("TEST A: TAT BARO — estimator VAN valid (bug cu: valid=baro)")
e = Est()
now = 1_000_000
# Bay, chi co ToF correction (baro tat hoan toan)
for i in range(500):        # 2s
    now += DT_US
    got = (i % 8 == 0)      # ToF ~30Hz
    e.tick(now, inertial_enabled=True, got_correction=got)
check("valid = True du KHONG co baro", e.valid)
check("KHONG degraded (ToF van correction)", not e.degraded,
      f"no_corr={e.no_correction_ms}ms")
check("commander KHONG trip", commander_soft(True, not e.valid, e.degraded) is None)

# ============================================================================
print("\nTEST B: MAT CA HAI nguon -> degraded sau nguong, commander LANDING")
e = Est()
now = 1_000_000
for i in range(250):        # 1s co correction
    now += DT_US
    e.tick(now, True, got_correction=(i % 8 == 0))
check("truoc do khong degraded", not e.degraded)
trip_at_ms = None
for i in range(2000):       # 8s KHONG correction nao
    now += DT_US
    e.tick(now, True, got_correction=False)
    if e.degraded and trip_at_ms is None:
        trip_at_ms = e.no_correction_ms
check("VAN valid (state van huu han)", e.valid)
check("degraded = True", e.degraded, f"no_corr={e.no_correction_ms}ms")
check("trip dung quanh nguong 3000ms", trip_at_ms is not None and 3000 <= trip_at_ms <= 3010,
      f"trip o {trip_at_ms}ms")
check("commander -> SOFT (degraded)", commander_soft(True, not e.valid, e.degraded) == "degraded")

# ============================================================================
print("\nTEST C: O DAT (ground lock) — KHONG BAO GIO degraded")
e = Est()
now = 1_000_000
for i in range(15000):      # 60s nam yen, khong correction nao
    now += DT_US
    e.tick(now, inertial_enabled=False, got_correction=False)
check("valid", e.valid)
check("KHONG degraded sau 60s o dat", not e.degraded, f"no_corr={e.no_correction_ms}ms")
check("commander KHONG trip", commander_soft(False, not e.valid, e.degraded) is None)

# ============================================================================
print("\nTEST D: NaN lot vao -> valid=False VA state duoc don sach")
e = Est()
now = 1_000_000
e.tick(now, True, got_correction=True)
e.alt_m = float("nan")
now += DT_US
e.tick(now, True)
check("valid = False khi NaN", not e.valid)
check("alt_m da duoc don ve 0 (khong lan xuong PID)", e.alt_m == 0.0, f"alt={e.alt_m}")
check("commander -> lost", commander_soft(True, not e.valid, e.degraded) == "lost")
# hoi phuc
now += DT_US
e.tick(now, True, got_correction=True)
check("hoi phuc valid sau khi state sach", e.valid)

# ============================================================================
print("\nTEST E: CASE THUC — ToF-only bay qua ban")
# ToF chuyen OTHER -> khong correction nao (baro tat). Phai trip sau ~3s.
e = Est()
now = 1_000_000
for i in range(500):        # bay tren san 2s, ToF correction deu
    now += DT_US
    e.tick(now, True, got_correction=(i % 8 == 0))
check("tren san: khong degraded", not e.degraded)
# len ban: ToF -> OTHER -> KHONG correction
trip = False
for i in range(1250):       # 5s tren ban
    now += DT_US
    e.tick(now, True, got_correction=False)
    if e.degraded:
        trip = True
        break
check("tren ban >3s (ToF-only) -> degraded -> se LANDING", trip,
      f"no_corr={e.no_correction_ms}ms")
print("       => day la GIOI HAN THAT cua ToF-only, khong phai bug.")
print("       => muon bay lau qua ban thi PHAI bat SENSOR_BARO_ENABLED=1")

# ============================================================================
print("\nTEST F: co baro -> qua ban VAN an toan (baro giu watchdog song)")
e = Est()
now = 1_000_000
for i in range(500):
    now += DT_US
    e.tick(now, True, got_correction=(i % 8 == 0))
# len ban: ToF tat, NHUNG baro ~50Hz van correction
for i in range(7500):       # 30s tren ban
    now += DT_US
    e.tick(now, True, got_correction=(i % 5 == 0))   # baro
check("30s tren ban voi baro -> KHONG degraded", not e.degraded,
      f"no_corr={e.no_correction_ms}ms")
check("commander KHONG trip", commander_soft(True, not e.valid, e.degraded) is None)

print()
if FAILS:
    print(f"FAIL: {len(FAILS)} test -> {FAILS}")
    sys.exit(1)
print("PASS: tat ca TEST A..F (validity vs degraded)")
