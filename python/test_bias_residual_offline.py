"""Hoc accel-bias Z TU RESIDUAL DO CAO (duong thu HAI, chay LUC BAY).

Duong CU chi hoc khi `stationary` (gyro/accel norm nho) = chi hoc duoc luc con
tren dat. Nhung bias accel Z troi theo NHIET DO, ma nhiet do doi nhieu nhat
chinh luc dang bay (motor nong). Khong co duong nay thi sai so DC cua Az khong
bao gio duoc sua luc bay -> Z troi deu mot chieu suot chuyen bay.

TRANSCRIPTION cua khoi "HOC BIAS ACCEL TU RESIDUAL" trong alt_estimator.c.
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
    assert m, f"khong doc duoc {name}"
    return float(m.group(1))


AE = "components/flight_core/include/flight_core/alt_estimator.h"
K_BIAS      = dnum(AE, "ALT_EST_BIAS_FROM_RESIDUAL_K")
LPF_HZ      = dnum(AE, "ALT_EST_BIAS_RESIDUAL_LPF_HZ")
MAX_AZ      = dnum(AE, "ALT_EST_BIAS_ADAPT_MAX_AZ_MS2")
MIN_TILTCOS = dnum(AE, "ALT_EST_BIAS_ADAPT_MIN_TILT_COS")
MAX_RES     = dnum(AE, "ALT_EST_BIAS_ADAPT_MAX_RESIDUAL_M")
BIAS_CLAMP  = dnum(AE, "ALT_EST_ACCEL_BIAS_MAX_MS2")

print("Hang so doc tu alt_estimator.h:")
print(f"  K              = {K_BIAS}")
print(f"  residual LPF   = {LPF_HZ} Hz")
print(f"  |Az| toi da    = {MAX_AZ} m/s^2")
print(f"  cos tilt toi thieu = {MIN_TILTCOS} (~{math.degrees(math.acos(MIN_TILTCOS)):.0f} do)")
print(f"  |residual| toi da  = {MAX_RES} m")
print(f"  clamp bias     = +-{BIAS_CLAMP} m/s^2")

DT = 0.004


def lpf_alpha(dt, hz):
    return dt / (dt + 1.0 / (2.0 * math.pi * hz))


def clampf(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class Est:
    """Chi phan hoc bias -- KHONG mo phong lai ca estimator."""

    def __init__(self):
        self.bias = 0.0
        self.res_f = 0.0
        self.res_init = False
        self.adapt_count = 0

    def tick(self, residual, have_residual, az_corr, tilt_cos, inertial=True):
        gate = (inertial and have_residual
                and abs(residual) <= MAX_RES
                and abs(az_corr) <= MAX_AZ
                and tilt_cos >= MIN_TILTCOS)
        if not inertial:
            self.res_f = 0.0
            self.res_init = False
            return
        if gate:
            if not self.res_init:
                self.res_f = residual
                self.res_init = True
            else:
                self.res_f += lpf_alpha(DT, LPF_HZ) * (residual - self.res_f)
            self.bias -= K_BIAS * self.res_f * DT
            self.bias = clampf(self.bias, -BIAS_CLAMP, BIAS_CLAMP)
            self.adapt_count += 1


# ============================================================================
print("\nTEST 1 (DAU): sensor lech +0.10 m/s^2 -> bias phai TANG ve +0.10")
# Sensor doc cao hon that -> Az uoc luong cao -> Z bo len -> do cao THAT thap
# hon uoc luong -> innovation = (do - uoc luong) AM.
e = Est()
for _ in range(int(60.0 / DT)):          # 60s bay bang
    e.tick(residual=-0.05, have_residual=True, az_corr=0.0, tilt_cos=1.0)
check("bias di theo chieu DUONG (bu sensor lech duong)", e.bias > 0.0,
      f"bias={e.bias:+.4f}")
# CHU Y: day la VONG HO -- residual bi giu CO DINH mai. Trong thuc te no CO LAI
# khi bias duoc hoc (vong kin), nen bias that chi toi ~0.08 chu khong toi clamp.
# O vong ho, chay toi clamp la HANH VI DUNG: clamp sinh ra chinh de chan viec do.
check("bias khong vuot clamp du residual giu mai (vong ho)",
      abs(e.bias) <= BIAS_CLAMP + 1e-6, f"bias={e.bias:+.4f}")

print("\nTEST 2 (DAU nguoc lai): residual DUONG -> bias phai GIAM")
e = Est()
for _ in range(int(60.0 / DT)):
    e.tick(residual=+0.05, have_residual=True, az_corr=0.0, tilt_cos=1.0)
check("bias di theo chieu AM", e.bias < 0.0, f"bias={e.bias:+.4f}")

print("\nTEST 3 (TOC DO): phai CHAM, khong tranh viec voi alpha-beta")
e = Est()
for _ in range(int(1.0 / DT)):           # chi 1 giay
    e.tick(residual=-0.10, have_residual=True, az_corr=0.0, tilt_cos=1.0)
# Nguong SUY RA TU K, khong phai so cung -- doi K trong tuning thi test tu bam
# theo thay vi do oan. Sau t giay voi residual r: bias ~ K*r*t (bo qua LPF).
_expect = K_BIAS * 0.10 * 1.0
check("sau 1s bias bam dung bac do lon K*r*t (khong nhanh hon)",
      abs(e.bias) <= _expect * 1.2, f"bias={e.bias:+.5f} ky vong<={_expect*1.2:.5f}")
# VA phai CHAM hon nhieu so voi vong vi tri (hang so thoi gian ~0.67s @ alpha=0.03):
# sau 1s bias moi tao ra duoc mot phan nho cua gia toc can thiet.
check("van CHAM hon vong vi tri (khong tranh viec voi alpha-beta)",
      abs(e.bias) < 0.10, f"bias={e.bias:+.5f}")

print("\nTEST 4 (GATE): khong adapt khi dang tang/giam toc manh")
e = Est()
for _ in range(int(10.0 / DT)):
    e.tick(residual=-0.10, have_residual=True, az_corr=MAX_AZ + 0.5, tilt_cos=1.0)
check("Az lon -> KHONG adapt lan nao", e.adapt_count == 0, f"count={e.adapt_count}")
check("bias khong doi", e.bias == 0.0)

print("\nTEST 5 (GATE): khong adapt khi nghieng nhieu")
e = Est()
for _ in range(int(10.0 / DT)):
    e.tick(residual=-0.10, have_residual=True, az_corr=0.0,
           tilt_cos=MIN_TILTCOS - 0.05)
check("nghieng qua nguong -> KHONG adapt", e.adapt_count == 0, f"count={e.adapt_count}")

print("\nTEST 6 (GATE): khong hoc theo OUTLIER (va cham / doi be mat)")
e = Est()
for _ in range(int(10.0 / DT)):
    e.tick(residual=-(MAX_RES + 0.5), have_residual=True, az_corr=0.0, tilt_cos=1.0)
check("residual qua lon -> KHONG adapt", e.adapt_count == 0, f"count={e.adapt_count}")

print("\nTEST 7 (GATE): khong co mau absolute -> khong doan mo")
e = Est()
for _ in range(int(10.0 / DT)):
    e.tick(residual=0.0, have_residual=False, az_corr=0.0, tilt_cos=1.0)
check("khong co mau -> KHONG adapt", e.adapt_count == 0, f"count={e.adapt_count}")

print("\nTEST 8: con tren dat -> xoa bo loc, khong adapt")
e = Est()
for _ in range(int(5.0 / DT)):
    e.tick(residual=-0.10, have_residual=True, az_corr=0.0, tilt_cos=1.0,
           inertial=False)
check("tren dat -> KHONG adapt", e.adapt_count == 0, f"count={e.adapt_count}")
check("bo loc residual bi xoa", e.res_init is False)

print("\nTEST 9 (AN TOAN): bias bi CLAMP, khong bo vo han")
e = Est()
for _ in range(int(3000.0 / DT)):        # 50 phut residual mot chieu
    e.tick(residual=-MAX_RES, have_residual=True, az_corr=0.0, tilt_cos=1.0)
check("bias khong vuot clamp", abs(e.bias) <= BIAS_CLAMP + 1e-6,
      f"bias={e.bias:+.4f} clamp={BIAS_CLAMP}")

print("\nTEST 10 (GIU): mot doan nghieng NGAN khong xoa phan da hoc")
e = Est()
for _ in range(int(30.0 / DT)):
    e.tick(-0.05, True, 0.0, 1.0)
bias_before = e.bias
for _ in range(int(0.5 / DT)):           # 0.5s nghieng
    e.tick(-0.05, True, 0.0, MIN_TILTCOS - 0.05)
check("gate dong tam thoi -> bias GIU NGUYEN, khong ve 0",
      abs(e.bias - bias_before) < 1e-9, f"{bias_before:+.5f} -> {e.bias:+.5f}")
check("bo loc residual VAN con (khong reset)", e.res_init is True)

# ---- Kiem TRUC TIEP tren source C ----
print("\nTEST 11: source C phai co dung duong nay")
c = io.open(REPO / "components/flight_core/src/alt_estimator.c", encoding="utf-8").read()
check("co tru dan (dau AM) khi cap nhat bias",
      "accel_bias_ms2 -= ALT_EST_BIAS_FROM_RESIDUAL_K" in c)
check("chi dung mau DA DUOC CHAP NHAN (so last_*_accept_us voi now)",
      "last_tof_accept_us == now_us" in c and "last_baro_accept_us == now_us" in c)
check("co clamp bias sau khi cap nhat",
      re.search(r"accel_bias_ms2 = clampf\(e->accel_bias_ms2", c) is not None)
_body = c.split("static void reset_flight_state")[1].split(chr(10) + "}")[0]
_code = chr(10).join(ln for ln in _body.split(chr(10))
                     if not ln.strip().startswith("//"))
check("KHONG GAN accel_bias trong reset_flight_state (giu phan da hoc)",
      re.search(r"accel_bias[A-Za-z0-9_]*[ ]*=", _code) is None)

print()
if FAILS:
    print(f"FAIL: {len(FAILS)} test -> {FAILS}")
    sys.exit(1)
print("PASS: tat ca TEST 1..11 (hoc accel-bias Z tu residual do cao)")
