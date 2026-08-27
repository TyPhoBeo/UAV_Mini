"""Offline acceptance checks for the MPU6050 fresh gyro-calibration path.

This does not replace the stationary 20--30 s hardware bench test.  It proves
the sign/unit equations with representative data and guards the source wiring
that previously allowed the manual calibration path to reuse corrected gyro.
"""
import re
from pathlib import Path
from statistics import fmean, stdev


ROOT = Path(__file__).resolve().parents[1]
CORE = (ROOT / "components/flight_core/src/flight_core.c").read_text(encoding="utf-8")
DRIVER = (ROOT / "components/flight_core/src/drivers/imu_driver.c").read_text(encoding="utf-8")
HUB = (ROOT / "components/flight_core/src/sensor_hub.c").read_text(encoding="utf-8")
ATTITUDE = (ROOT / "components/flight_core/src/attitude_control.c").read_text(encoding="utf-8")


def welford(samples):
    count = 0
    mean = [0.0, 0.0, 0.0]
    m2 = [0.0, 0.0, 0.0]
    for sample in samples:
        count += 1
        for axis in range(3):
            delta = sample[axis] - mean[axis]
            mean[axis] += delta / count
            m2[axis] += delta * (sample[axis] - mean[axis])
    std = [(value / (count - 1)) ** 0.5 for value in m2]
    return mean, std


# Mandatory current-bug example: bias has the SAME sign as stationary RAW.
base = (0.08, -2.78, -1.94)
collect = [
    tuple(v + noise for v, noise in zip(base, n))
    for n in ((0.01, -0.01, 0.00), (-0.01, 0.01, 0.01),
              (0.00, 0.00, -0.01), (0.01, 0.01, 0.00),
              (-0.01, -0.01, 0.00))
]
bias, raw_std = welford(collect)

# Independent samples: validation does not reuse the collect window.
validation_raw = [
    tuple(v + noise for v, noise in zip(base, n))
    for n in ((0.00, 0.01, -0.01), (0.01, -0.01, 0.01),
              (-0.01, 0.00, 0.00), (0.00, 0.00, 0.00))
]
corrected = [tuple(raw[i] - bias[i] for i in range(3)) for raw in validation_raw]
corr_mean = [fmean(row[i] for row in corrected) for i in range(3)]
corr_std = [stdev(row[i] for row in corrected) for i in range(3)]

for axis in range(3):
    assert abs(bias[axis] - base[axis]) < 0.02
    assert abs(corr_mean[axis]) < 0.02
    assert raw_std[axis] < 0.25
    assert corr_std[axis] < 0.25

# Source-level invariants: RAW is accumulated; validation is explicit
# raw-candidate; the obsolete corrected-plus-old-bias accumulator is gone.
assert "gyro_cal_window_add(&s_gcal_win, imu->gyro_raw_dps" in CORE

# ---------------------------------------------------------------------------
# PHA VALIDATE DA BI BO (yeu cau nguoi dung: "chi lay mau va tinh ra bias thoi")
# ---------------------------------------------------------------------------
# Truoc day co mot cua so DOC LAP thu sau COLLECT de kiem GCORR mean ~ 0 truoc
# khi commit.  Gio COLLECT tinh mean(gyro_raw_dps) roi commit THANG.
#
# ⚠ CAI MAT: khong con phep thu tu dong chung minh "tru bias xong thi con lai
# ~0".  Bang chung do gio nam O HAI CHO KHAC, va ca hai PHAI con:
#   - phan tinh toan ngay tren day (raw mean -> bias -> residual ~ 0)
#   - pre-arm ARM_REJECT_GYRO_BIAS_LARGE do gyro corrected THAT luc DISARMED
assert "GCAL_VALIDATE" not in CORE,     "pha VALIDATE da bi bo -- khong duoc dung lai"
assert "s_gcal_candidate_bias = raw_mean;" in CORE,     "COLLECT phai chot bias = mean(raw) roi commit thang"
assert "s_calib_gyro_sum" not in CORE
assert "gyro_bias_dps.x +=" not in CORE
assert "gyro_bias_dps.y +=" not in CORE
assert "gyro_bias_dps.z +=" not in CORE

# Exactly one runtime subtraction site, before the shared sensor->body helper.
subtracts = [
    "out->gyro_raw_dps.x - calib->gyro_bias_dps.x",
    "out->gyro_raw_dps.y - calib->gyro_bias_dps.y",
    "out->gyro_raw_dps.z - calib->gyro_bias_dps.z",
]
assert all(DRIVER.count(expr) == 1 for expr in subtracts)
assert DRIVER.index(subtracts[0]) < DRIVER.index("sensor_to_body(out->gyro_corrected_sensor_dps)")

# Sensor hub is the sole runtime MPU reader; calibration consumes its snapshot.
assert HUB.count("imu_driver_read(&calib, &imu)") == 1
assert "imu_driver_read(&" not in CORE

# Both fusion and the rate controller consume the same corrected body sample.
assert "mahony_update(&s_mahony, imu.gyro_dps" in CORE
assert "ain.gyro_roll_dps = imu.gyro_dps.x" in CORE
assert "ain.gyro_pitch_dps = imu.gyro_dps.y" in CORE
assert "ain.gyro_yaw_dps = imu.gyro_dps.z" in CORE
assert "in->gyro_yaw_dps" in ATTITUDE

# ---------------------------------------------------------------------------
# NVS gyro bias is PERSISTENT AUTHORITATIVE: calibrate once, then only load.
# ---------------------------------------------------------------------------
# This block previously asserted the OPPOSITE (NVS = diagnostics only, fresh
# calibration mandatory every boot, PX4 style).  The user reversed that call
# deliberately: "calib lan dau tien, roi luu vao nvs.  Moi khi boot thi load
# phan do de bay, khong con 1 chut luong nao phai calib lai sau lan do."
#
# Trade-off now locked in: boot is instant and never blocked by a moving
# vehicle, but the bias no longer tracks temperature drift.  The guards below
# are what keep that acceptable -- they must not be removed together with the
# mandatory-recalibration flow.
boot = CORE[CORE.index("esp_err_t flight_core_start"):]

# 1. A valid NVS bias must reach the runtime struct that sensor_hub_start()
#    receives.  Without this the load is decorative and yaw drifts exactly as
#    it did before any of this work.
assert "s_imu_calib.gyro_bias_dps = s_calib.gyro_bias_dps" in boot
assert "s_calib.gyro_valid = true" in boot

# 2. Calibration must be CONDITIONAL.  Calling gyro_calibration_start()
#    unconditionally would wipe the freshly loaded bias, because the first
#    thing it does is gyro_cal_set_runtime_bias(vec3f_zero()).
assert "if (!s_calib.gyro_valid) {" in boot
start_idx = boot.index("gyro_calibration_start();")
guard_idx = boot.index("if (!s_calib.gyro_valid) {")
assert guard_idx < start_idx, "gyro_calibration_start() phai nam SAU cong !gyro_valid"

# 3. Empty NVS still calibrates -- first boot of a new board must work.
assert "gyro_calibration_start();" in boot

# 4. Mahony reset flag must be pre-satisfied on the NVS path, otherwise the
#    control loop waits forever for a reset that never comes (the reset is
#    gated on GCAL_PASS, and no calibration runs at all here).
assert "s_gcal_reset_done = true;" in boot

# 5. SAFETY NETS THAT MUST SURVIVE.  A stale NVS bias is only acceptable while
#    the pre-arm path still measures the REAL corrected gyro and refuses to arm
#    when it is off -- that is what catches temperature drift.
assert "s_prearm.gyro_calibrated = s_calib.gyro_valid;" in CORE

# ⚠ PHAI kiem TRANG THAI THAT, khong grep suong: hai cong duoi day tung bi
# comment out ca khoi, va grep "ARM_REJECT_GYRO_BIAS_LARGE in CORE" VAN PASS vi
# chuoi do nam trong comment -> phep thu rong, khoa mot lop bao ve khong ton tai.
def strip_c_comments(text):
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return re.sub(r"//[^\n]*", "", text)

CORE_CODE = strip_c_comments(CORE)

GYRO_BIAS_GUARD_ACTIVE = "ARM_REJECT_GYRO_BIAS_LARGE" in CORE_CODE
if not GYRO_BIAS_GUARD_ACTIVE:
    # Khong assert fail: nguoi dung CO QUYEN tat cong nay (da tung yeu cau bo
    # bot rang buoc ARM). Nhung phai NOI TO, vi no la lop DUY NHAT con lai bat
    # duoc bias NVS da troi theo nhiet do -- ma bias NVS gio la authoritative.
    print("  ⚠ CANH BAO: ARM_REJECT_GYRO_BIAS_LARGE DANG BI TAT (comment out).")
    print("    Bias NVS cu se KHONG con duoc kiem lai luc ARM -> bias troi theo")
    print("    nhiet do se di thang vao chuyen bay ma khong co gi chan.")
    print("    Bu lai bang: theo doi co gyro_cal_temp_warn, hoac go 'calib_gyro'")
    print("    khi nhiet do lech nhieu so voi luc calib.")
else:
    # Cong DANG BAT. Kiem nguong con nam trong khoang co nghia.
    TUNING = (ROOT / "components/flight_core/include/flight_core/tuning.h").read_text(encoding="utf-8")
    m_thr = re.search(r"#define\s+PREARM_GYRO_MAX_MEAN_DPS\s+([\d.]+)f", TUNING)
    assert m_thr, "thieu PREARM_GYRO_MAX_MEAN_DPS"
    thr = float(m_thr.group(1))
    # Can duoi: phai bat duoc loi da tung gap that (Gz = -1.94 dps, bias chua ap).
    # Nguong >= 1.94 se cho chinh cai bug goc lot qua -> vo nghia.
    assert thr < 1.94, (
        "PREARM_GYRO_MAX_MEAN_DPS=%.2f >= 1.94: se KHONG bat duoc chinh loi goc "
        "(Gz=-1.94 dps khi bias chua duoc ap)" % thr)
    # Can tren: qua chat thi drone bi tu choi ARM vi nhieu binh thuong.
    assert thr >= 0.10, "nguong qua chat, nhieu gyro binh thuong se chan ARM"
    print(f"  (ARM_REJECT_GYRO_BIAS_LARGE DANG BAT, nguong={thr:.2f} dps"
          f" -> yaw troi toi da ~{thr*60:.0f} deg/phut neu lech kich nguong)")
# ...and the temperature warning stays the user-facing signal to re-run it.
assert "gyro_cal_temp_warn" in CORE

# 6. The manual escape hatch must remain: it is now the ONLY way to re-measure.
assert "case CMD_CALIB_GYRO" in CORE

print("PASS: raw mean -> same-sign bias -> independent raw-bias residual near zero")
print(f"GRAW mean={tuple(round(v, 4) for v in bias)}")
print(f"GBIAS    ={tuple(round(v, 4) for v in bias)}")
print(f"GCORR mean={tuple(round(v, 4) for v in corr_mean)}")
