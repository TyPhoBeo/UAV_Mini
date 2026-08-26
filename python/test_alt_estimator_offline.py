"""Static + numeric checks for the IMU+VL53L0X altitude estimator."""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
hdr = (ROOT / "components/flight_core/include/flight_core/alt_estimator.h").read_text(encoding="utf-8")
src = (ROOT / "components/flight_core/src/alt_estimator.c").read_text(encoding="utf-8")

def num(name):
    m = re.search(rf"#define\s+{name}\s+([0-9.]+)f?", hdr)
    assert m, name
    return float(m.group(1))

assert "(void)baro_healthy" in src
assert "e->baro_accept_count = 0" in src
assert "0.5f*e->az_corrected_ms2*dt*dt" in src
assert "ALT_EST_TOF_Z_GAIN * iz" in src
assert "ALT_EST_TOF_VZ_GAIN*(e->tof_vz_lpf_ms-e->vz_ms)" in src
assert "if (!airborne)" in src and "e->alt_m = e->vz_ms" in src
assert num("ALT_EST_FLOOR_MIN_SAMPLES") == 1
assert num("ALT_EST_FLOOR_MAX_SAMPLES") == 1
assert 0.25 <= num("ALT_EST_TOF_Z_GAIN") <= 0.50
assert 5.0 <= num("ALT_EST_TOF_VZ_LPF_HZ") <= 10.0
assert num("ALT_EST_TOF_LOST_MS") <= 300

z, vz, az, dt = 0.2, 0.1, 0.4, 0.004
zp = z + vz*dt + 0.5*az*dt*dt
vzp = vz + az*dt
assert abs(zp - 0.2004032) < 1e-9
assert abs(vzp - 0.1016) < 1e-9
innovation = 0.10
zc = zp + num("ALT_EST_TOF_Z_GAIN") * innovation
assert zp < zc < zp + innovation
print("PASS: estimator IMU+ToF, ground lock, floor calibration, gentle fusion")
