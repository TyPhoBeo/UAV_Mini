"""Checks ToF geometry, surface/innovation gate and reacquisition policy."""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
h = (ROOT / "components/flight_core/include/flight_core/alt_estimator.h").read_text(encoding="utf-8")
c = (ROOT / "components/flight_core/src/alt_estimator.c").read_text(encoding="utf-8")

assert "tof_range_m * rzz" in c
assert "e->tof_vertical_m - e->tof_ground_range_m" in c
assert "fabsf(e->tof_innovation_m) <= ALT_EST_TOF_INNOV_GATE_M" in c
assert "e->tof_reacquire_count >= ALT_EST_TOF_REACQUIRE_SAMPLES" in c
assert "e->tof_dt_s <= ALT_EST_TOF_GAP_DERIV_MAX_MS/1000.0f" in c
assert "fabsf(zt - e->prev_tof_z_m) <= ALT_EST_TOF_DERIV_JUMP_M" in c

def integer(name):
    m = re.search(rf"#define\s+{name}\s+(\d+)", h)
    assert m, name
    return int(m.group(1))

assert integer("ALT_EST_TOF_REACQUIRE_SAMPLES") >= 3
assert integer("ALT_EST_TOF_TRACK_MAX_AGE_MS") < integer("ALT_EST_TOF_LOST_MS")
assert integer("ALT_EST_TOF_BRIDGE_MS") < integer("ALT_EST_TOF_LOST_MS")
print("PASS: ToF geometry/gates/derivative/reacquisition")
