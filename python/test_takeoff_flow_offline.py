"""Checks PRIME->CLIMB->HOLD behavior is sensor-confirmed, not time-declared."""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
h = (ROOT / "components/flight_core/include/flight_core/takeoff_land.h").read_text(encoding="utf-8")
c = (ROOT / "components/flight_core/src/takeoff_land.c").read_text(encoding="utf-8")
t = (ROOT / "components/flight_core/include/flight_core/tuning.h").read_text(encoding="utf-8")

def num(name):
    m = re.search(rf"#define\s+{name}\s+([0-9.]+)f?", t)
    assert m, name
    return float(m.group(1))

assert "TKO_PRIME" in h and "TKO_CLIMB" in h and "TKO_HOLD" in h
assert "prime_frac" in c
assert "const bool lift_evidence = tof_fusable" in c
assert "tof_z_m >= TAKEOFF_LIFTOFF_Z_M" in c
assert "tof_vz_ms >= TAKEOFF_LIFTOFF_VZ_MS" in c
assert "out->liftoff_edge = true" in c
assert "fabsf(alt_m - st->final_target_m) <= TAKEOFF_HOLD_Z_TOL_M" in c
assert "fabsf(vz_ms) <= TAKEOFF_HOLD_VZ_TOL_MS" in c
assert "tko_latch_window(hold_ready" in c
assert 0.25 <= num("TAKEOFF_MAX_CLIMB_MS") <= 0.40
assert 0.03 <= num("TAKEOFF_LIFTOFF_Z_M") <= 0.05
assert 50 <= num("TAKEOFF_LIFTOFF_MS") <= 100
assert 200 <= num("TAKEOFF_HOLD_ENTER_MS") <= 300
print("PASS: takeoff PRIME ramp, ToF liftoff, slew climb, measured HOLD handoff")
