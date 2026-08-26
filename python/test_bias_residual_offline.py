"""Checks accel-bias adaptation can only run with trusted ToF or stationary ground."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
c = (ROOT / "components/flight_core/src/alt_estimator.c").read_text(encoding="utf-8")
assert "(!airborne && stationary)" in c
assert "airborne && e->tof_fusable && e->tof_vz_valid" in c
assert "if (bias_ok)" in c
assert "e->bias_adapt_count++" in c
assert "ALT_EST_ACCEL_BIAS_LIMIT_MS2" in c
assert "baro_healthy" not in c[c.index("const bool bias_ok"):c.index("if (!airborne)", c.index("const bool bias_ok"))]
print("PASS: accel bias trust gates and clamp; barometer excluded")
