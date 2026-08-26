"""Baro debug calibration remains isolated from ARM and estimator reset."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
c = (ROOT / "components/flight_core/src/flight_core.c").read_text(encoding="utf-8")
arm = c[c.index("case CMD_ARM:"):c.index("case CMD_DISARM:")]
baro = c[c.index("case CMD_CALIB_BARO_GROUND:"):c.index("case CMD_MAG_SELFTEST:")]
assert "baro_driver_calibrate_ground" not in arm
assert "alt_estimator_reanchor" not in arm
assert "baro_driver_calibrate_ground" in baro
assert "alt_estimator_reanchor" not in baro
assert "DEBUG ONLY" in baro
print("PASS: baro calibration is explicit debug-only and cannot reset flight Z/Vz")
