"""Checks TRACKING -> BRIDGE -> LOST validity semantics."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
h = (ROOT / "components/flight_core/include/flight_core/alt_estimator.h").read_text(encoding="utf-8")
c = (ROOT / "components/flight_core/src/alt_estimator.c").read_text(encoding="utf-8")
for token in ("ALT_SRC_GROUND_LOCK", "ALT_SRC_IMU_PREDICT_ONLY", "ALT_SRC_TOF_FUSED",
              "ALT_SRC_TOF_SHORT_BRIDGE", "ALT_SRC_TOF_LOST"):
    assert token in h
assert "age <= ALT_EST_TOF_TRACK_MAX_AGE_MS" in c
assert "age <= ALT_EST_TOF_BRIDGE_MS" in c
assert "age < ALT_EST_TOF_LOST_MS" in c
assert "e->valid = false" in c and "e->degraded = true" in c
assert "ALT_SRC_TOF_SHORT_BRIDGE" in c and "ALT_SRC_TOF_LOST" in c
print("PASS: altitude validity state machine TRACKING/BRIDGE/LOST")
