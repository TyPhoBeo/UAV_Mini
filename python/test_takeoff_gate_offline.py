"""Takeoff gate is ToF-floor-only; baro-only must be rejected."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
c = (ROOT / "components/flight_core/src/flight_core.c").read_text(encoding="utf-8")
start = c.index("case CMD_TAKEOFF:")
end = c.index("case CMD_LAND:", start)
block = c[start:end]
assert "alt_estimator_floor_ready(&s_alt_est)" in block
assert "alt_estimator_lock_floor(&s_alt_est)" in block
assert "TAKEOFF_REJECT_NO_CORRECTION" in block
assert "has_baro_src" not in block
assert "try_acquire_tof_ground_ref" not in block
assert "TAKEOFF_DEFAULT_TARGET_M" in block

# Nhu test_arm_gate: cac assert tren chi la grep chuoi, van pass ke ca khi cong
# floor da tat (chuoi nam trong khoi #if). Kiem trang thai THAT cua co.
import re
feat = (ROOT / "components/flight_core/include/flight_core/fc_features.h").read_text(encoding="utf-8")
m = re.search(r"#\s*define\s+FC_FEATURE_FLOOR_GATE\s+(\d)", feat)
assert m, "khong tim thay dinh nghia mac dinh cua FC_FEATURE_FLOOR_GATE"
floor_gate = m.group(1) == "1"
assert "#if FC_FEATURE_FLOOR_GATE" in block, "cong floor phai nam sau co FC_FEATURE_FLOOR_GATE"
if not floor_gate:
    # Cong TAT: van PHAI chot duoc mot goc toa do (fallback), va van phai tu
    # choi khi khong co ca mau ToF nao — bay ma khong biet goc toa do la mu.
    assert "alt_estimator_lock_floor_fallback(&s_alt_est)" in block, \
        "cong floor tat nhung khong co duong fallback chot goc toa do"
    assert "s_tof_ok_driver" in block, \
        "cong floor tat van phai doi ToF driver song"
print(f"  (FC_FEATURE_FLOOR_GATE={'1 - cong BAT' if floor_gate else '0 - cong TAT, dung fallback'})")
print("PASS: TAKEOFF locks a ToF floor datum (stable window or fallback); no baro fallback")
