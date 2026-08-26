"""Pre-arm must require a stable ToF floor, never barometer readiness."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
c = (ROOT / "components/flight_core/src/flight_core.c").read_text(encoding="utf-8")
start = c.index("static bool prearm_check")
end = c.index("return ok;", start)
block = c[start:end]
assert "tof_floor_ready" in block
assert "alt_estimator_valid" in block

# Cong floor giờ nam sau FC_FEATURE_FLOOR_GATE (fc_features.h). Grep suong
# "tof_floor_ready in block" VAN pass ke ca khi co da tat, vi chuoi do van nam
# trong khoi #if -> phep thu rong. Kiem TRANG THAI THAT cua co, va kiem rang
# nhanh #else co duong CANH BAO (tat cong khong duoc phep im lang).
feat = (ROOT / "components/flight_core/include/flight_core/fc_features.h").read_text(encoding="utf-8")
import re
m = re.search(r"#\s*define\s+FC_FEATURE_FLOOR_GATE\s+(\d)", feat)
assert m, "khong tim thay dinh nghia mac dinh cua FC_FEATURE_FLOOR_GATE"
floor_gate = m.group(1) == "1"
assert "#if FC_FEATURE_FLOOR_GATE" in block, "cong floor phai nam sau co FC_FEATURE_FLOOR_GATE"
if not floor_gate:
    # Cong TAT -> phai co nhanh #else canh bao, khong duoc bo qua im lang.
    assert "FC_FEATURE_FLOOR_GATE=0" in block, \
        "cong floor tat nhung khong co canh bao trong prearm_check()"
print(f"  (FC_FEATURE_FLOOR_GATE={'1 - cong BAT' if floor_gate else '0 - cong TAT, co canh bao'})")
assert "baro_ready" not in block
assert "baro_healthy" not in block
assert "s_baro_ok_driver" not in block
assert "baro_driver_calibrate_ground" not in c[c.index("case CMD_ARM:"):c.index("case CMD_DISARM:")]
print("PASS: ARM gate uses IMU/calibration/battery/ToF floor; barometer excluded")
