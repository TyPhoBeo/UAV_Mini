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
    # Cong TAT: van PHAI chot duoc mot goc toa do (fallback).
    assert "alt_estimator_lock_floor_fallback(&s_alt_est)" in block, \
        "cong floor tat nhung khong co duong fallback chot goc toa do"
    assert "s_tof_ok_driver" in block, \
        "cong floor tat van phai doi ToF driver song"

# =============================================================================
# NAM SAT SAN KHONG DUOC CHAN TAKEOFF
# =============================================================================
# Drone nam tren san -> VL53L1X doc 0.000m (duoi tam mu ~4cm) -> driver loai
# mau (dist_mm > 0 sai) -> floor_add() khong bao gio chay ->
# tof_ground_ref_valid dung o false -> lock_floor VA lock_floor_fallback deu
# tra false. Ban truoc `break` o day, tuc la MOI lenh TAKEOFF bi tu choi
# NO_CORRECTION VINH VIEN khi drone dang o dung noi no phai o. Disarm/arm lai
# khong cuu duoc vi drone van nam nguyen cho.
#
# Gio: khong co mau -> chot goc = 0 va VAN cat canh.
def _strip_comments(src):
    import re
    src = re.sub(r"/\*.*?\*/", "", src, flags=re.S)
    return re.sub(r"//[^\n]*", "", src)

code = _strip_comments(block)

assert "alt_estimator_lock_floor_at_zero(&s_alt_est)" in code, \
    "thieu duong chot goc = 0 -> nam sat san se lai chan takeoff"

# Nhanh fallback KHONG duoc con `break` (tu choi lenh).
i_fb = code.index("alt_estimator_lock_floor_fallback")
i_zero = code.index("alt_estimator_lock_floor_at_zero")
assert "break" not in code[i_fb:i_zero], \
    "nhanh fallback van con `break` -> nam sat san van bi tu choi takeoff"

# Dieu kien tu choi CON LAI phai la loi phan cung THAT (driver khong init duoc),
# khong phai 'chua co mau'.
assert "if (!s_tof_ok_driver)" in code, \
    "phai giu duong tu choi khi ToF khong init duoc (do moi la loi that)"
assert "if (!has_tof_src)" in code, \
    "has_tof_src van phai duoc kiem -- nhung chi de CANH BAO, khong tu choi"
i_has = code.index("if (!has_tof_src)")
tail = code[i_has:i_has + 400]
assert "TAKEOFF_REJECT" not in tail, \
    "nhanh has_tof_src khong duoc tu choi lenh nua, chi duoc canh bao"

print(f"  (FC_FEATURE_FLOOR_GATE={'1 - cong BAT' if floor_gate else '0 - cong TAT, dung fallback'})")
print("PASS: TAKEOFF locks a ToF floor datum (stable window or fallback); no baro fallback")
