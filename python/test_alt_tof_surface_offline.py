"""Checks ToF geometry, surface/innovation gate and reacquisition policy."""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
h = (ROOT / "components/flight_core/include/flight_core/alt_estimator.h").read_text(encoding="utf-8")
c = (ROOT / "components/flight_core/src/alt_estimator.c").read_text(encoding="utf-8")

assert "tof_range_m * rzz" in c
assert "e->tof_vertical_m - e->tof_ground_range_m" in c
assert "e->tof_reacquire_count >= ALT_EST_TOF_REACQUIRE_SAMPLES" in c
assert "e->tof_dt_s <= ALT_EST_TOF_GAP_DERIV_MAX_MS/1000.0f" in c
assert "fabsf(zt - e->prev_tof_z_m) <= ALT_EST_TOF_DERIV_JUMP_M" in c

# ---------------------------------------------------------------------------
# INNOVATION GATE DA BI BO — thay bang slew-rate limit.
# ---------------------------------------------------------------------------
# Khoi nay TRUOC DAY khoa dung hanh vi nguoc lai. Gate cu bien "bay qua vat the"
# thanh "tu dong ha canh": range tut dot ngot -> mau bi vut -> tof_fusable=false
# -> sau ALT_EST_TOF_LOST_MS thi valid=false -> Commander soft-fault -> LANDING.
# Da quan sat duoc tren bo. Nguoi dung yeu cau bo gate.
assert "ALT_EST_TOF_INNOV_GATE_M" not in c, \
    "innovation gate da duoc bo -- khong duoc dung lai trong code"
assert "e->tof_surface_gate_ok = true;" in c, \
    "surface gate phai luon mo sau khi bo innovation gate"

# Slew-rate limit la bo loc chong nhay DUY NHAT con lai tren duong ToF.
# No phai chay tren THOI GIAN THAT (tof_dt_s), khong phai tren so mau: nhip ToF
# doi theo chip (L0X 33ms vs L1X 40ms) nen tinh theo mau se cho toc do gioi han
# khac nhau voi CUNG mot hang so.
assert "ALT_EST_TOF_MAX_SLEW_MS * e->tof_dt_s" in c, \
    "slew limit phai nhan voi tof_dt_s (thoi gian that), khong dem theo mau"

m_slew = re.search(r"#define\s+ALT_EST_TOF_MAX_SLEW_MS\s+([\d.]+)f", h)
assert m_slew, "thieu ALT_EST_TOF_MAX_SLEW_MS"
slew = float(m_slew.group(1))
m_vzlim = re.search(r"#define\s+ALT_HOLD_VZ_LIMIT_MS\s+([\d.]+)f",
                    (ROOT / "components/flight_core/include/flight_core/tuning.h")
                    .read_text(encoding="utf-8"))
if m_vzlim:
    # Slew limit KHONG duoc chan chuyen dong THAT cua drone. Neu no thap hon toc
    # do leo/ha ma alt_hold duoc phep ra lenh thi ToF se tut lai phia sau drone
    # va estimator bao mot do cao cham hon thuc te -- sai theo huong nguy hiem.
    assert slew > float(m_vzlim.group(1)), (
        "ALT_EST_TOF_MAX_SLEW_MS (%.2f) phai LON HON ALT_HOLD_VZ_LIMIT_MS (%.2f): "
        "bo loc khong duoc cham hon chuyen dong that cua drone" % (slew, float(m_vzlim.group(1))))

def integer(name):
    m = re.search(rf"#define\s+{name}\s+(\d+)", h)
    assert m, name
    return int(m.group(1))

assert integer("ALT_EST_TOF_REACQUIRE_SAMPLES") >= 3
assert integer("ALT_EST_TOF_TRACK_MAX_AGE_MS") < integer("ALT_EST_TOF_LOST_MS")
assert integer("ALT_EST_TOF_BRIDGE_MS") < integer("ALT_EST_TOF_LOST_MS")
print("PASS: ToF geometry/gates/derivative/reacquisition")
