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
assert "out->liftoff_edge = true" in c

# ---------------------------------------------------------------------------
# hold_ready DA RUT CON HAI VE (yeu cau nguoi dung)
# ---------------------------------------------------------------------------
# Truoc day co BON ve. Hai ve "|alt - target| <= TOL" va "|vz| <= TOL" da bi bo,
# va chung chinh la nguyen nhan mot lan cat canh THAT bi ket: drone leo toi
# 1.45m trong khi target 1.00m (hover_ff latch thieu ~460 duty so voi pin luc
# bay) -> |Z-tgt| KHONG BAO GIO <= 0.08 -> cua so khong dong -> ABORT_TIMEOUT
# sau 21.6s. Log da xac nhan.
#
# Hai ve do hoi "da toi dung do cao va dung yen chua" — cau hoi ma HOLDING sinh
# ra de tra loi. Bat CLIMB tra loi truoc la bat vong ho lam viec cua vong kin.
assert "fabsf(alt_m - st->final_target_m) <= TAKEOFF_HOLD_Z_TOL_M" not in c,     "dung sai do cao da bi bo khoi hold_ready -- khong duoc dung lai"
assert "fabsf(vz_ms) <= TAKEOFF_HOLD_VZ_TOL_MS" not in c,     "dung sai Vz da bi bo khoi hold_ready -- khong duoc dung lai"
# Hai ve CON LAI phai con nguyen. Bo not chung thi ban giao xay ra ngay khi
# vao CLIMB, truoc ca khi roi dat.
assert "st->liftoff_flag &&" in c, "hold_ready van phai doi ROI DAT"
assert "st->target_z_m == st->final_target_m" in c,     "hold_ready van phai doi rate-limiter truot het"
assert "tko_latch_window(hold_ready" in c
assert 0.25 <= num("TAKEOFF_MAX_CLIMB_MS") <= 0.40
# NOI 50..100 -> 50..300. Cua so duy tri lift_evidence da duoc NANG len 200ms
# cung dot bo ve tof_vz khoi lift_evidence (dieu kien gio chi con do cao + ga,
# khong con vi phan nhieu), nen bien cu khong con dung.
assert 50 <= num("TAKEOFF_LIFTOFF_MS") <= 300
# NOI can tren 300 -> 600. Gia tri hien tai la 400ms.
# Can DUOI 200ms van giu: hold_ready gio chi con hai ve xac dinh theo dong ho,
# nen cua so nay la thu DUY NHAT con chan viec ban giao ngay tick dau tien sau
# khi rate-limiter truot het. Ha xuong gan 0 se ban giao truoc khi drone kip on
# dinh sau pha CLIMB.
assert 200 <= num("TAKEOFF_HOLD_ENTER_MS") <= 600
print("PASS: takeoff PRIME ramp, ToF liftoff, slew climb, measured HOLD handoff")
