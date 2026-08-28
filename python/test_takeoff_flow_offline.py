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
# hold_ready: 2 ve XAC DINH + 1 ve CHONG VOT LO (co tran thoi gian)
# ---------------------------------------------------------------------------
# LICH SU (doc ky truoc khi sua):
# Ban dau co BON ve. Ve "|alt - target| <= TOL" da lam mot lan cat canh THAT
# bi ket: drone leo toi 1.45m khi target 1.00m (hover_ff latch thieu ~460 duty)
# -> dieu kien do cao KHONG BAO GIO dung -> ABORT_TIMEOUT sau 21.6s.
#
# Ve do bi bo VINH VIEN va KHONG duoc dung lai: no hoi "da toi DUNG DO CAO
# chua", cau hoi ma HOLDING sinh ra de tra loi.
assert 'fabsf(alt_m - st->final_target_m) <= TAKEOFF_HOLD_Z_TOL_M' not in c, \
    'dung sai DO CAO da bi bo khoi hold_ready -- KHONG duoc dung lai (ket 21.6s)'

# Hai ve XAC DINH (khong phu thuoc cam bien do cao) phai con nguyen.
assert 'st->liftoff_flag &&' in c, 'hold_ready van phai doi ROI DAT'
assert 'st->target_z_m == st->final_target_m' in c, \
    'hold_ready van phai doi rate-limiter truot het'
assert 'tko_latch_window(hold_ready' in c

# --- VE THU BA: |vz| da lang (chong vot lo) ---
# KHAC ve do cao da bi bo: ve nay hoi "con dang di len nhanh khong", va vz LUON
# ve gan 0 sau khi target ngung truot, bat ke drone dung o cao do nao.
#
# ⚠ BAT BUOC di kem TRAN THOI GIAN. Khong co tran thi day lai la mot dieu kien
# co the khong bao gio dung -- dung cai bay da lam ket 21.6s.
assert 'fabsf(vz_ms) <= TAKEOFF_HOLD_VZ_TOL_MS' in c, \
    'thieu ve vz da lang -> ban giao khi con dang leo -> vot lo'
assert 'settle_timeout' in c, \
    've vz KHONG duoc thieu tran thoi gian -- se lap lai lan ket 21.6s'
assert 'TAKEOFF_VZ_SETTLE_TIMEOUT_MS' in c
assert 'vz_settled || settle_timeout' in c, \
    'tran thoi gian phai OR voi ve vz, khong phai AND'

# Tran cho vz phai NGAN. (TAKEOFF_TOTAL_TIMEOUT_MS da bi bo khoi firmware —
# chi con TAKEOFF_NO_LIFT_TIMEOUT_MS cho pha chua nhac noi.) Chan tren 3s: dai
# hon the thi nguoi lai cam thay 'takeoff bi treo' truoc khi tran kip cuu.
assert num('TAKEOFF_VZ_SETTLE_TIMEOUT_MS') <= 3000, \
    'tran cho vz qua dai -> nhin nhu takeoff bi treo'

# Toc do leo: cang cham cang it vot lo. Khong go cung mot khoang hep -- day la
# num chinh nguoi dung hay doi. Chi chan hai dau vo ly.
assert 0.05 <= num('TAKEOFF_MAX_CLIMB_MS') <= 0.60, \
    'TAKEOFF_MAX_CLIMB_MS ngoai khoang hop ly'# NOI 50..100 -> 50..300. Cua so duy tri lift_evidence da duoc NANG len 200ms
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
