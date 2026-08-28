"""Chuoi FSM tu ARM -> TAKEOFF -> HOLDING -> FLYING -> LANDING, va ma MODE=
ma GUI nhan duoc o TUNG buoc.

VI SAO CAN TEST NAY:
Nguoi dung bao "GUI khong nhan duoc dang o HOLD hay FLYING". Dieu tra cho thay
GUI khong sai — trong MOI log thuc te, MODE chi tung la 3 (TAKING_OFF) roi 4
(LANDING), CHUA BAO GIO la 2 (HOLDING) hay 5 (FLYING). Nghia la drone chua bao
gio ban giao xong takeoff.

Test nay khoa hai thu KHAC NHAU ma truoc day bi lan lon:
  (1) LUAT CHUYEN STATE  — dung o tang FSM (flight_state_machine.c)
  (2) MA MODE= tren day  — telemetry_format_alt_mode() (telemetry_format.c)
Sai o (1) thi drone khong bao gio vao duoc state; sai o (2) thi vao roi ma GUI
van khong biet. Hai loi khac han nhau, trieu chung tren man hinh GIONG NHAU.

KHONG thay the bay thu that: day la mo phong luat chuyen tiep doc tu source,
khong phai mo phong dong luc hoc bay.
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FSM_C = (ROOT / "components/flight_core/src/flight_state_machine.c").read_text(encoding="utf-8")
FMT_C = (ROOT / "src/telemetry_format.c").read_text(encoding="utf-8")
CORE_C = (ROOT / "components/flight_core/src/flight_core.c").read_text(encoding="utf-8")
GUI_PY = (ROOT / "tools/uav_udp_console.py").read_text(encoding="utf-8")

fails = []


def check(name, cond, detail=""):
    if cond:
        print("  PASS  %s" % name)
    else:
        print("  FAIL  %s%s" % (name, ("  -- " + detail) if detail else ""))
        fails.append(name)


# =============================================================================
# (1) LUAT CHUYEN STATE — trich TRUC TIEP tu source, khong go tay lai
# =============================================================================
# Doc luat tu file C thay vi chep lai: chep lai nghia la test se van xanh khi ai
# do doi luat trong firmware ma quen doi test — dung loai test vo dung nhat.
def fsm_rule(fn_name):
    """Tra ve list (dieu_kien_state, state_dich) cua mot ham fsm_on_*."""
    m = re.search(r"fsm_state_t\s+" + fn_name + r"\s*\([^)]*\)\s*\{(.*?)\n\}",
                  FSM_C, re.S)
    assert m, fn_name
    return m.group(1)


DISARMED, ARMED, TAKING_OFF, HOLDING, FLYING, LANDING, EMERGENCY, BENCH = (
    "FSM_DISARMED", "FSM_ARMED", "FSM_TAKING_OFF", "FSM_HOLDING",
    "FSM_FLYING", "FSM_LANDING", "FSM_EMERGENCY", "FSM_BENCH_RAMP")

print("== (1) Luat chuyen state (doc tu flight_state_machine.c) ==")

arm = fsm_rule("fsm_on_arm_request")
check("ARM: DISARMED + guard -> ARMED",
      "cur == FSM_DISARMED && guard_ok" in arm and "return FSM_ARMED" in arm)

tko = fsm_rule("fsm_on_takeoff_request")
check("TAKEOFF: ARMED -> TAKING_OFF",
      "cur == FSM_ARMED" in tko and "return FSM_TAKING_OFF" in tko)

handoff = fsm_rule("fsm_on_takeoff_handoff")
check("HANDOFF: TAKING_OFF -> HOLDING",
      "cur == FSM_TAKING_OFF" in handoff and "return FSM_HOLDING" in handoff)

move = fsm_rule("fsm_on_move_command")
check("MOVE: HOLDING + moving -> FLYING",
      "cur == FSM_HOLDING && moving" in move and "return FSM_FLYING" in move)
check("MOVE: FLYING + !moving -> HOLDING",
      "cur == FSM_FLYING && !moving" in move and "return FSM_HOLDING" in move)

# ⚠ DIEU KIEN THEN CHOT cho bao cao cua nguoi dung: FLYING CHI vao duoc tu
# HOLDING. Khong co duong tat nao tu TAKING_OFF. Nen neu takeoff khong ban giao
# duoc thi GUI se KHONG BAO GIO thay HOLD lan FLYING — dung nhu da quan sat.
check("KHONG co duong TAKING_OFF -> FLYING (phai qua HOLDING)",
      "FSM_TAKING_OFF" not in move,
      "co duong tat thi ket luan chan doan o tren se sai")

land = fsm_rule("fsm_on_land_request")
check("LAND: HOLDING hoac FLYING -> LANDING",
      "cur == FSM_HOLDING || cur == FSM_FLYING" in land
      and "return FSM_LANDING" in land)

soft = fsm_rule("fsm_on_soft_fault")
check("SOFT FAULT o TAKING_OFF -> LANDING", "FSM_TAKING_OFF" in soft)

# =============================================================================
# (2) MA MODE= ma GUI nhan — telemetry_format_alt_mode()
# =============================================================================
print("\n== (2) Ma MODE= tren day (telemetry_format.c) ==")

m_mode = re.search(r"int telemetry_format_alt_mode\([^)]*\)\s*\{(.*?)\n\}",
                   FMT_C, re.S)
check("tim thay telemetry_format_alt_mode()", m_mode is not None)

MODE_OF = {}
if m_mode:
    for st, code in re.findall(r"case\s+(FSM_\w+):\s*return\s+(\d+);", m_mode.group(1)):
        MODE_OF[st] = int(code)
    m_def = re.search(r"default:\s*return\s+(\d+);", m_mode.group(1))
    default_code = int(m_def.group(1)) if m_def else None

    check("HOLDING -> MODE=2", MODE_OF.get(HOLDING) == 2, str(MODE_OF.get(HOLDING)))
    check("TAKING_OFF -> MODE=3", MODE_OF.get(TAKING_OFF) == 3, str(MODE_OF.get(TAKING_OFF)))
    check("LANDING -> MODE=4", MODE_OF.get(LANDING) == 4, str(MODE_OF.get(LANDING)))
    check("FLYING -> MODE=5", MODE_OF.get(FLYING) == 5, str(MODE_OF.get(FLYING)))
    # ⚠ FLYING PHAI co ma RIENG. Truoc day no tra CUNG gia tri 2 voi HOLDING,
    # nen GUI luon hien "HOLD" ke ca khi dang bay ngang.
    check("FLYING co ma RIENG, khong trung HOLDING",
          MODE_OF.get(FLYING) != MODE_OF.get(HOLDING))
    check("DISARMED/ARMED -> MODE=0 (default)", default_code == 0)

# =============================================================================
# (3) MO PHONG CHUOI — ghep (1) va (2)
# =============================================================================
print("\n== (3) Mo phong chuoi bay + ma MODE tung buoc ==")


def mode_of(state):
    return MODE_OF.get(state, default_code)


# Chuoi KY VONG cua mot chuyen bay binh thuong.
sequence = [
    ("boot",              DISARMED,   0),
    ("ARM",               ARMED,      0),
    ("TAKEOFF",           TAKING_OFF, 3),
    ("handoff xong",      HOLDING,    2),   # <-- buoc CHUA BAO GIO dat duoc
    ("nghieng can",       FLYING,     5),
    ("tha can",           HOLDING,    2),
    ("LAND",              LANDING,    4),
]
for label, state, want in sequence:
    got = mode_of(state)
    check("%-16s -> %-16s MODE=%s" % (label, state, got), got == want,
          "ky vong %s" % want)

# Chuoi THUC TE quan sat duoc trong log nguoi dung: khong bao gio qua HOLDING.
print("\n  (chuoi THUC TE trong log: takeoff khong ban giao duoc)")
observed = [("TAKEOFF", TAKING_OFF), ("soft-fault", LANDING), ("disarm", DISARMED)]
seen = [mode_of(s) for _, s in observed]
check("chuoi loi cho MODE 3->4->0, KHONG co 2 hay 5",
      seen == [3, 4, 0] and 2 not in seen and 5 not in seen,
      str(seen))

# =============================================================================
# (4) GUI phai giai ma DU ca 6 ma
# =============================================================================
print("\n== (4) GUI giai ma MODE ==")
for code, name in (("0", "OFF"), ("2", "HOLD"), ("3", "TAKEOFF"),
                   ("4", "LANDING"), ("5", "FLYING")):
    check('GUI co bang ma "%s" -> %s' % (code, name),
          '"%s": "%s"' % (code, name) in GUI_PY)

# W/S re nhanh phai dung DUNG hai ma nay, khong phai ten state.
check("GUI re nhanh W/S theo MODE 2 (HOLD)", 'amode == "2"' in GUI_PY)
check("GUI re nhanh W/S theo MODE 5 (FLYING)", 'amode == "5"' in GUI_PY)
check("GUI luu mode moi frame STATUS", "self._last_alt_mode = amode" in GUI_PY)
check("_ws_press() doc mode da luu", "_ws_mode()" in GUI_PY)

# =============================================================================
# (5) DIEU KIEN BAN GIAO — thu chan lai loi that
# =============================================================================
print("\n== (5) Dieu kien ban giao takeoff -> HOLDING ==")
TKO_C = (ROOT / "components/flight_core/src/takeoff_land.c").read_text(encoding="utf-8")

def strip_c_comments(src):
    """Bo comment truoc khi kiem 'code co dung X khong'.

    Khoi giai thich o takeoff_land.c CON NHAC TEN cac hang so da bo, nen
    kiem thang tren van ban se bao dong gia.
    """
    src = re.sub(r"/\*.*?\*/", "", src, flags=re.S)
    return re.sub(r"//[^\n]*", "", src)


TKO_CODE = strip_c_comments(TKO_C)
m_hr = re.search(r"const bool hold_ready\s*=(.*?);", TKO_C, re.S)
check("tim thay hold_ready", m_hr is not None)
if m_hr:
    expr = " ".join(m_hr.group(1).split())
    # hold_ready gio la: slew_done && (vz_settled || settle_timeout).
    # Hai ve xac dinh nam trong slew_done, nen kiem tren CA HAM thay vi chi
    # tren dong hold_ready.
    check("hold_ready doi ROI DAT", "liftoff_flag" in TKO_CODE, expr)
    check("hold_ready doi rate-limiter truot het",
          "target_z_m == st->final_target_m" in TKO_CODE, expr)
    check("hold_ready doi vz da lang (chong vot lo)",
          "vz_settled" in expr, expr)
    check("ve vz co tran thoi gian (khong the ket vinh vien)",
          "settle_timeout" in expr, expr)
    # Ve DO CAO da bo VINH VIEN: no tung lam takeoff ket 21.6s roi timeout
    # (drone leo 1.45m khi target 1.00m -> |Z-tgt| khong bao gio <= 0.08).
    # Kiem tren CA HAM, khong chi tren dong hold_ready: kiem hep se cho ket qua
    # XANH GIA neu ai do khai lai no o mot bien trung gian.
    check("KHONG con doi |alt - target| <= TOL (tung lam ket 21.6s)",
          "TAKEOFF_HOLD_Z_TOL_M" not in TKO_CODE,
          "ve nay tung lam takeoff khong bao gio ban giao duoc")

print("")
if fails:
    print("KET QUA: %d FAIL -- %s" % (len(fails), ", ".join(fails)))
    sys.exit(1)
print("KET QUA: ALL PASS -- chuoi mode DISARMED->ARMED->TAKEOFF->HOLD->FLYING->LAND")
print("         va ma MODE= tuong ung deu dung.")
