"""Dung sai ToF: MOT mau xau KHONG duoc huy mot lan cat canh.

BOI CANH (loi that, da quan sat duoc tren bo):
    I TAKEOFF: target=0.50m | PRIME 1188 duty trong 800ms
    W SOFT FAULT -> LANDING: altitude estimator lost mid-flight
    loi: takeoff bi tu choi hoac ABORT

Chuoi nhan qua:
  1. Trong PRIME, airborne=false  -> alt_estimator di nhanh `!airborne`.
  2. update_age(airborne=false) RETURN SOM, khong dung toi e->valid
     -> toan bo luoi TRACKING/BRIDGE/LOST chi ton tai KHI DANG BAY.
  3. e->valid = tof_healthy, danh gia lai moi tick, khong do tre.
  4. commander KHONG loai tru FSM_TAKING_OFF khoi `alt_estimator_lost`.
  => MOT mau ToF co range_status != 0 = huy cat canh, ngay lap tuc.

Bat doi xung khong bien minh duoc: khi DANG BAY cung su kien do duoc cho
100+220+300ms; khi NAM DAT — dung luc motor rung manh nhat va ToF o cu ly kho
doc nhat — duoc cho 0ms.

Test nay khoa hai duong sua lai:
  (1) cua so SENSOR_TOF_STALE_US phai THAT SU hoat dong (truoc day la code chet)
  (2) mat dat phai co luoi do CUNG NGUONG voi tren khong
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FC   = (ROOT / "components/flight_core/src/flight_core.c").read_text(encoding="utf-8")
EST  = (ROOT / "components/flight_core/src/alt_estimator.c").read_text(encoding="utf-8")
ESTH = (ROOT / "components/flight_core/include/flight_core/alt_estimator.h").read_text(encoding="utf-8")
HUB  = (ROOT / "components/flight_core/src/sensor_hub.c").read_text(encoding="utf-8")
HUBH = (ROOT / "components/flight_core/include/flight_core/sensor_hub.h").read_text(encoding="utf-8")
CMD  = (ROOT / "components/flight_core/src/commander.c").read_text(encoding="utf-8")

fails = []


def check(name, cond, detail=""):
    if cond:
        print("  PASS  %s" % name)
    else:
        print("  FAIL  %s%s" % (name, ("  -- " + detail) if detail else ""))
        fails.append(name)


def strip_comments(text):
    """Comment CO QUYEN mo ta lai loi cu; chi CODE moi phai dung."""
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return re.sub(r"//[^\n]*", "", text)


FC_CODE  = strip_comments(FC)
EST_CODE = strip_comments(EST)
HUB_CODE = strip_comments(HUB)

# =============================================================================
# (1) Cua so SENSOR_TOF_STALE_US phai THAT SU hoat dong
# =============================================================================
print("== (1) Cua so dung sai 200ms phai co that ==")

# mark_err() KHONG duoc dong toi seq/timestamp. Day la GIA THIET NEN cua ca
# huong sua: neu mark_err doi timestamp thi "tuoi" khong con nghia "bao lau roi
# chua co mau DUNG DUOC", va noi long theo tuoi se thanh tin vao du lieu chet.
m_err = re.search(r"static void mark_err\s*\([^)]*\)\s*\{(.*?)\n\}", HUB_CODE, re.S)
check("tim thay mark_err()", m_err is not None)
if m_err:
    body = m_err.group(1)
    check("mark_err() KHONG dong toi timestamp_us",
          "timestamp_us" not in body,
          "doi timestamp -> du lieu chet trong nhu vua moi -> noi long thanh nguy hiem")
    check("mark_err() KHONG tang seq",
          not re.search(r"seq\s*(\+\+|\+=)", body),
          "tang seq -> estimator correction LAI tren mau cu")

# mark_ok() PHAI dời timestamp va tang seq -- ve con lai cua cung gia thiet.
m_ok = re.search(r"static void mark_ok\s*\([^)]*\)\s*\{(.*?)\n\}", HUB_CODE, re.S)
check("tim thay mark_ok()", m_ok is not None)
if m_ok:
    body = m_ok.group(1)
    check("mark_ok() doi timestamp_us", "timestamp_us" in body)
    check("mark_ok() tang seq", re.search(r"seq\s*(\+\+|\+=)", body) is not None)

# Dieu kien suc khoe ToF phai tinh bang TUOI, KHONG duoc AND them tof_h.valid.
# Day chinh la cho ma cua so 200ms tung bi vo hieu hoa.
m_health = re.search(
    r"const bool tof_healthy_for_alt\s*=([^;]*);", FC_CODE, re.S)
check("tim thay dinh nghia tof_healthy_for_alt", m_health is not None)
if m_health:
    expr = " ".join(m_health.group(1).split())
    check("tof_healthy_for_alt tinh bang TUOI (SENSOR_TOF_STALE_US)",
          "SENSOR_TOF_STALE_US" in expr, expr)
    check("tof_healthy_for_alt KHONG AND voi tof_h.valid",
          "tof_h.valid" not in expr,
          "AND voi valid lam ve tuoi thanh code chet -- MOT mau xau = mat ToF: %s" % expr)

# sensor_hub_age_us() phai tra INT64_MAX khi chua tung co mau tot. Khong co ve
# nay thi bo tof_h.valid se mo ra truong hop "ToF chua bao gio chay ma van
# duoc coi la khoe".
check("sensor_hub_age_us() tra INT64_MAX khi seq == 0",
      re.search(r"if\s*\(\s*h->seq\s*==\s*0\s*\)\s*return\s+INT64_MAX", HUBH) is not None,
      "thieu ve nay -> ToF chua tung chay van co the duoc coi la khoe")

# =============================================================================
# (2) Mat dat phai co luoi do CUNG NGUONG voi tren khong
# =============================================================================
print("\n== (2) Luoi do tren mat dat ==")

# Lay DUNG nhanh !airborne cua alt_estimator_update().
#
# ⚠ PHAI neo tu than ham truoc: update_age() CUNG mo dau bang `if (!airborne)`
# va nam TRUOC trong file, nen tim thang se bat nham nhanh do -- test se xanh/do
# vi mot ly do hoan toan khac cho voi cai dang kiem.
i_upd = EST_CODE.find("void alt_estimator_update")
check("tim thay alt_estimator_update()", i_upd != -1)
m_ground = re.search(r"if\s*\(\s*!airborne\s*\)\s*\{(.*?)\n        return;",
                     EST_CODE[i_upd:], re.S) if i_upd != -1 else None
check("tim thay nhanh !airborne trong alt_estimator_update()", m_ground is not None)

if m_ground:
    g = m_ground.group(1)

    # Ve cu: valid = tof_healthy (khong gia han). Phai KHONG con nua.
    check("mat dat KHONG con 'valid = tof_healthy' tran (khong gia han)",
          not re.search(r"e->valid\s*=\s*tof_healthy\s*;", g),
          "mot mau xau van huy cat canh")

    # Ve moi: phai co duong gia han theo tuoi, dung CUNG nguong voi tren khong.
    check("mat dat gia han theo ALT_EST_TOF_LOST_MS (cung nguong tren khong)",
          "ALT_EST_TOF_LOST_MS" in g,
          "phai dung CUNG nguong, khong duoc tu bia so rieng")
    check("mat dat van doi tof_healthy HOAC con trong cua so gia han",
          re.search(r"tof_healthy\s*\|\|", g) is not None,
          "phai la OR: mau tuoi HOAC gia han con hieu luc")

    # Chua TUNG co mau nao dung duoc -> KHONG duoc gia han. Day la truong hop
    # ToF chet/khong han, va no PHAI chan cat canh.
    check("khong gia han khi chua tung co mau nao dung duoc",
          re.search(r"last_tof_accept_us\s*!=\s*0", g) is not None,
          "thieu -> ToF chet van duoc coi la 'vua moi mat', cho cat canh")

    # Moc phai duoc refresh TRUOC khi tinh valid, neu khong thi valid doc mot
    # moc cu hon dung mot tick.
    i_refresh = g.find("last_tof_accept_us = e->last_correction_us")
    i_valid = g.find("e->valid =")
    check("moc last_tof_accept_us duoc refresh TRUOC khi tinh valid",
          i_refresh != -1 and i_valid != -1 and i_refresh < i_valid,
          "refresh=%d valid=%d" % (i_refresh, i_valid))

    # Cong floor (neu bat) van phai chan duoc -- noi long dung sai KHONG duoc
    # bien thanh noi long cong floor.
    check("cong floor van duoc giu khi FC_FEATURE_FLOOR_GATE=1",
          "alt_estimator_floor_ready" in g)

# Nguong tren khong van nguyen ven -- test nay khong duoc lam hong cai da co.
for token in ("ALT_EST_TOF_TRACK_MAX_AGE_MS", "ALT_EST_TOF_BRIDGE_MS",
              "ALT_EST_TOF_LOST_MS"):
    check("nguong tren khong con nguyen: %s" % token, token in ESTH)

# =============================================================================
# (3) Bat doi xung ban dau: commander khong loai tru TAKING_OFF
# =============================================================================
# KHONG sua commander (cat canh mat ToF that thi VAN phai huy) -- chi ghi lai
# rang day la ly do vi sao hai fix tren PHAI nam o tang duoi.
print("\n== (3) Commander: TAKING_OFF van bi soft-fault khi mat ToF that ==")

# Dieu kien co ngoac LONG NHAU -- `[^)]*` dung ngay o ngoac dau tien. Lay theo
# vi tri: lui ve `if (` gan nhat truoc alt_estimator_lost, tien toi `) {`.
CMD_CODE = strip_comments(CMD)
i_lost = CMD_CODE.find("in->alt_estimator_lost")
i_if = CMD_CODE.rfind("if (", 0, i_lost) if i_lost != -1 else -1
i_end = CMD_CODE.find(") {", i_lost) if i_lost != -1 else -1
m_lost = (i_if != -1 and i_end != -1)
check("tim thay dieu kien alt_estimator_lost trong commander", m_lost)
if m_lost:
    cond = " ".join(CMD_CODE[i_if:i_end].split())
    check("commander VAN fault o TAKING_OFF (khong loai tru)",
          "FSM_TAKING_OFF" not in cond,
          "neu loai tru thi cat canh voi ToF chet that se khong con bi chan")
    for st in ("FSM_LANDING", "FSM_BENCH_RAMP", "FSM_ARMED"):
        check("commander van loai tru %s" % st, st in cond)

print("")
if fails:
    print("KET QUA: %d FAIL -- %s" % (len(fails), ", ".join(fails)))
    sys.exit(1)
print("KET QUA: ALL PASS -- mot mau ToF xau khong con huy duoc mot lan cat canh")
