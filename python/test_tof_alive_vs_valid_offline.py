"""Phan biet "KHONG CO GI DE DO" voi "MAT CAM BIEN".

VI SAO CAN TEST NAY:
Dat drone xuong san thi VL53L1X doc 0.000m (duoi tam mu ~4cm). Mau do bi loai
o driver (dist_mm > 0 sai) -> khong mau nao duoc accept -> tuoi mau HOP LE tang
vo han -> estimator ket luan "mat ToF" -> Commander soft-fault. Tuc la chi can
de drone nam dat du lau la firmware TU BAO HONG cam bien.

Sua bang cach tach hai moc thoi gian:
  last_good_us     -> lan cuoi co mau HOP LE      ("co so de bay khong")
  last_consumed_us -> lan cuoi DOC XONG ket qua   ("con cam bien khong")

Test nay khoa CA HAI CHIEU. Chieu de quen la chieu thu hai: neu chi lo cho
"nam dat khong fault" ma lam hong "ToF chet van fault" thi da doi mot phien toai
lay mot loi an toan.
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EST_C = (ROOT / "components/flight_core/src/alt_estimator.c").read_text(encoding="utf-8")
EST_H = (ROOT / "components/flight_core/include/flight_core/alt_estimator.h").read_text(encoding="utf-8")
HUB_C = (ROOT / "components/flight_core/src/sensor_hub.c").read_text(encoding="utf-8")
HUB_H = (ROOT / "components/flight_core/include/flight_core/sensor_hub.h").read_text(encoding="utf-8")
CORE_C = (ROOT / "components/flight_core/src/flight_core.c").read_text(encoding="utf-8")
TOF_C = (ROOT / "components/flight_core/src/drivers/tof_driver.c").read_text(encoding="utf-8")
L1X_C = (ROOT / "components/flight_core/src/drivers/vl53l1x_driver.c").read_text(encoding="utf-8")
L0X_C = (ROOT / "components/flight_core/src/drivers/vl53l0x_driver.c").read_text(encoding="utf-8")

fails = []


def check(name, cond, detail=""):
    if cond:
        print("  PASS  %s" % name)
    else:
        print("  FAIL  %s%s" % (name, ("  -- " + detail) if detail else ""))
        fails.append(name)


def strip_comments(src):
    """Bo comment truoc khi kiem tra 'code co ton tai khong'.

    Khong co buoc nay thi mot cau lenh nam trong comment van lam test xanh —
    dung loai test vo dung nhat.
    """
    src = re.sub(r"/\*.*?\*/", "", src, flags=re.S)
    src = re.sub(r"//[^\n]*", "", src)
    return src


EST = strip_comments(EST_C)
HUB = strip_comments(HUB_C)
CORE = strip_comments(CORE_C)
TOF = strip_comments(TOF_C)

# =============================================================================
# (1) DRIVER: hai moc thoi gian TON TAI va TACH BIET
# =============================================================================
print("== (1) Driver: tach last_consumed_us khoi last_sample_us ==")

BACKEND_H = strip_comments(
    (ROOT / "components/flight_core/src/drivers/tof_backend.h").read_text(encoding="utf-8"))
check("tof_sensor_state_t co last_consumed_us", "last_consumed_us" in BACKEND_H)

for name, src in (("L1X", strip_comments(L1X_C)), ("L0X", strip_comments(L0X_C))):
    check("%s: poll dat last_consumed_us" % name, "last_consumed_us" in src)
    # Ca hai moc phai lay tu CUNG mot bien thoi gian -> cung mo ta MOT mau vat ly.
    check("%s: dung chung sample_now_us" % name,
          "sample_now_us" in src and "last_sample_us   = sample_now_us" in src.replace(
              "last_sample_us = sample_now_us", "last_sample_us   = sample_now_us"))

# Moc chi duoc dat SAU khi clear interrupt thanh cong.
for name, src in (("L1X", strip_comments(L1X_C)), ("L0X", strip_comments(L0X_C))):
    i_clear = src.rfind("clear_interrupt") if name == "L1X" else src.rfind("SYSTEM_INTERRUPT_CLEAR")
    i_stamp = src.find("sample_now_us = esp_timer_get_time")
    check("%s: dat moc SAU clear interrupt" % name,
          i_clear != -1 and i_stamp != -1 and i_clear < i_stamp,
          "clear@%d stamp@%d" % (i_clear, i_stamp))

# =============================================================================
# (2) WATCHDOG KHONG DUOC LAM GIA "chip con song"
# =============================================================================
# Day la cai bay tinh te nhat cua ca thay doi nay.
# poll_with_watchdog() DOI last_sample_us toi hien tai moi lan no restart, de
# ban than no khong lap vo han. Neu getter tra last_sample_us thi mot con ToF da
# chet han VAN trong nhu "vua do xong" moi 300ms -> dieu kien mat-sensor KHONG
# BAO GIO dung -> pha dung cai dang can bao ve.
print("\n== (2) Watchdog khong duoc lam gia moc 'chip con song' ==")

m_get = re.search(r"int64_t tof_driver_last_sample_us\(void\)\s*\{(.*?)\n\}", TOF, re.S)
check("tim thay tof_driver_last_sample_us()", m_get is not None)
if m_get:
    body = m_get.group(1)
    check("getter tra last_consumed_us", "last_consumed_us" in body, body.strip())
    check("getter KHONG tra last_sample_us", "last_sample_us" not in body,
          "watchdog doi last_sample_us -> ToF chet se trong nhu con song")

# Chung minh watchdog THAT SU co doi last_sample_us (neu khong, ca lo lang tren
# la vo can cu va test nay dang bao ve mot thu khong ton tai).
m_wd = re.search(r"poll_with_watchdog\(void\)\s*\{(.*?)\n\}", TOF, re.S)
check("watchdog co doi last_sample_us (ly do phai tach moc)",
      m_wd is not None and "last_sample_us = now_us" in m_wd.group(1))
check("watchdog KHONG dong vao last_consumed_us",
      m_wd is not None and "last_consumed_us" not in m_wd.group(1))

# =============================================================================
# (3) SENSOR_HUB: publish moc song BAT KE mau co hop le hay khong
# =============================================================================
print("\n== (3) sensor_hub publish tof_alive_us ==")

check("snapshot co tof_alive_us", "tof_alive_us" in strip_comments(HUB_H))

m_pub = re.search(r"static void publish_tof\([^)]*\)\s*\{(.*?)\n\}", HUB, re.S)
check("tim thay publish_tof()", m_pub is not None)
if m_pub:
    body = m_pub.group(1)
    check("publish_tof dat tof_alive_us", "tof_alive_us" in body)
    # Phai nam NGOAI nhanh if(ok) — day chinh la diem mau chot.
    i_alive = body.find("tof_alive_us")
    i_ok = body.find("if (ok)")
    check("tof_alive_us dat TRUOC nhanh if(ok) -> cap nhat ca khi mau xau",
          i_alive != -1 and i_ok != -1 and i_alive < i_ok,
          "alive@%d if_ok@%d" % (i_alive, i_ok))

# =============================================================================
# (4) ESTIMATOR: nam dat KHONG fault, ToF chet VAN fault
# =============================================================================
print("\n== (4) update_age phan biet hai ca ==")

m_age = re.search(r"static void update_age\((.*?)\)\s*\{(.*?)\n\}", EST, re.S)
check("tim thay update_age()", m_age is not None)
if m_age:
    sig, body = m_age.group(1), m_age.group(2)
    check("update_age nhan tof_hw_alive", "tof_hw_alive" in sig)

    # Cau truc DA DOI: bac thang theo tuoi mau da bi bo han, gio chi con mot
    # nhanh `if (!tof_hw_alive)`. Xem test_alt_validity_offline.py de biet
    # day du ly do.
    check("co nhanh `if (!tof_hw_alive)`", "if (!tof_hw_alive)" in body)

    i_dead = body.find("if (!tof_hw_alive)")
    branch_dead = body[i_dead:i_dead + 300] if i_dead != -1 else ""

    # --- Chieu 1: chip IM -> VAN phai fault ---
    check("chip im -> valid = false (VAN soft-fault)",
          "e->valid = false" in branch_dead.replace(
              "e->tof_fusable = e->valid = false;", "e->valid = false;"),
          branch_dead.strip()[:120])
    check("chip im -> ALT_SRC_TOF_LOST", "ALT_SRC_TOF_LOST" in branch_dead)

    # --- Chieu 2: chip CON DO nhung khong fuse duoc -> KHONG duoc fault ---
    i_fus = body.find("if (e->tof_fusable)")
    check("co nhanh re theo tof_fusable", i_fus != -1)
    if i_fus != -1:
        rest = body[i_fus:]
        i_else = rest.find("} else {")
        branch_coast = rest[i_else:] if i_else != -1 else ""
        check("chip con do -> valid = true (khong soft-fault)",
              "e->valid = true;" in branch_coast, branch_coast.strip()[:140])
        # ⚠ TRUOC DAY test nay doi DUNG chuoi "e->degraded = true;".
        # Da doi: gan cung true la mot LOI THAT (mot mau ToF khong fusable ->
        # commander SOFT FAULT ngay tick dau -> LANDING giua chuyen; commander.c
        # khong co debounce o nhanh do). Gio degraded duoc tinh theo THOI GIAN
        # ke tu correction cuoi -- dung hop dong viet o commander.h.
        # Kiem YEU CAU chu khong kiem CACH VIET.
        # Moc dem phai DOC LAP voi tof_fusable. last_tof_accept_us khong dat
        # yeu cau do: terr_pending va terr_offset_stale deu ep tof_fusable=false
        # nen moc dung yen, va moi lan roi ban lai dem nguoc toi fault.
        check("chip do + khong fuse -> degraded tinh theo thoi gian (khong gan cung true)",
              "e->degraded = true;" not in branch_coast and
              re.search(r"last_\w*_us", branch_coast) is not None and
              "last_tof_accept_us" not in branch_coast and
              "ALT_EST_NO_CORRECTION_DEGRADED_MS" in branch_coast,
              branch_coast.strip()[:200])

# =============================================================================
# (5) FLIGHT_CORE: tinh tof_hw_alive tu snapshot
# =============================================================================
print("\n== (5) flight_core tinh tof_hw_alive ==")

check("khai bao tof_hw_alive", "const bool tof_hw_alive" in CORE)
check("tof_hw_alive doc snap.tof_alive_us", "snap.tof_alive_us" in CORE)
check("truyen tof_hw_alive vao alt_estimator_update", "tof_hw_alive, stationary_for_alt" in CORE)

m_hw = re.search(r"const bool tof_hw_alive\s*=(.*?);", CORE, re.S)
check("tim thay bieu thuc tof_hw_alive", m_hw is not None)
if m_hw:
    expr = " ".join(m_hw.group(1).split())
    # Dung lai nguong san co thay vi de ra hang so moi -> hai tang khong the lech.
    check("dung ALT_EST_TOF_LOST_MS (khong dat hang so moi)",
          "ALT_EST_TOF_LOST_MS" in expr, expr)
    # 0 = chua tung doc duoc mau nao -> KHONG duoc coi la con song.
    check("guard snap.tof_alive_us != 0", "tof_alive_us != 0" in expr, expr)

# tof_healthy_for_alt gio CUNG dung tof_hw_alive.
# Ban truoc no van la (tof_age_us <= SENSOR_TOF_STALE_US) — tuc la con sot lai
# dung cai dieu kien theo TUOI MAU HOP LE ma ca thay doi nay sinh ra de bo.
# Hau qua da quan sat: nam sat san -> bien nay false vinh vien -> chan neo
# target trong FLYING, chan takeoff, va GUI bao "ToF LOI: mau STALE".
check("tof_healthy_for_alt dung tof_hw_alive (khong con theo tuoi mau)",
      "const bool tof_healthy_for_alt = tof_hw_alive;" in CORE)
check("khong con so sanh SENSOR_TOF_STALE_US de quyet dinh",
      "tof_age_us <= SENSOR_TOF_STALE_US" not in CORE)

# =============================================================================
# (6) MO PHONG: ba kich ban tren mot truc thoi gian
# =============================================================================
print("\n== (6) Mo phong ba kich ban ==")

LOST_MS = 300


def decide(ms_since_good, ms_since_consumed):
    """Mo phong dung logic update_age() da sua."""
    hw_alive = ms_since_consumed is not None and ms_since_consumed <= LOST_MS
    if ms_since_good < LOST_MS:
        return "OK"
    return "DEGRADED" if hw_alive else "FAULT"

# Nam tren san: chip do deu (0mm moi 40ms) nhung khong mau nao hop le.
for t in (500, 1000, 5000, 60000):
    check("nam dat %5dms -> DEGRADED (khong fault)" % t,
          decide(t, 40) == "DEGRADED", decide(t, 40))

# ToF chet han: ca hai moc deu dung.
# Bat dau tu 301 chu khong phai 300: tai DUNG nguong, dieu kien la "<=" nen van
# tinh la con song. Do la chu y — fault chi bat dau SAU khi da qua han, khong
# phai ngay tai han. Bien chinh xac duoc kiem rieng o hai check ben duoi.
for t in (301, 1000, 5000):
    check("ToF chet %5dms -> FAULT (van bat duoc)" % t,
          decide(t, t) == "FAULT", decide(t, t))

# Bay binh thuong.
check("bay binh thuong (mau tuoi) -> OK", decide(50, 50) == "OK")

# Bien: vua cham nguong.
check("bien ms_since_consumed = LOST_MS -> con song",
      decide(9999, LOST_MS) == "DEGRADED")
check("bien ms_since_consumed = LOST_MS+1 -> chet",
      decide(9999, LOST_MS + 1) == "FAULT")

# Chua tung doc duoc mau nao (chua init xong) -> phai la FAULT, khong duoc
# nham thanh "con song".
check("chua tung co mau (alive_us = 0) -> FAULT",
      decide(9999, None) == "FAULT")

print("")
if fails:
    print("KET QUA: %d FAIL -- %s" % (len(fails), ", ".join(fails)))
    sys.exit(1)
print("KET QUA: ALL PASS")
print("         nam dat/ngoai tam -> DEGRADED (khong soft-fault)")
print("         ToF chet that     -> FAULT (duong an toan con nguyen)")
