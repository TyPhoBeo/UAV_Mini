"""Do cao con dung duoc khong: CHI hoi "chip con do khong".

TRUOC DAY day la mot bac thang 4 muc theo TUOI MAU HOP LE:
    <=TRACK_MAX_AGE -> TRACKING
    <=BRIDGE_MS     -> SHORT_BRIDGE
    < LOST_MS       -> IMU_PREDICT (degraded)
    >=LOST_MS       -> LOST -> valid=false -> Commander soft-fault

Bac cuoi la nguon goc cua ca mot chuoi loi da phai va tung cai mot: bay qua vat
the, nam sat san (ToF doc 0.000m duoi tam mu ~4cm), nhin ra khoang khong, be mat
hap thu. Tat ca deu lam tuoi mau tang vo han TRONG KHI cam bien hoan toan lanh,
va tat ca deu ket thuc bang TU DONG HA CANH giua chung.

GIO chi con MOT dieu kien: tof_hw_alive (chip co con TIEU THU duoc ket qua
khong). Test nay khoa lai ca hai chieu -- chieu de quen la chieu thu hai:
neu chi lo cho "khong tu ha canh oan" ma lam hong "chip chet van fault" thi da
doi mot phien toai lay mot loi an toan.
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EST_H = (ROOT / "components/flight_core/include/flight_core/alt_estimator.h").read_text(encoding="utf-8")
EST_C = (ROOT / "components/flight_core/src/alt_estimator.c").read_text(encoding="utf-8")

fails = []


def check(name, cond, detail=""):
    if cond:
        print("  PASS  %s" % name)
    else:
        print("  FAIL  %s%s" % (name, ("  -- " + detail) if detail else ""))
        fails.append(name)


def strip_comments(src):
    """Bo comment truoc khi kiem 'code co ton tai khong'.

    Khong co buoc nay thi mot cau lenh nam trong comment van lam test xanh --
    dung loai test vo dung nhat. Cac comment giai thich o update_age() con
    NHAC DEN ten hang so cu, nen bo comment la bat buoc o day.
    """
    src = re.sub(r"/\*.*?\*/", "", src, flags=re.S)
    return re.sub(r"//[^\n]*", "", src)


CODE = strip_comments(EST_C)

# =============================================================================
# (1) enum nguon do cao van du
# =============================================================================
print("== (1) alt_source_t ==")
for token in ("ALT_SRC_GROUND_LOCK", "ALT_SRC_IMU_PREDICT_ONLY", "ALT_SRC_TOF_FUSED",
              "ALT_SRC_TOF_SHORT_BRIDGE", "ALT_SRC_TOF_LOST"):
    check("enum co %s" % token, token in EST_H)

# =============================================================================
# (2) BAC THANG THEO TUOI DA BI BO
# =============================================================================
print("\n== (2) Khong con dieu kien theo tuoi mau ==")

m_age = re.search(r"static void update_age\((.*?)\)\s*\{(.*?)\n\}", CODE, re.S)
check("tim thay update_age()", m_age is not None)
if m_age:
    body = m_age.group(2)
    # Ba nguong nay tung la dieu kien re nhanh. Gio khong duoc dung de QUYET DINH
    # nua (chung van co the ton tai trong header cho telemetry/tuning).
    for tok in ("ALT_EST_TOF_TRACK_MAX_AGE_MS",
                "ALT_EST_TOF_BRIDGE_MS",
                "ALT_EST_TOF_LOST_MS"):
        check("update_age KHONG con dung %s" % tok, tok not in body,
              "tuoi mau khong duoc quay lai lam dieu kien")

    # Khong con so sanh tuoi kieu `age <= ...` / `age < ...`
    check("khong con so sanh `age` de re nhanh",
          not re.search(r"\bage\s*[<>]=?", body), body[:160])

# =============================================================================
# (3) CHI CON MOT DIEU KIEN: tof_hw_alive
# =============================================================================
print("\n== (3) Dieu kien duy nhat la tof_hw_alive ==")
if m_age:
    body = m_age.group(2)
    check("update_age nhan tof_hw_alive", "tof_hw_alive" in m_age.group(1))
    check("co nhanh `if (!tof_hw_alive)`", "if (!tof_hw_alive)" in body)

    # --- Chieu 1: chip IM -> PHAI fault (duong an toan) ---
    i_dead = body.find("if (!tof_hw_alive)")
    branch_dead = body[i_dead:i_dead + 300] if i_dead != -1 else ""
    check("chip im -> valid = false", "e->valid = false" in branch_dead.replace(
        "e->tof_fusable = e->valid = false;", "e->valid = false;"), branch_dead.strip()[:120])
    check("chip im -> ALT_SRC_TOF_LOST", "ALT_SRC_TOF_LOST" in branch_dead)

    # --- Chieu 2: chip CON DO nhung khong fuse duoc -> VAN valid ---
    i_fus = body.find("if (e->tof_fusable)")
    check("co nhanh re theo tof_fusable", i_fus != -1)
    if i_fus != -1:
        rest = body[i_fus:]
        i_else = rest.find("} else {")
        branch_coast = rest[i_else:] if i_else != -1 else ""
        check("chip do + khong fuse -> valid = true (KHONG fault)",
              "e->valid = true;" in branch_coast, branch_coast.strip()[:140])
        check("chip do + khong fuse -> degraded = true (bao ra telemetry)",
              "e->degraded = true;" in branch_coast)
        check("chip do + khong fuse -> ALT_SRC_IMU_PREDICT_ONLY",
              "ALT_SRC_IMU_PREDICT_ONLY" in branch_coast)

# =============================================================================
# (4) MAY DO SAN DA BI BO HAN
# =============================================================================
# ToF gio do TUYET DOI: alt = khoang cach toi be mat, khong tru goc toa do nao.
print("\n== (4) May do san (floor_add + cua so thong ke) da bi bo ==")

check("khong con ham floor_add()", "static void floor_add" not in CODE)
check("khong con goi floor_add(", "floor_add(e" not in CODE)

# tof_ground_ref_valid khong duoc con lam CONG chan duong ToF fusion.
m_upd = re.search(r"void alt_estimator_update\(.*?\n\}", CODE, re.S)
if m_upd:
    check("tof_ground_ref_valid khong con gac duong fusion",
          "if (e->tof_ground_ref_valid)" not in m_upd.group(0))

# floor_ready gio luon true -> khong con chan ARM/TAKEOFF gian tiep.
m_ready = re.search(r"bool alt_estimator_floor_ready\([^)]*\)\s*\{(.*?)\n\}", CODE, re.S)
check("tim thay alt_estimator_floor_ready()", m_ready is not None)
if m_ready:
    rb = m_ready.group(1)
    check("floor_ready khong con doi floor_sample_count/std",
          "floor_sample_count" not in rb and "floor_std_m" not in rb, rb.strip())

# =============================================================================
# (5) MO PHONG
# =============================================================================
print("\n== (5) Mo phong ==")


def decide(hw_alive, fusable, airborne=True):
    if not airborne:
        return "GROUND"
    if not hw_alive:
        return "FAULT"
    return "FUSED" if fusable else "COAST"


# Nam sat san / ngoai tam / bay qua vat the: chip do deu, mau khong dung duoc.
check("chip do + khong mau dung duoc -> COAST (khong fault)",
      decide(True, False) == "COAST")
# Bay binh thuong.
check("chip do + fuse duoc -> FUSED", decide(True, True) == "FUSED")
# Mat cam bien that.
check("chip im -> FAULT", decide(False, False) == "FAULT")
check("chip im van FAULT du co co fusable cu", decide(False, True) == "FAULT")
# Duoi dat: khong xet.
check("chua cat canh -> GROUND", decide(True, False, airborne=False) == "GROUND")

print("")
if fails:
    print("KET QUA: %d FAIL -- %s" % (len(fails), ", ".join(fails)))
    sys.exit(1)
print("KET QUA: ALL PASS")
print("         tuoi mau khong con quyet dinh gi; chi 'chip con do' moi quyet dinh.")
