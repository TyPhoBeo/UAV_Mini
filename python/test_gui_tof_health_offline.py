"""GUI phai phan biet "khong co gi de do" voi "mat cam bien".

VI SAO CAN TEST NAY:
Anh chup man hinh cua nguoi dung:
    Z<-KHONG NGUON | ToF LOI: mau STALE 80784ms (>200) -> mat tin hieu

Hai loi RIENG BIET trong mot dong:

 (1) "Z<-KHONG NGUON" -- bang ALT_SOURCE_NAMES cua GUI LECH HAN enum firmware.
     GUI cu:      0=KHONG NGUON 1=ToF 2=BARO 3=ToF+BARO
     alt_source_t: 0=GROUND_LOCK 1=IMU_PREDICT 2=TOF_FUSED 3=TOF_SHORT_BRIDGE
                   4=TOF_LOST
     Nam tren san (ALTSRC=0 = GROUND_LOCK, HOAN TOAN BINH THUONG) bi hien do
     loet thanh "KHONG NGUON". Va nguy hiem hon: dang bay bang ToF (ALTSRC=2 =
     TOF_FUSED) lai hien "BARO" trong khi baro da TAT tu lau.

 (2) "mau STALE 80784ms -> mat tin hieu" -- GUI ket luan mat cam bien chi tu
     TOFAGE (tuoi mau HOP LE). Dat drone xuong san thi ToF doc 0.000m (duoi tam
     mu ~4cm cua VL53L1X), mau bi loai -> TOFAGE tang vo han TRONG KHI chip van
     do deu moi 40ms. Firmware da duoc sua de phan biet (TOFALIVE); GUI thi
     chua.

Bao do cho mot tinh huong binh thuong khong chi gay hoang mang -- no day nguoi
dung toi cho quen canh bao do, nen luc mat cam bien THAT thi khong ai nhin nua.
"""
import importlib.util
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
GUI_PATH = ROOT / "tools/uav_udp_console.py"
FMT_C = (ROOT / "src/telemetry_format.c").read_text(encoding="utf-8")
EST_H = (ROOT / "components/flight_core/include/flight_core/alt_estimator.h").read_text(encoding="utf-8")

fails = []


def check(name, cond, detail=""):
    if cond:
        print("  PASS  %s" % name)
    else:
        print("  FAIL  %s%s" % (name, ("  -- " + detail) if detail else ""))
        fails.append(name)


# Nap GUI nhu mot module (khong chay main).
spec = importlib.util.spec_from_file_location("gui", GUI_PATH)
gui = importlib.util.module_from_spec(spec)
try:
    spec.loader.exec_module(gui)
except SystemExit:
    pass

# =============================================================================
# (1) ALT_SOURCE_NAMES phai KHOP enum firmware
# =============================================================================
# Doc enum TU SOURCE thay vi go tay lai: go tay nghia la test van xanh khi ai do
# doi enum ben firmware ma quen doi GUI -- dung cai loi dang sua.
print("== (1) ALT_SOURCE_NAMES khop alt_source_t ==")

m_enum = re.search(r"typedef enum \{(.*?)\} alt_source_t;", EST_H, re.S)
check("tim thay alt_source_t", m_enum is not None)
if m_enum:
    names = re.findall(r"(ALT_SRC_\w+)", m_enum.group(1))
    check("enum co dung 5 gia tri", len(names) == 5, str(names))

    # Thuoc tinh cua class PidTunerApp, khong phai module-level.
    tbl = gui.PidTunerApp.ALT_SOURCE_NAMES
    check("GUI co du 5 ma (0..4)", sorted(tbl.keys()) == ["0", "1", "2", "3", "4"],
          str(sorted(tbl.keys())))

    # Ma 4 (TOF_LOST) truoc day GUI KHONG HE CO -> mat cam bien that thi GUI
    # khong hien gi ca.
    check("co ma 4 = TOF_LOST (ban cu thieu han)", "4" in tbl)

    # Kiem tung ma mang dung Y NGHIA, khong chi dung so luong.
    if "0" in tbl:
        check('ma 0 KHONG con la "KHONG NGUON" (do la GROUND_LOCK)',
              "KHONG NGUON" not in tbl["0"][0], tbl["0"][0])
        check("ma 0 khong to do (nam dat la binh thuong)",
              tbl["0"][1] != "#c0392b", tbl["0"][1])
    if "2" in tbl:
        check('ma 2 = ToF (TOF_FUSED), KHONG phai "BARO"',
              "ToF" in tbl["2"][0] and "BARO" not in tbl["2"][0], tbl["2"][0])
        check("ma 2 to xanh (dang fuse ToF = trang thai tot)",
              tbl["2"][1] == "#0a7d2c", tbl["2"][1])
    if "4" in tbl:
        check("ma 4 to do (mat ToF that su)", tbl["4"][1] == "#c0392b", tbl["4"][1])

# =============================================================================
# (2) Firmware CO gui TOFALIVE ra day
# =============================================================================
print("\n== (2) Firmware phat TOFALIVE ==")
check("telemetry_format.c co TOFALIVE=", "TOFALIVE=%d" in FMT_C)
check("STATUS_RE bat duoc TOFALIVE", "TOFALIVE=" in GUI_PATH.read_text(encoding="utf-8"))

# =============================================================================
# (3) _tof_health: nam dat KHONG do, mat cam bien VAN do
# =============================================================================
print("\n== (3) _tof_health phan biet hai ca ==")

RED = "#c0392b"


class Fake:
    """Chi can TOF_STALE_MS + method that -- khong dung tkinter."""
    TOF_STALE_MS = gui.PidTunerApp.TOF_STALE_MS
    _tof_health = gui.PidTunerApp._tof_health
    # _tof_health() goi _tof_reject_reason() de giai thich VI SAO mau bi loai
    # (doc TOFRS/TOFSIG/TOFAMB). Phai co ca hai o day, neu khong Fake se lech
    # khoi lop that va test bao loi gia.
    _tof_reject_reason = gui.PidTunerApp._tof_reject_reason
    # None = firmware cu / chua nhan dong STATUS nao co ba field chan doan.
    # _tof_reject_reason() tra "" -> thong bao giu nguyen dang cu.
    _last_tof_diag = None


f = Fake()

# --- Chieu 1: NAM TREN SAN. TOFAGE khong lo, nhung chip van do deu. ---
txt, color = f._tof_health("80784", "0.00", tofen="1", tofalive="40")
check("nam dat (age=80784, alive=40) -> KHONG to do", color != RED, "%s | %s" % (color, txt))
check("nam dat -> noi ro 'khong co gi de do'",
      "khong co gi de do" in txt.lower() or "sat san" in txt.lower(), txt)

# --- Chieu 2: MAT CAM BIEN THAT. Ca hai moc deu qua han. ---
txt, color = f._tof_health("80784", "0.00", tofen="1", tofalive="80784")
check("ToF chet (age=80784, alive=80784) -> VAN to do", color == RED, "%s | %s" % (color, txt))
check("ToF chet -> noi 'mat tin hieu'", "mat tin hieu" in txt.lower(), txt)

# --- alive = -1: chua tung do duoc mau nao -> phai coi la chet. ---
txt, color = f._tof_health("-1", "0.00", tofen="1", tofalive="-1")
check("alive=-1 (chua tung do duoc) -> to do", color == RED, "%s | %s" % (color, txt))

# --- Chip do duoc nhung chua mau hop le nao (dung luc vua boot tren san). ---
txt, color = f._tof_health("-1", "0.00", tofen="1", tofalive="40")
check("age=-1 nhung alive=40 -> KHONG to do", color != RED, "%s | %s" % (color, txt))

# --- Bay binh thuong. ---
txt, color = f._tof_health("45", "1.20", tofen="1", tofalive="45")
check("bay binh thuong -> xanh", color == "#0a7d2c", "%s | %s" % (color, txt))

# --- ToF tat luc bien dich: khong duoc bao loi. ---
txt, color = f._tof_health("-1", "0.00", tofen="0")
check("SENSOR_TOF_ENABLED=0 -> xam, khong phai loi", color == "gray", txt)

# --- TUONG THICH NGUOC: firmware cu khong gui TOFALIVE. ---
# Khong duoc am tham bo qua loi that chi vi thieu truong moi.
txt, color = f._tof_health("80784", "0.00", tofen="1", tofalive=None)
check("firmware cu (khong co TOFALIVE) -> giu hanh vi cu, van to do",
      color == RED, "%s | %s" % (color, txt))

# =============================================================================
# (4) Bien quanh nguong
# =============================================================================
print("\n== (4) Bien quanh TOF_STALE_MS ==")
S = Fake.TOF_STALE_MS

_, c_at = f._tof_health("99999", "0.00", tofen="1", tofalive=str(S))
check("alive = TOF_STALE_MS -> con song (khong do)", c_at != RED, c_at)

_, c_over = f._tof_health("99999", "0.00", tofen="1", tofalive=str(S + 1))
check("alive = TOF_STALE_MS+1 -> chet (do)", c_over == RED, c_over)

print("")
if fails:
    print("KET QUA: %d FAIL -- %s" % (len(fails), ", ".join(fails)))
    sys.exit(1)
print("KET QUA: ALL PASS")
print("         ALTSRC hien dung ten; nam dat khong bao do; mat ToF that van do.")
