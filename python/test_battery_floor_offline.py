"""San pin -> ep LANDING, va vi sao BAT BUOC phai co debounce.

Yeu cau: BATV <= 3.10V thi ep ha canh -- so voi TRUNG VI, khong phai mau tho.

⚠ KHONG duoc kiem tung mau. Do tren log hover THAT:
    BATV: 3.14 .. 3.93  quanh trung binh 3.48   (bien do 0.79V!)
Pin THAT o 3.4V co the doc ra 3.00V tren mot mau nhieu -> mot glitch ADC ep
HA CANH GIUA CHUYEN. Nen phai giu duoi san LIEN TUC bay nhieu moi trip.

Con mot cai bay nua: ADC tra 4.936V cho pin 1S (do duoc) -> BATVALID=0 ->
battery_v = 0.00. Mau nhu vay KHONG duoc xoa dong ho, neu khong thi ADC chap
chon se lam fault khong bao gio trip duoc.
"""
import io, os, re, sys

FAILED = []


def check(name, cond, detail=""):
    print("  %s  %s" % ("PASS" if cond else "FAIL", name))
    if not cond:
        if detail:
            print("        %s" % detail)
        FAILED.append(name)


def read(rel):
    p = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), rel)
    return io.open(p, encoding="utf-8", errors="replace").read()


def strip_c(s):
    s = re.sub(r"/\*.*?\*/", " ", s, flags=re.S)
    return re.sub(r"//[^\n]*", "", s)


CMD = strip_c(read("components/flight_core/src/commander.c"))
HDRC = read("components/flight_core/include/flight_core/commander.h")
TUN = read("components/flight_core/include/flight_core/tuning.h")


def const_of(name, src):
    m = re.search(r"#define\s+" + re.escape(name) + r"\s+([0-9.]+)f?\b", src)
    return float(m.group(1)) if m else None


FLOOR = const_of("COMMANDER_DEFAULT_BATTERY_FLOOR_V", TUN)
HOLD = const_of("COMMANDER_BATTERY_LOW_HOLD_MS", TUN)

print("== (1) San pin dat dung ==")
check("COMMANDER_DEFAULT_BATTERY_FLOOR_V = 3.10", FLOOR == 3.10,
      "dang la %s" % FLOOR)
check("khong con 2.0V (gia tri cu khong bao gio trip)", FLOOR != 2.0)

print()
print("== (2) Co debounce ==")
check("co hang so thoi gian giu", HOLD is not None and HOLD > 0, str(HOLD))
check("state co dong ho battery_low_since_us",
      re.search(r"int64_t\s+battery_low_since_us\s*;", HDRC) is not None)
check("commander_init xoa dong ho",
      re.search(r"battery_low_since_us\s*=\s*0", HDRC) is not None)
check("fault chi trip khi da giu du lau",
      re.search(r"battery_low_since_us\s*\)\s*>=\s*\(int64_t\)\s*"
                r"COMMANDER_BATTERY_LOW_HOLD_MS", CMD) is not None,
      "kiem tung mau -> mot glitch ADC ep ha canh giua chuyen")

print()
print("== (3) Mau KHONG hop le khong duoc xoa dong ho ==")
m = re.search(r"if\s*\([^)]*state\s*!=\s*FSM_LANDING[^)]*battery_v\s*>\s*0\.0f\s*\)\s*\{(.*?)\n    \}",
              CMD, re.S)
check("cong battery_v > 0 bao NGOAI ca nhanh xoa", m is not None,
      "neu battery_v==0 roi vao else thi ADC chap chon = fault khong bao gio trip")
if m:
    body = m.group(1)
    check("nhanh else (tren san) moi xoa dong ho",
          re.search(r"else\s*\{[^}]*battery_low_since_us\s*=\s*0", body, re.S) is not None)

print()
print("== (4) La SOFT fault (-> LANDING), khong phai HARD (cat motor) ==")
i = CMD.find("battery_low_since_us")
seg = CMD[i:i + 700] if i >= 0 else ""
check("dung FAULT_SOFT", "FAULT_SOFT" in seg)
check("KHONG dung FAULT_HARD", "FAULT_HARD" not in seg,
      "cat motor khi pin yeu = roi thang, phai ha canh co kiem soat")

print()
print("== (5) Mo phong tren BATV THAT ==")
DT = 100  # ADC pin ~10Hz


def sim(seq, floor, hold_ms):
    t = 0
    since = None
    for v in seq:
        t += DT
        if v > 0.0:                     # mau hop le
            if v <= floor:
                if since is None:
                    since = t
                if t - since >= hold_ms:
                    return True, t
            else:
                since = None            # tren san -> xoa
        # v == 0: mau hong -> GIU nguyen dong ho
    return False, None


hover = [3.58, 3.34, 3.34, 3.33, 3.14, 3.36, 3.57, 3.33, 3.80, 3.55, 3.68, 3.93]
glitch = [3.45, 3.40, 3.02, 3.44, 3.38, 3.41, 2.99, 3.50, 3.42]   # 2 mau nhieu don le
dead = [3.02, 3.01, 2.98, 3.00, 2.95, 3.03, 2.99, 2.97, 3.01, 2.96, 2.94, 2.99]
adc = [3.00, 0.0, 2.98, 0.0, 3.01, 0.0, 2.97, 0.0, 2.99, 0.0, 2.96, 0.0, 2.95, 0.0]

for ten, seq, mong in (("hover binh thuong", hover, False),
                       ("2 glitch don le", glitch, False),
                       ("pin het THAT", dead, True),
                       ("pin het + ADC chap chon", adc, True)):
    got, t = sim(seq, FLOOR, HOLD)
    print("  %-26s -> %s%s" % (ten, "LANDING" if got else "khong trip",
                                " @%dms" % t if t else ""))
    check("%s: %s" % (ten, "phai trip" if mong else "KHONG duoc trip"), got == mong)

print()



# =====================================================================
# (N) LOC TRUNG VI -- phan quyet dinh, them sau khi bay thu
# =====================================================================
# LOI THAT: nguoi dung de xuat "chi can 1 mau <= 3.1V la ep landing".
# Do tren log hover that:
#     BATV  2.87 .. 3.37   (spread 0.50 V)
#     ga    1244 .. 1286   (bien thien 3.4%)
# Tai gan nhu dung yen ma dien ap swing gap 5 lan -> ripple + nhieu ADC,
# khong phai pin sut. battery_driver dung adc_oneshot_read: MOT mau tho,
# khong trung binh (BFILT=0.00 BSTD=0.00 trong telemetry xac nhan).
# Mot mau tho so voi san 3.10 se ep ha canh khi pin THAT con 3.16 V.
print()
print("== (N) battery_v phai la TRUNG VI, khong phai mau tho ==")

FC = read("components/flight_core/src/flight_core.c")
check("battery_v lay qua hover_vbat_median()",
      "hover_vbat_median(&s_vbat_ring, &battery_med_v)" in FC,
      "con doc thang snap.battery.voltage_v -> bao dong gia")
check("KHONG con gan battery_v = mau tho",
      "battery_v = snap.battery_h.valid ? snap.battery.voltage_v" not in FC,
      "duong cu van con -> loc bi bo qua")
check("van giu quy uoc health invalid -> 0.0f (Commander bo qua)",
      re.search(r"hover_vbat_median\([^)]*\)\)\s*\?\s*battery_med_v\s*:\s*0\.0f", FC) is not None)

# Ring co san (hover latch) duoc tai dung: no da khu trung lap theo seq --
# dung van de gap khi doc log (mot mau ADC bi in lai 2-4 lan vi telemetry
# 50ms nhanh hon ADC ~125ms).
HM = read("components/flight_core/include/flight_core/hover_model.h")
check("ring khu trung lap theo seq (mot mau khong bi dem nhieu lan)",
      re.search(r"seq\s*!=\s*.*last_seq|last_seq", HM) is not None)
N_SAMP = const_of("HOVER_MODEL_SAMPLES", HM)
check("cua so trung vi = 5 mau", N_SAMP == 5, "dang la %s" % N_SAMP)

print()
print("== (N.1) Mo phong tren DU LIEU LOG THAT ==")
def med(xs):
    t = sorted(xs); n = len(t)
    return t[n // 2] if n % 2 else 0.5 * (t[n // 2 - 1] + t[n // 2])

def first_below(seq, floor, filt):
    """Chi so mau dau tien bao 'duoi san'; None neu khong bao gio."""
    for i in range(len(seq)):
        if not filt:
            if seq[i] <= floor: return i
        else:
            if i < 4: continue
            if med(seq[i - 4:i + 1]) <= floor: return i
    return None

# Mau ADC THAT, da khu lap (telemetry in 50ms, ADC refresh ~125ms).
tot = [3.18, 2.98, 2.87, 3.16, 3.37, 3.34, 3.16, 3.15, 3.16, 3.15]
i_raw, i_med = first_below(tot, FLOOR, False), first_below(tot, FLOOR, True)
print("  pin CON TOT (trung binh %.2f V):" % (sum(tot) / len(tot)))
print("    mau tho   -> %s" % ("bao o mau %d" % (i_raw + 1) if i_raw is not None else "im"))
print("    trung vi  -> %s" % ("bao o mau %d" % (i_med + 1) if i_med is not None else "im"))
check("mau tho BAO DONG GIA (tai hien duoc de xuat bi bac)", i_raw is not None)
check("trung vi IM LANG tren pin con tot", i_med is None,
      "bao o mau %s -> loc khong an" % i_med)

yeu = [3.20, 2.90, 3.05, 2.72, 3.08, 2.85, 2.98, 2.75, 3.12, 2.88]
j = first_below(yeu, FLOOR, True)
print("  pin YEU THAT (trung binh %.2f V):" % (sum(yeu) / len(yeu)))
print("    trung vi  -> %s" % ("bat duoc o mau %d (~%.1f s)" % (j + 1, j * 0.125) if j is not None else "BO SOT"))
check("trung vi VAN bat duoc pin yeu that", j is not None,
      "loc qua manh -> guard mu")

print()
if FAILED:
    print("KET QUA: %d FAIL -- %s" % (len(FAILED), FAILED[0]))
    sys.exit(1)
print("KET QUA: TAT CA PASS")
