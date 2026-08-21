"""Test ToF surface-gated correction — TRANSCRIPTION cua
components/flight_core/src/alt_estimator.c (khoi correct_from_tof +
tof_classify_surface) sang Python.

⚠ GIOI HAN: khong phai test tren binary da bien dich (may nay khong co host C
compiler). Day la ban chep lai THUAT TOAN. No bat duoc loi logic/thu tu/gate;
khong bat duoc loi kieu C.

TEST QUAN TRONG NHAT: TEST 42 (long table). UAV hover 0.80m, bay len mat ban cao
0.40m, giu ToF=0.40m trong 60 GIAY. Z_UAV PHAI van ~0.80m va ToF PHAI KHONG bao
gio tu reacquire theo thoi gian. Do la yeu cau PASS/FAIL cua ca task.
"""
import sys

# ---- Hang so — CHEP TU alt_estimator.h ----
TOF_MIN_RANGE_M = 0.03
TOF_MAX_RANGE_M = 1.80
TOF_TILT_MIN_COS = 0.87
TOF_FLOOR_MATCH_GATE_M = 0.10
TOF_SURFACE_STEP_GATE_M = 0.18
TOF_FLOOR_TICKS = 3
TOF_OTHER_TICKS = 3
TOF_REACQUIRE_GATE_M = 0.07
TOF_REACQUIRE_TICKS = 6
TOF_ALPHA = 0.25
TOF_BETA = 0.02
TOF_VZ_CORRECTION_MAX_MS = 0.30

UNKNOWN, FLOOR, OTHER = 0, 1, 2
SURF_NAMES = {UNKNOWN: "UNKNOWN", FLOOR: "FLOOR", OTHER: "OTHER"}


def clampf(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class Est:
    """alt_estimator_t — chi phan lien quan ToF."""

    def __init__(self, alt_m=0.0, vz_ms=0.0, ground_range=0.05):
        self.alt_m = alt_m
        self.vz_ms = vz_ms
        self.floor_plane_z_m = 0.0
        self.tof_ground_range_m = ground_range
        self.tof_ground_ref_valid = ground_range > 0.0
        self.tof_raw_m = 0.0
        self.tof_vertical_m = 0.0
        self.tof_innovation_m = 0.0
        self.tof_surface_z_m = 0.0
        self.tof_tilt_cos = 1.0
        self.tof_surface_state = UNKNOWN
        self.tof_floor_ticks = 0
        self.tof_other_ticks = 0
        self.tof_correction_enabled = False
        self.tof_accept_count = 0
        self.tof_reject_count = 0
        self.landing_surface_z_m = 0.0
        self.landing_surface_valid = False

    # ---- chep tof_classify_surface() ----
    def classify(self, innovation):
        matches_floor = abs(innovation) <= TOF_FLOOR_MATCH_GATE_M
        higher_surface = innovation <= -TOF_SURFACE_STEP_GATE_M

        if self.tof_surface_state == OTHER:
            if abs(innovation) <= TOF_REACQUIRE_GATE_M:
                self.tof_floor_ticks = min(self.tof_floor_ticks + 1, TOF_REACQUIRE_TICKS)
            else:
                self.tof_floor_ticks = 0
            if self.tof_floor_ticks >= TOF_REACQUIRE_TICKS:
                self.tof_surface_state = FLOOR
                self.tof_other_ticks = 0
            return

        if higher_surface:
            self.tof_other_ticks = min(self.tof_other_ticks + 1, TOF_OTHER_TICKS)
        else:
            self.tof_other_ticks = 0
        if self.tof_other_ticks >= TOF_OTHER_TICKS:
            self.tof_surface_state = OTHER
            self.tof_floor_ticks = 0
            return

        if matches_floor:
            self.tof_floor_ticks = min(self.tof_floor_ticks + 1, TOF_FLOOR_TICKS)
        else:
            self.tof_floor_ticks = 0
        if self.tof_floor_ticks >= TOF_FLOOR_TICKS:
            self.tof_surface_state = FLOOR

    # ---- chep correct_from_tof() ----
    def correct_from_tof(self, range_m, tilt_cos, dt_tof_s, inertial_enabled):
        self.tof_raw_m = range_m
        self.tof_tilt_cos = tilt_cos
        self.tof_correction_enabled = False

        if (range_m < TOF_MIN_RANGE_M or range_m > TOF_MAX_RANGE_M
                or tilt_cos < TOF_TILT_MIN_COS or not self.tof_ground_ref_valid):
            self.tof_reject_count += 1
            return

        vertical = range_m * tilt_cos
        self.tof_vertical_m = vertical
        self.tof_surface_z_m = (self.alt_m + self.tof_ground_range_m) - vertical

        if not inertial_enabled:
            self.tof_innovation_m = 0.0
            return

        predicted = (self.alt_m - self.floor_plane_z_m) + self.tof_ground_range_m
        innovation = vertical - predicted
        self.tof_innovation_m = innovation

        self.classify(innovation)

        # HAI dieu kien (xem alt_estimator.c): phan loai BEN VUNG + gate TUNG MAU.
        # Thieu gate thi trong cua so cho xac nhan OTHER, correction van chay
        # voi innovation lon -> Z tut truoc khi ban duoc nhan ra.
        if (self.tof_surface_state != FLOOR
                or abs(innovation) > TOF_FLOOR_MATCH_GATE_M):
            self.tof_reject_count += 1
            return

        self.alt_m += TOF_ALPHA * innovation
        dvz = clampf(TOF_BETA * innovation / dt_tof_s,
                     -TOF_VZ_CORRECTION_MAX_MS, TOF_VZ_CORRECTION_MAX_MS)
        self.vz_ms += dvz
        self.tof_accept_count += 1
        self.tof_correction_enabled = True

    def select_landing_surface(self):
        if not self.tof_ground_ref_valid or self.tof_vertical_m <= 0.0:
            self.landing_surface_valid = False
            return False
        self.landing_surface_z_m = self.tof_surface_z_m
        self.landing_surface_valid = True
        return True

    def height_above_landing_surface(self):
        if not self.landing_surface_valid:
            return self.alt_m
        return self.alt_m - self.landing_surface_z_m


# Ground offset THUC TE: sensor lap cach san ~3cm. KHONG dung 0.0 — firmware
# (dung) coi ground_range < ALT_EST_TOF_MIN_RANGE_M(0.03) la ref KHONG hop le,
# va sensor that cung khong bao gio doc 0 khi UAV nam tren san.
G = 0.03

DT_TOF = 1.0 / 30.0
FAILS = []


def check(name, cond, extra=""):
    print(("  OK   " if cond else "  FAIL ") + name + (f"   {extra}" if extra else ""))
    if not cond:
        FAILS.append(name)


def feed(est, tof_range, n, tilt_cos=1.0, inertial=True):
    """Bom n mau ToF cung gia tri."""
    for _ in range(n):
        est.correct_from_tof(tof_range, tilt_cos, DT_TOF, inertial)


# ============================================================================
print("TEST 38: GROUND — inertial chua active, ToF khong sua gi")
e = Est(alt_m=0.0, ground_range=0.05)
feed(e, 0.05, 20, inertial=False)
check("Z van 0", e.alt_m == 0.0, f"Z={e.alt_m}")
check("khong correction", not e.tof_correction_enabled)
check("surface_z tinh duoc cho telemetry", abs(e.tof_surface_z_m - 0.0) < 1e-6,
      f"surf_z={e.tof_surface_z_m}")

# ============================================================================
print("\nTEST 40: ToF small correction — Z=0.50, ToF san=0.48 -> sua MOT PHAN")
# ground_range=0 cho de doi chieu voi vi du trong spec
e = Est(alt_m=0.50, ground_range=0.02)
# ToF vertical tuong ung san o Z=0: range = z + ground_range = 0.52
# Gia lap sensor doc 0.50 (san hoi gan hon du doan 0.02m)
z_before = e.alt_m
feed(e, 0.50, 3)   # sensor doc 0.50, du doan 0.53 -> san gan hon 3cm
check("vao FLOOR", e.tof_surface_state == FLOOR, SURF_NAMES[e.tof_surface_state])
check("Z chi sua MOT PHAN, khong nhay", z_before > e.alt_m > 0.48,
      f"Z {z_before:.4f} -> {e.alt_m:.4f}")
check("KHONG set thang z = do duoc", abs(e.alt_m - 0.48) > 1e-3, f"Z={e.alt_m:.4f}")

# ============================================================================
print("\nTEST 41: TABLE STEP — Z=0.80, ToF 0.80 -> 0.40")
e = Est(alt_m=0.80, ground_range=G)
feed(e, 0.80 + G, 5)                  # dang nhin san
check("truoc do la FLOOR", e.tof_surface_state == FLOOR, SURF_NAMES[e.tof_surface_state])
z_on_floor = e.alt_m
feed(e, 0.43, TOF_OTHER_TICKS)        # bay len mat ban cao 0.40 (0.80+G-0.40)
check("phat hien OTHER", e.tof_surface_state == OTHER, SURF_NAMES[e.tof_surface_state])
check("ToF correction TAT", not e.tof_correction_enabled)
check("surface_z ~ 0.40", abs(e.tof_surface_z_m - 0.40) < 0.02, f"={e.tof_surface_z_m:.3f}")
check("Z van ~0.80 (khong tut theo ToF)", abs(e.alt_m - 0.80) < 0.05, f"Z={e.alt_m:.4f}")

# ============================================================================
print("\nTEST 42 (PASS/FAIL CRITICAL): LONG TABLE — giu ToF=0.40 trong 60 GIAY")
e = Est(alt_m=0.80, ground_range=G)
feed(e, 0.80 + G, 5)
acc_before_table = e.tof_accept_count   # (2 mau dau bi reject vi state khoi dau UNKNOWN)
n_60s = int(60.0 / DT_TOF)   # ~1800 mau
feed(e, 0.43, n_60s)
print(f"       (da bom {n_60s} mau ToF = 60s)")
check("surface_state VAN OTHER sau 60s", e.tof_surface_state == OTHER,
      SURF_NAMES[e.tof_surface_state])
check("ToF correction VAN TAT sau 60s", not e.tof_correction_enabled)
check("Z_UAV VAN ~0.80 sau 60s (KHONG bi keo ve 0.40)",
      abs(e.alt_m - 0.80) < 0.05, f"Z={e.alt_m:.4f}")
check("accept_count KHONG tang MOT LAN NAO trong suot 60s tren ban",
      e.tof_accept_count == acc_before_table,
      f"accept {acc_before_table} -> {e.tof_accept_count} sau 1800 mau ban")
check("surface_z on dinh ~0.40", abs(e.tof_surface_z_m - 0.40) < 0.02,
      f"={e.tof_surface_z_m:.3f}")

# ============================================================================
print("\nTEST 43: EXIT TABLE — ToF 0.40 -> 0.80, khop lai san da khoa")
# tiep tuc tu trang thai TEST 42
before = e.alt_m
feed(e, 0.80 + G, TOF_REACQUIRE_TICKS - 1)
check("chua du mau -> VAN OTHER", e.tof_surface_state == OTHER,
      SURF_NAMES[e.tof_surface_state])
feed(e, 0.80 + G, 1)
check("du REACQUIRE_TICKS -> ve FLOOR", e.tof_surface_state == FLOOR,
      SURF_NAMES[e.tof_surface_state])
feed(e, 0.80 + G, 3)
check("correction bat lai", e.tof_correction_enabled)
check("khong co buoc nhay Z", abs(e.alt_m - before) < 0.05,
      f"Z {before:.4f} -> {e.alt_m:.4f}")

# ============================================================================
print("\nTEST 44: REAL DESCENT — tut that 0.80 -> 0.40, KHONG duoc coi la ban")
# Day la case doi lap voi TEST 41: cung ToF 0.80 -> 0.40 NHUNG z_uav cung tut
# theo (quan tinh thay). Innovation phai ~0 -> van FLOOR.
e = Est(alt_m=0.80, ground_range=G)
feed(e, 0.80 + G, 5)
check("bat dau o FLOOR", e.tof_surface_state == FLOOR)
steps = 30
for i in range(steps):
    # UAV tut that: quan tinh (prediction) da ha z_uav truoc khi ToF toi
    e.alt_m -= 0.40 / steps
    tof_now = (0.80 + G) - 0.40 * (i + 1) / steps
    e.correct_from_tof(tof_now, 1.0, DT_TOF, True)
check("VAN la FLOOR (khong nham thanh ban)", e.tof_surface_state == FLOOR,
      SURF_NAMES[e.tof_surface_state])
check("ToF correction VAN bat", e.tof_correction_enabled)
check("Z theo dung xuong ~0.40", abs(e.alt_m - 0.40) < 0.05, f"Z={e.alt_m:.4f}")

# ============================================================================
print("\nTEST: TILT GATE — nghieng qua -> reject, KHONG phan loai nham")
e = Est(alt_m=0.80, ground_range=G)
feed(e, 0.80 + G, 5)
st_before = e.tof_surface_state
rej_before = e.tof_reject_count
# tilt_cos = 0.70 (~45 deg) -> duoi nguong 0.87
feed(e, 0.43, 10, tilt_cos=0.70)
check("mau nghieng bi reject", e.tof_reject_count > rej_before)
check("surface_state GIU NGUYEN (khong ket luan tu mau xau)",
      e.tof_surface_state == st_before, SURF_NAMES[e.tof_surface_state])
check("khong correction", not e.tof_correction_enabled)

# ============================================================================
print("\nTEST: OUT OF RANGE — ngoai dai tin duoc -> reject")
e = Est(alt_m=0.80, ground_range=G)
feed(e, 0.80 + G, 5)
st_before = e.tof_surface_state
feed(e, 3.0, 10)      # > MAX_RANGE
check("range qua xa bi reject", not e.tof_correction_enabled)
check("surface_state giu nguyen", e.tof_surface_state == st_before)

# ============================================================================
print("\nTEST: GROUND REF — sensor cach san 5cm, san van phai la FLOOR")
e = Est(alt_m=0.0, ground_range=0.05)
# UAV o Z=0.50 -> range du doan toi san = 0.50 + 0.05 = 0.55
e.alt_m = 0.50
feed(e, 0.55, 5)
check("nhan dung la FLOOR (co bu ground offset)", e.tof_surface_state == FLOOR,
      SURF_NAMES[e.tof_surface_state])
check("innovation ~ 0", abs(e.tof_innovation_m) < 0.02, f"innov={e.tof_innovation_m:.4f}")
# Neu QUEN bu ground_range thi innovation = 0.55-0.50 = +0.05 (van trong gate),
# nhung voi offset lon hon thi se lech han -> kiem tra truong hop 15cm:
e2 = Est(alt_m=0.50, ground_range=0.15)
feed(e2, 0.65, 5)
check("ground offset 15cm van FLOOR", e2.tof_surface_state == FLOOR,
      SURF_NAMES[e2.tof_surface_state])
e3 = Est(alt_m=0.50, ground_range=G)   # ground ref SAI (0.03 thay vi 0.15)
feed(e3, 0.65, 5)
# Ground ref SAI (0.03 thay vi 0.15 that) -> innovation lech dung 0.12 = sai so
# cua ground ref. Chung minh: dat sai ground ref se lam ToF phan loai nham.
check("ground ref SAI -> innovation lech dung bang sai so ref",
      abs(e3.tof_innovation_m - 0.12) < 0.02, f"innov={e3.tof_innovation_m:.4f}")

# ============================================================================
print("\nTEST 48: LANDING ON TABLE — floor != landing surface")
e = Est(alt_m=0.80, ground_range=G)
feed(e, 0.80 + G, 5)
feed(e, 0.43, TOF_OTHER_TICKS)     # dang tren ban
check("dang o OTHER", e.tof_surface_state == OTHER)
floor_before = e.floor_plane_z_m
ok = e.select_landing_surface()
check("chon duoc be mat ha canh", ok)
check("landing_surface_z ~0.40", abs(e.landing_surface_z_m - 0.40) < 0.02,
      f"={e.landing_surface_z_m:.3f}")
check("floor_plane_z KHONG bi ghi de", e.floor_plane_z_m == floor_before,
      f"floor={e.floor_plane_z_m}")
h = e.height_above_landing_surface()
check("do cao tren ban ~0.40 (KHONG phai 0.80 world-Z)", abs(h - 0.40) < 0.05,
      f"h={h:.3f}")

# ============================================================================
print("\nTEST 46: SAME-SAMPLE — mot mau ToF chi correction MOT LAN")
# Mo phong gate seq o alt_estimator_update(): 250Hz tick, ToF 30Hz.
e = Est(alt_m=0.50, ground_range=G)
feed(e, 0.50 + G, 5)                # vao FLOOR
acc0 = e.tof_accept_count
seq = 100
last_seq, seq_init = 0, False
for tick in range(10):              # 10 tick estimator, CUNG mot seq
    tof_new = (not seq_init) or (seq != last_seq)
    if tof_new:
        last_seq, seq_init = seq, True
        e.correct_from_tof(0.50 + G, 1.0, DT_TOF, True)
check("10 tick cung seq -> chi 1 correction", e.tof_accept_count == acc0 + 1,
      f"accept {acc0} -> {e.tof_accept_count}")

print()
if FAILS:
    print(f"FAIL: {len(FAILS)} test -> {FAILS}")
    sys.exit(1)
print("PASS: tat ca test ToF surface-gated (gom TEST 42 long-table 60s CRITICAL)")
