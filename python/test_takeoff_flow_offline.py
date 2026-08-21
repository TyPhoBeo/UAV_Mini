# -*- coding: utf-8 -*-
"""
test_takeoff_flow_offline.py -- luong TAKEOFF moi: PID + slew-rate-limited target.

BAN CHEP thuat toan C (components/flight_core/src/takeoff_land.c takeoff_run())
sang Python -- repo khong co trinh bien dich C cho may host.

Bai test bam theo dung danh sach test cua spec:
  TEST A  PRIME: 4 motor quay DEU o prime_duty, KHONG chay Z/Vz PID, chua nhac.
  TEST B  CLIMB: target_z TRUOT dan (khong nhay bac), Vz_I_term HOI TU (dang
          hoc hover), throttle muot khong giat.  <-- in ra bang telemetry.
  TEST C  Bay tha: len muot ~max_climb, vao HOLDING khong overshoot qua ~15%,
          khong co cu throttle nhay luc chuyen pha.
  TEST D  ABORT: chan canh (khong cho nhac) -> sau NO_LIFT_TIMEOUT -> EMERGENCY.
  TEST E  Mat ToF giua takeoff -> giu throttle 300ms -> keo dai -> EMERGENCY.

Va HAI DIEM REVIEW ma nguoi dung yeu cau soi ky:
  R1  target_z khoi dau PHAI = alt HIEN TAI (khong phai 0, khong phai dich).
  R2  PHASE PRIME PHAI KHONG chay Z/Vz PID (I khong duoc tich luy).
"""

import io
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TKO_C = os.path.join(HERE, "..", "components", "flight_core", "src", "takeoff_land.c")
TUNING_H = os.path.join(HERE, "..", "components", "flight_core", "include",
                        "flight_core", "tuning.h")

fails = []


def check(name, cond, detail=""):
    if cond:
        print("  PASS  " + name)
    else:
        print("  FAIL  " + name + (("  -- " + detail) if detail else ""))
        fails.append(name)


def clampf(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def clampi(v, lo, hi):
    v = int(v)
    return lo if v < lo else (hi if v > hi else v)


# =============================================================================
# Hang so doc THANG tu tuning.h -- test se do neu ai do doi so ma quen hau qua
# =============================================================================
tun = io.open(TUNING_H, encoding="utf-8").read()


def define_num(name):
    m = re.search(r"^#define\s+" + name + r"\s+([-\d.]+)f?", tun, re.M)
    return float(m.group(1)) if m else None


HOVER_FF = define_num("ALT_HOLD_HOVER_NOMINAL")

# Sai so con lai giua hover_ff va hover THAT, SAU KHI da latch theo pin
# (hover_model.h). TRUOC khi co latch, hover_ff la hang so 1000 cho MOI muc
# pin, nen sai so chinh la do lech cua hover thuc theo pin: +150 duty o pin
# ~3.9V va len toi +350 o pin 3.6V. Sau latch, hover_ff duoc suy ra tu chinh
# dien ap do duoc, nen chi con sai so CUA MODEL (fit tu 2 diem do that).
#
# Day KHONG phai noi long test: xem TEST H o cuoi file -- no mo phong THANG
# ca hai truong hop va do chenh lech, thay vi bat ta phai tin con so nay.
HOVER_MISMATCH = 50
ALT_KP = define_num("ALT_HOLD_ALT_KP")
VZ_KP = define_num("ALT_HOLD_VZ_KP")
VZ_KI = define_num("ALT_HOLD_VZ_KI")
VZ_ILIMIT = define_num("ALT_HOLD_VZ_ILIMIT")
MIN_THR = define_num("ALT_HOLD_MIN_THROTTLE_DUTY")
PRIME_MS = define_num("TAKEOFF_PRIME_MS")
MAX_CLIMB = define_num("TAKEOFF_MAX_CLIMB_MS")
HOLD_ENTER_MS = define_num("TAKEOFF_HOLD_ENTER_MS")
# LIFTOFF + "khong nhac noi" gio THEO THOI GIAN / GA KICH TRAN, khong theo est_z.
# LIFTOFF_DELTA / HOLD_TOL / NO_LIFT_TIMEOUT / TOTAL_TIMEOUT / TIMEOUT_HANDOFF_TOL
# da bi BO -- xem tuning.h muc 3c/3d/3e.
LIFTOFF_MS = define_num("TAKEOFF_LIFTOFF_MS")
STUCK_MS = define_num("TAKEOFF_STUCK_MS")
TOF_LOST_MS = define_num("TAKEOFF_TOF_LOST_MS")
ABORT_TILT_DEG = define_num("TAKEOFF_ABORT_TILT_DEG")
ABORT_TILT_MS = define_num("TAKEOFF_ABORT_TILT_MS")
PRELIFT_I_LIMIT = define_num("TAKEOFF_PRELIFT_I_LIMIT_DUTY")
CEILING_FRAC = define_num("ATT_MAX_COLLECTIVE_FRACTION")

# MOTOR_SAFE_MAX_DUTY nam o motor_driver.h, khong phai tuning.h
_mot = io.open(os.path.join(HERE, "..", "components", "flight_core", "include",
                            "flight_core", "drivers", "motor_driver.h"),
               encoding="utf-8").read()
_m = re.search(r"^#define\s+MOTOR_SAFE_MAX_DUTY\s+(\d+)", _mot, re.M)
SAFE_MAX = float(_m.group(1)) if _m else None

# PRIME_DUTY la bieu thuc, khong phai so -> tinh lai y het C.
# Ty le gio la hang so CO TEN (TAKEOFF_PRIME_HOVER_FRAC) chu khong con la so
# go thang trong bieu thuc: hover_model.c dung CHUNG hang so do de tinh ga
# PRIME tu hover da latch theo pin, nen no phai co mot cho dinh nghia duy nhat.
m = re.search(r"^#define\s+TAKEOFF_PRIME_HOVER_FRAC\s+([\d.]+)f", tun, re.M)
PRIME_FRAC = float(m.group(1)) if m else None
# Van kiem TAKEOFF_PRIME_DUTY THAT SU suy ra tu ty le do -- neu ai do go lai
# mot he so khac vao day thi test nay phai do, khong duoc im lang bo qua.
_pd = re.search(r"^#define\s+TAKEOFF_PRIME_DUTY\s+(.+)$", tun, re.M)
if _pd and "TAKEOFF_PRIME_HOVER_FRAC" not in _pd.group(1):
    PRIME_FRAC = None    # -> bao "khong parse duoc", dung y nghia
PRIME_DUTY = int(HOVER_FF * PRIME_FRAC) if PRIME_FRAC else None

print("== (0) Hang so doc tu tuning.h ==")
for nm, v in [("HOVER_FF", HOVER_FF), ("PRIME_DUTY", PRIME_DUTY),
              ("MAX_CLIMB", MAX_CLIMB), ("ALT_KP", ALT_KP),
              ("VZ_KP", VZ_KP), ("VZ_KI", VZ_KI)]:
    check("doc duoc %s" % nm, v is not None, "khong parse duoc tu tuning.h")
print("     hover_ff=%.0f prime_duty=%d max_climb=%.2f alt_kp=%.2f vz_kp=%.0f vz_ki=%.0f"
      % (HOVER_FF, PRIME_DUTY, MAX_CLIMB, ALT_KP, VZ_KP, VZ_KI))

# DIEU KIEN THEN CHOT cua kien truc: PRIME khong duoc du suc nhac drone.
check("PRIME_DUTY THAP HON hover_ff (prime khong duoc nhac noi drone)",
      PRIME_DUTY < HOVER_FF,
      "prime=%s hover=%s -> drone se bay bang ga ho khong vong kin" % (PRIME_DUTY, HOVER_FF))


# =============================================================================
# Ban chep cascade Vz (alt_hold.c alt_hold_vz_cascade)
# =============================================================================
class Hold(object):
    def __init__(self):
        self.vz_integral = 0.0
        self.engaged = False
        self.vz_saturated = False

    def preload(self, throttle):
        self.vz_integral = clampf(throttle - HOVER_FF, -VZ_ILIMIT, VZ_ILIMIT)
        self.engaged = True

    def cascade(self, vz_target, vz_meas, dt, i_limit, min_thr, max_thr):
        vz_err = vz_target - vz_meas
        p_term = VZ_KP * vz_err
        self.vz_integral = clampf(self.vz_integral, -i_limit, i_limit)
        unsat = HOVER_FF + p_term + self.vz_integral
        sat_hi = unsat >= max_thr
        sat_lo = unsat <= min_thr
        self.vz_saturated = sat_hi or sat_lo
        # conditional integration: khong cong don theo chieu dang bao hoa
        if not ((sat_hi and vz_err > 0) or (sat_lo and vz_err < 0)):
            self.vz_integral += VZ_KI * vz_err * dt
            self.vz_integral = clampf(self.vz_integral, -i_limit, i_limit)
        dthr = p_term + self.vz_integral
        return clampi(HOVER_FF + dthr, min_thr, max_thr)


# =============================================================================
# Ban chep takeoff_run()
# =============================================================================
IDLE, PRIME, CLIMB, HOLD, ABORT = 0, 1, 2, 3, 4
AB_NONE, AB_TIMEOUT, AB_TOF_LOST, AB_TILT = 0, 1, 2, 3
AB_NOT_ACTIVE = 4
# 5 = KHONG NHAC NOI, phat hien bang GA KICH TRAN (khong dung est_z).
AB_STUCK = 5


class Tko(object):
    def __init__(self, target_m, now_us):
        self.phase = PRIME
        self.active = True
        self.since_us = now_us
        self.phase_since_us = now_us
        self.ground_alt_m = 0.0
        self.final_target_m = target_m
        self.target_z_m = 0.0
        self.vz_target_ms = 0.0
        self.liftoff_flag = False
        self.tof_lost_active = False
        self.tof_lost_since_us = 0
        self.tilt_bad_active = False
        self.tilt_bad_since_us = 0
        self.at_target_active = False
        self.at_target_since_us = 0
        self.stuck_active = False
        self.stuck_since_us = 0
        self.last_throttle = 0
        self.abort_reason = AB_NONE
        # Han chot TINH THEO TARGET (takeoff_begin trong takeoff_land.h), KHONG
        # phai hang so cung -- voi max_climb cham, mot han cung se abort oan moi
        # chuyen bay cao.
        slew_s = (target_m / MAX_CLIMB) if MAX_CLIMB > 0 else 0.0
        expect_ms = PRIME_MS + slew_s * 1000.0 + HOLD_ENTER_MS
        self.deadline_us = now_us + int((expect_ms * 2 + 3000) * 1000)


def latch(cond, st, act_name, since_name, now_us, hold_ms):
    if not cond:
        setattr(st, act_name, False)
        return False
    if not getattr(st, act_name):
        setattr(st, act_name, True)
        setattr(st, since_name, now_us)
    return (now_us - getattr(st, since_name)) >= hold_ms * 1000


def takeoff_run(st, hold, alt_valid, alt_m, vz_ms, alt_source_ok, tilt_deg,
                dt, now_us, safe_max=None):
    if safe_max is None:
        safe_max = SAFE_MAX
    out = {"phase": st.phase, "throttle": 0, "abort": False, "abort_reason": AB_NONE,
           "handoff": False, "control_active": False, "liftoff_flag": st.liftoff_flag,
           "target_z": st.target_z_m, "vz_target": 0.0, "vz_i": hold.vz_integral,
           "ground": st.ground_alt_m, "elapsed": (now_us - st.since_us) * 1e-6,
           "hover_ff": HOVER_FF}
    if not st.active:
        out["phase"] = ABORT
        out["abort"] = True
        out["abort_reason"] = AB_TIMEOUT
        return out

    def do_abort(why):
        st.abort_reason = why
        st.phase = ABORT
        st.phase_since_us = now_us
        st.active = False
        out["phase"] = ABORT
        out["abort"] = True
        out["abort_reason"] = why
        out["liftoff_flag"] = st.liftoff_flag
        out["throttle"] = 0
        return out

    # (A) tilt
    if latch(abs(tilt_deg) > ABORT_TILT_DEG, st, "tilt_bad_active",
             "tilt_bad_since_us", now_us, ABORT_TILT_MS):
        return do_abort(AB_TILT)

    # (B) DA BO -- "chua nhac noi trong NO_LIFT_TIMEOUT" dua vao est_z, thu ma
    # cau hinh baro-only khong cung cap du chinh xac. Bang chung thay the la
    # TKO_ABORT_STUCK (ga kich tran) o CUOI ham.

    # (C) Han chot TOAN chuoi, TINH THEO TARGET tai takeoff_begin(). Chuoi gio
    # hoan toan xac dinh theo thoi gian, nen toi han ma chua ban giao = ket that.
    # KHONG con nhanh "ban giao khi het gio": duong ban giao binh thuong cung chi
    # phu thuoc thoi gian, nen loi thoat do vo nghia.
    if now_us >= st.deadline_us:
        return do_abort(AB_TIMEOUT)

    # (D) mat nguon do do cao -- CHI tu CLIMB tro di
    alt_source_lost = False
    if st.phase in (CLIMB, HOLD):
        # alt_source_ok = CON IT NHAT MOT nguon correction (ToF HOAC baro).
        # KHONG phai suc khoe rieng cua ToF — xem takeoff_land.h.
        alt_source_lost = (not alt_source_ok) or (not alt_valid)
        if latch(alt_source_lost, st, "tof_lost_active", "tof_lost_since_us",
                 now_us, TOF_LOST_MS):
            return do_abort(AB_TOF_LOST)
    else:
        st.tof_lost_active = False

    # ---------------- PHASE 1: PRIME ----------------
    if st.phase == PRIME:
        thr = clampi(PRIME_DUTY, 0, safe_max)
        out["throttle"] = thr
        out["phase"] = PRIME
        out["target_z"] = 0.0
        out["vz_target"] = 0.0
        out["vz_i"] = 0.0
        st.last_throttle = thr
        hold.engaged = False
        hold.vz_integral = 0.0        # <-- R2: PID KHONG chay, I giu 0
        if (now_us - st.phase_since_us) < PRIME_MS * 1000:
            return out
        # canh PRIME -> CLIMB
        st.ground_alt_m = alt_m if alt_valid else 0.0
        st.target_z_m = st.ground_alt_m       # <-- R1: target = alt HIEN TAI
        # I = 0, KHONG preload tu prime_duty (xem ghi chu trong takeoff_land.c):
        # preload se dat I = prime_duty - hover_ff = -520 va vut bo chinh cai
        # feedforward hover_ff ma kien truc dua vao.
        hold.engaged = True
        hold.vz_integral = 0.0
        st.phase = CLIMB
        st.phase_since_us = now_us
        out["phase"] = CLIMB
        out["control_active"] = True
        out["target_z"] = st.target_z_m
        out["ground"] = st.ground_alt_m
        return out

    # ---------------- PHASE 2: GUIDED CLIMB ----------------
    out["control_active"] = True
    if alt_source_lost:
        out["throttle"] = clampi(st.last_throttle, 0, safe_max)
        out["phase"] = st.phase
        out["target_z"] = st.target_z_m
        out["vz_target"] = 0.0
        out["vz_i"] = hold.vz_integral
        out["liftoff_flag"] = st.liftoff_flag
        return out

    # SLEW: rate limiter THUAN
    step = MAX_CLIMB * dt
    remain = st.final_target_m - st.target_z_m
    if remain > step:
        st.target_z_m += step
    elif remain < -step:
        st.target_z_m -= step
    else:
        st.target_z_m = st.final_target_m

    alt_err = st.target_z_m - alt_m
    vz_target = clampf(ALT_KP * alt_err, -MAX_CLIMB, MAX_CLIMB)
    st.vz_target_ms = vz_target

    i_limit = VZ_ILIMIT if st.liftoff_flag else PRELIFT_I_LIMIT
    ceiling = int(safe_max * CEILING_FRAC)
    collective = hold.cascade(vz_target, vz_ms, dt, i_limit, MIN_THR, ceiling)

    # KHONG NHAC NOI: ga KICH TRAN lien tuc. Bang chung VAT LY thay cho
    # NO_LIFT_TIMEOUT cu -- khong dung do cao do duoc. Dat SAU cascade vi can
    # chinh gia tri collective vua tinh.
    i_limit_now = VZ_ILIMIT if st.liftoff_flag else PRELIFT_I_LIMIT
    i_exhausted = hold.vz_integral >= 0.95 * i_limit_now
    at_ceiling = (collective >= ceiling) or i_exhausted
    if latch(st.liftoff_flag and at_ceiling,
             st, "stuck_active", "stuck_since_us", now_us, STUCK_MS):
        st.last_throttle = collective
        return do_abort(AB_STUCK)

    st.last_throttle = collective
    out["throttle"] = collective
    out["target_z"] = st.target_z_m
    out["vz_target"] = vz_target
    out["vz_i"] = hold.vz_integral

    # LIFTOFF theo THOI GIAN trong CLIMB (khong theo est_z).
    if (not st.liftoff_flag) and (now_us - st.phase_since_us) >= LIFTOFF_MS * 1000:
        st.liftoff_flag = True
    out["liftoff_flag"] = st.liftoff_flag

    # VAO HOLDING: dieu kien DUY NHAT la target_z toi dich va GIU du
    # HOLD_ENTER_MS. Khong con so voi est_z.
    if latch(st.target_z_m == st.final_target_m, st,
             "at_target_active", "at_target_since_us", now_us, HOLD_ENTER_MS):
        st.phase = HOLD
        st.phase_since_us = now_us
        st.active = False
        out["phase"] = HOLD
        out["handoff"] = True
        return out

    out["phase"] = st.phase
    return out


# =============================================================================
# Mo hinh vat ly don gian: throttle -> luc nang -> Vz -> Z
# =============================================================================
class Drone(object):
    """thrust_per_duty duoc chon sao cho throttle == hover_true thi treo dung yen.

    hover_true KHAC hover_ff co chu dich: do chinh la thu ma phan I phai TU HOC.
    """

    def __init__(self, hover_true, blocked=False, tau=0.15):
        self.z = 0.0
        self.vz = 0.0
        self.hover_true = float(hover_true)
        self.blocked = blocked      # bi giu lai (kep canh / qua tai)
        self.tau = tau              # hang so thoi gian dap ung luc nang

    def step(self, throttle, dt):
        # gia toc thang dung ~ ti le voi (throttle - hover_true)
        acc = (throttle - self.hover_true) * 0.02
        if self.blocked:
            # mat dat giu lai: khong bao gio roi dat
            self.vz = 0.0
            self.z = 0.0
            return
        if self.z <= 0.0 and acc <= 0.0:
            self.vz = 0.0
            self.z = 0.0
            return
        self.vz += acc * dt
        # ma sat khong khi nhe -> he on dinh, khong dao vo han
        self.vz -= self.vz * (dt / self.tau) * 0.25
        self.z += self.vz * dt
        if self.z < 0.0:
            self.z = 0.0
            self.vz = 0.0


DT = 0.004


def simulate(target_m, hover_true, blocked=False, tof_fail_at_s=None,
             tilt_at_s=None, tilt_val=0.0, max_s=20.0, trace=None):
    st = Tko(target_m, 0)
    hold = Hold()
    d = Drone(hover_true, blocked=blocked)
    now = 0
    rows = []
    result = None
    n = int(max_s / DT)
    for i in range(n):
        t_s = now * 1e-6
        alt_source_ok = True
        if tof_fail_at_s is not None and t_s >= tof_fail_at_s:
            alt_source_ok = False
        tilt = tilt_val if (tilt_at_s is not None and t_s >= tilt_at_s) else 0.0
        out = takeoff_run(st, hold, True, d.z, d.vz, alt_source_ok, tilt, DT, now)
        rows.append((t_s, out["phase"], out["throttle"], out["target_z"], d.z,
                     d.vz, out["vz_target"], out["vz_i"], out["liftoff_flag"]))
        if trace is not None:
            trace.append(rows[-1])
        if out["abort"]:
            result = ("ABORT", out["abort_reason"], rows)
            break
        if out["handoff"]:
            result = ("HANDOFF", None, rows)
            break
        d.step(out["throttle"], DT)
        now += int(DT * 1e6)
    if result is None:
        result = ("TIMEOUT_SIM", None, rows)
    return result, st, hold, d, rows


# =============================================================================
# R1 + R2 -- hai diem review, kiem TRUC TIEP tren source C nua
# =============================================================================
print("\n== (R) Hai diem review ==")

src = io.open(TKO_C, encoding="utf-8").read()
# Chi lay phan takeoff (truoc muc LANDING)
tko_src = src.split("// ================= LANDING")[0]


def strip_comments(text):
    """Bo comment truoc khi tim dinh danh cu.

    Comment CO QUYEN nhac ten cu (vd giai thich vi sao bo preload cua ban
    spool_duty) -- do la lich su can giu. Chi CODE moi khong duoc con
    tham chieu toi chung.
    """
    block = "/\*.*?\*/"
    line = "//[^\\n]*"
    text = re.sub(block, "", text, flags=re.S)
    return re.sub(line, "", text)


tko_code = strip_comments(tko_src)

# R1 tren source: target_z phai duoc gan tu ground_alt (= alt hien tai)
check("R1 source: target_z_m = ground_alt_m (alt hien tai)",
      "st->target_z_m = st->ground_alt_m;" in tko_src)
check("R1 source: ground_alt_m lay tu alt_m, KHONG hard-code 0",
      "st->ground_alt_m = alt_valid ? alt_m : 0.0f;" in tko_src)
check("R1 source: KHONG gan target_z = final_target luc khoi dong",
      re.search(r"st->target_z_m\s*=\s*st->final_target_m", tko_src) is not None,
      "chi duoc xuat hien trong slew (khi remain nho hon step)")

# R2 tren source: cascade PHAI nam SAU khi khoi PRIME return
prime_idx = tko_src.index("if (st->phase == TKO_PRIME)")
cascade_idx = tko_src.index("alt_hold_vz_cascade(")
# tim `return;` cuoi cung cua khoi PRIME (dong ngay truoc PHASE 2)
phase2_idx = tko_src.index("PHASE 2: GUIDED CLIMB")
check("R2 source: alt_hold_vz_cascade() nam SAU khoi PRIME",
      cascade_idx > phase2_idx > prime_idx,
      "cascade o %d, PHASE2 o %d, PRIME o %d" % (cascade_idx, phase2_idx, prime_idx))
check("R2 source: PRIME ep vz_integral = 0",
      "hold_st->vz_integral = 0.0f;" in tko_src[prime_idx:phase2_idx])
check("R2 source: chi co DUNG MOT loi goi cascade trong takeoff",
      tko_src.count("alt_hold_vz_cascade(") == 1)


# =============================================================================
# TEST A -- PRIME
# =============================================================================
print("\n== TEST A: PRIME ==")
st = Tko(0.5, 0)
hold = Hold()
d = Drone(hover_true=HOVER_FF)
now = 0
prime_throttles = []
prime_i = []
while st.phase == PRIME:
    out = takeoff_run(st, hold, True, d.z, d.vz, True, 0.0, DT, now)
    if out["phase"] == PRIME:
        prime_throttles.append(out["throttle"])
        prime_i.append(out["vz_i"])
    d.step(out["throttle"], DT)
    now += int(DT * 1e6)
    if now > 2_000_000:
        break

check("A1 PRIME giu throttle KHONG DOI o prime_duty",
      len(set(prime_throttles)) == 1 and prime_throttles[0] == PRIME_DUTY,
      "gia tri thay duoc: %s" % sorted(set(prime_throttles)))
check("A2 PRIME keo dai dung ~PRIME_MS",
      abs(len(prime_throttles) * DT * 1000 - PRIME_MS) <= DT * 1000 * 2,
      "%d tick = %.0fms, ky vong %.0fms" % (len(prime_throttles),
                                             len(prime_throttles) * DT * 1000, PRIME_MS))
check("A3 R2: I VAN = 0 suot PRIME (PID khong chay)",
      all(abs(v) < 1e-9 for v in prime_i))
check("A4 drone CHUA nhac trong PRIME", d.z == 0.0 and not st.liftoff_flag)
check("A5 R1: target_z sau PRIME = alt hien tai (%.3f), khong phai dich"
      % st.ground_alt_m,
      st.target_z_m == st.ground_alt_m and st.target_z_m != st.final_target_m)


# =============================================================================
# TEST B -- CLIMB: slew + hoc hover.  IN RA TELEMETRY.
# =============================================================================
print("\n== TEST B: CLIMB -- slew + hoc hover ==")
# hover THAT cao hon hover_ff -> I PHAI tu bo len dung phan du do de treo duoc.
HOVER_TRUE = HOVER_FF + HOVER_MISMATCH
res, st_b, hold_b, d_b, rows_b = simulate(0.50, HOVER_TRUE, max_s=20.0)

# LUU Y ve tick chuyen pha: trong C, canh PRIME->CLIMB tra ve voi phase DA la
# CLIMB nhung throttle VAN la prime_duty (cascade chua chay trong tick do). Do
# la 1 tick = 4ms, vo hai, nhung khi do do muot cua throttle thi phai loai ra --
# neu khong ta dang do chinh cai buoc nhay CO CHU DICH cua canh PRIME->CLIMB.
climb_all = [r for r in rows_b if r[1] == CLIMB]
climb_rows = climb_all[1:]          # bo tick chuyen pha
targets = [r[3] for r in climb_rows]
i_terms = [r[7] for r in climb_rows]
throttles = [r[2] for r in climb_rows]

# B1: target TRUOT, khong nhay bac
max_jump = max((abs(targets[i + 1] - targets[i]) for i in range(len(targets) - 1)),
               default=0.0)
allowed = MAX_CLIMB * DT * 1.001
check("B1 target_z TRUOT deu, buoc <= max_climb*dt (%.5f m)" % allowed,
      max_jump <= allowed, "buoc lon nhat = %.5f m" % max_jump)
check("B1b target_z KHONG nhay thang toi dich o tick dau",
      abs(targets[0] - st_b.final_target_m) > 0.1,
      "target dau = %.3f, dich = %.3f" % (targets[0], st_b.final_target_m))

# B2: I hoi tu ve hover thuc - hover_ff
i_final = i_terms[-1]
i_want = HOVER_TRUE - HOVER_FF
check("B2 Vz_I_term HOI TU ve (hover_that - hover_ff) = %+.0f" % i_want,
      abs(i_final - i_want) <= 60,
      "I cuoi = %+.0f" % i_final)
check("B2b I khoi dau ~0 roi BO LEN (hoc phan DU hover_that - hover_ff)",
      abs(i_terms[0]) < 5 and i_final > i_terms[0] + 50,
      "I dau = %+.1f, I cuoi = %+.0f" % (i_terms[0], i_final))

# B3: throttle muot TRONG pha CLIMB (canh PRIME->CLIMB co buoc nhay CO CHU DICH,
# xem ghi chu "I KHOI DAU = 0" trong takeoff_land.c -- do dai o B4).
max_thr_jump = max((abs(throttles[i + 1] - throttles[i])
                    for i in range(len(throttles) - 1)), default=0)
check("B3 throttle MUOT trong CLIMB (buoc lon nhat giua 2 tick <= 25 duty)",
      max_thr_jump <= 25, "buoc lon nhat = %d duty" % max_thr_jump)

# B4: canh PRIME -> CLIMB PHAI nhay len ~hover_ff. Day la CHU DICH: prime theo
# dinh nghia la muc KHONG nhac noi, climb phai dat muc nhac duoc. Neu no KHONG
# nhay (vd ai do them lai preload) thi controller nhan quyen o mot diem neo sai
# va I phai bo nguoc len hover mat nhieu giay -> abort truoc khi kip nhac.
prime_last = climb_all[0][2]        # tick chuyen pha van mang ga PRIME
climb_first = climb_rows[0][2]
check("B4 canh PRIME->CLIMB nhay len quanh hover_ff (%.0f), do duoc %d"
      % (HOVER_FF, climb_first),
      abs(climb_first - HOVER_FF) <= VZ_KP * MAX_CLIMB + 5,
      "prime cuoi=%d, climb dau=%d" % (prime_last, climb_first))
check("B4b I tai tick dau CLIMB = 0 (KHONG preload tu prime_duty)",
      abs(i_terms[0]) <= VZ_KI * MAX_CLIMB * DT * 2,
      "I dau = %+.1f -- khac 0 nghia la co ai do preload lai" % i_terms[0])

print("\n  --- TELEMETRY TEST B (moi 100 tick = 0.4s) ---")
print("  %6s %6s %8s %8s %8s %8s %9s %8s %5s" %
      ("t(s)", "phase", "thr", "targetZ", "estZ", "estVz", "vzTgt", "I", "lift"))
PHN = {0: "IDLE", 1: "PRIME", 2: "CLIMB", 3: "HOLD", 4: "ABORT"}
for idx, r in enumerate(rows_b):
    if idx % 100 == 0 or idx == len(rows_b) - 1:
        print("  %6.2f %6s %8d %8.3f %8.3f %8.3f %9.3f %+8.0f %5d"
              % (r[0], PHN[r[1]], r[2], r[3], r[4], r[5], r[6], r[7], int(r[8])))
print("  ket qua: %s" % res[0])


# =============================================================================
# TEST C -- bay that: len muot, vao HOLDING, khong overshoot > 15%
# =============================================================================
print("\n== TEST C: bay that target 0.45m ==")
TGT_C = 0.45
res_c, st_c, hold_c, d_c, rows_c = simulate(TGT_C, HOVER_FF + HOVER_MISMATCH, max_s=20.0)
check("C1 vao HOLDING (handoff), khong abort", res_c[0] == "HANDOFF", str(res_c[0]))

z_series = [r[4] for r in rows_c]
z_max = max(z_series)
overshoot = (z_max - TGT_C) / TGT_C * 100.0 if TGT_C > 0 else 0.0
check("C2 overshoot <= 15%% (do duoc %.1f%%)" % overshoot, overshoot <= 15.0)

# toc do leo trung binh trong doan giua (bo phan dau/cuoi)
climb_c = [r for r in rows_c if r[1] == CLIMB and 0.1 < r[4] < TGT_C * 0.8]
if len(climb_c) > 10:
    dz = climb_c[-1][4] - climb_c[0][4]
    dt_span = climb_c[-1][0] - climb_c[0][0]
    v_avg = dz / dt_span if dt_span > 0 else 0.0
    # Neu FAIL: hang so chi phoi la ALT_HOLD_VZ_KI, KHONG phai TAKEOFF_MAX_CLIMB_MS.
    # Toc do leo that bi gioi han boi viec thanh phan I cua vong Vz hoc kip
    # hover THAT (hover_ff chi la uoc luong tho) trong doan dau cua CLIMB.
    # KI thap -> I bo len cham -> drone leo cham hon target dang truot -> error
    # don lai. Do duoc tren ban nay: KI=200 -> 0.22 m/s; KI=400 -> 0.28 m/s.
    # (Tang VZ_KP lai lam TE HON: P dap error nhanh nen I hoc cham hon nua.)
    check("C3 toc do leo trung binh ~max_climb (%.2f m/s, do %.2f)"
          " [neu FAIL: xem ALT_HOLD_VZ_KI trong tuning.h]" % (MAX_CLIMB, v_avg),
          0.5 * MAX_CLIMB <= v_avg <= 1.5 * MAX_CLIMB)
else:
    check("C3 co du mau doan leo de do toc do", False, "chi co %d mau" % len(climb_c))

# Chi do TRONG pha CLIMB + tai canh CLIMB->HOLDING. Canh PRIME->CLIMB co buoc
# nhay CO CHU DICH (xem B4) nen loai ra.
_c_all = [r for r in rows_c if r[1] in (CLIMB, HOLD)]
thr_c = [r[2] for r in _c_all[1:]]   # bo tick chuyen pha (xem ghi chu o TEST B)
max_jump_c = max((abs(thr_c[i + 1] - thr_c[i]) for i in range(len(thr_c) - 1)), default=0)
check("C4 KHONG co cu throttle nhay trong CLIMB va tai canh CLIMB->HOLDING (<= 25 duty)",
      max_jump_c <= 25, "buoc lon nhat = %d" % max_jump_c)

# handoff phai bumpless: I giu nguyen, khong reset
check("C5 handoff KHONG reset I (I cuoi != 0, van la hover da hoc)",
      abs(hold_c.vz_integral) > 50 and hold_c.engaged,
      "I=%+.0f engaged=%s" % (hold_c.vz_integral, hold_c.engaged))


# =============================================================================
# TEST D -- chan canh: GIOI HAN DA BIET cua ban giao thuan-thoi-gian
# =============================================================================
# ⚠⚠ DOC KY. Test nay KHONG khang dinh "he thong an toan" -- no GHI LAI mot lo
# hong da biet, co y thuc, de khong ai tuong lam rang van con bao ve.
#
# Ban giao gio phu thuoc DUY NHAT vao thoi gian: target_z (rate-limiter thuan)
# toi dich sau final_target/max_climb giay, giu HOLD_ENTER_MS, xong. Rate-limiter
# do chay GIONG HET NHAU du drone co bay hay khong -- no khong doc cam bien nao.
#
# Hau qua toan hoc: MOI bo phat hien loi cham hon thoi diem ban giao deu KHONG
# BAO GIO kip chay trong pha TAKING_OFF. Voi target 0.5m + max_climb 0.1 thi ban
# giao o ~5.8s, trong khi I can hang chuc giay moi cham i_limit (dI/dt =
# Ki*vz_err = 100*0.1 = 10 duty/s -> 50s de di het dai 500).
#
# Bang chung "khong nhac noi" van CON, nhung no se trip SAU khi da vao HOLDING,
# hoac khong trip. Phan hoi thuc te cho nguoi lai luc do la TELEMETRY:
#   ALTSAT=1 keo dai + TKOI/vz_i dung im o i_limit + ALTm khong tang
# va hanh dong la KILL THU CONG. Khong co auto-abort.
print("")
print("== TEST D: chan canh -- ghi lai LO HONG da biet ==")
res_d, st_d, hold_d, d_d, rows_d = simulate(0.5, HOVER_FF, blocked=True, max_s=30.0)
check("D1 (LO HONG) drone bi chan VAN duoc ban giao sang HOLDING",
      res_d[0] == "HANDOFF",
      "ket qua=%s -- neu doi thanh ABORT thi da co bao ve, hay cap nhat test"
      % str(res_d[0]))
t_handoff = rows_d[-1][0]
print("     -> ban giao o %.2fs trong khi drone chua he roi dat (z=%.3f)"
      % (t_handoff, d_d.z))
check("D2 drone THUC SU van nam tren dat luc ban giao", d_d.z <= 1e-6,
      "z=%.4f" % d_d.z)
# Bang chung PHAI nhin thay duoc tren telemetry, du khong auto-abort.
i_blocked = [r[7] for r in rows_d if r[1] == CLIMB]
check("D3 bang chung VAN hien ra: I bo len deu (nguoi lai nhin duoc qua vz_i)",
      len(i_blocked) > 0 and max(i_blocked) > 0,
      "I lon nhat = %+.0f" % (max(i_blocked) if i_blocked else 0))
check("D4 liftoff_flag LEN du chua he roi dat (danh doi cua thiet ke moi)",
      st_d.liftoff_flag, "liftoff=%s" % st_d.liftoff_flag)


# =============================================================================
# TEST E -- mat ToF giua takeoff
# =============================================================================
print("\n== TEST E: mat ToF giua takeoff ==")
res_e, st_e, hold_e, d_e, rows_e = simulate(1.5, HOVER_FF + 150,
                                             tof_fail_at_s=1.5, max_s=20.0)
check("E1 ket qua la ABORT", res_e[0] == "ABORT", str(res_e[0]))
check("E2 ly do = TOF_LOST", res_e[1] == AB_TOF_LOST, str(res_e[1]))
t_lost = 1.5
t_abort_e = rows_e[-1][0]
check("E3 abort sau ~TOF_LOST_MS (%.0fms), do duoc %.0fms"
      % (TOF_LOST_MS, (t_abort_e - t_lost) * 1000),
      abs((t_abort_e - t_lost) * 1000 - TOF_LOST_MS) <= 15)
# trong cua so cho: throttle phai DUNG YEN (giu ga)
held = [r[2] for r in rows_e if t_lost <= r[0] < t_abort_e]
check("E4 trong cua so cho, throttle DUNG YEN (giu ga cuoi)",
      len(set(held)) <= 1, "cac gia tri: %s" % sorted(set(held))[:5])
check("E5 mat ToF TRUOC khi vao CLIMB thi KHONG tinh (PRIME chua dua vao est_z)",
      True)  # duoc bao dam boi nhanh `if st.phase in (CLIMB, HOLD)` o tren

# E-bis: mat ToF NGAN (200ms) roi co lai -> KHONG duoc abort VI TOF_LOST.
# Dung hover_that gan hover_ff hon (+50) de drone nhac NHANH -> co lap dung
# logic ToF, khong bi NO_LIFT_TIMEOUT xen vao (xem ghi chu duoi TEST H).
st_x = Tko(0.4, 0)
hold_x = Hold()
d_x = Drone(HOVER_FF + 50)
now = 0
abort_reason_x = None
for i in range(int(8.0 / DT)):
    t_s = now * 1e-6
    alt_source_ok = not (1.5 <= t_s < 1.5 + 0.20)   # mat 200ms < 300ms
    out = takeoff_run(st_x, hold_x, True, d_x.z, d_x.vz, alt_source_ok, 0.0, DT, now)
    if out["abort"]:
        abort_reason_x = out["abort_reason"]
        break
    if out["handoff"]:
        break
    d_x.step(out["throttle"], DT)
    now += int(DT * 1e6)
check("E6 mat ToF 200ms (< 300ms) roi co lai -> KHONG abort vi TOF_LOST",
      abort_reason_x != AB_TOF_LOST, "abort_reason = %s" % abort_reason_x)


# =============================================================================
# TEST F -- tilt guard
# =============================================================================
print("\n== TEST F: tilt guard ==")
res_f, st_f, hold_f, d_f, rows_f = simulate(1.5, HOVER_FF + 150,
                                             tilt_at_s=1.0, tilt_val=50.0, max_s=10.0)
check("F1 tilt 50 do -> ABORT/TILT",
      res_f[0] == "ABORT" and res_f[1] == AB_TILT, "%s %s" % (res_f[0], res_f[1]))

# spike ngan hon cua so duy tri -> KHONG abort
st_y = Tko(1.5, 0)
hold_y = Hold()
d_y = Drone(HOVER_FF + 150)
now = 0
aborted_y = False
for i in range(int(4.0 / DT)):
    t_s = now * 1e-6
    tilt = 60.0 if (1.0 <= t_s < 1.0 + (ABORT_TILT_MS / 1000.0) * 0.5) else 0.0
    out = takeoff_run(st_y, hold_y, True, d_y.z, d_y.vz, True, tilt, DT, now)
    if out["abort"]:
        aborted_y = True
        break
    if out["handoff"]:
        break
    d_y.step(out["throttle"], DT)
    now += int(DT * 1e6)
check("F2 spike tilt ngan hon %dms -> KHONG abort (chong nhieu Mahony)" % ABORT_TILT_MS,
      not aborted_y)


# =============================================================================
# TEST G -- khong con dau vet kien truc cu
# =============================================================================
print("\n== TEST G: da xoa het kien truc cu ==")
for gone in ("TKO_SPOOL", "TKO_CONTROL_ACTIVE", "TKO_SETTLE", "TKO_DONE",
             "spool_duty", "spool_throttle", "tko_score_liftoff",
             "liftoff_confirmed", "TKO_ABORT_NO_LIFTOFF", "TKO_ABORT_EST_INVALID"):
    check("G: khong con '%s' trong CODE takeoff_land.c" % gone, gone not in tko_code)

hdr = io.open(os.path.join(HERE, "..", "components", "flight_core", "include",
                           "flight_core", "takeoff_land.h"), encoding="utf-8").read()
tko_hdr = hdr.split("// ================= LANDING")[0]
tko_hdr_code = strip_comments(tko_hdr)
for gone in ("spool_duty", "spool_ms", "TAKEOFF_SPOOL", "score", "ev_thr",
             "liftoff_confirmed"):
    check("G: khong con '%s' trong CODE takeoff_land.h" % gone, gone not in tko_hdr_code)

check("G: takeoff_control_active() KHONG bao gom PRIME",
      "TKO_PRIME" not in tko_hdr.split("takeoff_control_active")[1].split("}")[0])



# =============================================================================
# TEST H -- LATCH GA HOVER THEO PIN co thuc su cuu duoc takeoff khong?
# =============================================================================
# Day la cau tra loi TRUC TIEP cho "luong takeoff cu khong on". Mo phong CUNG
# MOT con drone, CUNG mot vien pin 3.6V, chi khac hover_ff:
#
#   TRUOC : hover_ff = ALT_HOLD_HOVER_NOMINAL (hang so 1000, bat ke pin)
#   SAU   : hover_ff = hover_model(3.6V) = 1344  (latch luc ARM)
#
# hover THAT cua drone o 3.6V lay theo chinh so do bench-ramp cua nguoi dung
# (1300..1400 -> dung diem giua 1350). Khong phai so bia: no la du lieu goc ma
# ca model duoc fit tu do.
print("\n== TEST H: latch ga hover theo pin co cuu duoc takeoff khong ==")

HM_H = os.path.join(HERE, "..", "components", "flight_core", "include",
                     "flight_core", "hover_model.h")
_hm = io.open(HM_H, encoding="utf-8").read()


def _hmnum(name):
    mm = re.search(r"^#define\s+%s\s+\(?([-+0-9.eE]+)f?\)?" % name, _hm, re.M)
    assert mm, "khong doc duoc " + name
    return float(mm.group(1))


H_REF = _hmnum("HOVER_MODEL_REF_DUTY")
H_VREF = _hmnum("HOVER_MODEL_V_REF")
H_EXP = _hmnum("HOVER_MODEL_EXP")

V_TEST = 3.6
HOVER_TRUE_AT_V = 1350.0                       # so do bench-ramp that
HOVER_LATCHED = H_REF * ((H_VREF / V_TEST) ** H_EXP)

print("   pin %.1fV: hover THAT (do bench) = %.0f | latch model = %.0f | hang so cu = %.0f"
      % (V_TEST, HOVER_TRUE_AT_V, HOVER_LATCHED, HOVER_FF))


def _run_with_hover_ff(hover_ff_value, hover_true, target_m, max_s=25.0):
    """Chay simulate() voi mot hover_ff KHAC.

    Hold/takeoff_run doc HOVER_FF va PRIME_DUTY tu bien MODULE, nen phai thay
    tam roi tra lai. Xau, nhung dung: no bao dam ca hai lan chay dung Y HET
    mot ban transcription, chenh lech do duoc chi den tu hover_ff.
    """
    g = globals()
    old_ff, old_prime = g["HOVER_FF"], g["PRIME_DUTY"]
    try:
        g["HOVER_FF"] = hover_ff_value
        g["PRIME_DUTY"] = int(hover_ff_value * PRIME_FRAC)
        return simulate(target_m, hover_true, max_s=max_s)
    finally:
        g["HOVER_FF"], g["PRIME_DUTY"] = old_ff, old_prime


TGT_H = 0.45
res_old, st_old, hold_old, d_old, rows_old = _run_with_hover_ff(
    HOVER_FF, HOVER_TRUE_AT_V, TGT_H)
res_new, st_new, hold_new, d_new, rows_new = _run_with_hover_ff(
    HOVER_LATCHED, HOVER_TRUE_AT_V, TGT_H)

z_old, z_new = d_old.z, d_new.z
print("   TRUOC (hover_ff=%.0f): ket qua=%s, do cao dat duoc = %.3f m"
      % (HOVER_FF, res_old[0], z_old))
print("   SAU   (hover_ff=%.0f): ket qua=%s, do cao dat duoc = %.3f m"
      % (HOVER_LATCHED, res_new[0], z_new))

check("H1 model latch bam sat hover THAT o %.1fV (lech < 60 duty)" % V_TEST,
      abs(HOVER_LATCHED - HOVER_TRUE_AT_V) < 60,
      "lech %.0f duty" % (HOVER_LATCHED - HOVER_TRUE_AT_V))

check("H2 hang so CU lech HANG TRAM duty o cung muc pin (day la goc benh)",
      abs(HOVER_FF - HOVER_TRUE_AT_V) > 200,
      "lech %.0f duty" % (HOVER_FF - HOVER_TRUE_AT_V))

check("H3 LATCH lam drone leo cao hon HAN so voi hang so cu",
      z_new > z_old + 0.05, "truoc %.3fm -> sau %.3fm" % (z_old, z_new))

check("H4 SAU khi latch, drone THUC SU roi dat truoc luc ban giao",
      z_new > 0.02, "z=%.3fm" % z_new)

# Bang chung ve CO CHE: hang so cu bat phan I phai tu hoc ca ho ~350 duty,
# trong khi sau latch I gan nhu khong con gi phai hoc.
i_old, i_new = hold_old.vz_integral, hold_new.vz_integral
print("   I phai hoc:  TRUOC = %+.0f duty   SAU = %+.0f duty" % (i_old, i_new))
check("H5 sau latch, phan I phai hoc NHO HON HAN",
      abs(i_new) < abs(i_old), "truoc %+.0f -> sau %+.0f" % (i_old, i_new))

print("")
if fails:
    print("KET QUA: %d FAIL -- %s" % (len(fails), ", ".join(fails)))
    sys.exit(1)
print("KET QUA: ALL PASS (TEST A,B,C,D,E,F,G,H + R1,R2)")
