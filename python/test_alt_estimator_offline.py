"""Test THUAT TOAN alt_estimator (TEST 1..6 trong spec).

PHAM VI: file nay CHEP LAI logic tu components/flight_core/src/alt_estimator.c
sang Python de chay duoc tren may host (khong co host C compiler trong moi
truong nay - chi co cross-toolchain xtensa). No kiem tra THUAT TOAN/THIET KE,
KHONG kiem tra binary da bien dich. Neu sua alt_estimator.c thi phai sua ca
day, neu khong test se kiem tra mot phien ban da loi thoi.

Gia tri kiem tra lay tu alt_estimator.h.
"""
import math
import sys

GRAVITY = 9.81

# --- hang so, khop alt_estimator.h ---
VERT_ACC_LPF_HZ = 10.0
VERT_ACC_DEADBAND = 0.05
BIAS_LEARN_HZ = 0.03
BIAS_MAX = 0.50
BARO_LPF_HZ = 2.0
GATE_M = 0.6
ALPHA = 0.03
BETA = 0.003
TIMEOUT_MS = 500
REACQ_MS = 3000
REACQ_ALPHA = 0.15
REACQ_BETA = 0.01
VZ_CORR_MAX = 0.10

# --- hang so takeoff, khop tuning.h ---
SPOOL_DUTY = 1600.0
SPOOL_MS = 800
HOVER = 1300.0


def lpf_alpha(dt, fc):
    rc = 1.0 / (2.0 * math.pi * fc)
    return dt / (dt + rc)


def clampf(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class Est:
    def __init__(self):
        self.reset()

    def _reset_flight_state(self):
        self.alt_m = 0.0
        self.vz_ms = 0.0
        self.valid = False
        self.vz_accel_only_ms = 0.0
        self.z_inertial_m = 0.0
        self.baro_med = []
        self.baro_lpf_alt_m = 0.0
        self.baro_lpf_init = False
        self.last_processed_baro_seq = 0
        self.baro_seq_init = False
        self.last_baro_timestamp_us = 0
        self.last_baro_received_us = 0
        self.baro_fusion_initialized = False
        self.baro_new_sample = False
        self.baro_innovation_m = 0.0
        self.baro_dt_s = 0.0
        self.baro_accept_count = 0
        self.baro_reject_count = 0
        self.baro_reject_consecutive = 0
        self.baro_disagreement_active = False
        self.baro_disagreement_since_us = 0
        self.baro_disagreement_sign = 0.0
        self.baro_reacquire_active = False

    def reset(self):
        self._reset_flight_state()
        self.accel_bias_ms2 = 0.0
        self.az_raw_ms2 = 0.0
        self.az_lpf_ms2 = 0.0
        self.az_corrected_ms2 = 0.0
        self.az_lpf_init = False
        self.was_inertial_enabled = False

    def reanchor(self):
        self._reset_flight_state()

    def _baro_prefilter(self, alt, dt_baro):
        self.baro_med.append(alt)
        if len(self.baro_med) > 3:
            self.baro_med.pop(0)
        med = alt if len(self.baro_med) < 3 else sorted(self.baro_med)[1]
        if not self.baro_lpf_init:
            self.baro_lpf_alt_m = med
            self.baro_lpf_init = True
        else:
            self.baro_lpf_alt_m += lpf_alpha(dt_baro, BARO_LPF_HZ) * (med - self.baro_lpf_alt_m)
        return self.baro_lpf_alt_m

    def _apply_vz_correction(self, beta, innov, dt_baro):
        self.vz_ms += clampf(beta * innov / dt_baro, -VZ_CORR_MAX, VZ_CORR_MAX)

    def _correct_airborne(self, baro_filtered, dt_baro, now_us):
        innov = baro_filtered - self.alt_m
        self.baro_innovation_m = innov
        if not self.baro_fusion_initialized:
            self.alt_m = baro_filtered
            self.baro_fusion_initialized = True
            self.baro_accept_count += 1
            self.baro_reject_consecutive = 0
            self.baro_disagreement_active = False
            self.baro_reacquire_active = False
            return
        if abs(innov) <= GATE_M:
            self.alt_m += ALPHA * innov
            self._apply_vz_correction(BETA, innov, dt_baro)
            self.baro_accept_count += 1
            self.baro_reject_consecutive = 0
            self.baro_disagreement_active = False
            self.baro_reacquire_active = False
            return
        sign = 1.0 if innov > 0.0 else -1.0
        if not self.baro_disagreement_active or sign != self.baro_disagreement_sign:
            self.baro_disagreement_active = True
            self.baro_disagreement_since_us = now_us
            self.baro_disagreement_sign = sign
        dis_ms = (now_us - self.baro_disagreement_since_us) / 1000
        if dis_ms >= REACQ_MS:
            self.baro_reacquire_active = True
            self.alt_m += REACQ_ALPHA * innov
            self._apply_vz_correction(REACQ_BETA, innov, dt_baro)
            self.baro_accept_count += 1
            self.baro_reject_consecutive = 0
        else:
            self.baro_reject_count += 1
            self.baro_reject_consecutive += 1

    def update(self, acc_body, q, baro_healthy, baro_seq, baro_ts_us, baro_alt,
               stationary, liftoff_candidate, airborne, now_us, dt):
        w, x, y, z = q
        ax, ay, az = acc_body
        az_world_g = (2.0 * (x * z - w * y) * ax +
                      2.0 * (y * z + w * x) * ay +
                      (1.0 - 2.0 * x * x - 2.0 * y * y) * az)
        self.az_raw_ms2 = (az_world_g - 1.0) * GRAVITY

        if not self.az_lpf_init:
            self.az_lpf_ms2 = self.az_raw_ms2
            self.az_lpf_init = True
        else:
            self.az_lpf_ms2 += lpf_alpha(dt, VERT_ACC_LPF_HZ) * (self.az_raw_ms2 - self.az_lpf_ms2)

        # HOC BIAS TU az_lpf (TRUOC deadband) - day la ban sua
        if stationary:
            self.accel_bias_ms2 += lpf_alpha(dt, BIAS_LEARN_HZ) * (self.az_lpf_ms2 - self.accel_bias_ms2)
            self.accel_bias_ms2 = clampf(self.accel_bias_ms2, -BIAS_MAX, BIAS_MAX)

        az = self.az_lpf_ms2 - self.accel_bias_ms2
        if abs(az) < VERT_ACC_DEADBAND:
            az = 0.0
        self.az_corrected_ms2 = az

        inertial_enabled = airborne or liftoff_candidate
        if inertial_enabled and not self.was_inertial_enabled:
            self.baro_fusion_initialized = True
            self.baro_reject_consecutive = 0
            self.baro_disagreement_active = False
            self.baro_reacquire_active = False
        self.was_inertial_enabled = inertial_enabled

        if inertial_enabled:
            vz_old = self.vz_ms
            self.vz_ms += az * dt
            self.alt_m += vz_old * dt + 0.5 * az * dt * dt
            vz_ao_old = self.vz_accel_only_ms
            self.vz_accel_only_ms += az * dt
            self.z_inertial_m += vz_ao_old * dt + 0.5 * az * dt * dt
        else:
            self.alt_m = 0.0
            self.vz_ms = 0.0
            self.z_inertial_m = 0.0
            self.vz_accel_only_ms = 0.0

        new_sample = (baro_healthy and
                      (not self.baro_seq_init or baro_seq != self.last_processed_baro_seq) and
                      baro_seq != 0)
        self.baro_new_sample = new_sample
        if new_sample:
            self.last_processed_baro_seq = baro_seq
            self.baro_seq_init = True
            self.last_baro_received_us = now_us
            if self.last_baro_timestamp_us == 0:
                dt_baro = 1.0 / 50.0
            else:
                dt_baro = clampf((baro_ts_us - self.last_baro_timestamp_us) / 1e6, 0.005, 0.5)
            self.last_baro_timestamp_us = baro_ts_us
            self.baro_dt_s = dt_baro
            bf = self._baro_prefilter(baro_alt, dt_baro)
            if airborne:
                self._correct_airborne(bf, dt_baro, now_us)
            else:
                self.baro_innovation_m = bf - self.alt_m

        self.valid = (self.last_baro_received_us != 0 and
                      (now_us - self.last_baro_received_us) <= TIMEOUT_MS * 1000)


DT = 1.0 / 250.0
IDENT_Q = (1.0, 0.0, 0.0, 0.0)


def acc_for_az(az_ms2):
    """accel body-frame (g) de az_world = az_ms2, quaternion don vi."""
    return (0.0, 0.0, 1.0 + az_ms2 / GRAVITY)


def run(est, secs, az_ms2, stationary, cand, airborne, baro_alt=0.0,
        baro_hz=50.0, t0_us=0, baro_seq0=0):
    """Chay est trong `secs` giay. Tra seq cuoi.

    Ghi lai est.reacq_seen = da TUNG vao REACQUIRE trong luc chay hay chua —
    kiem tra co reacquire_active o cuoi doan chay la SAI: reacquire lam xong
    viec (keo Z ve trong gate) roi TU THOAT, nen no thuong da tat truoc khi
    doan chay ket thuc. Dieu can kiem tra la "co xay ra" + "Z da hoi tu".
    """
    n = int(secs / DT)
    baro_every = int(round((1.0 / baro_hz) / DT))
    seq = baro_seq0
    ts = t0_us
    if not hasattr(est, "reacq_seen"):
        est.reacq_seen = False
    for i in range(n):
        now = t0_us + int(i * DT * 1e6)
        if i % baro_every == 0:
            seq += 1
            ts = now
        est.update(acc_for_az(az_ms2), IDENT_Q, True, seq, ts, baro_alt,
                   stationary, cand, airborne, now, DT)
        if est.baro_reacquire_active:
            est.reacq_seen = True
    return seq


fails = []


def check(name, cond, detail=""):
    if not cond:
        fails.append(f"  {name}: {detail}")


# ============ TEST 1 - GROUND 60s ============
# UAV nam yen, co bias gia tao -0.03 m/s2 (giong log that: VACC~-0.03).
e = Est()
run(e, 60.0, az_ms2=-0.03, stationary=True, cand=False, airborne=False)
check("TEST1 Z_est=0", e.alt_m == 0.0, f"alt_m={e.alt_m}")
check("TEST1 Vz_est=0", e.vz_ms == 0.0, f"vz_ms={e.vz_ms}")
check("TEST1 Z_inertial=0", e.z_inertial_m == 0.0, f"z_inertial={e.z_inertial_m}")
check("TEST1 Vz_accel_only=0", e.vz_accel_only_ms == 0.0, f"vzao={e.vz_accel_only_ms}")
# Bias PHAI hoi tu ve ~-0.03 (day la bug cu: deadband 0.05 > |−0.03| nen bias
# cu hoc mai ve 0 va khong bao gio bu duoc gi).
check("TEST1 bias hoi tu ~ -0.03", abs(e.accel_bias_ms2 - (-0.03)) < 0.005,
      f"bias={e.accel_bias_ms2:.5f} (ky vong ~-0.030)")
# Sau khi bias hoi tu, az_corrected phai ~0
check("TEST1 az_corrected ~ 0", abs(e.az_corrected_ms2) < 1e-6,
      f"az_corrected={e.az_corrected_ms2}")

# ============ TEST 1b - REGRESSION: bug thu tu deadband/bias cu ============
# Mo phong THU TU CU (deadband TRUOC khi hoc bias) de chung minh no hong.
class EstOldOrder(Est):
    def update(self, acc_body, q, *a, **kw):
        # chi can chung minh: neu deadband truoc, bias khong bao gio hoc duoc
        pass


e_old_bias = 0.0
_lpf = -0.03  # gia su LPF da on dinh o -0.03
for _ in range(int(60.0 / DT)):
    az_db = _lpf if abs(_lpf) >= VERT_ACC_DEADBAND else 0.0   # deadband TRUOC (bug cu)
    e_old_bias += lpf_alpha(DT, BIAS_LEARN_HZ) * (az_db - e_old_bias)
check("TEST1b thu tu CU that su hong", abs(e_old_bias) < 1e-3,
      f"bias(thu tu cu)={e_old_bias:.6f} - dung nhu trieu chung ABIAS~-0.003 tren log")

# ============ TEST 5 - BARO 1 CORRECTION / SAMPLE, BDT dung ============
e = Est()
# airborne, baro 50Hz, estimator 250Hz -> 1 mau baro xuat hien 5 tick lien tiep
seq_end = run(e, 2.0, az_ms2=0.0, stationary=False, cand=False, airborne=True,
              baro_alt=0.0, baro_hz=50.0)
total_corr = e.baro_accept_count + e.baro_reject_count
check("TEST5 so correction == so mau baro", total_corr == seq_end,
      f"correction={total_corr} mau_baro={seq_end} (neu =5x thi dang xu ly lai mau cu)")
check("TEST5 BDT ~ 0.020s", abs(e.baro_dt_s - 0.020) < 1e-6,
      f"baro_dt_s={e.baro_dt_s} (phai la nhip baro that, KHONG phai 0.004)")

# ============ TEST 6 - SYNTHETIC +1 m/s2, candidate=1 ============
e = Est()
run(e, 1.0, az_ms2=1.0, stationary=False, cand=True, airborne=False)
check("TEST6 Vz ~ +1.0 m/s", 0.95 <= e.vz_ms <= 1.005, f"vz={e.vz_ms:.4f}")
check("TEST6 Z ~ +0.5 m", 0.45 <= e.alt_m <= 0.505, f"z={e.alt_m:.4f}")
check("TEST6 Z KHONG con bi khoa 0", e.alt_m > 0.0, f"z={e.alt_m}")

# ============ TEST 2/3 - CANDIDATE PHAI TAO RA DO CAO DE BAN GIAO ============
# Chuoi cat canh moi: ram ga -> het spool -> CHOT do cao dang co lam target.
# Neu ground lock van khoa Z=0 trong suot pha TAKING_OFF thi target chot duoc
# se LUON = 0 -> alt_hold nhan target 0 va khong giu duoc gi. Day la ly do pha
# CANDIDATE van phai ton tai du bo do roi dat da bi bo.
e_nocand = Est()
run(e_nocand, 0.8, az_ms2=1.0, stationary=False, cand=False, airborne=False)
check("TEST2 khong candidate -> khong co do cao de chot", e_nocand.alt_m == 0.0,
      f"alt={e_nocand.alt_m}")

e_cand = Est()
run(e_cand, 0.8, az_ms2=1.0, stationary=False, cand=True, airborne=False)
check("TEST2 co candidate -> co do cao THAT de chot", e_cand.alt_m > 0.05,
      f"alt={e_cand.alt_m:.4f}")

# TEST 3 - CONTINUITY khi candidate -> airborne (Policy B)
# Timestamp baro phai TIEP NOI chuoi truoc (mono tang) - truyen 0 se tao dt am
# bi clamp ve 0.005 va lam correction manh gap 4 lan, khong phan anh thuc te.
vz_before, z_before = e_cand.vz_ms, e_cand.alt_m
_ts_next = e_cand.last_baro_timestamp_us + 20_000     # +20ms = 1 chu ky baro 50Hz
e_cand.update(acc_for_az(1.0), IDENT_Q, True, 999, _ts_next, 0.0,
              False, False, True, int(1.0 * 1e6), DT)   # tick dau tien airborne
check("TEST3 Vz GIU continuity (khong reset 0)", e_cand.vz_ms > vz_before * 0.9,
      f"vz truoc={vz_before:.4f} sau={e_cand.vz_ms:.4f}")
check("TEST3 Z GIU continuity (khong reset 0)", e_cand.alt_m > z_before * 0.9,
      f"z truoc={z_before:.4f} sau={e_cand.alt_m:.4f}")

# ============ TEST 4 - CANDIDATE TIMEOUT -> ve GROUND, reset 0 ============
e = Est()
run(e, 1.0, az_ms2=1.0, stationary=False, cand=True, airborne=False)
check("TEST4 candidate da tich phan", e.alt_m > 0.1, f"z={e.alt_m}")
# takeoff abort -> FSM roi TAKING_OFF -> candidate=0
e.update(acc_for_az(0.0), IDENT_Q, True, 500, 0, 0.0, False, False, False, 2_000_000, DT)
check("TEST4 Z ve 0", e.alt_m == 0.0, f"z={e.alt_m}")
check("TEST4 Vz ve 0", e.vz_ms == 0.0, f"vz={e.vz_ms}")
check("TEST4 Z_inertial ve 0", e.z_inertial_m == 0.0, f"zine={e.z_inertial_m}")

# ============ TEST 7 - REACQUIRE pha deadlock innovation gate ============
# Airborne, inertial drift manh (bias gia) lam Z chay xa baro (baro on dinh 0).
e = Est()
e.was_inertial_enabled = True
e.baro_fusion_initialized = True
e.alt_m = 5.0     # da troi xa 5m so voi baro
e.vz_ms = 0.0
run(e, 1.0, az_ms2=0.0, stationary=False, cand=False, airborne=True, baro_alt=0.0)
check("TEST7 giai doan dau: reject (spike-like)", e.baro_reject_count > 0,
      f"reject={e.baro_reject_count}")
check("TEST7 chua reacquire trong 1s dau (chua du 3000ms bat dong)",
      not e.reacq_seen, "")
_rej_1s = e.baro_reject_count
# chay tiep qua nguong 3s
run(e, 3.0, az_ms2=0.0, stationary=False, cand=False, airborne=True, baro_alt=0.0,
    t0_us=1_000_000, baro_seq0=e.last_processed_baro_seq)
check("TEST7 da TUNG vao REACQUIRE", e.reacq_seen, "")
check("TEST7 Z da duoc keo ve gan baro", abs(e.alt_m) < 1.0,
      f"z={e.alt_m:.3f} (bat dau 5.0, baro=0)")
# Day la muc tieu chinh cua ca co che: KHONG duoc reject mai mai.
check("TEST7 BREJ da NGUNG tang (het deadlock)",
      e.baro_reject_count < _rej_1s + 200,
      f"reject 1s dau={_rej_1s} sau 4s={e.baro_reject_count}")
check("TEST7 BACC da tang tro lai", e.baro_accept_count > 0,
      f"accept={e.baro_accept_count}")

# ============ TEST 8 - BARO KHONG DUOC GIAT Vz (tran correction) ============
# Innovation RAT lon (5m) trong REACQUIRE. Khong co tran thi
# REACQ_BETA*5/0.02 = 2.5 m/s bi nhoi vao Vz chi tu MOT mau baro.
e = Est()
e.was_inertial_enabled = True
e.baro_fusion_initialized = True
e.baro_disagreement_active = True
e.baro_disagreement_since_us = 0
e.baro_disagreement_sign = -1.0
e.alt_m = 5.0
e.vz_ms = 0.0
e.last_baro_timestamp_us = 1_000_000
e.baro_lpf_init = True
e.baro_lpf_alt_m = 0.0
e.baro_med = [0.0, 0.0, 0.0]
e.last_processed_baro_seq = 1
e.baro_seq_init = True
vz0 = e.vz_ms
e.update(acc_for_az(0.0), IDENT_Q, True, 2, 1_020_000, 0.0,
         False, False, True, 4_000_000, DT)   # dis_ms = 4000 >= 3000 -> REACQUIRE
check("TEST8 da REACQUIRE", e.baro_reacquire_active, "")
check("TEST8 Vz bi chan tran (khong giat 2.5m/s)", abs(e.vz_ms - vz0) <= VZ_CORR_MAX + 1e-9,
      f"dVz={e.vz_ms - vz0:.4f} (tran={VZ_CORR_MAX})")
check("TEST8 Z VAN duoc keo ve manh", e.alt_m < 4.3,
      f"z={e.alt_m:.3f} (tu 5.0, nhanh alpha KHONG bi chan)")

if fails:
    print("FAIL:")
    print("\n".join(fails))
    sys.exit(1)
print("PASS: tat ca TEST 1,1b,2,3,4,5,6,7,8 (thuat toan alt_estimator)")
