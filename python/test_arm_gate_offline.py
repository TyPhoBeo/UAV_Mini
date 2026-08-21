"""Test cong ARM — TRANSCRIPTION cua prearm_check() + case CMD_ARM
(components/flight_core/src/flight_core.c) sang Python.

⚠ GIOI HAN: khong phai test tren binary da bien dich (may nay khong co host C
compiler). Day la ban chep lai LOGIC CONG.

MUC DICH CHINH: khoa lai bug "ARM bi tu choi IM LANG".
Truoc ban nay, nhanh `guard PASS nhung FSM khong o DISARMED` KHONG log gi va
KHONG bao gi ra ngoai -> nguoi dung bam ARM, console reply "ARMED" (reply do
chi doan theo attitude guard), roi khong co gi xay ra va khong co cach nao biet
vi sao. Test 3 la test quan trong nhat: MOI duong tu choi PHAI tra ve mot ly do
khac ARM_REJECT_NONE.
"""
import sys

# ---- arm_reject_t — chep tu telemetry.h, thu tu PHAI khop ----
(NONE, UNCALIBRATED, STATE, ATTITUDE_INVALID, TILT, IMU, GYRO_CALIB,
 ACCEL_CALIB, BATTERY_SAMPLE, BATTERY_LOW, BARO_NOT_READY, BARO_UNHEALTHY,
 ALT_EST_INVALID, LOOP_UNHEALTHY, BUS_LEASE, BARO_CALIB_FAILED) = range(16)

FSM_DISARMED, FSM_ARMED, FSM_BENCH_RAMP, FSM_TAKING_OFF, \
    FSM_HOLDING, FSM_FLYING, FSM_LANDING, FSM_EMERGENCY = range(8)

FSM_NAMES = {
    FSM_DISARMED: "DISARMED", FSM_ARMED: "ARMED", FSM_BENCH_RAMP: "BENCH_RAMP",
    FSM_TAKING_OFF: "TAKING_OFF", FSM_HOLDING: "HOLDING", FSM_FLYING: "FLYING",
    FSM_LANDING: "LANDING", FSM_EMERGENCY: "EMERGENCY",
}

FSM_ARM_MAX_TILT_DEG = 10.0
BATTERY_FLOOR_V = 3.3
DEADLINE_MISS_HARD = 50


class Prearm:
    """s_prearm — mac dinh la trang thai LANH MANH (moi thu OK)."""

    def __init__(self, **kw):
        self.attitude_valid = True
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.imu_fresh = True
        self.imu_healthy = True
        self.gyro_calibrated = True
        self.accel_calibrated = True
        self.battery_sample_ok = True
        self.battery_v = 3.9
        self.baro_ready = True
        self.baro_healthy = True
        self.alt_estimator_valid = True
        self.deadline_miss_streak = 0
        for k, v in kw.items():
            assert hasattr(self, k), f"field khong ton tai: {k}"
            setattr(self, k, v)

    @property
    def loop_healthy(self):
        return self.deadline_miss_streak < DEADLINE_MISS_HARD


def prearm_check(p, battery_ok_driver=True, baro_ok_driver=True):
    """Tra (ok, reject_dau_tien, [tat_ca_log]). Chep dung THU TU kiem cua C."""
    ok = True
    reject = NONE
    logs = []

    def rec(r, msg):
        nonlocal ok, reject
        logs.append(msg)
        if reject == NONE:
            reject = r
        ok = False

    if not p.attitude_valid:
        rec(ATTITUDE_INVALID, "attitude CHUA hop le")
    else:
        if abs(p.roll_deg) >= FSM_ARM_MAX_TILT_DEG or abs(p.pitch_deg) >= FSM_ARM_MAX_TILT_DEG:
            rec(TILT, "nghieng vuot nguong")

    if not p.imu_fresh or not p.imu_healthy:
        rec(IMU, "IMU stale/khong khoe")
    if not p.gyro_calibrated:
        rec(GYRO_CALIB, "gyro chua calib")
    if not p.accel_calibrated:
        rec(ACCEL_CALIB, "accel chua calib")

    if battery_ok_driver:
        if not p.battery_sample_ok or p.battery_v <= 1.0:
            rec(BATTERY_SAMPLE, "khong doc duoc pin")
        elif p.battery_v < BATTERY_FLOOR_V:
            rec(BATTERY_LOW, "pin duoi san")

    if baro_ok_driver:
        if not p.baro_ready:
            rec(BARO_NOT_READY, "baro chua co moc 0m")
        elif not p.baro_healthy:
            rec(BARO_UNHEALTHY, "baro unhealthy")
        if not p.alt_estimator_valid:
            rec(ALT_EST_INVALID, "alt_estimator chua hop le")

    if not p.loop_healthy:
        rec(LOOP_UNHEALTHY, "vong dieu khien tre han")

    return ok, reject, logs


def fsm_on_arm_request(cur, guard_ok):
    return FSM_ARMED if (cur == FSM_DISARMED and guard_ok) else cur


def recompute_uncalibrated(accel_valid, mag_valid, mag_enabled):
    """Chep recompute_uncalibrated() (flight_core.c).

    BAT BUOC de bay: CHI accel (+ gyro qua prearm_check). MAG KHONG BAT BUOC —
    truoc day dieu kien la `!accel_valid || (mag_enabled && !mag_valid)`.
    """
    return not accel_valid


def cmd_arm(state, p, uncalibrated=False, baro_ok_driver=True,
            battery_ok_driver=True, bus_lease_ok=True, baro_calib_ok=True):
    """Chep case CMD_ARM. Tra (armed, reject, logs)."""
    reject = NONE          # xoa dau moi lan thu
    logs = []

    if uncalibrated:
        logs.append("CHUA CALIBRATE")
        return False, UNCALIBRATED, logs

    ok, pre_reject, pre_logs = prearm_check(p, battery_ok_driver, baro_ok_driver)
    logs += pre_logs
    reject = pre_reject
    guard = ok

    nxt = fsm_on_arm_request(state, guard)
    if nxt != state:
        if baro_ok_driver:
            if not bus_lease_ok:
                logs.append("khong muon duoc bus I2C")
                return False, BUS_LEASE, logs
            if not baro_calib_ok:
                logs.append("calib baro that bai")
                return False, BARO_CALIB_FAILED, logs
        logs.append("ARM OK -> FSM_ARMED")
        return True, NONE, logs
    elif not guard:
        logs.append("prearm_check KHONG pass")
        return False, reject, logs
    else:
        # NHANH TRUOC DAY IM LANG HOAN TOAN
        logs.append(f"FSM dang o {FSM_NAMES[state]}, chi ARM duoc tu DISARMED")
        return False, STATE, logs


FAILS = []


def check(name, cond, extra=""):
    print(("  OK   " if cond else "  FAIL ") + name + (f"   {extra}" if extra else ""))
    if not cond:
        FAILS.append(name)


# ============================================================================
print("TEST 1: trang thai lanh manh + DISARMED -> ARM THANH CONG")
armed, rej, logs = cmd_arm(FSM_DISARMED, Prearm())
check("armed = True", armed)
check("reject = NONE", rej == NONE, f"={rej}")

# ============================================================================
print("\nTEST 2: tung dieu kien prearm sai -> dung ma reject, KHONG armed")
cases = [
    ("attitude invalid", dict(attitude_valid=False), ATTITUDE_INVALID),
    ("nghieng 15 deg",   dict(roll_deg=15.0),        TILT),
    ("IMU stale",        dict(imu_fresh=False),      IMU),
    ("gyro chua calib",  dict(gyro_calibrated=False), GYRO_CALIB),
    ("accel chua calib", dict(accel_calibrated=False), ACCEL_CALIB),
    ("khong doc duoc pin", dict(battery_sample_ok=False), BATTERY_SAMPLE),
    ("pin 3.0V duoi san", dict(battery_v=3.0),       BATTERY_LOW),
    ("baro chua co moc",  dict(baro_ready=False),    BARO_NOT_READY),
    ("baro unhealthy",    dict(baro_healthy=False),  BARO_UNHEALTHY),
    ("alt_est invalid",   dict(alt_estimator_valid=False), ALT_EST_INVALID),
    ("loop tre han",      dict(deadline_miss_streak=60), LOOP_UNHEALTHY),
]
for name, kw, want in cases:
    armed, rej, logs = cmd_arm(FSM_DISARMED, Prearm(**kw))
    check(f"{name} -> reject dung", (not armed) and rej == want,
          f"armed={armed} rej={rej} want={want}")

# ============================================================================
print("\nTEST 3 (QUAN TRONG NHAT): MOI duong tu choi PHAI co ly do != NONE")
print("  -> khoa lai bug 'ARM bi tu choi IM LANG'")

silent = []

# 3a) moi FSM state KHAC DISARMED, voi prearm HOAN TOAN LANH MANH.
#     Day chinh la nhanh truoc day khong log gi ca.
for st in (FSM_ARMED, FSM_BENCH_RAMP, FSM_TAKING_OFF, FSM_HOLDING,
           FSM_FLYING, FSM_LANDING, FSM_EMERGENCY):
    armed, rej, logs = cmd_arm(st, Prearm())
    ok = (not armed) and rej == STATE and len(logs) > 0
    check(f"state={FSM_NAMES[st]:11s} -> reject=STATE + co log", ok,
          f"rej={rej} logs={len(logs)}")
    if not armed and rej == NONE:
        silent.append(FSM_NAMES[st])

# 3b) uncalibrated
armed, rej, logs = cmd_arm(FSM_DISARMED, Prearm(), uncalibrated=True)
check("uncalibrated -> reject=UNCALIBRATED", (not armed) and rej == UNCALIBRATED)
if not armed and rej == NONE:
    silent.append("uncalibrated")

# 3c) bus lease / calib baro that bai (chi xay ra khi prearm DA pass)
armed, rej, logs = cmd_arm(FSM_DISARMED, Prearm(), bus_lease_ok=False)
check("bus lease fail -> reject=BUS_LEASE", (not armed) and rej == BUS_LEASE, f"={rej}")
if not armed and rej == NONE:
    silent.append("bus_lease")

armed, rej, logs = cmd_arm(FSM_DISARMED, Prearm(), baro_calib_ok=False)
check("calib baro fail -> reject=BARO_CALIB_FAILED",
      (not armed) and rej == BARO_CALIB_FAILED, f"={rej}")
if not armed and rej == NONE:
    silent.append("baro_calib")

check("KHONG CON duong tu choi IM LANG nao", not silent, f"im lang: {silent}")

# ============================================================================
print("\nTEST 4: baro/battery TAT -> KHONG bi chan boi dieu kien cua chung")
armed, rej, logs = cmd_arm(FSM_DISARMED,
                            Prearm(baro_ready=False, baro_healthy=False,
                                   alt_estimator_valid=False),
                            baro_ok_driver=False)
check("baro driver TAT + moi dk baro sai -> VAN armed", armed, f"rej={rej}")
armed, rej, logs = cmd_arm(FSM_DISARMED, Prearm(battery_v=0.0),
                            battery_ok_driver=False)
check("battery driver TAT + V=0 -> VAN armed", armed, f"rej={rej}")

# ============================================================================
print("\nTEST 5: reject ghi ly do DAU TIEN theo thu tu kiem, va log DU HET")
# Nhieu loi cung luc: dau tien theo thu tu C la attitude (truoc IMU/pin/baro)
p = Prearm(attitude_valid=False, imu_fresh=False, battery_v=3.0,
           baro_ready=False, deadline_miss_streak=60)
armed, rej, logs = cmd_arm(FSM_DISARMED, p)
check("reject = ATTITUDE_INVALID (dau tien theo thu tu)",
      rej == ATTITUDE_INVALID, f"={rej}")
check("van log DU moi ly do (>=5)", len(logs) >= 5, f"logs={len(logs)}")

# ============================================================================
print("\nTEST 6: moi ma reject deu co text huong dan trong GUI")
import importlib.util
import pathlib
REPO = pathlib.Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "uav_udp_console", REPO / "tools" / "uav_udp_console.py")
gui = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gui)
missing = [str(c) for c in range(1, 16)
           if not gui.ARM_REJECT_NAMES.get(str(c))]
check("ARM_REJECT_NAMES phu het ma 1..15", not missing, f"thieu: {missing}")
check("ma 2 (STATE) noi ro cach thoat (KILL)",
      "KILL" in gui.ARM_REJECT_NAMES["2"], gui.ARM_REJECT_NAMES["2"])

# ============================================================================
print("")
print("TEST 7: MAG KHONG con la dieu kien bat buoc")
# mag chua calib, mag_enabled=True -> VAN phai arm duoc
unc = recompute_uncalibrated(accel_valid=True, mag_valid=False, mag_enabled=True)
check("mag chua calib + mag BAT -> KHONG uncalibrated", not unc)
armed, rej, _ = cmd_arm(FSM_DISARMED, Prearm(), uncalibrated=unc)
check("=> VAN ARM duoc", armed and rej == NONE, f"armed={armed} rej={rej}")

# mag chua calib, mag_enabled=False -> cung phai arm duoc
unc = recompute_uncalibrated(accel_valid=True, mag_valid=False, mag_enabled=False)
check("mag chua calib + mag TAT -> KHONG uncalibrated", not unc)

# nhung ACCEL van BAT BUOC
unc = recompute_uncalibrated(accel_valid=False, mag_valid=True, mag_enabled=True)
check("accel chua calib -> VAN uncalibrated", unc)
armed, rej, _ = cmd_arm(FSM_DISARMED, Prearm(), uncalibrated=unc)
check("=> bi tu choi UNCALIBRATED", (not armed) and rej == UNCALIBRATED, f"rej={rej}")

# va GYRO van BAT BUOC (qua prearm_check, ma rieng)
armed, rej, _ = cmd_arm(FSM_DISARMED, Prearm(gyro_calibrated=False))
check("gyro chua calib -> VAN bi chan (ma GYRO_CALIB)",
      (not armed) and rej == GYRO_CALIB, f"rej={rej}")

# text GUI khong duoc nhac mag nhu bat buoc nua
import importlib.util as _ilu, pathlib as _pl
_spec = _ilu.spec_from_file_location(
    "uav_udp_console", _pl.Path(__file__).resolve().parents[1] / "tools" / "uav_udp_console.py")
_gui = _ilu.module_from_spec(_spec)
_spec.loader.exec_module(_gui)
_msg = _gui.ARM_REJECT_NAMES["1"]
check("text GUI ma 1 KHONG doi mag", "accel/mag" not in _msg, _msg)
check("text GUI ma 1 noi ro mag khong bat buoc", "KHONG bat buoc" in _msg, _msg)

print()
if FAILS:
    print(f"FAIL: {len(FAILS)} test -> {FAILS}")
    sys.exit(1)
print("PASS: tat ca TEST 1..7 (cong ARM — khong im lang; mag khong bat buoc)")
