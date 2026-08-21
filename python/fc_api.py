"""fc_api — wrapper NGỮ NGHĨA BLOCKING quanh module C `fc` (raw, non-blocking).

Nguyên tắc: `fc.*` (module C, xem micropython_module/fc/fc_module.c) chỉ ĐẨY
LỆNH xuống flight_core rồi trả về NGAY — không có ý niệm "đợi tới khi xong".
Module này (`fc_api`) là nơi DUY NHẤT thêm ngữ nghĩa blocking: takeoff()/
land()/hover() tự POLL fc.get_state()/fc.get_altitude() tới khi hoàn thành
hoặc timeout, có timeout an toàn (quá giờ -> tự land() thay vì treo mission
mãi). KHÔNG có phép toán điều khiển nào ở đây — chỉ vòng lặp chờ + gọi lệnh.

MicroPython trên board thật: `import fc` là module C (USER_C_MODULES).
Không có `fc` (test offline trên CPython) -> import lỗi, dùng mock (xem
test_fc_api_offline.py) bằng cách gán sys.modules['fc'] trước khi import file
này.
"""
import time

import fc

try:
    import _thread
    _HAS_THREAD = True
except ImportError:
    _HAS_THREAD = False


class FcTimeoutError(Exception):
    """Poll quá timeout mà chưa đạt trạng thái mong đợi."""


class FcFaultError(Exception):
    """FSM rơi vào EMERGENCY/DISARMED ngoài ý muốn giữa lúc đang chờ."""


DEFAULT_TAKEOFF_TIMEOUT_S = 5.0
DEFAULT_CLIMB_TIMEOUT_S = 8.0
DEFAULT_LAND_TIMEOUT_S = 15.0
POLL_INTERVAL_S = 0.1
DEFAULT_HEARTBEAT_INTERVAL_S = 0.3
ALTITUDE_TOLERANCE_MM = 50

_FAULT_STATES = ("EMERGENCY",)


def arm():
    fc.arm()


def disarm():
    fc.disarm()


def kill():
    """BYPASS toan bo FSM guard, cat motor + ep DISARMED cuong buc bat ke
    state hien tai -- nut khan cap, KHONG cho PID/landing. KHAC disarm()
    (chi hop le tu ARMED/BENCH_RAMP)."""
    fc.kill()


def takeoff(alt_mm, timeout_s=DEFAULT_TAKEOFF_TIMEOUT_S,
            climb_timeout_s=DEFAULT_CLIMB_TIMEOUT_S):
    """Blocking: kích takeoff tới alt_mm, đợi tới khi firmware báo HOLDING.

    TOÀN BỘ chuỗi cất cánh nằm trong flight_core (SPOOL -> LIFTOFF_CONFIRM ->
    CLIMB -> SETTLE -> HOLDING, xem takeoff_land.h). Hàm này CHỈ POLL trạng
    thái — KHÔNG còn tự gửi set_altitude() sau khi thấy HOLDING như bản cũ.

    Vì sao đổi: bản cũ để semantics bay rò lên Python — firmware chỉ nhấc lên
    rồi chốt target = độ cao lúc đó (~20cm), Python phải gửi tiếp lệnh thứ hai
    mới leo tới alt_mm. Script crash/mất kết nối giữa hai lệnh thì drone treo
    lơ lửng sát đất, không ai hoàn tất cũng không ai huỷ. Giờ firmware tự chịu
    trách nhiệm hoàn thành HOẶC tự abort an toàn, kể cả khi Python biến mất.

    timeout_s tính cho CẢ chuỗi (gồm cả pha leo), nên phải rộng hơn bản cũ.
    """
    fc.takeoff(alt_mm)

    deadline_s = max(timeout_s, climb_timeout_s)
    t0 = time.time()
    while True:
        state = fc.get_state()
        if state == "HOLDING":
            break
        if state in _FAULT_STATES:
            raise FcFaultError("takeoff that bai, state=%s" % state)
        if state == "DISARMED":
            raise FcFaultError("takeoff bi tu choi hoac ABORT (xem log firmware)")
        if time.time() - t0 > deadline_s:
            fc.land()
            raise FcTimeoutError(
                "takeoff qua %.1fs ma chua HOLDING -> da goi land() an toan" % deadline_s)
        time.sleep(POLL_INTERVAL_S)


def _wait_altitude(alt_mm, timeout_s, tolerance_mm=ALTITUDE_TOLERANCE_MM):
    """Đợi độ cao chạm alt_mm (± tolerance).

    KHÔNG còn được takeoff() dùng — firmware tự leo trong pha TKO_CLIMB. Giữ
    lại cho set_altitude() thủ công: đổi độ cao KHI ĐANG BAY vẫn là lệnh một
    chiều không có ngữ nghĩa "xong", nên script cần chỗ để chờ.
    """
    t0 = time.time()
    while True:
        cur = fc.get_altitude()
        if abs(cur - alt_mm) <= tolerance_mm:
            return
        state = fc.get_state()
        if state in _FAULT_STATES or state in ("DISARMED", "LANDING"):
            return   # đang xử lý fault/hạ — không chờ leo nữa, an toàn ưu tiên
        if time.time() - t0 > timeout_s:
            return   # đã an toàn trên không (HOLDING), chỉ là chưa tới đúng độ cao
        time.sleep(POLL_INTERVAL_S)


def land(timeout_s=DEFAULT_LAND_TIMEOUT_S):
    fc.land()
    t0 = time.time()
    while fc.get_state() != "DISARMED":
        if time.time() - t0 > timeout_s:
            raise FcTimeoutError("land qua %.1fs ma chua DISARMED" % timeout_s)
        time.sleep(POLL_INTERVAL_S)


def hover(sec):
    """Giữ nguyên HOLDING/FLYING trong sec giây (poll, dừng sớm nếu fault)."""
    fc.hover(sec)
    t0 = time.time()
    while time.time() - t0 < sec:
        state = fc.get_state()
        if state not in ("HOLDING", "FLYING"):
            return
        time.sleep(POLL_INTERVAL_S)


def move(direction, pct, sec):
    """direction: forward/back/left/right/up/down/cw/ccw. Blocking đúng sec
    giây (khớp timed-command bên flight_core.c tự trả về HOLDING sau sec)."""
    fc.move(direction, pct, sec)
    time.sleep(sec)


def set_altitude(alt_mm):
    fc.set_altitude(alt_mm)


def set_yaw(deg):
    fc.set_yaw(deg)


def get_altitude():
    return fc.get_altitude()


def get_state():
    return fc.get_state()


def get_attitude():
    return fc.get_attitude()


def battery():
    return fc.battery()


def set_param(name, value):
    fc.set_param(name, value)


# ================= Bench-test (test_motor / tang ga tay tune PID) =================
# Ca hai deu CHI chay khi CHUA bay (test_motor: CHI DISARMED; bench_*: CHI
# ARMED/BENCH_RAMP) -- firmware tu chan (khong raise loi, chi bo qua am tham +
# log canh bao, xem flight_core.c). fc_api KHONG them ngu nghia blocking o day
# (giong het move()) -- goi status()/get_state() de tu xac nhan.

def test_motor(motor_idx, duty_pct):
    """motor_idx: 1..4 = spin RIENG LE M1..M4 (xac nhan vi tri vat ly/chieu
    quay), 0 = CA 4 CUNG LUC (sanity-check nhanh, KHONG dung xac nhan vi tri).
    CHI chay khi DISARMED -- THAO CANH QUAT truoc khi goi."""
    fc.test_motor(motor_idx, duty_pct)


def bench_start():
    """Vao che do bench-test tang ga tay de tune PID -- CHI hop le tu ARMED.
    Drone PHAI duoc GIU CHAT/KEP TREN GIA DO, day KHONG phai che do bay."""
    fc.bench_start()


def bench_step(delta_duty=20):
    """Cong delta_duty (am de giam) vao throttle bench-test hien tai, thang
    duty 0..MOTOR_SAFE_MAX_DUTY (KHONG phai %). CHI hop le khi dang BENCH_RAMP
    (goi bench_start() truoc)."""
    fc.bench_step(delta_duty)


def bench_stop():
    """Cat throttle bench-test NGAY (khong ramp xuong), ve ARMED -- KHONG
    latch (khac disarm()/kill()), bench_start() lai duoc luon."""
    fc.bench_stop()


# ================= fc.control() joystick angle-mode =================
# KHÔNG blocking (giống fc.move() ở tầng C) — script tự gọi lại định kỳ để
# "giữ stick", đúng kiểu RC transmitter/pyDrone. Xem fc_module.c CMD_CONTROL.

def control(rol, pit, yaw, thr):
    """rol/pit/yaw/thr: -100..100. rol/pit=GÓC (thả 0 -> tự cân bằng phẳng),
    yaw=TỐC ĐỘ quay (thả 0 -> giữ heading), thr=slew độ cao khi HOLDING/FLYING."""
    fc.control(rol, pit, yaw, thr)


def set_flightmode(mode):
    """0=headless (STUB, hien chay nhu head - xem README), 1=head (mac dinh)."""
    fc.set_flightmode(mode)


def get_states():
    """(roll, pitch, yaw [deg], rol_in, pit_in, yaw_in [-100..100], battery [V], alt [mm])."""
    return fc.get_states()


# ================= Calibration (bench-only, CHỈ khi DISARMED) =================
# Ngữ nghĩa BLOCKING (poll fc.calib_status()) sống Ở ĐÂY, giống hệt nguyên
# tắc arm/takeoff/land — fc.calibrate_*() (module C) chỉ đẩy lệnh, trả về ngay.

DEFAULT_CALIB_GYRO_TIMEOUT_S = 3.0
DEFAULT_CALIB_ACCEL_FACE_TIMEOUT_S = 2.0
DEFAULT_CALIB_MAG_MIN_DURATION_S = 60.0   # khớp CALIB_MAG_DURATION_MS firmware (tuning.h) — firmware
                                            # TỰ ĐỘNG kết thúc ở mốc này dù Python không gọi stop()


def calibrate_gyro(timeout_s=DEFAULT_CALIB_GYRO_TIMEOUT_S):
    """Blocking: đo lại gyro bias tĩnh (~1.5s, DỪNG YÊN drone). Raise FcTimeoutError
    nếu quá timeout mà firmware chưa báo xong (không tự retry)."""
    fc.calibrate_gyro()
    t0 = time.time()
    while fc.calib_status()[1]:   # gyro_active
        if time.time() - t0 > timeout_s:
            raise FcTimeoutError("calibrate_gyro qua %.1fs ma chua xong" % timeout_s)
        time.sleep(POLL_INTERVAL_S)


def calibrate_gyro_abort():
    """Huy phien calib_gyro dang do (neu co), KHONG luu."""
    fc.calibrate_gyro_abort()


def calibrate_accel_6face(prompt=None, face_timeout_s=DEFAULT_CALIB_ACCEL_FACE_TIMEOUT_S):
    """Blocking: chạy đủ 6 mặt. `prompt(face_idx)` (idx 0..5) nếu truyền vào sẽ
    được gọi TRƯỚC mỗi mặt (vd hàm in() nhắc người dùng đổi hướng đặt drone rồi
    input() chờ Enter) — mặc định KHÔNG nhắc gì, chỉ cách nhau 1 khoảng nghỉ
    ngắn (người dùng phải tự lo đổi hướng đủ nhanh nếu không truyền prompt).
    Thứ tự/tên mặt KHÔNG quan trọng — chỉ cần 6 lần đặt drone bao phủ +-g cả 3
    trục (xem flight_core.c). Raise FcTimeoutError nếu 1 mặt không xong kịp."""
    for face_idx in range(6):
        if prompt is not None:
            prompt(face_idx)
        else:
            time.sleep(0.5)
        fc.calibrate_accel_face()
        t0 = time.time()
        while fc.calib_status()[2]:   # accel_capturing
            if time.time() - t0 > face_timeout_s:
                raise FcTimeoutError("calibrate_accel_face (mat %d) qua %.1fs ma chua xong" % (face_idx, face_timeout_s))
            time.sleep(POLL_INTERVAL_S)


def calibrate_accel_reset():
    fc.calibrate_accel_reset()


def calibrate_mag(duration_s=DEFAULT_CALIB_MAG_MIN_DURATION_S):
    """Blocking: thu mẫu mag trong duration_s giây -- XOAY drone hình số 8 LIÊN
    TỤC trong lúc hàm này đang chạy (blocking đúng bằng thời gian ngủ, không
    poll gì thêm vì việc "đủ mẫu" chỉ biết được SAU khi stop). Firmware TỰ
    ĐỘNG kết thúc sau CALIB_MAG_DURATION_MS (mặc định 60s, xem tuning.h) dù
    không gọi calibrate_mag_stop() -- gọi hàm này với duration_s < 60 để kết
    thúc SỚM hơn (fc.calibrate_mag_stop() vẫn được gọi ở cuối, vô hại nếu
    firmware đã tự kết thúc trước đó)."""
    fc.calibrate_mag_start()
    time.sleep(duration_s)
    fc.calibrate_mag_stop()


def calibrate_mag_abort():
    """Huy phien calib_mag dang thu (neu co), KHONG tinh/luu du da co du mau."""
    fc.calibrate_mag_abort()


def calibrate_baro_ground():
    """Lay lai moc 0m + std-dev baro NGAY LUC GOI (KHONG luu NVS -- moc doi
    theo thoi tiet/vi tri moi lan bay). Khuyen nghi goi ngay TRUOC arm()."""
    fc.calibrate_baro_ground()


def calibrate_erase():
    """Bench/test ONLY -- xóa toàn bộ calib NVS, drone sẽ KHÔNG arm được tới
    khi calib lại. KHÔNG dùng trong mission bình thường."""
    fc.calibrate_erase()


def is_calibrated():
    return fc.is_calibrated()


def calib_status():
    """(uncalibrated, gyro_active, accel_capturing, accel_faces_done, mag_active, mag_sample_count)."""
    return fc.calib_status()


class Heartbeat:
    """Gửi fc.heartbeat() định kỳ để Commander không soft-fault do watchdog
    timeout trong mission dài. Dùng _thread nền nếu port hỗ trợ; nếu không,
    caller PHẢI tự gọi .beat() thủ công trong vòng lặp mission của mình."""

    def __init__(self, interval_s=DEFAULT_HEARTBEAT_INTERVAL_S):
        self.interval_s = interval_s
        self._running = False

    def start(self):
        if not _HAS_THREAD:
            return False
        self._running = True
        _thread.start_new_thread(self._loop, ())
        return True

    def _loop(self):
        while self._running:
            fc.heartbeat()
            time.sleep(self.interval_s)

    def stop(self):
        self._running = False

    def beat(self):
        fc.heartbeat()


heartbeat = Heartbeat()
