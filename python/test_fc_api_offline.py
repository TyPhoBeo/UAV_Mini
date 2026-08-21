"""Test offline (CPython, KHÔNG cần MicroPython/phần cứng) cho logic BLOCKING
của fc_api.py — mock hẳn module `fc` (raw) để kiểm vòng lặp poll/timeout/fault
tự nó đúng, tách biệt khỏi C/firmware. Chạy: python test_fc_api_offline.py
"""
import sys
import types


class FakeClock:
    """Đồng hồ giả: sleep() nhảy thời gian ngay lập tức thay vì chờ thật —
    test chạy tức thời kể cả khi timeout mặc định là vài giây."""
    def __init__(self):
        self.t = 1000.0

    def time(self):
        return self.t

    def sleep(self, s):
        self.t += s


class MockFc:
    def __init__(self):
        self.commands = []
        self._state = "DISARMED"
        self._alt_mm = 0
        self._target_alt_mm = None
        self._climb_rate_mm_per_call = 100
        self._get_state_calls = 0
        self._takeoff_ticks_to_holding = 3
        self._get_state_calls_since_land = 0
        self._land_ticks_to_disarmed = 3
        self._force_fault_after_calls = None
        self._never_reach_holding = False

        # ---- control()/get_states() ----
        self._alt_mm_for_states = 0

        # ---- calibration ----
        self._uncalibrated = True
        self._calib_gyro_active = False
        self._gyro_active_calls = 0
        self._gyro_ticks_to_done = 2
        self._calib_accel_capturing = False
        self._accel_capture_calls = 0
        self._accel_ticks_to_done = 2
        self._calib_accel_faces_done = 0
        self._calib_mag_active = False
        self._mag_sample_count = 0

    def arm(self):
        self.commands.append(("arm",)); self._state = "ARMED"

    def disarm(self):
        self.commands.append(("disarm",)); self._state = "DISARMED"

    def takeoff(self, mm):
        self.commands.append(("takeoff", mm))
        self._state = "TAKING_OFF"
        self._get_state_calls = 0
        # Firmware giờ TỰ leo tới target trong pha TKO_CLIMB (xem
        # takeoff_land.h) — mock phản ánh đúng: nhận target ngay từ lệnh
        # takeoff, KHÔNG đợi set_altitude() thứ hai từ Python.
        self._target_alt_mm = mm

    def land(self):
        self.commands.append(("land",))
        self._state = "LANDING"
        self._get_state_calls_since_land = 0

    def set_altitude(self, mm):
        self.commands.append(("set_altitude", mm))
        self._target_alt_mm = mm

    def set_yaw(self, deg):
        self.commands.append(("set_yaw", deg))

    def move(self, direction, pct, sec):
        self.commands.append(("move", direction, pct, sec))

    def hover(self, sec):
        self.commands.append(("hover", sec))

    def heartbeat(self):
        self.commands.append(("heartbeat",))

    def set_param(self, name, value):
        self.commands.append(("set_param", name, value))

    def get_altitude(self):
        if self._target_alt_mm is not None and self._alt_mm < self._target_alt_mm:
            self._alt_mm = min(self._target_alt_mm, self._alt_mm + self._climb_rate_mm_per_call)
        return self._alt_mm

    def battery(self):
        return 7.4

    def get_attitude(self):
        return (0.0, 0.0, 0.0)

    def get_state(self):
        self._get_state_calls += 1
        if self._force_fault_after_calls is not None and \
                self._get_state_calls >= self._force_fault_after_calls:
            self._state = "EMERGENCY"
            return self._state
        if self._state == "TAKING_OFF" and not self._never_reach_holding and \
                self._get_state_calls >= self._takeoff_ticks_to_holding:
            self._state = "HOLDING"
        if self._state == "LANDING":
            self._get_state_calls_since_land += 1
            if self._get_state_calls_since_land >= self._land_ticks_to_disarmed:
                self._state = "DISARMED"
        return self._state

    # ---- fc.control()/get_states() ----
    def control(self, rol, pit, yaw, thr):
        self.commands.append(("control", rol, pit, yaw, thr))

    def set_flightmode(self, mode):
        self.commands.append(("set_flightmode", mode))

    def get_states(self):
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 7.4, self._alt_mm_for_states)

    # ---- Calibration ----
    def calibrate_gyro(self):
        self.commands.append(("calibrate_gyro",))
        self._calib_gyro_active = True
        self._gyro_active_calls = 0

    def calibrate_accel_face(self):
        self.commands.append(("calibrate_accel_face",))
        self._calib_accel_capturing = True
        self._accel_capture_calls = 0

    def calibrate_gyro_abort(self):
        self.commands.append(("calibrate_gyro_abort",))
        self._calib_gyro_active = False

    def calibrate_accel_reset(self):
        self.commands.append(("calibrate_accel_reset",))
        self._calib_accel_faces_done = 0

    def calibrate_mag_start(self):
        self.commands.append(("calibrate_mag_start",))
        self._calib_mag_active = True

    def calibrate_mag_stop(self):
        self.commands.append(("calibrate_mag_stop",))
        self._calib_mag_active = False

    def calibrate_mag_abort(self):
        self.commands.append(("calibrate_mag_abort",))
        self._calib_mag_active = False

    def calibrate_baro_ground(self):
        self.commands.append(("calibrate_baro_ground",))

    def calibrate_erase(self):
        self.commands.append(("calibrate_erase",))
        self._uncalibrated = True

    def is_calibrated(self):
        return not self._uncalibrated

    def calib_status(self):
        if self._calib_gyro_active:
            self._gyro_active_calls += 1
            if self._gyro_active_calls >= self._gyro_ticks_to_done:
                self._calib_gyro_active = False
        if self._calib_accel_capturing:
            self._accel_capture_calls += 1
            if self._accel_capture_calls >= self._accel_ticks_to_done:
                self._calib_accel_capturing = False
                self._calib_accel_faces_done += 1
        return (self._uncalibrated, self._calib_gyro_active, self._calib_accel_capturing,
                self._calib_accel_faces_done, self._calib_mag_active, self._mag_sample_count)


def fresh_import(mock):
    """Cài mock vào sys.modules['fc'] rồi (re)import fc_api sạch."""
    sys.modules["fc"] = mock
    sys.modules.pop("fc_api", None)
    import fc_api
    return fc_api


def patch_clock(fc_api_mod):
    clock = FakeClock()
    fc_api_mod.time.time = clock.time
    fc_api_mod.time.sleep = clock.sleep
    return clock


def test_takeoff_happy_path():
    mock = MockFc()
    fc_api = fresh_import(mock)
    patch_clock(fc_api)

    fc_api.arm()
    fc_api.takeoff(800)

    assert mock._state == "HOLDING", mock._state
    assert ("arm",) in mock.commands
    assert ("takeoff", 800) in mock.commands
    # KHÔNG được gửi set_altitude() sau takeoff: toàn bộ chuỗi (gồm pha leo)
    # nằm trong flight_core. Lệnh thứ hai từ Python nghĩa là semantics bay lại
    # rò lên tầng script — đúng thứ refactor này loại bỏ.
    assert ("set_altitude", 800) not in mock.commands, \
        "takeoff() KHONG duoc gui set_altitude() nua — firmware tu leo (TKO_CLIMB)"
    print("test_takeoff_happy_path: PASS")


def test_takeoff_timeout_lands_safely():
    mock = MockFc()
    mock._never_reach_holding = True   # kẹt mãi ở TAKING_OFF
    fc_api = fresh_import(mock)
    patch_clock(fc_api)

    fc_api.arm()
    try:
        fc_api.takeoff(800, timeout_s=2.0)
        assert False, "phai raise FcTimeoutError"
    except fc_api.FcTimeoutError:
        pass

    assert ("land",) in mock.commands, "timeout PHAI tu goi land() -> an toan"
    print("test_takeoff_timeout_lands_safely: PASS")


def test_takeoff_fault_raises():
    mock = MockFc()
    mock._force_fault_after_calls = 2
    fc_api = fresh_import(mock)
    patch_clock(fc_api)

    fc_api.arm()
    try:
        fc_api.takeoff(800)
        assert False, "phai raise FcFaultError"
    except fc_api.FcFaultError:
        pass
    print("test_takeoff_fault_raises: PASS")


def test_land_happy_path():
    mock = MockFc()
    mock._state = "HOLDING"
    fc_api = fresh_import(mock)
    patch_clock(fc_api)

    fc_api.land()
    assert mock._state == "DISARMED"
    assert ("land",) in mock.commands
    print("test_land_happy_path: PASS")


def test_land_timeout_raises():
    mock = MockFc()
    mock._state = "HOLDING"
    mock._land_ticks_to_disarmed = 10 ** 9   # không bao giờ chạm đất trong test
    fc_api = fresh_import(mock)
    patch_clock(fc_api)

    try:
        fc_api.land(timeout_s=2.0)
        assert False, "phai raise FcTimeoutError"
    except fc_api.FcTimeoutError:
        pass
    print("test_land_timeout_raises: PASS")


def test_hover_stops_early_on_fault():
    mock = MockFc()
    mock._state = "HOLDING"
    fc_api = fresh_import(mock)
    clock = patch_clock(fc_api)

    calls = {"n": 0}
    real_get_state = mock.get_state

    def flaky_get_state():
        calls["n"] += 1
        if calls["n"] == 2:
            mock._state = "LANDING"   # fault/land giữa chừng
        return real_get_state()

    mock.get_state = flaky_get_state

    t_before = clock.t
    fc_api.hover(9999.0)   # sẽ KHÔNG chờ đủ 9999s vì state rời HOLDING sớm
    assert clock.t - t_before < 9999.0
    print("test_hover_stops_early_on_fault: PASS")


def test_move_sends_command_and_blocks_duration():
    mock = MockFc()
    fc_api = fresh_import(mock)
    clock = patch_clock(fc_api)

    t_before = clock.t
    fc_api.move("forward", 40, 2.0)
    assert ("move", "forward", 40, 2.0) in mock.commands
    assert abs((clock.t - t_before) - 2.0) < 1e-9
    print("test_move_sends_command_and_blocks_duration: PASS")


def test_heartbeat_manual_beat():
    mock = MockFc()
    fc_api = fresh_import(mock)
    patch_clock(fc_api)

    fc_api.heartbeat.beat()
    assert ("heartbeat",) in mock.commands
    print("test_heartbeat_manual_beat: PASS")


def test_control_and_get_states_passthrough():
    mock = MockFc()
    fc_api = fresh_import(mock)
    patch_clock(fc_api)

    fc_api.control(50, -20, 100, -30)
    assert ("control", 50, -20, 100, -30) in mock.commands
    fc_api.set_flightmode(0)
    assert ("set_flightmode", 0) in mock.commands
    assert fc_api.get_states() == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 7.4, 0)
    print("test_control_and_get_states_passthrough: PASS")


def test_calibrate_gyro_happy_path():
    mock = MockFc()
    fc_api = fresh_import(mock)
    patch_clock(fc_api)

    fc_api.calibrate_gyro()
    assert ("calibrate_gyro",) in mock.commands
    assert mock._calib_gyro_active is False   # đã poll tới khi xong (mock: 2 tick)
    print("test_calibrate_gyro_happy_path: PASS")


def test_calibrate_gyro_timeout_raises():
    mock = MockFc()
    mock._gyro_ticks_to_done = 10 ** 9   # không bao giờ xong trong test
    fc_api = fresh_import(mock)
    patch_clock(fc_api)

    try:
        fc_api.calibrate_gyro(timeout_s=2.0)
        assert False, "phai raise FcTimeoutError"
    except fc_api.FcTimeoutError:
        pass
    print("test_calibrate_gyro_timeout_raises: PASS")


def test_calibrate_accel_6face_happy_path():
    mock = MockFc()
    fc_api = fresh_import(mock)
    patch_clock(fc_api)

    fc_api.calibrate_accel_6face()
    assert mock.commands.count(("calibrate_accel_face",)) == 6
    assert mock._calib_accel_faces_done == 6
    print("test_calibrate_accel_6face_happy_path: PASS")


def test_calibrate_mag_start_sleep_stop():
    mock = MockFc()
    fc_api = fresh_import(mock)
    clock = patch_clock(fc_api)

    t_before = clock.t
    fc_api.calibrate_mag(duration_s=10.0)
    assert ("calibrate_mag_start",) in mock.commands
    assert ("calibrate_mag_stop",) in mock.commands
    assert abs((clock.t - t_before) - 10.0) < 1e-9
    print("test_calibrate_mag_start_sleep_stop: PASS")


def test_calibrate_erase_and_is_calibrated():
    mock = MockFc()
    mock._uncalibrated = False
    fc_api = fresh_import(mock)
    patch_clock(fc_api)

    assert fc_api.is_calibrated() is True
    fc_api.calibrate_erase()
    assert fc_api.is_calibrated() is False
    print("test_calibrate_erase_and_is_calibrated: PASS")


if __name__ == "__main__":
    test_takeoff_happy_path()
    test_takeoff_timeout_lands_safely()
    test_takeoff_fault_raises()
    test_land_happy_path()
    test_land_timeout_raises()
    test_hover_stops_early_on_fault()
    test_move_sends_command_and_blocks_duration()
    test_heartbeat_manual_beat()
    test_control_and_get_states_passthrough()
    test_calibrate_gyro_happy_path()
    test_calibrate_gyro_timeout_raises()
    test_calibrate_accel_6face_happy_path()
    test_calibrate_mag_start_sleep_stop()
    test_calibrate_erase_and_is_calibrated()
    print("\nALL fc_api OFFLINE TESTS PASS")
