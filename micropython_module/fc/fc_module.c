// fc — module MicroPython lộ API điều khiển drone. TẦNG NÀY CHỈ MARSHAL tham
// số + gọi fc_bridge (== flight_core qua 2 kênh command/telemetry). KHÔNG một
// phép toán điều khiển nào (PID/mixer/state machine) được viết ở đây — nguyên
// tắc bất di của kiến trúc 2 tầng (xem README.md).
//
// CHƯA BUILD-VERIFY được với micropython thật trong lần scaffold này (cần
// clone micropython + esp-idf phiên bản pin theo port esp32 của nó — ngoài
// phạm vi lần đầu). Pattern dưới đây theo ĐÚNG API chuẩn micropython user C
// module (mp_obj_fun_builtin_*, MP_DEFINE_CONST_FUN_OBJ_*, MP_REGISTER_MODULE)
// — anh build thử theo README rồi báo lỗi cụ thể nếu API lệch phiên bản.
#include <string.h>

#include "py/obj.h"
#include "py/runtime.h"

#include "fc_bridge.h"
#include "flight_core/flight_state_machine.h"

static void raise_if_queue_full(bool pushed) {
    if (!pushed) {
        mp_raise_msg(&mp_type_RuntimeError,
            MP_ERROR_TEXT("fc: command queue day (COMMAND_QUEUE_DEPTH) - thu lai"));
    }
}

// ---- arm() / disarm() ----
static mp_obj_t fc_arm(void) {
    command_t cmd = {0};
    cmd.type = CMD_ARM;
    raise_if_queue_full(fc_bridge_push(&cmd));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(fc_arm_obj, fc_arm);

static mp_obj_t fc_disarm(void) {
    command_t cmd = {0};
    cmd.type = CMD_DISARM;
    raise_if_queue_full(fc_bridge_push(&cmd));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(fc_disarm_obj, fc_disarm);

// kill() — BYPASS toàn bộ FSM guard, cắt motor + ép DISARMED cưỡng bức bất kể
// state hiện tại (xem CMD_KILL trong command.h) — nút khẩn cấp, KHÔNG chờ
// PID/landing. KHÁC disarm() (chỉ hợp lệ từ ARMED/BENCH_RAMP).
static mp_obj_t fc_kill(void) {
    command_t cmd = {0};
    cmd.type = CMD_KILL;
    raise_if_queue_full(fc_bridge_push(&cmd));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(fc_kill_obj, fc_kill);

// ---- takeoff(mm) ----
// LƯU Ý (xem takeoff_land.h + flight_core.c): pha spool KHÔNG leo tới mm —
// nó chỉ rời đất an toàn rồi CHỐT target = độ cao lúc đó. Muốn thật sự leo
// tới mm, wrapper Python (python/fc_api.py) tự gọi set_altitude(mm) SAU khi
// thấy get_state() == "HOLDING". fc.takeoff() ở tầng C chỉ kích trình tự.
static mp_obj_t fc_takeoff(mp_obj_t alt_mm_obj) {
    command_t cmd = {0};
    cmd.type = CMD_TAKEOFF;
    cmd.as.takeoff.alt_mm = mp_obj_get_int(alt_mm_obj);
    raise_if_queue_full(fc_bridge_push(&cmd));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(fc_takeoff_obj, fc_takeoff);

static mp_obj_t fc_land(void) {
    command_t cmd = {0};
    cmd.type = CMD_LAND;
    raise_if_queue_full(fc_bridge_push(&cmd));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(fc_land_obj, fc_land);

static mp_obj_t fc_set_altitude(mp_obj_t alt_mm_obj) {
    command_t cmd = {0};
    cmd.type = CMD_SET_ALTITUDE;
    cmd.as.set_altitude.alt_mm = mp_obj_get_int(alt_mm_obj);
    raise_if_queue_full(fc_bridge_push(&cmd));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(fc_set_altitude_obj, fc_set_altitude);

// set_yaw(deg): quay TƯƠNG ĐỐI open-loop (xem command.h + flight_core.c —
// KHÔNG phải absolute heading, cần mag + heading-hold để làm đúng nghĩa đó).
static mp_obj_t fc_set_yaw(mp_obj_t deg_obj) {
    command_t cmd = {0};
    cmd.type = CMD_SET_YAW;
    cmd.as.set_yaw.yaw_deg = mp_obj_get_float(deg_obj);
    raise_if_queue_full(fc_bridge_push(&cmd));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(fc_set_yaw_obj, fc_set_yaw);

// move(dir, pct, sec) — dir: "forward"/"back"/"left"/"right"/"up"/"down"/"cw"/"ccw".
static mp_obj_t fc_move(mp_obj_t dir_obj, mp_obj_t pct_obj, mp_obj_t sec_obj) {
    move_dir_t dir;
    const char *dir_str = mp_obj_str_get_str(dir_obj);
    if (!fc_bridge_parse_move_dir(dir_str, &dir)) {
        mp_raise_ValueError(MP_ERROR_TEXT(
            "fc.move: dir phai la forward/back/left/right/up/down/cw/ccw"));
    }
    command_t cmd = {0};
    cmd.type = CMD_MOVE;
    cmd.as.move.dir = dir;
    cmd.as.move.pct = mp_obj_get_int(pct_obj);
    cmd.as.move.sec = mp_obj_get_float(sec_obj);
    raise_if_queue_full(fc_bridge_push(&cmd));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_3(fc_move_obj, fc_move);

// hover(sec) — về/giữ HOLDING. Bản thân KHÔNG chặn (blocking) — wrapper Python
// (fc_api.py) mới là chỗ sleep+poll theo đúng "ngữ nghĩa blocking" của spec.
static mp_obj_t fc_hover(mp_obj_t sec_obj) {
    (void)sec_obj;   // sec chỉ có ý nghĩa ở tầng Python (polling loop)
    command_t cmd = {0};
    cmd.type = CMD_HOVER;
    raise_if_queue_full(fc_bridge_push(&cmd));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(fc_hover_obj, fc_hover);

static mp_obj_t fc_heartbeat(void) {
    command_t cmd = {0};
    cmd.type = CMD_HEARTBEAT;
    raise_if_queue_full(fc_bridge_push(&cmd));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(fc_heartbeat_obj, fc_heartbeat);

// set_param(name, value) — xem flight_core.c: apply_set_param() cho danh sách
// tên hợp lệ hiện có (hover_duty, spool_ms, spool_duty).
static mp_obj_t fc_set_param(mp_obj_t name_obj, mp_obj_t value_obj) {
    command_t cmd = {0};
    cmd.type = CMD_SET_PARAM;
    const char *name = mp_obj_str_get_str(name_obj);
    strncpy(cmd.as.set_param.name, name, COMMAND_PARAM_NAME_MAX - 1);
    cmd.as.set_param.name[COMMAND_PARAM_NAME_MAX - 1] = '\0';
    cmd.as.set_param.value = mp_obj_get_float(value_obj);
    raise_if_queue_full(fc_bridge_push(&cmd));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(fc_set_param_obj, fc_set_param);

// ---- test_motor(motor_idx, duty_pct) — bench-test, xem command.h CMD_TEST_MOTOR ----
// motor_idx: 1..4 = spin RIÊNG LẺ M1..M4 (xác nhận vị trí vật lý/chiều quay),
// 0 = CẢ 4 CÙNG LÚC (sanity-check nhanh, KHÔNG dùng xác nhận vị trí). CHỈ chạy
// khi DISARMED (flight_core.c tự chặn) — THÁO CÁNH QUẠT trước khi gọi.
static mp_obj_t fc_test_motor(mp_obj_t motor_idx_obj, mp_obj_t duty_pct_obj) {
    command_t cmd = {0};
    cmd.type = CMD_TEST_MOTOR;
    cmd.as.test_motor.motor_idx = mp_obj_get_int(motor_idx_obj);
    cmd.as.test_motor.duty_pct = mp_obj_get_int(duty_pct_obj);
    raise_if_queue_full(fc_bridge_push(&cmd));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(fc_test_motor_obj, fc_test_motor);

// ---- bench_start()/bench_step(delta_duty)/bench_stop() — bench-test tăng ga
// tay để tune PID (xem GHI CHÚ 3 flight_state_machine.h + CMD_BENCH_RAMP_*
// trong command.h). Drone PHẢI được giữ chặt/kẹp trên giá đỡ — KHÔNG phải chế
// độ bay. start() CHỈ hợp lệ từ ARMED; step() CHỈ hợp lệ khi đang BENCH_RAMP
// (bị firmware bỏ qua âm thầm nếu gọi sai state, xem flight_core.c); stop()
// cắt máy NGAY, KHÔNG latch (khác disarm()/kill()), bench_start() lại được luôn.
static mp_obj_t fc_bench_start(void) {
    command_t cmd = {0};
    cmd.type = CMD_BENCH_RAMP_START;
    raise_if_queue_full(fc_bridge_push(&cmd));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(fc_bench_start_obj, fc_bench_start);

static mp_obj_t fc_bench_step(mp_obj_t delta_duty_obj) {
    command_t cmd = {0};
    cmd.type = CMD_BENCH_THROTTLE_STEP;
    cmd.as.bench_step.delta_duty = mp_obj_get_int(delta_duty_obj);
    raise_if_queue_full(fc_bridge_push(&cmd));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(fc_bench_step_obj, fc_bench_step);

static mp_obj_t fc_bench_stop(void) {
    command_t cmd = {0};
    cmd.type = CMD_BENCH_RAMP_STOP;
    raise_if_queue_full(fc_bridge_push(&cmd));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(fc_bench_stop_obj, fc_bench_stop);

// ---- control(rol, pit, yaw, thr) — joystick angle-mode THƯỜNG TRỰC (kiểu
// pyDrone), mỗi trục -100..100. rol/pit -> GÓC mục tiêu (thả 0 = tự cân bằng
// phẳng). yaw -> TỐC ĐỘ quay (thả 0 = giữ nguyên heading, KHÔNG tự quay lại).
// thr -> slew độ cao mục tiêu khi đang HOLDING/FLYING. DÙNG CHUNG kho setpoint
// với ground-station UDP (@SP SET) — xem command.h CMD_CONTROL, chỉ 1 nguồn
// setpoint tay lái thắng tại 1 thời điểm (lệnh mới nhất). KHÔNG blocking —
// script tự gọi lại định kỳ (giữ "stick position", giống RC transmitter).
static mp_obj_t fc_control(mp_obj_t rol_obj, mp_obj_t pit_obj, mp_obj_t yaw_obj, mp_obj_t thr_obj) {
    command_t cmd = {0};
    cmd.type = CMD_CONTROL;
    cmd.as.control.rol = mp_obj_get_float(rol_obj);
    cmd.as.control.pit = mp_obj_get_float(pit_obj);
    cmd.as.control.yaw = mp_obj_get_float(yaw_obj);
    cmd.as.control.thr = mp_obj_get_float(thr_obj);
    raise_if_queue_full(fc_bridge_push(&cmd));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_4(fc_control_obj, fc_control);

// set_flightmode(mode) — 0=headless (STUB, xem command.h CMD_SET_FLIGHTMODE:
// hành xử NHƯ head vì world-frame rotation cần trục mag đã xác nhận trên
// phần cứng, hiện CHƯA), 1=head (mặc định).
static mp_obj_t fc_set_flightmode(mp_obj_t mode_obj) {
    command_t cmd = {0};
    cmd.type = CMD_SET_FLIGHTMODE;
    cmd.as.set_flightmode.mode = mp_obj_get_int(mode_obj);
    raise_if_queue_full(fc_bridge_push(&cmd));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(fc_set_flightmode_obj, fc_set_flightmode);

// ---- get_states() — tuple gộp: (roll, pitch, yaw [độ], rol_in, pit_in,
// yaw_in [input hiện tại, -100..100], battery [V], alt [mm]) ----
static mp_obj_t fc_get_states(void) {
    telemetry_snapshot_t t;
    fc_bridge_read_telemetry(&t);
    float rol_in = 0.0f, pit_in = 0.0f, yaw_in = 0.0f, thr_in = 0.0f;
    fc_bridge_get_control_input(&rol_in, &pit_in, &yaw_in, &thr_in);
    mp_obj_t tuple[8] = {
        mp_obj_new_float(t.roll_deg), mp_obj_new_float(t.pitch_deg), mp_obj_new_float(t.yaw_deg),
        mp_obj_new_float(rol_in), mp_obj_new_float(pit_in), mp_obj_new_float(yaw_in),
        mp_obj_new_float(t.battery_v),
        mp_obj_new_int((mp_int_t)(t.alt_m * 1000.0f)),
    };
    return mp_obj_new_tuple(8, tuple);
}
static MP_DEFINE_CONST_FUN_OBJ_0(fc_get_states_obj, fc_get_states);

// ---- Calibration (bench-only, CHỈ chạy khi DISARMED — xem calibration.h) ----
static mp_obj_t fc_calibrate_gyro(void) {
    command_t cmd = {0};
    cmd.type = CMD_CALIB_GYRO;
    raise_if_queue_full(fc_bridge_push(&cmd));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(fc_calibrate_gyro_obj, fc_calibrate_gyro);

static mp_obj_t fc_calibrate_gyro_abort(void) {
    command_t cmd = {0};
    cmd.type = CMD_CALIB_GYRO_ABORT;
    raise_if_queue_full(fc_bridge_push(&cmd));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(fc_calibrate_gyro_abort_obj, fc_calibrate_gyro_abort);

// calibrate_accel_face() — bắt 1 mặt trong quy trình 6-face (~0.5s, giữ yên
// drone). Gọi lại 6 lần, đổi hướng đặt drone mỗi lần (miễn sao 6 lần gộp lại
// mỗi trục thấy được cả +g và -g — thứ tự/tên mặt không quan trọng, xem
// flight_core.c). Poll fc.get_states()... thực ra dùng telemetry riêng qua
// status console/UDP để biết capturing xong chưa (calib_accel_capturing).
static mp_obj_t fc_calibrate_accel_face(void) {
    command_t cmd = {0};
    cmd.type = CMD_CALIB_ACCEL_FACE;
    raise_if_queue_full(fc_bridge_push(&cmd));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(fc_calibrate_accel_face_obj, fc_calibrate_accel_face);

static mp_obj_t fc_calibrate_accel_reset(void) {
    command_t cmd = {0};
    cmd.type = CMD_CALIB_ACCEL_RESET;
    raise_if_queue_full(fc_bridge_push(&cmd));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(fc_calibrate_accel_reset_obj, fc_calibrate_accel_reset);

// calibrate_mag_start() — TỰ ĐỘNG chạy CALIB_MAG_DURATION_MS (60s, xem
// tuning.h) rồi tự tính hard/soft-iron — XOAY drone hình số 8 SUỐT thời gian
// này. calibrate_mag_stop() vẫn dùng được để kết thúc SỚM (tính ngay với mẫu
// đã thu) — không bắt buộc gọi nếu muốn đợi hết đủ 60s.
static mp_obj_t fc_calibrate_mag_start(void) {
    command_t cmd = {0};
    cmd.type = CMD_CALIB_MAG_START;
    raise_if_queue_full(fc_bridge_push(&cmd));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(fc_calibrate_mag_start_obj, fc_calibrate_mag_start);

static mp_obj_t fc_calibrate_mag_stop(void) {
    command_t cmd = {0};
    cmd.type = CMD_CALIB_MAG_STOP;
    raise_if_queue_full(fc_bridge_push(&cmd));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(fc_calibrate_mag_stop_obj, fc_calibrate_mag_stop);

static mp_obj_t fc_calibrate_mag_abort(void) {
    command_t cmd = {0};
    cmd.type = CMD_CALIB_MAG_ABORT;
    raise_if_queue_full(fc_bridge_push(&cmd));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(fc_calibrate_mag_abort_obj, fc_calibrate_mag_abort);

// calibrate_erase() — bench/test ONLY, xóa toàn bộ calib NVS (nghiệm thu
// "xóa NVS -> UNCALIBRATED -> arm bị từ chối"). KHÔNG dùng trong mission bình thường.
static mp_obj_t fc_calibrate_erase(void) {
    command_t cmd = {0};
    cmd.type = CMD_CALIB_ERASE;
    raise_if_queue_full(fc_bridge_push(&cmd));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(fc_calibrate_erase_obj, fc_calibrate_erase);

// calibrate_baro_ground() — lấy lại mốc 0m + std-dev NGAY LÚC GỌI (KHÔNG lưu
// NVS — mốc áp suất nền đổi theo thời tiết/vị trí mỗi lần bay, khác gyro/
// accel/mag). Khuyến nghị gọi ngay TRƯỚC arm() để có mốc mới nhất, xem
// README mục "Pre-arm ground calibration". Block ~1s bên phía firmware.
static mp_obj_t fc_calibrate_baro_ground(void) {
    command_t cmd = {0};
    cmd.type = CMD_CALIB_BARO_GROUND;
    raise_if_queue_full(fc_bridge_push(&cmd));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(fc_calibrate_baro_ground_obj, fc_calibrate_baro_ground);

static mp_obj_t fc_is_calibrated(void) {
    telemetry_snapshot_t t;
    fc_bridge_read_telemetry(&t);
    return mp_obj_new_bool(!t.uncalibrated);
}
static MP_DEFINE_CONST_FUN_OBJ_0(fc_is_calibrated_obj, fc_is_calibrated);

// calib_status() — tuple (uncalibrated, gyro_active, accel_capturing,
// accel_faces_done, mag_active, mag_sample_count). Dùng cho python/fc_api.py
// poll tiến trình calib (ngữ nghĩa blocking CHỈ nằm ở fc_api.py, xem đó).
static mp_obj_t fc_calib_status(void) {
    telemetry_snapshot_t t;
    fc_bridge_read_telemetry(&t);
    mp_obj_t tuple[6] = {
        mp_obj_new_bool(t.uncalibrated),
        mp_obj_new_bool(t.calib_gyro_active),
        mp_obj_new_bool(t.calib_accel_capturing),
        mp_obj_new_int(t.calib_accel_faces_done),
        mp_obj_new_bool(t.calib_mag_active),
        mp_obj_new_int((mp_int_t)t.calib_mag_sample_count),
    };
    return mp_obj_new_tuple(6, tuple);
}
static MP_DEFINE_CONST_FUN_OBJ_0(fc_calib_status_obj, fc_calib_status);

// ---- Getters: đọc telemetry snapshot (kênh LÊN) ----
static mp_obj_t fc_get_altitude(void) {
    telemetry_snapshot_t t;
    fc_bridge_read_telemetry(&t);
    return mp_obj_new_int((mp_int_t)(t.alt_m * 1000.0f));   // mm
}
static MP_DEFINE_CONST_FUN_OBJ_0(fc_get_altitude_obj, fc_get_altitude);

static mp_obj_t fc_battery(void) {
    telemetry_snapshot_t t;
    fc_bridge_read_telemetry(&t);
    return mp_obj_new_float(t.battery_v);
}
static MP_DEFINE_CONST_FUN_OBJ_0(fc_battery_obj, fc_battery);

static mp_obj_t fc_get_state(void) {
    telemetry_snapshot_t t;
    fc_bridge_read_telemetry(&t);
    return mp_obj_new_str(fsm_state_name(t.state), strlen(fsm_state_name(t.state)));
}
static MP_DEFINE_CONST_FUN_OBJ_0(fc_get_state_obj, fc_get_state);

static mp_obj_t fc_get_attitude(void) {
    telemetry_snapshot_t t;
    fc_bridge_read_telemetry(&t);
    mp_obj_t tuple[3] = {
        mp_obj_new_float(t.roll_deg),
        mp_obj_new_float(t.pitch_deg),
        mp_obj_new_float(t.yaw_deg),
    };
    return mp_obj_new_tuple(3, tuple);
}
static MP_DEFINE_CONST_FUN_OBJ_0(fc_get_attitude_obj, fc_get_attitude);

// ---- module table ----
static const mp_rom_map_elem_t fc_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),      MP_ROM_QSTR(MP_QSTR_fc) },
    { MP_ROM_QSTR(MP_QSTR_arm),           MP_ROM_PTR(&fc_arm_obj) },
    { MP_ROM_QSTR(MP_QSTR_disarm),        MP_ROM_PTR(&fc_disarm_obj) },
    { MP_ROM_QSTR(MP_QSTR_kill),          MP_ROM_PTR(&fc_kill_obj) },
    { MP_ROM_QSTR(MP_QSTR_takeoff),       MP_ROM_PTR(&fc_takeoff_obj) },
    { MP_ROM_QSTR(MP_QSTR_land),          MP_ROM_PTR(&fc_land_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_altitude),  MP_ROM_PTR(&fc_set_altitude_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_yaw),       MP_ROM_PTR(&fc_set_yaw_obj) },
    { MP_ROM_QSTR(MP_QSTR_move),          MP_ROM_PTR(&fc_move_obj) },
    { MP_ROM_QSTR(MP_QSTR_hover),         MP_ROM_PTR(&fc_hover_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_altitude),  MP_ROM_PTR(&fc_get_altitude_obj) },
    { MP_ROM_QSTR(MP_QSTR_battery),       MP_ROM_PTR(&fc_battery_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_state),     MP_ROM_PTR(&fc_get_state_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_attitude),  MP_ROM_PTR(&fc_get_attitude_obj) },
    { MP_ROM_QSTR(MP_QSTR_heartbeat),     MP_ROM_PTR(&fc_heartbeat_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_param),     MP_ROM_PTR(&fc_set_param_obj) },
    { MP_ROM_QSTR(MP_QSTR_test_motor),    MP_ROM_PTR(&fc_test_motor_obj) },
    { MP_ROM_QSTR(MP_QSTR_bench_start),   MP_ROM_PTR(&fc_bench_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_bench_step),    MP_ROM_PTR(&fc_bench_step_obj) },
    { MP_ROM_QSTR(MP_QSTR_bench_stop),    MP_ROM_PTR(&fc_bench_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_control),       MP_ROM_PTR(&fc_control_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_flightmode),MP_ROM_PTR(&fc_set_flightmode_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_states),    MP_ROM_PTR(&fc_get_states_obj) },
    { MP_ROM_QSTR(MP_QSTR_calibrate_gyro),        MP_ROM_PTR(&fc_calibrate_gyro_obj) },
    { MP_ROM_QSTR(MP_QSTR_calibrate_gyro_abort),  MP_ROM_PTR(&fc_calibrate_gyro_abort_obj) },
    { MP_ROM_QSTR(MP_QSTR_calibrate_accel_face),  MP_ROM_PTR(&fc_calibrate_accel_face_obj) },
    { MP_ROM_QSTR(MP_QSTR_calibrate_accel_reset), MP_ROM_PTR(&fc_calibrate_accel_reset_obj) },
    { MP_ROM_QSTR(MP_QSTR_calibrate_mag_start),   MP_ROM_PTR(&fc_calibrate_mag_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_calibrate_mag_stop),    MP_ROM_PTR(&fc_calibrate_mag_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_calibrate_mag_abort),   MP_ROM_PTR(&fc_calibrate_mag_abort_obj) },
    { MP_ROM_QSTR(MP_QSTR_calibrate_erase),       MP_ROM_PTR(&fc_calibrate_erase_obj) },
    { MP_ROM_QSTR(MP_QSTR_calibrate_baro_ground), MP_ROM_PTR(&fc_calibrate_baro_ground_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_calibrated),         MP_ROM_PTR(&fc_is_calibrated_obj) },
    { MP_ROM_QSTR(MP_QSTR_calib_status),          MP_ROM_PTR(&fc_calib_status_obj) },
};
static MP_DEFINE_CONST_DICT(fc_module_globals, fc_module_globals_table);

const mp_obj_module_t fc_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&fc_module_globals,
};

// Đăng ký module "fc" — sau bước này Python gõ được `import fc`. Lần gọi ĐẦU
// TIÊN vào bất kỳ hàm fc.* nào sẽ tự chạy fc_bridge_init() (lazy — xem
// fc_bridge.c) nên KHÔNG cần hook thêm vào lúc boot VM.
MP_REGISTER_MODULE(MP_QSTR_fc, fc_user_cmodule);
