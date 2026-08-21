// Command — struct XUỐNG duy nhất mà MicroPython đẩy vào flight_core. Đây là
// kênh biên giới còn lại (cùng với telemetry.h). fc_bridge.c (phía
// micropython_module) chỉ làm việc dịch mp_obj_t Python <-> command_t rồi đẩy
// vào command_queue (FreeRTOS queue, an toàn giữa 2 core) — KHÔNG có con trỏ
// chung nào vào control state của flight_core.
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CMD_ARM = 0,
    CMD_DISARM,
    CMD_TAKEOFF,
    CMD_LAND,
    CMD_SET_ALTITUDE,
    CMD_SET_YAW,
    CMD_MOVE,
    CMD_HOVER,
    CMD_HEARTBEAT,
    CMD_SET_PARAM,
    // CMD_TEST_MOTOR — spin động cơ ở duty thấp để bench-test (xem app_config.h
    // mục "XÁC NHẬN VỊ TRÍ VẬT LÝ ĐỘNG CƠ"). as.test_motor.motor_idx: 1..4 =
    // spin RIÊNG LẺ M1..M4 (dùng để xác nhận vị trí vật lý + chiều quay từng
    // góc), 0 = spin CẢ 4 CÙNG LÚC cùng duty (sanity check nhanh — đủ cả 4
    // quay, không dùng để xác nhận vị trí/chiều, vì 4 con giống hệt nhau
    // không phân biệt được góc nào là góc nào khi chạy đồng thời).
    // CHỈ chạy khi FSM đang DISARMED (flight_core.c::apply_command() tự chặn
    // nếu không) — KHÔNG phải kênh điều khiển bay, chỉ dùng lúc bench-test
    // với cánh quạt đã THÁO HẾT.
    CMD_TEST_MOTOR,

    // CMD_BENCH_RAMP_START/CMD_BENCH_THROTTLE_STEP/CMD_BENCH_RAMP_STOP — chế
    // độ bench-test để tiện tune PID mà KHÔNG cần bay thật (drone PHẢI được
    // giữ chặt/kẹp trên giá đỡ — đây KHÔNG phải chế độ bay). Xem FSM_BENCH_RAMP
    // (flight_state_machine.h GHI CHÚ 3):
    //   START : ARMED -> BENCH_RAMP, throttle bắt đầu từ 0.
    //   STEP  : cộng as.bench_step.delta_duty (dương/âm) vào throttle hiện
    //           tại, clamp [0, MOTOR_SAFE_MAX_DUTY] — đây là nút "+"/"-" tăng/
    //           giảm ga từng nấc nhỏ (console mặc định bước 20, xem src/main.c).
    //           CHỈ hợp lệ khi đang BENCH_RAMP.
    //   STOP  : cắt máy NGAY (không ramp xuống) rồi về ARMED, KHÔNG latch
    //           (khác CMD_DISARM) — để lặp lại nhanh nhiều lần khi tune.
    // Attitude PID (roll/pitch/yaw) chạy BÌNH THƯỜNG suốt (dùng chung ngưỡng
    // ATT_MIN_THROTTLE_DUTY như mọi state khác) — mục đích DUY NHẤT của mode
    // này là tăng ga từ từ (qua nhiều lần STEP) và quan sát phản ứng roll/
    // pitch/yaw qua CMD_MOVE/CMD_CONTROL/@PID SET. Toàn bộ safety net (tilt/
    // attitude-stale cắt ngay, Commander heartbeat/battery/hard-tilt) áp dụng
    // đầy đủ cho state này.
    CMD_BENCH_RAMP_START,
    CMD_BENCH_THROTTLE_STEP,
    CMD_BENCH_RAMP_STOP,

    // CMD_BENCH_THROTTLE_OFFSET — offset TẠM THỜI cộng vào throttle bench,
    // dùng cho phím GIỮ (giữ W = +100, nhả = về 0). KHÁC HẲN
    // CMD_BENCH_THROTTLE_STEP: STEP cộng DỒN vĩnh viễn, OFFSET là giá trị
    // TUYỆT ĐỐI thay thế offset cũ và tự về 0 khi ngừng gửi.
    //
    // VÌ SAO TUYỆT ĐỐI + WATCHDOG chứ không phải "+100 lúc nhấn, -100 lúc nhả":
    // đây là UDP. Mất đúng gói "nhả phím" thì cặp +/- không cân và throttle
    // dính +100 VĨNH VIỄN trong lúc motor đang quay. Với offset tuyệt đối, một
    // gói mất chỉ làm trễ 1 chu kỳ gửi; và nếu ground-station chết hẳn thì
    // watchdog (BENCH_OFFSET_STALE_US) tự đưa về 0. Cùng nguyên tắc đã dùng
    // cho CMD_SET_ATTITUDE (xem SP_STALE_TIMEOUT_US).
    CMD_BENCH_THROTTLE_OFFSET,

    // ================= Ground-station UDP (xem net_link/command_parser) =================
    // CMD_KILL — KHÁC HẲN CMD_DISARM: CMD_DISARM chỉ hợp lệ từ FSM_ARMED/
    // FSM_BENCH_RAMP (xem fsm_on_disarm_request() trong flight_state_machine.c
    // — cố ý, đúng topology "disarm bình thường" chỉ từ trạng thái chưa-bay-
    // thật). CMD_KILL BYPASS toàn bộ guard — cắt motor + ép FSM về DISARMED
    // bất kể state hiện tại
    // (HOLDING/FLYING/TAKING_OFF/LANDING/EMERGENCY đều bị "thắng"), dùng cho
    // nút KILL/phím Esc — PHẢI thắng mọi thứ, không chờ PID/landing.
    CMD_KILL,

    // CMD_SET_ATTITUDE — setpoint bay tay THƯỜNG TRỰC (roll/pitch = góc tuyệt
    // đối [deg], yaw_rate = tốc độ quay [dps]) — KHÁC CMD_MOVE (xung có thời
    // hạn tự hết). Giữ nguyên tới khi có CMD_SET_ATTITUDE mới HOẶC watchdog
    // stale tự zero (xem SP_STALE_TIMEOUT_US trong flight_core.c) HOẶC
    // CMD_HOVER/CMD_KILL/CMD_LAND. Đây là kênh @SP SET của ground station.
    CMD_SET_ATTITUDE,

    // CMD_SET_TRIM — bias cộng thêm vào roll/pitch target (bù lệch cơ khí/CG),
    // KHÔNG sửa sensor raw. Xem @TRIM SET.
    CMD_SET_TRIM,

    // CMD_SET_PID — chỉnh 1 bộ gain (ANGLE|RATE × ROLL|PITCH|YAW) trong
    // attitude_gains_t, RESET integrator của ĐÚNG bộ đó (không đụng 5 bộ còn
    // lại, không reset toàn bộ flight controller). Xem @PID SET.
    CMD_SET_PID,

    // CMD_SET_MAHONY — đổi Kp/Ki Mahony LIVE, KHÔNG reset quaternion/attitude
    // hiện tại (xem mahony_set_config()). Xem @MAH SET.
    CMD_SET_MAHONY,

    // CMD_SET_ALT_TUNE — 4 gain cascade alt_hold (alt_kp/vz_kp/vz_ki/vz_ilimit).
    // hover TÁCH RIÊNG (xem CMD_SET_TKO_TUNE — dùng chung s_hold_tune.hover
    // với takeoff, khớp đúng thiết kế "1 hover neo" của alt_hold.h). Xem @ALT SET.
    CMD_SET_ALT_TUNE,

    // CMD_SET_TKO_TUNE — hover (ghi vào alt_hold_tune_t.hover, DÙNG CHUNG với
    // HOLD) + prime_duty/prime_ms/max_climb_ms (takeoff_tune_t). Xem @TKO SET.
    CMD_SET_TKO_TUNE,

    // CMD_SET_LAND_TUNE — descent_vz/flare_alt_m/flare_vz/touchdown_alt_m
    // (landing_tune_t). Xem @LAND SET.
    CMD_SET_LAND_TUNE,

    // CMD_SET_COMMANDER_CFG — geofence (alt_min/max) + ngưỡng fault
    // (battery_floor_v/heartbeat_timeout_ms/hard_tilt_deg/motor_sat_hard_ms),
    // xem commander.h. Chạy được BẤT KỲ LÚC NÀO (kể cả đang bay — geofence/
    // failsafe PHẢI chỉnh được live, không chỉ lúc DISARMED). Xem @CMDR SET.
    CMD_SET_COMMANDER_CFG,

    // ================= MicroPython fc.control()/get_states() =================
    // CMD_CONTROL — joystick angle-mode THƯỜNG TRỰC (rol/pit/yaw/thr, mỗi trục
    // -100..100, kiểu pyDrone). rol/pit -> target GÓC (roll/pitch), thả về 0 ->
    // TỰ CÂN BẰNG phẳng. yaw -> target TỐC ĐỘ quay, thả về 0 -> GIỮ heading
    // hiện tại (không tự quay lại). thr -> SLEW target_altitude (không step)
    // khi đang HOLDING/FLYING. DÙNG CHUNG kho lưu + watchdog stale với
    // CMD_SET_ATTITUDE (ground-station UDP, xem tuning.h mục 7) — lệnh mới
    // nhất từ NGUỒN NÀO cũng thắng, chỉ 1 setpoint tay lái tại 1 thời điểm.
    CMD_CONTROL,

    // CMD_SET_FLIGHTMODE — 0=headless (STUB, xem flight_core.c: hành xử NHƯ
    // head vì chưa có world-frame rotation đã xác nhận trục), 1=head (mặc định).
    CMD_SET_FLIGHTMODE,

    // ================= Calibration (calibration.h) =================
    // CHỈ chạy khi FSM đang DISARMED (flight_core.c tự chặn) — bench-only,
    // KHÔNG phải kênh điều khiển bay. Xem calibration.h + README.md.
    CMD_CALIB_GYRO,          // đo bias tĩnh ~CALIB_GYRO_DURATION_MS, không chặn stabilize_task
    CMD_CALIB_GYRO_ABORT,    // hủy phiên gyro đang đo (nếu có), KHÔNG lưu
    CMD_CALIB_ACCEL_FACE,    // bắt 1 mặt trong quy trình 6-face (gọi lại 6 lần, đổi hướng giữa các lần)
    CMD_CALIB_ACCEL_RESET,   // hủy tiến trình 6-face đang dở, làm lại từ đầu
    CMD_CALIB_MAG_START,     // bắt đầu thu mẫu mag, TỰ ĐỘNG chạy CALIB_MAG_DURATION_MS (60s, xem
                              // tuning.h) rồi tự tính — xoay hình số 8 SUỐT thời gian này
    CMD_CALIB_MAG_STOP,      // kết thúc SỚM (trước khi hết 60s) + tính hard/soft-iron + lưu NVS ngay
    CMD_CALIB_MAG_ABORT,     // hủy phiên mag đang thu (nếu có), KHÔNG tính/lưu dù đã đủ mẫu
    CMD_CALIB_ERASE,         // xóa TOÀN BỘ calib NVS (bench/test only, xem calibration.h)

    // CMD_CALIB_BARO_GROUND — lấy lại mốc 0m + std-dev áp suất nền NGAY LÚC
    // GỌI (xem baro_driver_calibrate_ground()) — KHÔNG lưu NVS (khác gyro/
    // accel/mag ở trên, mốc áp suất nền đổi theo thời tiết/vị trí mỗi lần bay,
    // KHÔNG có ý nghĩa "lưu vĩnh viễn"). Gọi được BẤT KỲ LÚC NÀO khi DISARMED
    // — khuyến nghị gọi lại NGAY TRƯỚC arm để có mốc mới nhất (xem README mục
    // "Pre-arm ground calibration"), không chỉ dựa vào lần calib tự động lúc
    // boot. Block ~1s (32 mẫu, giống pattern CMD_TEST_MOTOR) — chấp nhận được
    // vì DISARMED, không có gì đang bay.
    CMD_CALIB_BARO_GROUND,

    // ================= Sensor bring-up / bench diagnostics =================
    // CMD_MAG_SELFTEST — KHÔNG phải calibration (không đo/không lưu gì cả).
    // Chạy lại toàn bộ trình tự init QMC5883P và LOG TỪNG BƯỚC (CHIP_ID ->
    // reset -> [SIGN 0x29] -> CTRL2 -> CTRL1 -> DRDY -> XYZ -> vòng đọc 20ms)
    // để biết CHÍNH XÁC chip hỏng ở bước nào — xem mag_driver_selftest().
    // Dùng khi mag báo init fail hoặc calib_mag thu 0 mẫu: phải chắc chắn chip
    // phát mẫu ổn định TRƯỚC khi động vào bất cứ thứ gì ở tầng calibration.
    // as.mag_selftest.write_sign_reg: chạy 2 lần (0 rồi 1) và so log để chốt
    // xem con chip này có cần REG 0x29=0x06 như datasheet QST hay không.
    // BLOCK stabilize_task ~read_loop_ms+0.3s, chỉ chạy khi DISARMED (giống
    // CMD_TEST_MOTOR/CMD_CALIB_BARO_GROUND — chấp nhận được vì không có gì đang bay).
    CMD_MAG_SELFTEST,
} command_type_t;

typedef enum {
    MOVE_FORWARD = 0,
    MOVE_BACK,
    MOVE_LEFT,
    MOVE_RIGHT,
    MOVE_UP,
    MOVE_DOWN,
    MOVE_CW,     // xoay yaw phải (dps dương — khớp quy ước firmware)
    MOVE_CCW,
} move_dir_t;

// PID loop/axis — mã hóa gọn cho CMD_SET_PID (khớp "ANGLE|RATE" × "ROLL|PITCH|YAW"
// của giao thức @PID SET, xem command_parser.c).
typedef enum { PID_LOOP_ANGLE = 0, PID_LOOP_RATE } pid_loop_t;
typedef enum { PID_AXIS_ROLL = 0, PID_AXIS_PITCH, PID_AXIS_YAW } pid_axis_t;

// set_param(): tên tham số dạng chuỗi ngắn (vd "prime_ms", "hover_duty") ->
// map sang field trong AltTune/takeoff_tune_t ở flight_core.c. Danh sách hợp
// lệ + TODO ánh xạ: xem flight_core.c: command_apply_set_param().
#define COMMAND_PARAM_NAME_MAX 16

typedef struct {
    command_type_t type;
    union {
        struct { int32_t alt_mm; } takeoff;
        struct { int32_t alt_mm; } set_altitude;
        struct { float   yaw_deg; } set_yaw;
        struct { move_dir_t dir; int32_t pct; float sec; } move;
        struct { float sec; } hover;
        struct { char name[COMMAND_PARAM_NAME_MAX]; float value; } set_param;
        // motor_idx: 0 = CẢ 4 cùng lúc, 1..4 = M1..M4 riêng lẻ (KHÔNG phải
        // 0-based cho trường hợp riêng lẻ) — xem CMD_TEST_MOTOR ở trên.
        // duty_pct: 0..100, clamp về TEST_MOTOR_MAX_DUTY_PCT (an toàn, thấp
        // hơn nhiều so với 100%).
        struct { int32_t motor_idx; int32_t duty_pct; } test_motor;
        // delta_duty: cộng thẳng vào throttle hiện tại (thang duty
        // 0..MOTOR_SAFE_MAX_DUTY, KHÔNG phải %) — âm để giảm. Chỉ dùng cho
        // CMD_BENCH_THROTTLE_STEP (START/STOP không cần tham số).
        struct { int32_t delta_duty; } bench_step;
        // offset_duty: giá trị TUYỆT ĐỐI (không cộng dồn) cộng vào throttle
        // bench trong lúc giữ phím. 0 = nhả. Xem CMD_BENCH_THROTTLE_OFFSET.
        struct { int32_t offset_duty; } bench_offset;

        // roll_deg/pitch_deg: góc mục tiêu TUYỆT ĐỐI (chưa cộng trim — trim
        // cộng ở flight_core.c). yaw_rate_dps: tốc độ quay mục tiêu.
        struct { float roll_deg; float pitch_deg; float yaw_rate_dps; } set_attitude;
        struct { float roll_deg; float pitch_deg; } set_trim;
        struct { pid_loop_t loop; pid_axis_t axis; float kp, ki, kd, ilimit, outlimit; } set_pid;
        struct { float kp; float ki; } set_mahony;
        struct { float alt_kp, vz_kp, vz_ki, vz_ilimit; } set_alt_tune;
        // prime_duty/prime_ms = pha PRIME (motor quay deu, KHONG chay Z/Vz PID);
        // max_climb_ms = toc do TRUOT cua target + tran |vz_target|. Xem
        // takeoff_land.h. hover la feedforward THO cua vong Vz (alt_hold_tune_t).
        struct { float hover; int32_t prime_duty; int32_t prime_ms;
                 float max_climb_ms; } set_tko_tune;
        struct { float descent_vz, flare_alt_m, flare_vz, touchdown_alt_m; } set_land_tune;
        // Trường tên khớp 1-1 commander_config_t (commander.h) — xem @CMDR SET.
        struct { float alt_min_m, alt_max_m, battery_floor_v, hard_tilt_deg;
                 int32_t heartbeat_timeout_ms, motor_sat_hard_ms; } set_commander_cfg;

        // rol/pit/yaw/thr: -100..100 (clamp trong flight_core.c). Xem CMD_CONTROL.
        struct { float rol; float pit; float yaw; float thr; } control;
        struct { int32_t mode; } set_flightmode;   // 0=headless(STUB) 1=head
        // write_sign_reg: 0/1 — có ghi REG 0x29=0x06 trong bài test hay không.
        // read_loop_ms: thời lượng vòng đọc 20ms (0 = bỏ qua). Xem CMD_MAG_SELFTEST.
        struct { int32_t write_sign_reg; int32_t read_loop_ms; } mag_selftest;
        // CMD_CALIB_GYRO/CMD_CALIB_ACCEL_FACE/CMD_CALIB_ACCEL_RESET/
        // CMD_CALIB_MAG_START/CMD_CALIB_MAG_STOP/CMD_CALIB_ERASE — không cần
        // tham số, chỉ dùng cmd.type.
    } as;
} command_t;

// Độ sâu command queue — nhỏ có chủ đích: Python là nguồn lệnh chậm (con
// người/script điều khiển ở Hz thấp), không cần buffer lớn. Đầy queue ->
// command_queue_push() trả false, Python wrapper phải coi là lỗi (không rơi
// lệnh âm thầm).
#define COMMAND_QUEUE_DEPTH 8

#ifdef __cplusplus
}
#endif
