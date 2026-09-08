// Telemetry snapshot — struct LÊN duy nhất mà tầng C phơi cho MicroPython đọc.
// Đây là MỘT trong 2 kênh biên giới (kênh còn lại là command_queue phía
// micropython_module/fc). CHỈ stabilize task (flight_core.c) được GHI struct
// này; mọi bên khác (fc_bridge.c phía MicroPython) chỉ ĐỌC qua telemetry_read()
// (snapshot có khóa nhẹ — xem flight_core.h) — 1 WRITER, không race.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "flight_core/commander.h"
#include "flight_core/flight_state_machine.h"

#ifdef __cplusplus
extern "C" {
#endif

// arm_reject_t — vì sao lệnh ARM gần nhất bị từ chối. Thứ tự KHỚP thứ tự kiểm
// trong prearm_check()/CMD_ARM (flight_core.c) và ghi lý do ĐẦU TIÊN thất bại.
//
// KHÔNG gộp với fault_class_t của Commander: đó là fault RUNTIME khi đã armed,
// còn cái này là "vì sao chưa vào được ARMED" — hai câu hỏi khác nhau.
// GUI map sang text (xem ARM_REJECT_NAMES trong tools/uav_udp_console.py).
typedef enum {
    ARM_REJECT_NONE = 0,
    ARM_REJECT_UNCALIBRATED,        // thiếu accel/mag hợp lệ trong NVS
    ARM_REJECT_STATE,               // FSM KHÔNG ở DISARMED (trước đây IM LẶNG HOÀN TOÀN)
    ARM_REJECT_ATTITUDE_INVALID,    // Mahony chưa hợp lệ
    ARM_REJECT_TILT,                // nghiêng vượt FSM_ARM_MAX_TILT_DEG
    ARM_REJECT_IMU,                 // IMU stale/không khoẻ
    ARM_REJECT_GYRO_CALIB,
    ARM_REJECT_ACCEL_CALIB,
    ARM_REJECT_BATTERY_SAMPLE,      // không đọc được điện áp pin
    ARM_REJECT_BATTERY_LOW,         // dưới sàn
    ARM_REJECT_BARO_NOT_READY,      // chưa có mốc 0m
    ARM_REJECT_BARO_UNHEALTHY,      // std áp suất nền quá lớn lúc calib
    ARM_REJECT_ALT_EST_INVALID,
    ARM_REJECT_LOOP_UNHEALTHY,      // vòng điều khiển trượt hạn liên tục
    ARM_REJECT_BUS_LEASE,           // không mượn được I2C để calib baro lúc ARM
    ARM_REJECT_BARO_CALIB_FAILED,   // calib baro lúc ARM thất bại
    // Heartbeat của nguồn điều khiển NGOÀI đã cũ hơn heartbeat_timeout_ms NGAY
    // TẠI LÚC BẤM ARM. Không chặn ở đây thì ARM "thành công" rồi Commander
    // SOFT-fault ngay tick sau -> tự DISARM sau vài chục ms, và người dùng chỉ
    // thấy "tự nhiên disarm" chứ không biết lý do. Thêm mã riêng để câu trả lời
    // hiện ra ĐÚNG LÚC BẤM. Xem thêm src/main.c (đã bỏ heartbeat tự sinh).
    ARM_REJECT_HEARTBEAT,
    // Không latch được ga hover theo pin (hover_model.h). HAI nguyên nhân, tách
    // riêng vì cách xử lý KHÁC HẲN nhau:
    //   NO_SAMPLE : chưa đủ HOVER_MODEL_MIN_SAMPLES mẫu vbat hợp lệ trong vòng
    //               đệm -> chờ vài trăm ms rồi ARM lại là xong (ADC pin chạy
    //               10Hz). Nếu KHÔNG tự hết thì ADC pin đang hỏng thật.
    //   VOLT_LOW  : vbat dưới HOVER_MODEL_MIN_LATCH_V (3.4V) -> SẠC PIN. Khác
    //               ARM_REJECT_BATTERY_LOW (sàn của Commander): ngưỡng này là
    //               biên VÙNG HỢP LỆ CỦA MODEL, không phải mức pin an toàn.
    //               Dưới 3.4V model ngoại suy ra ga vượt trần collective, tức
    //               hover_ff bị clamp thấp hơn hover thật -> quay lại đúng cái
    //               hố "drone nằm ì" mà latch sinh ra để lấp.
    ARM_REJECT_HOVER_LATCH_NO_SAMPLE,
    ARM_REJECT_HOVER_LATCH_VOLT_LOW,
    // ---- Thêm cùng startup gyro calibration (xem tuning.h mục 8b) ----
    ARM_REJECT_IMU_CFG,             // read-back GYRO/ACCEL_CONFIG không khớp -> scale KHÔNG tin được
    ARM_REJECT_GYRO_BIAS_LARGE,     // gyro corrected khi đứng yên vẫn lệch > PREARM_GYRO_MAX_MEAN_DPS
    ARM_REJECT_GYRO_NOT_STATIONARY, // chưa có cửa sổ stationary corrected-gyro đủ dài ngay trước ARM
} arm_reject_t;

// takeoff_reject_t — vì sao lệnh TAKEOFF gần nhất KHÔNG khởi động được chuỗi
// cất cánh. CÙNG một loại lỗi với arm_reject_t và ra đời vì cùng một lý do:
// mọi điều kiện của CMD_TAKEOFF nằm trong apply_command() và trước đây chỉ báo
// bằng ESP_LOGW — thứ KHÔNG bao giờ tới được GUI qua UDP. Tệ hơn nữa,
// command_parser.c trả lời "TAKEOFF started" dựa trên DỰ ĐOÁN chỉ xét state
// FSM, nên GUI hiện thành công trong khi firmware đã từ chối. Nhìn từ ngoài:
// "bấm TAKEOFF không có gì xảy ra, không rõ vì sao".
//
// KHÁC takeoff_abort_reason_t (takeoff_land.h): cái kia là "đã cất cánh rồi
// nhưng phải huỷ giữa chừng"; cái này là "chưa từng bắt đầu".
typedef enum {
    TAKEOFF_REJECT_NONE = 0,
    TAKEOFF_REJECT_STATE,            // FSM không ở ARMED (fsm_on_takeoff_request)
    TAKEOFF_REJECT_NO_CORRECTION,    // floor ToF chua gom du/on dinh (wire value giu tuong thich)
    TAKEOFF_REJECT_ALT_EST_INVALID,  // alt_estimator state không hữu hạn
    // ĐÃ THỬ rồi GỠ: latch ga hover từng được chuyển sang CMD_TAKEOFF, nay đã
    // trả về CMD_ARM (đo được điện áp KHÔNG TẢI ở đó). Hai lý do từ chối tương
    // ứng quay lại ARM_REJECT_HOVER_LATCH_*. GIỮ CHỖ trong enum thay vì xoá:
    // xoá sẽ dịch số của mọi mã thêm về sau, mà GUI ánh xạ số -> chuỗi theo
    // đúng thứ tự này.
    TAKEOFF_REJECT_HOVER_LATCH_UNUSED,
} takeoff_reject_t;

typedef struct {
    fsm_state_t state;
    bool        armed;

    float roll_deg, pitch_deg, yaw_deg;
    float gyro_roll_dps, gyro_pitch_dps, gyro_yaw_dps;
    float accel_x_g, accel_y_g, accel_z_g;   // body-frame, sau scale driver (KHÔNG có LPF riêng)
    bool  attitude_valid;
    float acc_norm_g;      // |accel| — Mahony status, 1.0 = pure gravity
    bool  accel_used;      // false = mẫu accel bị Mahony gate (rung/non-gravity)
    float yaw_rel_deg;     // yaw - mốc lúc ARM gần nhất (xem s_yaw_ref_offset)

    float alt_m, vz_ms;
    bool  alt_valid;
    float alt_target_m;
    // vz_target TẦNG NGOÀI cascade alt_hold (xem alt_hold.h) — CHỈ có nghĩa
    // khi state HOLDING/FLYING VÀ hold đang engaged (0 lúc khác, kể cả
    // TAKING_OFF/LANDING — 2 pha đó tự ép vz_target riêng không qua field
    // này, xem takeoff_land.c).
    float alt_target_vz_ms;

    int   throttle_duty;
    int   m1, m2, m3, m4;

    // battery_v = điện áp pin ĐÃ QUA sanity check (xem battery_driver.h) —
    // 0.0f nghĩa là "KHÔNG có mẫu dùng được", commander/compensation bỏ qua.
    // Số THÔ (kể cả khi không hợp lệ) nằm ở khối battery debug bên dưới.
    float battery_v;
    // LUÔN = 1.0f. Bù throttle theo pin đã BỎ HẲN (xem tuning.h mục 10b +
    // flight_core.c bước 9b) — field giữ lại CHỈ để dòng STATUS không đổi
    // format, GUI/log cũ khỏi vỡ regex. Khác 1.0 = firmware cũ.
    float battery_comp;

    // ---- Battery debug (xem battery_driver.h) — cho phép chẩn đoán thang đo
    // TỪ XA: sai ở ADC (raw kẹp 0/4095?), ở calibration (bat_calibrated=0?),
    // hay ở chia áp (raw+mV đúng nhưng volt sai)? Trước đây chỉ có mỗi
    // battery_v nên không có cách nào biết. bat_voltage_raw_v CÓ GIÁ TRỊ kể
    // cả khi bat_valid=0 — đó chính là lúc cần nhìn nó nhất. ----
    int   bat_adc_raw;         // giá trị ADC thô (0..4095)
    int   bat_adc_mv;          // điện áp tại CHÂN ADC (mV), sau adc_cali nếu có
    float bat_divider_ratio;   // BATTERY_DIVIDER_RATIO đang biên dịch vào firmware
    float bat_voltage_raw_v;   // = mV/1000*ratio TRƯỚC sanity check (luôn có số)
    bool  bat_valid;           // qua sanity 1S + có ADC calibration thật
    bool  bat_calibrated;      // adc_cali scheme đang hoạt động
    int32_t bat_age_ms;        // tuổi mẫu pin gần nhất (ms, -1 = chưa từng có)

    // ---- Latch ga hover theo pin (hover_model.h) ----
    // CHỐT LÚC ARM, đóng băng suốt chuyến bay. Đây là hai số cần nhìn ĐẦU TIÊN
    // khi drone "nằm ì không nhấc": hover_latch_duty lệch nhiều so với hover
    // THẬT nghĩa là model sai (đổi motor/cánh/pin mà chưa đo lại 2 điểm gốc).
    // hover_latched=false -> đang chạy hằng số ALT_HOLD_HOVER_NOMINAL như cũ
    // (chưa ARM lần nào, hoặc HOVER_LATCH_ENABLED=0).
    float hover_latch_v;       // vbat trung vị lúc latch (V, KHÔNG TẢI)
    float hover_latch_duty;    // hover_ff suy ra từ model
    bool  hover_latched;

    fault_class_t last_fault;

    // motor_kill_latched — kill latch ĐANG đóng (xem flight_core.h
    // flight_core_kill_now()). true = motor bị cắt cứng, KHÔNG đường code nào
    // ghi được duty khác 0; chỉ CMD_ARM tường minh mới hạ. KHÁC !armed: có thể
    // DISARMED mà latch = false (vừa boot, chưa từng kill) — latch nói "đã có
    // sự kiện an toàn xảy ra và cần người xác nhận", không chỉ "hiện không bay".
    bool motor_kill_latched;

    // ---- Control loop health (xem stabilize_task bước 0/0b) ----
    // loop_dt_us      : dt THỰC của tick vừa rồi (us) — đây CHÍNH LÀ giá trị
    //                    đưa vào mọi bộ tích phân, không phải hằng số 1/250s.
    // loop_max_us     : dt lớn nhất từ lúc boot (không tự reset).
    // deadline_miss_count : số tick trượt quá CONTROL_DEADLINE_FACTOR * chu kỳ.
    // Vòng chậm dần là dấu hiệu SỚM của bus I2C treo/CPU bị chiếm — phải nhìn
    // thấy được từ ground station trước khi nó thành mất kiểm soát.
    int32_t  loop_dt_us;
    int32_t  loop_max_us;
    uint32_t deadline_miss_count;
    // ---- CHAN DOAN NGUON NHIP VONG DIEU KHIEN ----
    // Log bay cho thay DT dao dong 4.0 -> 11.6ms va DLM tich 20000+ tick, nhung
    // KHONG co cach nao biet TAI SAO: dt do giua hai lan BAT DAU tick nen no gop
    // ca "cho cam bien" lan "xu ly lau". Ba so duoi day tach chung ra.
    //
    // loop_wake_by_hub  = so vong duoc sensor_hub danh thuc (duong NHANH, dung).
    // loop_wake_timeout = so vong phai roi ve dong ho FreeRTOS vi cho qua han.
    //   Ty le timeout/wake cao = hub khong publish kip -> nghi I2C/ToF chiem bus.
    // loop_busy_us      = thoi gian XU LY cua tick truoc (khong ke phan cho).
    //   Gan 4000us = CPU that su khong du. Nho ma dt van lon = dang CHO, khong
    //   phai dang ban -> nguyen nhan nam o nguon nhip chu khong o khoi luong tinh.
    uint32_t loop_wake_by_hub;
    uint32_t loop_wake_timeout;
    uint32_t loop_busy_us;

    // ================= Vì sao lệnh ARM gần nhất bị TỪ CHỐI =================
    // VẤN ĐỀ NÓ SINH RA ĐỂ GIẢI QUYẾT: mọi lý do từ chối ARM trước đây CHỈ đi ra
    // bằng ESP_LOGW — tức là CHỈ thấy được trên console USB. Người dùng ngồi ở
    // GUI qua WiFi/UDP KHÔNG thấy gì cả: bấm ARM, nhận reply "ARMED" (reply đó
    // chỉ đoán theo attitude guard, xem command_parser.c case 'r'), rồi không có
    // gì xảy ra và không có lời giải thích nào. Tệ nhất là nhánh "FSM không ở
    // DISARMED" vốn KHÔNG log gì cả — lệnh biến mất hoàn toàn im lặng.
    //
    // Giờ lý do được publish ra telemetry (wire: ARMREJ=) nên GUI đọc được.
    // Ghi lý do ĐẦU TIÊN thất bại; console vẫn log ĐỦ mọi lý do như trước.
    // arm_reject_seq tăng mỗi lần có lệnh ARM bị từ chối -> GUI phân biệt được
    // "vẫn lý do cũ" với "vừa bấm lại và lại trượt".
    int32_t  arm_reject;       // arm_reject_t
    uint32_t arm_reject_seq;

    // takeoff_reject / takeoff_reject_seq — CÙNG cơ chế, cho lệnh TAKEOFF (wire:
    // TKOREJ= / TKORSEQ=). Xem takeoff_reject_t ở đầu file để biết vì sao cần.
    int32_t  takeoff_reject;   // takeoff_reject_t
    uint32_t takeoff_reject_seq;

    // ================= Chuỗi cất cánh (takeoff_land.h takeoff_phase_t) =======
    // ⚠ 3 field dưới đây có ngữ nghĩa KHÁC NHAU, KHÔNG được dùng lẫn:
    //   takeoff_phase        = pha NỘI BỘ của chuỗi
    //                          (0=IDLE 1=PRIME 2=CLIMB 3=HOLD 4=ABORT)
    //   takeoff_control_active = altitude dynamics/controller ĐANG chạy
    //                          (true từ CLIMB, KHÔNG phải từ PRIME)
    //   alt_airborne         = liftoff_flag (est_z vượt ground + delta)
    // takeoff_control_active=1 với alt_airborne=0 là trạng thái HỢP LỆ và mong
    // đợi — đó chính là cửa sổ controller đang nhấc drone lên.
    int32_t takeoff_phase;           // 0=IDLE 1=PRIME 2=CLIMB 3=HOLD 4=ABORT
    bool    takeoff_control_active;
    float   takeoff_target_alt_m;    // final_target TỪ LỆNH CMD_TAKEOFF (đã clamp geofence)
    float   takeoff_z_sp_m;          // target_z ĐANG TRƯỢT (slew-rate-limited)
    float   takeoff_vz_target_ms;    // output tầng Z -> đầu vào Vz-PID
    float   takeoff_base_thrust;     // hover_ff (feedforward THÔ, = ALT_HOLD_HOVER_NOMINAL)
    float   takeoff_alt_corr;        // throttle - hover_ff (= P + I của vòng Vz)

    // ---- Debug slew + học-hover (spec test B) ----
    // takeoff_vz_i_term là thứ QUAN TRỌNG NHẤT để soi: nó là phần vòng Vz TỰ
    // HỌC hover thật. Bắt đầu ÂM (prime_duty < hover_ff) rồi bò lên và hội tụ
    // = học đúng. Không hội tụ / bám trần = hover_ff sai quá xa hoặc drone
    // không đủ lực.
    float   takeoff_vz_i_term;
    float   takeoff_ground_alt_m;    // mốc est_z lúc rời PRIME
    bool    takeoff_liftoff_flag;    // est_z > ground + delta (THÔNG TIN, không gate)
    float   takeoff_tilt_deg;
    float   takeoff_elapsed_s;

    // ---- Abort ----
    // 0=NONE 1=TIMEOUT 2=TOF_LOST 3=TILT (takeoff_abort_reason_t).
    int32_t takeoff_abort_reason;
    bool    alt_pid_saturated;       // cascade Vz đang bị kẹp trần/sàn duty

    // ---- Mixer saturation (xem attitude_control.h mixer_status_t) ----
    bool  mixer_saturated;
    bool  mixer_roll_limited, mixer_pitch_limited, mixer_yaw_limited;
    float mixer_headroom_duty;   // dải duty còn thừa cho attitude
    int   base_throttle_duty;    // collective TRƯỚC bù pin (throttle_duty là SAU)

    // ---- Tuổi mẫu cảm biến (ms) — nguồn sự thật cho "stale", xem sensor_hub.h ----
    int32_t imu_age_ms, mag_age_ms, baro_age_ms;
    // Tuoi mau ToF (ms, -1 = CHUA TUNG co mau). Day la cach DUY NHAT tu xa de
    // biet sensor_hub co that su dang lay mau ToF khong — tof_range_m=0 mot
    // minh khong phan biet duoc "sensor doc 0" voi "chua bao gio duoc doc".
    int32_t tof_age_ms;
    // Tuoi cua lan cuoi CHIP DO DUOC (ms), khac han tof_age_ms.
    //   tof_age_ms    = bao lau roi chua co mau HOP LE  -> 'co so de bay khong'
    //   tof_alive_ms  = bao lau roi chip khong do duoc  -> 'con cam bien khong'
    // Nam sat san (0mm, duoi tam mu) hay ngoai tam: tof_age_ms tang vo han
    // NHUNG tof_alive_ms van nho. Chi khi chip chet / bus dut thi ca hai cung
    // tang. Khong co so nay thi GUI khong the phan biet 'khong co gi de do'
    // voi 'mat cam bien' -- va no da bao do nham dung o ca hai ca.
    // -1 = chua tung doc duoc mau nao.
    int32_t tof_alive_ms;

    // ---- CHAN DOAN ToF THO (TOFRS / TOFSIG / TOFAMB) ----
    // Ba so nay tra loi cau hoi "vi sao mau bi loai", thu ma TOFA/TOFR khong
    // noi duoc. Chung den THANG tu chip, TRUOC moi tang loc cua firmware.
    //
    // TOFRS = range_status THO (5 bit), TRUOC khi tra bang L1X_STATUS_MAP.
    //         Driver chi chap nhan mau khi map(TOFRS) == 0. Ma tho hay gap:
    //           0  = RANGE_VALID          (tot)
    //           1  = SIGMA_FAIL           (nhieu do lech chuan qua lon)
    //           2  = SIGNAL_FAIL          (be mat hap thu / qua xa / khong du
    //                                      anh phan xa)
    //           4  = OUTOFBOUNDS_FAIL     (ngoai tam)
    //           7  = WRAP_TARGET_FAIL     (bong ma do phan xa vong lai)
    //           5  = HARDWARE_FAIL
    //         ⚠ 255 = raw_status >= 24 (ngoai bang) — gan nhu chac chan la loi
    //         doc I2C, KHONG phai chip tu bao.
    //
    // TOFSIG = peak signal count rate (MCPS x 100). Cang cao cang tin cay.
    //          Sat 0 nghia la KHONG CO anh phan xa quay ve — be mat den/hap thu,
    //          hoac khong co gi trong tam nhin.
    // TOFAMB = ambient count rate (MCPS x 100). Cao = anh sang nen manh (nang,
    //          den halogen) dang lan at tin hieu.
    //
    // CACH DOC (ba kich ban thuong gap):
    //   TOFRS=2, TOFSIG thap, TOFAMB thap  -> be mat khong phan xa du
    //   TOFRS=2, TOFSIG thap, TOFAMB CAO   -> anh sang nen at tin hieu
    //   TOFRS=1, TOFSIG kha    -> co tin hieu nhung nhieu (rung / cua so ban)
    //   TOFRS=0 ma van TOFA=0  -> loi o TANG TREN, khong phai chip
    uint8_t  tof_range_status_raw;
    uint16_t tof_signal_mcps;
    uint16_t tof_ambient_mcps;
    bool    imu_healthy, mag_healthy, baro_healthy_hub;

    // heading_degraded — yaw đang CHỈ dựa vào tích phân gyro (mag mất/chưa
    // calib/bị gate). roll/pitch VẪN hợp lệ. KHÔNG phải lỗi IMU — xem
    // flight_core.c bước 1c/2.
    bool heading_degraded;

    // att_integral_active — I-term attitude ĐANG được cộng dồn hay không
    // (= !hold_integral_freeze ở flight_core loop). Là AND của HAI gate độc
    // lập: (a) FSM cho phép — trong TAKING_OFF phải prime_done && airborne,
    // xem takeoff_land.h; (b) throttle >= ATT_I_ENABLE_THROTTLE_DUTY.
    // KHÁC takeoff_prime_done (thuần thời gian prime) — trước đây field đó bị
    // dùng nhầm làm nguồn cho KI= trong STATUS line.
    bool att_integral_active;

    // ---- Trạng thái driver + cảm biến phụ (ground-station diagnostics) ----
    bool  imu_ok_driver, mag_ok_driver, baro_ok_driver, tof_ok_driver, battery_ok_driver;
    float tof_range_m;
    bool  tof_valid;
    float baro_alt_m, baro_pressure_pa;   // baro_alt_m = RAW (trước median-of-3+LPF), xem alt_estimator.h
    uint32_t sensor_err_count;   // cộng dồn lỗi đọc I2C (mọi driver, từ boot) — best-effort

    // ---- Altitude estimator debug (xem alt_estimator.h — accel-primary,
    // baro = anchor chậm, KHÔNG dùng ToF) ----
    float    baro_filtered_alt_m;    // SAU median-of-3 + LPF, giá trị thực sự dùng để tính innovation
    float    baro_innovation_m;      // baro_filtered - alt_m lần gần nhất (dấu hiệu spike nếu lớn)
    float    baro_dt_s;              // dt THẬT giữa 2 mẫu baro liên tiếp (giây, xem beta formula alt_estimator.h)
    uint32_t baro_accept_count;      // mẫu ĐÃ DÙNG sửa Z/Vz (gate thường HOẶC reacquire)
    uint32_t baro_reject_count;      // mẫu ngoài gate bị bỏ CỘNG DỒN (nghi spike/propwash/reacquire chưa đủ lâu)
    // ---- Bất đồng dai dẳng + REACQUIRE (xem alt_estimator.h "CORRECT" mục
    // (c)/(d) — fix deadlock "trôi -> reject -> trôi thêm -> reject mãi") ----
    uint32_t baro_reject_consecutive; // reject LIÊN TỤC hiện tại, reset về 0 mỗi lần ACCEPT — đứng im lâu dù baro_healthy=1 là dấu hiệu bất thường
    bool     baro_reacquire_active;   // đang dùng ALT_EST_BARO_REACQUIRE_ALPHA/BETA để kéo estimator về lại baro
    // alt_airborne — ĐỊNH NGHĨA DUY NHẤT của "airborne": liftoff đã được
    // detector CONFIRM (takeoff_land.h takeoff_airborne()), HOẶC đang ở state
    // hậu-cất-cánh (HOLDING/FLYING/LANDING). KHÔNG module nào được tự set nó từ
    // một accel spike riêng lẻ.
    bool     alt_airborne;
    // alt_liftoff_candidate — estimator ĐANG tích phân Z/Vz nhưng CHƯA fuse
    // baro (cửa sổ CONTROL_ACTIVE trước khi confirm). =1 với alt_airborne=0 là
    // hợp lệ và mong đợi.
    bool     alt_liftoff_candidate;
    // alt_degraded — estimator VẪN hợp lệ nhưng đã quá lâu không có correction
    // nào (ToF lẫn baro). Z đang dead-reckon thuần accel, sai số tăng BẬC HAI.
    // KHÁC alt_valid: valid=1 degraded=1 = "số hợp lệ nhưng đừng tin lâu".
    bool     alt_degraded;
    int32_t  alt_no_correction_ms;   // tuổi correction gần nhất (ms)

    // alt_source — NGUỒN nào đang thực sự giữ Z (alt_source_t: 0=NONE 1=TOF
    // 2=BARO 3=TOF+BARO). Trả lời câu mà alt_degraded KHÔNG trả lời được:
    // degraded=0 chỉ nói "có correction", không nói của ai. Với một ToF chập
    // chờn thì "đang chạy bằng BARO" là thông tin quan trọng nhất trên màn hình
    // — nó nghĩa là bạn không còn tham chiếu chính xác tới mặt sàn nữa.
    uint8_t  alt_source;

    // tof_built_in — ToF CO trong firmware nay khong (cờ biên dịch
    // SENSOR_TOF_ENABLED + cfg.tof_enabled), KHÁC HẲN tof_ok_driver.
    //
    // VÌ SAO cần một cờ riêng: tof_ok_driver=0 gộp HAI chuyện khác hẳn nhau —
    // "ToF bị TẮT có chủ đích" và "ToF CÓ nhưng init hỏng". Ground-station chỉ
    // nhìn tof_ok_driver/tof_age_ms thì không phân biệt được, nên nó báo ĐỎ
    // "ToF LOI, chay tof_test" ngay cả với một cấu hình baro-only hoàn toàn
    // hợp lệ — làm người dùng tưởng hệ thống BẮT BUỘC phải có ToF.
    bool     tof_built_in;

    // ---- LUONG SUA THUC TE cua lan correction gan nhat ----
    // Khac innovation: innovation = hai ben BAT DONG bao nhieu; may so nay =
    // ta DA SUA bao nhieu. Tach ra moi tune duoc gain:
    //   innovation lon + correction nho  -> gain thap (dung thiet ke)
    //   innovation nho + correction lon  -> gain qua tay
    float    tof_corr_z_m, tof_corr_vz_ms;
    float    baro_corr_z_m, baro_corr_vz_ms;

    // ---- Hoc bias accel TU RESIDUAL do cao (duong thu hai, chay LUC BAY) ----
    // bias_residual_m  : residual da loc dang lai bias (m)
    // bias_adapt_count : so tick THAT SU adapt. Bench: phai TANG DEU khi dang
    //                    bay bang. Dung yen suot chuyen bay = mot trong cac
    //                    cong gate dang dong (nghieng/gia toc/residual qua lon).
    float    bias_residual_m;
    uint32_t bias_adapt_count;

    // ---- ToF hướng xuống: CORRECTION CÓ ĐIỀU KIỆN THEO BỀ MẶT ----
    // tof_range_m/tof_valid ở khối driver phía trên là RAW. Khối này là kết quả
    // XỬ LÝ của estimator — nhìn nó mới biết ToF đang được dùng hay bị chặn.
    //
    // ⚠ tof_surface_state=OTHER với tof_correction_enabled=0 là trạng thái
    // HỢP LỆ và MONG ĐỢI khi bay qua bàn/ghế — KHÔNG phải lỗi cảm biến.
    float    tof_vertical_m;       // range đã bù nghiêng về phương thẳng đứng
    float    tof_innovation_m;     // tof_vertical - range dự đoán tới SÀN ĐÃ KHOÁ
                                    // âm lớn = có bề mặt CAO hơn sàn ở dưới
    float    tof_surface_z_m;      // world-Z của bề mặt ĐANG nhìn thấy
    int32_t  tof_surface_state;    // alt_est_tof_surface_t: 0=UNKNOWN 1=FLOOR 2=OTHER
    bool     tof_correction_enabled;  // ToF có ĐANG được sửa world-Z không
    float    tof_ground_range_m;   // range lúc UAV nằm trên sàn (chốt ở ARM)
    uint32_t tof_accept_count;
    uint32_t tof_reject_count;

    float    floor_plane_z_m;      // mặt sàn KHOÁ lúc cất cánh — KHÔNG đổi giữa bay
    float    landing_surface_z_m;  // bề mặt chọn khi CMD_LAND (KHÁC floor!)
    bool     landing_surface_valid;
    uint32_t baro_seq;                // sensor_health_t.seq của mẫu baro gần nhất — đếm mẫu THẬT của cảm biến
    bool     baro_fusion_initialized; // fusion đã có gốc toạ độ chung với baro (KHÁC "baro còn sống")
    float    az_corrected_ms2;        // az_lpf_ms2 - accel_bias_ms2, SAU deadband (m/s²) — giá trị THẬT được tích phân
    float    vert_accel_ms2;         // = az_world_filtered — accel world-frame, đã trừ gravity + LPF (TRƯỚC deadband/bias), ĐƠN VỊ m/s²
    float    vert_accel_raw_ms2;     // az_world TRƯỚC LPF (debug — so sánh raw vs filtered, xem test G README), ĐƠN VỊ m/s²
    float    accel_bias_ms2;         // state "gamma" đã học (xem alt_estimator.h) — ĐƠN VỊ m/s² (KHÔNG phải g), nên nhỏ, ổn định theo thời gian
    // ĐƠN VỊ: m/s (VẬN TỐC) — tên cũ vz_accel_only_ms2 gợi ý m/s² (gia tốc)
    // là SAI, đã đổi. Khoá 0 khi chưa inertial-enabled (xem alt_estimator.h).
    float    vz_accel_only_ms;       // Vz tích phân THUẦN accel, baro KHÔNG BAO GIỜ chạm — debug/test E
    float    z_inertial_m;           // Z tích phân THUẦN accel (cặp với vz_accel_only_ms) — debug/test E

    // ---- Baro ground calibration health (xem baro_driver.h) ----
    bool     baro_calibrated;        // đã calibrate_ground() thành công >= 1 lần
    bool     baro_healthy;           // std-dev lúc calib <= BARO_GROUND_NOISE_STD_MAX_PA
    float    baro_ground_noise_std_pa;

    // ---- Nguồn nhịp vòng điều khiển (xem imu_driver_enable_data_ready_int()) ----
    // imu_int_active=1 -> stabilize_task đang chạy theo data-ready INT của
    // MPU6050. =0 -> đang dùng đồng hồ FreeRTOS (chưa bật, hoặc ĐÃ RƠI VỀ vì
    // ngắt ngừng đến — phân biệt bằng imu_int_timeout_count > 0).
    // isr_count nên xấp xỉ wake_count: chênh nhiều = ngắt đến nhưng task không
    // kịp tiêu thụ (quá tải), hoặc có cạnh giả (dây INT thả nổi).
    bool     imu_int_active;
    uint32_t imu_int_isr_count;      // số lần ISR chạy (đếm trong imu_driver.c)
    uint32_t imu_int_wake_count;     // số vòng lặp được ngắt đánh thức
    uint32_t imu_int_timeout_count;  // số lần chờ ngắt quá hạn (>0 = dây/chip có vấn đề)
    // Số tick stabilize_task chạy mà KHÔNG có mẫu IMU mới (seq không đổi hoặc
    // mẫu đã quá hạn). Ở nhịp bình thường phải ĐỨNG YÊN: hub publish đúng 1 mẫu
    // rồi mới notify. Số này TĂNG ĐỀU = vòng bay đang quay nhanh hơn nguồn mẫu
    // (hub rơi về polling, INT giả, hoặc bus chậm) — các bước tích phân bị bỏ
    // qua ở đúng những tick đó, xem imu_updated trong flight_core.c.
    uint32_t imu_no_new_sample_count;

    // ---- Vòng lặp + watchdog (xem flight_core.c) ----
    float   loop_dt_ms;          // dt thực đo giữa 2 tick stabilize_task (jitter/drift)
    int32_t last_cmd_age_ms;     // now - lần CMD_SET_ATTITUDE gần nhất (watchdog setpoint)
    int32_t heartbeat_age_ms;    // now - commander_heartbeat() gần nhất (watchdog Commander)

    // ---- Phase con (để GUI hiện TKO=.. LAND=.. như UAV-Mini) ----
    bool takeoff_prime_done;     // true khi hết pha PRIME (cascade cầm lái) — KHÔNG phải "đã handoff"
    int  landing_phase;          // land_phase_t (LAND_IDLE..LAND_BLIND) — xem takeoff_land.h

    // ---- Calibration (xem calibration.h + flight_core.c CMD_CALIB_*) ----
    bool     uncalibrated;           // true -> CMD_ARM bị từ chối (thiếu accel/mag hợp lệ trong NVS)
    bool     calib_gyro_active;      // đang đo bias tĩnh (CMD_CALIB_GYRO)
    bool     calib_accel_capturing;  // đang lấy mẫu 1 mặt (CMD_CALIB_ACCEL_FACE, ~0.5s)
    int      calib_accel_faces_done; // 0..CALIB_ACCEL_FACES_NEEDED
    bool     calib_mag_active;       // đang thu mẫu mag (giữa CMD_CALIB_MAG_START..STOP)
    uint32_t calib_mag_sample_count; // số mẫu đã thu trong phiên mag hiện tại

    // Cờ hợp lệ TỪNG loại calib đang nạp trong RAM (nguồn: NVS lúc boot, hoặc
    // phiên calib thành công gần nhất). uncalibrated ở trên là tổng hợp của
    // accel+mag; 3 cờ này cho biết CHÍNH XÁC thiếu cái nào — quan trọng vì 1
    // phiên calib THẤT BẠI không xóa calib CŨ, nên "mag_valid=1" hoàn toàn có
    // thể tồn tại song song với 1 phiên calib mag vừa hỏng.
    bool     calib_gyro_valid, calib_accel_valid, calib_mag_valid;

    // ================= GYRO BIAS DIAGNOSTICS (xem tuning.h muc 8b) =================
    // Ba bo so nay ton tai de tra loi DUT KHOAT cau hoi "bias co thuc su duoc
    // khu khong", thay vi phai suy doan tu hanh vi bay. Quan he BAT BUOC:
    //
    //     gyro_corr_dps == gyro_raw_dps - gyro_bias_dps
    //
    // Drone dung yen, calib dat  => gyro_raw ~ gyro_bias, gyro_corr ~ 0.
    // Neu gyro_corr KHONG ~0 khi dung yen thi calib CHUA an, va nhin vao ba
    // hang so nay la biet ngay lech o dau — khong con phai doan.
    //
    // Don vi: dps, SENSOR FRAME (remap sensor->body hien la identity).
    // Publish o nhip STATUS binh thuong, KHONG phai 250Hz.
    vec3f_t  gyro_raw_dps;      // GRAWX/Y/Z  — truoc khi tru bias
    vec3f_t  gyro_bias_dps;     // GBIASX/Y/Z — bias dang ap dung
    vec3f_t  gyro_corr_dps;     // GCORRX/Y/Z — sau khi tru bias (= thu Mahony/PID dung)
    vec3f_t  gyro_std_dps;      // alias tương thích: corrected std validation gần nhất
    vec3f_t  gyro_raw_mean_dps; // mean RAW của cửa sổ COLLECT gần nhất
    vec3f_t  gyro_corr_mean_dps;// mean raw-bias của cửa sổ VALIDATE độc lập
    vec3f_t  gyro_raw_std_dps;  // GSTDRAWX/Y/Z
    vec3f_t  gyro_corr_std_dps; // GSTDCORRX/Y/Z

    vec3f_t  accel_raw_g;       // ARAWX/Y/Z  — truoc bias/scale
    vec3f_t  accel_corr_g;      // ACORRX/Y/Z — sau bias/scale (thu Mahony dung)
    float    accel_norm_g;      // ANORM — |accel_corr|, nen ~1.0 khi dung yen

    // ---- Trang thai startup gyro calibration ----
    // So mau KHONG dat tieu chi dung yen trong cua so gan nhat. Thay cho
    // gyro_cal_attempt cu (khong con retry): khi calib truot vi "qua nhieu
    // mau nghi dong", day la con so noi duoc TRUOT BAO NHIEU, chu khong chi
    // "co truot". So nho ma van fail = nguong qua chat; so lon = drone that
    // su bi dong/rung.
    int      gyro_cal_bad_samples;
    int      gyro_cal_state;        // 0 IDLE,1 SETTLING,2 COLLECT,3 VALIDATE,4 PASS,5 FAIL
    int      gyro_cal_fail;         // gyro_cal_fail_t; 0=NONE
    float    gyro_cal_temp_c;       // IMUTEMP luc calib — de so voi imu_temp_c hien tai (muc 9)
    bool     gyro_cal_temp_warn;    // |imu_temp_c - gyro_cal_temp_c| > CALIB_TEMP_WARN_DELTA_C
    bool     gyro_valid_from_nvs;   // NVS co bias cu (tham khao, KHONG dung de bay)

    // ---- Cau hinh MPU6050 doc lai tu chip (muc 2) ----
    // Scale THAT SU dang dung de convert, suy ra tu FS_SEL/AFS_SEL doc lai —
    // khong phai hang so bien dich. imu_cfg_valid=false -> ARM bi chan.
    bool     imu_cfg_valid;
    uint8_t  imu_gyro_config, imu_accel_config;   // GYROFS / ACCELFS (gia tri thanh ghi tho)
    uint8_t  imu_fs_sel, imu_afs_sel;
    float    imu_gyro_lsb_per_dps, imu_accel_lsb_per_g;

    // ---- Mag TRONG Mahony (khác hẳn mag_ok_driver ở trên!) ----
    // mag_ok_driver = "đọc I2C được". Mấy trường dưới = "vector đó có THẬT SỰ
    // được dùng để sửa yaw hay không" — hai chuyện hoàn toàn khác nhau:
    // mag chưa calib thì đọc tốt vẫn KHÔNG được dùng (hard-iron offset làm sai
    // hướng vector, dùng vào chỉ làm yaw tệ hơn gyro-only).
    // mag_used_count đứng yên trong khi mag_ok_driver=1 => mag đang bị chặn:
    // hoặc chưa calib (xem calib_mag_valid), hoặc lệch trục nên mag_error vượt
    // MAHONY_DEFAULT_MAG_ERROR_GATE (khi đó mag_rejected_count tăng nhanh).
    bool     mag_used;             // tick GẦN NHẤT có dùng mag để sửa yaw không
    bool     mag_rejected;         // tick gần nhất mag bị gate loại
    bool     mag_reference_valid;  // đã chốt được mốc từ trường world-frame chưa
    float    mag_norm;             // |mag| sau calib — nên GẦN NHƯ KHÔNG ĐỔI khi xoay
    float    mag_error_norm;       // |cross(mag_đo, mag_dự_đoán)|, so với gate 0.90
    uint32_t mag_used_count, mag_rejected_count;   // cộng dồn từ boot

    // ---- Mag calibration live values (chỉ có nghĩa khi calib_mag_active) ----
    int      calib_mag_seconds_left;    // còn bao nhiêu giây trong cửa sổ 60s
    float    calib_mag_range_x, calib_mag_range_y, calib_mag_range_z;  // max-min từng trục
                                        // (độ phủ; 3 số nên GẦN BẰNG NHAU, trục nào ~0 = chưa xoay)
    uint32_t calib_mag_read_err_count;  // tick lỗi đọc I2C trong phiên (>0 = vấn đề dây/bus)
    uint32_t calib_mag_notready_count;  // tick đọc OK nhưng chưa có mẫu mới (DRDY=0/OVFL/all-zero)

    // ---- Calibration quality debug (xem flight_core.c step 1b) ----
    float imu_temp_c;                // nhiệt độ MPU6050 mẫu gần nhất (°C) — xem imu_driver.h
    float gyro_calib_std_x_dps, gyro_calib_std_y_dps, gyro_calib_std_z_dps;   // std-dev lần calib gyro GẦN NHẤT
    float accel_calib_residual_g;    // residual max lần calib accel GẦN NHẤT (0 nếu chưa từng calib)

    // ---- IMU+ToF altitude refactor (append-only wire fields) ----
    float tof_z_m, tof_vz_ms;
    bool  tof_vz_valid, tof_fusable;
    int   tof_track_state;
    bool  floor_locked, floor_ready;
    uint32_t floor_sample_count;
    float floor_std_m;
    bool  baro_used_by_flight_control; // invariant: false
    float az_body_z_g, az_earth_raw_ms2, az_after_gravity_ms2, az_after_bias_ms2;
    float alt_request_m;
    float z_error_m, vz_error_ms;
    float vz_p_term, vz_i_term, vz_d_term, vz_output_duty;
    float hover_throttle_duty, throttle_correction_duty;

    int64_t stamp_us;
    // ---- TERRAIN (alt_estimator.h mục TERRAIN OFFSET) ----
    // TOFF  = terrain_off_m: độ cao BỀ MẶT đang nhìn so với SÀN cất cánh. 0 khi
    //         đang trên đúng mặt sàn; 0.75 khi đang trên mặt bàn cao 0.75m.
    // TPEND = đang NGHI có bậc, chưa xác nhận. Trong lúc này I-term freeze và
    //         alt COAST bằng accel — TPEND=1 kéo dài (>0.5s) nghĩa là mép bàn
    //         làm range nhảy loạn, không confirm nổi.
    // TCMT  = số lần đã COMMIT offset (tăng 1 mỗi lần qua một bậc).
    // TRES  = residual mẫu gần nhất (m) — số dùng để tune TERR_JUMP_THRESH_M.
    // CLR   = khoảng hở THẬT tới bề mặt dưới bụng (AGL). Đây là con số quyết
    //         định drone có đâm vào bàn hay không, KHÔNG phải ALTm.
    // FRAME = 0 DATUM (giữ độ cao so với sàn) / 1 AGL (terrain following).
    // TGTS  = TARGET HIỆU DỤNG so với BỀ MẶT đang bay trên = TGT - TOFF.
    //         Ở DATUM, TGT là Z so với SÀN và giữ NGUYÊN cả chuyến; thứ thực
    //         sự đổi khi qua bậc địa hình là con số này:
    //             bay 1.5m, lên bàn 1.0m -> TGT=1.50 TOFF=1.00 TGTS=0.50
    //             rời bàn                 -> TGT=1.50 TOFF=0.00 TGTS=1.50
    //         Z tuyệt đối không đổi — chỉ khoảng cách tới mặt dưới bụng đổi.
    //         ⚠ CHỈ ĐỂ ĐỌC. Không có đường điều khiển nào dùng TGTS: cascade
    //         so TGT với alt_m (DATUM) và phép trừ offset đã nằm sẵn trong
    //         alt_m, nên cộng thêm ở đây sẽ tính hai lần.
    //         TGTS < 0 = target nằm DƯỚI mặt bàn (bàn cao hơn độ cao đang bay)
    //         -> B8 guard sẽ ép leo giữ TERR_MIN_CLEARANCE_M.
    // TSNAP = so lan alt_m bi EP ve so do that vi coast qua
    //         ALT_EST_COAST_SNAP_MS (400ms) ma ToF khong sua duoc.
    //         0 = binh thuong. Tang deu = ToF dang bi chan fuse lien tuc
    //         (terrain ket / mau bi vut) -> di tim nguyen nhan do.
    uint16_t coast_snap_count;
    float    alt_target_surface_m;
    float    terrain_off_m;
    // TPEND = dang NGHI co bac, chua xac nhan.
    // TCMT  = so lan da COMMIT offset.  TREJ = so lan bi TU CHOI (sanity fail).
    // TTMO  = so lan nghi ngo bi HUY vi het han TERR_PENDING_TIMEOUT_MS.
    //         ⚠ SO QUAN TRONG NHAT khi debug terrain: xem app_config.h.
    // TRES  = residual mau gan nhat (m) -- de tune TERR_JUMP_THRESH_M.
    bool     terr_pending;
    uint32_t terr_commit_count;
    uint16_t terr_reject_count;
    uint16_t terr_timeout_count;
    float    terr_residual_m;
    // TSTALE = terrain offset da mat tin cay (transition timeout). Khi =1:
    //          ToF KHONG duoc fuse vao Z tuyet doi nua, estimator song bang
    //          IMU bridge. Day la so can nhin dau tien khi thay FAULT sau khi
    //          bay qua vat the.
    // TCAND  = offset UNG VIEN dang cho xac nhan (khac TOFF = offset da nhan).
    // TCNT   = so mau lien tiep da khop ung vien (0..TERR_CONFIRM_N).
    // TSEEN  = so mau ToF da di qua khoi confirm ke tu khi vao pending.
    //          TSEEN dung yen trong khi TPEND=1 nghia la ToF khong cap mau.
    bool     terr_offset_stale;
    float    terr_cand_offset_m;
    uint8_t  terr_confirm_cnt;
    uint8_t  terr_samples_seen;
    bool     terrain_pending;
    uint32_t terrain_commits;
    float    terrain_residual_m;
    float    clearance_m;
    int32_t  alt_frame;
} telemetry_snapshot_t;

static inline void telemetry_snapshot_init(telemetry_snapshot_t *t) {
    telemetry_snapshot_t z = {0};
    *t = z;
    t->state = FSM_DISARMED;
}

#ifdef __cplusplus
}
#endif
