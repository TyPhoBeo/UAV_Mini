// flight_core.c — GLUE: sở hữu toàn bộ static state (1 drone / 1 instance),
// wiring các module (Mahony, AltEstimator, attitude cascade, alt_hold,
// takeoff/landing, FlightStateMachine, Commander) + driver, chạy stabilize
// task 250Hz pin core 1. PORT lại luồng của stabilizer_task (UAV-Mini
// flight_control.cpp) trên khung FlightStateMachine/Commander mới thay vì các
// cờ mode rời rạc như bản gốc.
#include "flight_core/flight_core.h"

#include <math.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "flight_core/alt_estimator.h"
#include "flight_core/alt_hold.h"
#include "flight_core/attitude_control.h"
#include "flight_core/calibration.h"
#include "flight_core/commander.h"
#include "flight_core/drivers/baro_driver.h"
#include "flight_core/drivers/battery_driver.h"
#include "flight_core/drivers/imu_driver.h"
#include "flight_core/drivers/mag_driver.h"
#include "flight_core/drivers/motor_driver.h"
#include "flight_core/drivers/tof_driver.h"
#include "flight_core/flight_state_machine.h"
#include "flight_core/hover_model.h"
#include "flight_core/mahony_filter.h"
#include "flight_core/sensor_hub.h"
#include "flight_core/takeoff_land.h"
#include "flight_core/tuning.h"
#include "flight_core/types.h"

static const char *TAG = "flight_core";

#define CONTROL_TASK_HZ            250
// ĐƠN VỊ: BYTE. xTaskCreate*() của ESP-IDF nhận stack theo BYTE (khác FreeRTOS
// vanilla vốn nhận theo WORD) — 4096 ở đây là 4KB, KHÔNG phải 16KB.
#define STABILIZE_TASK_STACK_BYTES 4096
// 23 = mức cao nhất firmware được dùng trên core 1 (24 = ipc1, xem bảng đầy đủ
// ở đầu sensor_hub.c). Vòng điều khiển là thứ ưu tiên số một trên core 1.
#define STABILIZE_TASK_PRIORITY    23
#define STABILIZE_TASK_CORE        1     // TÁCH KHỎI mọi task application (core 0)

// BẤT BIẾN KIẾN TRÚC canh lúc BIÊN DỊCH: vòng điều khiển PHẢI ưu tiên cao hơn
// sensor_hub. Đảo lại (bản cũ: hub 23 > stabilize 20) thì xTaskNotifyGive()
// của hub KHÔNG preempt được vòng bay — hub giữ CPU chạy tiếp MAG/BARO/ToF,
// và vòng bay phải xếp hàng sau các transaction I2C đó dù đã có đủ mẫu IMU.
// Hai hằng số nằm ở hai file khác nhau nên không có gì bắt chúng khớp ngoài
// dòng này — đổi một cái mà quên cái kia thì build ĐỨT ở đây thay vì lộ ra lúc
// đang bay.
// Preemption CHỈ có ý nghĩa trong cùng một core. Nếu hai task bay bị tách ra
// hai core thì assert priority ngay dưới đây trở thành vô nghĩa mà vẫn PASS —
// nên phải khoá luôn cả "cùng core", không chỉ "stabilize ở core 1".
_Static_assert(STABILIZE_TASK_CORE == SENSOR_HUB_TASK_CORE,
               "stabilize_task va sensor_hub PHAI cung core, neu khong thi priority khong con quyet dinh thu tu chay");

_Static_assert(STABILIZE_TASK_PRIORITY > SENSOR_HUB_TASK_PRIORITY,
               "stabilize_task PHAI uu tien CAO HON sensor_hub (notify phai PREEMPT hub)");
// stabilize KHÔNG được ngang/trên ipc1 (configMAX_PRIORITIES-1) — ipc1 nằm
// trên core 1 và `esp_ipc_call*` là đường đồng bộ giữa 2 core của IDF.
_Static_assert(STABILIZE_TASK_PRIORITY < configMAX_PRIORITIES - 1,
               "stabilize_task KHONG duoc ngang/tren IPC task (configMAX_PRIORITIES-1)");
// Cả hai PHẢI cùng core, nếu không thì lập luận "hub kẹt I2C = BLOCKED, vòng
// điều khiển vẫn chạy" mất nghĩa (khác core thì chúng vốn không tranh CPU) VÀ
// notify không còn preempt được (preemption chỉ xảy ra trong cùng một core).
_Static_assert(STABILIZE_TASK_CORE == 1, "stabilize_task phai pin core 1");

#define ATTITUDE_STALE_US   (int64_t)15000
#define MAX_SAFE_TILT_DEG   45.0f   // cắt ngay bất kể state — không đợi FSM/Commander
#define GROUNDED_ALT_M      0.10f   // dưới ngưỡng này coi như "còn ở đất" cho EMERGENCY resolve

// Vận tốc quay / nghiêng tối đa cho move()/set_yaw() (100% pct). Copy tinh
// thần TILT step / YAW step của GUI UAV-Mini — CHỈNH LẠI khi bay thật.
#define MOVE_MAX_TILT_DEG   12.0f

// PHASE E2 -- expo can nghieng. Xem tuning.h muc "PHASE E2".
// Nhan x trong [-1,1], tra ve trong [-1,1], giu nguyen dau va hai dau mut.
static inline float move_expo(float x) {
    const float e = MOVE_TILT_EXPO;
    return e * x * x * x + (1.0f - e) * x;
}
#define MOVE_MAX_YAW_DPS    60.0f

// ================= Ground-station UDP setpoint (CMD_SET_ATTITUDE) =================
// SP = "setpoint" persistent (khác s_timed_* — xung có thời hạn của CMD_MOVE/
// CMD_SET_YAW, vẫn giữ nguyên cho MicroPython/console). Hằng số clamp
// (SP_TILT_MAX_DEG/SP_YAW_RATE_MAX_DPS/TRIM_MAX_DEG/SP_STALE_TIMEOUT_US) nằm
// trong tuning.h (mục 6) — DÙNG CHUNG với command_parser.c để reply "OK" khớp
// đúng giá trị sẽ áp dụng, KHÔNG định nghĩa lại ở đây.

// CMD_TEST_MOTOR — xác nhận vị trí vật lý động cơ (xem app_config.h). Duty
// thấp có chủ đích (đủ thấy cánh quay ở tốc độ chậm, KHÔNG đủ để văng nếu vô
// tình còn lắp cánh) + tự dừng sau thời gian ngắn, không cần lệnh stop riêng.
#define TEST_MOTOR_MAX_DUTY_PCT   25
#define TEST_MOTOR_PULSE_MS       500

// Thời gian tối đa chờ sensor_hub nhả bus I2C cho lệnh bench (mag_selftest,
// calib_baro_ground). Rộng rãi: hub có thể đang giữa một transaction (timeout
// runtime 8ms/driver, xem imu_driver.c) hoặc đang chờ DRDY của mag (150ms).
// Hết hạn -> HUỶ lệnh bench, KHÔNG chạy đè lên bus đang bận.
#define SENSOR_BUS_LEASE_TIMEOUT_MS   300

// ---- state 1-instance (singleton — 1 drone / 1 task) ----
static flight_core_board_config_t s_board;

// MỘT bus I2C DUY NHẤT cho toàn bộ cảm biến (IMU+mag+ToF+baro) — tạo 1 lần
// trong flight_core_start(), sống suốt vòng đời app. KHÔNG bao giờ xóa/tạo
// lại (không i2c_del_master_bus(), không recreate khi 1 sensor lỗi) — mỗi
// driver chỉ add/rm CHÍNH device handle của nó, không đụng bus chung.
static i2c_master_bus_handle_t s_i2c_bus = NULL;

// Tốc độ SCL ĐÃ ĐƯỢC KIỂM TRA/CHUẨN HOÁ trong flight_core_start() (validate
// 10k..1M, rơi về 400000 nếu ngoài dải). Giữ lại vì bring-up chạy lại lúc
// runtime (flight_core_tof_reinit()) PHẢI dùng ĐÚNG tốc độ đã dùng lúc boot —
// đọc lại s_board.i2c_freq_hz thô sẽ bỏ qua bước chuẩn hoá đó.
static uint32_t s_i2c_scl_hz = 400000;
// Tốc độ SCL RIÊNG của ToF (có thể thấp hơn bus chung — xem
// BOARD_TOF_I2C_FREQ_HZ). flight_core_tof_reinit() PHẢI dùng lại đúng giá trị
// này, nếu không lần init tay sẽ chạy ở tốc độ khác lúc boot và cho kết quả
// không so sánh được với nhau.
#if FC_FEATURE_TOF
static uint32_t s_tof_scl_hz = 400000;
#endif

static mahony_t          s_mahony;
static alt_estimator_t   s_alt_est;
static attitude_state_t  s_att_state;
static attitude_gains_t  s_att_gains;
static alt_hold_state_t  s_hold_state;
static alt_hold_tune_t   s_hold_tune;
static takeoff_state_t   s_tko_state;
static takeoff_tune_t    s_tko_tune;
// Kết quả takeoff_run() của tick gần nhất — CHỈ để publish telemetry ở bước 11.
// KHÔNG được đọc lại làm đầu vào logic bay: nguồn sự thật là s_tko_state
// (xem takeoff_control_active()/takeoff_airborne()).
static takeoff_result_t  s_tko_result;
static landing_state_t   s_land_state;
static landing_tune_t    s_land_tune;
// s_vbat_ring: lịch sử ~500ms điện áp pin, nạp ở bước 2b, đọc MỘT LẦN lúc ARM.
// KHÔNG reset trong reset_all_controllers(): đó là lịch sử CẢM BIẾN, không phải
// state điều khiển. Xoá nó lúc disarm sẽ khiến lần ARM ngay sau đó bị từ chối
// vì "chưa đủ mẫu" trong ~300ms — một lỗi tự gây, không có lợi ích nào.
//
// ⚠ NGOÀI mọi #if: CẢ HAI cơ chế bù pin đều đọc nó (hover latch ở CMD_TAKEOFF,
// hệ số nhân ở CMD_ARM). Gate nó theo một cờ sẽ làm cơ chế kia không build được
// khi cờ đó tắt — đúng lỗi vừa gặp.
static hover_vbat_ring_t s_vbat_ring;

#if FC_FEATURE_HOVER_LATCH
// ---- Latch ga hover theo pin (hover_model.h) ----
// Giá trị đã chốt của lần ARM gần nhất — CHỈ để telemetry/log. Nguồn sự thật
// khi bay là s_hold_tune.hover / s_tko_tune.prime_duty (đã ghi đè lúc ARM).
static float s_hover_latch_v    = 0.0f;   // vbat trung vị lúc latch
static float s_hover_latch_duty = 0.0f;   // hover suy ra
static bool  s_hover_latched    = false;  // đã latch lần nào chưa (từ lúc boot)
#endif  // FC_FEATURE_HOVER_LATCH


static fsm_t              s_fsm;
static commander_state_t  s_cmd_state;
static commander_config_t s_cmd_cfg;
static imu_calib_t        s_imu_calib;

// Moc bat dau khoang "khong co lenh dieu khien nao" trong FSM_FLYING.
// 0 = dang co lenh (hoac khong o FLYING). Dem lai tu 0 moi khi nguoi lai
// cham can, nen nguong FLYING_TO_HOLD_SETTLE_MS la 1s LIEN TUC yen.
static int64_t s_flying_idle_since_us = 0;
static bool s_imu_ok_driver = false;
static bool s_mag_ok_driver = false;
static bool s_tof_ok_driver = false;
static bool s_baro_ok_driver = false;
static bool s_battery_ok_driver = false;

// ---- FSM_BENCH_RAMP (bench-test PID tuning, xem GHI CHÚ 3
// flight_state_machine.h) — throttle đi thẳng từ đây, KHÔNG qua alt_hold. ----
static int s_bench_throttle_duty = 0;
// Offset TẠM THỜI của phím giữ (W/S ở GUI). TÁCH HẲN khỏi s_bench_throttle_duty:
// duty là mức ga đã "chốt" (cộng dồn bằng STEP), offset là thứ chỉ tồn tại
// trong lúc còn giữ phím và tự biến mất — nhả phím phải trả về ĐÚNG mức cũ.
// Gộp hai cái vào một biến thì không có cách nào biết phải trả về đâu.
static int     s_bench_throttle_offset = 0;
static int64_t s_bench_offset_update_us = 0;

// ============================================================================
// LATCH GA CHO FSM_FLYING — "bay ngang thì TẮT PID độ cao, giữ nguyên phần còn lại"
// ============================================================================
// FLYING = đang có lệnh nghiêng (tiến/lùi/trái/phải). Ở state này alt_hold
// KHÔNG chạy: throttle bị ĐÓNG BĂNG tại đúng giá trị mà alt_hold đang xuất ở
// tick cuối cùng của HOLDING, cộng thêm offset W/S nếu người lái đang giữ phím.
//
// VÌ SAO KHÔNG để alt_hold chạy tiếp khi nghiêng:
// Nghiêng để bay ngang làm giảm THÀNH PHẦN THẲNG ĐỨNG của lực đẩy (cos của góc
// nghiêng), nên drone tụt nhẹ. alt_hold nhìn thấy tụt và ĐẨY GA LÊN để bù —
// nhưng ga cao hơn ở góc nghiêng đó lại làm drone BAY NGANG NHANH HƠN. Vòng
// lặp này biến một lệnh "tiến nhẹ" thành phóng đi kèm dao động độ cao.
//
// ĐIỀU GÌ KHÔNG ĐỔI (đây là phần quan trọng): TOÀN BỘ phần còn lại vẫn chạy y
// hệt — attitude PID (roll/pitch/yaw), mixer, estimator độ cao (vẫn cập nhật
// alt_m/vz_ms để telemetry và geofence dùng), Commander/failsafe, terrain
// guard, W/S. CHỈ MỖI vòng PID giữ độ cao bị treo.
//
// -1 = chưa latch (đang HOLDING hoặc chưa bay). Đặt lại về -1 mỗi khi rời
// FLYING để lần vào sau chốt giá trị MỚI, không dùng lại số của chuyến trước.
static int     s_flying_throttle_latch = -1;
// Credit tich luy cho tran toc do tang ga (THROTTLE_MAX_RISE_DUTY_PER_S).
// Can vi 150 duty/s * dt(4ms) = 0.6 duty — lam tron nguyen se ra 0 va tran bien
// thanh khoa cung. Cong don phan le qua cac tick roi moi tieu tung duty nguyen.
static float   s_flying_rise_credit = 0.0f;
// PHASE A: da nap I cho vong Vz cua lan vao FLYING nay chua.
// Tach RIENG khoi (s_flying_throttle_latch < 0) vi latch gio duoc GHI LAI moi
// tick (no duoi theo output cascade), nen no khong con dung lam co "tick dau"
// duoc nua. Dung chung mot co se nap lai I moi tick -> I bi ghi de lien tuc ->
// vong Vz mat hoan toan phan tich phan.
static bool    s_flying_vz_engaged = false;

// Duty mà alt_hold xuất ở tick HOLDING gần nhất — nguồn để chốt latch ở trên.
// Cần biến RIÊNG vì throttle_cmd là biến CỤC BỘ, bị gán 0 ở đầu bước 9 mỗi
// tick, nên tại thời điểm vào FLYING không còn đọc lại được giá trị cũ.
// 0 = chưa từng ở HOLDING (rơi về hover_ff của tune, xem chỗ dùng).
static int     s_last_hold_throttle = 0;
// Watchdog: mất gói "nhả phím" (UDP) hoặc ground-station chết -> offset tự về
// 0. KHÔNG BAO GIỜ để một offset ga dương dính lại khi không còn ai điều khiển.
// 400ms = khớp SP_STALE_TIMEOUT_US, cùng lý do (xem tuning.h mục 6).
#define BENCH_OFFSET_STALE_US   ((int64_t)400000)

// Setpoint điều khiển bởi lệnh Python (xem apply_command()).
// ---- Frame độ cao (alt_estimator.h alt_frame_t) — ĐỔI ĐƯỢC LÚC RUNTIME qua
// fc.set_param("alt_frame", 0=DATUM | 1=AGL).
//   DATUM = giữ độ cao so với SÀN cất cánh (mặc định, hành vi cũ). Bay qua bàn
//           thì khoảng hở GIẢM đúng bằng chiều cao bàn.
//   AGL   = giữ KHOẢNG CÁCH so với bề mặt đang nhìn (terrain following). Bay
//           qua bàn thì drone LEO LÊN đúng bằng chiều cao bàn.
// KHÔNG reset trong reset_all_controllers(): đây là lựa chọn của người lái,
// không phải state điều khiển.
static alt_frame_t s_alt_frame = ALT_FRAME_DATUM;

// ---- D4: cửa sổ degrade khi mất nguồn Z giữa lúc HOLD ----
// Trong cửa sổ này alt_hold KHÔNG báo engage_lost: giữ vz≈0 bằng az_earth,
// I-term FREEZE. Hết cửa sổ mới nhả cho Commander -> LANDING blind.
static bool    s_alt_degrade_active = false;
static int64_t s_alt_degrade_since_us = 0;
// Guard khoảng hở (B8) — cờ EDGE để chỉ rebase terrain MỘT LẦN mỗi lần chạm
// guard, không phải mỗi tick (mỗi tick sẽ làm terr_commit_count chạy loạn và
// làm landing tưởng có bậc địa hình mới liên tục).
#if FC_FEATURE_TERRAIN_OFFSET
static bool    s_terr_guard_active = false;
#endif

static float   s_alt_target_m = 0.0f;
static float   s_alt_request_m = 0.0f; // Commander-clamped request; target slews toward it
static bool    s_timed_active = false;
static float   s_timed_roll_deg = 0.0f;
static float   s_timed_pitch_deg = 0.0f;
static float   s_timed_yaw_rate_dps = 0.0f;
static int64_t s_timed_until_us = 0;

// ---- Ground-station UDP setpoint (persistent, xem CMD_SET_ATTITUDE) ----
static float   s_sp_roll_deg = 0.0f;
static float   s_sp_pitch_deg = 0.0f;
static float   s_sp_yaw_rate_dps = 0.0f;
static int64_t s_last_sp_update_us = 0;
static bool    s_sp_stale_prev = true;   // trạng thái stale TRƯỚC (log 1 lần khi chuyển sang stale)

// ---- Trim (bias cộng thêm vào roll/pitch target, xem CMD_SET_TRIM) ----
// Giá trị mặc định nằm ở tuning.h (TRIM_*_DEG_DEFAULT) — trước đây là hai số
// ma thuật viết thẳng ở đây, tức là thứ DUY NHẤT trong đường chỉnh trim mà
// không sửa được từ file tune.
//
// HOÁN TRỤC ở đây, ĐÚNG MỘT LẦN, giống hệt case CMD_SET_TRIM và đường nạp NVS
// trong flight_core_start(): hằng số trong tuning.h theo QUY ƯỚC NGOÀI (roll =
// trái/phải như GUI hiển thị), còn hai biến này là trục VẬT LÝ. Giữ cả ba
// đường (mặc định / lệnh / NVS) hoán ở đúng biên là điều kiện để trim không
// bao giờ bị xoay 90° tuỳ theo nó đến từ đâu.
static float s_trim_roll_deg  = TRIM_PITCH_DEG_DEFAULT;   // trục vật lý roll  <- NGOÀI pitch
static float s_trim_pitch_deg = TRIM_ROLL_DEG_DEFAULT;    // trục vật lý pitch <- NGOÀI roll

// ================= MicroPython fc.control() (xem CMD_CONTROL, tuning.h mục 7) =================
// rol/pit/yaw/thr RAW (-100..100) — lưu riêng CHỈ để fc.get_states() đọc lại
// (khác s_sp_roll_deg/pitch/yaw_rate_dps đã convert sang deg/dps, DÙNG CHUNG
// với ground-station SP). Stale theo CHUNG s_last_sp_update_us/sp_stale (step
// 4b) — control() và ground-station SP là 1 kênh setpoint tay lái duy nhất.
static float s_ctrl_rol_pct = 0.0f, s_ctrl_pit_pct = 0.0f, s_ctrl_yaw_pct = 0.0f, s_ctrl_thr_pct = 0.0f;
static float s_max_lean_deg = CONTROL_MAX_LEAN_DEG_DEFAULT;       // runtime-tunable, xem apply_set_param()
static float s_max_yawrate_dps = CONTROL_MAX_YAWRATE_DPS_DEFAULT;
static int   s_flightmode = 1;          // 1=head (mac dinh), 0=headless (STUB, xem CMD_CONTROL)
static bool  s_headless_warned = false;

// Bù throttle theo điện áp pin ĐÃ BỎ HẲN — 2 biến state
// s_battery_comp_nominal_v/max_gain cùng 2 hằng số BATTERY_COMPENSATION_* đã
// xoá. Lý do đầy đủ ở bước 9b trong stabilize_task(). Điện áp pin giờ CHỈ dùng
// để so ngưỡng failsafe (Commander) và prearm_check().

// ================= Calibration (xem calibration.h + command.h CMD_CALIB_*) =================
static calibration_params_t s_calib;
static bool s_uncalibrated = true;   // true -> CMD_ARM tu choi (xem apply_command CMD_ARM)

static bool s_calib_gyro_active = false;

// ============================================================================
// GYRO CALIBRATION — MỘT FSM dùng chung cho boot và lệnh CAL GYRO.
// ============================================================================
// Máy trạng thái chạy TRONG stabilize_task (bước 1b), KHÔNG phải task riêng:
// nó cần đúng dòng mẫu IMU mà vòng bay dùng, và cần bảo đảm Mahony KHÔNG tích
// phân trong lúc nó chạy (mục 21). Chạy ở task khác thì phải đồng bộ hai thứ
// đó qua khoá — phức tạp hơn mà không được gì.
// ĐÃ BỎ GCAL_WAIT_STATIONARY và toàn bộ cơ chế attempt/retry.
//
// VÌ SAO BỎ (lỗi thật, quan sát được trên bo):
//   GYRO CAL retry 2/3: khong co cua so stationary lien tuc
//   GYRO CAL FAIL code=3 -> ARM BI CHAN
// Cổng cũ đòi GYRO_CAL_STATIONARY_CONFIRM_MS mẫu đứng yên LIÊN TỤC, và test
// mỗi mẫu là |gyro_raw| < GYRO_CAL_RAW_NORM_MAX_DPS. Nhưng gyro_raw ĐÃ MANG
// SẴN BIAS — bo này bias ~(0.08, -2.78, -1.94) nên |gyro_raw| ≈ 3.4 dps ngay
// khi nằm im. Ngưỡng 5.0 dps chỉ còn chừa 1.6 dps cho nhiễu, và MỘT mẫu vượt
// là reset bộ đếm về 0. Không bao giờ gom nổi một cửa sổ liên tục.
//
// Đó là lỗi vòng luẩn quẩn: cổng dùng để CHO PHÉP đo bias lại phụ thuộc vào
// chính cái bias chưa đo. Bias càng lớn — tức càng cần calib — thì càng không
// calib được.
//
// Quay lại luồng cũ (theo yêu cầu người dùng): SETTLE -> COLLECT thẳng, gom đủ
// GYRO_CAL_DURATION_MS rồi mới phán xét bằng THỐNG KÊ CẢ CỬA SỔ (std + tỉ lệ
// mẫu xấu), không chặn theo từng mẫu. std là đại lượng KHÔNG phụ thuộc bias
// (bias là DC, std là AC) nên nó đo đúng thứ cần đo: "drone có bị động không".
typedef enum {
    GCAL_IDLE = 0,
    GCAL_SETTLING = 1,
    GCAL_COLLECT = 2,
    GCAL_PASS = 3,
    GCAL_FAIL = 4,
} gyro_cal_state_t;

typedef enum {
    GCAL_FAIL_NONE = 0,
    GCAL_FAIL_IMU_UNAVAILABLE,
    GCAL_FAIL_IMU_CONFIG_INVALID,
    GCAL_FAIL_TOO_FEW_SAMPLES,
    GCAL_FAIL_BIAS_NONFINITE,
    GCAL_FAIL_ABORTED,
} gyro_cal_fail_t;

// Welford online mean/variance: không giữ 4000 mẫu và ổn định số hơn
// E[x^2]-E[x]^2 khi variance nhỏ so với DC bias.
typedef struct {
    vec3f_t gyro_mean, gyro_m2;
    vec3f_t accel_mean, accel_m2;
    int count;
    int ticks_left;
} gyro_cal_window_t;

static gyro_cal_state_t s_gcal_state = GCAL_IDLE;
static gyro_cal_fail_t  s_gcal_fail = GCAL_FAIL_NONE;
static gyro_cal_window_t s_gcal_win;
// Số mẫu KHÔNG đạt tiêu chí đứng yên trong cửa sổ hiện tại. Chỉ ĐẾM, không
// dùng để cắt ngang — phán xét bằng tỉ lệ ở CUỐI cửa sổ (xem enum ở trên).
static int              s_gcal_bad_samples = 0;
static vec3f_t          s_gcal_candidate_bias;
static float            s_gcal_temp_c = 0.0f;
static const char      *s_gcal_fail_reason = "";

static vec3f_t s_gcal_last_raw_mean;
static vec3f_t s_gcal_last_corr_mean;
static vec3f_t s_gcal_last_raw_std;
static vec3f_t s_gcal_last_corr_std;
static bool    s_gcal_reset_done = false;

// ---- Pre-arm gyro health (mục 18): trung bình trượt khi DISARMED ----
static vec3f_t s_prearm_gyro_sum;
static int     s_prearm_gyro_count = 0;
static vec3f_t s_prearm_gyro_mean;       // cập nhật mỗi khi đủ cửa sổ
static bool    s_prearm_gyro_mean_valid = false;

static void gyro_cal_window_reset(gyro_cal_window_t *w, int ticks) {
    memset(w, 0, sizeof(*w));
    w->ticks_left = ticks;
}

static void welford_axis(float sample, int n, float *mean, float *m2) {
    const float delta = sample - *mean;
    *mean += delta / (float)n;
    *m2 += delta * (sample - *mean);
}

static void gyro_cal_window_add(gyro_cal_window_t *w, vec3f_t gyro_dps, vec3f_t accel_g) {
    w->count++;
    welford_axis(gyro_dps.x, w->count, &w->gyro_mean.x, &w->gyro_m2.x);
    welford_axis(gyro_dps.y, w->count, &w->gyro_mean.y, &w->gyro_m2.y);
    welford_axis(gyro_dps.z, w->count, &w->gyro_mean.z, &w->gyro_m2.z);
    welford_axis(accel_g.x, w->count, &w->accel_mean.x, &w->accel_m2.x);
    welford_axis(accel_g.y, w->count, &w->accel_mean.y, &w->accel_m2.y);
    welford_axis(accel_g.z, w->count, &w->accel_mean.z, &w->accel_m2.z);
}

static vec3f_t gyro_cal_window_mean(const gyro_cal_window_t *w, bool gyro) {
    return gyro ? w->gyro_mean : w->accel_mean;
}

static vec3f_t gyro_cal_window_std(const gyro_cal_window_t *w, bool gyro) {
    if (w->count < 2) return vec3f_zero();
    const float inv = 1.0f / (float)(w->count - 1);
    const vec3f_t m2 = gyro ? w->gyro_m2 : w->accel_m2;
    return (vec3f_t){
        sqrtf(fmaxf(m2.x * inv, 0.0f)),
        sqrtf(fmaxf(m2.y * inv, 0.0f)),
        sqrtf(fmaxf(m2.z * inv, 0.0f)),
    };
}

static bool vec3_finite(vec3f_t v) {
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

static bool gyro_cal_sample_stationary(const imu_sample_t *imu) {
    if (imu == NULL || !imu->ok || !vec3_finite(imu->gyro_raw_dps) ||
        !vec3_finite(imu->accel_raw_g)) return false;
    const vec3f_t g = imu->gyro_raw_dps;
    const vec3f_t a = imu->accel_raw_g;
    const float gnorm = sqrtf(g.x * g.x + g.y * g.y + g.z * g.z);
    const float anorm = sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
    return gnorm < GYRO_CAL_RAW_NORM_MAX_DPS &&
           fabsf(anorm - 1.0f) < GYRO_CAL_ACCEL_NORM_TOL_G;
}

static bool    s_calib_accel_capturing = false;
static int     s_calib_accel_samples_left = 0;  // đếm TICK trôi qua (cửa sổ ~0.5s cố định)
static int     s_calib_accel_valid_count = 0;   // đếm mẫu THẬT SỰ cộng vào sum (imu.ok==true)
static vec3f_t s_calib_accel_sum;
static int     s_calib_accel_motion_bad_count;  // số tick |gyro| vượt ngưỡng trong lúc bắt 1 mặt
static int     s_calib_accel_faces_done = 0;
// Lưu RIÊNG từng mặt (KHÔNG chỉ min/max chạy) — cần đủ 6 vector để: (1) tính
// min/max lúc finalize, (2) verify residual bằng cách áp lại correction lên
// chính 6 mặt đã đo (xem "13. Accel calibration verification").
static vec3f_t s_calib_accel_face_avg[CALIB_ACCEL_FACES_NEEDED];

static bool     s_calib_mag_active = false;
static bool     s_calib_mag_extrema_init = false;
static vec3f_t  s_calib_mag_min, s_calib_mag_max;
static uint32_t s_calib_mag_sample_count = 0;
static int      s_calib_mag_ticks_left = 0;   // đếm TICK trôi qua (cửa sổ CALIB_MAG_DURATION_MS
                                                // cố định, KHÔNG phụ thuộc mag_ok — cần trôi qua
                                                // đúng THỜI GIAN thật để user xoay đủ, xem step 1b)
// Vì sao KHÔNG có mẫu — phân biệt 3 nguyên nhân hoàn toàn khác nhau mà trước
// đây gộp hết thành 1 thông báo "xoay lau hon" đổ lỗi cho người dùng:
//   read_err  : mag_driver_read() trả lỗi I2C (dây/pull-up/bus chung kẹt)
//   notready  : đọc OK nhưng ok=false (DRDY=0 chưa có mẫu mới, OVFL, hoặc
//               sample toàn 0) — bình thường chiếm ~60% tick vì ODR mag
//               (100Hz) THẤP HƠN tick rate (250Hz); CHỈ bất thường khi
//               chiếm 100% (chip không đo -> xem probe_drdy() mag_driver.c)
// Reset ở CMD_CALIB_MAG_START, đọc ở finalize_mag_calibration().
static uint32_t s_calib_mag_read_err_count = 0;
static uint32_t s_calib_mag_notready_count = 0;
// Đếm ngược tới lần in tiến độ kế tiếp (CALIB_MAG_PROGRESS_MS) — xem step 1b.
static int      s_calib_mag_progress_ticks_left = 0;
// seq của mẫu mag CUỐI CÙNG đã đưa vào min/max. Calibration KHÔNG đọc phần
// cứng lần hai (xem mag_driver.c "MỘT NGƯỜI ĐỌC DUY NHẤT") — nó tiêu thụ mẫu
// mà sensor task đã publish, và chỉ nhận khi seq ĐỔI để không đếm trùng 1 mẫu
// nhiều lần (tick 250Hz nhanh hơn ODR mag 100Hz nên phần lớn tick là mẫu cũ).
static uint32_t s_calib_mag_last_seq = 0;

// Nhiệt độ IMU mẫu gần nhất (°C, xem imu_driver.h) — publish telemetry +
// log kèm mỗi lần calib gyro (xem "8. Gyro thermal drift").
static float s_last_imu_temp_c = 0.0f;

// Quality metric của lần calib GẦN NHẤT (thành công hay thất bại đều cập
// nhật — hữu ích để debug ngay cả khi calib bị từ chối) — publish qua
// telemetry, xem telemetry.h.
static float   s_last_accel_calib_residual_g = 0.0f;

// recompute_uncalibrated() — gọi sau load NVS lúc boot + sau mỗi lần calib xong.
//
// BẮT BUỘC để bay: CHỈ gyro + accel. MAG KHÔNG CÒN BẮT BUỘC.
//
// VÌ SAO BỎ MAG: chế độ bay hiện tại dùng yaw-RATE (heading-hold kp=0 mặc định,
// xem attitude_control.c) — mag chết chỉ làm yaw trôi chậm, KHÔNG làm mất điều
// khiển roll/pitch. Khoá ARM vĩnh viễn vì một cảm biến mà chế độ bay không cần
// là chặn sai chỗ. Đây cũng đã là chính sách của prearm_check() từ trước (mag
// chỉ CẢNH BÁO, không chặn) — chỗ này trước đó mâu thuẫn với nó.
//
// Mag vẫn được dùng khi CÓ calib hợp lệ: Mahony chỉ fuse mag nếu
// s_calib.mag_valid (xem bước 1c) — mag chưa calib mà fuse vào sẽ làm hỏng yaw,
// tệ hơn là không dùng.
//
// gyro KHÔNG kiểm ở đây mà ở prearm_check() (ARM_REJECT_GYRO_CALIB) — cố ý:
// prearm_check cho mã lý do CỤ THỂ, còn nhánh này chỉ trả về
// ARM_REJECT_UNCALIBRATED chung chung. Cả hai đều chặn ARM, nên gyro vẫn là
// điều kiện bắt buộc; chỉ khác thông báo nào tới tay người dùng.
static void recompute_uncalibrated(void) {
    s_uncalibrated = !s_calib.accel_valid;
}

// ============================================================================
// Gyro calibration FSM — dùng CÙNG MỘT đường cho startup và CAL GYRO.
// ============================================================================
static bool gyro_cal_state_active(gyro_cal_state_t state) {
    return state == GCAL_SETTLING || state == GCAL_COLLECT;
}

static int gyro_cal_ticks(int duration_ms) {
    const int ticks = (int)((float)duration_ms * CONTROL_TASK_HZ / 1000.0f);
    return ticks > 0 ? ticks : 1;
}

static void gyro_cal_set_runtime_bias(vec3f_t bias) {
    // Một copy struct dưới mutex của hub: X/Y/Z đổi cùng một thời điểm, không
    // giữ lock quanh I2C. Đây là điểm DUY NHẤT thay bias runtime.
    s_imu_calib.gyro_bias_dps = bias;
    sensor_hub_set_imu_calib(&s_imu_calib);
}

static void gyro_cal_enter_collect(void) {
    gyro_cal_window_reset(&s_gcal_win, gyro_cal_ticks(GYRO_CAL_DURATION_MS));
    s_gcal_bad_samples = 0;
    s_gcal_state = GCAL_COLLECT;
    ESP_LOGI(TAG, "GYRO CAL: COLLECT RAW %dms — GIU YEN drone", GYRO_CAL_DURATION_MS);
}

static void gyro_cal_finish_fail(gyro_cal_fail_t fail, const char *reason) {
    s_gcal_fail = fail;
    s_gcal_fail_reason = reason;
    s_gcal_state = GCAL_FAIL;
    s_calib_gyro_active = false;
    s_calib.gyro_valid = false;
    gyro_cal_set_runtime_bias(vec3f_zero());
    s_prearm_gyro_mean_valid = false;
    s_prearm_gyro_count = 0;
    s_prearm_gyro_sum = vec3f_zero();
    ESP_LOGE(TAG, "GYRO CAL FAIL code=%d (%s) -> bias NVS KHONG duoc dung de bay, ARM BI CHAN",
             (int)fail, reason);
}

// KHÔNG còn retry tự động. Một lần đo, một kết luận.
//
// Retry tự động chỉ có nghĩa khi lần sau có cơ hội khác lần trước — mà nguyên
// nhân trượt ở đây (drone bị cầm/rung, hoặc bias vượt sanity bound) không tự
// thay đổi trong vài giây. Ba lần thử chỉ kéo dài 15s rồi báo cùng một lỗi,
// trong khi người dùng không biết phải làm gì giữa các lần.
// Trượt -> nói rõ lý do -> người dùng đặt lại drone -> gõ 'calib_gyro'.

static void gyro_calibration_start(void) {
    imu_cfg_readback_t cfg;
    imu_driver_get_config(&cfg);

    // Boot va CAL GYRO deu chi duoc chay khi DISARMED. Cat output them mot lan
    // ngay tai entry point de dieu kien "motors OFF" khong chi la suy luan tu
    // state FSM; trong suot calibration, pre-arm van bi khoa boi gyro_valid=false.
    motor_driver_all_off();

    s_gcal_candidate_bias = vec3f_zero();
    s_gcal_last_raw_mean = vec3f_zero();
    s_gcal_last_corr_mean = vec3f_zero();
    s_gcal_last_raw_std = vec3f_zero();
    s_gcal_last_corr_std = vec3f_zero();
    s_gcal_temp_c = 0.0f;
    s_gcal_bad_samples = 0;
    s_gcal_fail = GCAL_FAIL_NONE;
    s_gcal_fail_reason = "";
    s_gcal_reset_done = false;
    s_prearm_gyro_mean_valid = false;
    s_prearm_gyro_count = 0;
    s_prearm_gyro_sum = vec3f_zero();
    s_calib.gyro_valid = false;
    gyro_cal_set_runtime_bias(vec3f_zero());
    mahony_init(&s_mahony, NULL);

    if (!s_imu_ok_driver) {
        gyro_cal_finish_fail(GCAL_FAIL_IMU_UNAVAILABLE, "IMU init/read unavailable");
        return;
    }
    if (!cfg.valid) {
        gyro_cal_finish_fail(GCAL_FAIL_IMU_CONFIG_INVALID, "MPU6050 read-back config invalid");
        return;
    }

    s_calib_gyro_active = true;
    gyro_cal_window_reset(&s_gcal_win, gyro_cal_ticks(GYRO_CAL_SETTLE_MS));
    s_gcal_state = GCAL_SETTLING;
    ESP_LOGW(TAG, "GYRO CAL start: settle=%dms -> collect=%dms (NO VALIDATE, chi lay mau va tinh bias)",
             GYRO_CAL_SETTLE_MS, GYRO_CAL_DURATION_MS);
}

// Gọi mỗi tick TRƯỚC Mahony. Validation tự tính raw-candidate, không dùng
// gyro_dps đang mang bias runtime; vì vậy phép kiểm độc lập với producer.
static bool gyro_calibration_tick(const imu_sample_t *imu, bool imu_updated) {
    switch (s_gcal_state) {
        case GCAL_IDLE:
        case GCAL_PASS:
        case GCAL_FAIL:
            return false;

        case GCAL_SETTLING:
            if (--s_gcal_win.ticks_left <= 0) gyro_cal_enter_collect();
            return true;

        case GCAL_COLLECT: {
            if (imu_updated) {
                // Chỉ gom mẫu IMU để tính mean raw = bias. Không cấm cửa sổ theo
                // stationary; mục tiêu là ước lượng offset, không validate động cơ.
                gyro_cal_window_add(&s_gcal_win, imu->gyro_raw_dps, imu->accel_raw_g);
                s_gcal_temp_c = imu->temp_c;
            }
            if (--s_gcal_win.ticks_left > 0) return true;

            const int expected = gyro_cal_ticks(GYRO_CAL_DURATION_MS);
            if ((float)s_gcal_win.count < (float)expected * GYRO_CAL_MIN_VALID_FRACTION) {
                gyro_cal_finish_fail(GCAL_FAIL_TOO_FEW_SAMPLES,
                                     "qua it mau IMU trong COLLECT");
                return true;
            }

            const vec3f_t raw_mean = gyro_cal_window_mean(&s_gcal_win, true);
            const vec3f_t raw_std = gyro_cal_window_std(&s_gcal_win, true);
            const vec3f_t accel_std = gyro_cal_window_std(&s_gcal_win, false);
            s_gcal_last_raw_mean = raw_mean;
            s_gcal_last_raw_std = raw_std;
            s_gcal_last_corr_mean = vec3f_zero();
            s_gcal_last_corr_std = vec3f_zero();

            if (!vec3_finite(raw_mean) || !vec3_finite(raw_std) || !vec3_finite(accel_std)) {
                gyro_cal_finish_fail(GCAL_FAIL_BIAS_NONFINITE,
                                       "bias/std non-finite");
                return true;
            }

            s_gcal_candidate_bias = raw_mean;
            gyro_cal_set_runtime_bias(s_gcal_candidate_bias);
            s_calib.gyro_bias_dps = s_gcal_candidate_bias;
            s_calib.gyro_cal_temp_c = s_gcal_temp_c;
            s_calib.gyro_valid = true;
            s_gcal_state = GCAL_PASS;
            s_gcal_fail = GCAL_FAIL_NONE;
            s_gcal_fail_reason = "";
            s_calib_gyro_active = false;
            s_gcal_reset_done = false;
            const esp_err_t save_err = calibration_save_gyro(&s_calib.gyro_bias_dps,
                                                              s_gcal_temp_c);
            if (save_err == ESP_OK) {
                s_calib.gyro_valid_from_nvs = true;
                s_calib.gyro_cal_temp_valid_from_nvs = true;
            }
            ESP_LOGW(TAG, "GYRO CAL PASS: GRAWmean=(%.3f,%.3f,%.3f) GBIAS=(%.3f,%.3f,%.3f) "
                          "raw_std=(%.3f,%.3f,%.3f) accel_std=(%.3f,%.3f,%.3f) temp=%.1fC NVS=%s",
                     (double)raw_mean.x, (double)raw_mean.y, (double)raw_mean.z,
                     (double)s_calib.gyro_bias_dps.x, (double)s_calib.gyro_bias_dps.y,
                     (double)s_calib.gyro_bias_dps.z,
                     (double)raw_std.x, (double)raw_std.y, (double)raw_std.z,
                     (double)accel_std.x, (double)accel_std.y, (double)accel_std.z,
                     (double)s_gcal_temp_c,
                     save_err == ESP_OK ? "saved" : "save-failed");
            return true;
        }
    }
    return false;
}

// yaw tương đối so với mốc lúc ARM gần nhất (giống YAWREL= của UAV-Mini) —
// chỉ để hiển thị/log, KHÔNG dùng trong vòng điều khiển.
static float s_yaw_ref_offset_deg = 0.0f;

// Cộng dồn lỗi đọc I2C (mọi driver cảm biến) từ lúc boot — best-effort, chỉ
// để debug/telemetry, KHÔNG dùng để quyết định logic điều khiển.
static uint32_t s_sensor_err_count = 0;

// Đã cảnh báo "mag đọc được nhưng chưa calib" chưa — log 1 LẦN duy nhất, không
// spam 250 dòng/giây. Xem bước 1c.
static bool s_mag_uncalib_warned = false;

// Mahony ĐÃ THẬT SỰ dùng / ĐÃ TỪ CHỐI bao nhiêu mẫu mag (cộng dồn từ boot).
// Không có 2 số này thì không cách nào biết mag đang giúp hay đang bị gate
// chặn sạch — mag_ok=1 mà mag_used không tăng nghĩa là mag đọc được nhưng
// KHÔNG được dùng (chưa calib, hoặc lệch trục nên vượt mag_error_gate).
static uint32_t s_mag_used_count = 0;
static uint32_t s_mag_rejected_count = 0;

// dt thực đo giữa 2 tick — phát hiện jitter/task bị trễ (network task chiếm
// CPU quá lâu chẳng hạn).
static int64_t s_last_tick_us = 0;

// ---- dt TÍCH PHÂN, tách khỏi dt của tick ----
// Cộng dồn thời gian từ lần CUỐI có mẫu IMU MỚI. Mọi bộ tích phân dùng mẫu IMU
// (Mahony, alt_estimator) chỉ chạy khi có mẫu mới, và khi đó phải dùng ĐÚNG
// khoảng thời gian đã trôi qua giữa hai MẪU — không phải khoảng giữa hai TICK.
//
// Vì sao cần: vòng điều khiển có đồng hồ dự phòng (vTaskDelayUntil) nên nó VẪN
// chạy 250Hz khi sensor_hub ngừng báo mẫu mới. Nếu cứ tích phân mỗi tick thì
// CÙNG MỘT mẫu IMU cũ được đưa vào Mahony/Az/Vz/Z hàng chục lần liên tiếp:
// gyro bias giả được tích phân thành góc trôi thật, accel cũ (đã trừ gravity)
// được tích phân thành Vz/Z hoàn toàn bịa. Tệ nhất là nó KHÔNG trông như hỏng —
// attitude/altitude vẫn "chạy mượt", chỉ là chạy trên dữ liệu chết.
static float s_fusion_dt_accum_s = 0.0f;
// Số tick liên tiếp KHÔNG có mẫu IMU mới (telemetry/chẩn đoán).
static uint32_t s_imu_no_new_sample_count = 0;

// Nguồn correction độ cao ở tick TRƯỚC — chỉ để log CẠNH chuyển giao. Vòng bay
// chạy 250Hz nên log theo trạng thái sẽ ra 250 dòng/giây và không ai đọc được;
// log theo CẠNH cho đúng một dòng mỗi lần thật sự đổi nguồn.
static alt_source_t s_alt_source_prev = ALT_SRC_GROUND_LOCK;

// ---- Deadline monitoring (xem bước 0b trong stabilize_task) ----
// Chu kỳ danh nghĩa (us) + hệ số coi là "trượt". 1.5x = trượt nửa chu kỳ, đủ
// rộng để jitter bình thường của FreeRTOS không đếm nhầm, đủ hẹp để phát hiện
// một transaction I2C treo (hàng chục ms = hàng chục lần chu kỳ).
#define CONTROL_DEADLINE_US       (1000000 / CONTROL_TASK_HZ)
#define CONTROL_DEADLINE_FACTOR   1.5f
// Clamp dt đưa vào mọi bộ tích phân/vi phân. MIN chống chia-cho-0 trong D-term
// và trong beta/dt_baro; MAX chặn một bước tích phân khổng lồ sau khi task bị
// treo lâu (4x chu kỳ danh nghĩa).
#define CONTROL_DT_MIN_S          (0.5f / (float)CONTROL_TASK_HZ)
#define CONTROL_DT_MAX_S          (4.0f / (float)CONTROL_TASK_HZ)

// Đo ở CUỐI tick (bước 10), Commander đọc ở ĐẦU tick sau (bước 6) — trễ đúng
// 1 tick (4ms). Chấp nhận có chủ đích: cả hai ngưỡng dùng chúng đều tính theo
// thời gian DUY TRÌ (motor_sat_hard_ms hàng trăm ms), nên lệch 4ms không đổi
// kết quả. Đổi lại, thứ tự trong tick giữ nguyên tuyến tính: đo -> quyết định,
// không phải quyết định -> đo -> quay lại sửa quyết định.
static bool s_motor_saturated_prev = false;
static bool s_alt_hold_engage_lost_prev = false;

// Trạng thái bão hoà mixer của tick TRƯỚC — đầu vào cho conditional
// integration của attitude cascade (xem attitude_control.h mixer_status_t).
static mixer_status_t s_mixer_sat_prev;

static int64_t  s_loop_dt_us = 0;
static int64_t  s_loop_max_us = 0;           // đỉnh từ lúc boot (không tự reset — muốn xem "tệ nhất từng thấy")
static uint32_t s_deadline_miss_count = 0;   // cộng dồn từ boot
static int      s_deadline_miss_streak = 0;  // trượt LIÊN TIẾP hiện tại (Commander dùng, xem bước 6)

// ---- Nhịp vòng điều khiển: notification từ sensor_hub ----
// s_hub_notify_active = ĐANG thật sự chạy theo tín hiệu của hub. Cờ này bị hạ
// nếu hub ngừng báo mẫu mới (bus treo/INT đứt), và khi đó vòng lặp tự rơi về
// đồng hồ FreeRTOS — vẫn quay đúng nhịp, chỉ là dữ liệu cảm biến sẽ stale và
// Commander thấy điều đó qua timestamp. Xem wait_next_control_tick().
static bool     s_hub_notify_active = false;
// Tach nguon danh thuc vong dieu khien (xem telemetry.h muc CHAN DOAN).
static uint32_t s_loop_wake_by_hub = 0;
static uint32_t s_loop_wake_timeout = 0;
static uint32_t s_loop_busy_us = 0;
static uint32_t s_imu_int_wake_count = 0;     // số vòng được hub đánh thức
static uint32_t s_imu_int_timeout_count = 0;  // số lần chờ quá hạn
static int      s_imu_int_miss_streak = 0;    // trượt LIÊN TIẾP hiện tại

// Fault gần nhất do Commander phát hiện (persist qua nhiều tick để GUI kịp
// đọc — commander_evaluate() chỉ trả fault_class_t tại ĐÚNG tick xảy ra, KHÔNG
// tự lưu lại). Reset về FAULT_NONE khi ARM thành công (xem apply_command()).
static fault_class_t s_last_fault_class = FAULT_NONE;

static QueueHandle_t s_cmd_queue = NULL;

// ============================================================================
// KILL LATCH — quyền ưu tiên CAO NHẤT toàn hệ.
// ============================================================================
// Khi latch = true: motor bị cắt và KHÔNG đường code nào ghi được duty khác 0
// cho tới khi có CMD_ARM tường minh (chỉ CMD_ARM mới hạ latch — xem
// apply_command()). Latch phối hợp với armed gate trong motor_driver
// (motor_driver_set_armed()) thành 2 lớp: lớp này chặn ở tầng logic (thoát
// sớm khỏi tick), lớp kia chặn ở tầng driver (ép duty=0 dù có ai lỡ gọi).
//
// volatile + sig_atomic_t: latch được SET từ task KHÁC (net_task qua
// flight_core_kill_now(), không đi qua command queue) và ĐỌC trong
// stabilize_task. bool ghi 1 lần là atomic trên Xtensa (32-bit store), không
// cần mutex — và KHÔNG ĐƯỢC dùng mutex ở đây: đường KILL phải chạy được ngay
// cả khi stabilize_task đang giữ khoá hoặc đang treo.
//
// VÌ SAO KHÔNG ĐI QUA COMMAND QUEUE: queue depth 8 có thể đầy (Python/GUI spam
// setpoint) -> xQueueSend() trả false -> lệnh KILL BỊ RƠI ÂM THẦM. KILL không
// bao giờ được phép fail vì lý do "hàng đợi bận".
static volatile bool s_motor_kill_latched = false;

// Lý do latch gần nhất (chuỗi tĩnh) — telemetry/log. Ghi TRƯỚC khi set latch
// để stabilize_task đọc latch=true thì lý do đã sẵn sàng.
static const char *volatile s_kill_reason = "";

// s_kill_benign — latch này đến từ một lệnh DISARM CHỦ ĐÍCH trên mặt đất, KHÔNG
// phải từ fault/KILL. Cả hai loại latch chặn đường bay y hệt nhau (không có
// ngoại lệ nào ở gate 4a của stabilize_task); cờ này CHỈ để bench-test
// (CMD_TEST_MOTOR) phân biệt được "người dùng vừa disarm xong" với "vừa có sự
// cố".
//
// VÌ SAO CẦN: test_motor đòi DISARMED + !latched, nhưng đường DUY NHẤT hạ latch
// là CMD_ARM — mà ARM lại rời khỏi DISARMED, và DISARM quay về DISARMED thì
// latch lại ngay ở dưới. Không có chuỗi lệnh nào tới được DISARMED + !latched
// sau lần DISARM/KILL đầu tiên -> test_motor chết vĩnh viễn. Với board mới
// (chưa calib accel/mag) thì ARM còn bị từ chối, nên deadlock ngay từ boot đầu.
static volatile bool s_kill_benign = false;

// enter_kill_latch() — đường VÀO DUY NHẤT của latch. Gọi được từ bất kỳ task
// nào. Idempotent (gọi lại khi đã latched không làm gì thêm, không spam log).
// benign=true CHỈ cho CMD_DISARM hợp lệ từ ARMED (chưa bay) — mọi đường fault,
// failsafe, KILL đều phải để false.
static void enter_kill_latch_ex(const char *reason, bool benign) {
    if (s_motor_kill_latched) return;
    s_kill_reason = reason ? reason : "?";
    s_kill_benign = benign;
    s_motor_kill_latched = true;
    // Cắt phần cứng NGAY tại đây, không đợi tick sau của stabilize_task: nếu
    // stabilize_task đang bị trễ/treo thì "đợi tick sau" nghĩa là motor tiếp
    // tục quay ở duty cuối cùng vô thời hạn.
    motor_driver_all_off();
    ESP_LOGE(TAG, "KILL LATCH: %s -- motor cat, can ARM lai de bay tiep", s_kill_reason);
}

// Mặc định: latch KHÔNG benign. Mọi đường fault/failsafe/KILL gọi hàm này, nên
// một call site mới quên nghĩ tới benign sẽ tự động rơi vào phía AN TOÀN.
static void enter_kill_latch(const char *reason) {
    enter_kill_latch_ex(reason, false);
}

// Handle của stabilize_task — giữ ở phạm vi module (thay vì biến local trong
// flight_core_start()) để flight_core_get_task_stats() đọc được high-water-mark
// sau này. Ngoài mục đích chẩn đoán thì KHÔNG ai dùng handle này.
static TaskHandle_t s_stabilize_task = NULL;

static SemaphoreHandle_t s_telemetry_mtx = NULL;
static telemetry_snapshot_t s_telemetry;

// Bảo vệ MỌI tham số live-tune (gains/mahony/trim/alt/tko/land tune) — CHỈ
// stabilize_task ghi (trong apply_command()), net_link/command_parser chỉ đọc
// qua flight_core_get_*() — cùng nguyên tắc 1 writer như s_telemetry_mtx.
static SemaphoreHandle_t s_tuning_mtx = NULL;

// Sequence của mẫu cảm biến đã TIÊU THỤ ở tick trước. So với seq trong
// snapshot để biết mẫu nào THẬT SỰ mới — thay cho cơ chế đếm tick cũ
// (BARO_READ_TICK_DIVISOR). Cách cũ chỉ đúng khi nhịp đọc baro trùng khít bội
// số nhịp điều khiển; cách này đúng KỂ CẢ khi hub chạy nhịp khác, bị trễ, hay
// bỏ lỡ vài mẫu — vì nó hỏi "mẫu này tôi xử lý chưa" chứ không đoán theo giờ.
// (baro KHÔNG có ở đây: alt_estimator giữ seq baro của riêng nó — xem
// alt_estimator.h "CORRECT", tránh 2 bộ đếm song song cho cùng một việc.)
static uint32_t s_last_imu_seq = 0;
static uint32_t s_last_mag_seq = 0;

// ================= command handling =================

// wrap_deg_180() — đưa góc về [-180, 180) — dùng cho YAWREL (yaw tương đối so
// với mốc lúc ARM, xem s_yaw_ref_offset_deg), tránh nhảy số qua biên 180/-180.
static float wrap_deg_180(float deg) {
    while (deg >= 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

static void reset_all_controllers(void) {
    attitude_state_reset(&s_att_state);
    // ⚠ XOA TARGET DO CAO. Truoc day KHONG co dong nay, va do la mot loi
    // nhin thay duoc tren GUI: disarm xong, tab Manual van hien 'tgt 1.54m'
    // cua chuyen bay TRUOC. Te hon la no khong chi la hien thi — s_alt_target_m
    // la trang thai THAT, nen lan ARM ke tiep alt_hold khoi dong voi mot target
    // cu ma nguoi lai khong he dat.
    //
    // Ve 0 chu khong ve alt hien tai: luc reset drone dang o dat, va moi duong
    // vao bay (CMD_TAKEOFF / CMD_SET_ALTITUDE / vao HOLDING) deu tu dat lai
    // target truoc khi PID chay. 0 la 'chua co target', dung nghia nhat.
    s_alt_target_m = 0.0f;
    s_alt_request_m = 0.0f;
    alt_hold_reset(&s_hold_state);
    takeoff_reset(&s_tko_state);
    landing_reset(&s_land_state);
    // Xoá luôn bản telemetry: takeoff_reset() hạ active/liftoff_confirmed nên
    // để s_tko_result cũ lại sẽ báo AIRB=1/score cũ sau khi chuỗi đã bị huỷ.
    s_tko_result = (takeoff_result_t){0};
    s_timed_active = false;
    s_bench_throttle_duty = 0;
    s_bench_throttle_offset = 0;
    // Về NONE chứ không giữ nguồn cũ: sau reset estimator chưa có correction
    // nào, nếu giữ giá trị cũ thì lần đổi thật đầu tiên sẽ không log.
    s_alt_source_prev = ALT_SRC_GROUND_LOCK;
}

// finalize_mag_calibration() — tính hard/soft-iron từ min/max đã tích lũy +
// lưu NVS, dùng CHUNG cho cả 2 đường kết thúc: hết giờ TỰ ĐỘNG (step 1b,
// CALIB_MAG_DURATION_MS trôi hết) và CMD_CALIB_MAG_STOP (kết thúc SỚM theo
// yêu cầu). KHÔNG gọi khi s_calib_mag_active đã false (caller tự đảm bảo).
static void finalize_mag_calibration(void) {
    s_calib_mag_active = false;
    if (s_calib_mag_sample_count < CALIB_MAG_MIN_SAMPLES) {
        // KHÔNG nói "xoay lau hon" khi thu được ĐÚNG 0 mẫu — xoay thêm 1 tiếng
        // cũng vô ích, đó là lỗi phần cứng/driver chứ không phải thao tác. Phân
        // biệt rõ 3 tình huống (xem s_calib_mag_read_err_count/notready_count).
        if (s_calib_mag_sample_count == 0) {
            if (s_calib_mag_read_err_count > 0) {
                ESP_LOGE(TAG, "calib_mag: 0 mau - LOI DOC I2C (%u lan loi / %u lan bao chua san sang). "
                              "KHONG phai do xoay it -> kiem tra day/pull-up/nguon mag, xem 'status' "
                              "(mag_ok) va log boot mag_driver",
                          (unsigned)s_calib_mag_read_err_count, (unsigned)s_calib_mag_notready_count);
            } else {
                ESP_LOGE(TAG, "calib_mag: 0 mau - chip KHONG phat mau nao (%u tick deu DRDY=0/OVFL/all-zero, "
                              "0 loi I2C). KHONG phai do xoay it -> mag da ACK I2C nhung khong do; "
                              "xem log boot mag_driver (probe DRDY luc init)",
                          (unsigned)s_calib_mag_notready_count);
            }
        } else {
            ESP_LOGW(TAG, "calib_mag: qua it mau (%u < %d, loi I2C=%u, chua san sang=%u) -> HUY, KHONG luu "
                          "(xoay lau hon / xoay day du 3 truc hinh so 8)",
                      (unsigned)s_calib_mag_sample_count, CALIB_MAG_MIN_SAMPLES,
                      (unsigned)s_calib_mag_read_err_count, (unsigned)s_calib_mag_notready_count);
        }
        // Calib CŨ trong NVS (nếu có) KHÔNG bị xóa bởi phiên thất bại này —
        // s_calib.mag_valid giữ nguyên, nên 'status' vẫn báo mag đã calib.
        // Đúng theo thiết kế (thà giữ calib cũ còn hơn mất trắng), nhưng dễ
        // gây hiểu nhầm "đã calib rồi mà sao vẫn báo lỗi" -> nói rõ ở đây.
        if (s_calib.mag_valid) {
            ESP_LOGW(TAG, "calib_mag: GIU NGUYEN calib mag CU trong NVS (hard_iron=(%.0f,%.0f,%.0f)) - "
                          "phien that bai KHONG ghi de, 'status' van bao mag da calib",
                      s_calib.mag_hard_iron.x, s_calib.mag_hard_iron.y, s_calib.mag_hard_iron.z);
        }
        return;
    }
    const vec3f_t hard_iron = {
        (s_calib_mag_max.x + s_calib_mag_min.x) * 0.5f,
        (s_calib_mag_max.y + s_calib_mag_min.y) * 0.5f,
        (s_calib_mag_max.z + s_calib_mag_min.z) * 0.5f,
    };
    const vec3f_t range = {
        s_calib_mag_max.x - s_calib_mag_min.x,
        s_calib_mag_max.y - s_calib_mag_min.y,
        s_calib_mag_max.z - s_calib_mag_min.z,
    };
    const float avg_range = (range.x + range.y + range.z) / 3.0f;
    const vec3f_t soft_iron = {
        (range.x > 1.0f) ? avg_range / range.x : 1.0f,
        (range.y > 1.0f) ? avg_range / range.y : 1.0f,
        (range.z > 1.0f) ? avg_range / range.z : 1.0f,
    };
    s_calib.mag_hard_iron = hard_iron;
    s_calib.mag_soft_iron_scale = soft_iron;
    s_calib.mag_valid = true;
    calibration_save_mag(&hard_iron, &soft_iron);
    recompute_uncalibrated();
    ESP_LOGW(TAG, "MAG CALIB XONG (%u mau, loi I2C=%u, chua san sang=%u):",
              (unsigned)s_calib_mag_sample_count,
              (unsigned)s_calib_mag_read_err_count, (unsigned)s_calib_mag_notready_count);
    ESP_LOGW(TAG, "  min      = (%.0f, %.0f, %.0f)", s_calib_mag_min.x, s_calib_mag_min.y, s_calib_mag_min.z);
    ESP_LOGW(TAG, "  max      = (%.0f, %.0f, %.0f)", s_calib_mag_max.x, s_calib_mag_max.y, s_calib_mag_max.z);
    // range = độ phủ mỗi trục. 3 range CHÊNH NHAU NHIỀU = xoay chưa đều (trục
    // range nhỏ bị xoay thiếu) -> soft_iron sẽ méo. range gần bằng nhau và đủ
    // lớn mới là phiên calib tốt. range=0 trên 1 trục = trục đó chưa xoay tí nào.
    ESP_LOGW(TAG, "  range    = (%.0f, %.0f, %.0f)  <- 3 so nay nen GAN BANG NHAU",
              range.x, range.y, range.z);
    ESP_LOGW(TAG, "  hard_iron= (%.0f, %.0f, %.0f)  (offset tam hinh cau)",
              hard_iron.x, hard_iron.y, hard_iron.z);
    ESP_LOGW(TAG, "  soft_iron= (%.3f, %.3f, %.3f)  (he so chuan hoa truc, ly tuong ~1.0)",
              soft_iron.x, soft_iron.y, soft_iron.z);
    // Cảnh báo chất lượng: trục lệch quá nhiều so với trung bình -> xoay thiếu.
    const float min_range = fminf(range.x, fminf(range.y, range.z));
    const float max_range = fmaxf(range.x, fmaxf(range.y, range.z));
    if (max_range > 1.0f && (min_range / max_range) < CALIB_MAG_MIN_RANGE_RATIO) {
        ESP_LOGW(TAG, "  CHAT LUONG KEM: range min/max = %.2f (< %.2f) -> co truc xoay THIEU, "
                      "nen calib lai va xoay deu ca 3 truc",
                  min_range / max_range, CALIB_MAG_MIN_RANGE_RATIO);
    }
}

static void apply_set_param(const char *name, float value) {
    // TODO: mở rộng danh sách khi cần tune runtime từ Python (fc.set_param()).
    if (strncmp(name, "hover_duty", COMMAND_PARAM_NAME_MAX) == 0) {
        s_hold_tune.hover = value;
    } else if (strncmp(name, "prime_ms", COMMAND_PARAM_NAME_MAX) == 0) {
        s_tko_tune.prime_ms = (int)value;
    } else if (strncmp(name, "prime_duty", COMMAND_PARAM_NAME_MAX) == 0) {
        s_tko_tune.prime_duty = (int)value;
    } else if (strncmp(name, "max_climb", COMMAND_PARAM_NAME_MAX) == 0) {
        s_tko_tune.max_climb_ms = value;
    } else if (strncmp(name, "alt_frame", COMMAND_PARAM_NAME_MAX) == 0) {
        // B7 — chọn frame giữ độ cao LÚC RUNTIME. Đổi được cả khi đang bay:
        // không đụng vz_integral, chỉ đổi đại lượng ĐO đưa vào tầng alt, nên
        // ga không giật (chỉ có target hiệu dụng dịch đi bằng terrain_off_m).
        s_alt_frame = (value >= 0.5f) ? ALT_FRAME_AGL : ALT_FRAME_DATUM;
        ESP_LOGI(TAG, "alt_frame = %s (%s)",
                  (s_alt_frame == ALT_FRAME_AGL) ? "AGL" : "DATUM",
                  (s_alt_frame == ALT_FRAME_AGL)
                      ? "giu khoang cach so voi BE MAT — qua ban thi LEO LEN"
                      : "giu do cao so voi SAN cat canh — qua ban thi khoang ho GIAM");
    } else if (strncmp(name, "max_lean_deg", COMMAND_PARAM_NAME_MAX) == 0) {
        // Runtime-tunable riêng cho fc.control() (KHÔNG đụng SP_TILT_MAX_DEG
        // của ground-station UDP, xem tuning.h muc 7). Trần cứng 45deg — quá
        // đó vô nghĩa với MAX_SAFE_TILT_DEG failsafe (cắt motor ở 45deg).
        s_max_lean_deg = clampf(value, 1.0f, 45.0f);
    } else if (strncmp(name, "max_yawrate_dps", COMMAND_PARAM_NAME_MAX) == 0) {
        s_max_yawrate_dps = clampf(value, 1.0f, 360.0f);
    } else if (strncmp(name, "battery_comp_nominal_v", COMMAND_PARAM_NAME_MAX) == 0 ||
               strncmp(name, "battery_comp_max_gain", COMMAND_PARAM_NAME_MAX) == 0) {
        // Bù throttle theo pin ĐÃ BỎ HẲN (xem bước 9b). Vẫn NHẬN tên tham số
        // để script/GUI cũ gọi tới không ăn "khong ro ten" gây hiểu nhầm là gõ
        // sai — nhưng nói rõ nó KHÔNG còn tác dụng gì, thay vì im lặng chấp
        // nhận rồi để người dùng tưởng đã tune được cái gì.
        ESP_LOGW(TAG, "set_param('%s'): BO QUA — bu throttle theo pin da bo hoan toan "
                      "(VBAT tut theo tai tuc thi -> nhan vao throttle tao vong phan hoi "
                      "duong, xem flight_core.c buoc 9b). Pin chi con dung cho failsafe.", name);
    } else {
        ESP_LOGW(TAG, "set_param: khong ro ten '%s'", name);
    }
}

// ============================================================================
// PRE-ARM GUARD
// ============================================================================
// Ảnh chụp các điều kiện arm, cập nhật MỖI TICK ở bước 2b của stabilize_task.
//
// VÌ SAO CÓ STRUCT RIÊNG thay vì đọc s_telemetry: telemetry là ĐẦU RA cho
// Python/ground-station, không phải kho state nội bộ. apply_command() đọc
// ngược telemetry để quyết định arm là nối ngược chiều dữ liệu — sửa format
// telemetry hay thêm bộ lọc cho đẹp đồ thị sẽ vô tình đổi luôn logic bay.
static struct {
    bool  attitude_valid;
    float roll_deg, pitch_deg;
    bool  imu_fresh;
    bool  imu_healthy;
    bool  gyro_calibrated;
    bool  accel_calibrated;
    bool  mag_ok_for_heading;   // mag đọc được VÀ đã calib (chỉ CẢNH BÁO, không chặn)
    bool  battery_sample_ok;
    float battery_v;
    bool  alt_estimator_valid;
    bool  tof_floor_ready;
    bool  loop_healthy;         // không trượt deadline liên tục
} s_prearm;

// Yaw hiện tại — tách riêng khỏi s_prearm vì không phải điều kiện arm, chỉ là
// giá trị cần chốt làm mốc YAWREL lúc ARM thành công. Cập nhật cùng bước 2b.
static float s_prearm_yaw_deg = 0.0f;

// Lý do lệnh ARM gần nhất bị từ chối — publish ra telemetry (wire ARMREJ=) vì
// mọi ESP_LOGW ở đây CHỈ tới được console USB, không tới GUI qua UDP. Xem
// telemetry.h arm_reject_t.
static arm_reject_t s_arm_reject = ARM_REJECT_NONE;
static uint32_t     s_arm_reject_seq = 0;

// Lý do lệnh TAKEOFF gần nhất KHÔNG khởi động được chuỗi cất cánh — CÙNG lý do
// tồn tại như s_arm_reject: mọi điều kiện của CMD_TAKEOFF nằm trong
// apply_command() và trước đây chỉ báo bằng ESP_LOGW (console USB), trong khi
// command_parser.c đã trả lời "TAKEOFF started" dựa trên DỰ ĐOÁN chỉ xét FSM
// state. Nhìn từ GUI: bấm TAKEOFF, thấy báo thành công, không có gì xảy ra,
// không có lời giải thích nào. Xem telemetry.h takeoff_reject_t.
static takeoff_reject_t s_tko_reject = TAKEOFF_REJECT_NONE;
static uint32_t         s_tko_reject_seq = 0;

// prearm_check() — trả true nếu ĐỦ ĐIỀU KIỆN ARM; log rõ TỪNG lý do từ chối.
//
// Kiểm CẢ những thứ trước đây bỏ sót: IMU tươi/khoẻ, gyro+accel đã calib, pin
// hợp lệ và trên sàn, baro đã có mốc 0m, estimator đã hợp lệ, vòng điều khiển
// còn chạy đúng nhịp. Từng cái đều là thứ nếu thiếu thì drone vẫn ARM được
// nhưng sẽ hỏng NGAY SAU khi cất cánh — phát hiện lúc còn trên mặt đất rẻ hơn
// nhiều.
//
// MAG KHÔNG CHẶN ARM: firmware này bay bằng yaw-RATE (heading-hold kp=0 mặc
// định, xem attitude_control.c). Mag chết chỉ làm yaw trôi chậm, không làm mất
// điều khiển roll/pitch. Bắt cả hệ không arm được vì một cảm biến mà chế độ
// bay hiện tại không cần là chặn sai chỗ — chỉ CẢNH BÁO.
// prearm_check() — LƯU Ý: hàm này chỉ đọc s_prearm + state toàn cục, KHÔNG
// nhận tham số. now_us truyền vào CHỈ để đo tuổi heartbeat (xem dưới).
static bool prearm_check(int64_t now_us) {
    bool ok = true;

    // record_reject() — ghi lý do ĐẦU TIÊN thất bại. Vẫn log ĐỦ mọi lý do ra
    // console như trước (người có console muốn thấy HẾT danh sách cần sửa),
    // nhưng wire chỉ chở một mã — cái đầu tiên là cái người dùng phải sửa trước.
    #define record_reject(r) do { if (s_arm_reject == ARM_REJECT_NONE) s_arm_reject = (r); } while (0)

    // ---- Heartbeat của nguồn điều khiển NGOÀI ----
    // Kiểm NGAY TẠI ĐÂY thay vì để Commander phát hiện sau khi đã ARM. Nếu
    // heartbeat đã quá hạn lúc bấm ARM thì lần ARM đó CHẮC CHẮN chết ở tick kế
    // (commander_evaluate -> SOFT fault -> fsm_on_soft_fault(ARMED) = DISARMED)
    // — người dùng chỉ thấy "vừa arm xong tự disarm" mà không có lý do nào.
    //
    // Đây là hệ quả trực tiếp của việc firmware KHÔNG còn tự sinh heartbeat
    // (xem src/main.c): bằng chứng "còn nguồn điều khiển" giờ phải đến từ ngoài
    // chip, nên trạng thái "chưa ai ping" là trạng thái CÓ THẬT và cần một câu
    // trả lời rõ ràng, không phải một cú disarm im lặng.
    // ---- CỔNG HEARTBEAT: ĐÃ BỎ theo yêu cầu người dùng ----
    // Trước đây từ chối ARM khi heartbeat cũ hơn heartbeat_timeout_ms, tức GUI
    // phải đang kết nối mới arm được.
    //
    // ⚠ CÁI CÒN LẠI SAU KHI BỎ: heartbeat watchdog lúc ĐANG BAY vẫn nguyên
    // (commander.c) — mất liên lạc giữa chuyến vẫn auto-land như cũ. Chỗ này chỉ
    // bỏ điều kiện LÚC ARM, không đụng tới failsafe khi đã bay.
    //
    // Đánh đổi: arm được khi chưa có nguồn điều khiển nào sẵn sàng. Nếu bay bằng
    // console USB thì đó chính là điều mình muốn; nếu bay bằng GUI thì lệnh
    // takeoff sẽ tự tới ngay sau đó nên khoảng hở gần như bằng 0.

    if (!s_prearm.attitude_valid) {
        ESP_LOGW(TAG, "ARM tu choi: attitude CHUA hop le (Mahony chua init — de yen drone vai giay)");
        record_reject(ARM_REJECT_ATTITUDE_INVALID);
        ok = false;
    }
    // ---- CỔNG NGHIÊNG (FSM_ARM_MAX_TILT_DEG): ĐÃ BỎ theo yêu cầu người dùng ----
    // Trước đây từ chối ARM khi |roll| hoặc |pitch| vượt ngưỡng.
    //
    // ⚠ ĐÁNH ĐỔI THẬT, phải biết: giờ ARM được trên mặt nghiêng/mấp mô. Lúc
    // takeoff, PRIME sẽ đẩy ga lên trong khi khung đã nghiêng sẵn -> drone có
    // xu hướng trượt/lật về phía thấp thay vì bay thẳng lên.
    //
    // Cái CÒN LẠI đỡ cho tình huống đó: TKO_ABORT_TILT trong takeoff_land.c vẫn
    // huỷ cất cánh khi nghiêng vượt ngưỡng lúc đang leo, và Commander vẫn có
    // "tilt exceeded hard limit". Tức là mình đổi "chặn trước" lấy "bắt giữa
    // chừng" — vẫn có lưới, nhưng lưới đặt muộn hơn một bước.

    if (!s_prearm.imu_fresh || !s_prearm.imu_healthy) {
        ESP_LOGW(TAG, "ARM tu choi: IMU %s (sensor_hub khong cap mau moi — kiem tra bus I2C/day INT)",
                  !s_prearm.imu_fresh ? "STALE" : "KHONG KHOE");
        record_reject(ARM_REJECT_IMU);
        ok = false;
    }

    // Cấu hình MPU6050 phải TIN ĐƯỢC trước mọi thứ khác: scale sai thì mọi số
    // dps/g đều sai theo tỷ lệ, và cả gyro bias lẫn ngưỡng nghiêng bên dưới
    // đều mất ý nghĩa. Kiểm TRƯỚC gyro calib vì nó là tiền đề của gyro calib.
    {
        imu_cfg_readback_t cfg;
        imu_driver_get_config(&cfg);
        if (!cfg.valid) {
            ESP_LOGW(TAG, "ARM tu choi: MPU6050 CONFIG read-back KHONG HOP LE "
                          "(GYRO_CONFIG=0x%02X ACCEL_CONFIG=0x%02X) -> scale khong tin duoc. "
                          "Xem log 'MPU6050 CFG' luc boot.", cfg.gyro_config, cfg.accel_config);
            record_reject(ARM_REJECT_IMU_CFG);
            ok = false;
        }
    }

    if (!s_prearm.gyro_calibrated) {
        // Phân biệt hai nguyên nhân RẤT khác nhau: startup calib đã chạy và
        // TRƯỢT (có lý do cụ thể) vs chưa từng có calib nào. Gộp chung thành
        // "chưa calib" sẽ khiến người dùng gọi lại calib_gyro mà không biết
        // lần trước hỏng vì drone bị rung.
        if (s_gcal_state == GCAL_FAIL) {
            ESP_LOGW(TAG, "ARM tu choi: fresh gyro calib THAT BAI code=%d (%s) — dat drone that yen "
                          "tren mat phang roi go 'calib_gyro' de do lai",
                     (int)s_gcal_fail, s_gcal_fail_reason);
        } else if (gyro_cal_state_active(s_gcal_state)) {
            ESP_LOGW(TAG, "ARM tu choi: fresh gyro calib DANG CHAY state=%d — cho, GIU YEN drone",
                     (int)s_gcal_state);
        } else {
            ESP_LOGW(TAG, "ARM tu choi: gyro CHUA calib — chay 'calib_gyro' (drone dung yen)");
        }
        record_reject(ARM_REJECT_GYRO_CALIB);
        ok = false;
    }

    // ========================================================================
    // GYRO BIAS HEALTH — BẬT LẠI, ngưỡng 1.0 dps (yêu cầu người dùng)
    // ========================================================================
    // Đây là LƯỚI AN TOÀN DUY NHẤT còn lại cho gyro bias:
    //   - pha VALIDATE của calib đã bị bỏ
    //   - bias NVS là authoritative (calib 1 lần, boot sau chỉ nạp) nên KHÔNG
    //     có lần đo mới nào ở boot để mà tin
    // Calib PASS chỉ chứng minh mean(raw) - bias = 0 TẠI LÚC ĐÓ. Nó không nói
    // gì về nhiệt độ hôm nay, và cũng không phân biệt được "đứng yên" với
    // "đang xoay đều" (xoay đều bị hấp thụ vào chính bias, std vẫn thấp).
    // Chỗ này là nơi DUY NHẤT còn đo gyro corrected THẬT ngay trước khi bay.
    //
    // CỐ Ý KHÔNG bật lại ARM_REJECT_GYRO_NOT_STATIONARY (cổng "chưa gom đủ cửa
    // sổ thì chưa cho ARM"): đó đúng là loại ràng buộc gây khó chịu mà không
    // thêm an toàn — chưa đủ mẫu nghĩa là CHƯA BIẾT, không phải ĐÃ HỎNG. Chưa
    // đủ mẫu -> bỏ qua kiểm tra này, cửa sổ 100ms sẽ đầy sau vài tick.
    if (s_prearm_gyro_mean_valid) {
        // Trung bình CẢ cửa sổ PREARM_GYRO_WINDOW_MS, không phải một mẫu
        // (mục 18: "Do not fail based on one noisy sample").
        const vec3f_t m = s_prearm_gyro_mean;
        if (fabsf(m.x) > PREARM_GYRO_MAX_MEAN_DPS ||
            fabsf(m.y) > PREARM_GYRO_MAX_MEAN_DPS ||
            fabsf(m.z) > PREARM_GYRO_MAX_MEAN_DPS) {
            ESP_LOGW(TAG, "ARM tu choi: gyro corrected khi dung yen van lech "
                          "(%.3f,%.3f,%.3f)dps > %.2f — bias chua duoc ap, da troi theo "
                          "nhiet do, hoac drone dang bi cham. Go 'calib_gyro' de do lai, "
                          "hoac 'cal_status' de xem GRAW/GBIAS/GCORR.",
                      (double)m.x, (double)m.y, (double)m.z, (double)PREARM_GYRO_MAX_MEAN_DPS);
            record_reject(ARM_REJECT_GYRO_BIAS_LARGE);
            ok = false;
        }
    }
    if (!s_prearm.accel_calibrated) {
        ESP_LOGW(TAG, "ARM tu choi: accel CHUA calib 6-face — chay 'calib_accel_face' x6");
        record_reject(ARM_REJECT_ACCEL_CALIB);
        ok = false;
    }

    // Pin: 0.0f = chưa có mẫu. KHÔNG coi "chưa có mẫu" là lỗi nếu ADC pin bị
    // tắt hẳn theo app_config (s_battery_ok_driver=false) — nhưng nếu driver
    // CÓ mà không đọc được thì đó là hỏng thật.
    if (s_battery_ok_driver) {
        if (!s_prearm.battery_sample_ok || s_prearm.battery_v <= 1.0f) {
            ESP_LOGW(TAG, "ARM tu choi: KHONG doc duoc dien ap pin (ADC loi?)");
            record_reject(ARM_REJECT_BATTERY_SAMPLE);
            ok = false;
        } else if (s_prearm.battery_v < s_cmd_cfg.battery_floor_v) {
            ESP_LOGW(TAG, "ARM tu choi: pin %.2fV DUOI san %.2fV — sac truoc khi bay",
                      (double)s_prearm.battery_v, (double)s_cmd_cfg.battery_floor_v);
            record_reject(ARM_REJECT_BATTERY_LOW);
            ok = false;
        }
    }

    // Flight-control Z chi dung IMU + ToF. Barometer khong duoc phep chan ARM.
    //
    // FC_FEATURE_FLOOR_GATE=0 (mac dinh, theo yeu cau nguoi dung): BO cong
    // chan nay. Va may do san cung DA BI BO HAN -- ToF do tuyet doi, xem
    // alt_estimator.c::update(). Khoi #if duoi day chi con la duong quay lai
    // neu ai do bat co len; no se KHONG chay dung nua vi floor_ready() gio
    // luon true.
#if FC_FEATURE_FLOOR_GATE
    if (!s_prearm.tof_floor_ready || !s_prearm.alt_estimator_valid) {
        // In DU SO LIEU: khong co no thi "chua co mau floor hop le" khong phan
        // biet duoc 3 nguyen nhan hoan toan khac nhau — ToF chet (n=0), drone
        // dang bi rung/cam tay (std lon), hay estimator invalid vi ly do khac.
        ESP_LOGW(TAG, "ARM tu choi: floor ToF chua chot duoc — n=%u/%d std=%.4f (tran %.4f) "
                      "ref_valid=%d est_valid=%d. Dat drone DUNG YEN tren mat phang cung, "
                      "cho ~1s. std lon = dang cam tay/rung; n=0 = ToF khong ra mau.",
                  (unsigned)s_alt_est.floor_sample_count, ALT_EST_FLOOR_MIN_SAMPLES,
                  (double)s_alt_est.floor_std_m, (double)ALT_EST_FLOOR_MAX_STD_M,
                  (int)s_alt_est.tof_ground_ref_valid, (int)s_prearm.alt_estimator_valid);
        record_reject(ARM_REJECT_ALT_EST_INVALID);
        ok = false;
    }
#else
    // ToF do TUYET DOI (khong con goc toa do, xem alt_estimator.c) nen khong
    // con khai niem "da chot duoc mat san chua". alt_estimator_floor_ready()
    // gio luon true, nen canh bao cu o day la code chet -- va te hon, no in ra
    // n=0 std=0.0000 khien nguoi doc tuong ToF dang hong.
#endif

    if (!s_prearm.loop_healthy) {
        ESP_LOGW(TAG, "ARM tu choi: vong dieu khien dang TRE HAN lien tuc (%d tick) — "
                      "khong arm khi timing chua on", s_deadline_miss_streak);
        record_reject(ARM_REJECT_LOOP_UNHEALTHY);
        ok = false;
    }
    #undef record_reject

    // CẢNH BÁO, KHÔNG CHẶN — xem docstring.
    if (s_board.mag_enabled && !s_prearm.mag_ok_for_heading) {
        ESP_LOGW(TAG, "ARM: mag KHONG dung duoc -> yaw se CHI dua vao gyro va troi cham "
                      "(heading_degraded=1). Van cho ARM vi che do bay hien tai dung yaw-RATE.");
    }

    return ok;
}

static bool abort_landing_to_hold(const char *reason, int64_t now_us) {
    const fsm_state_t next = fsm_on_landing_abort_hold(s_fsm.state);
    if (next == s_fsm.state) return false;

    // landing và HOLD dùng chung vz_integral; không reset controller để tránh
    // giật ga. Chỉ neo target tại độ cao hiện tại và hủy state landing.
    s_alt_target_m = commander_clamp_altitude(&s_cmd_cfg, s_alt_est.alt_m);
    s_alt_request_m = s_alt_target_m;
    s_timed_active = false;
    landing_reset(&s_land_state);
    fsm_transition(&s_fsm, next, now_us);
    ESP_LOGW(TAG, "LANDING ABORT -> HOLDING (%s), target=%.2fm, giu Vz-I bumpless",
             reason, (double)s_alt_target_m);
    return true;
}

static void apply_command(const command_t *cmd, int64_t now_us) {
    switch (cmd->type) {
        case CMD_ARM: {
            // Xoá lý do của lần trước NGAY ĐẦU mỗi lần thử: nếu không thì lý do
            // cũ dính lại và GUI hiện sai (vừa sửa xong vẫn thấy lỗi cũ).
            // arm_reject_seq chỉ tăng khi THẬT SỰ bị từ chối (ở cuối case).
            s_arm_reject = ARM_REJECT_NONE;

            if (s_uncalibrated) {
                // CHỈ accel/gyro là bắt buộc — mag KHÔNG còn (xem
                // recompute_uncalibrated()). Không nhắc calib_mag ở đây nữa để
                // người dùng không đi làm một bước không cần thiết.
                ESP_LOGW(TAG, "ARM tu choi: CHUA CALIBRATE (thieu accel hop le trong NVS) "
                              "- chay 'calib_accel_face' x6 truoc (mag KHONG bat buoc)");
                s_arm_reject = ARM_REJECT_UNCALIBRATED;
                s_arm_reject_seq++;
                break;
            }
            // prearm_check() ĐỌC s_prearm (state nội bộ, cập nhật ở bước 2b) —
            // KHÔNG đọc ngược s_telemetry như bản cũ. Telemetry là đầu ra cho
            // Python/ground-station, không phải kho state để logic bay hỏi lại.
            const bool guard = prearm_check(now_us);
            fsm_state_t next = fsm_on_arm_request(s_fsm.state, guard);
            if (next != s_fsm.state) {
// ---- LATCH GA HOVER: ĐÃ CHUYỂN SANG CMD_TAKEOFF (yêu cầu người dùng) ----
                // Trước đây latch tại ĐÂY (lúc ARM). Xem case CMD_TAKEOFF để
                // biết khối đó giờ nằm ở đâu và đánh đổi kèm theo.

#if FC_FEATURE_HOVER_LATCH
                // ============================================================
                // LATCH GA HOVER THEO ĐIỆN ÁP PIN — xem hover_model.h
                // ============================================================
                // Đặt ở ĐÂY (lúc ARM) vì motor CHƯA quay (FSM còn DISARMED,
                // armed gate của driver chưa mở, throttle khoá 0) -> vbat đo
                // được là điện áp KHÔNG TẢI. Đó CHÍNH LÀ đại lượng model cần.
                //
                // ĐÃ THỬ đặt ở CMD_TAKEOFF rồi TRẢ VỀ ĐÂY: ở đó không bảo đảm
                // được "không tải" (vừa chạy BENCH_RAMP/test_motor thì pin còn
                // đang hồi), và latch nhằm vào số đã sụt sẽ cho hover_ff CAO
                // GIẢ TẠO.
                //
                // ⚠ ĐIỀU LATCH KHÔNG GIẢI QUYẾT ĐƯỢC: nó chốt theo điện áp lúc
                // ARM, nhưng pin sụt dưới tải khi bay. Log đo được sụt 0.38V ở
                // ~1200 duty — latch 3.74V ra 1219 trong khi hover thật ở 3.36V
                // cần ~1608. Phần thiếu do I-term của vòng Vz bù. Pin càng chai
                // (nội trở cao) thì khoảng hở này càng lớn.
                {
                    float vlatch = 0.0f;
                    if (!hover_vbat_median(&s_vbat_ring, &vlatch)) {
                        ESP_LOGE(TAG, "ARM tu choi: chua du mau pin de chot ga hover "
                                      "(can %d mau hop le, dang co %d). ADC pin chay 10Hz — "
                                      "cho ~0.5s roi TAKEOFF lai. Neu KHONG tu het thi ADC pin hong that.",
                                  HOVER_MODEL_MIN_SAMPLES, s_vbat_ring.count);
                        s_arm_reject = ARM_REJECT_HOVER_LATCH_NO_SAMPLE;
                        s_arm_reject_seq++;
                        break;
                    }
                    if (vlatch < HOVER_MODEL_MIN_LATCH_V) {
                        ESP_LOGE(TAG, "ARM tu choi: pin %.2fV duoi %.2fV — SAC PIN. "
                                      "Duoi nguong nay model hover ngoai suy ra ga vuot tran "
                                      "collective (%.0f duty), tuc hover_ff se bi clamp THAP HON "
                                      "hover that -> drone nam i khong nhac noi.",
                                  (double)vlatch, (double)HOVER_MODEL_MIN_LATCH_V,
                                  (double)HOVER_MODEL_MAX_DUTY);
                        s_arm_reject = ARM_REJECT_HOVER_LATCH_VOLT_LOW;
                        s_arm_reject_seq++;
                        break;
                    }

                    const float hover_duty = hover_model_from_voltage(vlatch);
                    const int   prime_duty = hover_model_prime_duty(hover_duty);

                    // GHI ĐÈ tune đang dùng. Từ đây tới lần TAKEOFF sau, hai giá
                    // trị này ĐÓNG BĂNG — không có đường nào cập nhật chúng theo
                    // vbat trong lúc bay (vòng phản hồi dương đã bị gỡ có chủ
                    // đích, xem bước 9b).
                    s_hold_tune.hover     = hover_duty;
                    s_tko_tune.prime_duty = prime_duty;

                    s_hover_latch_v    = vlatch;
                    s_hover_latch_duty = hover_duty;
                    s_hover_latched    = true;

                    ESP_LOGI(TAG, "ARM: CHOT GA HOVER theo pin — vbat=%.2fV (trung vi %d mau) "
                                  "-> hover_ff=%.0f duty, prime=%d duty (%.0f%% hover). "
                                  "Dong bang suot chuyen bay; pin tut dan se do I cua vong Vz bu.",
                              (double)vlatch, s_vbat_ring.count, (double)hover_duty, prime_duty,
                              (double)(TAKEOFF_PRIME_HOVER_FRAC * 100.0f));

                    if (hover_duty >= HOVER_MODEL_MAX_DUTY - 0.5f) {
                        ESP_LOGW(TAG, "ARM: hover_ff da CHAM TRAN %.0f duty — model doi cao hon "
                                      "nhung tran collective khong cho. Ga con lai cho mixer rat "
                                      "it, drone se leo yeu. SAC PIN.", (double)HOVER_MODEL_MAX_DUTY);
                    } else if (hover_duty <= HOVER_MODEL_MIN_DUTY + 0.5f) {
                        ESP_LOGW(TAG, "ARM: hover_ff da CHAM SAN %.0f duty (pin %.2fV cao hon diem "
                                      "do goc %.1fV?) — kiem tra lai thang do dien ap.",
                                  (double)HOVER_MODEL_MIN_DUTY, (double)vlatch,
                                  (double)HOVER_MODEL_V_REF);
                    }
                }
#endif  // FC_FEATURE_HOVER_LATCH

                // Floor ToF da duoc estimator gom lien tuc khi DISARMED.

                // ARM là đường DUY NHẤT hạ kill latch — có chủ đích. Sau một
                // lần KILL/hard fault, phải có hành động ARM tường minh của
                // người dùng mới bay lại được, KHÔNG tự phục hồi theo thời
                // gian hay theo việc fault "hết" (xem enter_kill_latch()).
                if (s_motor_kill_latched) {
                    ESP_LOGW(TAG, "ARM: ha kill latch (ly do cu: %s)", s_kill_reason);
                    s_motor_kill_latched = false;
                    s_kill_reason = "";
                    // Xoá LUÔN cờ benign: để sót true ở đây thì lần latch sau
                    // (enter_kill_latch_ex ghi cờ TRƯỚC khi set latch nên thực
                    // ra vẫn đúng, nhưng không dựa vào thứ tự đó) hoặc bất kỳ
                    // đoạn code nào đọc s_kill_benign khi !latched đều thấy giá
                    // trị cũ vô nghĩa.
                    s_kill_benign = false;
                }
                motor_driver_arm();   // mở LUÔN armed gate của driver
                reset_all_controllers();
                fsm_transition(&s_fsm, next, now_us);
                // Mốc YAWREL mới — lấy từ state NỘI BỘ (s_prearm, cập nhật ở
                // bước 2b), KHÔNG đọc ngược s_telemetry. Telemetry chỉ đi một
                // chiều ra ngoài; logic bay hỏi lại nó là nối ngược chiều dữ
                // liệu (đổi format/lọc telemetry sẽ vô tình đổi hành vi bay).
                s_yaw_ref_offset_deg = s_prearm_yaw_deg;
                s_last_fault_class = FAULT_NONE;               // ARM mới -> xóa fault cũ khỏi telemetry
                ESP_LOGI(TAG, "ARM OK -> FSM_ARMED");
            } else if (!guard) {
                // prearm_check() đã log CHI TIẾT lý do (và ghi s_arm_reject) —
                // ở đây chỉ tổng kết, KHÔNG ghi đè lý do cụ thể bằng một lý do
                // chung chung.
                ESP_LOGW(TAG, "ARM tu choi: prearm_check() KHONG pass (xem cac dong 'ARM tu choi' ngay tren)");
                s_arm_reject_seq++;
            } else {
                // ---- NHÁNH TRƯỚC ĐÂY IM LẶNG HOÀN TOÀN ----
                // guard PASS nhưng fsm_on_arm_request() không cho đi: nghĩa là
                // FSM KHÔNG ở DISARMED (chỉ DISARMED -> ARMED là hợp lệ, xem
                // flight_state_machine.c). Trước đây không có log nào ở đây nên
                // lệnh ARM biến mất không dấu vết — người dùng bấm ARM, console
                // reply "ARMED" (reply đó chỉ đoán theo attitude guard, xem
                // command_parser.c case 'r'), rồi KHÔNG có gì xảy ra và KHÔNG có
                // cách nào biết tại sao. Đây là bản sửa.
                ESP_LOGW(TAG, "ARM tu choi: FSM dang o %s, chi ARM duoc tu DISARMED. "
                              "Dung 'k' (KILL) de ve DISARMED roi ARM lai.",
                          fsm_state_name(s_fsm.state));
                s_arm_reject = ARM_REJECT_STATE;
                s_arm_reject_seq++;
            }
            break;
        }
        case CMD_DISARM: {
            fsm_state_t next = fsm_on_disarm_request(s_fsm.state);
            if (next != s_fsm.state) {
                // DISARM cũng LATCH (không chỉ disarm motor): theo yêu cầu an
                // toàn "DISARM command -> motor_kill_latched = true". Chỉ hợp
                // lệ từ FSM_ARMED/FSM_BENCH_RAMP (chưa bay, xem
                // fsm_on_disarm_request()) nên latch ở đây không cắt máy giữa
                // trời — muốn hạ khi đang bay thì dùng LAND, muốn cắt ngay bất
                // kể state thì dùng KILL.
                //
                // benign=true: đây là hành động CHỦ ĐÍCH của người dùng trên
                // mặt đất, không phải sự cố. Latch vẫn chặn bay y hệt; chỉ
                // bench-test (CMD_TEST_MOTOR) được phép đi tiếp — nếu không thì
                // ARM->DISARM là ngõ cụt, không còn đường nào test motor nữa.
                enter_kill_latch_ex("CMD_DISARM", true);
                reset_all_controllers();
                alt_estimator_unlock_floor(&s_alt_est);
                fsm_transition(&s_fsm, next, now_us);
            }
            break;
        }
        case CMD_KILL: {
            // BYPASS toàn bộ FSM guard — cắt motor + ép DISARMED bất kể state
            // hiện tại (xem comment CMD_KILL trong command.h). fsm_transition()
            // là primitive KHÔNG validate topology (khác các fsm_on_*_request()
            // có guard) — dùng thẳng ở đây đúng ý nghĩa "override khẩn cấp".
            ESP_LOGW(TAG, "KILL: cat motor + DISARM cuong buc tu state=%s",
                      fsm_state_name(s_fsm.state));
            enter_kill_latch("CMD_KILL");
            reset_all_controllers();
            alt_estimator_unlock_floor(&s_alt_est);
            fsm_transition(&s_fsm, FSM_DISARMED, now_us);
            s_sp_roll_deg = s_sp_pitch_deg = s_sp_yaw_rate_dps = 0.0f;
            break;
        }
        case CMD_TAKEOFF: {
            if (s_fsm.state == FSM_LANDING) {
                // Go-around trên không: KHÔNG chạy lại PRIME mặt đất. Về HOLD
                // bumpless rồi dùng target của lệnh để leo bằng cascade Z/Vz.
                const float requested_m = commander_clamp_altitude(
                    &s_cmd_cfg, (float)cmd->as.takeoff.alt_mm * 0.001f);
                if (abort_landing_to_hold("CMD_TAKEOFF go-around", now_us)) {
                    if (requested_m > s_alt_request_m) s_alt_request_m = requested_m;
                }
                break;
            }
            // Xoá lý do lần trước NGAY ĐẦU mỗi lần thử (cùng quy ước với
            // CMD_ARM): không thì lý do cũ dính lại và GUI hiện sai sau khi
            // người dùng đã sửa xong. _seq chỉ tăng khi THẬT SỰ bị từ chối.
            s_tko_reject = TAKEOFF_REJECT_NONE;

            fsm_state_t next = fsm_on_takeoff_request(s_fsm.state);
            if (next == s_fsm.state) {
                // Nhánh này TRƯỚC ĐÂY IM LẶNG HOÀN TOÀN — không log, không cờ.
                ESP_LOGW(TAG, "TAKEOFF tu choi: FSM dang o %s, chi TAKEOFF duoc tu ARMED.",
                          fsm_state_name(s_fsm.state));
                s_tko_reject = TAKEOFF_REJECT_STATE;
                s_tko_reject_seq++;
            }
            if (next != s_fsm.state) {
// ---- LATCH GA HOVER: DA TRA VE CMD_ARM (yeu cau nguoi dung) ----
                // Xem case CMD_ARM. Ly do: o ARM motor CHUA quay nen vbat do
                // duoc la dien ap KHONG TAI — dung dai luong ma model can.

                // KHÔNG calib baro ở đây nữa — mốc 0m đã được chốt lúc ARM
                // (xem case CMD_ARM, nơi calib là BẮT BUỘC và ARM bị từ chối
                // nếu thất bại). Bỏ đi có 2 cái lợi thật: (1) bấm takeoff phản
                // hồi TỨC THÌ, không còn treo ~1s; (2) không còn khả năng
                // reanchor làm valid=false đúng lúc chuẩn bị vào TKO_PRIME.
                //
                // Baro vẫn được dùng làm anchor correction trong estimator y
                // như trước — chỗ này chỉ bỏ việc LẤY LẠI MỐC, không đụng gì
                // tới fusion (xem alt_estimator.c).
                //
                // Thay bằng guard ĐỌC state: mốc chốt lúc ARM có thể đã hỏng
                // trong khoảng ARM -> TAKEOFF (baro rớt khỏi bus, timeout mẫu
                // mới...). Chặn NGAY LÚC BẤM còn hơn để nó cất cánh rồi bị
                // Commander ép LANDING giữa chừng.
                // ĐÒI ÍT NHẤT MỘT nguồn correction, KHÔNG đòi riêng baro.
                // Bản trước hard-code `if (!s_baro_ok_driver) từ chối`, nghĩa là
                // tắt baro thì KHÔNG BAO GIỜ cất cánh được dù ToF hoạt động tốt.
                // Giờ ToF là nguồn correction đầy đủ (xem alt_estimator.h), nên
                // điều kiện đúng là "có ít nhất một thứ sửa được trôi".
                //
                // ⚠ ToF-only là cấu hình BỊ GIỚI HẠN, không phải tương đương:
                // ToF hết tầm trên ~1.8m và TỰ TẮT khi nhìn bề mặt khác (bàn/
                // ghế). Lúc đó không còn nguồn nào -> Commander trip degraded
                // sau ALT_EST_NO_CORRECTION_DEGRADED_MS -> LANDING. Cảnh báo rõ
                // ở đây thay vì để người dùng phát hiện giữa không trung.
                // ---- LẤY LẠI ground-ref ToF nếu lúc ARM chưa lấy được ----
                // ARM chụp ground-ref bằng ĐÚNG MỘT mẫu snapshot. Với ToF chập
                // chờn (mất rồi tự nối lại), nếu khoảnh khắc ARM rơi vào đúng
                // cửa sổ mất mẫu thì tof_ground_ref_valid ở lại false VĨNH VIỄN
                // — ARM vẫn báo thành công (chỉ log cảnh báo), rồi MỌI lệnh
                // TAKEOFF sau đó bị từ chối NO_CORRECTION. Đường thoát duy nhất
                // trước đây là disarm/arm lại cho tới khi trúng lúc ToF tỉnh,
                // và người dùng không có cách nào biết vì sao.
                //
                // Ở đây drone vẫn đang nằm yên trên sàn (FSM_ARMED, chưa có lực
                // nâng nào), nên đây là thời điểm lấy mốc sàn HỢP LỆ Y HỆT lúc
                // ARM. Đọc từ snapshot của hub, KHÔNG chạm I2C, không block.
                // FC_FEATURE_FLOOR_GATE=0 -> KHONG doi floor_ready nua (xem
                // fc_features.h). Van doi s_tof_ok_driver: khong co ToF song
                // thi khong co nguon Z nao ca, cat canh la bay mu hoan toan.
#if FC_FEATURE_FLOOR_GATE
                const bool has_tof_src = s_tof_ok_driver && alt_estimator_floor_ready(&s_alt_est);
#else
                const bool has_tof_src = s_tof_ok_driver;
                if (s_tof_ok_driver && !alt_estimator_floor_ready(&s_alt_est)) {
                    ESP_LOGW(TAG, "TAKEOFF: floor ToF chua chot nhung cong floor DANG TAT "
                                  "(FC_FEATURE_FLOOR_GATE=0) -> van cat canh. Do cao co the lech.");
                }
#endif
                // ⚠ KHONG con tu choi TAKEOFF vi "chua co mau ToF dung duoc".
                //
                // Nam tren san, ToF doc 0.000m (duoi tam mu ~4cm cua VL53L1X)
                // -> driver loai mau (dist_mm > 0 sai) -> floor_add() khong
                // bao gio duoc goi -> tof_ground_ref_valid dung o false ->
                // MOI lenh TAKEOFF bi tu choi NO_CORRECTION, vinh vien.
                // Nguoi dung khong co duong thoat: disarm/arm lai cung the,
                // vi drone van dang nam dung cho do.
                //
                // GIU LAI dieu kien DUY NHAT that su la loi phan cung:
                // s_tof_ok_driver = driver init duoc chip hay khong. Sai =
                // khong co ToF tren bus, luc do tu choi moi dung.
                if (!s_tof_ok_driver) {
                    ESP_LOGW(TAG, "TAKEOFF tu choi: ToF khong init duoc (khong thay chip tren I2C) "
                                  "-> khong co nguon Z; chay 'i2c_scan' va 'tof_test'.");
                    s_tko_reject = TAKEOFF_REJECT_NO_CORRECTION;
                    s_tko_reject_seq++;
                    break;
                }
                if (!has_tof_src) {
                    ESP_LOGW(TAG, "TAKEOFF: chua co mau ToF on dinh (nam sat san?) -> VAN cat canh, "
                                  "goc toa do se duoc chot bang mau dau tien do duoc.");
                }
                if (!s_alt_est.valid) {
                    ESP_LOGW(TAG, "TAKEOFF tu choi: alt_estimator KHONG hop le (state khong huu han) — "
                                  "ARM lai de reset");
                    s_tko_reject = TAKEOFF_REJECT_ALT_EST_INVALID;
                    s_tko_reject_seq++;
                    break;
                }
                // Precondition còn lại (attitude/tilt/IMU) đã được canh TRƯỚC
                // khi tới đây: fsm_on_takeoff_request() chỉ cho đi từ FSM_ARMED
                // — mà vào được ARMED thì prearm_check() đã pass. KHÔNG kiểm
                // lại ở đây để không có hai bộ điều kiện song song lệch nhau.
                //
                // Kill latch KHÔNG cần kiểm ở đây (và KHÔNG được chặn ở đầu
                // apply_command() như comment cũ nói sai): latch chỉ hạ bằng
                // CMD_ARM tường minh, mà TAKEOFF lại đòi FSM_ARMED — nên latch
                // còn đóng thì không thể đang ở ARMED. Nếu có kill NGAY giữa
                // ARMED và tick này thì motor đã bị cắt cứng ở bước ghi duty
                // (không đường code nào ghi được duty khác 0) và FSM bị ép về
                // DISARMED ở tick kế — chuỗi takeoff tự chết, không bay được.

                // ---- TARGET ĐỘ CAO ĐẾN TỪ LỆNH (alt_mm) ----
                // Bản trước BỎ QUA alt_mm và giữ "độ cao tình cờ đạt được lúc
                // hết ram ga" (~1-2cm). Giờ một lệnh fc.takeoff(800) là ĐỦ:
                // firmware tu chay PRIME -> CLIMB -> HOLD -> HOLDING(0.8m),
                // Python KHÔNG cần gửi thêm set_altitude.
                //
                // alt_mm <= 0 -> dùng geofence min hợp lý thay vì bay lên 0m
                // (target 0 nghĩa là "cất cánh rồi hạ ngay", vô nghĩa). Lấy
                // ALT_HOLD_MIN_ENGAGE_M làm sàn: dưới mức đó alt_hold còn không
                // engage được nên bàn giao sẽ thất bại ngay.
                float tko_target_m = (float)cmd->as.takeoff.alt_mm * 0.001f;
                if (tko_target_m <= 0.0f) {
                    tko_target_m = TAKEOFF_DEFAULT_TARGET_M;
                    ESP_LOGW(TAG, "TAKEOFF: alt khong hop le trong lenh -> dung san %.2fm",
                              (double)tko_target_m);
                }
                // Clamp geofence — CÙNG hàm Commander dùng, không tự viết lại
                // ngưỡng (lệch nhau thì drone leo tới độ cao mà Commander coi là
                // vi phạm rồi tự ép LANDING).
                tko_target_m = commander_clamp_altitude(&s_cmd_cfg, tko_target_m);

                // Event chuẩn bị estimator (xem alt_estimator.h): zero Z/Vz +
                // XOÁ trạng thái reacquire baro cũ. GIỮ accel_bias đã học,
                // GIỮ valid, KHÔNG chạm Mahony.
                if (!alt_estimator_lock_floor(&s_alt_est)) {
#if FC_FEATURE_FLOOR_GATE
                    ESP_LOGW(TAG, "TAKEOFF tu choi: floor ToF mat validity truoc luc lock");
                    s_tko_reject = TAKEOFF_REJECT_NO_CORRECTION;
                    s_tko_reject_seq++;
                    break;
#else
                    // Cong floor DANG TAT -> khong tu choi. Chot goc toa do
                    // bang mau ToF hop le HIEN CO (fallback), thay vi bang
                    // trung binh cua so mau on dinh nhu duong chuan.
                    //
                    // Khong lam gi ca thi tof_ground_range_m giu gia tri CU
                    // (cua lan bay truoc, hoac 0 neu chua tung) -> alt_m sai
                    // ngay tu tick dau. Chot bang mau hien tai it nhat cho ra
                    // mot goc DUNG NGHIA, chi la kem on dinh hon.
                    if (!alt_estimator_lock_floor_fallback(&s_alt_est)) {
                        // ⚠ KHONG tu choi nua. Day chinh la ca "nam sat san":
                        // ToF doc 0mm nen khong co mau nao de chot goc toa do.
                        // Truoc day break o day -> takeoff bat kha thi khi drone
                        // dang o dung noi no phai o (tren mat dat).
                        //
                        // Chot goc = 0: drone DANG nam san, nen "do cao hien tai
                        // = 0" la gia thiet DUNG, khong phai gia thiet lieu.
                        // Mau ToF hop le dau tien sau khi nhac len se cho
                        // estimator so that de bam theo.
                        alt_estimator_lock_floor_at_zero(&s_alt_est);
                        ESP_LOGW(TAG, "TAKEOFF: khong co mau ToF nao de chot goc (nam sat san?) -> chot goc = 0m. Do cao se dung sau mau ToF hop le dau tien.");
                    } else {
                        ESP_LOGW(TAG, "TAKEOFF: chot goc toa do bang mau ToF hien tai (%.3fm) vi "
                                      "floor chua on dinh va cong floor DANG TAT -> do cao co the lech",
                                  (double)s_alt_est.tof_ground_range_m);
                    }
#endif
                }
                alt_estimator_prepare_takeoff(&s_alt_est);

                takeoff_begin(&s_tko_state, tko_target_m, now_us);
                ESP_LOGI(TAG, "TAKEOFF: target=%.2fm | PRIME %d duty trong %dms (KHONG chay Z/Vz PID) "
                              "-> CLIMB: target truot len voi %.2f m/s, hover tu hoc bang I "
                              "-> HOLDING (lien mach, khong doi nguon throttle)",
                          (double)tko_target_m, s_tko_tune.prime_duty,
                          s_tko_tune.prime_ms, (double)s_tko_tune.max_climb_ms);
                fsm_transition(&s_fsm, next, now_us);
            }
            break;
        }
        case CMD_LAND: {
            fsm_state_t next = fsm_on_land_request(s_fsm.state);
            if (next != s_fsm.state) {
                // Chọn bề mặt ĐANG ở dưới làm đích hạ cánh. KHÔNG đụng
                // floor_plane_z_m: hạ xuống một cái bàn không được ghi đè mốc
                // sàn của cả chuyến bay (floor != landing surface).
                if (alt_estimator_select_landing_surface(&s_alt_est)) {
                    ESP_LOGI(TAG, "LAND: be mat ha canh o world-Z %.2fm (cao hon san %.2fm), "
                                  "do cao tren be mat = %.2fm",
                              (double)s_alt_est.landing_surface_z_m,
                              (double)(s_alt_est.landing_surface_z_m - s_alt_est.floor_plane_z_m),
                              (double)alt_estimator_height_above_landing_surface(&s_alt_est));
                } else {
#if FC_FEATURE_TOF
                    ESP_LOGI(TAG, "LAND: flow rut gon ha theo altitude ToF truc tiep");
#else
                    ESP_LOGE(TAG, "LAND: ToF TAT -> khong co nguon Z cho flight control; "
                                  "chi con blind throttle ramp");
#endif
                }
                landing_reset(&s_land_state);
                fsm_transition(&s_fsm, next, now_us);
            }
            break;
        }
        case CMD_SET_ALTITUDE: {
            const float m = (float)cmd->as.set_altitude.alt_mm * 0.001f;
            s_alt_request_m = commander_clamp_altitude(&s_cmd_cfg, m);
            break;
        }
        case CMD_SET_YAW: {
            // Quay TƯƠNG ĐỐI cmd->as.set_yaw.yaw_deg độ, open-loop theo thời
            // gian (KHÔNG phải absolute heading — cần mag + heading-hold để
            // làm đúng nghĩa, xem mag_driver.h). Dùng chung timed-command với move().
            const float deg = cmd->as.set_yaw.yaw_deg;
            const float rate = (deg >= 0.0f) ? MOVE_MAX_YAW_DPS : -MOVE_MAX_YAW_DPS;
            const float dur_s = fabsf(deg) / MOVE_MAX_YAW_DPS;
            s_timed_active = true;
            s_timed_roll_deg = 0.0f;
            s_timed_pitch_deg = 0.0f;
            s_timed_yaw_rate_dps = rate;
            s_timed_until_us = now_us + (int64_t)(dur_s * 1e6f);
            fsm_transition(&s_fsm, fsm_on_move_command(s_fsm.state, true), now_us);
            break;
        }
        case CMD_MOVE: {
            const float pct_raw = clampf((float)cmd->as.move.pct, 0.0f, 100.0f) / 100.0f;
            // Expo CHI ap cho ROLL/PITCH (goc nghieng). KHONG ap cho:
            //   - MOVE_UP/DOWN: buoc do cao 10cm phai la 10cm, khong cong bang
            //   - yaw: toc do quay tuyen tinh de doan hon khi ngam huong
            const float pct = move_expo(pct_raw);
            float roll = 0.0f, pitch = 0.0f, yaw_rate = 0.0f;
            switch (cmd->as.move.dir) {
                case MOVE_FORWARD: pitch = -MOVE_MAX_TILT_DEG * pct; break;
                case MOVE_BACK:    pitch =  MOVE_MAX_TILT_DEG * pct; break;
                case MOVE_LEFT:    roll  =  MOVE_MAX_TILT_DEG * pct; break;
                case MOVE_RIGHT:   roll  = -MOVE_MAX_TILT_DEG * pct; break;
                case MOVE_UP:      s_alt_request_m += 0.10f * pct_raw; break;
                case MOVE_DOWN:    s_alt_request_m -= 0.10f * pct_raw; break;
                case MOVE_CW:      yaw_rate = -MOVE_MAX_YAW_DPS * pct_raw; break;
                case MOVE_CCW:     yaw_rate =  MOVE_MAX_YAW_DPS * pct_raw; break;
            }
            s_alt_request_m = commander_clamp_altitude(&s_cmd_cfg, s_alt_request_m);
            if (roll != 0.0f || pitch != 0.0f || yaw_rate != 0.0f) {
                s_timed_active = true;
                s_timed_roll_deg = roll;
                s_timed_pitch_deg = pitch;
                s_timed_yaw_rate_dps = yaw_rate;
                s_timed_until_us = now_us + (int64_t)(cmd->as.move.sec * 1e6f);
                fsm_transition(&s_fsm, fsm_on_move_command(s_fsm.state, true), now_us);
            }
            break;
        }
        case CMD_HOVER: {
            // Idempotent: hủy timed-command đang chạy (nếu có), về HOLDING.
            s_timed_active = false;
            if (!abort_landing_to_hold("CMD_HOVER", now_us)) {
                fsm_transition(&s_fsm, fsm_on_move_command(s_fsm.state, false), now_us);
            }
            break;
        }
        case CMD_HEARTBEAT: {
            commander_heartbeat(&s_cmd_state, now_us);
            break;
        }
        case CMD_SET_PARAM: {
            apply_set_param(cmd->as.set_param.name, cmd->as.set_param.value);
            break;
        }
        case CMD_TEST_MOTOR: {
            // CHỈ cho phép khi DISARMED (xem TODO app_config.h) — chặn cứng,
            // không phụ thuộc caller kiểm tra trước.
            if (s_fsm.state != FSM_DISARMED) {
                ESP_LOGW(TAG, "test_motor tu choi: chi cho phep khi DISARMED (state hien tai=%s)",
                          fsm_state_name(s_fsm.state));
                break;
            }
            // KILL LATCH do SỰ CỐ thắng test_motor. Lệnh này gọi
            // motor_driver_arm() — tức MỞ LẠI armed gate ở tầng driver — nên
            // nếu không chặn thì một CMD_TEST_MOTOR (từ core 0: console/UDP)
            // làm motor quay 500ms NGAY SAU một KILL. Latch chỉ có nghĩa khi
            // MỌI đường mở gate đều tôn trọng nó, không riêng vòng điều khiển.
            //
            // NGOẠI LỆ DUY NHẤT — latch benign (CMD_DISARM chủ đích trên mặt
            // đất, xem s_kill_benign). Không có ngoại lệ này thì test_motor là
            // ngõ cụt: nó đòi DISARMED + !latched, mà đường DUY NHẤT hạ latch
            // là CMD_ARM (rời khỏi DISARMED), còn DISARM (về DISARMED) thì
            // latch lại ngay -> sau lần DISARM/KILL đầu tiên KHÔNG còn chuỗi
            // lệnh nào tới được DISARMED + !latched. Board mới chưa calib còn
            // không ARM được, deadlock ngay từ boot đầu.
            //
            // An toàn không đổi: vẫn phải DISARMED, vẫn cắt sau
            // TEST_MOTOR_PULSE_MS, và một latch từ fault/failsafe/KILL (benign
            // = false) vẫn chặn cứng như cũ.
            if (s_motor_kill_latched && !s_kill_benign) {
                ESP_LOGW(TAG, "test_motor tu choi: KILL LATCH dang bat (%s) -- ARM lai truoc",
                          s_kill_reason);
                break;
            }
            if (s_motor_kill_latched) {
                ESP_LOGW(TAG, "test_motor: kill latch dang bat nhung la loai BENIGN (%s) -- "
                              "cho phep bench-test. Latch VAN chan bay, phai ARM de bay.",
                          s_kill_reason);
            }
            const int raw_idx = (int)cmd->as.test_motor.motor_idx;
            const int pct = clampi((int)cmd->as.test_motor.duty_pct, 0, TEST_MOTOR_MAX_DUTY_PCT);
            const int duty = (pct * MOTOR_SAFE_MAX_DUTY) / 100;
            int duties[4] = {0, 0, 0, 0};

            if (raw_idx == 0) {
                // Cả 4 cùng lúc, ĐỒNG duty — chỉ để sanity-check (đủ 4 con
                // quay, không kẹt/chết motor nào), KHÔNG dùng để xác nhận vị
                // trí/chiều quay (xem command.h).
                duties[0] = duties[1] = duties[2] = duties[3] = duty;
                ESP_LOGW(TAG, "TEST_MOTOR CA 4 @ %d%% duty trong %dms -- XAC NHAN DA THAO CANH QUAT",
                          pct, TEST_MOTOR_PULSE_MS);
            } else {
                const int idx_1based = clampi(raw_idx, 1, 4);
                duties[idx_1based - 1] = duty;
                ESP_LOGW(TAG, "TEST_MOTOR M%d @ %d%% duty trong %dms -- XAC NHAN DA THAO CANH QUAT",
                          idx_1based, pct, TEST_MOTOR_PULSE_MS);
            }
            motor_driver_arm();
            motor_driver_set_duties(duties[0], duties[1], duties[2], duties[3]);
            vTaskDelay(pdMS_TO_TICKS(TEST_MOTOR_PULSE_MS));
            motor_driver_stop_all();
            motor_driver_disarm();
            break;
        }

        // ================= Bench-test PID tuning (xem GHI CHÚ 3 =================
        // flight_state_machine.h + command.h) — throttle đi thẳng từ
        // s_bench_throttle_duty, KHÔNG qua alt_hold. Drone PHẢI được giữ chặt/
        // kẹp trên giá đỡ — đây KHÔNG phải chế độ bay.
        case CMD_BENCH_RAMP_START: {
            if (s_fsm.state != FSM_ARMED) {
                ESP_LOGW(TAG, "bench_ramp_start tu choi: chi cho phep tu ARMED (state hien tai=%s)",
                          fsm_state_name(s_fsm.state));
                break;
            }
            s_bench_throttle_duty = 0;
            fsm_transition(&s_fsm, fsm_on_bench_ramp_start(s_fsm.state), now_us);
            ESP_LOGW(TAG, "BENCH_RAMP bat dau -- throttle=0/%d. XAC NHAN drone da duoc GIU CHAT/KEP "
                          "TREN GIA DO truoc khi tang ga. Dung lenh +/- de tang/giam, bench_ramp_stop "
                          "de cat ngay.", MOTOR_SAFE_MAX_DUTY);
            break;
        }
        case CMD_BENCH_THROTTLE_STEP: {
            if (s_fsm.state != FSM_BENCH_RAMP) {
                ESP_LOGW(TAG, "bench throttle step tu choi: chi cho phep khi dang BENCH_RAMP "
                              "(state hien tai=%s, goi bench_ramp_start truoc)", fsm_state_name(s_fsm.state));
                break;
            }
            s_bench_throttle_duty = clampi(s_bench_throttle_duty + (int)cmd->as.bench_step.delta_duty,
                                             0, MOTOR_SAFE_MAX_DUTY);
            ESP_LOGI(TAG, "BENCH_RAMP throttle -> %d/%d", s_bench_throttle_duty, MOTOR_SAFE_MAX_DUTY);
            break;
        }
        case CMD_BENCH_THROTTLE_OFFSET: {
            // Ba nhóm state, ba nghĩa khác nhau cho CÙNG một offset:
            //   BENCH_RAMP       -> cộng THẲNG vào duty (không có PID độ cao)
            //   HOLDING/FLYING   -> cộng THẲNG vào duty, KÈM đóng băng I và neo
            //                       alt_target (alt_hold sở hữu throttle nên
            //                       thiếu 2 việc đó là phím bị I triệt tiêu —
            //                       xem tuning.h mục "Offset throttle phím GIỮ")
            //   còn lại          -> bỏ qua, ép 0
            //
            // Bỏ qua IM LẶNG có chủ đích ở nhóm cuối: phím giữ gửi lại ~10Hz,
            // log mỗi lần sẽ ngập console.
            if (s_fsm.state == FSM_LANDING && cmd->as.bench_offset.offset_duty != 0) {
                abort_landing_to_hold("throttle W/S", now_us);
            }
            const bool offset_usable = (s_fsm.state == FSM_BENCH_RAMP) ||
                                        (s_fsm.state == FSM_HOLDING) ||
                                        (s_fsm.state == FSM_FLYING);
            if (!offset_usable) {
                s_bench_throttle_offset = 0;
                break;
            }
            s_bench_throttle_offset = clampi((int)cmd->as.bench_offset.offset_duty,
                                              -MOTOR_SAFE_MAX_DUTY, MOTOR_SAFE_MAX_DUTY);
            s_bench_offset_update_us = now_us;
            break;
        }
        case CMD_BENCH_RAMP_STOP: {
            if (s_fsm.state == FSM_BENCH_RAMP) {
                s_bench_throttle_duty = 0;
                motor_driver_stop_all();
                reset_all_controllers();
                fsm_transition(&s_fsm, fsm_on_bench_ramp_stop(s_fsm.state), now_us);
                ESP_LOGI(TAG, "BENCH_RAMP STOP -> ARMED (motor cat ngay)");
            }
            break;
        }

        // ================= Ground-station UDP: setpoint bay tay =================
        //
        // QUY ƯỚC TRỤC NGOÀI (theo yêu cầu người dùng — KHÁC chuẩn hàng không):
        // BÊN NGOÀI (giao thức @SP/@TRIM/@PID, fc.control(), telemetry hiển
        // thị) "roll" = tiến/lùi, "pitch" = trái/phải. BÊN TRONG (Mahony/PID/
        // mixer, s_sp_roll_deg/s_sp_pitch_deg, ain.target_roll_deg/pitch_deg,
        // g.angle_roll/angle_pitch) HOÀN TOÀN KHÔNG ĐỔI — vẫn đúng nghĩa vật lý
        // chuẩn (internal "roll" = nghiêng trái/phải, "pitch" = nghiêng tiến/
        // lùi), vì mixer Quad-X (attitude_control.c) CỐ ĐỊNH gắn pitch_correction
        // với vi sai động cơ trước/sau và roll_correction với trái/phải — đổi
        // 1 trong 2 phía (chỉ input HOẶC chỉ mixer) mà không đổi phía kia sẽ
        // TẠO VÒNG ĐIỀU KHIỂN SAI (PID sửa đúng trục nhưng mixer đẩy sai động
        // cơ). Thay vì đổi mixer (rủi ro cao, chạm logic bay), ĐẢO Ở BIÊN: mọi
        // nơi input ngoài đi vào (CMD_SET_ATTITUDE/CMD_SET_TRIM/CMD_CONTROL/
        // CMD_SET_PID) và mọi nơi telemetry đi ra (bước 3 publish) đều đảo
        // roll<->pitch ĐÚNG MỘT LẦN — nội bộ giữa 2 điểm đảo đó (mixer/PID/
        // Mahony) không hề biết hay cần biết chuyện đổi quy ước này.
        case CMD_SET_ATTITUDE: {
            // NGOÀI "roll_deg" (tiến/lùi) -> TRONG s_sp_pitch_deg (trục vi sai
            // trước/sau thật của mixer). NGOÀI "pitch_deg" (trái/phải) -> TRONG
            // s_sp_roll_deg. Xem giải thích đảo-ở-biên ngay trên.
            s_sp_pitch_deg = clampf(cmd->as.set_attitude.roll_deg, -SP_TILT_MAX_DEG, SP_TILT_MAX_DEG);
            s_sp_roll_deg = clampf(cmd->as.set_attitude.pitch_deg, -SP_TILT_MAX_DEG, SP_TILT_MAX_DEG);
            s_sp_yaw_rate_dps = clampf(cmd->as.set_attitude.yaw_rate_dps,
                                        -SP_YAW_RATE_MAX_DPS, SP_YAW_RATE_MAX_DPS);
            s_last_sp_update_us = now_us;
            break;
        }

        // ================= Ground-station UDP: live tuning =================
        // MỌI case dưới đây ghi state dưới s_tuning_mtx (giữ RẤT NGẮN, chỉ gán
        // struct — không I/O, không log bên trong critical section) để
        // flight_core_get_*() không đọc phải dữ liệu dở dang. Không block
        // stabilize task vì mutex chỉ tranh chấp với các lần gọi get() hiếm
        // (network task, không phải mỗi tick).
        case CMD_SET_TRIM: {
            // Đảo ở biên GIỐNG HỆT CMD_SET_ATTITUDE ở trên (NGOÀI roll=tiến/
            // lùi -> TRONG s_trim_pitch_deg, NGOÀI pitch=trái/phải -> TRONG
            // s_trim_roll_deg) — xem giải thích đầu case CMD_SET_ATTITUDE.
            const float r = clampf(cmd->as.set_trim.roll_deg, -TRIM_MAX_DEG, TRIM_MAX_DEG);
            const float p = clampf(cmd->as.set_trim.pitch_deg, -TRIM_MAX_DEG, TRIM_MAX_DEG);
            xSemaphoreTake(s_tuning_mtx, portMAX_DELAY);
            s_trim_pitch_deg = r;
            s_trim_roll_deg = p;
            xSemaphoreGive(s_tuning_mtx);

            // LƯU NVS ngay. Trước đây trim CHỈ sống trong RAM nên mất sạch sau
            // mỗi lần cắm lại điện — người dùng dò trim cả buổi rồi mất hết, và
            // tệ hơn là dễ quên mà cất cánh với khung lệch.
            //
            // Lưu theo QUY ƯỚC NGOÀI (r, p — đúng thứ @TRIM SET nhận), KHÔNG
            // phải biến nội bộ đã hoán trục: nếu lưu biến nội bộ thì lúc nạp
            // lại phải nhớ hoán ngược, và chỉ cần một bên quên là trim bị xoay
            // 90° — kiểu lỗi im lặng khó lần nhất.
            //
            // Ghi NVS trong stabilize_task (250Hz) nghe đáng ngại, nhưng
            // CMD_SET_TRIM chỉ đến khi NGƯỜI DÙNG bấm nút/kéo slider — vài chục
            // lần một buổi, không phải mỗi tick. Lỗi ghi KHÔNG chặn gì: trim
            // trong RAM đã áp dụng rồi, mất NVS chỉ nghĩa là lần boot sau phải
            // dò lại — nên chỉ log.
            if (calibration_save_trim(r, p) != ESP_OK) {
                ESP_LOGW(TAG, "TRIM: ap dung roll=%.2f pitch=%.2f NHUNG luu NVS that bai "
                              "-> se mat sau khi tat nguon", (double)r, (double)p);
            }
            break;
        }
        case CMD_SET_PID: {
            const pid_gains_t g = {
                .kp = fmaxf(cmd->as.set_pid.kp, 0.0f),
                .ki = fmaxf(cmd->as.set_pid.ki, 0.0f),
                .kd = fmaxf(cmd->as.set_pid.kd, 0.0f),
                .integrator_limit = fmaxf(cmd->as.set_pid.ilimit, 0.0f),
                .output_limit = fmaxf(cmd->as.set_pid.outlimit, 0.0f),
            };
            // Đảo ở biên GIỐNG HỆT CMD_SET_ATTITUDE (xem giải thích ở đó): NGOÀI
            // "ROLL" (người dùng tune trục tiến/lùi) -> TRONG bộ gain angle_pitch/
            // rate_pitch (bộ THẬT SỰ điều khiển vi sai động cơ trước/sau). NGOÀI
            // "PITCH" -> TRONG bộ angle_roll/rate_roll.
            pid_gains_t *gains_ptr = NULL;
            pid_state_t *state_ptr = NULL;
            if (cmd->as.set_pid.loop == PID_LOOP_ANGLE) {
                switch (cmd->as.set_pid.axis) {
                    case PID_AXIS_ROLL:  gains_ptr = &s_att_gains.angle_pitch; state_ptr = &s_att_state.angle_pitch; break;
                    case PID_AXIS_PITCH: gains_ptr = &s_att_gains.angle_roll;  state_ptr = &s_att_state.angle_roll;  break;
                    case PID_AXIS_YAW:   gains_ptr = &s_att_gains.angle_yaw;   state_ptr = &s_att_state.angle_yaw;   break;
                }
            } else {
                switch (cmd->as.set_pid.axis) {
                    case PID_AXIS_ROLL:  gains_ptr = &s_att_gains.rate_pitch; state_ptr = &s_att_state.rate_pitch; break;
                    case PID_AXIS_PITCH: gains_ptr = &s_att_gains.rate_roll;  state_ptr = &s_att_state.rate_roll;  break;
                    case PID_AXIS_YAW:   gains_ptr = &s_att_gains.rate_yaw;   state_ptr = &s_att_state.rate_yaw;   break;
                }
            }
            xSemaphoreTake(s_tuning_mtx, portMAX_DELAY);
            *gains_ptr = g;
            pid_reset(state_ptr);   // CHỈ reset integrator của ĐÚNG bộ vừa đổi — không đụng 5 bộ còn lại
            xSemaphoreGive(s_tuning_mtx);
            break;
        }
        case CMD_SET_MAHONY: {
            mahony_config_t mc = s_mahony.config;
            mc.kp = clampf(cmd->as.set_mahony.kp, 0.0f, 10.0f);
            mc.ki = clampf(cmd->as.set_mahony.ki, 0.0f, 2.0f);
            xSemaphoreTake(s_tuning_mtx, portMAX_DELAY);
            mahony_set_config(&s_mahony, &mc);   // KHÔNG reset quaternion — xem mahony_filter.h
            xSemaphoreGive(s_tuning_mtx);
            break;
        }
        case CMD_SET_ALT_TUNE: {
            xSemaphoreTake(s_tuning_mtx, portMAX_DELAY);
            s_hold_tune.alt_kp = clampf(cmd->as.set_alt_tune.alt_kp, 0.0f, 100.0f);
            s_hold_tune.vz_kp = clampf(cmd->as.set_alt_tune.vz_kp, 0.0f, 5000.0f);
            s_hold_tune.vz_ki = clampf(cmd->as.set_alt_tune.vz_ki, 0.0f, 5000.0f);
            s_hold_tune.vz_ilimit = clampf(cmd->as.set_alt_tune.vz_ilimit, 0.0f, (float)MOTOR_SAFE_MAX_DUTY);
            xSemaphoreGive(s_tuning_mtx);
            break;
        }
        case CMD_SET_TKO_TUNE: {
            xSemaphoreTake(s_tuning_mtx, portMAX_DELAY);
            // hover DÙNG CHUNG với HOLD (1 điểm neo duy nhất, xem alt_hold.h).
            s_hold_tune.hover = clampf(cmd->as.set_tko_tune.hover, 0.0f, (float)MOTOR_SAFE_MAX_DUTY);
            s_tko_tune.prime_duty = clampi(cmd->as.set_tko_tune.prime_duty, 0, MOTOR_SAFE_MAX_DUTY);
            s_tko_tune.prime_ms = clampi(cmd->as.set_tko_tune.prime_ms, 1, 5000);
            // max_climb <= 0 nghĩa là caller không gửi field này (protocol cũ) ->
            // GIỮ giá trị đang dùng thay vì đặt 0. max_climb = 0 sẽ làm target
            // không bao giờ trượt tới đích -> treo ở CLIMB tới khi timeout.
            if (cmd->as.set_tko_tune.max_climb_ms > 0.0f) {
                s_tko_tune.max_climb_ms = clampf(cmd->as.set_tko_tune.max_climb_ms, 0.05f, 2.0f);
            }
            xSemaphoreGive(s_tuning_mtx);
            break;
        }
        case CMD_SET_LAND_TUNE: {
            xSemaphoreTake(s_tuning_mtx, portMAX_DELAY);
            s_land_tune.descent_vz = clampf(cmd->as.set_land_tune.descent_vz, 0.05f, 2.0f);
            s_land_tune.flare_alt_m = clampf(cmd->as.set_land_tune.flare_alt_m,
                                               s_cmd_cfg.alt_min_m, s_cmd_cfg.alt_max_m);
            s_land_tune.flare_vz = clampf(cmd->as.set_land_tune.flare_vz, 0.02f, 1.0f);
            s_land_tune.touchdown_alt_m = clampf(cmd->as.set_land_tune.touchdown_alt_m,
                                                   0.02f, s_land_tune.flare_alt_m);
            xSemaphoreGive(s_tuning_mtx);
            break;
        }
        case CMD_SET_COMMANDER_CFG: {
            // Chạy được BẤT KỲ LÚC NÀO (kể cả đang bay) — geofence/failsafe
            // PHẢI chỉnh live, KHÁC hẳn CMD_CALIB_* (chỉ DISARMED). alt_max_m
            // sàn trên alt_min_m + 0.1 để geofence không bao giờ rỗng/đảo
            // ngược (commander_clamp_altitude() sẽ vô nghĩa nếu min>=max).
            xSemaphoreTake(s_tuning_mtx, portMAX_DELAY);
            s_cmd_cfg.alt_min_m = clampf(cmd->as.set_commander_cfg.alt_min_m,
                                         0.0f, COMMANDER_DEFAULT_ALT_MAX_M - 0.1f);
            s_cmd_cfg.alt_max_m = clampf(cmd->as.set_commander_cfg.alt_max_m,
                                           s_cmd_cfg.alt_min_m + 0.1f,
                                           COMMANDER_DEFAULT_ALT_MAX_M);
            s_cmd_cfg.battery_floor_v = clampf(cmd->as.set_commander_cfg.battery_floor_v, 0.0f, 30.0f);
            s_cmd_cfg.heartbeat_timeout_ms = clampi(cmd->as.set_commander_cfg.heartbeat_timeout_ms, 100, 60000);
            s_cmd_cfg.hard_tilt_deg = clampf(cmd->as.set_commander_cfg.hard_tilt_deg, 10.0f, 90.0f);
            s_cmd_cfg.motor_sat_hard_ms = clampi(cmd->as.set_commander_cfg.motor_sat_hard_ms, 100, 30000);
            xSemaphoreGive(s_tuning_mtx);
            ESP_LOGW(TAG, "CMDR SET: altmin=%.2f altmax=%.2f battfloor=%.2fV hbtimeout=%dms "
                          "hardtilt=%.1fdeg motorsat=%dms",
                      s_cmd_cfg.alt_min_m, s_cmd_cfg.alt_max_m, s_cmd_cfg.battery_floor_v,
                      s_cmd_cfg.heartbeat_timeout_ms, s_cmd_cfg.hard_tilt_deg, s_cmd_cfg.motor_sat_hard_ms);
            break;
        }

        // ================= MicroPython fc.control()/get_states() =================
        case CMD_CONTROL: {
            if (s_flightmode == 0 && !s_headless_warned) {
                s_headless_warned = true;
                ESP_LOGW(TAG, "fc.control(): flightmode=headless nhung CHUA IMPLEMENT (STUB) "
                              "-> dang chay NHU head mode (can world-frame rotation qua mag, "
                              "trục CHƯA xác nhận trên phần cứng, xem README)");
            }
            const float rol = clampf(cmd->as.control.rol, -100.0f, 100.0f);
            const float pit = clampf(cmd->as.control.pit, -100.0f, 100.0f);
            const float yaw = clampf(cmd->as.control.yaw, -100.0f, 100.0f);
            const float thr = clampf(cmd->as.control.thr, -100.0f, 100.0f);
            if (s_fsm.state == FSM_LANDING && fabsf(thr) > 1.0f) {
                abort_landing_to_hold("joystick throttle", now_us);
            }
            // rol/pit ECHO thẳng (fc.get_states() đọc lại NGUYÊN VĂN input
            // ngoài, không cần đảo — xem s_ctrl_rol_pct/pit_pct).
            s_ctrl_rol_pct = rol;
            s_ctrl_pit_pct = pit;
            s_ctrl_yaw_pct = yaw;
            s_ctrl_thr_pct = thr;
            // DÙNG CHUNG kho s_sp_*/s_last_sp_update_us với CMD_SET_ATTITUDE
            // (ground-station UDP) — xem comment CMD_CONTROL trong command.h.
            // Đảo ở biên GIỐNG HỆT CMD_SET_ATTITUDE: NGOÀI "rol" (tiến/lùi) ->
            // TRONG s_sp_pitch_deg, NGOÀI "pit" (trái/phải) -> TRONG
            // s_sp_roll_deg (xem giải thích ở case CMD_SET_ATTITUDE).
            s_sp_pitch_deg = clampf(rol * 0.01f * s_max_lean_deg, -s_max_lean_deg, s_max_lean_deg);
            s_sp_roll_deg = clampf(pit * 0.01f * s_max_lean_deg, -s_max_lean_deg, s_max_lean_deg);
            s_sp_yaw_rate_dps = clampf(yaw * 0.01f * s_max_yawrate_dps, -s_max_yawrate_dps, s_max_yawrate_dps);
            s_last_sp_update_us = now_us;
            break;
        }
        case CMD_SET_FLIGHTMODE: {
            const int mode = clampi((int)cmd->as.set_flightmode.mode, 0, 1);
            if (mode != s_flightmode) s_headless_warned = false;   // đổi mode -> cho log STUB nhắc lại
            s_flightmode = mode;
            break;
        }

        // ================= Calibration (CHỈ khi DISARMED, xem calibration.h) =================
        case CMD_CALIB_GYRO: {
            if (s_fsm.state != FSM_DISARMED) {
                ESP_LOGW(TAG, "calib_gyro tu choi: chi cho phep khi DISARMED"); break;
            }
            // Dùng đúng FSM của startup: RAW -> stationary Welford -> candidate
            // -> validation độc lập -> atomic commit. Lệnh mới reset toàn bộ
            // phiên đang chạy, không mang old bias/running mean sang phiên mới.
            gyro_calibration_start();
            break;
        }
        case CMD_CALIB_GYRO_ABORT: {
            if (gyro_cal_state_active(s_gcal_state)) {
                gyro_cal_finish_fail(GCAL_FAIL_ABORTED, "CAL GYRO aborted by user");
                ESP_LOGI(TAG, "calib_gyro_abort: da huy phien dang do; gyro invalid, ARM bi chan");
            } else {
                ESP_LOGW(TAG, "calib_gyro_abort: khong co phien nao dang chay");
            }
            break;
        }
        case CMD_CALIB_ACCEL_FACE: {
            if (s_fsm.state != FSM_DISARMED) {
                ESP_LOGW(TAG, "calib_accel_face tu choi: chi cho phep khi DISARMED"); break;
            }
            if (s_calib_accel_capturing) {
                ESP_LOGW(TAG, "calib_accel_face: dang capture mat truoc, bo qua lenh moi"); break;
            }
            s_calib_accel_capturing = true;
            s_calib_accel_samples_left = CALIB_ACCEL_FACE_SAMPLES;
            s_calib_accel_valid_count = 0;
            s_calib_accel_motion_bad_count = 0;
            s_calib_accel_sum = vec3f_zero();
            ESP_LOGI(TAG, "calib_accel_face: bat mat %d/%d - GIU YEN drone o huong nay ~0.5s",
                      s_calib_accel_faces_done + 1, CALIB_ACCEL_FACES_NEEDED);
            break;
        }
        case CMD_CALIB_ACCEL_RESET: {
            s_calib_accel_capturing = false;
            s_calib_accel_faces_done = 0;
            ESP_LOGI(TAG, "calib_accel_reset: huy tien trinh 6-face dang do, lam lai tu dau");
            break;
        }
        case CMD_CALIB_MAG_START: {
            if (s_fsm.state != FSM_DISARMED) {
                ESP_LOGW(TAG, "calib_mag_start tu choi: chi cho phep khi DISARMED"); break;
            }
            // TỪ CHỐI NGAY nếu không có mag chạy được — trước đây vẫn nhận
            // lệnh, chạy đủ 60s rồi mới báo "qua it mau (0) -> xoay lau hon",
            // bắt người dùng xoay 1 phút vô ích rồi đổ lỗi sai nguyên nhân.
            if (!s_board.mag_enabled) {
                ESP_LOGW(TAG, "calib_mag_start tu choi: mag TAT theo app_config (SENSOR_MAG_ENABLED=0)");
                break;
            }
            // Cắt mag lúc biên dịch => s_board.mag_enabled luôn false => đã
            // break ở ngay trên. Phần dưới là code chết, gate để nó không kéo
            // theo mag_driver_* vào firmware.
#if FC_FEATURE_MAG
            if (!s_mag_ok_driver) {
                ESP_LOGE(TAG, "calib_mag_start TU CHOI: mag_driver init THAT BAI luc boot -> khong co mau "
                              "nao de thu. Xem log boot 'mag_driver' (CHIP_ID / read-back CTRL / probe DRDY) "
                              "va 'status' (mag_ok=0). Sua phan cung/day I2C roi KHOI DONG LAI truoc khi calib");
                break;
            }
            s_calib_mag_active = true;
            s_calib_mag_extrema_init = false;
            s_calib_mag_sample_count = 0;
            s_calib_mag_read_err_count = 0;
            s_calib_mag_notready_count = 0;
            {
                // Neo seq vào mẫu HIỆN TẠI: mẫu đã publish TRƯỚC khi bấm start
                // là mẫu cũ, không thuộc phiên này — không được tính vào min/max.
                mag_published_t pub;
                s_calib_mag_last_seq = (mag_driver_get_latest(&pub) == ESP_OK) ? pub.seq : 0;
            }
            s_calib_mag_ticks_left = (int)((float)CALIB_MAG_DURATION_MS * CONTROL_TASK_HZ / 1000.0f);
            s_calib_mag_progress_ticks_left = (int)((float)CALIB_MAG_PROGRESS_MS * CONTROL_TASK_HZ / 1000.0f);
            ESP_LOGI(TAG, "calib_mag: bat dau thu mau TU DONG trong %ds - XOAY drone hinh so 8 LIEN TUC "
                          "suot thoi gian nay, tu dong tinh ket qua khi het gio (hoac goi calib_mag_stop "
                          "de ket thuc som). In tien do moi %ds",
                      CALIB_MAG_DURATION_MS / 1000, CALIB_MAG_PROGRESS_MS / 1000);
#endif  // FC_FEATURE_MAG
            break;
        }
        case CMD_CALIB_MAG_STOP: {
            if (!s_calib_mag_active) { ESP_LOGW(TAG, "calib_mag_stop: khong co phien nao dang chay"); break; }
            ESP_LOGI(TAG, "calib_mag_stop: ket thuc SOM (con %d ms), tinh ngay voi %u mau da thu",
                      (int)(s_calib_mag_ticks_left * 1000 / CONTROL_TASK_HZ), (unsigned)s_calib_mag_sample_count);
            finalize_mag_calibration();
            break;
        }
        case CMD_CALIB_MAG_ABORT: {
            if (s_calib_mag_active) {
                s_calib_mag_active = false;
                ESP_LOGI(TAG, "calib_mag_abort: da huy phien dang thu mau (khong tinh/luu, du da co %u mau)",
                          (unsigned)s_calib_mag_sample_count);
            } else {
                ESP_LOGW(TAG, "calib_mag_abort: khong co phien nao dang chay");
            }
            break;
        }
        case CMD_CALIB_ERASE: {
            if (s_fsm.state != FSM_DISARMED) {
                ESP_LOGW(TAG, "calib_erase tu choi: chi cho phep khi DISARMED"); break;
            }
            // Dung moi phien gyro dang chay va xoa bias trong ban sao cua
            // sensor_hub qua cung duong atomic. Chi memset s_imu_calib o day
            // se lam telemetry thay 0 nhung hub van giu ban sao bias cu.
            gyro_cal_finish_fail(GCAL_FAIL_ABORTED, "all calibration erased by user");
            calibration_erase_all();
            memset(&s_calib, 0, sizeof(s_calib));
            s_calib_accel_faces_done = 0;
            recompute_uncalibrated();
            ESP_LOGW(TAG, "CALIB_ERASE: da xoa toan bo calib (RAM+NVS) -> UNCALIBRATED, ARM se bi tu choi");
            break;
        }
        case CMD_CALIB_BARO_GROUND: {
            if (s_fsm.state != FSM_DISARMED) {
                ESP_LOGW(TAG, "calib_baro_ground tu choi: chi cho phep khi DISARMED"); break;
            }
            if (!s_baro_ok_driver) {
                ESP_LOGW(TAG, "calib_baro_ground tu choi: baro chua init/khong co (SENSOR_BARO_ENABLED?)"); break;
            }
            // Lệnh này nói chuyện TRỰC TIẾP với BMP280 -> phải mượn bus từ
            // sensor_hub, nếu không hai bên cùng ghi/đọc chip sẽ cho ra mốc áp
            // suất nền sai mà vẫn "thành công". Không mượn được -> HUỶ lệnh,
            // KHÔNG chạy đè (xem sensor_hub_suspend()).
            if (!sensor_hub_suspend(SENSOR_BUS_LEASE_TIMEOUT_MS)) {
                ESP_LOGE(TAG, "calib_baro_ground HUY: khong muon duoc bus I2C tu sensor_hub");
                break;
            }
            // Block ~1s (BARO_GROUND_SETTLE_DELAY_MS + 32*BARO_GROUND_REF_DELAY_MS,
            // xem baro_driver.c) — chấp nhận được, cùng pattern CMD_TEST_MOTOR
            // (DISARMED-only, motor không quay, không có gì đang bay bị ảnh hưởng).
#if FC_FEATURE_BARO
            const esp_err_t berr = baro_driver_calibrate_ground();
            sensor_hub_resume();
            if (berr != ESP_OK)
                ESP_LOGE(TAG, "calib_baro_ground that bai: %s", esp_err_to_name(berr));
            else
                ESP_LOGI(TAG, "calib_baro_ground OK (DEBUG ONLY, khong doi Z/Vz/floor ToF)");
#endif  // FC_FEATURE_BARO
            break;
        }
        case CMD_MAG_SELFTEST: {
            if (s_fsm.state != FSM_DISARMED) {
                ESP_LOGW(TAG, "mag_selftest tu choi: chi cho phep khi DISARMED"); break;
            }
            if (!s_board.mag_enabled) {
                ESP_LOGW(TAG, "mag_selftest tu choi: mag TAT theo app_config (SENSOR_MAG_ENABLED=0)"); break;
            }
            // Không tới được khi mag bị cắt lúc biên dịch (mag_enabled luôn
            // false -> đã break ở trên); gate để phần dưới không kéo driver vào.
#if FC_FEATURE_MAG
            // KHÔNG đòi s_mag_ok_driver: bài test này tồn tại CHÍNH LÀ để chẩn
            // đoán trường hợp init thất bại. Hủy mọi phiên calib mag đang chạy
            // — selftest sẽ reset/cấu hình lại chip, mẫu thu được trước đó
            // không còn cùng một cấu hình nữa.
            if (s_calib_mag_active) {
                s_calib_mag_active = false;
                ESP_LOGW(TAG, "mag_selftest: huy phien calib_mag dang chay (chip sap bi reset lai)");
            }
            // Reset + cấu hình lại chip QMC5883P TRỰC TIẾP -> BẮT BUỘC mượn
            // bus. Nếu hub vẫn đang đọc mag song song, nó sẽ ăn mất DRDY giữa
            // bài test và log sẽ chỉ ra "hỏng" ở bước ngẫu nhiên — đúng thứ
            // làm bài chẩn đoán này thành vô dụng.
            if (!sensor_hub_suspend(SENSOR_BUS_LEASE_TIMEOUT_MS)) {
                ESP_LOGE(TAG, "mag_selftest HUY: khong muon duoc bus I2C tu sensor_hub");
                break;
            }
            // Block stabilize_task suốt bài test — cùng pattern
            // CMD_TEST_MOTOR/CMD_CALIB_BARO_GROUND (DISARMED-only).
            const esp_err_t merr = mag_driver_selftest(s_i2c_bus, s_board.mag_addr,
                                                       cmd->as.mag_selftest.write_sign_reg != 0,
                                                       cmd->as.mag_selftest.read_loop_ms);
            sensor_hub_resume();
            // selftest tự khôi phục driver bằng mag_driver_init() khi xong —
            // lần init đó có thể thành công HOẶC thất bại, nên phải hỏi lại
            // driver thay vì suy từ kết quả bài test, để 'status' (mag_ok) nói
            // đúng sự thật ngay lập tức mà không cần reboot.
            s_mag_ok_driver = mag_driver_is_ready();
            ESP_LOGW(TAG, "mag_selftest ket qua: %s (xem cac dong '[mag_test ...]' phia tren de biet "
                          "hong o buoc nao)", esp_err_to_name(merr));
#endif  // FC_FEATURE_MAG
            break;
        }
    }
}

// ================= stabilize task =================

// Lệnh Vz của phím W/S KHÔNG được vượt trần vz_target chung của cascade. Vượt
// thì phím lái đòi một tốc độ mà tầng ngoài Z-PID không bao giờ được phép ra
// lệnh -> hai đường điều khiển cùng một vòng nhưng khác thẩm quyền, và người
// lái sẽ thấy drone phản ứng mạnh hơn hẳn lúc tự bay. Xem ALT_HOLD_WS_VZ_MS.
_Static_assert(ALT_HOLD_WS_VZ_MS <= ALT_HOLD_VZ_LIMIT_MS,
               "ALT_HOLD_WS_VZ_MS (phim W/S) phai <= ALT_HOLD_VZ_LIMIT_MS (tran vz_target chung)");

// sensor_hub gom đúng N mẫu IMU rồi mới đánh thức vòng điều khiển. Canh quan
// hệ này lúc biên dịch để PID/estimator luôn nhận đúng nhịp 250Hz.
_Static_assert(IMU_SAMPLE_RATE_HZ == CONTROL_TASK_HZ * IMU_SAMPLES_PER_CONTROL,
               "IMU_SAMPLE_RATE_HZ phai bang CONTROL_TASK_HZ * IMU_SAMPLES_PER_CONTROL");

// Timeout chờ ngắt = 2 chu kỳ. Ở nhịp bình thường ngắt luôn tới trước hạn này
// (và nếu task bị trễ thì notify đã nằm sẵn -> lấy ngay, không chờ). Chỉ hết
// hạn khi ngắt THẬT SỰ ngừng đến.
#define IMU_INT_WAIT_TIMEOUT_MS   (2 * 1000 / CONTROL_TASK_HZ)
// pdMS_TO_TICKS() làm tròn xuống -> sàn 1 tick, không bao giờ để timeout = 0
// (timeout 0 biến ulTaskNotifyTake thành non-blocking = busy-loop 100% CPU).
#define IMU_INT_WAIT_TIMEOUT_TICKS \
    ((pdMS_TO_TICKS(IMU_INT_WAIT_TIMEOUT_MS) > 0) ? pdMS_TO_TICKS(IMU_INT_WAIT_TIMEOUT_MS) : 1)
// Số lần trượt LIÊN TIẾP trước khi bỏ hẳn chế độ INT, rơi về đồng hồ FreeRTOS.
// Vài lần trượt lẻ tẻ (nhiễu/glitch) không đáng bỏ chế độ INT; trượt liên tục
// nghĩa là dây/chip hỏng thật.
#define IMU_INT_MISS_STREAK_MAX   25   // ~100ms @250Hz

// wait_next_control_tick() — nguồn nhịp của vòng điều khiển.
//
// Đường chính: ngủ chờ SENSOR_HUB báo "đã publish mẫu IMU mới"
// (xTaskNotifyGive từ sensor_task). Đường dự phòng: đồng hồ FreeRTOS.
//
// ĐỔI SO VỚI BẢN CŨ: trước đây task này chờ THẲNG ISR của MPU6050 rồi TỰ đọc
// I2C. Giờ chân INT đánh thức sensor_hub, hub đọc I2C và publish, rồi mới đánh
// thức task này. Ý nghĩa: MỌI transaction I2C (timeout 50ms mỗi cái) nằm ở
// task khác — bus treo thì task đó BLOCKED, còn vòng điều khiển vẫn quay đúng
// nhịp bằng timeout dự phòng và nhìn thấy dữ liệu stale qua timestamp.
//
// Quy tắc AN TOÀN không được phá: vòng điều khiển KHÔNG BAO GIỜ chờ vô hạn.
// Task đứng im nghĩa là motor giữ nguyên duty cuối cùng và Commander/failsafe/
// watchdog cũng ngừng chạy — hỏng theo kiểu tệ nhất có thể.
//
// ulTaskNotifyTake(pdTRUE, ...) XÓA bộ đếm khi lấy: nếu task bị trễ và
// notification dồn lại, ta bỏ phần dồn và xử lý mẫu MỚI NHẤT thay vì chạy bù.
// Với vòng điều khiển đó là hành vi đúng — dữ liệu cũ không còn giá trị.
static bool wait_next_control_tick(TickType_t *last_wake, TickType_t period) {
    if (s_hub_notify_active) {
        if (ulTaskNotifyTake(pdTRUE, IMU_INT_WAIT_TIMEOUT_TICKS) > 0) {
            // Giữ mốc thời gian luôn mới để nếu sau này phải rơi về
            // vTaskDelayUntil() thì nó không thấy mốc quá cũ rồi quay bù
            // một loạt vòng liên tiếp.
            *last_wake = xTaskGetTickCount();
            s_imu_int_miss_streak = 0;
            s_imu_int_wake_count++;
            return true;
        }
        s_imu_int_timeout_count++;
        if (++s_imu_int_miss_streak >= IMU_INT_MISS_STREAK_MAX) {
            s_hub_notify_active = false;
            ESP_LOGE(TAG, "SENSOR_HUB NGUNG BAO MAU MOI (%d lan lien tiep) -> vong dieu khien ROI VE "
                          "dong ho FreeRTOS. VAN CHAY dung %dHz nhung du lieu cam bien se stale — "
                          "Commander se thay qua imu_age_ms. Kiem tra day INT/bus I2C",
                      IMU_INT_MISS_STREAK_MAX, CONTROL_TASK_HZ);
        }
    }
    vTaskDelayUntil(last_wake, period);
    return false;
}

// publish_kill_tick_telemetry() — phần telemetry cho tick KẾT THÚC SỚM vì kill
// latch. Chỉ ghi khối OUTPUT (state/throttle/motor/latch); khối attitude +
// cảm biến đã được bước 3 publish trong CÙNG tick nên không lặp lại ở đây.
// GUI/Python vẫn phải thấy được số liệu để biết vì sao dừng — cắt motor mà
// telemetry đứng hình thì người dùng không phân biệt nổi "đã kill" với "firmware
// treo".
static void publish_kill_tick_telemetry(int64_t now_us) {
    xSemaphoreTake(s_telemetry_mtx, portMAX_DELAY);
    s_telemetry.state = FSM_DISARMED;
    s_telemetry.armed = false;
    s_telemetry.motor_kill_latched = true;
    s_telemetry.throttle_duty = 0;
    s_telemetry.m1 = s_telemetry.m2 = s_telemetry.m3 = s_telemetry.m4 = 0;
    s_telemetry.att_integral_active = false;
    s_telemetry.last_fault = s_last_fault_class;
    s_telemetry.stamp_us = now_us;
    xSemaphoreGive(s_telemetry_mtx);
}

static void stabilize_task(void *arg) {
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(1000 / CONTROL_TASK_HZ);
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        // Do rieng phan CHO va phan XU LY. dt (raw_dt_us) gop ca hai nen mot
        // minh no khong tra loi duoc "vong cham vi CPU khong du" hay "vi cam
        // bien khong bao mau kip" — hai nguyen nhan can hai cach chua khac han.
        const bool woke_by_hub = wait_next_control_tick(&last_wake, period);
        const int64_t now_us = esp_timer_get_time();
        if (woke_by_hub) s_loop_wake_by_hub++;
        else             s_loop_wake_timeout++;

        // ---- 0) dt THỰC ĐO, không phải hằng số 1/250s ----
        // Mọi tích phân/vi phân phía dưới (Mahony, alt_estimator, PID I/D,
        // ramp takeoff/landing) dùng CHÍNH giá trị này. Dùng hằng số 1/250s
        // trong khi vòng thực chạy lệch nhịp thì mọi hệ số bị sai theo tỷ lệ
        // lệch mà KHÔNG có gì báo lỗi — sai âm thầm, đúng loại lỗi tệ nhất.
        //
        // Clamp [CONTROL_DT_MIN_S, CONTROL_DT_MAX_S]: dt=0 (2 tick cùng
        // timestamp) sẽ chia-cho-0 trong D-term/beta baro; dt quá lớn (task bị
        // treo lâu rồi chạy lại) sẽ tạo một bước tích phân khổng lồ đá
        // estimator/I-term đi rất xa. Clamp = "thà chạy sai nhịp nhẹ còn hơn
        // nhảy một bước vô lý".
        const int64_t raw_dt_us = (s_last_tick_us == 0) ? (1000000 / CONTROL_TASK_HZ)
                                                         : (now_us - s_last_tick_us);
        const float loop_dt_ms = (float)raw_dt_us / 1000.0f;
        const float dt = clampf((float)raw_dt_us / 1e6f, CONTROL_DT_MIN_S, CONTROL_DT_MAX_S);
        s_last_tick_us = now_us;

        // ---- 0b) deadline monitoring ----
        // Nhịp danh nghĩa 1/CONTROL_TASK_HZ. Trượt quá CONTROL_DEADLINE_FACTOR
        // lần chu kỳ = tick này đã "muộn" -> đếm. Commander đọc số đếm này
        // (bước 6) để biết vòng điều khiển có còn chạy đúng tốc độ không —
        // vòng chậm dần là dấu hiệu sớm của bus I2C treo / task khác chiếm CPU,
        // phải phát hiện TRƯỚC khi nó thành mất kiểm soát.
        if (s_last_tick_us != 0 && raw_dt_us > (int64_t)(CONTROL_DEADLINE_US * CONTROL_DEADLINE_FACTOR)) {
            s_deadline_miss_count++;
            s_deadline_miss_streak++;
        } else {
            s_deadline_miss_streak = 0;
        }
        if (raw_dt_us > s_loop_max_us) s_loop_max_us = raw_dt_us;
        s_loop_dt_us = raw_dt_us;

        // ---- 1) COPY snapshot cảm biến — KHÔNG CÓ I2C Ở ĐÂY ----
        // Mọi transaction I2C đã chuyển sang sensor_hub (task riêng, xem
        // sensor_hub.h). Vòng này chỉ memcpy dưới mutex ngắn. Bus treo 50ms
        // KHÔNG còn kéo theo vòng điều khiển — đó là toàn bộ lý do tách task.
        sensor_snapshot_t snap;
        sensor_hub_read(&snap);

        // "Mẫu MỚI" xác định bằng SEQUENCE, không bằng cờ ok của lần đọc cuối:
        // hub chạy nhịp riêng cho từng cảm biến (mag/baro ~50Hz) nên nhiều tick
        // 250Hz liên tiếp sẽ thấy CÙNG một mẫu.
        //
        // BARO KHÔNG còn lọc ở đây: alt_estimator tự so seq của nó (xem
        // alt_estimator.h "CORRECT") vì đó mới là nơi biết mẫu đã được DÙNG
        // hay chưa. Giữ lại seq baro ở tầng này sẽ tạo ra hai bộ đếm song song
        // cho cùng một việc và chắc chắn có ngày lệch nhau.
        const bool imu_new  = (snap.imu_h.seq  != s_last_imu_seq);
        const bool mag_new  = (snap.mag_h.seq  != s_last_mag_seq);
        s_last_imu_seq  = snap.imu_h.seq;
        s_last_mag_seq  = snap.mag_h.seq;

        // Tuổi mẫu (us) — nguồn sự thật cho "stale", KHÔNG dùng cờ ok.
        const int64_t imu_age_us  = sensor_hub_age_us(&snap.imu_h,  now_us);
        const int64_t mag_age_us  = sensor_hub_age_us(&snap.mag_h,  now_us);
        const int64_t baro_age_us = sensor_hub_age_us(&snap.baro_h, now_us);

        imu_sample_t imu = snap.imu;
        // imu_fresh: có mẫu hợp lệ VÀ chưa quá hạn. Đây là điều kiện dùng cho
        // Mahony/estimator — mẫu cũ 100ms tuy "valid" nhưng vô dụng cho điều
        // khiển, và tệ hơn: nó làm mọi thứ trông như đang chạy bình thường.
        const bool imu_fresh = snap.imu_h.valid && (imu_age_us <= SENSOR_IMU_STALE_US);
        if (!imu_fresh) {
            // Không dùng mẫu quá hạn cho fusion — zero để mọi guard dưới
            // (attitude_valid, stationary, calib) không vô tình tin vào nó.
            imu.ok = false;
        }
        if (imu.ok) s_last_imu_temp_c = imu.temp_c;   // xem telemetry.imu_temp_c + calib gyro log

        // ToF hướng xuống — GIỜ ĐẾN TỪ SNAPSHOT THẬT.
        // TRƯỚC ĐÂY dòng này là `tof_reading_t tof = {0}` hardcode: phần cứng
        // được init lúc boot nhưng sensor_hub KHÔNG hề đọc nó, nên telemetry
        // luôn báo 0 và estimator không có gì để dùng. Đó là lý do bật
        // SENSOR_TOF_ENABLED=1 một mình không đủ.
        //
        // tof_healthy_for_alt = "cảm biến còn sống, mẫu chưa quá hạn" — CHỈ sức
        // khoẻ. Việc phân biệt "mẫu này có MỚI không" do alt_estimator tự làm
        // bằng seq (ToF ~30Hz vs estimator 250Hz), giống hệt baro.
        //
        // ⚠ TÍNH THUẦN BẰNG TUỔI, CỐ Ý KHÔNG ĐỌC snap.tof_h.valid.
        //
        // sensor_hub.h (mục SENSOR_TOF_STALE_US) đã luôn nói ý định là: "200ms
        // = ~6 mẫu bị mất mới coi là stale — đủ rộng cho vài lần range_status
        // lỗi (bề mặt hấp thụ/ngoài tầm là chuyện BÌNH THƯỜNG với ToF, không
        // phải hỏng cảm biến)". Nhưng nó CHƯA BAO GIỜ được hiện thực: mark_err()
        // đặt tof_h.valid = false ngay từ MẪU XẤU ĐẦU TIÊN, nên vế `&&
        // tof_h.valid` làm điều kiện false trước khi vế tuổi kịp có ý nghĩa.
        // Cửa sổ 200ms là code chết — nó chỉ có thể làm điều kiện CHẶT HƠN,
        // chưa bao giờ nới ra như đã hứa.
        //
        // Hậu quả thật: một mẫu ToF xấu trong lúc TAKING_OFF/PRIME -> estimator
        // invalid -> Commander soft-fault -> huỷ cất cánh. Xem thêm khối "LUOI
        // DO TREN MAT DAT" trong alt_estimator.c.
        //
        // AN TOÀN vì mark_err() KHÔNG tăng seq và KHÔNG dời timestamp:
        //   - timestamp chỉ nhích khi có mẫu TỐT -> tuổi ở đây đúng nghĩa
        //     "bao lâu rồi chưa có mẫu dùng được". Cảm biến chết thật thì tuổi
        //     cứ tăng và sau 200ms vẫn thành false, đúng như trước.
        //   - seq đứng yên -> alt_estimator thấy tof_new = false -> KHÔNG
        //     correction lại trên mẫu cũ (xem publish_tof() trong sensor_hub.c).
        //     Đây là điều kiện làm cho việc nới lỏng này không tạo ra rủi ro
        //     "kéo Z nhiều lần bằng cùng một measurement".
        //   - chưa từng có mẫu tốt nào -> seq vẫn 0 -> sensor_hub_age_us() trả
        //     INT64_MAX -> luôn > 200ms -> false. Không cần guard riêng. Đây
        //     cũng là đường bảo vệ khi ToF bị tắt/không init được: hub không
        //     publish lần nào nên seq đứng ở 0 vĩnh viễn.
        const tof_reading_t tof = snap.tof;
        const int64_t tof_age_us = sensor_hub_age_us(&snap.tof_h, now_us);
        // tof_hw_alive = CHIP CON DANG DO, khac han tof_healthy_for_alt.
        //
        // VI SAO CAN RIENG: tof_h.timestamp_us chi nhich khi co mau HOP LE.
        // Dat drone xuong san thi ToF doc 0mm (duoi tam mu ~4cm cua L1X) ->
        // khong mau nao hop le -> tuoi tang vo han -> estimator ket luan
        // "mat ToF" -> Commander soft-fault. Tuc la chi can de drone nam dat
        // du lau la firmware tu bao hong cam bien.
        //
        // tof_alive_us den tu tof_driver_last_sample_us(): moc lan cuoi MCU
        // doc TRON VEN mot ket qua tu chip, bat ke ket qua do co hop le hay
        // khong. Nam sat san / ngoai tam -> van nhich. Chip chet / bus dut ->
        // dung yen.
        //
        // Nguong dung DUNG ALT_EST_TOF_LOST_MS cua estimator, khong dat hang
        // so moi: hai ben dang tra loi cung mot cau hoi "bao lau thi coi la mat".
        const bool tof_hw_alive =
            snap.tof_alive_us != 0 &&
            (now_us - snap.tof_alive_us) <= (int64_t)ALT_EST_TOF_LOST_MS * 1000;

        // ⚠ DUNG tof_hw_alive, KHONG dung tuoi mau hop le.
        //
        // Ban truoc: (tof_age_us <= SENSOR_TOF_STALE_US), tuc la hoi "bao lau
        // roi chua co mau HOP LE". Nam sat san thi ToF doc 0.000m -> khong mau
        // nao hop le -> tuoi tang vo han -> bien nay false VINH VIEN, trong khi
        // chip van do deu. Hau qua day chuyen:
        //   - neo target trong FLYING bi chan (dieu kien && tof_healthy_for_alt)
        //   - takeoff_run() nhan alt_source_ok = false
        //   - GUI hien "ToF LOI: mau STALE"
        // Day chinh la "dieu kien ToF cu" con sot lai sau khi update_age() da
        // chuyen sang mo hinh chi-hoi-chip-con-do.
        //
        // GIO: chip con tieu thu duoc ket qua = con dung duoc. Con mau do co
        // FUSE duoc khong la cau hoi RIENG, do s_alt_est.tof_fusable tra loi —
        // va moi cho dung bien nay deu da AND them tof_fusable san.
        const bool tof_healthy_for_alt = tof_hw_alive;



        mag_sample_t mag = snap.mag;
        // mag_ok = có mẫu hợp lệ, MỚI, và chưa stale. mag ODR thấp nên "không
        // mới ở tick này" là bình thường — KHÔNG phải lỗi.
        const bool mag_ok = snap.mag_h.valid && mag_new &&
                             (mag_age_us <= SENSOR_MAG_STALE_US);

#if FC_FEATURE_BARO
        baro_sample_t baro = snap.baro;
#endif
        // baro_healthy_for_alt = "cảm biến còn sống, mẫu chưa quá hạn" — CHỈ
        // sức khoẻ, KHÔNG bao gồm "mẫu này có mới không". Việc phân biệt mẫu
        // mới giờ do alt_estimator tự làm bằng seq (xem alt_estimator.h
        // "CORRECT") — trước đây gộp cả 2 vào một cờ tên `baro_ok`, đúng chức
        // năng nhưng che mất ranh giới ngữ nghĩa giữa health và new-sample.
#if FC_FEATURE_BARO
        const bool baro_healthy_for_alt = snap.baro_h.valid &&
                                           (baro_age_us <= SENSOR_BARO_STALE_US);
#endif

        // battery_v = 0.0f nếu chưa có mẫu HỢP LỆ -> Commander coi là "chưa có
        // mẫu", KHÔNG trip fault (quy ước sẵn có của commander_evaluate()) VÀ
        // battery_comp giữ 1.0 (bước 9b gate `battery_v > 1.0f`). battery_h.valid
        // đã bao gồm sanity 1S + yêu cầu ADC calibration thật (xem
        // battery_driver.h) — một mẫu 6.07V trên pin 1S rơi vào đây, KHÔNG
        // chảy tiếp vào failsafe/compensation.
        const float battery_v = snap.battery_h.valid ? snap.battery.voltage_v : 0.0f;

        // Đếm lỗi I2C cộng dồn cho telemetry — lấy TỔNG từ hub thay vì tự đếm
        // (hub mới là bên thực sự chạm bus).
        s_sensor_err_count = snap.imu_h.total_errors + snap.mag_h.total_errors +
                             snap.baro_h.total_errors;

        // ====================================================================
        // imu_updated — CÓ MẪU IMU MỚI THẬT SỰ Ở TICK NÀY HAY KHÔNG
        // ====================================================================
        // = seq đã tăng (mẫu CHƯA TỪNG được tick nào tiêu thụ) VÀ mẫu chưa quá
        // hạn. Đây là điều kiện DUY NHẤT cho phép chạy các bộ tích phân dùng
        // IMU (Mahony ở bước 2, alt_estimator ở bước 2, I-term của attitude
        // cascade ở bước 10).
        //
        // Vì sao KHÔNG dùng imu_fresh một mình: khi hub ngừng publish, mẫu cuối
        // vẫn "fresh" trong 20ms đầu (SENSOR_IMU_STALE_US) và imu.ok vẫn true —
        // 5 tick liên tiếp sẽ tích phân LẠI cùng một mẫu. Khi hub publish LỖI
        // (mark_err) thì seq đứng yên nhưng giá trị cũ vẫn nằm nguyên trong
        // snapshot, nên `imu.ok=false` KHÔNG chặn được gì: mahony_update()
        // không nhận cờ ok, nó chỉ nhận thẳng gyro/accel.
        //
        // Tick KHÔNG có mẫu mới VẪN chạy đầy đủ: deadline monitor (bước 0b),
        // kill latch (4a), SP watchdog (4b), tilt/attitude-stale failsafe (5),
        // Commander (6) — thấy imu_stale qua imu_age_us —, FSM, và mixer. Chỉ
        // các bước TÍCH PHÂN bị bỏ. Đó là toàn bộ ý: mất mẫu phải làm safety
        // chạy nhiều hơn, không phải làm estimator bịa thêm dữ liệu.
        const bool imu_updated = imu_new && imu_fresh;
        s_fusion_dt_accum_s += (float)raw_dt_us / 1e6f;
        if (!imu_updated) {
            if (s_imu_no_new_sample_count < UINT32_MAX) s_imu_no_new_sample_count++;
        }

        // ---- 1a-bis) STARTUP GYRO CALIB — chạy TRƯỚC Mahony (xem mục 21) ----
        // Trong lúc máy này chạy, Mahony KHÔNG được tích phân: gyro chưa hiệu
        // chỉnh (Gz lệch ~2dps trên phần cứng này) sẽ nạp vài giây yaw sai vào
        // quaternion, và cái sai đó KHÔNG tự mất đi sau khi calib xong — Mahony
        // không có tham chiếu yaw tuyệt đối để kéo lại (board này không có mag
        // dùng được). Vì vậy calib xong còn reset hẳn quaternion, xem ngay dưới.
        const bool gcal_running = gyro_calibration_tick(&imu, imu_updated);
        if (s_gcal_state == GCAL_PASS && !s_gcal_reset_done) {
            // Bắt đầu lại từ quaternion sạch với gyro ĐÃ hiệu chỉnh. Mahony hội
            // tụ lại roll/pitch từ accel trong vài trăm ms; yaw bắt đầu từ 0 —
            // đúng như mọi lần boot khác (không mag thì yaw luôn là tương đối).
            mahony_init(&s_mahony, NULL);
            s_gcal_reset_done = true;
            ESP_LOGI(TAG, "Mahony RESET sau gyro calib PASS -> tich phan voi gyro DA hieu chinh");
        }

        // ---- 1b) Calibration state machines (CHỈ tích lũy khi DISARMED — xem
        // command.h CMD_CALIB_*). Dùng mẫu RAW imu.accel_g / mag.mag_body (TRƯỚC
        // khi áp correction bước 1c bên dưới) — calib mới PHẢI đo từ gốc, không
        // compound lên calib cũ đang áp dụng. Không block: mỗi lệnh CMD_CALIB_*
        // chỉ set cờ "active" trong apply_command(), số liệu tích lũy ở ĐÂY qua
        // nhiều tick, stabilize_task/telemetry/command queue vẫn chạy bình
        // thường suốt quá trình (khác accel/gyro dead-reckoning nào khác).
        if (s_fsm.state != FSM_DISARMED) {
            // An toàn: nếu vừa ARM giữa lúc đang calib dở (hiếm, xem apply_command
            // CMD_ARM guard vẫn cho phép nếu ĐÃ có calib cũ hợp lệ) -> hủy ngay.
            if (gyro_cal_state_active(s_gcal_state)) {
                gyro_cal_finish_fail(GCAL_FAIL_ABORTED, "left DISARMED during gyro calibration");
            }
            if (s_calib_accel_capturing) { s_calib_accel_capturing = false; ESP_LOGW(TAG, "calib_accel huy: khong con DISARMED"); }
            if (s_calib_mag_active)      { s_calib_mag_active = false;      ESP_LOGW(TAG, "calib_mag huy: khong con DISARMED"); }
            // Rời DISARMED -> xoá cửa sổ prearm gyro health. Giữ lại thì lần
            // DISARM sau sẽ đánh giá bằng dữ liệu trộn giữa hai phiên.
            s_prearm_gyro_count = 0;
            s_prearm_gyro_sum = vec3f_zero();
        } else {
            // ---- Pre-arm corrected-gyro health: chỉ tạo cửa sổ khi RAW IMU
            // stationary. Một mẫu chuyển động reset TOÀN BỘ cửa sổ và invalid
            // kết quả cũ, nên ARM không thể dùng mean từ trước khi drone bị nhấc.
            if (imu_updated && !gcal_running && s_calib.gyro_valid) {
                if (!gyro_cal_sample_stationary(&imu)) {
                    s_prearm_gyro_count = 0;
                    s_prearm_gyro_sum = vec3f_zero();
                    s_prearm_gyro_mean_valid = false;
                } else {
                    s_prearm_gyro_sum.x += imu.gyro_dps.x;
                    s_prearm_gyro_sum.y += imu.gyro_dps.y;
                    s_prearm_gyro_sum.z += imu.gyro_dps.z;
                    s_prearm_gyro_count++;
                    const int win_n = gyro_cal_ticks(PREARM_GYRO_WINDOW_MS);
                    if (s_prearm_gyro_count >= win_n) {
                        const float inv = 1.0f / (float)s_prearm_gyro_count;
                        s_prearm_gyro_mean = (vec3f_t){ s_prearm_gyro_sum.x * inv,
                                                         s_prearm_gyro_sum.y * inv,
                                                         s_prearm_gyro_sum.z * inv };
                        s_prearm_gyro_mean_valid = true;
                        s_prearm_gyro_count = 0;
                        s_prearm_gyro_sum = vec3f_zero();
                    }
                }
            }

            if (s_calib_accel_capturing) {
                // CHỈ cộng mẫu MỚI và đọc I2C THÀNH CÔNG — cùng lý do với gyro
                // ở trên (mẫu lỗi = {0,0,0} do imu_driver_read() fallback, cộng
                // vào sẽ kéo trung bình mặt hiện tại lệch sai; mẫu LẶP LẠI làm
                // motion-detect tưởng gyro đứng yên hơn thực tế).
                if (imu_updated) {
                    s_calib_accel_sum.x += imu.accel_g.x;
                    s_calib_accel_sum.y += imu.accel_g.y;
                    s_calib_accel_sum.z += imu.accel_g.z;
                    s_calib_accel_valid_count++;

                    // Motion detect (xem "10. Accel six-position acquisition" —
                    // không commit pose nếu gyro cho thấy đang chuyển động).
                    if (fabsf(imu.gyro_dps.x) > CALIB_ACCEL_MOTION_GYRO_DPS ||
                        fabsf(imu.gyro_dps.y) > CALIB_ACCEL_MOTION_GYRO_DPS ||
                        fabsf(imu.gyro_dps.z) > CALIB_ACCEL_MOTION_GYRO_DPS) {
                        s_calib_accel_motion_bad_count++;
                    }
                }
                if (--s_calib_accel_samples_left <= 0) {
                    const bool too_few_valid =
                        (float)s_calib_accel_valid_count < (float)CALIB_ACCEL_FACE_SAMPLES * CALIB_MIN_VALID_FRACTION;
                    const bool moving = s_calib_accel_valid_count > 0 &&
                        (float)s_calib_accel_motion_bad_count / (float)s_calib_accel_valid_count >
                            CALIB_MOTION_MAX_BAD_FRACTION;
                    if (too_few_valid || moving) {
                        ESP_LOGE(TAG, "calib_accel: mat nay THAT BAI (%s, %d/%d mau hop le, %d mau nghi dong) "
                                      "-> KHONG tinh mat nay, goi lai calib_accel_face de thu lai DUNG mat nay",
                                  moving ? "PHAT HIEN CHUYEN DONG" : "qua it mau IMU doc thanh cong",
                                  s_calib_accel_valid_count, CALIB_ACCEL_FACE_SAMPLES, s_calib_accel_motion_bad_count);
                        // KHÔNG tăng s_calib_accel_faces_done -> user gọi lại
                        // calib_accel_face() sẽ tính lại ĐÚNG mặt này, không mất
                        // tiến trình các mặt trước đó.
                    } else {
                        const vec3f_t avg = {
                            s_calib_accel_sum.x / (float)s_calib_accel_valid_count,
                            s_calib_accel_sum.y / (float)s_calib_accel_valid_count,
                            s_calib_accel_sum.z / (float)s_calib_accel_valid_count,
                        };
                        s_calib_accel_face_avg[s_calib_accel_faces_done] = avg;
                        s_calib_accel_faces_done++;
                        ESP_LOGI(TAG, "calib_accel: mat %d/%d xong (%d mau hop le), avg=(%.3f,%.3f,%.3f)g",
                                  s_calib_accel_faces_done, CALIB_ACCEL_FACES_NEEDED, s_calib_accel_valid_count,
                                  avg.x, avg.y, avg.z);

                        if (s_calib_accel_faces_done >= CALIB_ACCEL_FACES_NEEDED) {
                            // min/max TỪ 6 mặt đã lưu riêng (KHÔNG chạy min/max
                            // dần — xem khai báo s_calib_accel_face_avg).
                            vec3f_t mn = s_calib_accel_face_avg[0], mx = s_calib_accel_face_avg[0];
                            for (int i = 1; i < CALIB_ACCEL_FACES_NEEDED; i++) {
                                const vec3f_t v = s_calib_accel_face_avg[i];
                                mn.x = fminf(mn.x, v.x); mx.x = fmaxf(mx.x, v.x);
                                mn.y = fminf(mn.y, v.y); mx.y = fmaxf(mx.y, v.y);
                                mn.z = fminf(mn.z, v.z); mx.z = fmaxf(mx.z, v.z);
                            }
                            const vec3f_t range = { mx.x - mn.x, mx.y - mn.y, mx.z - mn.z };
                            if (range.x < CALIB_ACCEL_MIN_RANGE_G || range.y < CALIB_ACCEL_MIN_RANGE_G ||
                                range.z < CALIB_ACCEL_MIN_RANGE_G) {
                                ESP_LOGE(TAG, "ACCEL CALIB THAT BAI: range qua nho (%.2f,%.2f,%.2f)g "
                                              "(can >=%.1fg moi truc) -> KHONG luu, lam lai dung quy trinh 6-face "
                                              "(moi truc phai thay ca +g va -g giua cac mat)",
                                          range.x, range.y, range.z, CALIB_ACCEL_MIN_RANGE_G);
                            } else {
                                const vec3f_t bias = {
                                    (mx.x + mn.x) * 0.5f, (mx.y + mn.y) * 0.5f, (mx.z + mn.z) * 0.5f,
                                };
                                const vec3f_t scale = { 2.0f / range.x, 2.0f / range.y, 2.0f / range.z };

                                // "13. Accel calibration verification" — áp lại
                                // correction lên chính 6 mặt vừa đo, mặt nào
                                // |a_hieu_chinh| lệch 1.0g quá CALIB_ACCEL_MAX_RESIDUAL_G
                                // -> HỦY toàn bộ (không giả định calibration tốt
                                // chỉ vì công thức chạy xong không lỗi).
                                float max_residual = 0.0f;
                                for (int i = 0; i < CALIB_ACCEL_FACES_NEEDED; i++) {
                                    const vec3f_t v = s_calib_accel_face_avg[i];
                                    const float cx = (v.x - bias.x) * scale.x;
                                    const float cy = (v.y - bias.y) * scale.y;
                                    const float cz = (v.z - bias.z) * scale.z;
                                    const float norm = sqrtf(cx * cx + cy * cy + cz * cz);
                                    const float residual = fabsf(norm - 1.0f);
                                    if (residual > max_residual) max_residual = residual;
                                }
                                s_last_accel_calib_residual_g = max_residual;
                                if (max_residual > CALIB_ACCEL_MAX_RESIDUAL_G) {
                                    ESP_LOGE(TAG, "ACCEL CALIB THAT BAI: residual %.3fg > nguong %.3fg sau khi "
                                                  "hieu chinh lai 6 mat -> KHONG luu (co mat bi dong/khong phang "
                                                  "luc do?), lam lai",
                                              (double)max_residual, (double)CALIB_ACCEL_MAX_RESIDUAL_G);
                                } else {
                                    s_calib.accel_bias_g = bias;
                                    s_calib.accel_scale = scale;
                                    s_calib.accel_valid = true;
                                    calibration_save_accel(&bias, &scale);
                                    recompute_uncalibrated();
                                    ESP_LOGW(TAG, "ACCEL CALIB XONG (residual max %.3fg): bias=(%.3f,%.3f,%.3f)g scale=(%.3f,%.3f,%.3f)",
                                              (double)max_residual, bias.x, bias.y, bias.z, scale.x, scale.y, scale.z);
                                }
                            }
                            s_calib_accel_faces_done = 0;
                        }
                    }
                    s_calib_accel_capturing = false;
                }
            }

            if (s_calib_mag_active) {
                // Lấy mẫu từ SNAPSHOT của sensor_hub — TUYỆT ĐỐI KHÔNG gọi
                // mag_driver_read() ở đây: đọc STATUS clear DRDY, nên đọc thêm
                // một lần nữa sẽ cướp mẫu của hub và cả hai bên đều thấy dữ
                // liệu chập chờn (xem sensor_hub.h "AI SỞ HỮU PHẦN CỨNG").
                // seq đổi = mẫu chưa từng xử lý -> không bao giờ đếm trùng.
                const bool new_sample = (snap.mag_h.seq != 0) &&
                                         (snap.mag_h.seq != s_calib_mag_last_seq);
                // Đếm LÝ DO thiếu mẫu (xem s_calib_mag_read_err_count đầu file)
                // — để finalize nói đúng nguyên nhân thay vì đổ tại "xoay it".
                if (!snap.mag_h.healthy) s_calib_mag_read_err_count++;
                else if (!new_sample)    s_calib_mag_notready_count++;
                if (new_sample) {
                    s_calib_mag_last_seq = snap.mag_h.seq;
                    const vec3f_t mb = snap.mag.mag_body;
                    if (!s_calib_mag_extrema_init) {
                        s_calib_mag_min = mb; s_calib_mag_max = mb;
                        s_calib_mag_extrema_init = true;
                    } else {
                        s_calib_mag_min.x = fminf(s_calib_mag_min.x, mb.x);
                        s_calib_mag_min.y = fminf(s_calib_mag_min.y, mb.y);
                        s_calib_mag_min.z = fminf(s_calib_mag_min.z, mb.z);
                        s_calib_mag_max.x = fmaxf(s_calib_mag_max.x, mb.x);
                        s_calib_mag_max.y = fmaxf(s_calib_mag_max.y, mb.y);
                        s_calib_mag_max.z = fmaxf(s_calib_mag_max.z, mb.z);
                    }
                    s_calib_mag_sample_count++;
                }
                // Đếm TICK trôi qua KHÔNG PHỤ THUỘC mag_ok (cần đúng THỜI GIAN
                // thật ~60s trôi qua, không phải "60000 mẫu mag" — ODR mag thấp
                // hơn tick rate stabilize_task nhiều, xem mag_driver.c) — hết
                // giờ thì TỰ ĐỘNG tính kết quả, không cần CMD_CALIB_MAG_STOP.
                if (--s_calib_mag_ticks_left <= 0) {
                    ESP_LOGI(TAG, "calib_mag: het %ds, tu dong tinh ket qua", CALIB_MAG_DURATION_MS / 1000);
                    finalize_mag_calibration();
                } else if (--s_calib_mag_progress_ticks_left <= 0) {
                    // Tiến độ mỗi CALIB_MAG_PROGRESS_MS: thấy NGAY đang xoay đủ
                    // hay không (range 3 trục tăng đều?) thay vì chờ hết 60s mới
                    // biết hỏng. mau=0 kéo dài -> dừng luôn, không xoay vô ích.
                    s_calib_mag_progress_ticks_left =
                        (int)((float)CALIB_MAG_PROGRESS_MS * CONTROL_TASK_HZ / 1000.0f);
                    const int sec_left = s_calib_mag_ticks_left / CONTROL_TASK_HZ;
                    if (s_calib_mag_extrema_init) {
                        ESP_LOGI(TAG, "calib_mag [con %2ds]: mau=%u loi_i2c=%u chua_sansang=%u | "
                                      "range=(%.0f,%.0f,%.0f) min=(%.0f,%.0f,%.0f) max=(%.0f,%.0f,%.0f)",
                                  sec_left, (unsigned)s_calib_mag_sample_count,
                                  (unsigned)s_calib_mag_read_err_count, (unsigned)s_calib_mag_notready_count,
                                  s_calib_mag_max.x - s_calib_mag_min.x,
                                  s_calib_mag_max.y - s_calib_mag_min.y,
                                  s_calib_mag_max.z - s_calib_mag_min.z,
                                  s_calib_mag_min.x, s_calib_mag_min.y, s_calib_mag_min.z,
                                  s_calib_mag_max.x, s_calib_mag_max.y, s_calib_mag_max.z);
                    } else {
                        ESP_LOGW(TAG, "calib_mag [con %2ds]: CHUA CO MAU NAO (loi_i2c=%u chua_sansang=%u) - "
                                      "xoay them cung vo ich neu so nay khong dung yen, xem log boot mag_driver",
                                  sec_left, (unsigned)s_calib_mag_read_err_count,
                                  (unsigned)s_calib_mag_notready_count);
                    }
                }
            }
        }

        // ---- 1c) áp accel bias/scale + mag hard/soft-iron (nếu đã calib) —
        // TRƯỚC Mahony, xem calibration.h. imu_driver.c/mag_driver.c CHỈ trả
        // raw+scale-vật-lý (giữ nguyên, không đụng 2 file driver này) — toàn bộ
        // correction calib nằm ở TẦNG FUSION này để 1 chỗ duy nhất áp dụng cho
        // cả console/UDP/MicroPython (dùng chung stabilize_task).
        if (s_calib.accel_valid) {
            imu.accel_g.x = (imu.accel_g.x - s_calib.accel_bias_g.x) * s_calib.accel_scale.x;
            imu.accel_g.y = (imu.accel_g.y - s_calib.accel_bias_g.y) * s_calib.accel_scale.y;
            imu.accel_g.z = (imu.accel_g.z - s_calib.accel_bias_g.z) * s_calib.accel_scale.z;
        }
        if (s_calib.mag_valid && mag_ok) {
            mag.mag_body.x = (mag.mag_body.x - s_calib.mag_hard_iron.x) * s_calib.mag_soft_iron_scale.x;
            mag.mag_body.y = (mag.mag_body.y - s_calib.mag_hard_iron.y) * s_calib.mag_soft_iron_scale.y;
            mag.mag_body.z = (mag.mag_body.z - s_calib.mag_hard_iron.z) * s_calib.mag_soft_iron_scale.z;
        }

        // mag CHƯA CALIB thì TUYỆT ĐỐI KHÔNG đưa vào Mahony. mag_ok chỉ có
        // nghĩa "đọc I2C thành công", KHÔNG có nghĩa "vector này chỉ đúng
        // hướng". Hard-iron offset trên mẫu thật đo được lên tới hàng nghìn
        // count trên 1 trục — vector raw khi đó vẽ ra mặt cầu KHÔNG tâm gốc,
        // nên hướng sau normalize sai lệch rất lớn và SAI KHÁC NHAU theo từng
        // hướng xoay. Đưa nó vào yaw correction thì mag không "giúp" mà ĐÁNH
        // NHAU với gyro: yaw vừa sai vừa nhảy, TỆ HƠN hẳn so với không có mag
        // (gyro-only tuy trôi nhưng trôi CHẬM và MƯỢT).
        //
        // Đây chính là "magnetic validity gating" mà mag_driver.h ghi TODO là
        // việc của tầng estimator — driver chỉ lọc được DRDY/OVFL/all-zero ở
        // mức 1 mẫu, nó không biết gì về calibration.
        const bool mag_usable = mag_ok && s_calib.mag_valid;
        if (mag_ok && !s_calib.mag_valid && !s_mag_uncalib_warned) {
            s_mag_uncalib_warned = true;
            ESP_LOGW(TAG, "mag DOC DUOC nhung CHUA CALIB -> KHONG dung cho yaw (yaw se chi dua vao "
                          "gyro va troi cham). Chay 'calib_mag_start' + xoay hinh so 8 60s de bat mag");
        }

        // ---- 2) Mahony — CHỈ khi có mẫu IMU MỚI ----
        // fusion_dt = thời gian giữa hai MẪU (cộng dồn qua các tick không có
        // mẫu), KHÔNG phải dt giữa hai tick. Mất 3 mẫu rồi có lại thì bước tích
        // phân phải bằng 3 chu kỳ — dùng dt của 1 tick sẽ làm góc/vận tốc bị
        // tính hụt đúng bằng phần đã mất. Clamp CÙNG dải với dt (CONTROL_DT_
        // MAX_S) để mất mẫu kéo dài không tạo một bước tích phân khổng lồ.
        const float fusion_dt = clampf(s_fusion_dt_accum_s, CONTROL_DT_MIN_S, CONTROL_DT_MAX_S);
        // gcal_running -> KHÔNG tích phân (mục 21). Vẫn xả s_fusion_dt_accum_s
        // để khi calib xong, bước tích phân đầu tiên không mang theo cả vài
        // giây dt dồn lại — clamp CONTROL_DT_MAX_S sẽ chặn phần lớn nhưng ý
        // định phải rõ ràng ở đây, không dựa vào clamp ở chỗ khác.
        if (imu_updated && !gcal_running && s_calib.gyro_valid) {
            mahony_update(&s_mahony, imu.gyro_dps, imu.accel_g, mag.mag_body, mag_usable, fusion_dt);
            s_fusion_dt_accum_s = 0.0f;

            if (s_mahony.status.mag_used)     s_mag_used_count++;
            if (s_mahony.status.mag_rejected) s_mag_rejected_count++;
        } else if (imu_updated) {
            // Đang calib HOẶC calib đã FAIL: bỏ mẫu khỏi tích phân và xả dt.
            // Không bao giờ tạo attitude "flight-ready" từ gyro chưa validate.
            // Không xả thì s_fusion_dt_accum_s lớn dần suốt ~5s calib, và bước
            // tích phân đầu tiên sau đó nhận một dt khổng lồ (bị clamp, nhưng
            // vẫn sai) — Mahony sẽ giật một nhịp đúng lúc vừa reset xong.
            s_fusion_dt_accum_s = 0.0f;
        }
        // Không có mẫu mới -> quaternion GIỮ NGUYÊN giá trị lần cập nhật cuối.
        // KHÔNG reset, KHÔNG suy diễn tiếp: Commander thấy imu_stale ở bước 6
        // và bước 5 cắt ngay nếu attitude mất hẳn — đó là đường xử lý đúng, chứ
        // không phải để estimator tự bịa ra trạng thái mới từ mẫu chết.

        float roll_deg = 0.0f, pitch_deg = 0.0f, yaw_deg = 0.0f;
        mahony_get_euler_deg(&s_mahony, &roll_deg, &pitch_deg, &yaw_deg);
        const bool attitude_valid = s_mahony.status.initialized;

        // "airborne" cho alt_estimator (xem alt_estimator.h "BA PHA") — kể từ
        // lúc BÀN GIAO xong chuỗi cất cánh (FSM rời TAKING_OFF) cho tới khi hạ cánh
        // xong. Trong lúc còn TAKING_OFF thì estimator ở pha CANDIDATE bên
        // dưới: vẫn tích phân, nhưng CHƯA cho baro fusion chạy (sát đất baro
        // nhiễu nhất, xem alt_estimator.h). BENCH_RAMP KHÔNG bao giờ airborne
        // (motor quay tại chỗ trên giá — xem commander.c exemption cùng lý do).
        // ====================================================================
        // GATE ESTIMATOR — ĐÂY LÀ CHỖ PHÁ PHỤ THUỘC VÒNG TRÒN
        // ====================================================================
        // NGUYÊN TẮC (xem takeoff_land.h đầu mục TAKEOFF): lệnh TAKEOFF MỞ
        // altitude dynamics; `airborne` là KẾT QUẢ detector xác nhận sau đó.
        // TUYỆT ĐỐI KHÔNG được viết lại thành "phải airborne trước thì Z/Vz mới
        // chạy" — đó là vòng tròn: ground lock khoá Z/Vz=0 -> bằng chứng Vz/Z
        // không bao giờ đạt -> score không vượt ngưỡng -> airborne mãi false.
        //
        // Đọc phase từ s_tko_state (state BỀN, do takeoff_run() ghi ở CUỐI tick
        // trước). Có 1 tick (~4ms) trễ tại cạnh TKO_PRIME->TKO_CLIMB — vô hại
        // và CÓ CHỦ ĐÍCH: bước 2 (estimator) chạy TRƯỚC bước 9 (throttle) trong
        // cùng tick, nên không thể lấy phase "của tick này" mà không đảo thứ tự
        // cả vòng lặp. KHÔNG dùng telemetry field làm nguồn sự thật.
        const bool tko_control_active = takeoff_control_active(&s_tko_state);
        const bool tko_airborne_confirmed = takeoff_airborne(&s_tko_state);

        // airborne = liftoff CONFIRMED (trong pha cất cánh) HOẶC đã ở các state
        // hậu-cất-cánh. MỘT định nghĩa duy nhất, dùng chung cho estimator +
        // Commander + gate attitude. BENCH_RAMP không bao giờ airborne.
        const bool airborne_for_alt =
            tko_airborne_confirmed ||
            s_fsm.state == FSM_HOLDING || s_fsm.state == FSM_FLYING ||
            s_fsm.state == FSM_LANDING;

        // altitude_dynamics_active — semantics spec §6: estimator được phép
        // tích phân Z/Vz. True từ TKO_CLIMB, KHÔNG phải từ TKO_PRIME: trong
        // PRIME drone chắc chắn còn trên đất và motor đang rung mạnh nhất, tích
        // phân lúc đó chỉ nạp nhiễu vào Z/Vz trước khi có gì thật để đo.
        const bool altitude_dynamics_active = tko_control_active || airborne_for_alt;

        // liftoff_candidate của estimator = "tích phân NHƯNG chưa fuse baro".
        // Đúng cửa sổ TKO_CLIMB trước khi liftoff: sát đất là lúc baro
        // nhiễu nhất (propwash + ground effect), kéo Z về baro khi drone mới
        // nhấc vài cm sẽ triệt tiêu đúng phần vừa tích phân được. Sau khi
        // confirm -> airborne -> fusion bật lại đầy đủ. Baro KHÔNG BAO GIỜ là
        // liftoff detector và KHÔNG BAO GIỜ tạo Vz bằng đạo hàm (spec §12).
        const bool liftoff_candidate_for_alt =
            altitude_dynamics_active && !airborne_for_alt;

        // "stationary" cho ZUPT + học accel_bias (xem alt_estimator.h đầu
        // file) — CHỈ true khi CHƯA airborne (KHÔNG còn giới hạn cứng theo
        // FSM_DISARMED — bản cũ bỏ sót trường hợp ARMED đứng yên chờ lệnh
        // takeoff hàng chục giây, khiến bias không kịp hội tụ VÀ z_inertial
        // trôi tự do, xem alt_estimator.h) VÀ gyro/accel cho thấy đứng yên
        // thật NGAY TỨC THÌ (không phải trung bình cửa sổ như calib — ZUPT
        // quyết định lại mỗi tick). Rung động cơ lúc ARMED/BENCH_RAMP tự
        // nhiên làm rớt điều kiện gyro/accel này, không cần chặn cứng theo
        // FSM state nữa. Dùng imu.gyro_dps/accel_g SAU calib 6-face (bước
        // 1c) — trạng thái "đứng yên" phải đo trên số liệu ĐÃ hiệu chỉnh.
        const float accel_norm_for_stationary = sqrtf(imu.accel_g.x * imu.accel_g.x +
                                                        imu.accel_g.y * imu.accel_g.y +
                                                        imu.accel_g.z * imu.accel_g.z);
        // THÊM !s_tko_state.active: bias learning phải FREEZE từ PRIME trở đi
        // (spec §17), KHÔNG chỉ từ lúc tích phân bắt đầu. Trong PRIME motor đã
        // quay và rung — để bộ học chạy lúc đó là để rung động cơ kéo
        // accel_bias sai NGAY TRƯỚC khoảnh khắc ta bắt đầu tin vào nó. Điều
        // kiện gyro/accel quiet thường tự rớt khi motor quay, nhưng "thường" là
        // không đủ cho thứ này: chặn tường minh theo phase.
        const bool stationary_for_alt =
            !airborne_for_alt && !altitude_dynamics_active && !s_tko_state.active &&
            fabsf(imu.gyro_dps.x) < ALT_EST_STATIONARY_GYRO_DPS &&
            fabsf(imu.gyro_dps.y) < ALT_EST_STATIONARY_GYRO_DPS &&
            fabsf(imu.gyro_dps.z) < ALT_EST_STATIONARY_GYRO_DPS &&
            fabsf(accel_norm_for_stationary - 1.0f) < ALT_EST_STATIONARY_ACCEL_TOL_G;

        if (!attitude_valid) {
            alt_estimator_reset(&s_alt_est);
        } else if (imu_updated) {
            // CÙNG GATE với Mahony và CÙNG fusion_dt: bước PREDICT của
            // estimator là tích phân accel (vz += az*dt; z += ...), chạy lại
            // trên mẫu accel cũ sẽ sinh ra Vz/Z hoàn toàn bịa — và bịa theo
            // hướng NHẤT QUÁN (cùng một az lặp lại), nên nó không tự triệt
            // tiêu như nhiễu mà cộng dồn thành một quỹ đạo giả trông rất hợp lý.
            //
            // Truyền seq + timestamp baro/ToF THÔ: estimator TỰ quyết định "mẫu
            // mới" và tự tính dt_baro thật (~20ms). Trước đây flight_core.c
            // tự lọc rồi truyền 1 cờ bool `baro_ok` — cùng kết quả nhưng API
            // mơ hồ (một caller khác truyền nhầm "baro còn khoẻ" vào đó sẽ
            // làm correction chạy 250Hz mà không có gì báo lỗi).
            alt_estimator_update(&s_alt_est, imu.accel_g, mahony_quaternion(&s_mahony),
#if FC_FEATURE_BARO
                                  baro_healthy_for_alt, snap.baro_h.seq,
                                  snap.baro_h.timestamp_us, baro.alt_m,
#endif
                                  tof_healthy_for_alt, snap.tof_h.seq,
                                  snap.tof_h.timestamp_us, tof.distance_m,
                                  tof_hw_alive, stationary_for_alt,
                                  liftoff_candidate_for_alt,
                                  airborne_for_alt, now_us, fusion_dt);
        }

        // ---- 2a-bis) BÁO KHI ĐỔI NGUỒN GIỮ ĐỘ CAO ----
        // Baro vốn ĐÃ tự gánh khi ToF chết — estimator chỉ đơn giản correction
        // từ nguồn nào còn mẫu được chấp nhận. Nhưng trước đây việc đó diễn ra
        // HOÀN TOÀN IM LẶNG: cùng một dòng telemetry cho "ToF tốt" lẫn "ToF
        // chết 10 giây, đang bay bằng baro". Với ToF chập chờn thì đó là khác
        // biệt quan trọng nhất, vì baro KHÔNG có tham chiếu tới mặt sàn (nó đo
        // áp suất, trôi theo thời tiết và bị propwash) — độ cao so với sàn có
        // thể sai vài chục cm mà mọi cờ vẫn xanh.
        if (s_alt_est.active_source != s_alt_source_prev) {
            const char *from = alt_source_name(s_alt_source_prev);
            const char *to   = alt_source_name(s_alt_est.active_source);
            if (s_alt_est.active_source == ALT_SRC_TOF_LOST) {
                ESP_LOGE(TAG, "DO CAO: %s -> TOF_LOST tai Z=%.2fm; Commander se ha canh",
                          from, (double)s_alt_est.alt_m);
            } else if (s_alt_est.active_source == ALT_SRC_TOF_SHORT_BRIDGE ||
                       s_alt_est.active_source == ALT_SRC_IMU_PREDICT_ONLY) {
                ESP_LOGW(TAG, "DO CAO: %s -> %s tai Z=%.2fm (bridge IMU ngan)",
                          from, to, (double)s_alt_est.alt_m);
            } else {
                ESP_LOGI(TAG, "DO CAO: nguon correction %s -> %s tai Z=%.2fm",
                          from, to, (double)s_alt_est.alt_m);
            }
            s_alt_source_prev = s_alt_est.active_source;
        }

        // ---- 2b) cập nhật ảnh chụp PRE-ARM (state NỘI BỘ) ----
        // apply_command()/prearm_check() đọc struct này, KHÔNG đọc ngược
        // telemetry. Telemetry chỉ đi MỘT CHIỀU ra ngoài — xem s_prearm.
        s_prearm.attitude_valid = attitude_valid;
        s_prearm.roll_deg = roll_deg;
        s_prearm.pitch_deg = pitch_deg;
        s_prearm.imu_fresh = imu_fresh;
        s_prearm.imu_healthy = snap.imu_h.healthy;
        s_prearm.gyro_calibrated = s_calib.gyro_valid;
        s_prearm.accel_calibrated = s_calib.accel_valid;
        s_prearm.mag_ok_for_heading = mag_usable;
        s_prearm.battery_sample_ok = snap.battery_h.valid;
        s_prearm.battery_v = battery_v;
#if FC_FEATURE_HOVER_LATCH
        // Nạp vòng đệm vbat cho latch hover lúc ARM. Chạy MỌI TICK nhưng
        // hover_vbat_push() tự lọc theo seq -> chỉ nhận mẫu THẬT SỰ mới (~10Hz),
        // không để một mẫu bị đếm 25 lần làm hỏng trung vị.
        //
        // Dùng snap.battery.voltage_v (giá trị THÔ của driver) chứ KHÔNG dùng
        // battery_v: battery_v đã bị ép về 0.0f khi health invalid (quy ước sẵn
        // có), mà 0.0f là một SỐ ĐO GIẢ — nạp nó vào trung vị sẽ kéo hover
        // xuống. Cờ valid truyền riêng để push() tự bỏ mẫu hỏng.
        hover_vbat_push(&s_vbat_ring, snap.battery.voltage_v,
                         snap.battery_h.seq, snap.battery_h.valid);
#endif
        s_prearm.alt_estimator_valid = s_alt_est.valid;
        s_prearm.tof_floor_ready = alt_estimator_floor_ready(&s_alt_est);
        s_prearm.loop_healthy = (s_deadline_miss_streak < COMMANDER_DEADLINE_MISS_HARD);
        s_prearm_yaw_deg = yaw_deg;

        // ---- 3) publish telemetry (ĐẦU RA — không ai đọc ngược) ----
        // Đảo ở biên GIỐNG HỆT input CMD_SET_ATTITUDE/CMD_SET_TRIM/CMD_CONTROL/
        // CMD_SET_PID (xem giải thích đầu case CMD_SET_ATTITUDE): telemetry
        // "roll" hiển thị ra ngoài (console status/UDP STATUS) PHẢI là trục
        // tiến/lùi -> gán TỪ pitch_deg vật lý (Mahony), và ngược lại. roll_deg/
        // pitch_deg cục bộ ở đây VẪN LÀ Mahony gốc (không đổi) — chỉ đảo lúc
        // ghi vào field telemetry, không ảnh hưởng gì tới s_prearm/cin ở trên
        // (đã gán XONG, dùng field TRƯỚC swap này, đúng ý nghĩa vật lý chuẩn).
        xSemaphoreTake(s_telemetry_mtx, portMAX_DELAY);
        s_telemetry.roll_deg = pitch_deg;
        s_telemetry.pitch_deg = roll_deg;
        s_telemetry.yaw_deg = yaw_deg;
        s_telemetry.gyro_roll_dps = imu.gyro_dps.y;
        s_telemetry.gyro_pitch_dps = imu.gyro_dps.x;
        s_telemetry.gyro_yaw_dps = imu.gyro_dps.z;
        s_telemetry.accel_x_g = imu.accel_g.x;
        s_telemetry.accel_y_g = imu.accel_g.y;
        s_telemetry.accel_z_g = imu.accel_g.z;
        s_telemetry.attitude_valid = attitude_valid;
        // BUG ĐÃ SỬA: trước đây gán = s_hub_notify_active (cờ "sensor_hub có
        // đang đánh thức stabilize_task không") — cờ đó LUÔN true một khi hub
        // start thành công, kể cả khi ngắt phần cứng CHƯA BAO GIỜ chạy (hub tự
        // rơi về đồng hồ FreeRTOS nội bộ mà vẫn đánh thức đều). Kết quả: field
        // này từng báo "1" ngay cả khi imu_int_isr_count=0 — trông như ngắt
        // đang hoạt động trong khi thực ra dây INT/cấu hình ngắt chưa bao giờ
        // chạy được. Nguồn ĐÚNG là sensor_hub_imu_int_active() (cờ nội bộ
        // s_imu_int_active của sensor_hub.c, phản ánh CHÍNH XÁC "đã bật ngắt
        // MPU6050 thành công và chưa rơi về polling").
        s_telemetry.imu_int_active = sensor_hub_imu_int_active();
        s_telemetry.imu_int_isr_count = imu_driver_int_isr_count();
        s_telemetry.imu_int_wake_count = s_imu_int_wake_count;
        s_telemetry.imu_int_timeout_count = s_imu_int_timeout_count;
        s_telemetry.imu_no_new_sample_count = s_imu_no_new_sample_count;
        s_telemetry.acc_norm_g = s_mahony.status.acc_norm;
        s_telemetry.accel_used = s_mahony.status.accel_used;
        s_telemetry.mag_used = s_mahony.status.mag_used;
        s_telemetry.mag_rejected = s_mahony.status.mag_rejected;
        s_telemetry.mag_reference_valid = s_mahony.status.mag_reference_valid;
        s_telemetry.mag_norm = s_mahony.status.mag_norm;
        s_telemetry.mag_error_norm = s_mahony.status.mag_error_norm;
        s_telemetry.mag_used_count = s_mag_used_count;
        s_telemetry.mag_rejected_count = s_mag_rejected_count;
        s_telemetry.yaw_rel_deg = wrap_deg_180(yaw_deg - s_yaw_ref_offset_deg);
        s_telemetry.alt_m = s_alt_est.alt_m;
        s_telemetry.vz_ms = s_alt_est.vz_ms;
        s_telemetry.alt_valid = s_alt_est.valid;
        s_telemetry.battery_v = battery_v;
        s_telemetry.imu_ok_driver = s_imu_ok_driver;
        s_telemetry.mag_ok_driver = s_mag_ok_driver;
        s_telemetry.baro_ok_driver = s_baro_ok_driver;
        s_telemetry.tof_ok_driver = s_tof_ok_driver;
        s_telemetry.battery_ok_driver = s_battery_ok_driver;
        s_telemetry.tof_range_m = tof.distance_m;
        s_telemetry.tof_valid = tof.valid;
        s_telemetry.tof_vertical_m = s_alt_est.tof_vertical_m;
        s_telemetry.tof_innovation_m = s_alt_est.tof_innovation_m;
        s_telemetry.tof_surface_z_m = s_alt_est.tof_surface_z_m;
        s_telemetry.tof_surface_state = (int32_t)s_alt_est.tof_surface_state;
        s_telemetry.tof_correction_enabled = s_alt_est.tof_correction_enabled;
        s_telemetry.tof_ground_range_m = s_alt_est.tof_ground_range_m;
        s_telemetry.tof_z_m = s_alt_est.tof_z_m;
        s_telemetry.tof_vz_ms = s_alt_est.tof_vz_lpf_ms;
        s_telemetry.tof_vz_valid = s_alt_est.tof_vz_valid;
        s_telemetry.tof_fusable = s_alt_est.tof_fusable;
        s_telemetry.tof_track_state = (int)s_alt_est.tof_track_state;
        s_telemetry.floor_locked = s_alt_est.floor_locked;
        s_telemetry.floor_ready = alt_estimator_floor_ready(&s_alt_est);
        s_telemetry.floor_sample_count = s_alt_est.floor_sample_count;
        s_telemetry.floor_std_m = s_alt_est.floor_std_m;
        s_telemetry.baro_used_by_flight_control = false;
        s_telemetry.az_body_z_g = s_alt_est.az_body_z_g;
        s_telemetry.az_earth_raw_ms2 = s_alt_est.az_earth_raw_ms2;
        s_telemetry.az_after_gravity_ms2 = s_alt_est.az_after_gravity_ms2;
        s_telemetry.az_after_bias_ms2 = s_alt_est.az_after_bias_ms2;
        s_telemetry.tof_accept_count = s_alt_est.tof_accept_count;
        s_telemetry.tof_reject_count = s_alt_est.tof_reject_count;
        s_telemetry.floor_plane_z_m = s_alt_est.floor_plane_z_m;
        s_telemetry.landing_surface_z_m = s_alt_est.landing_surface_z_m;
        s_telemetry.landing_surface_valid = s_alt_est.landing_surface_valid;
        // ---- Telemetry baro ----
        // ⚠ NGOẠI LỆ CÓ CHỦ ĐÍCH của việc compile-out baro: các FIELD baro trong
        // telemetry_snapshot_t và các token BALT=/BFILT=/BINNOV=/BACC=/BREJ=/
        // BDT=/BCREJ=/BREACQ=/BSEQ=/BFI=/BAROCORRZ=... trong STATUS VẪN ĐƯỢC
        // GIỮ khi FC_FEATURE_BARO=0, chỉ phát ra 0.
        //
        // VÌ SAO KHÔNG bỏ luôn cho nhẹ: mấy token đó nằm RẢI RÁC GIỮA dòng
        // STATUS, không phải ở cuối. STATUS_RE bên tools/uav_udp_console.py bắt
        // theo GROUP INDEX cố định, nên bỏ một token ở giữa sẽ DỊCH toàn bộ
        // index phía sau -> GUI đọc sai field mà KHÔNG báo lỗi. Chính
        // telemetry_format.c đã cảnh báo đúng chuyện này hai lần ("them field
        // vao GIUA se lam DICH moi group index... da mot lan suyt lam GUI hong
        // IM LANG").
        //
        // GIÁ PHẢI TRẢ, đo được chứ không đoán: ~15 field trong telemetry_t
        // (~60 B RAM) + phần chuỗi format (~200 B flash). Đổi lại: wire format
        // BẤT BIẾN giữa hai cấu hình, cùng một GUI/log parser dùng được cho cả
        // hai. Toàn bộ phần ĐẮT (driver BMP280, state trong alt_estimator,
        // nhánh CORRECT #2, giao dịch I2C) thì ĐÃ biến mất thật.
#if FC_FEATURE_BARO
        s_telemetry.baro_alt_m = baro.alt_m;
        s_telemetry.baro_pressure_pa = baro.pressure_pa;
        s_telemetry.baro_filtered_alt_m = s_alt_est.baro_lpf_alt_m;
        s_telemetry.baro_innovation_m = s_alt_est.baro_innovation_m;
        s_telemetry.baro_dt_s = s_alt_est.baro_dt_s;
        s_telemetry.baro_accept_count = s_alt_est.baro_accept_count;
        s_telemetry.baro_reject_count = s_alt_est.baro_reject_count;
        s_telemetry.baro_reject_consecutive = s_alt_est.baro_reject_consecutive;
        s_telemetry.baro_reacquire_active = s_alt_est.baro_reacquire_active;
        s_telemetry.baro_fusion_initialized = s_alt_est.baro_fusion_initialized;
        s_telemetry.baro_seq = snap.baro_h.seq;
#else
        s_telemetry.baro_alt_m = 0.0f;
        s_telemetry.baro_pressure_pa = 0.0f;
        s_telemetry.baro_filtered_alt_m = 0.0f;
        s_telemetry.baro_innovation_m = 0.0f;
        s_telemetry.baro_dt_s = 0.0f;
        s_telemetry.baro_accept_count = 0;
        s_telemetry.baro_reject_count = 0;
        s_telemetry.baro_reject_consecutive = 0;
        s_telemetry.baro_reacquire_active = false;
        s_telemetry.baro_fusion_initialized = false;
        s_telemetry.baro_seq = 0;
#endif
        s_telemetry.alt_airborne = airborne_for_alt;
        s_telemetry.alt_liftoff_candidate = liftoff_candidate_for_alt;
        s_telemetry.alt_degraded = s_alt_est.degraded;
        s_telemetry.alt_no_correction_ms = s_alt_est.no_correction_ms;
        s_telemetry.alt_source = (uint8_t)s_alt_est.active_source;
        s_telemetry.tof_corr_z_m = s_alt_est.tof_corr_z_m;
        s_telemetry.tof_corr_vz_ms = s_alt_est.tof_corr_vz_ms;
#if FC_FEATURE_BARO
        s_telemetry.baro_corr_z_m = s_alt_est.baro_corr_z_m;
        s_telemetry.baro_corr_vz_ms = s_alt_est.baro_corr_vz_ms;
#else
        s_telemetry.baro_corr_z_m = 0.0f;
        s_telemetry.baro_corr_vz_ms = 0.0f;
#endif
        s_telemetry.bias_residual_m = s_alt_est.bias_residual_m;
        s_telemetry.bias_adapt_count = s_alt_est.bias_adapt_count;
#if FC_FEATURE_TOF
        s_telemetry.tof_built_in = s_board.tof_enabled;
#else
        s_telemetry.tof_built_in = false;
#endif
        s_telemetry.az_corrected_ms2 = s_alt_est.az_corrected_ms2;
        s_telemetry.vert_accel_ms2 = s_alt_est.az_lpf_ms2;
        s_telemetry.vert_accel_raw_ms2 = s_alt_est.az_raw_ms2;
        s_telemetry.accel_bias_ms2 = s_alt_est.accel_bias_ms2;
        s_telemetry.vz_accel_only_ms = s_alt_est.vz_accel_only_ms;
        s_telemetry.z_inertial_m = s_alt_est.z_inertial_m;
#if FC_FEATURE_BARO
        s_telemetry.baro_calibrated = baro_driver_ground_ready();
        s_telemetry.baro_healthy = baro_driver_ground_healthy();
        s_telemetry.baro_ground_noise_std_pa = baro_driver_ground_noise_std_pa();
#else
        s_telemetry.baro_calibrated = false;
        s_telemetry.baro_healthy = false;
        s_telemetry.baro_ground_noise_std_pa = 0.0f;
#endif
        s_telemetry.sensor_err_count = s_sensor_err_count;
        // Tuổi mẫu: INT64_MAX (chưa từng có mẫu) -> kẹp về -1 để phía Python/GUI
        // phân biệt được "chưa có dữ liệu" với "vừa mới" mà không tràn int32.
        s_telemetry.imu_age_ms  = (imu_age_us  == INT64_MAX) ? -1 : (int32_t)(imu_age_us  / 1000);
        s_telemetry.mag_age_ms  = (mag_age_us  == INT64_MAX) ? -1 : (int32_t)(mag_age_us  / 1000);
        s_telemetry.baro_age_ms = (baro_age_us == INT64_MAX) ? -1 : (int32_t)(baro_age_us / 1000);
        s_telemetry.tof_age_ms = (tof_age_us == INT64_MAX) ? -1 : (int32_t)(tof_age_us / 1000);
        // Tuoi lan cuoi CHIP DO DUOC. snap.tof_alive_us == 0 nghia la chua
        // tung doc duoc mau nao -> -1, KHONG duoc bao cao thanh 0 (0 se doc
        // ra thanh 'vua do xong').
        s_telemetry.tof_alive_ms = (snap.tof_alive_us == 0)
            ? -1 : (int32_t)((now_us - snap.tof_alive_us) / 1000);
        s_telemetry.imu_healthy = snap.imu_h.healthy;
        s_telemetry.mag_healthy = snap.mag_h.healthy;
        s_telemetry.baro_healthy_hub = snap.baro_h.healthy;
        s_telemetry.heading_degraded = !mag_usable;
        s_telemetry.loop_dt_ms = loop_dt_ms;
        s_telemetry.loop_dt_us = (int32_t)s_loop_dt_us;
        s_telemetry.loop_max_us = (int32_t)s_loop_max_us;
        s_telemetry.deadline_miss_count = s_deadline_miss_count;
        s_telemetry.last_cmd_age_ms = (int32_t)((now_us - s_last_sp_update_us) / 1000);
        s_telemetry.heartbeat_age_ms = (int32_t)((now_us - s_cmd_state.last_heartbeat_us) / 1000);
        // Battery debug — lấy TỪ snapshot thô (không qua gate valid) để nhìn
        // được số sai khi nó sai, xem telemetry.h.
        {
            const int64_t bat_age_us = sensor_hub_age_us(&snap.battery_h, now_us);
            s_telemetry.bat_adc_raw = snap.battery.raw;
            s_telemetry.bat_adc_mv = snap.battery.adc_mv;
            s_telemetry.bat_divider_ratio = BATTERY_DIVIDER_RATIO;
            s_telemetry.bat_voltage_raw_v = snap.battery.voltage_v;
            s_telemetry.bat_valid = snap.battery.valid;
            s_telemetry.bat_calibrated = snap.battery.calibrated;
            s_telemetry.bat_age_ms = (bat_age_us == INT64_MAX) ? -1 : (int32_t)(bat_age_us / 1000);
#if FC_FEATURE_HOVER_LATCH
            s_telemetry.hover_latch_v    = s_hover_latch_v;
            s_telemetry.hover_latch_duty = s_hover_latch_duty;
            s_telemetry.hover_latched    = s_hover_latched;
#else
            // Cờ tắt: KHÔNG có latch nào cả. Publish 0/false thay vì bỏ trống
            // để dòng STATUS giữ nguyên số field (GUI đọc theo VỊ TRÍ).
            s_telemetry.hover_latch_v    = 0.0f;
            s_telemetry.hover_latch_duty = 0.0f;
            s_telemetry.hover_latched    = false;
#endif
        }
        xSemaphoreGive(s_telemetry_mtx);

        // ---- 4) rút HẾT command queue (Python chậm, xử lý ngay trong tick) ----
        //
        // ĐO THỜI GIAN quanh cả khối: vài lệnh debug chạy công việc CHẶN dài
        // NGAY TRONG task này (vd CMD_CALIB_BARO_GROUND ~970ms). Trong lúc đó không lệnh nào
        // được rút và commander_evaluate() không chạy, nên mọi đồng hồ watchdog
        // già đi bằng đúng khoảng chặn đó DÙ trạm mặt đất vẫn gửi đều.
        //
        // Không bù lại thì bấm ARM là tự disarm: quan sát trên phần cứng thật
        // HBAGE nhảy 155ms -> 1026ms, vượt heartbeat_timeout_ms (1000ms) ->
        // soft fault -> DISARMED ngay tick sau khi vừa arm xong.
        command_t cmd;
        const int64_t cmd_drain_start_us = esp_timer_get_time();
        while (xQueueReceive(s_cmd_queue, &cmd, 0) == pdTRUE) {
            apply_command(&cmd, now_us);
        }
        {
            const int64_t after_us = esp_timer_get_time();
            const int64_t drain_us = after_us - cmd_drain_start_us;
            // Ngưỡng = 1 chu kỳ điều khiển: dưới mức đó là xử lý lệnh bình
            // thường, không phải "chiếm" thời gian của ai.
            if (drain_us > CONTROL_DEADLINE_US) {
                commander_credit_stall(&s_cmd_state, drain_us);
                // KHÔNG cho phép đẩy mốc vào TƯƠNG LAI: nếu lệnh vừa xử lý
                // chính là CMD_HEARTBEAT thì last_heartbeat_us đã = now_us rồi,
                // cộng thêm nữa sẽ thành "heartbeat của tương lai" và watchdog
                // ngủ quên đúng bằng chừng đó.
                if (s_cmd_state.last_heartbeat_us > after_us) {
                    s_cmd_state.last_heartbeat_us = after_us;
                }
                // Cú stall này do CHÍNH TA gây ra, không phải vòng bay hụt hạn.
                // Không xoá thì tick kế tiếp thấy dt khổng lồ và cộng một
                // deadline-miss oan, làm bẩn cả loop_max lẫn prearm.loop_healthy.
                s_last_tick_us = after_us;
                // LOG ở ngưỡng CAO HƠN NHIỀU so với ngưỡng bù. Bù từ 1 chu kỳ
                // là đúng (mọi ms bị chiếm đều phải trả lại), nhưng LOG ở đó
                // thì một đợt xử lý lệnh bình thường ~4ms cũng kêu và ngập
                // console — đã gặp trên phần cứng thật. Chỉ báo khi thật sự
                // bất thường: >=50ms là dấu hiệu có công việc CHẶN, không phải
                // xử lý lệnh thường.
                if (drain_us >= 50000) {
                    ESP_LOGW(TAG, "xu ly lenh chiem %lldms cua vong bay -> da bu lai watchdog "
                                  "(neu khong se tu disarm ngay sau ARM)",
                              (long long)(drain_us / 1000));
                }
            }
        }

        // ---- 4a) KILL LATCH GATE — THOÁT TICK NGAY, KHÔNG chạy bất kỳ đường
        // tính duty nào phía dưới. Đặt NGAY SAU bước rút queue có chủ đích:
        // CMD_ARM (đường DUY NHẤT hạ latch) phải được xử lý trước, nếu không
        // sẽ không bao giờ thoát được latch. Mọi thứ sau điểm này — FSM,
        // takeoff/land, attitude cascade, mixer, ghi motor — đều BỊ BỎ QUA.
        //
        // Đây là điểm mấu chốt của lỗi cũ: trước đây hard fault chỉ gọi
        // motor_driver_stop_all() rồi ĐỂ CODE CHẠY TIẾP, nên attitude cascade +
        // mixer ở bước 10 tính duty mới và BẬT LẠI MOTOR trong CÙNG MỘT TICK.
        if (s_motor_kill_latched) {
            motor_driver_all_off();   // re-assert mỗi tick, không tin vào trạng thái trước
            if (s_fsm.state != FSM_DISARMED) {
                fsm_transition(&s_fsm, FSM_DISARMED, now_us);
                reset_all_controllers();
            }
            s_timed_active = false;
            s_sp_roll_deg = s_sp_pitch_deg = s_sp_yaw_rate_dps = 0.0f;

            publish_kill_tick_telemetry(now_us);
            continue;   // END TICK
        }

        // ---- 4b) setpoint bay tay từ ground-station UDP (@SP SET) ----
        // Watchdog stale: mất CMD_SET_ATTITUDE mới quá SP_STALE_TIMEOUT_US ->
        // tự zero (xem command.h). Độc lập với Commander heartbeat (1000ms) —
        // đây là lớp BẢO VỆ RIÊNG cho chính setpoint, phản ứng nhanh hơn.
        {
            const bool sp_stale = (now_us - s_last_sp_update_us) > SP_STALE_TIMEOUT_US;
            if (sp_stale) {
                if (!s_sp_stale_prev &&
                    (s_sp_roll_deg != 0.0f || s_sp_pitch_deg != 0.0f || s_sp_yaw_rate_dps != 0.0f)) {
                    ESP_LOGW(TAG, "SP setpoint stale (qua %lldms khong co CMD_SET_ATTITUDE/CMD_CONTROL moi) -> ve 0",
                              (long long)(SP_STALE_TIMEOUT_US / 1000));
                }
                s_sp_roll_deg = 0.0f;
                s_sp_pitch_deg = 0.0f;
                s_sp_yaw_rate_dps = 0.0f;
                s_ctrl_rol_pct = 0.0f;
                s_ctrl_pit_pct = 0.0f;
                s_ctrl_yaw_pct = 0.0f;
                s_ctrl_thr_pct = 0.0f;
            } else if ((s_fsm.state == FSM_HOLDING || s_fsm.state == FSM_FLYING) && s_ctrl_thr_pct != 0.0f) {
                // fc.control() thr -> SLEW target altitude (KHÔNG step), chỉ khi
                // alt_hold thực sự đang chạy. Freeze (không slew) ngoài
                // HOLDING/FLYING — vd TAKING_OFF/LANDING tự quản lý target riêng.
                s_alt_request_m += (s_ctrl_thr_pct * 0.01f) * CONTROL_ALT_SLEW_MPS * dt;
                s_alt_request_m = commander_clamp_altitude(&s_cmd_cfg, s_alt_request_m);
            }
            if (s_fsm.state == FSM_HOLDING || s_fsm.state == FSM_FLYING) {
                const float step = CONTROL_ALT_SLEW_MPS * dt;
                const float rem = s_alt_request_m - s_alt_target_m;
                if (rem > step) s_alt_target_m += step;
                else if (rem < -step) s_alt_target_m -= step;
                else s_alt_target_m = s_alt_request_m;
                s_alt_target_m = commander_clamp_altitude(&s_cmd_cfg, s_alt_target_m);
            }
            s_sp_stale_prev = sp_stale;

            // ---- 4c) Offset throttle của phím GIỮ (W/S) ----
            // Watchdog stale + áp dụng, đặt Ở ĐÂY (mỗi tick, MỌI state) chứ
            // không trong nhánh FSM_BENCH_RAMP của bước 9 như trước.
            //
            // VÌ SAO PHẢI RA KHỎI NHÁNH ĐÓ: giờ offset còn có nghĩa ở
            // HOLDING/FLYING. Nếu watchdog vẫn nằm trong case BENCH_RAMP thì
            // ground-station chết giữa lúc người dùng đang GIỮ W trong khi BAY
            // sẽ để offset dương dính lại VĨNH VIỄN — target độ cao bò lên tới
            // trần geofence không ai điều khiển. Đó đúng là kịch bản mà
            // BENCH_OFFSET_STALE_US sinh ra để chặn, nên nó phải chạy ở mọi
            // state có dùng offset, không riêng bench.
            if (s_bench_throttle_offset != 0 &&
                (now_us - s_bench_offset_update_us) > BENCH_OFFSET_STALE_US) {
                ESP_LOGW(TAG, "offset ga (phim giu W/S) stale qua %lldms -> ve 0",
                          (long long)(BENCH_OFFSET_STALE_US / 1000));
                s_bench_throttle_offset = 0;
            }
            // ÁP DỤNG offset không nằm ở đây nữa — xem nhánh FSM_HOLDING/
            // FSM_FLYING ở bước 9. Ở đó offset cộng THẲNG vào duty (đúng nghĩa
            // "W/S = +/-100 throttle"), thay cho bản trước biến nó thành lệnh
            // trượt target độ cao. Chỉ giữ WATCHDOG ở đây vì nó phải chạy ở
            // MỌI state, kể cả state không dùng offset.

            // Chỉ đổi FSM HOLDING<->FLYING theo SP khi KHÔNG có timed-command
            // (CMD_MOVE/CMD_SET_YAW) đang chạy — timed-command tự quản lý cạnh
            // này ở bước 8. fsm_on_move_command() là hàm THUẦN (no-op nếu cạnh
            // không khớp state hiện tại) nên gọi mỗi tick an toàn, không cần
            // "edge detect" riêng.
            if (!s_timed_active) {
                const bool sp_active = fabsf(s_sp_roll_deg) > 0.01f ||
                                        fabsf(s_sp_pitch_deg) > 0.01f ||
                                        fabsf(s_sp_yaw_rate_dps) > 0.01f;

                // ---- HOLDING -> FLYING: TUC THI ----
                // Nguoi lai day can thi phai co tac dung ngay.
                if (sp_active) {
                    s_flying_idle_since_us = 0;
                    fsm_transition(&s_fsm, fsm_on_move_command(s_fsm.state, true), now_us);
                } else {
                    // ---- FLYING -> HOLDING: CHO YEN FLYING_TO_HOLD_SETTLE_MS ----
                    // Tha can KHONG doi state ngay. Luc vua tha, drone con
                    // nghieng va dang troi -> ToF (nhin theo truc than) chua on
                    // -> chot do cao vao dung tick do cho ra target sai, da quan
                    // sat thay vot len ~2m.
                    //
                    // Moc dem lai tu 0 moi khi co lenh dieu khien, nen 1s nay la
                    // "1s LIEN TUC khong dieu khien", khong phai 1s ke tu lan
                    // dau tien tha can.
                    if (s_fsm.state == FSM_FLYING) {
                        if (s_flying_idle_since_us == 0) {
                            s_flying_idle_since_us = now_us;
                        } else if ((now_us - s_flying_idle_since_us) >=
                                   (int64_t)FLYING_TO_HOLD_SETTLE_MS * 1000) {
                            s_flying_idle_since_us = 0;
                            fsm_transition(&s_fsm,
                                fsm_on_move_command(s_fsm.state, false), now_us);
                        }
                    } else {
                        s_flying_idle_since_us = 0;
                    }
                }
            }
        }

        // ---- 5) failsafe KHÔNG ĐỢI FSM: nghiêng quá / attitude stale -> cắt ngay ----
        const bool stale = !attitude_valid;   // stamp riêng không cần vì đo trong cùng vòng
        const bool tilt_fault = fabsf(roll_deg) > MAX_SAFE_TILT_DEG ||
                                 fabsf(pitch_deg) > MAX_SAFE_TILT_DEG;
        const bool armed_now = (s_fsm.state != FSM_DISARMED) && (s_fsm.state != FSM_ARMED);
        if (armed_now && (stale || tilt_fault)) {
            // LATCH, không chỉ stop_all(): đây là 2 trong các điều kiện
            // "uncontrollable hard fault" (mất attitude hoàn toàn khi đang bay
            // / nghiêng quá ngưỡng cứu được). Trước đây chỉ stop_all() rồi để
            // code chạy tiếp -> mixer ở bước 10 bật lại motor NGAY trong tick
            // đó. Latch làm tick này kết thúc ở bước 4a của vòng KẾ, và mọi
            // đường ghi duty bị chặn ở cả tầng logic lẫn tầng driver.
            enter_kill_latch(stale ? "attitude INVALID khi dang bay"
                                    : "tilt vuot MAX_SAFE_TILT_DEG");
            reset_all_controllers();
            if (s_fsm.state != FSM_DISARMED) fsm_transition(&s_fsm, FSM_DISARMED, now_us);
            publish_kill_tick_telemetry(now_us);
            continue;   // END TICK — không chạy Commander/FSM/mixer nữa
        }

        // ---- 6) Commander — CHẠY Ở MỌI STATE ĐÃ ARMED ----
        // Trước đây chỉ chạy ở HOLDING/FLYING, nghĩa là mất heartbeat lúc
        // TAKING_OFF, pin tụt lúc LANDING, hay IMU chết lúc ARMED đều KHÔNG
        // được phát hiện. Commander tự lọc theo cin.state (xem commander.c) —
        // caller không được lọc hộ nữa, đó chính là chỗ tạo ra lỗ hổng.
        {
            commander_inputs_t cin = {0};
            cin.state = s_fsm.state;
            // Dùng CHUNG định nghĩa với alt_estimator (bước 2) — một nguồn
            // sự thật duy nhất cho "đã bay hay chưa". Trong lúc còn PRIME thì
            // coi như CHƯA bay: drone bắt đầu pha đó từ mặt đất.
            cin.airborne = airborne_for_alt;
            cin.imu_ok = attitude_valid;
            // imu_stale ĐỘC LẬP với imu_ok: Mahony vẫn báo valid khi nhai lại
            // mẫu cũ. Đây là trường hợp nguy hiểm nhất vì mọi thứ trông vẫn
            // bình thường — xem commander.c.
            cin.imu_stale = (imu_age_us > SENSOR_IMU_STALE_US);
            cin.roll_deg = roll_deg;
            cin.pitch_deg = pitch_deg;
            cin.battery_v = battery_v;   // TỪ SNAPSHOT, không đọc ngược telemetry (xem bước 3)
            cin.motor_saturated = s_motor_saturated_prev;   // đo ở tick TRƯỚC, xem bước 10
            cin.alt_hold_engage_lost = s_alt_hold_engage_lost_prev;
            cin.alt_estimator_lost = !s_alt_est.valid;
            cin.alt_estimator_degraded = s_alt_est.degraded;
            cin.deadline_miss_streak = s_deadline_miss_streak;

            commander_result_t cr = commander_evaluate(&s_cmd_state, &s_cmd_cfg, &cin, now_us);
            if (cr.fault == FAULT_SOFT) {
                ESP_LOGW(TAG, "SOFT FAULT -> LANDING: %s", cr.reason);
                s_last_fault_class = FAULT_SOFT;
                fsm_state_t next = fsm_on_soft_fault(s_fsm.state);
                if (next != s_fsm.state) { landing_reset(&s_land_state); fsm_transition(&s_fsm, next, now_us); }
            } else if (cr.fault == FAULT_HARD) {
                ESP_LOGE(TAG, "HARD FAULT -> EMERGENCY: %s", cr.reason);
                s_last_fault_class = FAULT_HARD;
                // KHÔNG cắt motor tại đây. Hard fault -> EMERGENCY, và bước 7
                // (cùng tick) mới quyết định EMERGENCY giải quyết thành
                // LANDING (còn kiểm soát được -> CẦN motor để hạ êm) hay
                // DISARMED (mất kiểm soát -> enter_kill_latch() cắt cứng).
                // Lệnh stop_all() cũ ở đây là no-op gây hiểu nhầm: bước 10
                // cùng tick ghi đè duty ngay sau đó.
                fsm_transition(&s_fsm, fsm_on_hard_fault(s_fsm.state), now_us);
            }
        }

        // ---- 7) giải quyết EMERGENCY NGAY trong tick này (transient state) ----
        if (s_fsm.state == FSM_EMERGENCY) {
            const bool controllable = attitude_valid && s_alt_est.valid &&
                                       s_alt_est.alt_m > GROUNDED_ALT_M &&
                                       fabsf(roll_deg) < s_cmd_cfg.hard_tilt_deg &&
                                       fabsf(pitch_deg) < s_cmd_cfg.hard_tilt_deg;
            fsm_state_t next = fsm_on_emergency_resolve(s_fsm.state, controllable);
            if (next == FSM_LANDING) landing_reset(&s_land_state);
            fsm_transition(&s_fsm, next, now_us);
            if (next == FSM_DISARMED) {
                // EMERGENCY giải quyết thành DISARMED = KHÔNG kiểm soát được
                // (hoặc còn ở mặt đất). Đây đúng nghĩa "uncontrollable hard
                // fault" -> LATCH, không chỉ disarm: phải có ARM tường minh
                // của người dùng mới bay lại, không tự phục hồi.
                enter_kill_latch("EMERGENCY khong kiem soat duoc -> DISARMED");
                reset_all_controllers();
                alt_estimator_unlock_floor(&s_alt_est);
                publish_kill_tick_telemetry(now_us);
                continue;   // END TICK
            }
        }

        // ---- 8) hết hạn timed-command (move/set_yaw) -> về HOLDING ----
        if (s_timed_active && now_us >= s_timed_until_us) {
            s_timed_active = false;
            fsm_transition(&s_fsm, fsm_on_move_command(s_fsm.state, false), now_us);
        }

        // ---- 9) tính throttle theo state hiện tại ----
        int  throttle_cmd = 0;
        bool hold_integral_freeze = true;   // mặc định freeze khi không active-flying
        bool takeoff_prime_done = (s_fsm.state != FSM_TAKING_OFF);   // true nếu không/đã qua PRIME
        float alt_target_vz_ms = 0.0f;   // xem alt_hold_result_t.vz_target_ms, publish telemetry bước 11
        // Reset mỗi tick: chỉ nhánh HOLDING/FLYING mới set true. Không reset thì
        // một lần engage_lost sẽ dính vĩnh viễn và Commander báo fault mãi.
        s_alt_hold_engage_lost_prev = false;
        takeoff_phase_t takeoff_phase = TKO_IDLE;   // telemetry, chỉ nhánh TAKING_OFF set
        // Xoá mỗi tick: chỉ nhánh TAKING_OFF ghi. Không xoá thì sau khi bàn giao
        // sang HOLDING, telemetry vẫn báo score/evidence/z_sp của lần cất cánh
        // trước như thể chuỗi đang chạy.
        s_tko_result = (takeoff_result_t){0};

        // Rời FLYING (về HOLDING vì thả phím nghiêng, hoặc sang LANDING/
        // EMERGENCY/DISARMED vì bất kỳ lý do gì) -> XOÁ latch ga. Đặt ở ĐÂY,
        // ngay trước switch, thay vì rải ở từng fsm_transition(): có hơn 20 chỗ
        // chuyển state và chỉ cần bỏ sót MỘT chỗ là lần vào FLYING sau sẽ dùng
        // lại con số của chuyến bay trước — đúng loại lỗi im lặng khó lần nhất.
        // Một điều kiện duy nhất, kiểm mỗi tick, không thể bỏ sót.
        if (s_fsm.state != FSM_FLYING) s_flying_vz_engaged = false;
        if (s_fsm.state != FSM_FLYING && s_flying_throttle_latch >= 0) {
            ESP_LOGI(TAG, "roi FLYING -> xoa latch ga (%d duty), alt_hold nhan lai quyen giu do cao",
                      s_flying_throttle_latch);
            s_flying_throttle_latch = -1;
            // Xoa CUNG voi latch: credit con lai cua chuyen truoc se cho phep
            // mot buoc nhay ngay tick dau cua lan vao FLYING sau.
            s_flying_rise_credit = 0.0f;
            // ---- PHASE A4: BAN GIAO FLYING -> HOLDING LA BUMPLESS SAN ----
            // KHONG dung toi s_hold_state.vz_integral o day, VA DO LA CO Y.
            // Vong Vz trong FLYING vua chay tren CHINH bien I do, nen no dang
            // giu dung luong ga can de vz = 0. alt_hold_run() o tick sau thay
            // st->engaged == true nen KHONG goi alt_hold_preload() -- no tiep
            // tuc tu chinh gia tri nay. Do la dinh nghia cua bumpless.
            //
            // Reset I ve 0 o day se lam ga tut ~mot cuc hover ngay tick dau
            // cua HOLDING -> drone hut xuong dung luc vua tha can.
        }

        switch (s_fsm.state) {
            case FSM_DISARMED:
            case FSM_ARMED:
                throttle_cmd = 0;
                break;

            case FSM_BENCH_RAMP:
                // Đi thẳng từ state do CMD_BENCH_THROTTLE_STEP set (+/- 20
                // mỗi lần bấm ở console) — KHÔNG qua alt_hold/takeoff. Khi
                // throttle_cmd vượt ATT_MIN_THROTTLE_DUTY, attitude PID tự
                // chạy (logic CHUNG ở bước 10, không cần gì riêng ở đây) —
                // đúng yêu cầu "throttle >= ngưỡng thì PID bắt đầu chạy".
                // Watchdog stale của offset đã chuyển lên bước 4c (chạy MỌI
                // state) vì offset giờ còn dùng ở HOLDING/FLYING — xem lý do ở
                // đó. Ở đây chỉ còn việc ÁP DỤNG.
                //
                // BENCH_RAMP là state DUY NHẤT cộng offset THẲNG vào duty:
                // không có PID độ cao nào sở hữu throttle nên không có gì chống
                // lại nó. duty (đã chốt) + offset (tạm, chỉ khi đang giữ phím);
                // nhả phím -> offset về 0 -> throttle trả về ĐÚNG mức trước đó.
                throttle_cmd = clampi(s_bench_throttle_duty + s_bench_throttle_offset,
                                       0, MOTOR_SAFE_MAX_DUTY);
                hold_integral_freeze = false;
                break;

            case FSM_TAKING_OFF: {
                takeoff_result_t tr;
                // tilt cho guard lật: lấy trục nghiêng LỚN HƠN, không phải tổng
                // — 40 roll cộng 40 pitch không nguy hiểm bằng 50 trên một trục.
                const float tko_tilt_deg = fmaxf(fabsf(roll_deg), fabsf(pitch_deg));
                // alt_source_ok = CÒN ÍT NHẤT MỘT nguồn correction độ cao.
                // TRƯỚC ĐÂY truyền thẳng tof_healthy_for_alt vào đây, và đó là
                // lý do "mọi thứ báo sẵn sàng nhưng không cất cánh được": ToF
                // không init được (hoặc bị tắt) => điều kiện abort đúng ngay từ
                // tick đầu của CLIMB => TKO_ABORT_TOF_LOST sau 300ms, MỖI LẦN.
                // Nó cũng abort giữa chừng khi leo quá tầm ToF (~1.8m) dù cảm
                // biến hoàn toàn lành. Xem takeoff_land.h phần alt_source_ok.
                takeoff_run(&s_tko_state, &s_tko_tune, &s_hold_state, &s_hold_tune,
                            s_alt_est.valid, s_alt_est.alt_m, s_alt_est.vz_ms,
                            s_alt_est.tof_fusable && tof_healthy_for_alt,
                            s_alt_est.tof_z_m, s_alt_est.tof_vz_lpf_ms,
                            tko_tilt_deg,
                            dt, now_us, MOTOR_SAFE_MAX_DUTY, &tr);
                if (tr.liftoff_edge) alt_estimator_confirm_liftoff(&s_alt_est);
                throttle_cmd = tr.throttle_duty;
                takeoff_phase = tr.phase;
                takeoff_prime_done = (tr.phase != TKO_PRIME);
                alt_target_vz_ms = tr.vz_target_ms;
                s_tko_result = tr;   // telemetry bước 11

                // Ki ATTITUDE mở theo LIFTOFF, không theo "đã qua PRIME". Giữa
                // lúc rời PRIME và lúc thật sự nhấc, drone VẪN đang đè mặt đất
                // trong khi controller đẩy ga lên — đó chính là cửa sổ dễ
                // ground-windup nhất, mở Ki ở đó là mở đúng lúc sai nhất. Bước
                // 9c còn AND thêm gate ga (ATT_I_ENABLE_THROTTLE_DUTY).
                // (Ki ALTITUDE có trần RIÊNG bên trong takeoff_run(), độc lập.)
                hold_integral_freeze = !tr.liftoff_flag;

                if (tr.abort) {
                    // ---- MỌI LÝ DO ABORT -> EMERGENCY ----
                    // KHÔNG tự chọn policy trước/sau liftoff ở đây nữa:
                    // EMERGENCY đã có sẵn bộ phân giải đúng cho việc đó (bước 7
                    // cùng tick) — nó xét attitude_valid + alt_est.valid + alt
                    // vượt GROUNDED_ALT_M + tilt, rồi ra LANDING (đang trên
                    // không, còn kiểm soát) hoặc DISARMED + kill latch (còn ở
                    // mặt đất / mất kiểm soát). Hai bộ luật song song cho cùng
                    // một quyết định là cách chắc chắn để chúng lệch nhau.
                    const char *why =
                        (tr.abort_reason == TKO_ABORT_TOF_LOST)   ? "ALT_SOURCE_LOST (mat HET ToF va baro qua lau)" :
                        (tr.abort_reason == TKO_ABORT_TILT)       ? "TILT (nghieng vuot nguong sap lat)" :
                        (tr.abort_reason == TKO_ABORT_NOT_ACTIVE) ? "NOT_ACTIVE (LOI LUONG: takeoff_run goi khi chua "
                                                                    "takeoff_begin — bao cao ngay, day KHONG phai su co bay)" :
                        (tr.abort_reason == TKO_ABORT_STUCK)      ? "STUCK (KHONG NHAC NOI: ga nam KICH TRAN lien tuc. "
                                                                    "Kiem theo thu tu: canh quat co bi vuong/lap nguoc khong -> "
                                                                    "pin con bao nhieu (xem BATV) -> khoi luong co vuot suc nang "
                                                                    "khong -> hover_ff (ALT_HOLD_HOVER_NOMINAL) co dat qua thap "
                                                                    "so voi hover THAT khong)" :
                        (tr.abort_reason == TKO_ABORT_NO_LIFT_EVIDENCE)
                                                                  ? "NO_LIFT (het NO_LIFT_TIMEOUT trong CLIMB ma ToF van doc "
                                                                    "duoi nguong do cao). Chuoi KHONG he bi ket, PID van chay "
                                                                    "suot — drone that su khong di len. Kiem: canh quat vuong/lap "
                                                                    "nguoc -> pin (BATV) -> tai trong -> hover_ff qua thap" :
                                                                    "TIMEOUT (qua han toan chuoi — chuoi bi ket, khong tien pha)";
                    ESP_LOGE(TAG, "TAKEOFF ABORT: %s (liftoff=%d, Z=%.2f, t=%.1fs) -> EMERGENCY",
                              why, (int)tr.liftoff_flag, (double)s_alt_est.alt_m,
                              (double)tr.elapsed_s);
                    if (tr.abort_reason == TKO_ABORT_TIMEOUT) {
                        // Deadline tong chuoi. hold_ready gio chi con HAI ve
                        // (xem takeoff_land.c) — in dung hai ve do.
                        //
                        // Truoc day in ca |Z-tgt| va |Vz| vi chung LA dieu kien;
                        // gio chung khong con la dieu kien nua nen in ra se noi
                        // doi ve ly do abort. Van in Z/Vz o dong tren de chan
                        // doan, nhung KHONG kem nguong nhu the chung con quyet
                        // dinh gi.
                        ESP_LOGE(TAG, "  hold_ready: lift=%d | zsp=%.3f==tgt=%.3f ? %d "
                                      "(Z=%.2f Vz=%.2f — chi de tham khao, KHONG con la dieu kien)",
                                  (int)tr.liftoff_flag,
                                  (double)tr.target_z_m, (double)tr.final_target_m,
                                  (int)(tr.target_z_m == tr.final_target_m),
                                  (double)s_alt_est.alt_m, (double)s_alt_est.vz_ms);
                    }
                    if (tr.abort_reason == TKO_ABORT_NO_LIFT_EVIDENCE) {
                        // In SO DO THAT kem NGUONG DA AP DUNG. Nguong co theo
                        // target (min voi target*FRAC) nen phai in ra ca hai,
                        // khong the bat nguoi doc tu nhan lai.
                        const float nl_thr = fminf(TAKEOFF_NO_LIFT_ALT_M,
                                                    tr.final_target_m * TAKEOFF_NO_LIFT_TARGET_FRAC);
                        ESP_LOGE(TAG, "  ToF doc %.3fm < nguong %.3fm sau %dms (target=%.2fm, "
                                      "collective=%d, hover_ff=%.0f, I=%+.0f, fusable=%d)",
                                  (double)s_alt_est.tof_z_m, (double)nl_thr,
                                  TAKEOFF_NO_LIFT_TIMEOUT_MS, (double)tr.final_target_m,
                                  tr.throttle_duty, (double)tr.hover_ff, (double)tr.vz_i_term,
                                  (int)(s_alt_est.tof_fusable && tof_healthy_for_alt));
                    }
                    s_last_fault_class = FAULT_HARD;
                    fsm_transition(&s_fsm, fsm_on_hard_fault(s_fsm.state), now_us);
                    break;
                }

                if (tr.handoff) {
                    // BÀN GIAO LIỀN MẠCH: target = target CỦA LỆNH (KHÔNG phải Z
                    // nhiễu hiện tại). alt_hold_state_t đang engaged với
                    // vz_integral = hover THẬT đã học trong chuỗi, nên
                    // alt_hold_run() ở tick sau tiếp tục trên CÙNG cascade từ
                    // CHÍNH state đó — KHÔNG preload/reset/đổi throttle gì thêm.
                    s_alt_target_m = tr.final_target_m;
                    s_alt_request_m = tr.final_target_m;

#if FC_FEATURE_HOVER_LATCH
                    // ========================================================
                    // TRẢ hover_ff VỀ HẰNG SỐ — latch CHỈ dùng cho TAKEOFF
                    // ========================================================
                    // (yêu cầu người dùng). Latch theo pin sinh ra để pha PRIME/
                    // CLIMB có một điểm khởi đầu đúng — lúc đó chưa có sai số độ
                    // cao nào để I-term học, nên feed-forward phải tự đúng.
                    //
                    // Từ HOLDING trở đi thì KHÔNG cần nữa: vòng Z/Vz đã đóng, và
                    // I-term đo được ga hover THẬT (kể cả phần latch đoán sai, kể
                    // cả pin đã sụt dưới tải — thứ latch không bao giờ biết).
                    //
                    // ⚠ PHẢI BÙ NGƯỢC VÀO I, KHÔNG ĐƯỢC ĐỔI hover TRẦN TRỤI.
                    // Cascade là:  output = hover + I  (alt_hold.c)
                    // Đổi hover từ 1219 -> 1000 mà để I nguyên là output TỤT 219
                    // duty NGAY MỘT TICK, đúng lúc drone vừa lên tới độ cao đích.
                    // Dồn đúng lượng đó sang I thì tổng không đổi -> bàn giao vẫn
                    // liền mạch, chỉ là "cùng một con số, chia phần khác đi".
                    //
                    // Sau bù, I mang cả phần hover mà trước đây feed-forward gánh.
                    // Nó có thể vượt vz_ilimit; clamp trong cascade sẽ cắt, và
                    // phần bị cắt là ga BỊ MẤT THẬT. Cảnh báo khi chạm để không
                    // phải đi tìm nguyên nhân "tụt ga ngay sau takeoff" về sau.
                    {
                        const float hover_old = s_hold_tune.hover;
                        const float hover_new = ALT_HOLD_HOVER_NOMINAL;
                        if (hover_old != hover_new) {
                            const float delta = hover_old - hover_new;
                            s_hold_tune.hover = hover_new;
                            s_hold_state.vz_integral += delta;
                            const float ilim = s_hold_tune.vz_ilimit;
                            if (fabsf(s_hold_state.vz_integral) > ilim) {
                                ESP_LOGW(TAG, "HANDOFF: I sau bu (%+.0f) VUOT ilimit %.0f -> bi cat. "
                                              "Ga se tut ~%.0f duty. Nang ALT_HOLD_HOVER_NOMINAL "
                                              "(dang %.0f) gan hover that hon, hoac nang vz_ilimit.",
                                          (double)s_hold_state.vz_integral, (double)ilim,
                                          (double)(fabsf(s_hold_state.vz_integral) - ilim),
                                          (double)hover_new);
                                s_hold_state.vz_integral =
                                    clampf(s_hold_state.vz_integral, -ilim, ilim);
                            }
                            ESP_LOGI(TAG, "HANDOFF: hover_ff %.0f -> %.0f (bo latch, ve hang so), "
                                          "I bu %+.0f -> %+.0f. Tong ga KHONG doi.",
                                      (double)hover_old, (double)hover_new,
                                      (double)delta, (double)s_hold_state.vz_integral);
                        }
                    }
#endif  // FC_FEATURE_HOVER_LATCH

                    ESP_LOGI(TAG, "TAKEOFF XONG -> HOLDING giu %.2fm (Z=%.2f Vz=%.2f, "
                                  "collective=%d, hover_ff=%.0f I=%+.0f)",
                              (double)tr.final_target_m, (double)s_alt_est.alt_m,
                              (double)s_alt_est.vz_ms, tr.throttle_duty,
                              (double)tr.hover_ff, (double)tr.vz_i_term);
                    fsm_transition(&s_fsm, fsm_on_takeoff_handoff(s_fsm.state), now_us);
                }
                break;
            }

            case FSM_HOLDING:
            case FSM_FLYING: {
                alt_hold_result_t hr = (alt_hold_result_t){0};
                const bool tilt_ok = fabsf(roll_deg) <= ALT_HOLD_TILT_GATE_DEG &&
                                      fabsf(pitch_deg) <= ALT_HOLD_TILT_GATE_DEG;

                // ---- B7: ĐẠI LƯỢNG ĐO đưa vào tầng alt phụ thuộc FRAME ----
                // DATUM -> alt_m (độ cao trên sàn cất cánh, hành vi cũ)
                // AGL   -> alt_m - terrain_off_m (độ cao trên bề mặt đang nhìn)
                // Chỉ đổi ĐẦU VÀO ĐO, KHÔNG đổi cascade/target/integral — nên
                // chuyển frame giữa chừng không gây bước nhảy ga.
                const float agl_now_m = alt_estimator_agl_m(&s_alt_est);
                const float alt_meas_m = (s_alt_frame == ALT_FRAME_AGL)
                                          ? agl_now_m : s_alt_est.alt_m;

                // ---- D4: MẤT NGUỒN Z GIỮA LÚC HOLD -> DEGRADE, KHÔNG CHẾT ----
                // Bản trước: mẫu đầu tiên mất validity là alt_hold trả
                // engage_lost -> Commander soft fault -> LANDING ngay. Một cú
                // nhấp nháy ToF 200ms cũng đủ kết thúc chuyến bay.
                // Giờ: giữ điều khiển bằng accel trong ALT_HOLD_TOF_DEGRADE_MS
                // (vz_target = 0, I FREEZE — không sạc I bằng số liệu chết),
                // quá cửa sổ mới nhả cho Commander -> LANDING (nhánh BLIND).
                bool alt_degraded_hold = false;
                if (!s_alt_est.valid) {
                    if (!s_alt_degrade_active) {
                        s_alt_degrade_active = true;
                        s_alt_degrade_since_us = now_us;
                        ESP_LOGW(TAG, "HOLD: mat nguon Z -> degrade accel-hold (vz=0, I freeze) "
                                      "trong toi da %dms roi moi ha canh", ALT_HOLD_TOF_DEGRADE_MS);
                    }
                    alt_degraded_hold =
                        (now_us - s_alt_degrade_since_us) < (int64_t)ALT_HOLD_TOF_DEGRADE_MS * 1000;
                } else {
                    s_alt_degrade_active = false;
                }

                // ---- B6: TRONG lúc terrain đang PENDING ----
                // FREEZE I của vòng Vz (không sạc khi số liệu đang loạn),
                // alt_m COAST bằng dự đoán từ accel (estimator đã không ăn
                // range), P/D VẪN CHẠY trên giá trị coast. KHÔNG tắt hẳn PID:
                // tắt thì drone trôi dọc tự do vài trăm ms — nguy hiểm hơn hẳn
                // so với mất tham chiếu ~100ms.
                const bool terr_freeze = s_alt_est.terr_pending;

                // Chụp I TRƯỚC khi gọi: nếu người dùng đang giữ W/S, hoặc guard
                // khoảng hở bắn, ta phải trả I về đúng giá trị này trước khi
                // chạy lại tầng trong (xem các khối ghi đè bên dưới).
                const float vz_i_before = s_hold_state.vz_integral;

                // ---- FLYING: TẮT PID ĐỘ CAO, GIỮ NGUYÊN MỌI THỨ KHÁC ----
                // Xem s_flying_throttle_latch (đầu file) để biết lý do đầy đủ.
                //
                // Latch được CHỐT ở tick ĐẦU TIÊN vào FLYING, lấy đúng duty mà
                // alt_hold đang xuất — nên chuyển HOLDING->FLYING không có bước
                // nhảy ga nào. Từ đó throttle đứng yên; chỉ W/S mới đổi được.
                //
                // I-term của vòng Vz được GIỮ NGUYÊN (không reset, không chạy):
                // nó chứa lượng ga hover đã học được: khi thả phím nghiêng về
                // HOLDING, alt_hold nhận lại với đúng I đó nên không phải học
                // lại từ đầu. Reset ở đây sẽ gây tụt ga ngay lúc vừa về HOLD.
                const bool flying_no_alt_pid = (s_fsm.state == FSM_FLYING);
                if (flying_no_alt_pid) {
                    if (s_flying_throttle_latch < 0) {
                        // Tick ĐẦU vào FLYING: chốt duty mà alt_hold đang xuất.
                        //
                        // KHÔNG lấy từ throttle_cmd: biến đó được gán 0 ở đầu
                        // bước 9 mỗi tick nên tại đây nó luôn là 0, latch sẽ
                        // rơi thẳng xuống sàn ALT_HOLD_MIN_THROTTLE_DUTY và
                        // drone tụt ngay khi bắt đầu bay ngang.
                        //
                        // s_last_hold_throttle được bước HOLDING ghi lại ở tick
                        // trước (xem cuối nhánh này) — đó mới là ga hover THẬT
                        // mà alt_hold đã học được.
                        s_flying_throttle_latch = clampi(
                            (s_last_hold_throttle > 0) ? s_last_hold_throttle
                                                        : (int)s_hold_tune.hover,
                            ALT_HOLD_MIN_THROTTLE_DUTY, MOTOR_SAFE_MAX_DUTY);
                        ESP_LOGI(TAG, "FLYING: TAT PID do cao, chot ga = %d duty "
                                      "(attitude PID/mixer/estimator/failsafe VAN chay)",
                                  s_flying_throttle_latch);
                    }
                    // ====================================================
                    // PHASE A: TANG TRONG (vz -> throttle) VAN CHAY
                    // ====================================================
                    // Xem tuning.h muc "PHASE A" de biet ly do day du.
                    // Tom tat: tang NGOAI (alt -> vz_target) tat vi range
                    // khong dang tin khi bay qua vat the; tang TRONG khong
                    // doc range nen van dung -> giu vz = 0 thay vi tha troi.
                    //
                    // Latch la FEEDFORWARD nen (hover that da hoc o HOLDING).
                    // Cascade tinh quanh tune->hover, nen phan chenh giua latch
                    // va hover phai duoc NAP vao I -- dung nguyen tac bumpless
                    // cua alt_hold_preload(): I = throttle_muon_co - hover.
#if FLYING_VZ_HOLD_ENABLED
                    if (!s_flying_vz_engaged) {
                        // Tick DAU vao FLYING: nap I sao cho output tick nay
                        // bang DUNG latch (vz_err ~ 0 -> P ~ 0).
                        s_hold_state.vz_integral =
                            clampf((float)s_flying_throttle_latch - s_hold_tune.hover,
                                   -FLYING_VZ_ILIMIT_DUTY, FLYING_VZ_ILIMIT_DUTY);
                        s_flying_vz_engaged = true;
                    }
                    // vz NGUON ACCEL-ONLY: khong dinh ToF, nen cu nhay range
                    // khi bay qua ban khong bom van toc gia vao vong nay.
#if FLYING_VZ_USE_ACCEL_ONLY
                    const float fly_vz_meas = s_alt_est.vz_accel_only_ms;
#else
                    const float fly_vz_meas = s_alt_est.vz_ms;
#endif
                    throttle_cmd = alt_hold_vz_cascade(
                        &s_hold_state, &s_hold_tune,
                        0.0f /* vz_target = GIU YEN DO CAO */,
                        fly_vz_meas, dt,
                        false /* I chay, nhung tran siet lai ben duoi */,
                        FLYING_VZ_ILIMIT_DUTY,
                        ALT_HOLD_MIN_THROTTLE_DUTY, MOTOR_SAFE_MAX_DUTY);
                    // Latch DUOI THEO output: khi tha can ve HOLDING, va khi
                    // guard khoang ho doc latch, ca hai deu thay ga THUC TE
                    // dang dung chu khong phai so chot tu luc vao FLYING.
                    s_flying_throttle_latch = throttle_cmd;
#else
                    throttle_cmd = s_flying_throttle_latch;
#endif
                    // hr phải phản ánh đúng những gì đang xảy ra: alt_hold KHÔNG
                    // lái throttle nữa. hold_driving=false chặn khối W/S bên dưới
                    // đi vào nhánh "hoàn tác I" (không có gì để hoàn tác) — W/S
                    // được xử lý riêng ngay sau đây.
                    hr.throttle_duty = throttle_cmd;
                    hr.hold_driving = false;
                    hr.engage_lost = false;
                    hr.vz_target_ms = 0.0f;
                    alt_target_vz_ms = 0.0f;

                    // ============================================================
                    // W/S TRONG FLYING = OFFSET TAM THOI, KHONG CONG DON
                    // ============================================================
                    // Giu phim -> throttle = ga nen + 100. Nha phim -> ga nen.
                    // Het. Do la toan bo hop dong.
                    //
                    // ⚠ BAN TRUOC CONG DON VAO LATCH va do la mot loi THAT, da
                    // do duoc tren log bay:
                    //     THR=999 -> 999 -> 1799 -> 2000 -> 2000 (MHR=0)
                    //     THRCORR=-1 -> -1 -> 799 -> 1000 -> 1000
                    // GUI gui keepalive moi 100ms khi giu phim (khong the khong
                    // gui: co watchdog BENCH_OFFSET_STALE_US phia firmware). Moi
                    // goi cong them 100 vao latch -> 5 lan la +500 -> kich tran
                    // MOTOR_SAFE_MAX_DUTY va O NGUYEN DO sau khi nha phim, vi
                    // latch la trang thai BEN VUNG. Drone vot len khong phanh.
                    //
                    // Ban chat: keepalive la co che GIU LENH SONG, khong phai
                    // mot lenh MOI. Doc no nhu lenh moi la dem so lan lap lai
                    // cua cung mot y dinh.
                    //
                    // GIO: latch giu nguyen la GA NEN, offset chi cong vao
                    // throttle cua TICK NAY. Nha phim -> s_bench_throttle_offset
                    // ve 0 (GUI gui, va firmware co watchdog stale rieng) ->
                    // throttle tu dong tro lai dung ga nen.
                    // ⚠ THU TU O DAY LA MOT PHAN CUA HOP DONG, KHONG DOI DUOC.
                    // Offset W/S phai cong vao SAU khi s_flying_throttle_latch
                    // da duoc ghi tu output cascade (ngay tren). Neu cong TRUOC
                    // thi tick sau cascade se coi ga-co-offset la ga nen va
                    // cong tiep -> chinh la loi cong don da do duoc tren log:
                    //     THR=999 -> 1799 -> 2000 (giu nguyen sau khi nha phim)
                    // GUI gui keepalive moi 100ms nen moi goi lai cong them mot
                    // lan nua. Keepalive la "GIU lenh song", khong phai lenh MOI.
                    if (s_bench_throttle_offset != 0) {
                        int ws_duty = s_bench_throttle_offset;
                        const float fly_alt_max = commander_clamp_altitude(&s_cmd_cfg, s_cmd_cfg.alt_max_m);
                        if (ws_duty > 0 && s_alt_est.alt_m >= fly_alt_max) ws_duty = 0;
                        if (ws_duty < 0 && s_alt_est.alt_m <= s_cmd_cfg.alt_min_m) ws_duty = 0;
                        // Cong vao throttle cua TICK NAY thoi. s_flying_throttle_latch
                        // (= ga nen do vong Vz giu) KHONG bi dung toi.
                        throttle_cmd = clampi(s_flying_throttle_latch + ws_duty,
                                              ALT_HOLD_MIN_THROTTLE_DUTY,
                                              MOTOR_SAFE_MAX_DUTY);
                        hr.throttle_duty = throttle_cmd;
                        // ⚠ VONG Vz SE CHONG LAI OFFSET NAY -- va do la CO Y.
                        // Giu W -> drone leo -> vz > 0 -> cascade ha I xuong de
                        // keo ve vz=0. Nha phim -> ga nen da thap hon truoc mot
                        // chut -> drone on dinh lai o do cao MOI. Do dung la
                        // hanh vi mong muon: W/S doi DO CAO, khong phai doi
                        // vinh vien mot con so ga.
                        //
                        // FREEZE I trong luc giu phim thi drone se tro lai DUNG
                        // do cao cu khi nha -- tuc W/S khong con tac dung gi.
                        // Nen KHONG freeze.
                    }
                    // ============================================================
                    // NEO TARGET BẰNG SỐ ĐO ToF TƯƠI (yêu cầu người dùng)
                    // ============================================================
                    // Mỗi tick trong FLYING, chốt lại target = độ cao ToF ĐANG
                    // ĐỌC ĐƯỢC. Thả phím nghiêng -> HOLDING nhận đúng con số vừa
                    // chốt và giữ NGAY tại đó; không vút lên vì một target cũ.
                    //
                    // VÌ SAO ToF TƯƠI chứ không phải s_alt_est.alt_m:
                    // trong FLYING drone nghiêng để bay ngang, và alt_m là giá
                    // trị ƯỚC LƯỢNG — khi ToF tạm không fusable nó COAST bằng
                    // tích phân accel và TRÔI. Neo vào một số đang trôi nghĩa là
                    // vừa thả phím đã giữ sai độ cao, rồi khi ToF bắt lại thì
                    // alt_m nhảy về số thật còn target thì không -> drone chạy đi
                    // sửa một sai lệch do chính cái neo tạo ra.
                    //
                    // tof_z_m là số đo ToF đã bù tilt + slew-limit (alt_estimator),
                    // tức là "mặt đất đang thật sự cách bao xa" — đúng thứ cần neo.
                    //
                    // ⚠ CHỈ neo khi ToF ĐANG DÙNG ĐƯỢC. Mất ToF giữa lúc bay
                    // ngang mà vẫn neo thì sẽ chốt vào một giá trị chết; lúc đó
                    // GIỮ NGUYÊN target của tick trước là đúng hơn.
                    if (s_alt_est.tof_fusable && tof_healthy_for_alt) {
                        s_alt_target_m = commander_clamp_altitude(&s_cmd_cfg,
                                                                   s_alt_est.tof_z_m);
                        s_alt_request_m = s_alt_target_m;
                    }
                    // KHÔNG có nhánh else: target giữ nguyên giá trị lần chốt
                    // gần nhất. Đó là hành vi an toàn khi ToF tạm mất.
                    //
                    // ⚠ TUYỆT ĐỐI KHÔNG chạy PID độ cao ở đây. Cả khối này chỉ
                    // GHI một con số để dùng SAU khi đã về HOLDING. throttle_cmd
                    // trong FLYING đến từ s_flying_throttle_latch (+W/S), và
                    // hr.hold_driving đã được đặt false ở trên.
                } else if (s_hold_state.engaged && (alt_degraded_hold || terr_freeze)) {
                    // Mất nguồn Z -> alt_meas_m là rác, KHÔNG cho vào tầng alt:
                    // ép vz_target = 0. Terrain pending -> alt_m vẫn là số coast
                    // dùng được nên tầng alt chạy bình thường.
                    const float vz_t = alt_degraded_hold
                        ? 0.0f
                        : clampf(s_hold_tune.alt_kp * (s_alt_target_m - alt_meas_m),
                                  -ALT_HOLD_VZ_LIMIT_MS, ALT_HOLD_VZ_LIMIT_MS);
                    throttle_cmd = alt_hold_vz_cascade(&s_hold_state, &s_hold_tune,
                                                        vz_t, s_alt_est.vz_ms, dt,
                                                        true /* FREEZE I */,
                                                        s_hold_tune.vz_ilimit,
                                                        ALT_HOLD_MIN_THROTTLE_DUTY,
                                                        MOTOR_SAFE_MAX_DUTY);
                    hr.throttle_duty = throttle_cmd;
                    hr.hold_driving = true;
                    hr.engage_lost = false;
                    hr.vz_target_ms = vz_t;
                    alt_target_vz_ms = vz_t;
                } else {
                    alt_hold_run(&s_hold_state, &s_hold_tune, s_alt_target_m,
                                 s_alt_est.valid, alt_meas_m, s_alt_est.vz_ms,
                                 tilt_ok, 0, dt, MOTOR_SAFE_MAX_DUTY, &hr);
                    throttle_cmd = hr.throttle_duty;
                    alt_target_vz_ms = hr.vz_target_ms;
                }
                hold_integral_freeze = false;   // ngoài takeoff, prime_done luôn true

                // ---- W/S = CỘNG THẲNG ±offset DUTY vào throttle ----
                // (THEO YÊU CẦU NGƯỜI DÙNG — trước đây chỗ này dịch offset
                //  thành lệnh vận tốc ALT_HOLD_WS_VZ_MS; xem tuning.h mục
                //  "W/S" để biết đánh đổi.)
                //
                // Ý nghĩa giờ ĐỒNG NHẤT ở mọi state: giá trị trên dây LÀ duty.
                // BENCH_RAMP đã luôn hiểu vậy (bước 9 case FSM_BENCH_RAMP), giờ
                // HOLDING/FLYING cũng vậy — một con số, một nghĩa.
                //
                // ⚠ ĐÁNH ĐỔI ĐÃ BIẾT VÀ CHẤP NHẬN:
                //   - "+100 duty" KHÔNG có đơn vị vật lý: cùng phím cho tốc độ
                //     leo khác nhau tuỳ pin (hover ~900 duty @4.2V so với ~1350
                //     @3.6V). Pin cạn thì cùng +100 sẽ leo chậm hơn rõ rệt.
                //   - Người lái phải TỰ điều tiết bằng mắt, không còn được vòng
                //     Vz bù giúp.
                //
                // Chỉ can thiệp khi alt_hold ĐANG thật sự lái throttle
                // (hold_driving). Mất estimator/nghiêng quá thì alt_hold đã trả
                // manual throttle và Commander đang xử lý soft fault — chồng
                // thêm lệnh người dùng vào lúc đó là làm nhiễu một quy trình an
                // toàn đang chạy.
                if (s_bench_throttle_offset != 0 && hr.hold_driving) {
                    // (1) ĐÓNG BĂNG I — BẮT BUỘC, không phải tuỳ chọn.
                    //     alt_hold giữ độ cao bằng I-term. Cộng +100 duty vào
                    //     output mà vẫn để I chạy thì I nhìn thấy drone đang
                    //     leo "sai" so với target và tự TRỪ dần đúng bằng +100
                    //     — sau vài giây phím "hết ăn" hoàn toàn dù vẫn đang
                    //     giữ. Khôi phục I về giá trị TRƯỚC khi alt_hold_run()
                    //     cộng ở tick này là cách đóng băng chính xác nhất.
                    s_hold_state.vz_integral = vz_i_before;

                    int ws_duty = s_bench_throttle_offset;

                    // (2) GEOFENCE — chặn LỆNH theo chiều đang vi phạm.
                    //     Với ±duty thì trần/sàn không tự chặn được như lệnh
                    //     vận tốc (duty đi thẳng ra motor, không qua tầng Z),
                    //     nên phải chặn TƯỜNG MINH ở đây. Chiều ngược lại vẫn
                    //     cho đi — luôn phải thoát ra được khỏi biên.
                    const float tof_alt_max = commander_clamp_altitude(&s_cmd_cfg, s_cmd_cfg.alt_max_m);
                    if (ws_duty > 0 && s_alt_est.alt_m >= tof_alt_max) ws_duty = 0;
                    if (ws_duty < 0 && s_alt_est.alt_m <= s_cmd_cfg.alt_min_m) ws_duty = 0;

                    // (3) CỘNG THẲNG vào duty mà alt_hold vừa tính. Sàn
                    //     ALT_HOLD_MIN_THROTTLE_DUTY giữ nguyên: giữ S không
                    //     được phép cắt motor về 0 khi đang bay.
                    throttle_cmd = clampi(throttle_cmd + ws_duty,
                                           ALT_HOLD_MIN_THROTTLE_DUTY, MOTOR_SAFE_MAX_DUTY);

                    // (4) NEO target theo độ cao HIỆN TẠI mỗi tick. Nhả phím là
                    //     giữ ngay tại chỗ đang ở, không giật về độ cao cũ.
                    s_alt_target_m = commander_clamp_altitude(&s_cmd_cfg, s_alt_est.alt_m);
                    s_alt_request_m = s_alt_target_m;
                }
#if FC_FEATURE_TERRAIN_OFFSET
                // ---- B8: GUARD KHOẢNG HỞ TỐI THIỂU — LƯỚI AN TOÀN CUỐI ----
                // Chạy SAU mọi đường tính throttle ở trên (kể cả W/S) vì nó
                // phải THẮNG tất cả: dù logic offset có sai, dù người lái đang
                // giữ S, drone vẫn không được cắm xuống mặt bàn.
                //
                // Hai việc, và thứ tự KHÔNG đổi được:
                //   1. ÉP LEO: vz_target = max(lệnh hiện tại, TERR_ESCAPE_VZ)
                //      — bất kể frame nào đang chọn.
                //   2. REBASE terrain_off_m theo range thật -> AGL khớp lại với
                //      cái đang thật sự ở dưới, thay vì để controller đánh nhau
                //      với một model terrain đã sai. Chỉ rebase ở CẠNH LÊN của
                //      guard: rebase mỗi tick sẽ làm terr_commit_count chạy
                //      loạn và landing tưởng có bậc địa hình mới liên tục.
                // hold_driving HOẶC flying_no_alt_pid: ở FLYING ta cố ý đặt
                // hold_driving=false (alt_hold không lái throttle nữa), nhưng
                // guard khoảng hở PHẢI VẪN CHẠY. Nó là lưới an toàn chống cắm
                // xuống đất — thứ cần nhất ĐÚNG LÚC đang bay ngang về phía một
                // vật cản, chứ không phải lúc treo yên một chỗ. Chỉ kiểm
                // hold_driving thôi là vô hiệu hoá guard trong toàn bộ FLYING.
                const bool clearance_low = (hr.hold_driving || flying_no_alt_pid) &&
                                            s_alt_est.tof_fusable &&
                                            agl_now_m < TERR_MIN_CLEARANCE_M;
                if (clearance_low) {
                    // ⚠ HOAN TAC I -- CHI o nhanh HOLDING, KHONG o FLYING.
                    // vz_i_before duoc chup TRUOC khoi tinh throttle. O HOLDING
                    // no dung: alt_hold_run() da tich phan mot lan, guard sap
                    // tich phan lan nua tren CUNG mot tick -> phai tra ve moc
                    // cu de khong dem hai lan.
                    //
                    // O FLYING thi NGUOC LAI: tu Phase A, vong Vz chay ngay
                    // trong nhanh FLYING va vz_integral luc nay CHINH LA trang
                    // thai da hoc cua no (nap bumpless o tick dau, roi tu chinh
                    // dan). Ghi de bang vz_i_before se vut bo dung cai do va ep
                    // vong Vz hoc lai tu dau MOI TICK guard con kich hoat --
                    // tuc la lam te di dung luc dang sap va cham.
                    if (!flying_no_alt_pid) {
                        s_hold_state.vz_integral = vz_i_before;
                    }
                    const float esc_vz = fmaxf(alt_target_vz_ms, TERR_ESCAPE_VZ_MS);
                    // Nguon vz phai KHOP voi nhanh dang chay, neu khong thi
                    // guard va vong Vz cua FLYING dieu khien tren hai so do
                    // khac nhau va se danh nhau.
#if FLYING_VZ_HOLD_ENABLED && FLYING_VZ_USE_ACCEL_ONLY
                    const float esc_vz_meas = flying_no_alt_pid
                        ? s_alt_est.vz_accel_only_ms : s_alt_est.vz_ms;
#else
                    const float esc_vz_meas = s_alt_est.vz_ms;
#endif
                    throttle_cmd = alt_hold_vz_cascade(
                        &s_hold_state, &s_hold_tune, esc_vz, esc_vz_meas, dt,
                        false,
                        flying_no_alt_pid ? FLYING_VZ_ILIMIT_DUTY : s_hold_tune.vz_ilimit,
                        ALT_HOLD_MIN_THROTTLE_DUTY, MOTOR_SAFE_MAX_DUTY);
                    alt_target_vz_ms = esc_vz;
                    // Đang FLYING: guard vừa tính một duty CAO HƠN để thoát lên.
                    // Phải đẩy vào LATCH, không chỉ vào throttle_cmd của tick
                    // này — nếu không thì tick sau latch cũ (thấp) ghi đè lại và
                    // guard chỉ có tác dụng đúng một tick, drone vẫn cắm xuống.
                    if (flying_no_alt_pid) {
                        // ⚠ TRAN TOC DO TANG GA — day chinh la cho da gay loi.
                        // Bay o ~23cm (duoi TERR_MIN_CLEARANCE_M=25cm) thi guard
                        // kich hoat MOI TICK, va moi tick deu ghi de latch:
                        //     THR=995 -> 995 -> 1095 -> 1995 -> 2000 (kich tran)
                        // +900 duty trong MOT tick 5ms. Guard duoc phep ep leo,
                        // nhung khong duoc phep nhay bac nhu vay.
                        //
                        // Chi chan chieu TANG. Guard ha ga (vd vua thoat xong)
                        // van duoc ve ngay: chan chieu giam la tao che do hong moi.
                        // ⚠ TICH LUY PHAN LE, KHONG duoc (int) thang.
                        // dt = 4ms -> 150 * 0.004 = 0.6 -> (int) = 0 -> rise_cap
                        // = latch -> ga KHONG BAO GIO tang duoc, tuc la vo hieu
                        // hoa hoan toan guard chong va cham. Mot tran toc do lai
                        // bien thanh mot cai khoa cung, va no im lang.
                        s_flying_rise_credit += THROTTLE_MAX_RISE_DUTY_PER_S * dt;
                        const int rise_cap = s_flying_throttle_latch +
                            (int)s_flying_rise_credit;
                        int guard_duty = clampi(throttle_cmd,
                                                ALT_HOLD_MIN_THROTTLE_DUTY,
                                                MOTOR_SAFE_MAX_DUTY);
                        if (guard_duty > rise_cap) guard_duty = rise_cap;
                        const int prev_latch = s_flying_throttle_latch;
                        s_flying_throttle_latch = clampi(guard_duty,
                                                          ALT_HOLD_MIN_THROTTLE_DUTY,
                                                          MOTOR_SAFE_MAX_DUTY);
                        // Tru phan credit DA TIEU. Khong tru thi credit cu tich
                        // mai va cu nay sau lai cho phep mot buoc nhay lon.
                        const int used = s_flying_throttle_latch - prev_latch;
                        if (used > 0) s_flying_rise_credit -= (float)used;
                        if (s_flying_rise_credit < 0.0f) s_flying_rise_credit = 0.0f;
                        throttle_cmd = s_flying_throttle_latch;
                    }
                    if (!s_terr_guard_active) {
                        s_terr_guard_active = true;
                        const bool rebased = alt_estimator_terrain_rebase(&s_alt_est);
                        ESP_LOGW(TAG, "KHOANG HO %.2fm < %.2fm -> EP LEO %.2f m/s%s "
                                      "(be mat duoi bung cao hon model terrain)",
                                  (double)agl_now_m, (double)TERR_MIN_CLEARANCE_M,
                                  (double)esc_vz,
                                  rebased ? " + rebase terrain" : " (khong rebase duoc: ToF khong dung duoc)");
                    }
                } else {
                    s_terr_guard_active = false;
                }
#endif  // FC_FEATURE_TERRAIN_OFFSET

                // KHÔNG tự chuyển FSM ở đây nữa. Commander là nơi DUY NHẤT
                // quyết định soft fault (đúng như docstring commander.h vẫn
                // luôn nói) — trước đây alt_hold có đường tắt riêng, tạo ra
                // hai nguồn quyết định cho cùng một sự kiện. Giờ chỉ báo cáo,
                // Commander đọc ở tick sau (xem s_alt_hold_engage_lost_prev).
                s_alt_hold_engage_lost_prev = hr.engage_lost;

                // Ghi lại ga của tick HOLDING này để lần vào FLYING kế tiếp có
                // cái mà chốt (xem s_last_hold_throttle). CHỈ ghi khi đang thật
                // sự ở HOLDING: ghi cả lúc FLYING thì latch sẽ tự "đuổi theo"
                // chính nó và mất hẳn ý nghĩa đóng băng.
                if (s_fsm.state == FSM_HOLDING) {
                    s_last_hold_throttle = throttle_cmd;
                }
                break;
            }

            case FSM_LANDING: {
                landing_result_t lr;
                const bool was_engaged = s_hold_state.engaged;
                // ĐỘ CAO TRÊN BỀ MẶT HẠ CÁNH, không phải world-Z.
                // Hạ xuống một cái bàn cao 0.4m: world-Z về 0.4 là đã CHẠM, còn
                // world-Z=0 là đã đâm xuyên qua bàn. flare/touchdown phải tính
                // theo bề mặt thật ở dưới. Nếu không chọn được bề mặt nào (ToF
                // hỏng) hàm trả về alt_m -> hành vi y hệt bản cũ.
                const float land_height_m =
                    alt_estimator_height_above_landing_surface(&s_alt_est);
                // BẰNG CHỨNG CHẠM ĐẤT cũng phải là AGL: tof_z_m là độ cao trên
                // SÀN CẤT CÁNH, dùng nó khi đang hạ xuống mặt bàn thì ngưỡng
                // touchdown 3.5cm không bao giờ đạt được (nó còn cách sàn 0.75m
                // dù đã nằm trên bàn) -> motor quay mãi.
                const float land_tof_agl_m = alt_estimator_tof_agl_m(&s_alt_est);
                landing_run(&s_land_state, &s_land_tune, &s_hold_state, &s_hold_tune,
                            was_engaged, s_alt_est.valid, land_height_m, s_alt_est.vz_ms,
                            0, s_alt_est.tof_fusable, land_tof_agl_m,
                            s_alt_est.tof_vz_lpf_ms,
                            s_alt_est.terr_pending, s_alt_est.terr_commit_count,
                            s_alt_est.az_after_bias_ms2,
                            dt, now_us, MOTOR_SAFE_MAX_DUTY, &lr);
                throttle_cmd = lr.throttle_duty;
                hold_integral_freeze = false;
                if (lr.touchdown_done) {
                    motor_driver_disarm();
                    reset_all_controllers();
                    alt_estimator_unlock_floor(&s_alt_est);
                    fsm_transition(&s_fsm, fsm_on_landing_touchdown(s_fsm.state), now_us);
                }
                break;
            }

            case FSM_EMERGENCY:
                throttle_cmd = 0;   // đã xử lý ở bước (7), không tới đây trong cùng tick
                break;
        }

        // ---- 9b) (ĐÃ BỎ) bù throttle theo điện áp pin ----
        // TRƯỚC ĐÂY: throttle_cmd *= clamp(NOMINAL_V / battery_v, 1.0, MAX_GAIN).
        // GIỜ: KHÔNG nhân gì cả. throttle_cmd đi thẳng từ bước (9) vào mixer.
        //
        // VÌ SAO BỎ (yêu cầu người dùng, và nó đúng về mặt điều khiển):
        // VBAT đo được KHÔNG phải chỉ là "mức pin còn lại" — nó tụt theo TẢI
        // TỨC THÌ. Ngay khi UAV nhấc lên, 4 motor rút dòng lớn, sụt áp trên nội
        // trở pin + dây làm VBAT đo được giảm mạnh dù pin còn đầy. Nhân throttle
        // theo con số đó tạo ra một VÒNG PHẢN HỒI DƯƠNG ký sinh, nằm NGOÀI mọi
        // vòng PID đã tune:
        //     ga lên -> dòng tăng -> VBAT đo giảm -> comp tăng -> ga lên nữa...
        // Nó cũng phá alt_hold: alt_hold xuất ra một duty đã tính toán, rồi bị
        // một hệ số lạ nhân vào sau lưng nên duty THỰC không còn là duty PID
        // yêu cầu — mọi gain tune ở bench đều sai khi bay thật. Bằng chứng đo
        // được từ log bench: BATV dao động 3.51..3.82V trong VÀI GIÂY ở tải gần
        // như không đổi -> comp nhảy 1.10..1.20, tức nhiễu áp được KHUẾCH ĐẠI
        // thẳng vào throttle.
        //
        // battery_v VẪN được dùng cho FAILSAFE (Commander: pin dưới sàn ->
        // LANDING, xem commander.c) và prearm_check() — đó là dùng ĐÚNG: so
        // ngưỡng, không nhân vào đường điều khiển.
        //
        // base_throttle_duty giữ lại (giờ == throttle_cmd) vì telemetry BTHR=
        // và GUI vẫn đọc; battery_comp giữ hằng 1.0 cho field BCOMP= để dòng
        // STATUS không đổi format (GUI cũ khỏi vỡ regex, xem telemetry_format.c).
        // ====================================================================
        // PHASE E3: BU cos(tilt) -- FEEDFORWARD, chay TRUOC khi chot base
        // ====================================================================
        // Xem tuning.h muc "PHASE E3" de biet ly do va bang so.
        //
        // rzz = phan tu (3,3) cua ma tran xoay = cos cua goc giua truc Z than
        // va truc Z the gioi -- chinh la cos(tilt) tong hop cua ca roll lan
        // pitch. Lay TRUC TIEP tu quaternion, khong phai cosf(roll)*cosf(pitch)
        // (cong thuc do chi dung khi mot trong hai goc bang 0, va no goi 2 ham
        // luong giac trong vong dieu khien 250Hz).
        //
        // alt_estimator da tinh dung bieu thuc nay moi tick cho ToF -- dung lai
        // de khong tinh hai lan va khong the lech nhau.
        //
        // ⚠ CHI BU KHI DANG THUC SU LAI DONG CO. throttle_cmd == 0 nghia la
        // disarmed / landing cutoff / abort -- nhan 1.02 vao 0 van la 0 nhung
        // de ro dieu kien de khong ai vo tinh lam no "hoi sinh" ga bang mot
        // thay doi sau nay.
#if TILT_COMP_ENABLED
        if (throttle_cmd > 0) {
            const float tilt_cos = s_alt_est.tilt_cos;
            if (isfinite(tilt_cos) && tilt_cos > TILT_COMP_MIN_COS && tilt_cos < 1.0f) {
                const float f = clampf(1.0f / tilt_cos, 1.0f, TILT_COMP_MAX_FACTOR);
                throttle_cmd = clampi((int)((float)throttle_cmd * f),
                                      0, MOTOR_SAFE_MAX_DUTY);
            }
        }
#endif

        const int base_throttle_duty = throttle_cmd;

        // battery_comp giữ hằng 1.0: bù throttle theo pin đã BỎ (xem khối trên).
        // Field BCOMP= trong STATUS giữ nguyên format cho GUI cũ.
        //
        // ĐÃ THỬ một cơ chế "hệ số NHÂN throttle chốt lúc ARM" rồi GỠ theo yêu
        // cầu người dùng — nó bù CHỒNG với hover latch (latch đã đặt base đúng
        // mức pin, nhân thêm tỷ lệ đó lần nữa là bù hai lần). Cách bù pin đang
        // dùng là HOVER LATCH ở CMD_ARM.
        const float battery_comp = 1.0f;

        // ---- 9c) GATE GA cho I-term — điều kiện ĐỘC LẬP, AND thêm vào gate
        // theo FSM ở bước (9). Ga thấp thì motor CHƯA đủ thẩm quyền tạo mô-men
        // sửa sai số, cộng dồn lúc này chỉ sinh windup rồi bung ra khi ga lên.
        //
        // FREEZE (giữ nguyên integral), KHÔNG reset — khác hẳn nhánh
        // ATT_MIN_THROTTLE_DUTY ở bước (10) vốn reset SẠCH cả 6 bộ vì lúc đó
        // PID không chạy chút nào. Ở đây PID vẫn chạy (P/D vẫn ổn định drone),
        // chỉ riêng I ngừng tích lũy — thả ga xuống rồi kéo lên lại không mất
        // trim đã học. Xem tuning.h ATT_I_ENABLE_THROTTLE_DUTY.
        if (throttle_cmd < ATT_I_ENABLE_THROTTLE_DUTY) {
            hold_integral_freeze = true;
        }

        // ---- 9d) GATE MẪU MỚI cho I-term — tick không có mẫu IMU mới thì
        // KHÔNG tích phân sai số. P/D vẫn chạy (chúng là hàm THUẦN của trạng
        // thái hiện tại, chạy lại trên cùng số liệu chỉ cho ra cùng một lệnh —
        // vô hại, và giữ được mô-men ổn định trong lúc chờ mẫu). I thì KHÁC:
        // nó CỘNG DỒN, nên mỗi tick lặp lại là một lần nạp thêm cùng một sai
        // số vào integrator. Mất mẫu 5 tick = I bị nạp gấp 5 lần thực tế, rồi
        // bung ra thành một cú giật mô-men đúng lúc mẫu quay lại.
        //
        // FREEZE chứ không reset — cùng lý do với gate ga ở 9c: trim đã học
        // được phải giữ nguyên qua một khoảng mất mẫu ngắn.
        //
        // Cửa sổ này ngắn theo thiết kế: quá SENSOR_IMU_STALE_US (20ms = 5
        // tick) thì Commander ở bước 6 trip HARD fault "imu stale" và tick sau
        // không còn chạy tới đây nữa.
        if (!imu_updated) {
            hold_integral_freeze = true;
        }

        // ---- 10) attitude cascade + mixer ----
        //
        // CHÍNH SÁCH (đã tách "đang bay" khỏi "mức ga"):
        //   airborne (TAKING_OFF sau liftoff / HOLDING / FLYING / LANDING):
        //       P/D LUÔN CHẠY, bất kể ga thấp đến đâu. Đây là chỗ sửa lỗi
        //       nguy hiểm nhất của bản cũ: LANDING hạ ga xuống dưới 200 thì
        //       PID bị TẮT HẲN và integrator bị RESET — trong khi drone VẪN
        //       ĐANG TRÊN KHÔNG. Không còn gì giữ thăng bằng ở đúng pha dễ
        //       lật nhất.
        //   chưa airborne (DISARMED/ARMED/TAKING_OFF trước liftoff):
        //       ga dưới ATT_MIN_THROTTLE_DUTY -> 4 motor quay đều, không PID.
        //       Ở đất thì không có gì để ổn định, và cho PID chạy khi drone
        //       đang tì lên mặt đất chỉ tạo windup chống lại phản lực nền.
        //
        // Reset integrator CHỈ khi thật sự không bay (mục dưới) — KHÔNG reset
        // vì ga tạm xuống thấp. Ga tụt rồi lên lại giữa lúc bay mà mất sạch
        // trim đã học là một bước nhảy mô-men ngay khi PID chạy lại.
        const bool airborne_now = airborne_for_alt;   // xem bước 2, dùng chung 1 định nghĩa
        const bool run_attitude = airborne_now || (throttle_cmd >= ATT_MIN_THROTTLE_DUTY);

        attitude_output_t att_out = {0};
        if (!run_attitude) {
            attitude_state_reset(&s_att_state);
            s_mixer_sat_prev = (mixer_status_t){0};
            motor_driver_set_duties(throttle_cmd, throttle_cmd, throttle_cmd, throttle_cmd);
            // Ghi LẠI vào att_out để telemetry (bước 11) báo ĐÚNG duty đã gửi
            // ra motor. Trước đây nhánh này để att_out={0} nên field M= báo
            // "0 0 0 0" trong khi 4 motor ĐANG QUAY ở throttle_cmd — nhìn log
            // bench (THR=112, M=0 0 0 0) sẽ tưởng mixer chết/PID không chạy.
            // Đây là telemetry NÓI DỐI về trạng thái motor, nguy hiểm hơn hẳn
            // một field hiển thị sai bình thường.
            att_out.m1 = att_out.m2 = att_out.m3 = att_out.m4 = throttle_cmd;
        } else {
            attitude_input_t ain = {0};
            ain.roll_deg = roll_deg;
            ain.pitch_deg = pitch_deg;
            ain.yaw_deg = yaw_deg;
            ain.gyro_roll_dps = imu.gyro_dps.x;
            ain.gyro_pitch_dps = imu.gyro_dps.y;
            ain.gyro_yaw_dps = imu.gyro_dps.z;
            // Timed-command (CMD_MOVE/CMD_SET_YAW, xung có thời hạn — vẫn dùng
            // bởi console/MicroPython) THẮNG setpoint bay tay UDP persistent
            // (CMD_SET_ATTITUDE) nếu đang active; ngược lại dùng SP + trim
            // (xem CMD_SET_TRIM — trim CỘNG vào SP, KHÔNG sửa sensor raw).
            //
            // TRIM CỘNG VÀO CẢ HAI NHÁNH — sửa lỗi: bản trước chỉ cộng trim ở
            // nhánh SP, nên trong suốt một lệnh `move forward 30 2` trim BIẾN
            // MẤT và drone dạt đúng theo cái lệch mà trim sinh ra để bù. Trim
            // KHÔNG phải setpoint, nó là hằng số bù lệch cơ khí/CG của khung —
            // "lệnh nào thắng" là câu hỏi về SETPOINT, không áp dụng cho trim.
            const float base_roll  = s_timed_active ? s_timed_roll_deg  : s_sp_roll_deg;
            const float base_pitch = s_timed_active ? s_timed_pitch_deg : s_sp_pitch_deg;
            ain.target_roll_deg = clampf(base_roll + s_trim_roll_deg,
                                          -SP_TILT_MAX_DEG, SP_TILT_MAX_DEG);
            ain.target_pitch_deg = clampf(base_pitch + s_trim_pitch_deg,
                                           -SP_TILT_MAX_DEG, SP_TILT_MAX_DEG);
            ain.target_yaw_rate_dps = s_timed_active ? s_timed_yaw_rate_dps : s_sp_yaw_rate_dps;
            ain.target_yaw_deg = yaw_deg;   // heading-hold kp=0 mac dinh -> khong anh huong
            ain.throttle_duty = throttle_cmd;
            ain.dt_s = dt;

            attitude_control_update(&s_att_state, &s_att_gains, &ain,
                                     hold_integral_freeze, MOTOR_SAFE_MAX_DUTY,
                                     &s_mixer_sat_prev, &att_out);
            motor_driver_set_duties(att_out.m1, att_out.m2, att_out.m3, att_out.m4);

            // Lưu cho anti-windup của tick SAU (conditional integration —
            // xem attitude_control.h mixer_status_t).
            s_mixer_sat_prev = att_out.sat;
        }

        // Commander đọc ở tick sau (bước 6). "Bão hoà" ở đây là cờ do mixer tự
        // báo sau desaturation, KHÔNG phải "duty == safe_max" như TODO cũ —
        // mixer giảm đều correction chứ không clip từng motor, nên so duty với
        // trần sẽ bỏ sót đúng trường hợp cần bắt.
        s_motor_saturated_prev = att_out.sat.saturated;

        // ---- 11) publish telemetry đầy đủ ----
        xSemaphoreTake(s_telemetry_mtx, portMAX_DELAY);
        s_telemetry.state = s_fsm.state;
        s_telemetry.armed = (s_fsm.state != FSM_DISARMED);
        s_telemetry.terrain_off_m      = s_alt_est.terrain_off_m;
        s_telemetry.terr_pending       = s_alt_est.terr_pending;
        s_telemetry.terr_commit_count  = s_alt_est.terr_commit_count;
        s_telemetry.terr_reject_count  = s_alt_est.terr_reject_count;
        s_telemetry.terr_timeout_count = s_alt_est.terr_timeout_count;
        s_telemetry.terr_residual_m    = s_alt_est.terr_residual_m;
        s_telemetry.terrain_pending    = s_alt_est.terr_pending;
        s_telemetry.terrain_commits    = s_alt_est.terr_commit_count;
        s_telemetry.terrain_residual_m = s_alt_est.terr_residual_m;
        s_telemetry.clearance_m        = alt_estimator_agl_m(&s_alt_est);
        s_telemetry.alt_frame          = (int32_t)s_alt_frame;
        s_telemetry.alt_target_m = s_alt_target_m;
        s_telemetry.alt_request_m = s_alt_request_m;
        s_telemetry.z_error_m = s_alt_target_m - s_alt_est.alt_m;
        s_telemetry.vz_error_ms = s_hold_state.last_vz_error;
        s_telemetry.vz_p_term = s_hold_state.last_p_term;
        s_telemetry.vz_i_term = s_hold_state.last_i_term;
        s_telemetry.vz_d_term = s_hold_state.last_d_term;
        s_telemetry.vz_output_duty = s_hold_state.last_output_duty;
        s_telemetry.hover_throttle_duty = s_hold_tune.hover;
        s_telemetry.alt_target_vz_ms = alt_target_vz_ms;
        s_telemetry.throttle_duty = throttle_cmd;
        s_telemetry.throttle_correction_duty = (float)throttle_cmd - s_hold_tune.hover;
        s_telemetry.battery_comp = battery_comp;
        s_telemetry.m1 = att_out.m1; s_telemetry.m2 = att_out.m2;
        s_telemetry.m3 = att_out.m3; s_telemetry.m4 = att_out.m4;
        s_telemetry.last_fault = s_last_fault_class;
        s_telemetry.motor_kill_latched = s_motor_kill_latched;
        s_telemetry.arm_reject = (int32_t)s_arm_reject;
        s_telemetry.arm_reject_seq = s_arm_reject_seq;
        s_telemetry.takeoff_reject = (int32_t)s_tko_reject;
        s_telemetry.takeoff_reject_seq = s_tko_reject_seq;
        s_telemetry.takeoff_prime_done = takeoff_prime_done;
        s_telemetry.takeoff_phase = (int32_t)takeoff_phase;
        // Toàn bộ khối takeoff lấy TỪ s_tko_result (bản chụp kết quả tick này,
        // đã xoá về 0 ở đầu bước 9 nếu không ở TAKING_OFF).
        s_telemetry.takeoff_control_active = s_tko_result.control_active;
        s_telemetry.takeoff_target_alt_m = s_tko_result.final_target_m;
        s_telemetry.takeoff_z_sp_m = s_tko_result.target_z_m;
        s_telemetry.takeoff_vz_target_ms = s_tko_result.vz_target_ms;
        s_telemetry.takeoff_base_thrust = s_tko_result.hover_ff;
        s_telemetry.takeoff_alt_corr = s_tko_result.alt_thrust_corr;
        s_telemetry.takeoff_vz_i_term = s_tko_result.vz_i_term;
        s_telemetry.takeoff_ground_alt_m = s_tko_result.ground_alt_m;
        s_telemetry.takeoff_liftoff_flag = s_tko_result.liftoff_flag;
        s_telemetry.takeoff_tilt_deg = s_tko_result.tilt_deg;
        s_telemetry.takeoff_elapsed_s = s_tko_result.elapsed_s;
        s_telemetry.takeoff_abort_reason = (int32_t)s_tko_result.abort_reason;
        s_telemetry.alt_pid_saturated = s_hold_state.vz_saturated;
        s_telemetry.mixer_saturated = att_out.sat.saturated;
        s_telemetry.mixer_roll_limited = att_out.sat.roll_limited_pos || att_out.sat.roll_limited_neg;
        s_telemetry.mixer_pitch_limited = att_out.sat.pitch_limited_pos || att_out.sat.pitch_limited_neg;
        s_telemetry.mixer_yaw_limited = att_out.sat.yaw_limited_pos || att_out.sat.yaw_limited_neg;
        s_telemetry.mixer_headroom_duty = att_out.sat.headroom_duty;
        s_telemetry.base_throttle_duty = base_throttle_duty;
        // Publish SAU bước 9c -> đã gồm CẢ gate FSM lẫn gate ga (xem telemetry.h).
        // AND thêm ATT_MIN_THROTTLE_DUTY vì dưới ngưỡng đó bước (10) bỏ qua PID
        // HOÀN TOÀN (và reset sạch integrator) — nếu ai đó tune
        // ATT_I_ENABLE_THROTTLE_DUTY < ATT_MIN_THROTTLE_DUTY thì không có
        // dòng này KI= sẽ báo "đang chạy" trong khi PID còn không được gọi.
        s_telemetry.att_integral_active =
            !hold_integral_freeze && (throttle_cmd >= ATT_MIN_THROTTLE_DUTY);
        s_telemetry.landing_phase = (int)s_land_state.phase;
        s_telemetry.uncalibrated = s_uncalibrated;
        s_telemetry.calib_gyro_active = s_calib_gyro_active;
        s_telemetry.calib_accel_capturing = s_calib_accel_capturing;
        s_telemetry.calib_accel_faces_done = s_calib_accel_faces_done;
        s_telemetry.calib_mag_active = s_calib_mag_active;
        s_telemetry.calib_mag_sample_count = s_calib_mag_sample_count;
        s_telemetry.calib_gyro_valid = s_calib.gyro_valid;
        s_telemetry.calib_accel_valid = s_calib.accel_valid;
        s_telemetry.calib_mag_valid = s_calib.mag_valid;

        // ---- Gyro bias diagnostics (xem telemetry.h, tuning.h muc 8b) ----
        // Quan he BAT BUOC giua ba dong nay: corr == raw - bias. Publish ca ba
        // de nguoi doc TU KIEM CHUNG duoc, khong phai tin loi firmware.
        // Lay tu imu.* cua CHINH tick nay (cung mau Mahony/PID vua dung).
        s_telemetry.gyro_raw_dps  = imu.gyro_raw_dps;
        s_telemetry.gyro_bias_dps = s_imu_calib.gyro_bias_dps;
        s_telemetry.gyro_corr_dps = imu.gyro_corrected_sensor_dps;
        s_telemetry.gyro_std_dps  = s_gcal_last_corr_std;
        s_telemetry.gyro_raw_mean_dps = s_gcal_last_raw_mean;
        s_telemetry.gyro_corr_mean_dps = s_gcal_last_corr_mean;
        s_telemetry.gyro_raw_std_dps = s_gcal_last_raw_std;
        s_telemetry.gyro_corr_std_dps = s_gcal_last_corr_std;
        s_telemetry.accel_raw_g   = imu.accel_raw_g;
        s_telemetry.accel_corr_g  = imu.accel_g;   // sau buoc 1c (bias/scale accel)
        s_telemetry.accel_norm_g  = sqrtf(imu.accel_g.x * imu.accel_g.x +
                                           imu.accel_g.y * imu.accel_g.y +
                                           imu.accel_g.z * imu.accel_g.z);

        s_telemetry.gyro_cal_bad_samples = s_gcal_bad_samples;
        s_telemetry.gyro_cal_state   = (int)s_gcal_state;
        s_telemetry.gyro_cal_fail    = (int)s_gcal_fail;
        s_telemetry.gyro_cal_temp_c  = s_gcal_temp_c;
        // Canh bao lech nhiet do (muc 9) — CHI canh bao, KHONG chan ARM va
        // KHONG tu sua bias. Chi co nghia khi da calib xong.
        s_telemetry.gyro_cal_temp_warn = s_calib.gyro_valid &&
            (fabsf(s_last_imu_temp_c - s_gcal_temp_c) > CALIB_TEMP_WARN_DELTA_C);
        s_telemetry.gyro_valid_from_nvs = s_calib.gyro_valid_from_nvs;

        {
            imu_cfg_readback_t cfg;
            imu_driver_get_config(&cfg);
            s_telemetry.imu_cfg_valid          = cfg.valid;
            s_telemetry.imu_gyro_config        = cfg.gyro_config;
            s_telemetry.imu_accel_config       = cfg.accel_config;
            s_telemetry.imu_fs_sel             = cfg.fs_sel;
            s_telemetry.imu_afs_sel            = cfg.afs_sel;
            s_telemetry.imu_gyro_lsb_per_dps   = cfg.gyro_lsb_per_dps;
            s_telemetry.imu_accel_lsb_per_g    = cfg.accel_lsb_per_g;
        }
        s_telemetry.calib_mag_seconds_left = s_calib_mag_active ? (s_calib_mag_ticks_left / CONTROL_TASK_HZ) : 0;
        // range = 0 khi chưa có mẫu nào (extrema chưa init) — KHÔNG đọc
        // s_calib_mag_min/max lúc đó vì chúng chưa được gán giá trị nào.
        s_telemetry.calib_mag_range_x = s_calib_mag_extrema_init ? (s_calib_mag_max.x - s_calib_mag_min.x) : 0.0f;
        s_telemetry.calib_mag_range_y = s_calib_mag_extrema_init ? (s_calib_mag_max.y - s_calib_mag_min.y) : 0.0f;
        s_telemetry.calib_mag_range_z = s_calib_mag_extrema_init ? (s_calib_mag_max.z - s_calib_mag_min.z) : 0.0f;
        s_telemetry.calib_mag_read_err_count = s_calib_mag_read_err_count;
        s_telemetry.calib_mag_notready_count = s_calib_mag_notready_count;
        s_telemetry.imu_temp_c = s_last_imu_temp_c;
        s_telemetry.gyro_calib_std_x_dps = s_gcal_last_corr_std.x;
        s_telemetry.gyro_calib_std_y_dps = s_gcal_last_corr_std.y;
        s_telemetry.gyro_calib_std_z_dps = s_gcal_last_corr_std.z;
        s_telemetry.accel_calib_residual_g = s_last_accel_calib_residual_g;
        s_telemetry.stamp_us = now_us;
        // Chan doan nhip vong: tach "cho" khoi "ban" (xem telemetry.h).
        s_telemetry.loop_wake_by_hub  = s_loop_wake_by_hub;
        s_telemetry.loop_wake_timeout = s_loop_wake_timeout;
        s_telemetry.loop_busy_us      = s_loop_busy_us;
        xSemaphoreGive(s_telemetry_mtx);

        // ---- Do THOI GIAN XU LY cua tick nay ----
        // Dat o CUOI vong, tru mocs now_us lay ngay sau khi thuc day. Hieu so
        // nay KHONG bao gom phan cho cam bien, nen no tra loi truc tiep cau
        // hoi "CPU co du khong":
        //   busy ~ 4000us -> CPU that su khong du, phai bot viec.
        //   busy nho ma dt lon -> dang CHO, van de o nguon nhip (INT/I2C).
        s_loop_busy_us = (uint32_t)(esp_timer_get_time() - now_us);
    }
}

// ================= public API =================

bool flight_core_push_command(const command_t *cmd) {
    if (s_cmd_queue == NULL) return false;

    // KILL KHÔNG BAO GIỜ được rơi vì queue đầy — latch NGAY tại đây, trước cả
    // khi thử đẩy vào queue (xem flight_core.h). Vẫn đẩy lệnh vào queue sau đó
    // để stabilize_task chạy phần dọn dẹp FSM/controller; nhưng dù xQueueSend
    // thất bại thì motor ĐÃ cắt rồi -> vẫn trả true, vì tác dụng an toàn thực
    // sự của KILL đã hoàn thành (trả false sẽ khiến caller báo "kill lỗi"
    // trong khi motor đã dừng — sai lệch nguy hiểm theo hướng ngược lại).
    if (cmd->type == CMD_KILL) {
        enter_kill_latch("CMD_KILL (push)");
        (void)xQueueSend(s_cmd_queue, cmd, 0);
        return true;
    }

    return xQueueSend(s_cmd_queue, cmd, 0) == pdTRUE;
}

void flight_core_kill_now(const char *reason) {
    enter_kill_latch(reason ? reason : "flight_core_kill_now()");
    // Đẩy CMD_KILL để stabilize_task dọn FSM/controller ở tick kế. Nếu queue
    // đầy thì bỏ qua — motor đã cắt bằng latch + armed gate, và block kill
    // latch ở đầu mỗi tick sẽ tự ép FSM về DISARMED dù lệnh này không tới nơi.
    if (s_cmd_queue != NULL) {
        const command_t kill = { .type = CMD_KILL };
        (void)xQueueSend(s_cmd_queue, &kill, 0);
    }
}

// flight_core_i2c_scan() — xem flight_core.h. Dải 0x08..0x77 là dải 7-bit HỢP
// LỆ theo spec I2C: 0x00..0x07 và 0x78..0x7F đều là địa chỉ dành riêng
// (general call, 10-bit addressing, CBUS...), quét chúng chỉ sinh nhiễu.
int flight_core_i2c_scan(uint8_t *found, int max_found) {
    if (found == NULL || max_found <= 0) return -1;
    if (s_i2c_bus == NULL) return -1;

    // Mượn bus THẬT SỰ (chờ hub xác nhận đã dừng chạm phần cứng) — không mượn
    // được thì BÁO LỖI chứ không quét đè: quét đè cho ra kết quả nửa vời (NACK
    // giả vì bus bận) mà người đọc lại tin là "chip không có" — tệ hơn hẳn
    // việc không có kết quả nào.
    if (!sensor_hub_suspend(SENSOR_BUS_LEASE_TIMEOUT_MS)) return -2;

    int n = 0;
    for (uint8_t addr = 0x08; addr <= 0x77 && n < max_found; ++addr) {
        if (i2c_master_probe(s_i2c_bus, addr, 20) == ESP_OK) {
            found[n++] = addr;
        }
    }

    sensor_hub_resume();
    return n;
}

// flight_core_tof_reinit() — xem flight_core.h. Chạy lại ĐÚNG lời gọi
// tof_driver_init() mà flight_core_start() dùng, với CÙNG tham số, để log của
// nó xuất hiện lúc terminal đang thật sự cắm.
void flight_core_get_task_stats(flight_core_task_stats_t *out) {
    if (out == NULL) return;
    out->stabilize_stack_total_bytes = (uint32_t)STABILIZE_TASK_STACK_BYTES;
    out->stabilize_stack_free_bytes =
        (s_stabilize_task != NULL) ? (uint32_t)uxTaskGetStackHighWaterMark(s_stabilize_task) : 0u;
    out->stabilize_core = (uint8_t)STABILIZE_TASK_CORE;
    out->stabilize_priority = (uint8_t)STABILIZE_TASK_PRIORITY;
    out->sensor_hub_stack_total_bytes = sensor_hub_stack_total_bytes();
    out->sensor_hub_stack_free_bytes = sensor_hub_stack_free_bytes();
    out->sensor_hub_core = (uint8_t)SENSOR_HUB_TASK_CORE;
    out->sensor_hub_priority = (uint8_t)SENSOR_HUB_TASK_PRIORITY;
}

int flight_core_tof_reinit(void) {
#if !FC_FEATURE_TOF
    // ToF không có trong firmware -> không có gì để init lại. Trả cùng mã -1
    // như nhánh "tof_enabled=false" bên dưới: console/GUI không cần phân biệt
    // "tắt lúc build" với "tắt lúc chạy", cả hai đều là "không có ToF".
    ESP_LOGE(TAG, "tof_reinit: ToF dang TAT (SENSOR_TOF_ENABLED=0 trong app_config.h) "
                  "-> khong co gi de init");
    return -1;
#else
    if (s_i2c_bus == NULL) return -1;
    if (!s_board.tof_enabled) {
        ESP_LOGE(TAG, "tof_reinit: ToF dang TAT (SENSOR_TOF_ENABLED=0 trong app_config.h) "
                      "-> khong co gi de init");
        return -1;
    }
    // Bring-up block tới ~1s (2 lần ref calibration, mỗi lần timeout 500ms) và
    // giữ bus suốt thời gian đó. Ở trạng thái đã armed, đó là snapshot đứng
    // hình + Commander trip hard fault — chặn cứng thay vì tin caller.
    if (s_fsm.state != FSM_DISARMED) {
        ESP_LOGE(TAG, "tof_reinit tu choi: chi cho phep khi DISARMED (hien tai=%s)",
                  fsm_state_name(s_fsm.state));
        return -3;
    }

    // Hub PHẢI dừng chạm bus trước: bring-up ghi hàng trăm thanh ghi và đọc
    // ngược lại, xen vào giữa vòng đọc cảm biến sẽ cho kết quả sai mà trông
    // như thật (cùng lý do với i2c_scan ở trên).
    if (!sensor_hub_suspend(SENSOR_BUS_LEASE_TIMEOUT_MS)) {
        ESP_LOGE(TAG, "tof_reinit: khong muon duoc bus I2C khoi sensor_hub");
        return -2;
    }

    // Hub NGỪNG đọc ToF trong lúc init lại. Nếu init hỏng, cờ này ở nguyên
    // false — hub không đọc một driver đang ở trạng thái nửa vời.
    sensor_hub_set_tof_present(false);
    s_tof_ok_driver = false;

    ESP_LOGW(TAG, "tof_reinit: bat dau bring-up lai %s (addr 0x%02X, XSHUT=%d, SCL %lu Hz)",
              tof_driver_chip_name(), s_board.tof_addr, s_board.tof_xshut_gpio,
              (unsigned long)s_tof_scl_hz);

    const esp_err_t err = tof_driver_init(s_i2c_bus, s_board.tof_xshut_gpio,
                                           s_board.tof_addr, s_tof_scl_hz);

    s_tof_ok_driver = (err == ESP_OK);
    sensor_hub_set_tof_present(s_tof_ok_driver);
    sensor_hub_resume();

    if (s_tof_ok_driver) {
        ESP_LOGI(TAG, "tof_reinit: THANH CONG -> hub bat dau doc ToF, xem 'tof_test' de "
                      "kiem tra nhip mau");
        return 0;
    }
    ESP_LOGE(TAG, "tof_reinit: THAT BAI (%s) -- dong 'VL53L0X init failed line NNN' ngay ben "
                  "tren chi dung buoc hong", esp_err_to_name(err));
    return (int)err;
#endif  // FC_FEATURE_TOF
}

void flight_core_read_telemetry(telemetry_snapshot_t *out) {
    if (s_telemetry_mtx == NULL) { telemetry_snapshot_init(out); return; }
    xSemaphoreTake(s_telemetry_mtx, portMAX_DELAY);
    *out = s_telemetry;
    xSemaphoreGive(s_telemetry_mtx);
}

// ---- Live-tuning getters (xem flight_core.h) ----

void flight_core_get_attitude_gains(attitude_gains_t *out) {
    if (s_tuning_mtx == NULL) { *out = attitude_default_gains(); return; }
    xSemaphoreTake(s_tuning_mtx, portMAX_DELAY);
    *out = s_att_gains;
    xSemaphoreGive(s_tuning_mtx);
}

void flight_core_get_mahony_config(mahony_config_t *out) {
    if (s_tuning_mtx == NULL) { *out = mahony_default_config(); return; }
    xSemaphoreTake(s_tuning_mtx, portMAX_DELAY);
    *out = s_mahony.config;
    xSemaphoreGive(s_tuning_mtx);
}

// Đảo ở biên KHỚP CMD_SET_TRIM (xem giải thích đầu case CMD_SET_ATTITUDE,
// flight_core.c) — trả roll_deg/pitch_deg THEO NGHĨA NGOÀI (tiến/lùi, trái/
// phải) để round-trip đúng với @TRIM SET vừa gửi, caller (command_parser.c)
// KHÔNG cần biết/tự đảo gì thêm.
void flight_core_get_trim(float *roll_deg, float *pitch_deg) {
    if (s_tuning_mtx == NULL) { *roll_deg = 0.0f; *pitch_deg = 0.0f; return; }
    xSemaphoreTake(s_tuning_mtx, portMAX_DELAY);
    *roll_deg = s_trim_pitch_deg;
    *pitch_deg = s_trim_roll_deg;
    xSemaphoreGive(s_tuning_mtx);
}

// setpoint (SP) KHÔNG ghi dưới s_tuning_mtx (chỉ CMD_SET_ATTITUDE ghi, tần
// suất cao hơn — xem apply_command()) nhưng đọc torn-free vẫn cần mutex vì
// caller ở task khác; dùng CHUNG s_tuning_mtx cho đơn giản (tranh chấp không
// đáng kể, GET setpoint không phải đường nóng).
// Đảo ở biên KHỚP CMD_SET_ATTITUDE/CMD_CONTROL (xem giải thích đầu case
// CMD_SET_ATTITUDE) — trả roll_deg/pitch_deg THEO NGHĨA NGOÀI, round-trip
// đúng với @SP SET vừa gửi.
void flight_core_get_setpoint(float *roll_deg, float *pitch_deg, float *yaw_rate_dps) {
    if (s_tuning_mtx == NULL) { *roll_deg = 0.0f; *pitch_deg = 0.0f; *yaw_rate_dps = 0.0f; return; }
    xSemaphoreTake(s_tuning_mtx, portMAX_DELAY);
    *roll_deg = s_sp_pitch_deg;
    *pitch_deg = s_sp_roll_deg;
    *yaw_rate_dps = s_sp_yaw_rate_dps;
    xSemaphoreGive(s_tuning_mtx);
}

void flight_core_get_alt_hold_tune(alt_hold_tune_t *out) {
    if (s_tuning_mtx == NULL) { *out = alt_hold_default_tune(); return; }
    xSemaphoreTake(s_tuning_mtx, portMAX_DELAY);
    *out = s_hold_tune;
    xSemaphoreGive(s_tuning_mtx);
}

void flight_core_get_takeoff_tune(takeoff_tune_t *out) {
    if (s_tuning_mtx == NULL) { *out = takeoff_default_tune(); return; }
    xSemaphoreTake(s_tuning_mtx, portMAX_DELAY);
    *out = s_tko_tune;
    xSemaphoreGive(s_tuning_mtx);
}

void flight_core_get_landing_tune(landing_tune_t *out) {
    if (s_tuning_mtx == NULL) { *out = landing_default_tune(); return; }
    xSemaphoreTake(s_tuning_mtx, portMAX_DELAY);
    *out = s_land_tune;
    xSemaphoreGive(s_tuning_mtx);
}

void flight_core_get_commander_cfg(commander_config_t *out) {
    if (s_tuning_mtx == NULL) { *out = commander_default_config(); return; }
    xSemaphoreTake(s_tuning_mtx, portMAX_DELAY);
    *out = s_cmd_cfg;
    xSemaphoreGive(s_tuning_mtx);
}

void flight_core_get_control_input(float *rol_pct, float *pit_pct, float *yaw_pct, float *thr_pct) {
    if (s_tuning_mtx == NULL) { *rol_pct = *pit_pct = *yaw_pct = *thr_pct = 0.0f; return; }
    xSemaphoreTake(s_tuning_mtx, portMAX_DELAY);
    *rol_pct = s_ctrl_rol_pct;
    *pit_pct = s_ctrl_pit_pct;
    *yaw_pct = s_ctrl_yaw_pct;
    *thr_pct = s_ctrl_thr_pct;
    xSemaphoreGive(s_tuning_mtx);
}

// init_i2c_bus() — tạo MỘT i2c_master_bus_handle_t DUY NHẤT cho bus vật lý
// (SDA/SCL dùng chung), lưu ở s_i2c_bus (static, sống suốt vòng đời app,
// KHÔNG bao giờ gọi i2c_del_master_bus()/tạo lại — xem comment s_i2c_bus).
// imu_driver/mag_driver/tof_driver/baro_driver mỗi driver chỉ
// i2c_master_bus_add_device() để lấy device handle riêng của MÌNH — driver
// nào init/read lỗi KHÔNG ảnh hưởng bus hay device handle của driver khác
// (khác object hoàn toàn, không có bước reset/reinit bus chung nào).
static esp_err_t init_i2c_bus(const flight_core_board_config_t *cfg, i2c_master_bus_handle_t *out_bus) {
    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = cfg->i2c_sda_gpio,
        .scl_io_num = cfg->i2c_scl_gpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        // Pull-up 2k2 ngoài đã có (R14/R15) — internal pull-up ESP32-S3
        // (~45kΩ) bật thêm ở đây vô hại song song, giữ hành vi tương đương
        // bản legacy trước đó (GPIO_PULLUP_ENABLE).
        .flags = { .enable_internal_pullup = true },
    };
    return i2c_new_master_bus(&bus_cfg, out_bus);
}

esp_err_t flight_core_start(const flight_core_board_config_t *board_cfg) {
    s_board = *board_cfg;

#if FC_FEATURE_BENCH_MODE
    // Khong the bo qua duoc: in 5 dong lien tiep luc boot. BENCH_MODE bi bo
    // quen o 1 nghia la drone khong bao gio quay dong co va nguoi dung se di
    // tim loi phan cung.
    for (int i = 0; i < 5; i++) {
        ESP_LOGW(TAG, "*** BENCH MODE BAT (BENCH_MODE_ENABLED=1) -- DONG CO SE KHONG BAO GIO QUAY ***");
    }
#endif

    s_att_gains = attitude_default_gains();
    s_hold_tune = alt_hold_default_tune();
    s_tko_tune = takeoff_default_tune();
    s_land_tune = landing_default_tune();
    s_cmd_cfg = commander_default_config();
    memset(&s_imu_calib, 0, sizeof(s_imu_calib));

    // Nạp NVS trước init. Accel/mag/trim/gyro ĐỀU là persistent authoritative.
    //
    // ============================================================================
    // GYRO: CALIB MỘT LẦN, SAU ĐÓ CHỈ NẠP — theo yêu cầu người dùng
    // ============================================================================
    // NVS có bias hợp lệ  -> nạp, ÁP DỤNG runtime, bay luôn. KHÔNG calib lại.
    // NVS trống           -> calib LẦN ĐẦU (xem cuối hàm), lưu lại, từ đó thôi.
    //
    // ⚠ ĐÂY LÀ CHỦ Ý, NGƯỢC với "fresh calibration mỗi boot" của PX4 mà bản
    // trước làm theo. Đánh đổi phải biết rõ:
    //   ĐƯỢC: cấp nguồn là bay được ngay; không cần giữ drone đứng yên vài giây
    //         mỗi lần boot; không bao giờ bị chặn ARM chỉ vì lúc khởi động drone
    //         đang được cầm trên tay.
    //   MẤT:  bias KHÔNG theo được trôi nhiệt. Bias đo ở 25°C dùng lại ở 40°C sẽ
    //         lệch, và yaw sẽ trôi trở lại.
    //   BÙ:   telemetry có gyro_cal_temp_warn (bật khi |temp hiện tại - temp lúc
    //         calib| > CALIB_TEMP_WARN_DELTA_C). Thấy cờ đó thì gõ 'calib_gyro'.
    //         Đó là đường DUY NHẤT còn lại để đo lại, và nó do người dùng chủ
    //         động gọi — không có luồng tự động nào ép calib lại nữa.
    memset(&s_calib, 0, sizeof(s_calib));
    if (calibration_load(&s_calib) != ESP_OK) {
        ESP_LOGE(TAG, "calibration_load() loi -> coi nhu CHUA calib gi (an toan mac dinh, se khong ARM duoc)");
    }
    if (s_calib.gyro_valid_from_nvs) {
        // Gán THẲNG vào s_imu_calib thay vì gọi gyro_cal_set_runtime_bias():
        // hàm đó đi qua sensor_hub_set_imu_calib(), mà hub CHƯA start ở đây
        // (sensor_hub_start() nằm cuối hàm này và nhận &s_imu_calib làm tham
        // số). Gán trực tiếp là đủ và không có hazard thứ tự nào.
        s_imu_calib.gyro_bias_dps = s_calib.gyro_bias_dps;
        s_calib.gyro_valid = true;
        ESP_LOGW(TAG, "GYRO: NAP BIAS TU NVS =(%.3f,%.3f,%.3f)dps (calib luc %s%.1fC) "
                      "-> AP DUNG NGAY, BO QUA calib. Go 'calib_gyro' neu muon do lai.",
                  (double)s_calib.gyro_bias_dps.x, (double)s_calib.gyro_bias_dps.y,
                  (double)s_calib.gyro_bias_dps.z,
                  s_calib.gyro_cal_temp_valid_from_nvs ? "" : "unknown/",
                  (double)s_calib.gyro_cal_temp_c);
    } else {
        s_calib.gyro_valid = false;
        ESP_LOGW(TAG, "GYRO: NVS CHUA co bias -> chay calib LAN DAU. DAT DRONE YEN "
                      "TREN MAT PHANG. Ket qua se duoc luu; cac lan boot sau chi nap tu NVS.");
    }
    // Trim nạp lại từ NVS — HOÁN TRỤC Y HỆT case CMD_SET_TRIM (NVS lưu theo quy
    // ước NGOÀI). Quên hoán ở đây thì trim đã dò đúng sẽ quay 90° sau khi reboot
    // và không ai nghĩ tới việc nghi ngờ đường nạp NVS.
    if (s_calib.trim_valid) {
        s_trim_pitch_deg = s_calib.trim_roll_deg;
        s_trim_roll_deg  = s_calib.trim_pitch_deg;
        ESP_LOGI(TAG, "TRIM nap tu NVS: roll=%.2f pitch=%.2f (deg)",
                  (double)s_calib.trim_roll_deg, (double)s_calib.trim_pitch_deg);
    }
    recompute_uncalibrated();

    attitude_state_reset(&s_att_state);
    alt_hold_reset(&s_hold_state);
    takeoff_reset(&s_tko_state);
    landing_reset(&s_land_state);
    mahony_init(&s_mahony, NULL);
    alt_estimator_reset(&s_alt_est);

    const int64_t now_us = esp_timer_get_time();
    fsm_init(&s_fsm, now_us);
    commander_init(&s_cmd_state, now_us);

    telemetry_snapshot_init(&s_telemetry);

    s_cmd_queue = xQueueCreate(COMMAND_QUEUE_DEPTH, sizeof(command_t));
    if (s_cmd_queue == NULL) return ESP_ERR_NO_MEM;

    s_telemetry_mtx = xSemaphoreCreateMutex();
    if (s_telemetry_mtx == NULL) return ESP_ERR_NO_MEM;

    s_tuning_mtx = xSemaphoreCreateMutex();
    if (s_tuning_mtx == NULL) return ESP_ERR_NO_MEM;

    esp_err_t err = motor_driver_init(board_cfg->motor_gpio);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor_driver_init FAILED: %s (khong the bay, kiem tra board_config.h)",
                  esp_err_to_name(err));
        return err;   // motor là bắt buộc — không có motor thì không nên chạy task
    }

    err = init_i2c_bus(board_cfg, &s_i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "I2C bus init that bai: %s -> cam bien se khong hoat dong", esp_err_to_name(err));
        s_i2c_bus = NULL;
    } else {
        // ---- Tốc độ SCL: MỘT nguồn sự thật cho MỌI device trên bus ----
        // board_cfg->i2c_freq_hz (= BOARD_I2C_FREQ_HZ) TRƯỚC ĐÂY LÀ FIELD CHẾT:
        // caller gán nhưng flight_core không đọc, trong khi mỗi driver tự
        // #define 400000 riêng. Sửa BOARD_I2C_FREQ_HZ khi đó KHÔNG có tác dụng
        // gì — đúng loại bẫy im lặng tệ nhất (config trông có thẩm quyền nhưng
        // không điều khiển gì cả). Giờ nó được truyền xuống từng driver.
        //
        // Validate thay vì tin: caller quên set field -> 0 -> mọi
        // i2c_master_bus_add_device() sẽ lỗi và không cảm biến nào chạy. Rơi về
        // 400kHz (Fast Mode, tốc độ CẢ 4 chip đều hỗ trợ) và log RÕ.
        uint32_t i2c_scl_hz = (uint32_t)board_cfg->i2c_freq_hz;
        if (i2c_scl_hz < 10000 || i2c_scl_hz > 1000000) {
            ESP_LOGE(TAG, "i2c_freq_hz=%d KHONG hop le (ngoai 10k..1M) -> dung 400000. "
                          "Kiem tra BOARD_I2C_FREQ_HZ trong board_config.h",
                      board_cfg->i2c_freq_hz);
            i2c_scl_hz = 400000;
        }
        ESP_LOGI(TAG, "I2C bus SDA=%d SCL=%d, tat ca device o %lu Hz%s",
                  board_cfg->i2c_sda_gpio, board_cfg->i2c_scl_gpio,
                  (unsigned long)i2c_scl_hz,
                  (i2c_scl_hz >= 400000) ? " (Fast Mode)" : "");
        s_i2c_scl_hz = i2c_scl_hz;   // flight_core_tof_reinit() dùng lại giá trị ĐÃ chuẩn hoá này

        s_imu_ok_driver = (imu_driver_init(s_i2c_bus, board_cfg->imu_addr, i2c_scl_hz) == ESP_OK);
        if (!s_imu_ok_driver) {
            // imu_driver.c KHÔNG còn là stub (WHO_AM_I probe + config thật) —
            // fail ở đây nghĩa là WHO_AM_I đọc sai/đọc lỗi thật sự (xem log
            // "WHO_AM_I sai"/"doc WHO_AM_I that bai" từ imu_driver.c ngay
            // TRƯỚC dòng này) — kiểm tra dây SDA/SCL, địa chỉ AD0, pull-up.
            ESP_LOGW(TAG, "IMU init THAT BAI -> attitude se khong bao gio valid "
                          "-> KHONG THE ARM, an toan mac dinh. Xem log imu_driver.c o tren de biet ly do.");
        }
        // Cờ *_enabled đến từ SENSOR_*_ENABLED trong main/app_config.h (caller
        // set trước khi gọi flight_core_start()) — =false thì BỎ QUA HẲN việc
        // init, không dò I2C, không log warning "not found". Mỗi driver chỉ
        // add device CỦA NÓ vào s_i2c_bus dùng chung — lỗi 1 driver không đụng
        // tới device handle/bus của driver khác (xem init_i2c_bus() ở trên).
#if FC_FEATURE_MAG
        if (board_cfg->mag_enabled) {
            s_mag_ok_driver = (mag_driver_init(s_i2c_bus, board_cfg->mag_addr, i2c_scl_hz) == ESP_OK);
        } else {
            ESP_LOGI(TAG, "mag TAT theo app_config (SENSOR_MAG_ENABLED=0) -> bo qua init");
        }
#else
        // Cùng một dòng log như nhánh runtime ở trên: với người dùng, "tắt lúc
        // biên dịch" và "tắt lúc chạy" phải nhìn giống hệt nhau. s_mag_ok_driver
        // giữ nguyên false -> mọi nhánh phía dưới hành xử y như cũ.
        ESP_LOGI(TAG, "mag TAT theo app_config (SENSOR_MAG_ENABLED=0) -> bo qua init");
#endif

#if FC_FEATURE_TOF
        if (board_cfg->tof_enabled) {
            // Tốc độ RIÊNG cho ToF nếu caller chỉ định (0 = dùng chung bus).
            // Validate cùng dải với bus chung: một số rác ở đây sẽ làm
            // i2c_master_bus_add_device() hỏng và ToF chết im lặng.
            uint32_t tof_hz = board_cfg->tof_freq_hz;
            if (tof_hz == 0) {
                tof_hz = i2c_scl_hz;
            } else if (tof_hz < 10000 || tof_hz > 1000000) {
                ESP_LOGE(TAG, "tof_freq_hz=%lu KHONG hop le (ngoai 10k..1M) -> dung %lu Hz cua bus chung. "
                              "Kiem tra BOARD_TOF_I2C_FREQ_HZ trong board_config.h",
                          (unsigned long)tof_hz, (unsigned long)i2c_scl_hz);
                tof_hz = i2c_scl_hz;
            }
            s_tof_scl_hz = tof_hz;
            if (tof_hz != i2c_scl_hz) {
                ESP_LOGI(TAG, "ToF chay RIENG o %lu Hz (bus chung %lu Hz) — hop le vi toc do la "
                              "thuoc tinh cua DEVICE, xem BOARD_TOF_I2C_FREQ_HZ",
                          (unsigned long)tof_hz, (unsigned long)i2c_scl_hz);
            }
            // Bring-up địa chỉ ToF (XSHUT sequencing, xem tof_driver.h).
            s_tof_ok_driver = (tof_driver_init(s_i2c_bus,
                                                board_cfg->tof_xshut_gpio,
                                                board_cfg->tof_addr,
                                                tof_hz) == ESP_OK);
        } else {
            ESP_LOGE(TAG, "ToF TAT theo app_config -> altitude takeoff/hold khong san sang");
        }
#else
        ESP_LOGE(TAG, "ToF bi cat khoi build -> altitude takeoff/hold khong san sang");
#endif

#if FC_FEATURE_BARO
        if (board_cfg->baro_enabled) {
            s_baro_ok_driver = (baro_driver_init(s_i2c_bus, board_cfg->baro_addr, i2c_scl_hz) == ESP_OK);
            if (s_baro_ok_driver) {
                // Calibrate chi de BALT/debug co moc tuong doi. Ket qua nay
                // khong di vao estimator, ARM, takeoff, landing hay PID do cao.
                if (baro_driver_calibrate_ground() != ESP_OK) {
                    ESP_LOGW(TAG, "baro calibrate_ground that bai -> chi mat BALT debug; flight control khong doi");
                }
            }
        } else {
            ESP_LOGI(TAG, "baro TAT theo app_config (SENSOR_BARO_ENABLED=0) -> bo qua init");
        }
#else
        ESP_LOGI(TAG, "baro TAT theo app_config (SENSOR_BARO_ENABLED=0) -> bo qua init");
#endif
    }

    // ADC1 độc lập với bus I2C — init dù I2C có lỗi hay không. Lỗi ở đây
    // KHÔNG chặn boot (giống các driver cảm biến khác): telemetry.battery_v
    // sẽ luôn 0, Commander coi là "chưa có mẫu" (an toàn mặc định, xem step 1
    // của stabilize_task).
#if FC_FEATURE_BATTERY
    if (board_cfg->battery_enabled) {
        s_battery_ok_driver = (battery_driver_init(board_cfg->battery_adc1_channel) == ESP_OK);
        if (!s_battery_ok_driver) {
            ESP_LOGW(TAG, "battery_driver_init that bai -> telemetry.battery_v se luon 0 "
                          "(Commander se KHONG bao gio bao pin thap, kiem tra board_config.h)");
        }
    } else {
        ESP_LOGI(TAG, "battery ADC TAT theo app_config -> bo qua init, telemetry.battery_v luon 0");
    }
#else
    ESP_LOGI(TAG, "battery ADC TAT theo app_config -> bo qua init, telemetry.battery_v luon 0");
#endif  // FC_FEATURE_BATTERY

    // Flight-control altitude co dung MOT external correction: VL53L0X.
#if FC_FEATURE_TOF
    ESP_LOGI(TAG, "do cao flight-control = IMU + VL53L0X (max target %.2fm). "
                  "Barometer neu co chi DEBUG/telemetry, BAROFC luon 0.",
              (double)ALT_EST_MAX_FLIGHT_Z_M);
#else
    ESP_LOGE(TAG, "KHONG co VL53L0X trong build -> takeoff/alt_hold khong dung duoc");
#endif

    // Calib gyro CHỈ chạy khi NVS chưa có bias — tức đúng MỘT LẦN trong đời bo
    // (hoặc sau 'calib_erase'). Boot sau nạp thẳng từ NVS ở đầu hàm này và
    // KHÔNG vào đây nữa.
    //
    // ⚠ KHÔNG được gọi gyro_calibration_start() vô điều kiện: việc đầu tiên nó
    // làm là gyro_cal_set_runtime_bias(vec3f_zero()) — tức XOÁ SẠCH bias vừa nạp
    // từ NVS. Chính vì thế điều kiện phải nằm ở ĐÂY chứ không phải bên trong
    // hàm đó.
    //
    // FSM chỉ consume snapshot của sensor_hub; không có task calibration nào gọi
    // imu_driver_read() song song.
    if (!s_calib.gyro_valid) {
        gyro_calibration_start();
    } else {
        // Đã có bias dùng được -> không có bước calib nào chạy -> Mahony không
        // cần reset (nó khởi tạo sạch ngay bên dưới với bias đã đúng từ tick
        // đầu tiên). Không set cờ này thì vòng điều khiển sẽ chờ một lần reset
        // không bao giờ tới.
        s_gcal_reset_done = true;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(
        stabilize_task, "stabilize", STABILIZE_TASK_STACK_BYTES, NULL,
        STABILIZE_TASK_PRIORITY, &s_stabilize_task, STABILIZE_TASK_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreatePinnedToCore(stabilize_task) FAILED");
        return ESP_ERR_NO_MEM;
    }

    // Khởi động sensor_hub SAU khi đã có TaskHandle thật của stabilize_task
    // (hub cần handle để đánh thức). Trong lúc chờ dòng này, stabilize_task
    // vẫn chạy bình thường bằng đồng hồ FreeRTOS (s_hub_notify_active=false)
    // và đọc snapshot rỗng — không có khoảng "chết" nào.
    //
    // Hub sở hữu chân INT của MPU6050 (trước đây stabilize_task giữ) và toàn
    // bộ I2C. Thất bại ở đây KHÔNG làm flight_core_start() fail nếu chỉ là
    // không bật được ngắt — hub tự rơi về đồng hồ FreeRTOS.
    const esp_err_t herr = sensor_hub_start(
        s_stabilize_task, board_cfg->imu_int_gpio, &s_imu_calib,
        s_imu_ok_driver, s_mag_ok_driver, s_baro_ok_driver,
        s_battery_ok_driver, s_tof_ok_driver);
    if (herr != ESP_OK) {
        // Không có hub = không có dữ liệu cảm biến = không bao giờ arm được.
        // Đây là lỗi THẬT, phải fail hẳn chứ không chạy tiếp im lặng.
        ESP_LOGE(TAG, "sensor_hub_start() FAILED: %s", esp_err_to_name(herr));
        return herr;
    }
    s_hub_notify_active = true;

    ESP_LOGI(TAG, "flight_core started: stabilize_task @ %dHz core %d prio %d > sensor_hub prio %d "
                  "(KHONG I2C trong vong dieu khien — moi transaction nam trong sensor_hub, "
                  "notify cua hub PREEMPT vong bay ngay, xem sensor_hub.h)",
              CONTROL_TASK_HZ, STABILIZE_TASK_CORE, STABILIZE_TASK_PRIORITY,
              SENSOR_HUB_TASK_PRIORITY);
    return ESP_OK;
}
