// Cascade giữ độ cao: alt_err -> vz_target (clamp) -> vz-PI -> dthr (offset
// quanh hover) -> throttle = hover + dthr. PORT từ nhánh ALT_MODE_HOLD của
// stabilizer_task (UAV-Mini flight_control.cpp).
//
// ĐƠN GIẢN HÓA CÓ CHỦ ĐÍCH so với bản gốc: bản gốc có failsafe-descend HÀN
// CỨNG bên trong nhánh HOLD (mất estimator giữa chừng -> tự trừ duty hạ chậm
// -> timeout thì cắt). Ở kiến trúc mới CÓ FlightStateMachine thật, việc đó dư
// thừa: alt_hold_run() chỉ báo "mất khả năng giữ" (engage_lost=true), Commander
// phân loại đây là SOFT FAULT và chuyển máy trạng thái HOLDING/FLYING ->
// LANDING — landing sequence (takeoff_land.h) đã có sẵn nhánh BLIND xử lý mất
// ToF, mạnh hơn failsafe cũ. Không cần viết lại logic hạ khẩn hai lần.
#pragma once

#include "flight_core/tuning.h"
#include "flight_core/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float alt_kp;      // alt_err -> vz_target [1/s]
    float vz_kp;       // vz_err -> duty (P)
    float vz_ki;       // vz_err -> duty (I)
    float vz_ilimit;   // trần |I-term| (duty)
    float hover;       // ga hover ước lượng (duty) — điểm neo cascade
} alt_hold_tune_t;

typedef struct {
    float vz_integral;
    bool  engaged;     // đang đóng vòng bumpless (đã preload I lần đầu)
    // Cascade vz vừa bị kẹp trần/sàn duty ở lần gọi gần nhất. Ghi bởi
    // alt_hold_vz_cascade() mỗi lần gọi — dùng cho telemetry (ALT_PID_SAT) và
    // để tầng trên biết PID đang đòi nhiều hơn phần cứng cho được.
    bool  vz_saturated;
} alt_hold_state_t;

// Giá trị mặc định: xem flight_core/tuning.h (mục 2 — ALT HOLD). Đổi số ở ĐÓ,
// không ở đây.

alt_hold_tune_t alt_hold_default_tune(void);

static inline void alt_hold_reset(alt_hold_state_t *st) {
    st->vz_integral = 0.0f;
    st->engaged = false;
    st->vz_saturated = false;
}

// Preload bumpless khi bắt đầu đóng vòng: vz_integral = clamp(throttle-hover, ±ilimit)
// (thuật toán cộng dồn PI sau đó tiếp tục mượt từ điểm này, không giật ga).
void alt_hold_preload(alt_hold_state_t *st, const alt_hold_tune_t *tune, int current_throttle_duty);

// Cascade vz THUẦN (dùng chung HOLD/TAKEOFF/LANDING): vz_target(m/s) -> vz-PI
// -> throttle. KHÔNG liên quan cờ freeze integrator của attitude PID (hai cơ
// chế hoàn toàn độc lập — xem tuning.h mục 1 vs mục 2).
//
// ANTI-WINDUP BA LỚP (spec §18 — không dùng cùng một rule mù cho attitude I và
// altitude I):
//   1. i_limit_duty (do caller quyết): trần |I| cho LẦN GỌI NÀY. Bình thường
//      truyền tune->vz_ilimit. Takeoff truyền một trần NHỎ HƠN trong cửa sổ
//      trước liftoff (xem TAKEOFF_PRELIFT_I_LIMIT_DUTY): drone còn bị mặt đất
//      giữ nên vz_err dương dai dẳng, nhưng cascade VẪN cần quyền đẩy ga lên để
//      TÌM điểm nhấc (hover chỉ là ước lượng). GIỚI HẠN, KHÔNG ĐÓNG BĂNG.
//   2. freeze_integral: ngừng cộng dồn HOÀN TOÀN nhưng GIỮ giá trị. Dùng cho
//      trường hợp thật sự không được phép tích lũy gì.
//      ⚠ KHÔNG dùng cái này cho cửa sổ trước liftoff — đã thử và nó chặn luôn
//      thẩm quyền nhấc drone (xem ghi chú ở TAKEOFF_PRELIFT_* trong tuning.h).
//   3. Conditional integration (tự động, luôn bật): không cộng dồn theo chiều
//      đang bị bão hoà. Đây là anti-windup THẬT, khác hẳn việc chỉ clamp giá
//      trị cuối — clamp một mình vẫn để I bám trần và mất nhiều thời gian mới
//      nhả ra khi sai số đổi dấu.
// Ghi st->vz_saturated mỗi lần gọi.
int alt_hold_vz_cascade(alt_hold_state_t *st, const alt_hold_tune_t *tune,
                         float vz_target_ms, float vz_measured_ms, float dt,
                         bool freeze_integral, float i_limit_duty,
                         int min_throttle_duty, int safe_max_duty);

typedef struct {
    int  throttle_duty;
    bool hold_driving;    // controller đang cầm lái (true trừ khi chưa từng engage)
    bool engage_lost;     // ĐANG engaged mà giờ mất điều kiện -> báo Commander soft-fault
    float vz_target_ms;   // vz_target TẦNG NGOÀI vừa tính (0 nếu không engage) — telemetry/debug (README điểm 10)
} alt_hold_result_t;

// alt_hold_run() — một bước HOLD hoàn chỉnh.
//   alt_target_m       : độ cao mục tiêu hiện tại (m).
//   alt_valid/alt_m/vz_ms : từ alt_estimator.
//   tilt_ok            : |roll|,|pitch| trong gate (tính ở tầng gọi, dùng chung
//                         guard với attitude_control nếu cần).
//   manual_throttle_duty : throttle tay hiện tại — dùng làm preload lúc mới engage
//                         VÀ làm fallback khi chưa từng engage được.
void alt_hold_run(alt_hold_state_t *st, const alt_hold_tune_t *tune,
                   float alt_target_m, bool alt_valid, float alt_m, float vz_ms,
                   bool tilt_ok, int manual_throttle_duty, float dt,
                   int safe_max_duty, alt_hold_result_t *out);

#ifdef __cplusplus
}
#endif
