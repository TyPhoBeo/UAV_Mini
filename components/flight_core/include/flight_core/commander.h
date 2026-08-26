// Commander — trọng tài an toàn, quyền override cao nhất. MỌI setpoint từ
// Python phải qua Commander TRƯỚC khi vào PID (geofence clamp), và Commander
// là nơi DUY NHẤT quyết định soft-fault (-> LANDING) hay hard-fault
// (-> EMERGENCY), dựa trên heartbeat watchdog, pin, và tình trạng cảm biến.
//
// PHÂN LOẠI FAULT (theo đúng yêu cầu):
//   SOFT (heartbeat timeout, pin thấp, mất estimator giữa chừng khi đang HOLD/
//         FLYING — xem alt_hold.h) -> LANDING. Treo script Python là chuyện
//         thường, phải hạ êm, KHÔNG cắt máy giữa trời.
//   HARD (IMU fail, |tilt| > hard_tilt_deg, motor bão hòa kéo dài) -> EMERGENCY.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "flight_core/alt_estimator.h"
#include "flight_core/flight_state_machine.h"
#include "flight_core/tuning.h"
#include "flight_core/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FAULT_NONE = 0,
    FAULT_SOFT,
    FAULT_HARD,
} fault_class_t;

typedef struct {
    float alt_min_m;
    float alt_max_m;

    float battery_floor_v;          // dưới ngưỡng này -> soft fault (LANDING)

    int   heartbeat_timeout_ms;     // Python không gọi fc.heartbeat() quá lâu -> soft fault

    float hard_tilt_deg;            // |roll|/|pitch| vượt -> hard fault (lật/mất kiểm soát)
    int   motor_sat_hard_ms;        // motor bão hòa (duty kịch trần) liên tục quá lâu -> hard fault
} commander_config_t;

// Mặc định: xem flight_core/tuning.h (mục 5 — COMMANDER). CHƯA tune trên phần
// cứng thật (S3 mới), chỉ là điểm khởi đầu an toàn. battery_floor_v PHẢI
// chỉnh theo số cell pin thực tế trước khi bay.
commander_config_t commander_default_config(void);

typedef struct {
    int64_t last_heartbeat_us;   // cập nhật mỗi lần commander_heartbeat() được gọi
    int64_t motor_sat_since_us;  // 0 = hiện không bão hòa
} commander_state_t;

static inline void commander_init(commander_state_t *st, int64_t now_us) {
    st->last_heartbeat_us = now_us;
    st->motor_sat_since_us = 0;
}

// Gọi mỗi khi nhận fc.heartbeat() từ Python (qua command queue — xem
// micropython_module/fc). Reset watchdog timer.
static inline void commander_heartbeat(commander_state_t *st, int64_t now_us) {
    st->last_heartbeat_us = now_us;
}

// commander_credit_stall() — TRẢ LẠI khoảng thời gian mà CHÍNH FIRMWARE đã
// chiếm của vòng bay, để watchdog không tính nó là "ground-station im lặng".
//
// VÌ SAO CẦN: watchdog heartbeat đo "bao lâu rồi trạm mặt đất chưa nói gì".
// Nhưng vài lệnh phải chạy công việc CHẶN dài ngay trong stabilize_task —
// baro_driver_calibrate_ground() chẳng hạn, chặn ~970ms (300ms settle + 32 mẫu
// x 20ms). Suốt thời gian đó KHÔNG lệnh nào được rút khỏi queue và
// commander_evaluate() cũng không chạy. Lúc vòng lặp chạy tiếp,
// last_heartbeat_us đã "cũ" thêm 970ms mà trạm mặt đất KHÔNG hề im lặng — nó
// vẫn gửi đều, chỉ là không ai đọc.
//
// Hậu quả đã quan sát được trên phần cứng thật: bấm ARM -> calib baro chặn
// 970ms -> HBAGE nhảy 155ms lên 1026ms -> vượt heartbeat_timeout_ms (1000) ->
// soft fault -> DISARMED NGAY sau khi vừa arm xong. Firmware tự disarm vì
// thời gian chính nó tiêu tốn. Dấu hiệu nhận ra: CMDAGE và HBAGE nhảy CÙNG
// một lượng (cả hai đồng hồ cùng bị đóng băng), khác hẳn mất link thật (chỉ
// CMDAGE tăng nếu trạm còn gửi heartbeat, hoặc cả hai tăng NHƯNG loop_max
// bình thường).
//
// KHÔNG che giấu mất link thật: chỉ cộng bù ĐÚNG khoảng đã đo được bằng đồng
// hồ quanh đoạn chặn. Nếu trạm mặt đất chết THẬT trong 970ms đó thì tick kế
// tiếp vẫn thấy tuổi heartbeat tăng bình thường và vẫn trip đúng hạn.
static inline void commander_credit_stall(commander_state_t *st, int64_t stalled_us) {
    if (stalled_us <= 0) return;
    st->last_heartbeat_us += stalled_us;
    if (st->motor_sat_since_us != 0) st->motor_sat_since_us += stalled_us;
}

// Geofence: kẹp altitude target người dùng đặt vào [alt_min, alt_max]. Gọi
// TRƯỚC khi ghi vào setpoint, mọi đường vào (takeoff target, move, set_altitude).
static inline float commander_clamp_altitude(const commander_config_t *cfg, float requested_alt_m) {
    const float tof_safe_max = ALT_EST_MAX_FLIGHT_Z_M;
    const float upper = (cfg->alt_max_m < tof_safe_max) ? cfg->alt_max_m : tof_safe_max;
    return clampf(requested_alt_m, cfg->alt_min_m, upper);
}

typedef struct {
    // ---- Trạng thái bay hiện tại — QUYẾT ĐỊNH CHÍNH SÁCH ----
    // Commander chạy ở MỌI state đã armed (ARMED/TAKING_OFF/HOLDING/FLYING/
    // LANDING/EMERGENCY), nhưng PHẢN ỨNG khác nhau theo state. Vì vậy nó phải
    // BIẾT mình đang ở đâu — trước đây caller tự lọc "chỉ gọi khi HOLDING/
    // FLYING", nghĩa là mất heartbeat lúc TAKING_OFF hay pin tụt lúc LANDING
    // đều KHÔNG được phát hiện.
    fsm_state_t state;
    bool  airborne;            // đã rời đất (từ takeoff detector / FSM) — quyết định
                                // "hạ êm được không" vs "cắt luôn cho an toàn"

    bool  imu_ok;              // attitude.valid từ Mahony
    bool  imu_stale;           // mẫu IMU quá hạn (sensor_hub timestamp) — KHÁC !imu_ok:
                                // Mahony có thể vẫn "valid" trong khi dữ liệu đã chết
    float roll_deg, pitch_deg;
    float battery_v;
    bool  motor_saturated;     // true nếu duty hiện tại đang kịch trần (caller đo)
    bool  alt_hold_engage_lost; // từ alt_hold_result_t.engage_lost (xem alt_hold.h)
    // HAI tín hiệu KHÁC NHAU về altitude estimator (xem alt_estimator.h
    // "VALIDITY vs DEGRADED"):
    //   alt_estimator_lost     = state KHÔNG dùng được (NaN/Inf, pipeline hỏng)
    //   alt_estimator_degraded = state vẫn hợp lệ nhưng đã quá lâu KHÔNG có
    //                             correction nào -> Z đang dead-reckon thuần
    //                             accel và trôi BẬC HAI theo thời gian.
    // Trước đây chỉ có `lost` và nó = "baro stale", nên tắt baro là estimator
    // bị coi như chết ngay dù IMU+ToF vẫn tốt. Tách ra để mất một nguồn
    // correction là DEGRADED (vẫn bay được, có hạn giờ) chứ không phải CHẾT.
    bool  alt_estimator_lost;
    bool  alt_estimator_degraded;
    int   deadline_miss_streak; // số tick LIÊN TIẾP vòng điều khiển trượt hạn
} commander_inputs_t;

typedef struct {
    fault_class_t fault;
    const char    *reason;      // chuỗi tĩnh, chỉ để log/telemetry
} commander_result_t;

// Số tick trượt hạn LIÊN TIẾP trước khi coi là hard fault. Vòng điều khiển
// chạy chậm hẳn nghĩa là PID/estimator đang tính trên dt sai và phản ứng trễ —
// không cứu được bằng cách bay tiếp.
#define COMMANDER_DEADLINE_MISS_HARD   50   // ~200ms @250Hz

// commander_evaluate() — gọi mỗi vòng lặp ở MỌI state đã armed (KHÔNG chỉ
// HOLDING/FLYING). Không tự thay đổi FSM — trả fault_class_t, caller
// (flight_core.c) map sang fsm_on_soft_fault()/fsm_on_hard_fault().
//
// CHÍNH SÁCH THEO STATE (xem commander.c để biết chi tiết từng nhánh):
//   - IMU fail/stale ở BẤT KỲ state airborne nào  -> HARD
//   - tilt vượt hard_tilt_deg                      -> HARD (mọi state)
//   - vòng điều khiển trượt hạn liên tục           -> HARD (mọi state)
//   - heartbeat mất khi HOLDING/FLYING             -> SOFT (hạ êm)
//   - heartbeat mất khi TAKING_OFF                 -> SOFT (huỷ leo, hạ ngay)
//   - heartbeat mất khi ARMED (chưa bay)           -> SOFT (về DISARMED)
//   - heartbeat mất khi LANDING                    -> BỎ QUA (đang hạ rồi)
//   - pin dưới sàn                                 -> SOFT, trừ khi đang LANDING
//   - mất estimator khi đang bay                   -> SOFT
commander_result_t commander_evaluate(commander_state_t *st, const commander_config_t *cfg,
                                       const commander_inputs_t *in, int64_t now_us);

#ifdef __cplusplus
}
#endif
