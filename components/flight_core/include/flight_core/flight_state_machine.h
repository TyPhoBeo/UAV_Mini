// FlightStateMachine — ĐÚNG TOPOLOGY theo yêu cầu (đã sửa từ bản nháp):
//
//   DISARMED → ARMED          : "arm hợp lệ" (guard: attitude.valid && |tilt|<10°)
//   ARMED → TAKING_OFF        : lệnh takeoff
//   ARMED → DISARMED          : motor stop
//   TAKING_OFF → HOLDING      : liftoff xác nhận (xem GHI CHÚ 1 dưới)
//   TAKING_OFF → EMERGENCY    : ramp chạm trần mà không nhấc (hard fault)
//   HOLDING → FLYING          : có move command
//   FLYING → HOLDING          : hết move / lệnh hold
//   HOLDING/FLYING → LANDING  : lệnh land HOẶC soft failsafe
//   LANDING → HOLDING         : hủy landing / go-around, giữ bumpless Vz-I
//   HOLDING/FLYING → EMERGENCY: hard fault
//   LANDING → DISARMED        : đã chạm đất
//   EMERGENCY → LANDING       : fault còn kiểm soát → hạ có kiểm soát
//   EMERGENCY → DISARMED      : fault mất kiểm soát (lật/IMU chết) → cắt motor ngay
//
// GHI CHÚ 1 — TAKING_OFF → HOLDING, lệch có chủ đích so với bản nháp gốc:
// Bản nháp mô tả guard là "liftoff && đạt ~70% target, chốt hover_est, alt_pid.
// reset() trước khi sang". Logic ĐÃ PORT (takeoff_land.h, nguyên bản từ UAV-Mini
// đã bay + debug thật) dùng guard KHÁC, cụ thể hơn và đã qua kiểm chứng:
//   - Guard = spool_done && airborne (KHÔNG phải %target — bản port không có
//     khái niệm "leo tới X% rồi bàn giao"; nó rời đất rồi CHỐT LUÔN độ cao hiện
//     tại làm target, giao HOLDING giữ nguyên đó).
//   - "chốt hover_est": KHÔNG cần bước riêng — cascade vz-PI (alt_hold.h) đã tự
//     hội tụ integral về (hover thực - hover danh nghĩa) liên tục, đó CHÍNH LÀ
//     ước lượng hover, không phải một giá trị "chốt" một lần.
//   - "alt_pid.reset() trước khi sang": CỐ Ý KHÔNG reset về 0 — mà PRELOAD
//     bumpless (vz_integral = throttle_hiện_tại - hover). Reset về 0 sẽ giật ga
//     ngay lúc vừa rời đất — đúng lớp bug I-term windup đã vá trong session port
//     gốc. Preload-bumpless là hành vi ĐÚNG, đã kiểm chứng trên phần cứng thật.
// Nêu rõ deviation này để người review biết đây là quyết định kỹ thuật có căn
// cứ, không phải bỏ sót.
//
// GHI CHÚ 2 — EMERGENCY 2 lối ra: quyết định lối nào dùng "controllable" — xem
// fsm_on_emergency_resolve(). Với TAKING_OFF→EMERGENCY (chưa từng rời đất),
// caller PHẢI truyền controllable=false (không có gì để "hạ" khi còn ở mặt đất)
// -> đi thẳng DISARMED. Với HOLDING/FLYING→EMERGENCY (hard fault giữa chừng),
// controllable = Commander tự đánh giá (attitude còn hợp lý, chưa lật hẳn) ->
// LANDING nếu còn cứu được, DISARMED nếu không (xem commander.h).
//
// GHI CHÚ 3 — FSM_BENCH_RAMP (bench-test PID tuning, xem CMD_BENCH_RAMP_START/
// CMD_BENCH_THROTTLE_STEP/CMD_BENCH_RAMP_STOP trong command.h): ARMED →
// BENCH_RAMP → ARMED (STOP, không latch) hoặc
// → DISARMED (soft fault/DISARM, latch) hoặc → EMERGENCY (hard fault). Xử lý
// GIỐNG HỆT ARMED trong mọi lớp an toàn CHUNG (Commander chạy đầy đủ, tilt/
// attitude-stale cắt-ngay-latch của stabilize_task áp dụng — xem flight_core.c
// bước 5 "armed_now") — CHỈ khác ARMED ở việc throttle KHÔNG bị khoá 0 (có
// ramp), nên PHẢI nằm NGOÀI danh sách "chưa airborne coi như ở đất" một cách
// nhất quán: KHÔNG thêm vào `airborne` (commander.c) vì đây là bench test bị
// giữ chặt, không phải đang bay — soft-fault vẫn đi thẳng DISARMED (cắt ngay
// là đúng, không cần "hạ êm" một thứ đang nằm trên giá đỡ).
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FSM_DISARMED = 0,
    FSM_ARMED,
    FSM_BENCH_RAMP,
    FSM_TAKING_OFF,
    FSM_HOLDING,
    FSM_FLYING,
    FSM_LANDING,
    FSM_EMERGENCY,
} fsm_state_t;

typedef struct {
    fsm_state_t state;
    int64_t     state_entered_us;   // mốc vào state hiện tại (debug/telemetry)
} fsm_t;

// Guard ARM: attitude phải valid VÀ nghiêng dưới ngưỡng.
#define FSM_ARM_MAX_TILT_DEG 10.0f

static inline bool fsm_arm_guard_ok(bool attitude_valid, float roll_deg, float pitch_deg) {
    if (!attitude_valid) return false;
    float ar = roll_deg < 0 ? -roll_deg : roll_deg;
    float ap = pitch_deg < 0 ? -pitch_deg : pitch_deg;
    return ar < FSM_ARM_MAX_TILT_DEG && ap < FSM_ARM_MAX_TILT_DEG;
}

static inline void fsm_init(fsm_t *f, int64_t now_us) {
    f->state = FSM_DISARMED;
    f->state_entered_us = now_us;
}

static inline const char *fsm_state_name(fsm_state_t s) {
    switch (s) {
        case FSM_DISARMED:   return "DISARMED";
        case FSM_ARMED:      return "ARMED";
        case FSM_BENCH_RAMP: return "BENCH_RAMP";
        case FSM_TAKING_OFF: return "TAKING_OFF";
        case FSM_HOLDING:    return "HOLDING";
        case FSM_FLYING:     return "FLYING";
        case FSM_LANDING:    return "LANDING";
        case FSM_EMERGENCY:  return "EMERGENCY";
        default:             return "?";
    }
}

// ---- Mỗi hàm dưới đây là MỘT cạnh của topology, hàm thuần (không side-effect
// ngoài trả state mới). Gọi sai state nguồn -> trả nguyên state cũ (no-op),
// không bao giờ nhảy cạnh không tồn tại trong topology. ----

fsm_state_t fsm_on_arm_request(fsm_state_t cur, bool guard_ok);
fsm_state_t fsm_on_disarm_request(fsm_state_t cur);         // ARMED|BENCH_RAMP -> DISARMED (latch, xem CMD_DISARM)
fsm_state_t fsm_on_bench_ramp_start(fsm_state_t cur);       // ARMED -> BENCH_RAMP (xem GHI CHÚ 3)
fsm_state_t fsm_on_bench_ramp_stop(fsm_state_t cur);        // BENCH_RAMP -> ARMED (KHÔNG latch)
fsm_state_t fsm_on_takeoff_request(fsm_state_t cur);        // ARMED -> TAKING_OFF
fsm_state_t fsm_on_takeoff_handoff(fsm_state_t cur);        // TAKING_OFF -> HOLDING (xem GHI CHÚ 1)
fsm_state_t fsm_on_takeoff_abort(fsm_state_t cur);          // TAKING_OFF -> EMERGENCY
fsm_state_t fsm_on_move_command(fsm_state_t cur, bool moving); // HOLDING<->FLYING
fsm_state_t fsm_on_land_request(fsm_state_t cur);           // HOLDING/FLYING -> LANDING
fsm_state_t fsm_on_landing_abort_hold(fsm_state_t cur);      // LANDING -> HOLDING
fsm_state_t fsm_on_soft_fault(fsm_state_t cur);             // HOLDING/FLYING -> LANDING
fsm_state_t fsm_on_hard_fault(fsm_state_t cur);             // HOLDING/FLYING -> EMERGENCY
fsm_state_t fsm_on_landing_touchdown(fsm_state_t cur);      // LANDING -> DISARMED
fsm_state_t fsm_on_emergency_resolve(fsm_state_t cur, bool controllable); // EMERGENCY -> LANDING | DISARMED

// fsm_transition() — helper đổi state + cập nhật mốc thời gian, dùng trong
// flight_core.c sau khi một trong các hàm fsm_on_* trả state mới khác state cũ.
static inline void fsm_transition(fsm_t *f, fsm_state_t next, int64_t now_us) {
    if (next != f->state) {
        f->state = next;
        f->state_entered_us = now_us;
    }
}

#ifdef __cplusplus
}
#endif
