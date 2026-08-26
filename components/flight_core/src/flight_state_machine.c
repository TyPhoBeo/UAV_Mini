#include "flight_core/flight_state_machine.h"

fsm_state_t fsm_on_arm_request(fsm_state_t cur, bool guard_ok) {
    if (cur == FSM_DISARMED && guard_ok) return FSM_ARMED;
    return cur;
}

fsm_state_t fsm_on_disarm_request(fsm_state_t cur) {
    if (cur == FSM_ARMED || cur == FSM_BENCH_RAMP) return FSM_DISARMED;
    return cur;
}

fsm_state_t fsm_on_bench_ramp_start(fsm_state_t cur) {
    if (cur == FSM_ARMED) return FSM_BENCH_RAMP;
    return cur;
}

fsm_state_t fsm_on_bench_ramp_stop(fsm_state_t cur) {
    if (cur == FSM_BENCH_RAMP) return FSM_ARMED;
    return cur;
}

fsm_state_t fsm_on_takeoff_request(fsm_state_t cur) {
    if (cur == FSM_ARMED) return FSM_TAKING_OFF;
    return cur;
}

fsm_state_t fsm_on_takeoff_handoff(fsm_state_t cur) {
    if (cur == FSM_TAKING_OFF) return FSM_HOLDING;
    return cur;
}

fsm_state_t fsm_on_takeoff_abort(fsm_state_t cur) {
    if (cur == FSM_TAKING_OFF) return FSM_EMERGENCY;
    return cur;
}

fsm_state_t fsm_on_move_command(fsm_state_t cur, bool moving) {
    if (cur == FSM_HOLDING && moving) return FSM_FLYING;
    if (cur == FSM_FLYING && !moving) return FSM_HOLDING;
    return cur;
}

fsm_state_t fsm_on_land_request(fsm_state_t cur) {
    if (cur == FSM_HOLDING || cur == FSM_FLYING) return FSM_LANDING;
    return cur;
}

fsm_state_t fsm_on_landing_abort_hold(fsm_state_t cur) {
    if (cur == FSM_LANDING) return FSM_HOLDING;
    return cur;
}

fsm_state_t fsm_on_soft_fault(fsm_state_t cur) {
    // Commander giờ chạy ở MỌI state đã armed (xem commander.h), nên hàm này
    // phải trả lời được cho từng state chứ không chỉ HOLDING/FLYING:
    switch (cur) {
        case FSM_HOLDING:
        case FSM_FLYING:
            return FSM_LANDING;
        case FSM_TAKING_OFF:
            // Đang cất cánh mà mất heartbeat/pin yếu -> HUỶ LEO, hạ ngay.
            // Vào LANDING (không phải DISARMED) vì có thể đã rời đất rồi —
            // landing sequence tự xử lý cả trường hợp còn sát đất.
            return FSM_LANDING;
        case FSM_ARMED:
            // Chưa bay, motor chưa quay -> về DISARMED là an toàn nhất và
            // KHÔNG có gì để "hạ êm" cả.
            return FSM_DISARMED;
        case FSM_BENCH_RAMP:
            // Motor CÓ quay nhưng drone bị giữ chặt trên giá đỡ (bench-test,
            // xem GHI CHÚ 3 flight_state_machine.h) — KHÔNG phải đang bay,
            // không có gì để "hạ êm". Cắt thẳng về DISARMED.
            return FSM_DISARMED;
        default:
            // LANDING: đã đang hạ, soft fault không đổi gì (chuyển sang chính
            // nó chỉ reset pha hạ đang chạy dở — tệ hơn là để yên).
            // EMERGENCY/DISARMED: có đường xử lý riêng.
            return cur;
    }
}

fsm_state_t fsm_on_hard_fault(fsm_state_t cur) {
    // Hard fault ở BẤT KỲ state đã armed nào đều vào EMERGENCY — bước resolve
    // (flight_core.c) mới quyết định tiếp: còn kiểm soát được thì LANDING,
    // không thì DISARMED + kill latch.
    switch (cur) {
        case FSM_HOLDING:
        case FSM_FLYING:
        case FSM_TAKING_OFF:
        case FSM_LANDING:
        case FSM_ARMED:
        case FSM_BENCH_RAMP:
            return FSM_EMERGENCY;
        default:
            return cur;
    }
}

fsm_state_t fsm_on_landing_touchdown(fsm_state_t cur) {
    if (cur == FSM_LANDING) return FSM_DISARMED;
    return cur;
}

fsm_state_t fsm_on_emergency_resolve(fsm_state_t cur, bool controllable) {
    if (cur != FSM_EMERGENCY) return cur;
    return controllable ? FSM_LANDING : FSM_DISARMED;
}
