#include "flight_core/alt_hold.h"

alt_hold_tune_t alt_hold_default_tune(void) {
    alt_hold_tune_t t;
    t.alt_kp = ALT_HOLD_ALT_KP;
    t.vz_kp = ALT_HOLD_VZ_KP;
    t.vz_ki = ALT_HOLD_VZ_KI;
    t.vz_ilimit = ALT_HOLD_VZ_ILIMIT;
    t.hover = ALT_HOLD_HOVER_NOMINAL;
    return t;
}

void alt_hold_preload(alt_hold_state_t *st, const alt_hold_tune_t *tune, int current_throttle_duty) {
    st->vz_integral = clampf((float)current_throttle_duty - tune->hover,
                              -tune->vz_ilimit, tune->vz_ilimit);
    st->engaged = true;
    st->vz_saturated = false;
}

int alt_hold_vz_cascade(alt_hold_state_t *st, const alt_hold_tune_t *tune,
                         float vz_target_ms, float vz_measured_ms, float dt,
                         bool freeze_integral, float i_limit_duty,
                         int min_throttle_duty, int safe_max_duty) {
    const float vz_err = vz_target_ms - vz_measured_ms;
    const float p_term = tune->vz_kp * vz_err;

    // Trần |I| hiệu lực: caller có thể siết chặt hơn tune->vz_ilimit nhưng
    // KHÔNG được nới rộng hơn (nới ra sẽ vượt giới hạn người dùng đã tune).
    float i_limit = i_limit_duty;
    if (i_limit < 0.0f) i_limit = 0.0f;
    if (i_limit > tune->vz_ilimit) i_limit = tune->vz_ilimit;
    // Trần siết lại giữa lúc chạy -> kéo I hiện tại vào trong trần ngay, nếu
    // không nó sẽ giữ giá trị cũ vượt trần cho tới khi tự trôi xuống.
    st->vz_integral = clampf(st->vz_integral, -i_limit, i_limit);

    // Đánh giá bão hoà TRÊN GIÁ TRỊ CHƯA cộng I mới — nếu đánh giá sau khi cộng
    // thì I đã kịp windup đúng cái tick ta muốn chặn.
    const float unsat = tune->hover + p_term + st->vz_integral;
    const bool sat_hi = unsat >= (float)safe_max_duty;
    const bool sat_lo = unsat <= (float)min_throttle_duty;
    st->vz_saturated = sat_hi || sat_lo;

    // Conditional integration: chỉ chặn chiều LÀM SÂU THÊM bão hoà. Sai số đổi
    // dấu (drone bắt đầu vọt lên) thì I được nhả ra ngay, không phải đợi.
    const bool pushing_up = (vz_err > 0.0f);
    const bool blocked = (sat_hi && pushing_up) || (sat_lo && !pushing_up);

    if (!freeze_integral && !blocked) {
        st->vz_integral += tune->vz_ki * vz_err * dt;
        st->vz_integral = clampf(st->vz_integral, -i_limit, i_limit);
    }

    const float dthr = p_term + st->vz_integral;
    const int output = clampi((int)(tune->hover + dthr), min_throttle_duty, safe_max_duty);
    st->last_vz_error = vz_err;
    st->last_p_term = p_term;
    st->last_i_term = st->vz_integral;
    st->last_d_term = 0.0f;
    st->last_output_duty = (float)output;
    return output;
}

void alt_hold_run(alt_hold_state_t *st, const alt_hold_tune_t *tune,
                   float alt_target_m, bool alt_valid, float alt_m, float vz_ms,
                   bool tilt_ok, int manual_throttle_duty, float dt,
                   int safe_max_duty, alt_hold_result_t *out) {
    // TÁCH "được phép engage LẦN ĐẦU" khỏi "được phép GIỮ engage":
    //   basic_ok   : điều kiện để alt_hold còn ý nghĩa (có Z tin cậy, không
    //                nghiêng quá) — áp dụng cho CẢ HAI.
    //   can_engage : thêm ngưỡng độ cao tối thiểu, CHỈ cho lần engage ĐẦU —
    //                tránh engage khi còn nằm trên mặt đất.
    // Trước đây dùng chung một điều kiện, nghĩa là ALT_HOLD_MIN_ENGAGE_M vừa
    // là ngưỡng vào vừa là ngưỡng rớt -> KHÔNG THỂ hover dưới 10cm: takeoff
    // mới bàn giao ở độ cao vừa đạt sau spool (thường chỉ vài cm) nên sẽ bị
    // engage_lost NGAY vòng đầu -> Commander soft fault -> LANDING, dù mọi
    // thứ đang hoàn toàn bình thường.
    const bool basic_ok = alt_valid && tilt_ok;
    const bool can_engage = st->engaged ? basic_ok
                                         : (basic_ok && alt_m > ALT_HOLD_MIN_ENGAGE_M);

    out->engage_lost = false;
    out->vz_target_ms = 0.0f;

    if (can_engage) {
        if (!st->engaged) {
            // Vào HOLD: preload I-term để bumpless (dthr đầu ~ throttle_tay-hover).
            alt_hold_preload(st, tune, manual_throttle_duty);
        }

        const float alt_err = alt_target_m - alt_m;
        const float vz_target = clampf(tune->alt_kp * alt_err,
                                        -ALT_HOLD_VZ_LIMIT_MS, ALT_HOLD_VZ_LIMIT_MS);
        out->vz_target_ms = vz_target;

        // freeze_integral=false: HOLD chạy khi đã bay, không có mặt đất giữ nên
        // không có nguồn windup dai dẳng nào cần chặn cứng — conditional
        // integration bên trong cascade đã đủ.
        out->throttle_duty = alt_hold_vz_cascade(st, tune, vz_target, vz_ms, dt,
                                                  false, tune->vz_ilimit,
                                                  ALT_HOLD_MIN_THROTTLE_DUTY, safe_max_duty);
        out->hold_driving = true;
    } else if (st->engaged) {
        // Mất điều kiện GIỮA CHỪNG (thường mất estimator/nghiêng quá) -> báo
        // Commander xử lý soft-fault (chuyển LANDING), KHÔNG tự hạ khẩn ở đây
        // (xem comment đầu file). Giữ throttle tay làm nền tạm trong lúc FSM
        // xử lý chuyển state ở vòng lặp kế.
        out->engage_lost = true;
        out->throttle_duty = manual_throttle_duty;
        out->hold_driving = false;
    } else {
        // HOLD nhưng CHƯA từng engage được: vẫn throttle tay để cất cánh trước.
        out->throttle_duty = manual_throttle_duty;
        out->hold_driving = false;
    }
}
