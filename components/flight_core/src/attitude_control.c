#include "flight_core/attitude_control.h"

#include <math.h>

static float wrap_deg_180(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

attitude_gains_t attitude_default_gains(void) {
    attitude_gains_t g;
    g.angle_roll  = (pid_gains_t){ATT_GAIN_ANGLE_ROLL_KP, ATT_GAIN_ANGLE_ROLL_KI,
                                   ATT_GAIN_ANGLE_ROLL_KD, ATT_GAIN_ANGLE_ROLL_ILIMIT,
                                   ATT_GAIN_ANGLE_ROLL_OUTLIM};
    g.angle_pitch = (pid_gains_t){ATT_GAIN_ANGLE_PITCH_KP, ATT_GAIN_ANGLE_PITCH_KI,
                                   ATT_GAIN_ANGLE_PITCH_KD, ATT_GAIN_ANGLE_PITCH_ILIMIT,
                                   ATT_GAIN_ANGLE_PITCH_OUTLIM};
    g.angle_yaw   = (pid_gains_t){ATT_GAIN_ANGLE_YAW_KP, ATT_GAIN_ANGLE_YAW_KI,
                                   ATT_GAIN_ANGLE_YAW_KD, ATT_GAIN_ANGLE_YAW_ILIMIT,
                                   ATT_GAIN_ANGLE_YAW_OUTLIM};
    g.rate_roll   = (pid_gains_t){ATT_GAIN_RATE_ROLL_KP, ATT_GAIN_RATE_ROLL_KI,
                                   ATT_GAIN_RATE_ROLL_KD, ATT_GAIN_RATE_ROLL_ILIMIT,
                                   ATT_GAIN_RATE_ROLL_OUTLIM};
    g.rate_pitch  = (pid_gains_t){ATT_GAIN_RATE_PITCH_KP, ATT_GAIN_RATE_PITCH_KI,
                                   ATT_GAIN_RATE_PITCH_KD, ATT_GAIN_RATE_PITCH_ILIMIT,
                                   ATT_GAIN_RATE_PITCH_OUTLIM};
    g.rate_yaw    = (pid_gains_t){ATT_GAIN_RATE_YAW_KP, ATT_GAIN_RATE_YAW_KI,
                                   ATT_GAIN_RATE_YAW_KD, ATT_GAIN_RATE_YAW_ILIMIT,
                                   ATT_GAIN_RATE_YAW_OUTLIM};
    g.roll_sign  = ATT_MIX_ROLL_SIGN;
    g.pitch_sign = ATT_MIX_PITCH_SIGN;
    g.yaw_sign   = ATT_MIX_YAW_SIGN;
    return g;
}

// axis_hold() — conditional integration cho MỘT trục.
// Trả true (freeze I) khi: đã freeze toàn cục, HOẶC mixer đang chặn trục này
// đúng theo hướng mà sai số hiện tại muốn đẩy tới. Hướng đẩy tính TRONG KHÔNG
// GIAN CORRECTION (đã nhân sign) vì cờ limited_* cũng đo ở đó.
//
// Vẫn cho tích lũy theo hướng NGƯỢC lại — đó là điểm khác biệt so với freeze
// mù: bị chặn trần dương thì I vẫn phải giảm được để thoát bão hoà.
static bool axis_hold(bool hold_all, float sign, float error,
                       bool limited_pos, bool limited_neg) {
    if (hold_all) return true;
    const float push = sign * error;   // hướng error này đẩy correction
    if (limited_pos && push > 0.0f) return true;
    if (limited_neg && push < 0.0f) return true;
    return false;
}

void attitude_control_update(attitude_state_t *state, const attitude_gains_t *gains,
                              const attitude_input_t *in, bool hold_integral,
                              int safe_max_duty, const mixer_status_t *prev_sat,
                              attitude_output_t *out) {
    // Không có phản hồi bão hoà (tick đầu) -> coi như không bị chặn.
    static const mixer_status_t k_no_sat = {0};
    const mixer_status_t *sat = (prev_sat != NULL) ? prev_sat : &k_no_sat;

    // ================= OUTER LOOP: ANGLE -> RATE TARGET =================
    // Vòng ngoài cũng dùng cùng cờ bão hoà: cascade mà vòng trong không giao
    // được mô-men thì vòng ngoài tiếp tục dồn I cũng vô ích, chỉ tạo thêm
    // windup ở tầng thứ hai.
    const float roll_error = in->target_roll_deg - in->roll_deg;
    const float roll_rate_target = pid_update(&state->angle_roll, &gains->angle_roll,
                                               roll_error, in->roll_deg, in->dt_s,
                                               axis_hold(hold_integral, gains->roll_sign, roll_error,
                                                          sat->roll_limited_pos, sat->roll_limited_neg));

    const float pitch_error = in->target_pitch_deg - in->pitch_deg;
    const float pitch_rate_target = pid_update(&state->angle_pitch, &gains->angle_pitch,
                                                pitch_error, in->pitch_deg, in->dt_s,
                                                axis_hold(hold_integral, gains->pitch_sign, pitch_error,
                                                           sat->pitch_limited_pos, sat->pitch_limited_neg));

    // Yaw heading-hold: kp=0 mặc định -> ~0, yaw_rate_target collapses về
    // feed-forward tốc độ của phi công (rate-only). CHÚ Ý: yaw angle Kd phải
    // giữ 0 — measurement (yaw_deg) wrap ±180, đạo hàm qua điểm wrap sẽ nhảy
    // ~360/dt và đá motor.
    const float yaw_angle_error = wrap_deg_180(in->target_yaw_deg - in->yaw_deg);
    const float yaw_rate_from_angle = pid_update(&state->angle_yaw, &gains->angle_yaw,
                                                  yaw_angle_error, in->yaw_deg, in->dt_s,
                                                  axis_hold(hold_integral, gains->yaw_sign, yaw_angle_error,
                                                             sat->yaw_limited_pos, sat->yaw_limited_neg));
    const float yaw_rate_target = yaw_rate_from_angle + in->target_yaw_rate_dps;

    out->roll_rate_target = roll_rate_target;
    out->pitch_rate_target = pitch_rate_target;
    out->yaw_rate_target = yaw_rate_target;

    // ================= INNER LOOP: RATE -> DUTY CORRECTION =================
    const float roll_rate_error = roll_rate_target - in->gyro_roll_dps;
    const float pitch_rate_error = pitch_rate_target - in->gyro_pitch_dps;
    const float yaw_rate_error = yaw_rate_target - in->gyro_yaw_dps;

    const float roll_correction = gains->roll_sign * pid_update(
        &state->rate_roll, &gains->rate_roll, roll_rate_error, in->gyro_roll_dps, in->dt_s,
        axis_hold(hold_integral, gains->roll_sign, roll_rate_error,
                   sat->roll_limited_pos, sat->roll_limited_neg));
    const float pitch_correction = gains->pitch_sign * pid_update(
        &state->rate_pitch, &gains->rate_pitch, pitch_rate_error, in->gyro_pitch_dps, in->dt_s,
        axis_hold(hold_integral, gains->pitch_sign, pitch_rate_error,
                   sat->pitch_limited_pos, sat->pitch_limited_neg));
    const float yaw_correction = gains->yaw_sign * pid_update(
        &state->rate_yaw, &gains->rate_yaw, yaw_rate_error, in->gyro_yaw_dps, in->dt_s,
        axis_hold(hold_integral, gains->yaw_sign, yaw_rate_error,
                   sat->yaw_limited_pos, sat->yaw_limited_neg));

    out->roll_correction = roll_correction;
    out->pitch_correction = pitch_correction;
    out->yaw_correction = yaw_correction;

    // ================= QUAD-X MIXER =================
    // Sơ đồ (dùng block-comment, KHÔNG dùng "//" từng dòng: dòng kết thúc
    // bằng ký tự backslash trong line-comment bị GCC -Wcomment coi là nối
    // dòng, build -Werror sẽ fail):
    /*
             FRONT
          M3       M2
            x     x
             x   x
             x   x
            x     x
          M4       M1
             BACK

      M1 = back-right  (CCW)   M2 = front-right (CW)
      M3 = front-left  (CCW)   M4 = back-left   (CW)
      (chiều quay nhìn TỪ TRÊN xuống — đã xác nhận, xem app_config.h)

      Dấu yaw suy ra TỪ chiều quay, không phải quy ước tuỳ ý: định luật 3
      Newton — cánh quay CCW đẩy KHUNG theo chiều CW (= yaw ÂM theo quy ước
      Z-up bàn tay phải), cánh quay CW đẩy khung theo CCW (= yaw DƯƠNG). Vậy:
        M1, M3 quay CCW -> mang -Y
        M2, M4 quay CW  -> mang +Y
      Đây cũng đúng cặp chéo (M1,M3) và (M2,M4) như Quad-X đòi hỏi.
      +yaw_correction => quay CCW (trái), khớp CMD_MOVE (xem command.h).
    */
    // Nếu test phản ứng ngược thì đảo roll_sign/pitch_sign/yaw_sign, KHÔNG sửa gain.
    float mf[4];
    const float thr_f = (float)in->throttle_duty;
    mf[0] = thr_f - pitch_correction + roll_correction - yaw_correction;  // M1 back-right
    mf[1] = thr_f + pitch_correction + roll_correction + yaw_correction;  // M2 front-right
    mf[2] = thr_f + pitch_correction - roll_correction - yaw_correction;  // M3 front-left
    mf[3] = thr_f - pitch_correction - roll_correction + yaw_correction;  // M4 back-left

    // Xử lý saturation ở MỨC MIXER (không clamp từng motor độc lập): throttle
    // cao -> motor cần TĂNG bị chặt trần trong khi motor cần GIẢM vẫn giảm
    // bình thường -> correction hiệu dụng còn một nửa, lệch tâm, các trục dính
    // chéo nhau đúng lúc cần authority nhất. Ưu tiên attitude > throttle:
    //   1. span (max-min) vượt cả dải vật lý -> co toàn bộ correction theo tỷ
    //      lệ quanh throttle (giữ nguyên HƯỚNG mô-men).
    //   2. chỉ tràn một đầu -> dịch cả cụm 4 motor vào trong dải (đổi throttle
    //      chung một chút, giữ nguyên chênh lệch).
    // Cái phải hy sinh là throttle trung bình — thà tụt/nhích độ cao một chút
    // còn hơn mất lái.
    {
        const float lim = (float)safe_max_duty;

        float mx = mf[0], mn = mf[0];
        for (int i = 1; i < 4; i++) {
            if (mf[i] > mx) mx = mf[i];
            if (mf[i] < mn) mn = mf[i];
        }

        const float span = mx - mn;
        if (span > lim && span > 1e-3f) {
            const float scale = lim / span;
            for (int i = 0; i < 4; i++) {
                mf[i] = thr_f + (mf[i] - thr_f) * scale;
            }
            mx = thr_f + (mx - thr_f) * scale;
            mn = thr_f + (mn - thr_f) * scale;
        }

        float shift = 0.0f;
        if (mx > lim) shift = lim - mx;
        else if (mn < 0.0f) shift = -mn;

        for (int i = 0; i < 4; i++) mf[i] += shift;
    }

    out->m1 = clampi((int)mf[0], 0, safe_max_duty);
    out->m2 = clampi((int)mf[1], 0, safe_max_duty);
    out->m3 = clampi((int)mf[2], 0, safe_max_duty);
    out->m4 = clampi((int)mf[3], 0, safe_max_duty);

    // ================= PHẢN HỒI BÃO HOÀ (cho anti-windup tick sau) =================
    // NGHỊCH ĐẢO mixer từ duty CUỐI CÙNG (sau desaturation + clamp) để biết
    // correction THỰC SỰ giao được là bao nhiêu. Với Quad-X ở trên:
    //   m1 = thr - P + R - Y      m2 = thr + P + R + Y
    //   m3 = thr + P - R - Y      m4 = thr - P - R + Y
    // => R = ((m1+m2) - (m3+m4)) / 4
    //    P = ((m2+m3) - (m1+m4)) / 4
    //    Y = ((m2+m4) - (m1+m3)) / 4
    // Đây là phép giải chính xác, KHÔNG phải ước lượng.
    {
        const float a1 = (float)out->m1, a2 = (float)out->m2;
        const float a3 = (float)out->m3, a4 = (float)out->m4;

        const float roll_done  = ((a1 + a2) - (a3 + a4)) * 0.25f;
        const float pitch_done = ((a2 + a3) - (a1 + a4)) * 0.25f;
        const float yaw_done   = ((a2 + a4) - (a1 + a3)) * 0.25f;

        // Ngưỡng chết: chênh lệch dưới 1 LSB duty là làm tròn số nguyên, không
        // phải bão hoà. Không có ngưỡng này thì mọi tick đều báo "limited" và
        // I-term bị khoá vĩnh viễn.
        const float eps = 1.0f;

        out->sat.roll_limited_pos  = (roll_correction  - roll_done)  > eps;
        out->sat.roll_limited_neg  = (roll_done  - roll_correction)  > eps;
        out->sat.pitch_limited_pos = (pitch_correction - pitch_done) > eps;
        out->sat.pitch_limited_neg = (pitch_done - pitch_correction) > eps;
        out->sat.yaw_limited_pos   = (yaw_correction   - yaw_done)   > eps;
        out->sat.yaw_limited_neg   = (yaw_done   - yaw_correction)   > eps;
        out->sat.saturated =
            out->sat.roll_limited_pos  || out->sat.roll_limited_neg ||
            out->sat.pitch_limited_pos || out->sat.pitch_limited_neg ||
            out->sat.yaw_limited_pos   || out->sat.yaw_limited_neg;

        // headroom = dải duty còn thừa phía trên motor cao nhất. Đây là thước
        // đo "còn bao nhiêu thẩm quyền attitude" — dùng cho battery
        // compensation (xem flight_core.c bước 9b) và telemetry.
        float top = a1;
        if (a2 > top) top = a2;
        if (a3 > top) top = a3;
        if (a4 > top) top = a4;
        out->sat.headroom_duty = (float)safe_max_duty - top;
    }
}
