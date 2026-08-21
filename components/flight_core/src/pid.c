#include "flight_core/pid.h"

#include <math.h>

static float clamp_dt(float dt) {
    if (dt < 0.001f) return 0.001f;
    if (dt > 0.05f)  return 0.05f;
    return dt;
}

float pid_update(pid_state_t *state, const pid_gains_t *gains,
                  float error, float measurement, float dt, bool hold_integral) {
    dt = clamp_dt(dt);

    // ----- I -----
    if (gains->ki != 0.0f && gains->integrator_limit > 0.0f) {
        // hold_integral (freeze): KHÔNG cộng dồn, giữ nguyên state->integral để
        // tránh windup khi drone bị giữ cứng (còn trên đất / chưa hết spool).
        // P/D vẫn chạy bình thường.
        if (!hold_integral) {
            state->integral += error * dt;

            const float integral_cap = gains->integrator_limit / fabsf(gains->ki);
            if (state->integral > integral_cap) state->integral = integral_cap;
            if (state->integral < -integral_cap) state->integral = -integral_cap;
        }
    } else {
        state->integral = 0.0f;
    }

    // ----- D (on measurement, có LPF) -----
    float d_raw = 0.0f;
    if (state->has_prev) {
        d_raw = -(measurement - state->prev_meas) / dt;
    } else {
        state->has_prev = true;
        state->d_filt = 0.0f;
    }
    state->prev_meas = measurement;

    const float d_rc = 1.0f / (2.0f * 3.14159265f * PID_D_LPF_HZ);
    const float d_alpha = dt / (dt + d_rc);
    state->d_filt += d_alpha * (d_raw - state->d_filt);

    float output = gains->kp * error + gains->ki * state->integral + gains->kd * state->d_filt;

    if (output > gains->output_limit) output = gains->output_limit;
    if (output < -gains->output_limit) output = -gains->output_limit;

    return output;
}
