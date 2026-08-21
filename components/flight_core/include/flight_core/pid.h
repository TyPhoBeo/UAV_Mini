// PID generic dùng chung cho mọi vòng (angle roll/pitch/yaw, rate roll/pitch/
// yaw). PORT trực tiếp từ MotorOutput::pid_update() (UAV-Mini). D-on-
// measurement (chống derivative kick), anti-windup icap=ILIMIT/|ki|, LPF bậc 1
// cho D, hold_integral để freeze I (không cộng dồn, KHÔNG reset — giữ nguyên
// state.integral) khi máy trạng thái yêu cầu (vd trên đất, chưa hết spool).
#pragma once

#include "flight_core/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;
    float integrator_limit;   // trần |ki * integral| (đơn vị output)
    float output_limit;       // trần |output|
} pid_gains_t;

typedef struct {
    float integral;
    bool  has_prev;
    float prev_meas;
    float d_filt;
} pid_state_t;

static inline pid_state_t pid_state_init(void) {
    pid_state_t s = {0};
    return s;
}

// Cutoff LPF cho D-term (Hz). Copy từ TUNE_PID_D_LPF_HZ của UAV-Mini.
#define PID_D_LPF_HZ 30.0f

// pid_update() — một bước PID.
//   error        : setpoint - measurement (hoặc rate_target - rate_đo, tùy vòng)
//   measurement  : giá trị đo (dùng cho D-on-measurement, không phải D-on-error)
//   dt           : giây, được clamp nội bộ [0.001, 0.05]
//   hold_integral: true -> KHÔNG cộng dồn integral (freeze, giữ nguyên state cũ)
// Trả output đã clamp [-output_limit, +output_limit].
float pid_update(pid_state_t *state, const pid_gains_t *gains,
                  float error, float measurement, float dt, bool hold_integral);

static inline void pid_reset(pid_state_t *state) {
    *state = pid_state_init();
}

#ifdef __cplusplus
}
#endif
