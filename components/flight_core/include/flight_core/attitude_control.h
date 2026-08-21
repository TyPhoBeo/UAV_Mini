// Cascade attitude PID (angle -> rate) roll/pitch/yaw + Quad-X mixer.
// PORT trực tiếp từ MotorOutput::update_balance_quad_x() (UAV-Mini, C++ class)
// sang C. Gain mặc định COPY Y HỆT bản đã tune — không đổi số khi port.
//
// RANH GIỚI CỐ Ý so với bản C++ gốc: module gốc gộp CHUNG PID + mixer + arm
// state + ghi PWM thật. Ở đây tách "armed" ra NGOÀI (do FlightStateMachine
// quyết định có gọi attitude_control_update() hay không — xem flight_state_
// machine.h), module này CHỈ làm toán PID+mixer thuần, không biết gì về motor
// driver / arm state. Lý do: state DISARMED/EMERGENCY-cutoff là quyết định
// cấp máy trạng thái, không phải chi tiết của vòng điều khiển.
#pragma once

#include "flight_core/pid.h"
#include "flight_core/tuning.h"
#include "flight_core/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    pid_gains_t angle_roll;
    pid_gains_t angle_pitch;
    pid_gains_t angle_yaw;    // heading-hold: kp=0 mặc định -> yaw rate-only (xem README)
    pid_gains_t rate_roll;
    pid_gains_t rate_pitch;
    pid_gains_t rate_yaw;

    // Dấu mixer — nếu lắp ngược board/motor thì đảo ở đây, KHÔNG sửa gain.
    float roll_sign;
    float pitch_sign;
    float yaw_sign;
} attitude_gains_t;

typedef struct {
    pid_state_t angle_roll;
    pid_state_t angle_pitch;
    pid_state_t angle_yaw;
    pid_state_t rate_roll;
    pid_state_t rate_pitch;
    pid_state_t rate_yaw;
} attitude_state_t;

typedef struct {
    // EKF/Mahony attitude hiện tại (deg) + gyro body-rate (dps).
    float roll_deg, pitch_deg, yaw_deg;
    float gyro_roll_dps, gyro_pitch_dps, gyro_yaw_dps;

    // Setpoint: roll/pitch = GÓC mục tiêu (deg, đã cộng trim ở tầng gọi).
    // yaw_deg dùng cho heading-hold (outer loop, kp=0 mặc định -> ~0 ảnh hưởng);
    // yaw_rate_dps = yaw-RATE mục tiêu (dps) — LUÔN là tốc độ, KHÔNG phải góc,
    // cộng thẳng vào yaw_rate_target (feed-forward phi công).
    float target_roll_deg, target_pitch_deg;
    float target_yaw_deg, target_yaw_rate_dps;

    int   throttle_duty;   // base throttle (0..safe_max_duty), từ tầng alt/manual
    float dt_s;
} attitude_input_t;

// Trạng thái bão hoà của mixer — PHẢN HỒI cho anti-windup của PID.
//
// VÌ SAO CẦN: mixer có desaturation (co correction + dịch cụm 4 motor, xem
// attitude_control.c) nên khi hết thẩm quyền nó KHÔNG clip từng motor mà giảm
// ĐỀU correction. Kết quả: mô-men thực tế nhỏ hơn mô-men PID đòi, sai số
// KHÔNG giảm, và I-term cứ thế phình lên để "cố hơn nữa" — trong khi phần
// cứng đã hết cỡ. Khi thoát bão hoà, cái I-term khổng lồ đó bung ra.
//
// Cờ *_limited_pos/neg nói: "trục này đã bị chặn theo hướng DƯƠNG/ÂM". PID
// dùng nó để NGỪNG tích lũy theo đúng hướng đang bị chặn (conditional
// integration), vẫn tích lũy bình thường theo hướng ngược lại.
//
// Đo bằng cách NGHỊCH ĐẢO mixer: từ duty cuối cùng tính ngược ra correction
// THỰC SỰ đạt được, so với correction PID ĐÒI. Chính xác tuyệt đối cho mixer
// Quad-X này (4 phương trình, 3 ẩn + throttle), không phải ước lượng.
typedef struct {
    bool saturated;            // bất kỳ trục nào bị chặn
    bool roll_limited_pos,  roll_limited_neg;
    bool pitch_limited_pos, pitch_limited_neg;
    bool yaw_limited_pos,   yaw_limited_neg;
    float headroom_duty;       // dải duty còn thừa cho attitude (safe_max - motor cao nhất)
} mixer_status_t;

typedef struct {
    int m1, m2, m3, m4;         // duty cuối, đã clamp [0, safe_max_duty]
    float roll_rate_target;
    float pitch_rate_target;
    float yaw_rate_target;
    float roll_correction, pitch_correction, yaw_correction;
    mixer_status_t sat;         // xem mixer_status_t — phản hồi cho anti-windup tick SAU
} attitude_output_t;

// GAIN MẶC ĐỊNH: xem flight_core/tuning.h (mục 1 — ATTITUDE CASCADE). Đổi số
// ở ĐÓ, không ở đây. attitude_control_update() KHÔNG tự áp ngưỡng ATT_MIN_
// THROTTLE_DUTY — tầng gọi (flight_core) phải kiểm tra throttle_duty trước
// khi gọi, giống bản gốc.

attitude_gains_t attitude_default_gains(void);

static inline void attitude_state_reset(attitude_state_t *s) {
    s->angle_roll = pid_state_init();
    s->angle_pitch = pid_state_init();
    s->angle_yaw = pid_state_init();
    s->rate_roll = pid_state_init();
    s->rate_pitch = pid_state_init();
    s->rate_yaw = pid_state_init();
}

// attitude_control_update() — một bước cascade + mixer.
//   hold_integral : freeze CẢ 6 integrator (angle+rate) — dùng khi trên đất /
//                   chưa hết spool (xem takeoff_land.h). P/D luôn chạy.
//   safe_max_duty : trần vật lý duty PWM (board-specific, từ board_config.h).
//   prev_sat      : mixer_status_t của tick TRƯỚC (out->sat lần gọi trước).
//                   Dùng cho conditional integration — xem mixer_status_t.
//                   NULL = không có anti-windup theo bão hoà (tick đầu tiên).
//                   Trễ 1 tick là chấp nhận được: bão hoà là hiện tượng kéo
//                   dài hàng chục-trăm ms, không phải sự kiện 1 tick.
void attitude_control_update(attitude_state_t *state, const attitude_gains_t *gains,
                              const attitude_input_t *in, bool hold_integral,
                              int safe_max_duty, const mixer_status_t *prev_sat,
                              attitude_output_t *out);

#ifdef __cplusplus
}
#endif
