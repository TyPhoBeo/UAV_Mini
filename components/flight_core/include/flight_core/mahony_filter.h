// Mahony attitude filter — PORT NGUYÊN THUẬT TOÁN từ UAV-Mini (C++ class ->
// C struct + hàm). Gain/ngưỡng mặc định copy y hệt bản đã tune (xem tuning.h).
//
// CHỈ nhận IMU (+ mag tùy chọn), KHÔNG biết ToF/altitude — giữ đúng ranh giới
// tầng như bản gốc (AltEstimator nhận quaternion qua tham số, không include
// ngược lại module này).
#pragma once

#include "flight_core/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;                    // proportional feedback (kéo roll/pitch về gravity)
    float ki;                    // integral feedback (bù gyro bias chậm)
    float integral_limit_rad_s;  // trần integral feedback (rad/s)

    float acc_min_g;             // chỉ dùng accel khi norm trong [min,max]
    float acc_max_g;
    float acc_error_gate;        // bỏ mẫu accel nếu error norm vượt (0..2)
    float mag_error_gate;

    float max_dt_s;              // chặn dt lỗi (mất mẫu, overflow timer...)
    bool  mag_yaw_only;          // true = mag chỉ sửa yaw, không kéo roll/pitch
} mahony_config_t;

typedef struct {
    bool initialized;

    bool accel_used;
    bool accel_rejected;

    bool mag_used;
    bool mag_rejected;
    bool mag_reference_valid;

    float acc_norm;
    float mag_norm;
    float acc_error_norm;
    float mag_error_norm;
} mahony_status_t;

typedef struct {
    mahony_config_t  config;
    mahony_status_t  status;

    float q[4];                  // quaternion body->world [w,x,y,z]
    float integral_fb[3];        // integral feedback (rad/s), cộng trực tiếp vào gyro
    float mag_ref_world[3];      // magnetic reference world-frame
} mahony_t;

// Gain mặc định — COPY Y HỆT giá trị đã tune trong UAV-Mini (tuning.hpp).
// Đừng đổi các số này khi port — nếu cần tune lại, tune trên phần cứng S3 mới.
#define MAHONY_DEFAULT_KP                  1.0f
#define MAHONY_DEFAULT_KI                  0.25f
#define MAHONY_DEFAULT_INTEGRAL_LIMIT_RADS 0.35f
#define MAHONY_DEFAULT_ACC_MIN_G           0.75f
#define MAHONY_DEFAULT_ACC_MAX_G           1.25f
#define MAHONY_DEFAULT_ACC_ERROR_GATE      0.80f
#define MAHONY_DEFAULT_MAG_ERROR_GATE      0.90f
#define MAHONY_DEFAULT_MAX_DT_S            0.050f

mahony_config_t mahony_default_config(void);

// reset() — về quaternion identity, xóa integral feedback + mag reference.
void mahony_reset(mahony_t *m);

// init() — nạp config mặc định + reset(). Gọi một lần lúc khởi tạo hệ thống.
void mahony_init(mahony_t *m, const mahony_config_t *config);

// set_config() — đổi config LIVE (vd Kp/Ki từ @MAH SET), KHÔNG đụng q/
// integral_fb/mag_ref_world — giữ nguyên attitude hiện tại, khác hẳn init()
// (reset). Dùng khi tune Mahony trong lúc đang bay/test trên bàn.
void mahony_set_config(mahony_t *m, const mahony_config_t *config);

// update() — một bước lọc. gyro_body_dps/acc_body_g đã qua calib (bias/scale/
// remap trục) ở driver; mag_valid=false nếu không có/lỗi từ kế.
//   - Lần gọi ĐẦU khi initialized=false: chỉ init tư thế ban đầu từ accel/mag
//     (yêu cầu accel gần 1g, tránh khởi tạo lúc rung/gia tốc), KHÔNG tích phân.
//   - Các lần sau: accel + mag (nếu có) làm sai số hiệu chỉnh Mahony, tích phân
//     gyro đã hiệu chỉnh vào quaternion.
void mahony_update(mahony_t *m,
                    vec3f_t gyro_body_dps,
                    vec3f_t acc_body_g,
                    vec3f_t mag_body,
                    bool mag_valid,
                    float dt_s);

void mahony_get_euler_rad(const mahony_t *m, float *roll, float *pitch, float *yaw);
void mahony_get_euler_deg(const mahony_t *m, float *roll_deg, float *pitch_deg, float *yaw_deg);

static inline quat_t mahony_quaternion(const mahony_t *m) {
    quat_t q = {m->q[0], m->q[1], m->q[2], m->q[3]};
    return q;
}

// Bias gyro ước lượng hiện tại (dps) — dấu ngược integral_fb (feedback cộng
// vào gyro để triệt bias, nên bias tương đương = -integral_fb).
// LƯU Ý: trục Z (yaw) chỉ observable nếu mag_valid=true (dùng từ kế). Không có
// từ kế + không heading-hold -> bias_z KHÔNG bao giờ được sửa -> yaw trôi tự do
// dù setpoint yaw-rate = 0. Đây là hạn chế vật lý, không phải bug filter.
vec3f_t mahony_gyro_bias_dps(const mahony_t *m);

#ifdef __cplusplus
}
#endif
