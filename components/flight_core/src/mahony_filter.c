#include "flight_core/mahony_filter.h"

#include <math.h>

static const float DEG_TO_RAD_F = 3.14159265358979323846f / 180.0f;
static const float RAD_TO_DEG_F = 180.0f / 3.14159265358979323846f;

static bool valid_float(float v) { return isfinite(v); }

static vec3f_t vec3_add(vec3f_t a, vec3f_t b) {
    vec3f_t r = {a.x + b.x, a.y + b.y, a.z + b.z};
    return r;
}

static float vec3_norm(vec3f_t v) {
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

static vec3f_t vec3_normalize(vec3f_t v) {
    const float n = vec3_norm(v);
    if (n <= 1e-8f || !valid_float(n)) {
        return vec3f_zero();
    }
    vec3f_t r = {v.x / n, v.y / n, v.z / n};
    return r;
}

static vec3f_t vec3_cross(vec3f_t a, vec3f_t b) {
    vec3f_t r = {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
    return r;
}

static void quat_normalize(float q[4]) {
    const float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n <= 1e-8f || !valid_float(n)) {
        q[0] = 1.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 0.0f;
        return;
    }
    q[0] /= n; q[1] /= n; q[2] /= n; q[3] /= n;
}

static void quat_mul(const float a[4], const float b[4], float out[4]) {
    const float aw = a[0], ax = a[1], ay = a[2], az = a[3];
    const float bw = b[0], bx = b[1], by = b[2], bz = b[3];
    out[0] = aw * bw - ax * bx - ay * by - az * bz;
    out[1] = aw * bx + ax * bw + ay * bz - az * by;
    out[2] = aw * by - ax * bz + ay * bw + az * bx;
    out[3] = aw * bz + ax * by - ay * bx + az * bw;
}

static void quat_conj(const float q[4], float out[4]) {
    out[0] = q[0]; out[1] = -q[1]; out[2] = -q[2]; out[3] = -q[3];
}

static vec3f_t quat_rotate_body_to_world(const float q[4], vec3f_t v) {
    float p[4] = {0.0f, v.x, v.y, v.z};
    float qc[4], tmp[4], outq[4];
    quat_conj(q, qc);
    quat_mul(q, p, tmp);
    quat_mul(tmp, qc, outq);
    vec3f_t r = {outq[1], outq[2], outq[3]};
    return r;
}

static vec3f_t quat_rotate_world_to_body(const float q[4], vec3f_t v) {
    float p[4] = {0.0f, v.x, v.y, v.z};
    float qc[4], tmp[4], outq[4];
    quat_conj(q, qc);
    quat_mul(qc, p, tmp);
    quat_mul(tmp, q, outq);
    vec3f_t r = {outq[1], outq[2], outq[3]};
    return r;
}

static void quat_from_euler(float roll, float pitch, float yaw, float q[4]) {
    const float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    const float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    const float cy = cosf(yaw * 0.5f), sy = sinf(yaw * 0.5f);
    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
    quat_normalize(q);
}

static void quat_to_euler(const float q[4], float *roll, float *pitch, float *yaw) {
    const float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    const float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    const float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    *roll = atan2f(sinr_cosp, cosr_cosp);
    const float sinp = 2.0f * (qw * qy - qz * qx);
    *pitch = asinf(clampf(sinp, -1.0f, 1.0f));
    const float siny_cosp = 2.0f * (qw * qz + qx * qy);
    const float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    *yaw = atan2f(siny_cosp, cosy_cosp);
}

static void estimate_initial_euler(vec3f_t acc_body_g, vec3f_t mag_body, bool mag_valid,
                                    float *roll, float *pitch, float *yaw) {
    const vec3f_t a = vec3_normalize(acc_body_g);
    *roll = atan2f(a.y, a.z);
    *pitch = atan2f(-a.x, sqrtf(a.y * a.y + a.z * a.z));
    *yaw = 0.0f;

    if (mag_valid && vec3_norm(mag_body) > 1e-6f) {
        const vec3f_t m = vec3_normalize(mag_body);
        const float sr = sinf(*roll), cr = cosf(*roll);
        const float sp = sinf(*pitch), cp = cosf(*pitch);
        const float mx2 = m.x * cp + m.z * sp;
        const float my2 = m.x * sr * sp + m.y * cr - m.z * sr * cp;
        *yaw = atan2f(-my2, mx2);
    }
}

mahony_config_t mahony_default_config(void) {
    mahony_config_t c;
    c.kp = MAHONY_DEFAULT_KP;
    c.ki = MAHONY_DEFAULT_KI;
    c.integral_limit_rad_s = MAHONY_DEFAULT_INTEGRAL_LIMIT_RADS;
    c.acc_min_g = MAHONY_DEFAULT_ACC_MIN_G;
    c.acc_max_g = MAHONY_DEFAULT_ACC_MAX_G;
    c.acc_error_gate = MAHONY_DEFAULT_ACC_ERROR_GATE;
    c.mag_error_gate = MAHONY_DEFAULT_MAG_ERROR_GATE;
    c.max_dt_s = MAHONY_DEFAULT_MAX_DT_S;
    c.mag_yaw_only = true;
    return c;
}

void mahony_reset(mahony_t *m) {
    m->q[0] = 1.0f; m->q[1] = 0.0f; m->q[2] = 0.0f; m->q[3] = 0.0f;
    m->integral_fb[0] = m->integral_fb[1] = m->integral_fb[2] = 0.0f;
    m->mag_ref_world[0] = 1.0f; m->mag_ref_world[1] = 0.0f; m->mag_ref_world[2] = 0.0f;
    mahony_status_t z = {0};
    m->status = z;
}

void mahony_init(mahony_t *m, const mahony_config_t *config) {
    m->config = (config != NULL) ? *config : mahony_default_config();
    mahony_reset(m);
}

void mahony_set_config(mahony_t *m, const mahony_config_t *config) {
    m->config = *config;   // KHÔNG gọi mahony_reset() — giữ nguyên q/integral_fb/mag_ref_world
}

static void set_mag_reference_from_current_q(mahony_t *m, vec3f_t mag_body_unit) {
    const vec3f_t w = quat_rotate_body_to_world(m->q, mag_body_unit);
    const vec3f_t wu = vec3_normalize(w);
    m->mag_ref_world[0] = wu.x;
    m->mag_ref_world[1] = wu.y;
    m->mag_ref_world[2] = wu.z;
    m->status.mag_reference_valid = true;
}

static void mahony_init_from_acc_mag(mahony_t *m, vec3f_t acc_body_g, vec3f_t mag_body, bool mag_valid) {
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    estimate_initial_euler(acc_body_g, mag_body, mag_valid, &roll, &pitch, &yaw);
    quat_from_euler(roll, pitch, yaw, m->q);

    if (mag_valid && vec3_norm(mag_body) > 1e-6f) {
        set_mag_reference_from_current_q(m, vec3_normalize(mag_body));
    } else {
        m->status.mag_reference_valid = false;
    }
    m->status.initialized = true;
}

static vec3f_t predict_gravity_body(const mahony_t *m) {
    vec3f_t g_world = {0.0f, 0.0f, 1.0f};
    return quat_rotate_world_to_body(m->q, g_world);
}

static vec3f_t predict_mag_body(const mahony_t *m) {
    vec3f_t w = {m->mag_ref_world[0], m->mag_ref_world[1], m->mag_ref_world[2]};
    return quat_rotate_world_to_body(m->q, w);
}

static bool compute_accel_error(mahony_t *m, vec3f_t acc_body_g, vec3f_t *error_body) {
    m->status.acc_norm = vec3_norm(acc_body_g);
    if (m->status.acc_norm < m->config.acc_min_g ||
        m->status.acc_norm > m->config.acc_max_g ||
        !valid_float(m->status.acc_norm)) {
        m->status.accel_rejected = true;
        *error_body = vec3f_zero();
        return false;
    }

    const vec3f_t z = vec3_normalize(acc_body_g);
    const vec3f_t h = predict_gravity_body(m);
    *error_body = vec3_cross(z, h);
    m->status.acc_error_norm = vec3_norm(*error_body);

    if (m->status.acc_error_norm > m->config.acc_error_gate) {
        m->status.accel_rejected = true;
        *error_body = vec3f_zero();
        return false;
    }

    m->status.accel_used = true;
    m->status.accel_rejected = false;
    return true;
}

static bool compute_mag_error(mahony_t *m, vec3f_t mag_body, vec3f_t *error_body) {
    m->status.mag_norm = vec3_norm(mag_body);
    if (m->status.mag_norm <= 1e-6f || !valid_float(m->status.mag_norm)) {
        m->status.mag_rejected = true;
        *error_body = vec3f_zero();
        return false;
    }

    const vec3f_t z = vec3_normalize(mag_body);
    if (!m->status.mag_reference_valid) {
        set_mag_reference_from_current_q(m, z);
        *error_body = vec3f_zero();
        return false;
    }

    const vec3f_t h = predict_mag_body(m);
    *error_body = vec3_cross(z, h);
    m->status.mag_error_norm = vec3_norm(*error_body);

    if (m->status.mag_error_norm > m->config.mag_error_gate) {
        m->status.mag_rejected = true;
        *error_body = vec3f_zero();
        return false;
    }

    if (m->config.mag_yaw_only) {
        vec3f_t error_world = quat_rotate_body_to_world(m->q, *error_body);
        error_world.x = 0.0f;
        error_world.y = 0.0f;
        *error_body = quat_rotate_world_to_body(m->q, error_world);
    }

    m->status.mag_used = true;
    m->status.mag_rejected = false;
    return true;
}

static void integrate_gyro_rad_s(mahony_t *m, vec3f_t gyro_body_rad_s, float dt_s) {
    const float omega_q[4] = {0.0f, gyro_body_rad_s.x, gyro_body_rad_s.y, gyro_body_rad_s.z};
    float qdot[4];
    quat_mul(m->q, omega_q, qdot);
    m->q[0] += 0.5f * qdot[0] * dt_s;
    m->q[1] += 0.5f * qdot[1] * dt_s;
    m->q[2] += 0.5f * qdot[2] * dt_s;
    m->q[3] += 0.5f * qdot[3] * dt_s;
    quat_normalize(m->q);
}

static void apply_feedback_and_integrate(mahony_t *m, vec3f_t gyro_body_rad_s, vec3f_t error_body, float dt_s) {
    vec3f_t gyro_corrected = gyro_body_rad_s;

    if (m->config.ki > 0.0f) {
        m->integral_fb[0] += m->config.ki * error_body.x * dt_s;
        m->integral_fb[1] += m->config.ki * error_body.y * dt_s;
        m->integral_fb[2] += m->config.ki * error_body.z * dt_s;

        const float lim = fabsf(m->config.integral_limit_rad_s);
        m->integral_fb[0] = clampf(m->integral_fb[0], -lim, lim);
        m->integral_fb[1] = clampf(m->integral_fb[1], -lim, lim);
        m->integral_fb[2] = clampf(m->integral_fb[2], -lim, lim);

        gyro_corrected.x += m->integral_fb[0];
        gyro_corrected.y += m->integral_fb[1];
        gyro_corrected.z += m->integral_fb[2];
    } else {
        m->integral_fb[0] = m->integral_fb[1] = m->integral_fb[2] = 0.0f;
    }

    gyro_corrected.x += m->config.kp * error_body.x;
    gyro_corrected.y += m->config.kp * error_body.y;
    gyro_corrected.z += m->config.kp * error_body.z;

    integrate_gyro_rad_s(m, gyro_corrected, dt_s);
}

void mahony_update(mahony_t *m, vec3f_t gyro_body_dps, vec3f_t acc_body_g,
                    vec3f_t mag_body, bool mag_valid, float dt_s) {
    m->status.accel_used = false;
    m->status.accel_rejected = false;
    m->status.mag_used = false;
    m->status.mag_rejected = false;
    m->status.acc_error_norm = 0.0f;
    m->status.mag_error_norm = 0.0f;
    m->status.acc_norm = vec3_norm(acc_body_g);
    m->status.mag_norm = mag_valid ? vec3_norm(mag_body) : 0.0f;

    if (!valid_float(dt_s) || dt_s <= 0.0f || dt_s > m->config.max_dt_s) {
        return;
    }

    if (!m->status.initialized) {
        // Init chỉ khi accel gần 1g để tránh khởi tạo lúc rung/gia tốc.
        const bool acc_still = m->status.acc_norm > 0.90f && m->status.acc_norm < 1.10f;
        if (acc_still) {
            mahony_init_from_acc_mag(m, acc_body_g, mag_body, mag_valid);
        }
        return;
    }

    vec3f_t error = vec3f_zero();

    vec3f_t acc_error = vec3f_zero();
    if (compute_accel_error(m, acc_body_g, &acc_error)) {
        error = vec3_add(error, acc_error);
    }

    if (mag_valid) {
        vec3f_t mag_error = vec3f_zero();
        if (compute_mag_error(m, mag_body, &mag_error)) {
            error = vec3_add(error, mag_error);
        }
    }

    vec3f_t gyro_rad_s = {
        gyro_body_dps.x * DEG_TO_RAD_F,
        gyro_body_dps.y * DEG_TO_RAD_F,
        gyro_body_dps.z * DEG_TO_RAD_F,
    };

    apply_feedback_and_integrate(m, gyro_rad_s, error, dt_s);
}

void mahony_get_euler_rad(const mahony_t *m, float *roll, float *pitch, float *yaw) {
    quat_to_euler(m->q, roll, pitch, yaw);
}

void mahony_get_euler_deg(const mahony_t *m, float *roll_deg, float *pitch_deg, float *yaw_deg) {
    float r = 0.0f, p = 0.0f, y = 0.0f;
    mahony_get_euler_rad(m, &r, &p, &y);
    *roll_deg = r * RAD_TO_DEG_F;
    *pitch_deg = p * RAD_TO_DEG_F;
    *yaw_deg = y * RAD_TO_DEG_F;
}

vec3f_t mahony_gyro_bias_dps(const mahony_t *m) {
    vec3f_t b = {
        -m->integral_fb[0] * RAD_TO_DEG_F,
        -m->integral_fb[1] * RAD_TO_DEG_F,
        -m->integral_fb[2] * RAD_TO_DEG_F,
    };
    return b;
}
