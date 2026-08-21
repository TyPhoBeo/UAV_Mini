// hover_model.c — xem hover_model.h cho lý do thiết kế (đặc biệt: VÌ SAO LATCH
// MỘT LẦN chứ không bù pin liên tục).
#include "flight_core/hover_model.h"

#include <math.h>

static float clampf_local(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

float hover_model_from_voltage(float vbat_v) {
    // Chặn chia-0/âm. KHÔNG phải guard an toàn — caller kiểm valid trước.
    if (!(vbat_v > 0.1f)) return HOVER_MODEL_MIN_DUTY;

    const float duty = HOVER_MODEL_REF_DUTY *
                       powf(HOVER_MODEL_V_REF / vbat_v, HOVER_MODEL_EXP);

    // NaN/Inf (powf tràn với vbat cực nhỏ) -> rơi về MIN chứ không lọt ra
    // ngoài. isfinite() phải kiểm TRƯỚC clamp: clamp một NaN trả về NaN vì mọi
    // phép so sánh với NaN đều false.
    if (!isfinite(duty)) return HOVER_MODEL_MIN_DUTY;

    return clampf_local(duty, HOVER_MODEL_MIN_DUTY, HOVER_MODEL_MAX_DUTY);
}

int hover_model_prime_duty(float hover_duty) {
    const float p = hover_duty * TAKEOFF_PRIME_HOVER_FRAC;
    return (int)(p < 0.0f ? 0.0f : p);
}

void hover_vbat_reset(hover_vbat_ring_t *r) {
    for (int i = 0; i < HOVER_MODEL_SAMPLES; ++i) r->v[i] = 0.0f;
    r->count = 0;
    r->head = 0;
    r->last_seq = 0;
}

void hover_vbat_push(hover_vbat_ring_t *r, float v, uint32_t seq, bool valid) {
    if (!valid) return;              // xem hover_model.h: valid=false = chưa có mẫu
    if (seq == 0) return;            // sensor_health_t: 0 = chưa có mẫu nào
    if (seq == r->last_seq) return;  // cùng một mẫu đọc lại ở tick sau -> bỏ

    r->last_seq = seq;
    r->v[r->head] = v;
    r->head = (r->head + 1) % HOVER_MODEL_SAMPLES;
    if (r->count < HOVER_MODEL_SAMPLES) r->count++;
}

bool hover_vbat_median(const hover_vbat_ring_t *r, float *out_v) {
    if (r->count < HOVER_MODEL_MIN_SAMPLES) return false;

    // Insertion sort trên bản sao — HOVER_MODEL_SAMPLES nhỏ (5) nên đây là
    // cách nhanh nhất và không cấp phát gì. Ring giữ nguyên (tham số const).
    float s[HOVER_MODEL_SAMPLES];
    const int n = r->count;
    for (int i = 0; i < n; ++i) s[i] = r->v[i];

    for (int i = 1; i < n; ++i) {
        const float key = s[i];
        int j = i - 1;
        while (j >= 0 && s[j] > key) {
            s[j + 1] = s[j];
            j--;
        }
        s[j + 1] = key;
    }

    // n lẻ -> phần tử giữa; n chẵn -> trung bình 2 phần tử giữa.
    *out_v = (n % 2 == 1) ? s[n / 2] : (0.5f * (s[n / 2 - 1] + s[n / 2]));
    return true;
}
