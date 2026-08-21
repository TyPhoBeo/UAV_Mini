// Kiểu dữ liệu dùng chung toàn bộ flight_core. C thuần — không class, không
// exception, không STL. Toàn bộ struct đều POD, có thể zero-init bằng {0}.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x, y, z;
} vec3f_t;

// Quaternion body -> world, convention [w, x, y, z] (khớp Mahony gốc).
typedef struct {
    float w, x, y, z;
} quat_t;

static inline vec3f_t vec3f_zero(void) {
    vec3f_t v = {0.0f, 0.0f, 0.0f};
    return v;
}

static inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline int clampi(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

#ifdef __cplusplus
}
#endif
