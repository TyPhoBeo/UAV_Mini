// cpu_bench.c — xem cpu_bench.h.
#include "cpu_bench.h"

// FreeRTOS.h PHAI dung truoc esp_freertos_hooks.h: header hook dung
// UBaseType_t/IRAM_ATTR do FreeRTOS + portmacro dinh nghia. Nguoc thu tu se ra
// loi "portYIELD_CORE must be defined" — mot thong bao khong lien quan gi toi
// nguyen nhan that.
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_attr.h"
#include "esp_freertos_hooks.h"
#include "esp_timer.h"

// Khong mutex: idle hook phai re nhat co the, va sai mot dem trong cua so 1
// giay khong doi duoc ket luan nao.
static volatile uint32_t s_idle_count[2];

static uint32_t s_last_count[2];
static uint32_t s_idle_hz[2];
static uint32_t s_idle_hz_ref[2];
static int64_t  s_window_us;
static bool     s_calibrated;

static bool IRAM_ATTR idle_hook_cpu0(void) { s_idle_count[0]++; return true; }
static bool IRAM_ATTR idle_hook_cpu1(void) { s_idle_count[1]++; return true; }

esp_err_t cpu_bench_init(void) {
    s_window_us = esp_timer_get_time();
    esp_err_t e0 = esp_register_freertos_idle_hook_for_cpu(idle_hook_cpu0, 0);
    esp_err_t e1 = esp_register_freertos_idle_hook_for_cpu(idle_hook_cpu1, 1);
    return (e0 != ESP_OK) ? e0 : e1;
}

// Goi tu cpu_bench_read() chu khong tu task rieng: them mot task chi de dem
// la them dung cai thu ta dang do.
static void tick_window(void) {
    const int64_t now = esp_timer_get_time();
    const int64_t dt  = now - s_window_us;
    if (dt < 1000000) return;

    for (int c = 0; c < 2; ++c) {
        const uint32_t cur = s_idle_count[c];
        const uint32_t d   = cur - s_last_count[c];   // tu quay vong dung
        s_last_count[c] = cur;
        s_idle_hz[c] = (uint32_t)((uint64_t)d * 1000000ULL / (uint64_t)dt);
        if (s_idle_hz[c] > s_idle_hz_ref[c]) s_idle_hz_ref[c] = s_idle_hz[c];
    }

    // MOC DUNG CHUNG: hai core la LX7 giong het nhau nen khi ranh that su phai
    // cho cung so dem. Moc rieng ngam gia dinh moi core tung co mot giay ranh —
    // sai voi core 1 (stabilize+sensor_hub chay TRUOC cpu_bench_init va khong
    // bao gio dung). Do that: moc core0=2423/s core1=1641/s, lech 1.48 lan.
    const uint32_t shared = (s_idle_hz_ref[0] > s_idle_hz_ref[1])
                             ? s_idle_hz_ref[0] : s_idle_hz_ref[1];
    s_idle_hz_ref[0] = shared;
    s_idle_hz_ref[1] = shared;
    s_window_us = now;
    s_calibrated = true;
}

void cpu_bench_read(cpu_bench_t *out) {
    if (out == NULL) return;
    tick_window();

    out->calibrated = s_calibrated;
    for (int c = 0; c < 2; ++c) {
        out->idle_hz[c]     = s_idle_hz[c];
        out->idle_hz_ref[c] = s_idle_hz_ref[c];
        if (s_idle_hz_ref[c] == 0) {
            out->load_pct[c] = 0.0f;      // chua co moc
        } else {
            float busy = 1.0f - (float)s_idle_hz[c] / (float)s_idle_hz_ref[c];
            if (busy < 0.0f) busy = 0.0f;
            if (busy > 1.0f) busy = 1.0f;
            out->load_pct[c] = busy * 100.0f;
        }
    }
}

void cpu_bench_reset(void) {
    for (int c = 0; c < 2; ++c) {
        s_idle_hz_ref[c] = 0;
        s_idle_hz[c] = 0;
        s_last_count[c] = s_idle_count[c];
    }
    s_window_us = esp_timer_get_time();
    s_calibrated = false;
}
