// cpu_bench.h — do tai CPU tung core bang idle hook.
//
// Core 1 da co so do truc tiep (LBUSY/LDT), nhung core 0 — net, udp_rx,
// console, camera + HTTP — thi khong co gi do ca. Do dung la core vua bi chat
// them viec vao.
//
// Cach do: dem so lan FreeRTOS goi idle hook moi giay; ty le thuan voi thoi
// gian core do ranh. Moc "100% ranh" DUNG CHUNG cho ca hai core (max cua hai)
// vi chung la LX7 giong het nhau — moc rieng thi core 1 khong bao gio co mot
// giay ranh de lay moc, va tai bao thap gia (da do: 13% trong khi that la 41%).
//
// ⚠ SO XAP XI, khong phai profiler: thoi gian trong ISR bi tinh nham la "ban".
// Dung de tra loi "con bao nhieu du dia", khong dung de ket luan task nao an
// bao nhieu phan tram. Voi core 1, LBUSY moi la so do truc tiep.
//
// Chi phi: mot phep tang bien trong idle hook. Khong doi sdkconfig, khong bat
// CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS (thu do them viec vao MOI lan chuyen
// ngu canh).
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    float    load_pct[2];      // tai uoc luong tung core, 0..100
    uint32_t idle_hz[2];
    uint32_t idle_hz_ref[2];   // moc "ranh hoan toan", dung chung hai core
    bool     calibrated;       // da xong it nhat mot cua so 1s
} cpu_bench_t;

// Goi MOT lan trong app_main, TRUOC khi tao cac task nang.
esp_err_t cpu_bench_init(void);

void cpu_bench_read(cpu_bench_t *out);

// Xoa moc va do lai. Dung khi da doi tai nen (vd vua tat camera).
void cpu_bench_reset(void);
