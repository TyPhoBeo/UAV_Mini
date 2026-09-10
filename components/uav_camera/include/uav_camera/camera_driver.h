// camera_driver.h — OV2640 DVP qua component chinh thuc espressif/esp32-camera.
//
// Lop wrapper nay ton tai de dem duoc thong ke va de MOI fb lay ra co dung MOT
// cho tra ve (xem s_stats.outstanding). Khong goi ham nao o day tu
// stabilize_task/sensor_hub: chung o core 1, camera o core 0 prio 2.
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// SENSOR_CAMERA_ENABLED den tu main/app_config.h. PHAI tu keo vao day: file .c
// cua component chi include header nay, thieu no thi #if ra 0 im lang va ca
// component bien dich thanh rong -> "undefined reference to camera_init".
#if defined(__has_include)
#  if __has_include("app_config.h")
#    include "app_config.h"
#  endif
#endif
#ifndef SENSOR_CAMERA_ENABLED
#  define SENSOR_CAMERA_ENABLED 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Luon khai bao, ke ca khi camera tat, de telemetry khong phai #if quanh moi
// cho dung. Camera tat -> moi truong = 0/false.
typedef struct {
    bool     ready;
    uint16_t width, height;
    float    fps;            // trung binh truot ~1s
    uint32_t frames;
    uint32_t drops;          // frame bo do throttle FPS hoac client cham
    uint32_t errors;         // esp_camera_fb_get() tra NULL
    uint32_t last_frame_ms;  // dau thoi gian frame gan nhat
    uint32_t last_frame_len;
    int32_t  outstanding;    // fb dang giu; >0 keo dai = ro ri
    uint32_t free_internal;
    uint32_t free_psram;     // 0 = khong co PSRAM
} camera_stats_t;

void camera_get_stats(camera_stats_t *out);

#if SENSOR_CAMERA_ENABLED

#include "board_config.h"

// Guard nay PHAI dung truoc #include "esp_camera.h". De sau thi build chet o
// "esp_camera.h: No such file" va nguoi doc se di cai thu vien thay vi di dien
// pin — thong bao dau tien phai la thong bao dung.
#if !BOARD_CAM_PINS_CONFIGURED
#error "SENSOR_CAMERA_ENABLED=1 nhung BOARD_CAM_PINS_CONFIGURED=0 -- chua dien chan camera. Mo main/board_config.h khoi 'Camera OV2640', dien cac macro CAM_*_GPIO tu SCHEMATIC (KHONG chep pinout ESP32-CAM AiThinker), roi doi co do thanh 1. Pin sai = ghi de chan motor hoac chan INT cua IMU tren drone dang bay."
#endif

#include "esp_camera.h"

// Tach ba nguyen nhan hong ma tu ngoai nhin giong het nhau:
//   !init_ok                   -> chua init: sai nguon/SCCB, cau hinh bi tu choi
//   init_ok, pid != 0x26       -> SCCB (SIOC/SIOD) khong noi chuyen duoc
//   pid dung, !frame_ok        -> bus du lieu hong (D2..D9/PCLK/VSYNC/HREF/XCLK)
//   frame_ok, !jpeg_magic_ok   -> du 8 duong nhung SAI THU TU bit
typedef struct {
    bool     init_ok;
    uint16_t pid;          // 0x26 = OV2640
    uint16_t ver;
    uint8_t  midh, midl;   // 0x7F 0xA2 voi OmniVision
    bool     frame_ok;
    uint32_t frame_len;
    bool     jpeg_magic_ok;
    uint32_t free_internal;
} camera_selftest_t;

// Chay duoc ke ca khi camera_init() da that bai. Co lay MOT khung -> vai chuc ms.
void camera_selftest(camera_selftest_t *out);

// Dem chuyen muc tren ba duong DONG BO trong window_ms. Tach hai nguyen nhan ma
// selftest gop lam mot khi khong ra khung:
//   vsync = 0            -> cam bien KHONG phat, hoac sai chan VSYNC
//   vsync > 0, pclk = 0  -> co khung hinh nhung khong co clock diem anh
//   ca ba > 0            -> dong bo tot -> loi nam o tam duong D2..D9
// pclk chi de biet CO dao hay khong: 10MHz thi vong lay mau phan mem se aliasing,
// con so tuyet doi vo nghia.
typedef struct {
    uint32_t vsync_edges, href_edges, pclk_edges;
    uint32_t samples;
    uint32_t window_ms;
} camera_sync_probe_t;

void camera_probe_sync(camera_sync_probe_t *out, uint32_t window_ms);

// Goi MOT lan, TRUOC khi tao task stream.
esp_err_t camera_init(void);

bool camera_is_ready(void);

// NULL = chua co/loi (da dem vao stats.errors). Moi lan khac NULL PHAI co dung
// mot camera_release_frame().
camera_fb_t *camera_acquire_frame(void);

// NULL duoc phep (no-op) de nhanh loi goi duoc ma khong phai kiem tra truoc.
void camera_release_frame(camera_fb_t *fb);

// Stream bao "toi da tut lai n khung". Driver khong tu biet: no van cap frame
// binh thuong, chinh consumer moi la ben bo.
void camera_note_dropped_frames(uint32_t n);

esp_err_t camera_deinit(void);

#endif  // SENSOR_CAMERA_ENABLED

#ifdef __cplusplus
}
#endif
