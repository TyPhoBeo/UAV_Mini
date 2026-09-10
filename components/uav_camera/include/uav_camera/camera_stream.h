// camera_stream.h — MJPEG-over-HTTP cho OV2640.
//
//   GET /stream         multipart/x-mixed-replace; boundary=frame
//   GET /snapshot       mot anh JPEG
//   GET /camera/status  JSON chan doan
//
// Cong rieng (CAMERA_HTTP_PORT), KHONG dung chung cong UDP 4210 cua telemetry:
// nghen video khong duoc lam nghen lenh bay. Server chay core 0 prio 2, duoi
// udp_rx(3) va net(5) — khong task nao o day duoc chay tren core 1.
#pragma once

#include <stdbool.h>
#include "esp_err.h"

// Xem giai thich cung quy uoc o camera_driver.h.
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

#if SENSOR_CAMERA_ENABLED

// Goi SAU camera_init() va SAU khi Wi-Fi co IP. Goi lai khi da chay = ESP_OK.
esp_err_t camera_stream_start(void);

esp_err_t camera_stream_stop(void);
bool camera_stream_is_running(void);

#endif  // SENSOR_CAMERA_ENABLED

#ifdef __cplusplus
}
#endif
