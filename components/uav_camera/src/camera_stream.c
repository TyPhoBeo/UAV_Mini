// camera_stream.c — xem camera_stream.h.
#include "uav_camera/camera_stream.h"

#if SENSOR_CAMERA_ENABLED

#include <stdio.h>
#include <string.h>

#include "uav_camera/camera_driver.h"
#include "esp_http_server.h"
#include "lwip/sockets.h"
#include "lwip/tcp.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "cam_http";

#define BOUNDARY "frame"
static const char *STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" BOUNDARY;
static httpd_handle_t s_server;

// Throttle o DAY chu khong o driver: driver chay full rate de /snapshot luon
// co anh moi, chi /stream bi gioi han.
#define FRAME_MIN_INTERVAL_US (1000000 / CAMERA_TARGET_FPS)

// ---------------------------------------------------------------- /stream
static esp_err_t stream_handler(httpd_req_t *req) {
    esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK) return res;
    // Proxy cache MJPEG se lam client ket cung o frame dau tien.
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    // TAT NAGLE. No giu cac doan nho cho ACK -> tre toi ~40ms moi khung, gan
    // nua ngan sach cua 12fps, va hien ra dung nhu "lag".
    {
        const int fd = httpd_req_to_sockfd(req);
        int one = 1;
        if (fd >= 0 && setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one)) != 0) {
            ESP_LOGW(TAG, "/stream: khong tat duoc Nagle (van chay, chi tre hon)");
        }
    }

    ESP_LOGI(TAG, "/stream: client vao");
    int64_t next_frame_us = esp_timer_get_time();

    while (true) {
        // Tre hon mot chu ky (Wi-Fi nghen / Python cham) = da lo may slot.
        // Dem vao drops roi NHAY THANG toi slot ke tiep, khong duoi kip bang
        // cach gui don — do la nghia cua "LATEST IMAGE".
        int64_t now_us = esp_timer_get_time();
        if (now_us < next_frame_us) {
            vTaskDelay(pdMS_TO_TICKS(((next_frame_us - now_us) / 1000) + 1));
            continue;
        }
        const int64_t behind_us = now_us - next_frame_us;
        if (behind_us >= FRAME_MIN_INTERVAL_US) {
            camera_note_dropped_frames((uint32_t)(behind_us / FRAME_MIN_INTERVAL_US));
        }
        next_frame_us = now_us + FRAME_MIN_INTERVAL_US;

        camera_fb_t *fb = camera_acquire_frame();
        if (fb == NULL) {
            vTaskDelay(pdMS_TO_TICKS(20));   // mot lan doc hong, khong dong ket noi
            continue;
        }

        // ⚠ TU DAY toi camera_release_frame() khong duoc co duong ra nao khac
        // (khong goto/return/break) — quen mot nhanh = ro ri fb = camera dung.

        // Nhanh nay khong bao gio dung voi cau hinh hien tai; giu de ai doi
        // pixel_format thi thay loi ro thay vi stream byte rac.
        if (fb->format != PIXFORMAT_JPEG) {
            ESP_LOGE(TAG, "/stream: frame khong phai JPEG (format=%d)", (int)fb->format);
            res = ESP_FAIL;
        } else {
            // Gop boundary + header vao MOT lan gui: moi send_chunk() la mot
            // lan ghi socket + mot chunk header rieng. Ba lan/khung -> hai.
            char part[112];
            const int part_len = snprintf(part, sizeof(part),
                                           "\r\n--" BOUNDARY "\r\n"
                                           "Content-Type: image/jpeg\r\n"
                                           "Content-Length: %u\r\n\r\n",
                                           (unsigned)fb->len);
            res = httpd_resp_send_chunk(req, part, (ssize_t)part_len);
            // Gui THANG fb->buf: khong copy, khong encode lai.
            if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)fb->buf,
                                                            (ssize_t)fb->len);
        }

        camera_release_frame(fb);   // duong tra fb DUY NHAT cua vong lap
        if (res != ESP_OK) break;   // client ngat / loi gui -> thoat
    }

    ESP_LOGI(TAG, "/stream: client ra (res=%s)", esp_err_to_name(res));
    // Chunk rong = ket thuc. Bo qua ma loi: client da ngat thi cai nay tat fail.
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

// -------------------------------------------------------------- /snapshot
static esp_err_t snapshot_handler(httpd_req_t *req) {
    camera_fb_t *fb = camera_acquire_frame();
    if (fb == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                             "camera khong tra duoc frame");
        return ESP_FAIL;
    }

    esp_err_t res = httpd_resp_set_type(req, "image/jpeg");
    if (res == ESP_OK) {
        httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=snap.jpg");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        res = httpd_resp_send(req, (const char *)fb->buf, (ssize_t)fb->len);
    }
    camera_release_frame(fb);
    return res;
}

// --------------------------------------------------------- /camera/status
static esp_err_t status_handler(httpd_req_t *req) {
    camera_stats_t st;
    camera_get_stats(&st);

    char json[320];
    const int n = snprintf(json, sizeof(json),
        "{\"ready\":%s,\"width\":%u,\"height\":%u,\"format\":\"JPEG\","
        "\"fps\":%.1f,\"frames\":%u,\"drops\":%u,\"errors\":%u,"
        "\"last_len\":%u,\"outstanding\":%d,"
        "\"free_internal\":%u,\"free_psram\":%u}",
        st.ready ? "true" : "false", (unsigned)st.width, (unsigned)st.height,
        (double)st.fps, (unsigned)st.frames, (unsigned)st.drops,
        (unsigned)st.errors, (unsigned)st.last_frame_len, (int)st.outstanding,
        (unsigned)st.free_internal, (unsigned)st.free_psram);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json, n > 0 ? (ssize_t)n : 0);
}

// ------------------------------------------------------------------ start
esp_err_t camera_stream_start(void) {
    if (s_server != NULL) return ESP_OK;

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port      = CAMERA_HTTP_PORT;
    cfg.ctrl_port        = CAMERA_HTTP_PORT + 1;
    cfg.core_id          = CAMERA_TASK_CORE;       // KHONG BAO GIO core 1
    cfg.task_priority    = CAMERA_TASK_PRIORITY;   // duoi udp_rx(3), net(5)
    cfg.stack_size       = CAMERA_TASK_STACK_BYTES;
    cfg.max_uri_handlers = 4;
    // /stream giu ket noi vo han -> bat lru_purge de client moi day duoc
    // ket noi cu ra, thay vi bi tu choi thang.
    cfg.lru_purge_enable = true;
    cfg.max_open_sockets = 3;

    esp_err_t err = httpd_start(&s_server, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start THAT BAI: %s", esp_err_to_name(err));
        s_server = NULL;
        return err;
    }

    static const httpd_uri_t uri_stream = {
        .uri = "/stream", .method = HTTP_GET, .handler = stream_handler };
    static const httpd_uri_t uri_snapshot = {
        .uri = "/snapshot", .method = HTTP_GET, .handler = snapshot_handler };
    static const httpd_uri_t uri_status = {
        .uri = "/camera/status", .method = HTTP_GET, .handler = status_handler };

    httpd_register_uri_handler(s_server, &uri_stream);
    httpd_register_uri_handler(s_server, &uri_snapshot);
    httpd_register_uri_handler(s_server, &uri_status);

    ESP_LOGI(TAG, "MJPEG server len cong %d (core %d, prio %d): "
                  "/stream /snapshot /camera/status",
             CAMERA_HTTP_PORT, CAMERA_TASK_CORE, CAMERA_TASK_PRIORITY);
    return ESP_OK;
}

esp_err_t camera_stream_stop(void) {
    if (s_server == NULL) return ESP_OK;
    const esp_err_t err = httpd_stop(s_server);
    s_server = NULL;
    return err;
}

bool camera_stream_is_running(void) { return s_server != NULL; }

#endif  // SENSOR_CAMERA_ENABLED
