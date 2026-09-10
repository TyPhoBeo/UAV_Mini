// camera_driver.c — xem camera_driver.h.
#include "uav_camera/camera_driver.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "cam";

// Song ngoai #if: telemetry doc ke ca khi camera tat.
static camera_stats_t s_stats;
static SemaphoreHandle_t s_stats_mtx;

static void stats_lock_init(void) {
    if (s_stats_mtx == NULL) s_stats_mtx = xSemaphoreCreateMutex();
}

void camera_get_stats(camera_stats_t *out) {
    if (out == NULL) return;
    stats_lock_init();
    // Khong lay duoc mutex -> tra ban sao tho thay vi chan: so chan doan
    // 1-2Hz, lech mot frame khong sao; chan task telemetry thi co.
    if (s_stats_mtx && xSemaphoreTake(s_stats_mtx, pdMS_TO_TICKS(5)) == pdTRUE) {
        *out = s_stats;
        xSemaphoreGive(s_stats_mtx);
    } else {
        *out = s_stats;
    }
}

#if SENSOR_CAMERA_ENABLED

static bool s_ready;
// Cua so truot tinh FPS: dem frame trong moi 1s roi chot.
static int64_t  s_fps_window_us;
static uint32_t s_fps_window_frames;

static void refresh_mem_stats(void) {
    s_stats.free_internal = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
#if CONFIG_SPIRAM
    s_stats.free_psram = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
#else
    s_stats.free_psram = 0;
#endif
}

// Chan luc BIEN DICH. check_pin_conflicts() duoi day chi chay khi drone da cam
// dien — voi chan motor thi the la muon. Moi dong mot chan de trinh bien dich
// chi ra dung dong sai.
#define CAM_PIN_NOT_TAKEN(p) _Static_assert(                                   \
    (p) < 0 || ((p) != MOTOR_CH1_PIN && (p) != MOTOR_CH2_PIN &&                 \
                (p) != MOTOR_CH3_PIN && (p) != MOTOR_CH4_PIN &&                 \
                (p) != BOARD_I2C_SDA_GPIO && (p) != BOARD_I2C_SCL_GPIO &&       \
                (p) != BOARD_MPU_INT_GPIO && (p) != VBAT_ADC_PIN &&             \
                (p) != RGB_LED_PIN && (p) != 19 && (p) != 20),                  \
    "Chan camera nay trung voi mot chan DANG DUNG (motor/I2C/IMU-INT/VBAT/"     \
    "RGB/USB). Xem so dong bao loi de biet chan nao, sua main/board_config.h.")

CAM_PIN_NOT_TAKEN(CAM_D2_GPIO);
CAM_PIN_NOT_TAKEN(CAM_D3_GPIO);
CAM_PIN_NOT_TAKEN(CAM_D4_GPIO);
CAM_PIN_NOT_TAKEN(CAM_D5_GPIO);
CAM_PIN_NOT_TAKEN(CAM_D6_GPIO);
CAM_PIN_NOT_TAKEN(CAM_D7_GPIO);
CAM_PIN_NOT_TAKEN(CAM_D8_GPIO);
CAM_PIN_NOT_TAKEN(CAM_D9_GPIO);
CAM_PIN_NOT_TAKEN(CAM_PCLK_GPIO);
CAM_PIN_NOT_TAKEN(CAM_VSYNC_GPIO);
CAM_PIN_NOT_TAKEN(CAM_HREF_GPIO);
CAM_PIN_NOT_TAKEN(CAM_XCLK_GPIO);
CAM_PIN_NOT_TAKEN(CAM_SIOC_GPIO);
CAM_PIN_NOT_TAKEN(CAM_SIOD_GPIO);
CAM_PIN_NOT_TAKEN(CAM_PWDN_GPIO);
CAM_PIN_NOT_TAKEN(CAM_RESET_GPIO);

// Luoi chot cuoi truoc khi esp_camera_init() cham GPIO: bat "dien nham so",
// thu ma #error o header khong bat duoc. Bang duoi lay THANG tu board_config.h
// nen doi pin motor thi lui nay tu cap nhat.
static esp_err_t check_pin_conflicts(void) {
    typedef struct { int gpio; const char *owner; } claim_t;
    static const claim_t used[] = {
        { BOARD_I2C_SDA_GPIO, "I2C SDA"   }, { BOARD_I2C_SCL_GPIO, "I2C SCL" },
        { BOARD_MPU_INT_GPIO, "MPU INT"   },
        { FLOW_MOSI_PIN,      "flow MOSI" }, { FLOW_MISO_PIN, "flow MISO" },
        { FLOW_SCLK_PIN,      "flow SCLK" }, { FLOW_CS_PIN,   "flow CS"   },
        { MOTOR_CH1_PIN, "motor 1" }, { MOTOR_CH2_PIN, "motor 2" },
        { MOTOR_CH3_PIN, "motor 3" }, { MOTOR_CH4_PIN, "motor 4" },
        { VBAT_ADC_PIN,  "VBAT ADC" }, { RGB_LED_PIN,   "RGB LED" },
        { 19, "USB D-" }, { 20, "USB D+" },   // cham vao = mat console
    };
    typedef struct { int gpio; const char *name; } camp_t;
    const camp_t cam[] = {
        { CAM_D2_GPIO, "D2" }, { CAM_D3_GPIO, "D3" }, { CAM_D4_GPIO, "D4" },
        { CAM_D5_GPIO, "D5" }, { CAM_D6_GPIO, "D6" }, { CAM_D7_GPIO, "D7" },
        { CAM_D8_GPIO, "D8" }, { CAM_D9_GPIO, "D9" },
        { CAM_PCLK_GPIO, "PCLK" }, { CAM_VSYNC_GPIO, "VSYNC" },
        { CAM_HREF_GPIO, "HREF" }, { CAM_XCLK_GPIO, "XCLK" },
        { CAM_SIOC_GPIO, "SIOC" }, { CAM_SIOD_GPIO, "SIOD" },
        { CAM_PWDN_GPIO, "PWDN" }, { CAM_RESET_GPIO, "RESET" },
    };

    bool bad = false;
    for (size_t i = 0; i < sizeof(cam) / sizeof(cam[0]); ++i) {
        if (cam[i].gpio < 0) continue;   // -1 = khong dua ra MCU, hop le
        for (size_t j = 0; j < sizeof(used) / sizeof(used[0]); ++j) {
            if (used[j].gpio >= 0 && cam[i].gpio == used[j].gpio) {
                ESP_LOGE(TAG, "XUNG DOT CHAN: CAM_%s = GPIO%d, nhung GPIO do la %s. "
                              "Sua board_config.h.", cam[i].name, cam[i].gpio, used[j].owner);
                bad = true;
            }
        }
        // Trung nhau giua cac chan camera cung la loi dien.
        for (size_t k = i + 1; k < sizeof(cam) / sizeof(cam[0]); ++k) {
            if (cam[k].gpio == cam[i].gpio) {
                ESP_LOGE(TAG, "XUNG DOT CHAN: CAM_%s va CAM_%s cung la GPIO%d",
                         cam[i].name, cam[k].name, cam[i].gpio);
                bad = true;
            }
        }
#if CONFIG_SPIRAM_MODE_OCTAL
        if (cam[i].gpio >= 33 && cam[i].gpio <= 37) {
            ESP_LOGE(TAG, "CAM_%s = GPIO%d nam trong 33..37 — octal PSRAM dang BAT "
                          "va chiem nhom chan do.", cam[i].name, cam[i].gpio);
            bad = true;
        }
#endif
    }
    return bad ? ESP_ERR_INVALID_STATE : ESP_OK;
}

esp_err_t camera_init(void) {
    stats_lock_init();
    if (s_ready) return ESP_OK;

    // Tu choi khoi dong re hon nhieu so voi de esp_camera_init() cau hinh
    // nham mot chan dang dieu khien motor.
    const esp_err_t pin_err = check_pin_conflicts();
    if (pin_err != ESP_OK) return pin_err;

    const uint32_t heap_before = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL);

    // fb_count=2 CHI khi co PSRAM. Khong co ma xin 2 thi driver cap phat trong
    // internal RAM — dung phan RAM flight stack dang song nho. Ha ve 1 la danh
    // doi FPS lay an toan.
    bool have_psram = false;
#if CONFIG_SPIRAM
    have_psram = (heap_caps_get_total_size(MALLOC_CAP_SPIRAM) > 0);
#endif

    camera_config_t cfg = {
        .pin_pwdn     = CAM_PWDN_GPIO,
        .pin_reset    = CAM_RESET_GPIO,
        .pin_xclk     = CAM_XCLK_GPIO,
        .pin_sccb_sda = CAM_SIOD_GPIO,
        .pin_sccb_scl = CAM_SIOC_GPIO,
        // CHO DUY NHAT doi so hieu bus. Schematic danh D0..D9 (bus 10 bit);
        // che do 8 bit bo D0/D1 nen D2..D9 la bus that, con esp32-camera danh
        // so tu 0 -> LECH HAI. Viet tuong minh de doi chieu duoc voi schematic:
        // dau lech mot bit cho anh nhieu hat + sai mau chu KHONG bao loi.
        .pin_d0       = CAM_D2_GPIO,   // schematic D2
        .pin_d1       = CAM_D3_GPIO,   // schematic D3
        .pin_d2       = CAM_D4_GPIO,   // schematic D4
        .pin_d3       = CAM_D5_GPIO,   // schematic D5
        .pin_d4       = CAM_D6_GPIO,   // schematic D6
        .pin_d5       = CAM_D7_GPIO,   // schematic D7
        .pin_d6       = CAM_D8_GPIO,   // schematic D8
        .pin_d7       = CAM_D9_GPIO,   // schematic D9
        .pin_vsync    = CAM_VSYNC_GPIO,
        .pin_href     = CAM_HREF_GPIO,
        .pin_pclk     = CAM_PCLK_GPIO,

        .xclk_freq_hz = CAMERA_XCLK_HZ,
        // DA KIEM motor_driver.c:44-48 — motor giu LEDC_TIMER_0 + channel 0..3.
        // Trung timer se doi tan so PWM motor => mat luc day.
        .ledc_timer   = LEDC_TIMER_3,
        .ledc_channel = LEDC_CHANNEL_7,

        // JPEG thang tu OV2640: khong decode/encode lai tren ESP32.
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size   = FRAMESIZE_QVGA,   // 320x240
        .jpeg_quality = CAMERA_JPEG_QUALITY,
        .fb_count     = have_psram ? CAMERA_FB_COUNT_PSRAM : CAMERA_FB_COUNT_NO_PSRAM,
        .fb_location  = have_psram ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM,

        // LATEST, khong phai ALL: voi fb_count=2 driver ghi de frame cu khi
        // consumer cham -> do tre khong tang theo thoi gian.
        .grab_mode    = have_psram ? CAMERA_GRAB_LATEST : CAMERA_GRAB_WHEN_EMPTY,
    };

    const esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init THAT BAI: %s (0x%x). Kiem tra 16 chan "
                      "CAM_*_GPIO trong board_config.h va nguon 2V8/1V2 cua module.",
                 esp_err_to_name(err), err);
        return err;
    }

    // Camera gan duoi day UAV -> lat o SENSOR (mien phi) chu khong tren CPU.
    sensor_t *s = esp_camera_sensor_get();
    if (s != NULL) {
        s->set_vflip(s, 1);
        s->set_hmirror(s, 1);
    }

    s_ready = true;
    s_stats.ready  = true;
    s_stats.width  = 320;
    s_stats.height = 240;
    s_fps_window_us = esp_timer_get_time();
    s_fps_window_frames = 0;
    refresh_mem_stats();

    ESP_LOGI(TAG, "OV2640 OK: QVGA 320x240 JPEG q%d, xclk %dHz, fb_count=%d, %s",
             CAMERA_JPEG_QUALITY, CAMERA_XCLK_HZ, cfg.fb_count,
             have_psram ? "PSRAM" : "DRAM (khong co PSRAM)");
    ESP_LOGI(TAG, "heap internal: %u -> %u byte (camera an %d), PSRAM free %u",
             (unsigned)heap_before, (unsigned)s_stats.free_internal,
             (int)heap_before - (int)s_stats.free_internal,
             (unsigned)s_stats.free_psram);
    return ESP_OK;
}

bool camera_is_ready(void) { return s_ready; }

camera_fb_t *camera_acquire_frame(void) {
    if (!s_ready) return NULL;

    camera_fb_t *fb = esp_camera_fb_get();
    if (fb == NULL) {
        if (s_stats.errors < UINT32_MAX) s_stats.errors++;
        return NULL;
    }

    s_stats.outstanding++;
    s_stats.frames++;
    s_stats.last_frame_len = (uint32_t)fb->len;
    s_stats.last_frame_ms  = (uint32_t)(esp_timer_get_time() / 1000);

    // FPS chot moi 1s thay vi tinh lai moi frame.
    s_fps_window_frames++;
    const int64_t now_us = esp_timer_get_time();
    const int64_t dt_us  = now_us - s_fps_window_us;
    if (dt_us >= 1000000) {
        s_stats.fps = (float)s_fps_window_frames * 1e6f / (float)dt_us;
        s_fps_window_us = now_us;
        s_fps_window_frames = 0;
        refresh_mem_stats();
    }
    return fb;
}

void camera_release_frame(camera_fb_t *fb) {
    if (fb == NULL) return;   // no-op co chu dich: nhanh loi goi duoc thang
    esp_camera_fb_return(fb);
    if (s_stats.outstanding > 0) s_stats.outstanding--;
}

void camera_selftest(camera_selftest_t *out) {
    if (out == NULL) return;
    memset(out, 0, sizeof(*out));
    out->free_internal = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    out->init_ok = s_ready;
    if (!s_ready) return;

    sensor_t *s = esp_camera_sensor_get();
    if (s != NULL) {
        out->pid  = s->id.PID;
        out->ver  = s->id.VER;
        out->midh = s->id.MIDH;
        out->midl = s->id.MIDL;
    }

    // Qua chinh duong acquire/release: quen tra fb o day thi lenh chan doan
    // lai thanh nguyen nhan lam camera dung.
    camera_fb_t *fb = camera_acquire_frame();
    if (fb != NULL) {
        out->frame_ok  = true;
        out->frame_len = (uint32_t)fb->len;
        // Kiem MAGIC chu khong chi do dai: dau sai bit van cho ra khung "co
        // do dai" nhung noi dung rac.
        if (fb->len >= 4 && fb->buf != NULL) {
            out->jpeg_magic_ok = (fb->buf[0] == 0xFF && fb->buf[1] == 0xD8 &&
                                   fb->buf[fb->len - 2] == 0xFF &&
                                   fb->buf[fb->len - 1] == 0xD9);
        }
        camera_release_frame(fb);
    }
}

void camera_probe_sync(camera_sync_probe_t *out, uint32_t window_ms) {
    if (out == NULL) return;
    memset(out, 0, sizeof(*out));
    out->window_ms = window_ms;

    // Doc muc GPIO tren chan DVP: esp_camera_init() cau hinh chung cho ngoai vi
    // nhung duong VAO van noi, gpio_get_level() van doc duoc.
    int p_vs = gpio_get_level(CAM_VSYNC_GPIO);
    int p_hr = gpio_get_level(CAM_HREF_GPIO);
    int p_pc = gpio_get_level(CAM_PCLK_GPIO);

    // Vong CHAT, khong nhuong CPU: mot vTaskDelay giua chung se bo lo canh.
    // Chay tren core 0 (task console) nen khong cham vong bay o core 1; window
    // ngan de khong lam doi telemetry qua lau.
    const int64_t end_us = esp_timer_get_time() + (int64_t)window_ms * 1000;
    while (esp_timer_get_time() < end_us) {
        const int vs = gpio_get_level(CAM_VSYNC_GPIO);
        const int hr = gpio_get_level(CAM_HREF_GPIO);
        const int pc = gpio_get_level(CAM_PCLK_GPIO);
        if (vs != p_vs) { out->vsync_edges++; p_vs = vs; }
        if (hr != p_hr) { out->href_edges++;  p_hr = hr; }
        if (pc != p_pc) { out->pclk_edges++;  p_pc = pc; }
        out->samples++;
    }
}

void camera_note_dropped_frames(uint32_t n) {
    if (n == 0) return;
    if (UINT32_MAX - s_stats.drops < n) s_stats.drops = UINT32_MAX;
    else                                s_stats.drops += n;
}

esp_err_t camera_deinit(void) {
    if (!s_ready) return ESP_OK;
    s_ready = false;
    s_stats.ready = false;
    return esp_camera_deinit();
}

#endif  // SENSOR_CAMERA_ENABLED
