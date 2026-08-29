#include "flight_core/drivers/motor_driver.h"

#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "flight_core/types.h"
#include "flight_core/fc_features.h"   // FC_FEATURE_BENCH_MODE

static const char *TAG = "motor_driver";

// ============================================================================
// s_mux — KHOÁ ĐA LÕI cho cặp (armed gate, duty ghi xuống LEDC)
// ============================================================================
// KILL đến từ CORE 0 (net_task/console gọi flight_core_kill_now() ->
// motor_driver_all_off()), còn duty được ghi từ CORE 1 (stabilize_task ->
// motor_driver_set_duties()). Hai lõi chạy THẬT SỰ SONG SONG, nên không có
// khoá thì đây là dãy hợp lệ:
//
//   CORE 1: set_duties(400,400,400,400)
//             đọc s_armed -> true, đi tiếp
//             ghi CH0 = 400
//                              CORE 0: all_off()
//                                        ghi CH0..CH3 = 0
//                                        s_armed = false
//             ghi CH1..CH3 = 400      <- GHI ĐÈ LÊN KẾT QUẢ CỦA KILL
//
// Kết cục: KILL "thành công" (mọi cờ đều nói đã cắt) nhưng 3 motor vẫn quay ở
// duty cũ tới lần ghi kế — và nếu stabilize_task treo ngay sau đó thì là VÔ
// THỜI HẠN. Kill latch ở tầng logic KHÔNG cứu được: nó chỉ chặn tick SAU, còn
// tick đang chạy dở đã qua điểm kiểm tra latch rồi.
//
// portMUX_TYPE là spinlock ĐA LÕI (khác mutex FreeRTOS vốn chỉ điều phối task):
// nó chặn cả lõi kia. Vùng găng cực ngắn (4 cặp ghi thanh ghi LEDC, ~vài µs) và
// KHÔNG chứa lệnh nào có thể block — bắt buộc, vì trong vùng găng portMUX ngắt
// đang bị tắt trên lõi hiện tại.
//
// KHÔNG dùng mutex ở đây: đường KILL phải chạy được kể cả khi task đang giữ
// khoá bị treo, và mutex có thể block/đảo ưu tiên. Spinlock luôn được nhả sau
// vài µs vì vùng găng không block.
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static const ledc_mode_t LEDC_MODE_USED = LEDC_LOW_SPEED_MODE;
static const ledc_timer_t LEDC_TIMER_USED = LEDC_TIMER_0;
static const ledc_timer_bit_t LEDC_RES_USED = LEDC_TIMER_11_BIT;

static const ledc_channel_t LEDC_CHANNELS[4] = {
    LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3,
};

static bool s_initialized = false;
// volatile: đọc/ghi từ CẢ HAI lõi (KILL trên core 0, vòng bay trên core 1).
// Mọi truy cập nằm trong vùng găng s_mux — volatile chỉ để chặn compiler cache
// giá trị này trong thanh ghi qua các lần vào/ra vùng găng.
static volatile bool s_armed = false;
static int  s_last_duty[4] = {0, 0, 0, 0};

static esp_err_t configure_channel(int gpio_num, int channel_index) {
    ledc_channel_config_t ch = {0};
    ch.gpio_num = gpio_num;
    ch.speed_mode = LEDC_MODE_USED;
    ch.channel = LEDC_CHANNELS[channel_index];
    ch.timer_sel = LEDC_TIMER_USED;
    ch.duty = 0;
    ch.hpoint = 0;

    esp_err_t err = ledc_channel_config(&ch);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config GPIO%d CH%d failed: %s",
                  gpio_num, channel_index, esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "LEDC channel OK: GPIO%d CH%d", gpio_num, channel_index);
    }
    return err;
}

// write_channel() — ghi 1 kênh. GỌI TRONG vùng găng s_mux (xem write_all_locked).
// Chỉ ghi thanh ghi LEDC, KHÔNG block: ledc_set_duty()/ledc_update_duty() ở
// LOW_SPEED mode là ghi thanh ghi thuần (khác ledc_set_duty_and_update() vốn
// dùng ngắt + semaphore — TUYỆT ĐỐI không được dùng ở đây).
static esp_err_t write_channel(int channel_index, int duty) {
    duty = clampi(duty, 0, MOTOR_PWM_MAX_DUTY);
    esp_err_t err = ledc_set_duty(LEDC_MODE_USED, LEDC_CHANNELS[channel_index], (uint32_t)duty);
    if (err != ESP_OK) return err;
    return ledc_update_duty(LEDC_MODE_USED, LEDC_CHANNELS[channel_index]);
}

// stop_all_locked() — cắt 4 kênh + đặt armed gate NGUYÊN TỬ trong CÙNG một
// vùng găng. Tách gate ra khỏi duty (dù chỉ hai lệnh liền nhau) là mở lại đúng
// khe mà lõi kia lọt vào được.
//   armed_after: ARMED_KEEP giữ nguyên gate, ARMED_OFF hạ, ARMED_ON mở.
#define ARMED_KEEP  (-1)
#define ARMED_OFF   0
#define ARMED_ON    1
static void stop_all_locked(int armed_after) {
    portENTER_CRITICAL(&s_mux);
    if (armed_after != ARMED_KEEP) s_armed = (armed_after == ARMED_ON);
    for (int i = 0; i < 4; i++) {
        write_channel(i, 0);
        s_last_duty[i] = 0;
    }
    portEXIT_CRITICAL(&s_mux);
}

esp_err_t motor_driver_init(const int motor_gpio[4]) {
    if (s_initialized) return ESP_OK;

    ledc_timer_config_t timer_cfg = {0};
    timer_cfg.speed_mode = LEDC_MODE_USED;
    timer_cfg.timer_num = LEDC_TIMER_USED;
    timer_cfg.duty_resolution = LEDC_RES_USED;
    timer_cfg.freq_hz = MOTOR_PWM_FREQ_HZ;
    timer_cfg.clk_cfg = LEDC_AUTO_CLK;

    esp_err_t err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %s", esp_err_to_name(err));
        return err;
    }

    for (int i = 0; i < 4; i++) {
        err = configure_channel(motor_gpio[i], i);
        if (err != ESP_OK) return err;
    }

    s_initialized = true;
    for (int i = 0; i < 4; i++) {
        write_channel(i, 0);
        s_last_duty[i] = 0;
    }

    ESP_LOGI(TAG, "Motor driver init OK (all motors at 0).");
    return ESP_OK;
}

void motor_driver_arm(void) {
    if (!s_initialized) return;
    // Ghi 0 XUỐNG PHẦN CỨNG rồi mới mở gate, TRONG CÙNG một vùng găng: mở gate
    // ở một vùng găng riêng sẽ để lõi kia lọt vào giữa và ghi duty cũ còn treo
    // trong logic của nó.
    stop_all_locked(ARMED_ON);
}

void motor_driver_disarm(void) {
    if (!s_initialized) return;
    stop_all_locked(ARMED_OFF);
}

bool motor_driver_is_armed(void) { return s_armed; }

void motor_driver_set_armed(bool armed) {
    if (!s_initialized) return;
    if (!armed) {
        // Đóng gate thì cắt NGAY, không đợi tick sau — nếu chỉ hạ cờ mà không
        // ghi 0, motor giữ nguyên duty cuối cùng cho tới lần set_duties() kế.
        // Hạ cờ + ghi 0 NGUYÊN TỬ (stop_all_locked) chứ không phải hai bước.
        stop_all_locked(ARMED_OFF);
        return;
    }
    // Mở gate KHÔNG ghi duty: duty hiện tại đã là 0 (mọi đường vào state này
    // đều đi qua stop_all_locked trước), và lần set_duties() kế sẽ ghi giá trị
    // thật. Ghi 1 biến 32-bit trong vùng găng là đủ.
    portENTER_CRITICAL(&s_mux);
    s_armed = true;
    portEXIT_CRITICAL(&s_mux);
}

// motor_driver_all_off() — ĐƯỜNG KILL. Gọi được từ BẤT KỲ lõi/task nào, kể cả
// khi core 1 đang ở giữa motor_driver_set_duties(). Vùng găng s_mux đảm bảo hai
// hàm này không bao giờ đan xen: hoặc set_duties() ghi xong rồi all_off() cắt
// (kết quả: TẮT), hoặc all_off() cắt xong rồi set_duties() thấy s_armed=false
// và tự ép 0 (kết quả: TẮT). Không còn dãy nào để lại motor đang quay.
void motor_driver_all_off(void) {
    if (!s_initialized) return;
    stop_all_locked(ARMED_OFF);
}

void motor_driver_set_duties(int m1, int m2, int m3, int m4) {
    if (!s_initialized) return;

    int d[4] = {m1, m2, m3, m4};

    // ---- HARDWARE ARMED GATE (xem motor_driver.h) ----
    // Gate đóng -> ÉP 0, KHÔNG trả về sớm: vẫn phải GHI 0 xuống LEDC. Trả về
    // sớm sẽ để motor giữ nguyên duty của lần ghi TRƯỚC (LEDC là thanh ghi
    // giữ giá trị, không tự về 0) — đúng kiểu lỗi mà gate này sinh ra để chặn.
    //
    // ĐỌC GATE VÀ GHI DUTY TRONG CÙNG MỘT VÙNG GĂNG. Tách ra ("đọc gate, ra
    // khỏi khoá, rồi ghi") là đúng cái race đã sửa: KILL từ lõi kia lọt vào
    // khe giữa hai bước và bị chính lần ghi này xoá sạch.
    // ---- BENCH MODE (app_config.h BENCH_MODE_ENABLED) ----
    // Chan o DAY, lop thap nhat, cung cho voi armed gate: khong co duong nao
    // trong firmware vong qua duoc. Van GHI 0 xuong LEDC (khong return som) —
    // cung ly do voi armed gate ngay duoi.
#if FC_FEATURE_BENCH_MODE
    const bool bench_block = true;
#else
    const bool bench_block = false;
#endif

    portENTER_CRITICAL(&s_mux);
    const bool armed = s_armed && !bench_block;
    for (int i = 0; i < 4; i++) {
        const int duty = armed ? d[i] : 0;
        write_channel(i, duty);
        s_last_duty[i] = clampi(duty, 0, MOTOR_PWM_MAX_DUTY);
    }
    portEXIT_CRITICAL(&s_mux);
}

void motor_driver_stop_all(void) {
    if (!s_initialized) return;
    // KHÔNG đụng armed gate — đây là "về 0", không phải "disarm". Đường đổi
    // gate là arm/disarm/set_armed/all_off.
    stop_all_locked(ARMED_KEEP);
}

int motor_driver_get_last_duty(int motor_id) {
    if (motor_id < 0 || motor_id > 3) return 0;
    return s_last_duty[motor_id];
}
