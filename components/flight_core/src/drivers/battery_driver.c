#include "flight_core/drivers/battery_driver.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

// Toàn bộ thân driver nằm sau cờ biên dịch này. Cờ = 0 (từ
// SENSOR_BATTERY_ENABLED trong main/app_config.h, xem fc_features.h) thì file
// này biên dịch ra một object RỖNG — driver không tốn byte flash nào.
#if FC_FEATURE_BATTERY

static const char *TAG = "battery_driver";

// ADC_ATTEN_DB_12 (~0-3.3V full-scale trên ESP32-S3) — tên HIỆN HÀNH của
// ESP-IDF 5.2+ (ADC_ATTEN_DB_11 cũ đã deprecated, cùng phần cứng). Dải này
// ĐÚNG cho ứng dụng: pin 1S đầy 4.2V qua divider 3.2 -> 4.2/3.2 = 1.3125V tại
// chân ADC, nằm gọn trong dải và còn thừa biên rất lớn (không bao giờ kẹp
// trần). Hạ xuống mức atten thấp hơn sẽ cho độ phân giải tốt hơn NHƯNG trần
// tụt xuống ~1.1V (DB_2_5) -> pin đầy sẽ kẹp trần, KHÔNG được đổi.
#define BATTERY_ADC_ATTEN     ADC_ATTEN_DB_12
#define BATTERY_ADC_BITWIDTH  ADC_BITWIDTH_DEFAULT
#define BATTERY_ADC_FALLBACK_FULL_SCALE_MV  3300.0f
#define BATTERY_ADC_FALLBACK_MAX_RAW        4095.0f   // 12-bit

static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static adc_cali_handle_t         s_cali_handle = NULL;
static adc_channel_t             s_channel = 0;
static bool                      s_has_cali = false;
static bool                      s_initialized = false;

esp_err_t battery_driver_init(int adc1_channel) {
    if (s_initialized) return ESP_OK;

    adc_oneshot_unit_init_cfg_t unit_cfg = {0};
    unit_cfg.unit_id = ADC_UNIT_1;
    esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &s_adc_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_new_unit failed: %s", esp_err_to_name(err));
        return err;
    }

    s_channel = (adc_channel_t)adc1_channel;
    adc_oneshot_chan_cfg_t chan_cfg = {0};
    chan_cfg.atten = BATTERY_ADC_ATTEN;
    chan_cfg.bitwidth = BATTERY_ADC_BITWIDTH;
    err = adc_oneshot_config_channel(s_adc_handle, s_channel, &chan_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_config_channel(ch=%d) failed: %s", adc1_channel, esp_err_to_name(err));
        return err;
    }

    adc_cali_curve_fitting_config_t cali_cfg = {0};
    cali_cfg.unit_id = ADC_UNIT_1;
    cali_cfg.chan = s_channel;
    cali_cfg.atten = BATTERY_ADC_ATTEN;
    cali_cfg.bitwidth = BATTERY_ADC_BITWIDTH;
    err = adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "calibration scheme KHONG san sang (%s) -> moi mau se co "
                      "calibrated=false VA valid=false (dien ap CHI dung de debug, "
                      "KHONG dung cho failsafe/compensation)", esp_err_to_name(err));
        s_has_cali = false;
    } else {
        s_has_cali = true;
    }

    s_initialized = true;
    // Log ĐỦ để đối chiếu với schematic ngay lúc boot mà không cần đọc code:
    // kênh, atten, calibration, hệ số chia áp, và dải hợp lệ đang áp dụng.
    ESP_LOGI(TAG, "battery_driver init OK: ADC1_CH%d atten=DB_12(~0-3.3V) cali=%s "
                  "divider=%.2f (R_top=%.0f R_bottom=%.0f) valid_range=%.1f-%.1fV",
              adc1_channel, s_has_cali ? "ON" : "OFF",
              (double)BATTERY_DIVIDER_RATIO, (double)BATTERY_R_TOP_OHM,
              (double)BATTERY_R_BOTTOM_OHM,
              (double)BATTERY_MIN_VALID_V, (double)BATTERY_MAX_VALID_V);
    // TODO VERIFY HARDWARE: kênh ADC1 truyền vào đây đến từ
    // BOARD_BATTERY_ADC1_CHANNEL (board_config.h). Trên ESP32-S3 ADC1_CHn <->
    // GPIO(n+1), nên CH5 <-> GPIO6 = net VBAT_ADC_PIN của schematic. Đã đối
    // chiếu code<->schematic; CHƯA đo bằng vôn kế trên bo thật.
    return ESP_OK;
}

esp_err_t battery_driver_read(battery_sample_t *out) {
    if (!s_initialized || out == NULL) return ESP_ERR_INVALID_STATE;

    out->raw = 0;
    out->adc_mv = 0;
    out->voltage_v = 0.0f;
    out->calibrated = s_has_cali;
    out->valid = false;

    int raw = 0;
    esp_err_t err = adc_oneshot_read(s_adc_handle, s_channel, &raw);
    if (err != ESP_OK) return err;
    out->raw = raw;

    int mv;
    if (s_has_cali) {
        err = adc_cali_raw_to_voltage(s_cali_handle, raw, &mv);
        if (err != ESP_OK) return err;
    } else {
        // Fallback tuyến tính THÔ — GIỮ LẠI để debug (thấy được ADC có sống
        // hay không) nhưng KHÔNG BAO GIỜ được coi là hợp lệ: nó bỏ qua toàn
        // bộ sai số offset/gain riêng của từng chip mà eFuse calibration sinh
        // ra để bù, sai số vài % là bình thường — quá lớn cho một ngưỡng
        // failsafe pin. valid vẫn = false ở dưới vì calibrated=false.
        mv = (int)((float)raw * BATTERY_ADC_FALLBACK_FULL_SCALE_MV / BATTERY_ADC_FALLBACK_MAX_RAW);
    }
    out->adc_mv = mv;
    out->voltage_v = ((float)mv * 0.001f) * BATTERY_DIVIDER_RATIO;

    // Hợp lệ = CÓ calibration THẬT **VÀ** nằm trong dải pin 1S (xem
    // battery_driver.h). Hai điều kiện độc lập, đều bắt buộc.
    out->valid = s_has_cali &&
                 (out->voltage_v >= BATTERY_MIN_VALID_V) &&
                 (out->voltage_v <= BATTERY_MAX_VALID_V);
    return ESP_OK;
}

esp_err_t battery_driver_read_v(float *out_voltage) {
    if (out_voltage == NULL) return ESP_ERR_INVALID_STATE;

    battery_sample_t s;
    const esp_err_t err = battery_driver_read(&s);
    if (err != ESP_OK) return err;
    // KHÔNG ghi *out_voltage khi mẫu không hợp lệ — caller chỉ hỏi "điện áp"
    // nên không có chỗ nào để nhận cờ valid, cách an toàn duy nhất là không
    // đưa số rác ra ngoài (xem battery_driver.h).
    if (!s.valid) return ESP_ERR_INVALID_RESPONSE;
    *out_voltage = s.voltage_v;
    return ESP_OK;
}

#else   // !FC_FEATURE_BATTERY
// Cảm biến bị tắt lúc biên dịch. Một translation unit hoàn toàn rỗng là
// undefined behaviour theo ISO C, nên để lại đúng một khai báo vô hại.
typedef int battery_driver_disabled_at_compile_time_t;
#endif  // FC_FEATURE_BATTERY
