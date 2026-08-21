#include "flight_core/drivers/tof_driver.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Toàn bộ thân driver nằm sau cờ biên dịch này. Cờ = 0 (từ
// SENSOR_TOF_ENABLED trong main/app_config.h, xem fc_features.h) thì file
// này biên dịch ra một object RỖNG — driver không tốn byte flash nào.
#if FC_FEATURE_TOF

static const char *TAG = "tof_driver";

// VL53L0X default 7-bit I2C address.
#define VL53L0X_DEFAULT_I2C_ADDR             0x29
// Datasheet ST VL53L0X, bảng "Power-up sequence": t_boot = 1.2ms MAX (thời gian
// firmware trong chip boot xong sau khi XSHUT được nhả).
//
// Giá trị cũ ở đây là 2 và nó VI PHẠM spec trong thực tế, không phải trên lý
// thuyết: vTaskDelay(pdMS_TO_TICKS(2)) với tick 1000Hz chỉ bảo đảm "ít nhất 1
// tick trọn vẹn" — tick đầu tiên bị cắt cụt theo pha, nên thời gian thật có thể
// chỉ ~1.0ms, tức NGẮN HƠN t_boot max. Kết quả là hỏng KHÔNG ĐỀU: có lần boot
// kịp, có lần không, và lần không kịp thì đọc MODEL_ID lỗi -> driver_ok=0.
// 10ms là con số mọi thư viện có uy tín (Pololu, Adafruit) đang dùng; 8ms thừa
// ra chỉ tốn một lần duy nhất lúc boot.
#define VL53L0X_BOOT_DELAY_MS                10
// Giữ XSHUT ở LOW bao lâu trước khi nhả. Datasheet không cho số tối thiểu, nên
// dùng cùng bậc với t_boot thay vì đoán nhỏ hơn — đây là đường boot 1 lần,
// không phải vòng nóng.
#define VL53L0X_XSHUT_HOLD_MS                10
// Dò địa chỉ: thử lại vì chip có thể chưa boot xong đúng lúc probe đầu tiên,
// và vì bus có thể đang bận với cảm biến khác.
#define VL53L0X_PROBE_ATTEMPTS               5
#define VL53L0X_PROBE_RETRY_MS               10
#define VL53L0X_MODEL_ID_VALUE               0xEE
// Timeout I2C: INIT rộng (chạy 1 lần lúc boot: probe địa chỉ, ~90 lần ghi cấu
// hình), RUNTIME hẹp (tof_driver_read() trong sensor_hub). Lý do đầy đủ: xem
// khối cùng tên ở đầu imu_driver.c. s_io_timeout_ms đổi ở CUỐI tof_driver_init().
#define VL53L0X_IO_TIMEOUT_MS                50
#define VL53L0X_IO_RUNTIME_TIMEOUT_MS        8
#define VL53L0X_INIT_TIMEOUT_MS              500
#define VL53L0X_DEFAULT_TIMING_BUDGET_US     33000U

// -----------------------------------------------------------------------------
// VL53L0X register map: 8-bit register addresses.
// -----------------------------------------------------------------------------
#define VL53L0X_REG_SYSRANGE_START                               0x00
#define VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG                       0x01
#define VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO                 0x0A
#define VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR                       0x0B
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS                      0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS                          0x14
#define VL53L0X_REG_MSRC_CONFIG_CONTROL                          0x60
#define VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT  0x44
#define VL53L0X_REG_MSRC_CONFIG_TIMEOUT_MACROP                   0x46
#define VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD                0x50
#define VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI           0x51
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD              0x70
#define VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI         0x71
#define VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH                      0x84
#define VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS                     0x8A
#define VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV             0x89
#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0             0xB0
#define VL53L0X_REG_GLOBAL_CONFIG_REF_EN_START_SELECT            0xB6
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID                      0xC0
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID                   0xC2

// Back-to-back continuous ranging.
#define VL53L0X_MODE_BACKTOBACK                                  0x02

// Driver bus frequency supplied by board/application.
static uint32_t s_scl_speed_hz = 100000;

// Ngưỡng watchdog: bao lâu KHÔNG có mẫu MỚI thì coi là chip đã treo.
// Timing budget 33ms -> ~30Hz. 300ms = 9 chu kỳ đo bị mất liên tiếp: đủ rộng để
// một vài lần trễ/nhiễu lẻ tẻ không kích nhầm, đủ hẹp để không kéo dài tình
// trạng "không có độ cao" quá lâu trong lúc bay.
//
// KHÁC TOF_STALE_TIMEOUT_MS (200ms): cái đó nói "số đo đã cũ, đừng dùng cho
// điều khiển" (an toàn cho consumer). Cái này nói "phần cứng đã ngừng đo, phải
// đá nó dậy" (phục hồi). Hai câu hỏi khác nhau, hai ngưỡng riêng.
#define TOF_STALL_RESTART_MS   300

typedef struct {
    i2c_master_dev_handle_t dev;    // NULL = slot chưa dùng
    uint8_t addr;                   // 7-bit I2C address
    uint8_t stop_variable;          // private VL53L0X state, must be per sensor
    bool ranging_started;
    int64_t last_good_us;
    // Lần cuối chip bật cờ ngắt = lần cuối nó CÓ ĐO. Tách hẳn khỏi
    // last_good_us (lần cuối có số đo HỢP LỆ): ngoài tầm/bề mặt hấp thụ là
    // range_status != 0 nhưng chip VẪN SỐNG và vẫn đo. Gộp hai cái làm một sẽ
    // khiến watchdog restart chip mỗi lần drone bay qua chỗ không có gì để đo —
    // biến một tình huống bình thường thành một chuỗi reset vô nghĩa.
    int64_t last_sample_us;
    tof_reading_t last;
} tof_sensor_state_t;

// Số lần watchdog phải khởi động lại ranging (xem tof_driver.h).
static uint32_t s_stall_restarts = 0;

// MỘT sensor duy nhất (hướng xuống, nguồn correction cho alt_estimator).
// Bản trước là s_sensors[2] cho cấu hình 2 chip — xem ghi chú "ĐÃ ĐƠN GIẢN HOÁ"
// ở tof_driver_init() để biết vì sao đường thứ hai bị xoá.
static tof_sensor_state_t s_sensor;

// -----------------------------------------------------------------------------
// Low-level I2C helpers — VL53L0X uses ONE-BYTE register addresses.
// -----------------------------------------------------------------------------
// Timeout dùng cho MỌI transaction bên dưới. Bắt đầu ở mức INIT, hạ xuống
// RUNTIME khi tof_driver_init() thành công (xem cuối hàm đó).
static int s_io_timeout_ms = VL53L0X_IO_TIMEOUT_MS;

static esp_err_t write_reg8(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(dev, buf, sizeof(buf), s_io_timeout_ms);
}

static esp_err_t write_reg16(i2c_master_dev_handle_t dev, uint8_t reg, uint16_t val) {
    uint8_t buf[3] = {
        reg,
        (uint8_t)(val >> 8),
        (uint8_t)(val & 0xFF),
    };
    return i2c_master_transmit(dev, buf, sizeof(buf), s_io_timeout_ms);
}

static esp_err_t read_reg8(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *val) {
    return i2c_master_transmit_receive(dev, &reg, 1, val, 1, s_io_timeout_ms);
}

static esp_err_t read_reg16(i2c_master_dev_handle_t dev, uint8_t reg, uint16_t *val) {
    uint8_t data[2] = {0};
    esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1, data, sizeof(data),
                                                s_io_timeout_ms);
    if (err != ESP_OK) return err;
    *val = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    return ESP_OK;
}

static esp_err_t read_multi(i2c_master_dev_handle_t dev, uint8_t reg,
                            uint8_t *buf, size_t len) {
    return i2c_master_transmit_receive(dev, &reg, 1, buf, len,
                                       s_io_timeout_ms);
}

static esp_err_t write_multi(i2c_master_dev_handle_t dev, uint8_t reg,
                             const uint8_t *data, size_t len) {
    if (len > 16) return ESP_ERR_INVALID_SIZE;
    uint8_t buf[17];
    buf[0] = reg;
    memcpy(&buf[1], data, len);
    return i2c_master_transmit(dev, buf, len + 1, s_io_timeout_ms);
}

// -----------------------------------------------------------------------------
// VL53L0X timing-budget helpers, adapted from ST API / Pololu implementation.
// -----------------------------------------------------------------------------
typedef struct {
    bool tcc;
    bool dss;
    bool msrc;
    bool pre_range;
    bool final_range;
} sequence_step_enables_t;

typedef struct {
    uint8_t pre_range_vcsel_period_pclks;
    uint8_t final_range_vcsel_period_pclks;
    uint32_t msrc_dss_tcc_mclks;
    uint32_t msrc_dss_tcc_us;
    uint32_t pre_range_mclks;
    uint32_t pre_range_us;
    uint32_t final_range_mclks;
    uint32_t final_range_us;
} sequence_step_timeouts_t;

static uint8_t decode_vcsel_period(uint8_t reg_val) {
    return (uint8_t)((reg_val + 1U) << 1U);
}

static uint32_t calc_macro_period_ns(uint8_t vcsel_period_pclks) {
    return (((uint32_t)2304U * vcsel_period_pclks * 1655U) + 500U) / 1000U;
}

static uint32_t decode_timeout(uint16_t reg_val) {
    return ((uint32_t)(reg_val & 0x00FFU) << (uint32_t)((reg_val & 0xFF00U) >> 8U)) + 1U;
}

static uint16_t encode_timeout(uint32_t timeout_mclks) {
    if (timeout_mclks == 0) return 0;

    uint32_t ls_byte = timeout_mclks - 1U;
    uint16_t ms_byte = 0;
    while ((ls_byte & 0xFFFFFF00U) != 0U) {
        ls_byte >>= 1U;
        ms_byte++;
    }
    return (uint16_t)((ms_byte << 8U) | (ls_byte & 0xFFU));
}

static uint32_t timeout_mclks_to_us(uint32_t timeout_mclks,
                                    uint8_t vcsel_period_pclks) {
    uint32_t macro_period_ns = calc_macro_period_ns(vcsel_period_pclks);
    return ((timeout_mclks * macro_period_ns) + 500U) / 1000U;
}

static uint32_t timeout_us_to_mclks(uint32_t timeout_us,
                                    uint8_t vcsel_period_pclks) {
    uint32_t macro_period_ns = calc_macro_period_ns(vcsel_period_pclks);
    return (((timeout_us * 1000U) + (macro_period_ns / 2U)) / macro_period_ns);
}

static esp_err_t get_sequence_step_enables(i2c_master_dev_handle_t dev,
                                            sequence_step_enables_t *en) {
    uint8_t cfg = 0;
    esp_err_t err = read_reg8(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, &cfg);
    if (err != ESP_OK) return err;

    en->tcc         = ((cfg >> 4) & 0x01) != 0;
    en->dss         = ((cfg >> 3) & 0x01) != 0;
    en->msrc        = ((cfg >> 2) & 0x01) != 0;
    en->pre_range   = ((cfg >> 6) & 0x01) != 0;
    en->final_range = ((cfg >> 7) & 0x01) != 0;
    return ESP_OK;
}

static esp_err_t get_sequence_step_timeouts(i2c_master_dev_handle_t dev,
                                             const sequence_step_enables_t *en,
                                             sequence_step_timeouts_t *to) {
    uint8_t u8 = 0;
    uint16_t u16 = 0;
    esp_err_t err;

    err = read_reg8(dev, VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD, &u8);
    if (err != ESP_OK) return err;
    to->pre_range_vcsel_period_pclks = decode_vcsel_period(u8);

    err = read_reg8(dev, VL53L0X_REG_MSRC_CONFIG_TIMEOUT_MACROP, &u8);
    if (err != ESP_OK) return err;
    to->msrc_dss_tcc_mclks = (uint32_t)u8 + 1U;
    to->msrc_dss_tcc_us = timeout_mclks_to_us(
        to->msrc_dss_tcc_mclks, to->pre_range_vcsel_period_pclks);

    err = read_reg16(dev, VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, &u16);
    if (err != ESP_OK) return err;
    to->pre_range_mclks = decode_timeout(u16);
    to->pre_range_us = timeout_mclks_to_us(
        to->pre_range_mclks, to->pre_range_vcsel_period_pclks);

    err = read_reg8(dev, VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD, &u8);
    if (err != ESP_OK) return err;
    to->final_range_vcsel_period_pclks = decode_vcsel_period(u8);

    err = read_reg16(dev, VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, &u16);
    if (err != ESP_OK) return err;
    to->final_range_mclks = decode_timeout(u16);
    if (en->pre_range && to->final_range_mclks >= to->pre_range_mclks) {
        to->final_range_mclks -= to->pre_range_mclks;
    }
    to->final_range_us = timeout_mclks_to_us(
        to->final_range_mclks, to->final_range_vcsel_period_pclks);

    return ESP_OK;
}

static esp_err_t set_measurement_timing_budget(i2c_master_dev_handle_t dev,
                                                uint32_t budget_us) {
    const uint32_t StartOverhead = 1910;
    const uint32_t EndOverhead = 960;
    const uint32_t MsrcOverhead = 660;
    const uint32_t TccOverhead = 590;
    const uint32_t DssOverhead = 690;
    const uint32_t PreRangeOverhead = 660;
    const uint32_t FinalRangeOverhead = 550;
    const uint32_t MinTimingBudget = 20000;

    if (budget_us < MinTimingBudget) return ESP_ERR_INVALID_ARG;

    sequence_step_enables_t en = {0};
    sequence_step_timeouts_t to = {0};
    esp_err_t err = get_sequence_step_enables(dev, &en);
    if (err != ESP_OK) return err;
    err = get_sequence_step_timeouts(dev, &en, &to);
    if (err != ESP_OK) return err;

    uint32_t used_budget_us = StartOverhead + EndOverhead;
    if (en.tcc) {
        used_budget_us += to.msrc_dss_tcc_us + TccOverhead;
    }
    if (en.dss) {
        used_budget_us += 2U * (to.msrc_dss_tcc_us + DssOverhead);
    } else if (en.msrc) {
        used_budget_us += to.msrc_dss_tcc_us + MsrcOverhead;
    }
    if (en.pre_range) {
        used_budget_us += to.pre_range_us + PreRangeOverhead;
    }
    if (!en.final_range) return ESP_ERR_INVALID_STATE;

    used_budget_us += FinalRangeOverhead;
    if (used_budget_us > budget_us) return ESP_ERR_INVALID_ARG;

    uint32_t final_range_timeout_us = budget_us - used_budget_us;
    uint32_t final_range_timeout_mclks = timeout_us_to_mclks(
        final_range_timeout_us, to.final_range_vcsel_period_pclks);
    if (en.pre_range) {
        final_range_timeout_mclks += to.pre_range_mclks;
    }

    return write_reg16(dev,
                       VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                       encode_timeout(final_range_timeout_mclks));
}

// -----------------------------------------------------------------------------
// Reference SPAD setup.
// -----------------------------------------------------------------------------
static esp_err_t get_spad_info(i2c_master_dev_handle_t dev,
                               uint8_t *count, bool *type_is_aperture) {
    esp_err_t err;
    uint8_t tmp = 0;

#define W8(reg_, val_) do { err = write_reg8(dev, (reg_), (val_)); if (err != ESP_OK) return err; } while (0)
#define R8(reg_, ptr_) do { err = read_reg8(dev, (reg_), (ptr_)); if (err != ESP_OK) return err; } while (0)

    W8(0x80, 0x01);
    W8(0xFF, 0x01);
    W8(0x00, 0x00);
    W8(0xFF, 0x06);

    R8(0x83, &tmp);
    W8(0x83, (uint8_t)(tmp | 0x04));

    W8(0xFF, 0x07);
    W8(0x81, 0x01);
    W8(0x80, 0x01);
    W8(0x94, 0x6B);
    W8(0x83, 0x00);

    int64_t deadline_us = esp_timer_get_time() + (int64_t)VL53L0X_INIT_TIMEOUT_MS * 1000;
    do {
        R8(0x83, &tmp);
        if (tmp != 0x00) break;
        if (esp_timer_get_time() >= deadline_us) {
            ESP_LOGE(TAG, "VL53L0X get_spad_info timeout");
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    } while (1);

    W8(0x83, 0x01);
    R8(0x92, &tmp);
    *count = tmp & 0x7F;
    *type_is_aperture = ((tmp >> 7) & 0x01) != 0;

    W8(0x81, 0x00);
    W8(0xFF, 0x06);
    R8(0x83, &tmp);
    W8(0x83, (uint8_t)(tmp & (uint8_t)~0x04));
    W8(0xFF, 0x01);
    W8(0x00, 0x01);
    W8(0xFF, 0x00);
    W8(0x80, 0x00);

#undef W8
#undef R8
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// ST default tuning settings used by common VL53L0X implementations.
// -----------------------------------------------------------------------------
typedef struct { uint8_t reg; uint8_t val; } reg8_pair_t;

static const reg8_pair_t VL53L0X_TUNING[] = {
    {0xFF,0x01},{0x00,0x00},{0xFF,0x00},{0x09,0x00},{0x10,0x00},{0x11,0x00},
    {0x24,0x01},{0x25,0xFF},{0x75,0x00},{0xFF,0x01},{0x4E,0x2C},{0x48,0x00},
    {0x30,0x20},{0xFF,0x00},{0x30,0x09},{0x54,0x00},{0x31,0x04},{0x32,0x03},
    {0x40,0x83},{0x46,0x25},{0x60,0x00},{0x27,0x00},{0x50,0x06},{0x51,0x00},
    {0x52,0x96},{0x56,0x08},{0x57,0x30},{0x61,0x00},{0x62,0x00},{0x64,0x00},
    {0x65,0x00},{0x66,0xA0},{0xFF,0x01},{0x22,0x32},{0x47,0x14},{0x49,0xFF},
    {0x4A,0x00},{0xFF,0x00},{0x7A,0x0A},{0x7B,0x00},{0x78,0x21},{0xFF,0x01},
    {0x23,0x34},{0x42,0x00},{0x44,0xFF},{0x45,0x26},{0x46,0x05},{0x40,0x40},
    {0x0E,0x06},{0x20,0x1A},{0x43,0x40},{0xFF,0x00},{0x34,0x03},{0x35,0x44},
    {0xFF,0x01},{0x31,0x04},{0x4B,0x09},{0x4C,0x05},{0x4D,0x04},{0xFF,0x00},
    {0x44,0x00},{0x45,0x20},{0x47,0x08},{0x48,0x28},{0x67,0x00},{0x70,0x04},
    {0x71,0x01},{0x72,0xFE},{0x76,0x00},{0x77,0x00},{0xFF,0x01},{0x0D,0x01},
    {0xFF,0x00},{0x80,0x01},{0x01,0xF8},{0xFF,0x01},{0x8E,0x01},{0x00,0x01},
    {0xFF,0x00},{0x80,0x00},
};

static esp_err_t load_tuning(i2c_master_dev_handle_t dev) {
    for (size_t i = 0; i < sizeof(VL53L0X_TUNING) / sizeof(VL53L0X_TUNING[0]); ++i) {
        esp_err_t err = write_reg8(dev, VL53L0X_TUNING[i].reg, VL53L0X_TUNING[i].val);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "VL53L0X tuning[%u] reg=0x%02X failed: %s",
                     (unsigned)i, VL53L0X_TUNING[i].reg, esp_err_to_name(err));
            return err;
        }
    }
    return ESP_OK;
}

static esp_err_t perform_single_ref_calibration(i2c_master_dev_handle_t dev,
                                                 uint8_t vhv_init_byte) {
    esp_err_t err = write_reg8(dev, VL53L0X_REG_SYSRANGE_START,
                               (uint8_t)(0x01 | vhv_init_byte));
    if (err != ESP_OK) return err;

    int64_t deadline_us = esp_timer_get_time() + (int64_t)VL53L0X_INIT_TIMEOUT_MS * 1000;
    while (1) {
        uint8_t status = 0;
        err = read_reg8(dev, VL53L0X_REG_RESULT_INTERRUPT_STATUS, &status);
        if (err != ESP_OK) return err;
        if ((status & 0x07) != 0) break;
        if (esp_timer_get_time() >= deadline_us) {
            ESP_LOGE(TAG, "VL53L0X reference calibration timeout");
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    err = write_reg8(dev, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    if (err != ESP_OK) return err;
    return write_reg8(dev, VL53L0X_REG_SYSRANGE_START, 0x00);
}

// -----------------------------------------------------------------------------
// Core VL53L0X initialization.
// ----------------------------------------------------------------------------
static esp_err_t vl53l0x_sensor_init(tof_sensor_state_t *s) {
    if (!s || !s->dev) return ESP_ERR_INVALID_ARG;

    i2c_master_dev_handle_t dev = s->dev;
    esp_err_t err;
    uint8_t model = 0, revision = 0, tmp = 0;

    err = read_reg8(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &model);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "doc MODEL_ID VL53L0X that bai tai 0x%02X: %s",
                 s->addr, esp_err_to_name(err));
        return err;
    }
    (void)read_reg8(dev, VL53L0X_REG_IDENTIFICATION_REVISION_ID, &revision);

    if (model != VL53L0X_MODEL_ID_VALUE) {
        ESP_LOGE(TAG, "MODEL_ID VL53L0X sai: doc 0x%02X, ky vong 0x%02X tai 0x%02X",
                 model, VL53L0X_MODEL_ID_VALUE, s->addr);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "VL53L0X detected tai 0x%02X: MODEL_ID=0x%02X REV=0x%02X",
             s->addr, model, revision);

#define CHECK(expr_) do { err = (expr_); if (err != ESP_OK) { \
    ESP_LOGE(TAG, "VL53L0X init failed line %d: %s", __LINE__, esp_err_to_name(err)); \
    return err; } } while (0)

    // DataInit: configure 2V8 I/O mode and standard I2C mode.
    CHECK(read_reg8(dev, VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, &tmp));
    CHECK(write_reg8(dev, VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV,
                     (uint8_t)(tmp | 0x01)));
    CHECK(write_reg8(dev, 0x88, 0x00));

    // Read per-sensor stop variable from private page.
    CHECK(write_reg8(dev, 0x80, 0x01));
    CHECK(write_reg8(dev, 0xFF, 0x01));
    CHECK(write_reg8(dev, 0x00, 0x00));
    CHECK(read_reg8(dev, 0x91, &s->stop_variable));
    CHECK(write_reg8(dev, 0x00, 0x01));
    CHECK(write_reg8(dev, 0xFF, 0x00));
    CHECK(write_reg8(dev, 0x80, 0x00));

    // Disable MSRC/pre-range signal-rate checks; final limit = 0.25 MCPS (Q9.7 = 32).
    CHECK(read_reg8(dev, VL53L0X_REG_MSRC_CONFIG_CONTROL, &tmp));
    CHECK(write_reg8(dev, VL53L0X_REG_MSRC_CONFIG_CONTROL, (uint8_t)(tmp | 0x12)));
    CHECK(write_reg16(dev, VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 32));
    CHECK(write_reg8(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xFF));

    // StaticInit: reference SPAD map.
    uint8_t spad_count = 0;
    bool spad_type_is_aperture = false;
    CHECK(get_spad_info(dev, &spad_count, &spad_type_is_aperture));

    uint8_t ref_spad_map[6] = {0};
    CHECK(read_multi(dev, VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0,
                     ref_spad_map, sizeof(ref_spad_map)));

    CHECK(write_reg8(dev, 0xFF, 0x01));
    CHECK(write_reg8(dev, 0x4F, 0x00)); // DYNAMIC_SPAD_REF_EN_START_OFFSET
    CHECK(write_reg8(dev, 0x4E, 0x2C)); // DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD
    CHECK(write_reg8(dev, 0xFF, 0x00));
    CHECK(write_reg8(dev, VL53L0X_REG_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4));

    uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0;
    uint8_t spads_enabled = 0;
    for (uint8_t i = 0; i < 48; ++i) {
        if (i < first_spad_to_enable || spads_enabled == spad_count) {
            ref_spad_map[i / 8] &= (uint8_t)~(1U << (i % 8));
        } else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x01U) {
            spads_enabled++;
        }
    }
    CHECK(write_multi(dev, VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0,
                      ref_spad_map, sizeof(ref_spad_map)));

    CHECK(load_tuning(dev));

    // New-sample-ready interrupt configuration (active low internally).
    CHECK(write_reg8(dev, VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04));
    CHECK(read_reg8(dev, VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH, &tmp));
    CHECK(write_reg8(dev, VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH,
                     (uint8_t)(tmp & (uint8_t)~0x10)));
    CHECK(write_reg8(dev, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01));

    // Disable MSRC and TCC, retain pre-range + final-range.
    CHECK(write_reg8(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xE8));
    CHECK(set_measurement_timing_budget(dev, VL53L0X_DEFAULT_TIMING_BUDGET_US));

    // Reference calibrations.
    CHECK(write_reg8(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x01));
    CHECK(perform_single_ref_calibration(dev, 0x40));
    CHECK(write_reg8(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x02));
    CHECK(perform_single_ref_calibration(dev, 0x00));
    CHECK(write_reg8(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xE8));

    // Restore stop variable and enter continuous back-to-back ranging.
    CHECK(write_reg8(dev, 0x80, 0x01));
    CHECK(write_reg8(dev, 0xFF, 0x01));
    CHECK(write_reg8(dev, 0x00, 0x00));
    CHECK(write_reg8(dev, 0x91, s->stop_variable));
    CHECK(write_reg8(dev, 0x00, 0x01));
    CHECK(write_reg8(dev, 0xFF, 0x00));
    CHECK(write_reg8(dev, 0x80, 0x00));
    CHECK(write_reg8(dev, VL53L0X_REG_SYSRANGE_START, VL53L0X_MODE_BACKTOBACK));

    s->ranging_started = true;
    s->last_good_us = 0;
    // Mốc watchdog bắt đầu TỪ ĐÂY: chip vừa được lệnh đo, nên 300ms tới phải có
    // mẫu đầu tiên. Để 0 thì watchdog không bao giờ chạy (xem điều kiện
    // last_sample_us != 0 trong poll_sensor).
    s->last_sample_us = esp_timer_get_time();
    memset(&s->last, 0, sizeof(s->last));
    s->last.range_status = 255;

    ESP_LOGI(TAG, "VL53L0X ranging START tai 0x%02X (timing budget %lu us)",
             s->addr, (unsigned long)VL53L0X_DEFAULT_TIMING_BUDGET_US);

#undef CHECK
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// I2C address change for VL53L0X.
// Register 0x8A receives the 7-bit address directly.
// -----------------------------------------------------------------------------
static esp_err_t vl53l0x_write_i2c_address(i2c_master_dev_handle_t dev,
                                            uint8_t new_addr) {
    if (new_addr == 0 || new_addr >= 0x80) return ESP_ERR_INVALID_ARG;

    esp_err_t err = write_reg8(dev, VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS,
                               (uint8_t)(new_addr & 0x7F));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "VL53L0X ghi lenh doi I2C -> 0x%02X that bai: %s",
                 new_addr, esp_err_to_name(err));
        return err;
    }

    err = i2c_master_device_change_address(dev, new_addr, VL53L0X_IO_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "device handle doi sang 0x%02X that bai: %s",
                 new_addr, esp_err_to_name(err));
        return err;
    }

    // Xac nhan giao tiep THUC SU tai dia chi moi. i2c_master_device_change_address()
    // chi cap nhat device handle phia ESP-IDF; doc MODEL_ID de dam bao ca sensor
    // va handle da cung chuyen sang new_addr.
    uint8_t model = 0;
    err = read_reg8(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &model);
    if (err != ESP_OK || model != VL53L0X_MODEL_ID_VALUE) {
        ESP_LOGE(TAG, "VL53L0X verify dia chi moi 0x%02X that bai: err=%s model=0x%02X",
                 new_addr, esp_err_to_name(err), model);
        return (err != ESP_OK) ? err : ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "VL53L0X doi dia chi I2C -> 0x%02X OK (MODEL_ID=0x%02X)",
             new_addr, model);
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// XSHUT — active LOW. Dung open-drain de LOW = shutdown, HIGH = release.
// -----------------------------------------------------------------------------
// Theo datasheet VL53L0X, XSHUT can co muc logic xac dinh; neu host co trang
// thai boot khong xac dinh thi nen co pull-up (ST khuyen nghi 10k). Voi carrier
// co pull-up XSHUT ve AVDD/IOVDD, GPIO_MODE_OUTPUT_OD la cach sach nhat:
//   gpio_set_level(..., 0) -> keo LOW, hardware standby
//   gpio_set_level(..., 1) -> nha high-Z, pull-up hardware keo HIGH
//
// Datasheet cung ghi cac I/O la failsafe; khong dua vao gia thuyet co diode ESD
// ve AVDD. Trong 2V8 mode, muc HIGH cua XSHUT/SDA/SCL/GPIO1 nen bang AVDD.
//
// gpio < 0 = chan XSHUT khong duoc dieu khien boi firmware. Single-ToF co the
// van hoat dong neu hardware pull-up XSHUT; dual-ToF thi BAT BUOC dieu khien
// duoc ca hai XSHUT de tach hai sensor cung boot tai dia chi 0x29.
static esp_err_t configure_xshut_gpio(int gpio_num) {
    if (gpio_num < 0) return ESP_ERR_INVALID_ARG;
    if (!GPIO_IS_VALID_OUTPUT_GPIO(gpio_num)) {
        ESP_LOGE(TAG, "XSHUT GPIO%d khong phai chan output hop le tren chip nay", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    gpio_config_t cfg = {0};
    cfg.pin_bit_mask = 1ULL << gpio_num;
    // INPUT_OUTPUT_OD, KHÔNG phải OUTPUT_OD.
    //
    // Cả hai lái open-drain giống hệt nhau; khác biệt DUY NHẤT là bit INPUT.
    // gpio_config() với GPIO_MODE_OUTPUT_OD (= OUTPUT|OD, KHÔNG có bit INPUT)
    // gọi gpio_input_disable() -> TẮT input buffer của pad -> gpio_get_level()
    // trên chân đó trả về 0 VĨNH VIỄN, bất kể điện áp thật trên chân.
    //
    // Hậu quả với chính file này: xshut_check_released_high() đọc bằng
    // gpio_get_level() nên nó LUÔN LUÔN thấy LOW và LUÔN LUÔN báo lỗi
    // "XSHUT vẫn LOW sau khi release" — kể cả khi dây và pull-up hoàn toàn
    // đúng. Với GPIO0 nó còn in nguyên cảnh báo strapping/DOWNLOAD MODE. Đó là
    // một báo động giả chỉ vào đúng chỗ KHÔNG hỏng, trong khi lỗi thật (chip
    // không ACK / sai MODEL_ID) nằm ở dưới.
    //
    // KHÔNG đổi sang GPIO_MODE_OUTPUT/GPIO_MODE_INPUT_OUTPUT (push-pull): xem
    // khối comment ở trên — XSHUT phải lái open-drain.
    cfg.mode = GPIO_MODE_INPUT_OUTPUT_OD;
    cfg.pull_up_en = GPIO_PULLUP_DISABLE;   // dung pull-up hardware ngoai/onboard theo mach
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;

    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) return err;

    // Sau gpio_config(), dua XSHUT ve LOW ngay de startup co trang thai xac dinh.
    // Khi release, GPIO_MODE_OUTPUT_OD + level=1 se nha duong XSHUT de pull-up
    // cua hardware keo len AVDD/IOVDD.
    return gpio_set_level(gpio_num, 0);
}

// Giữ chip trong reset (kéo LOW thật).
static void xshut_hold(int gpio_num) {
    if (gpio_num < 0) return;
    gpio_set_level(gpio_num, 0);
}

// Nha chip cho boot (HIGH-Z; pull-up hardware keo XSHUT len).
static void xshut_release(int gpio_num) {
    if (gpio_num < 0) return;
    gpio_set_level(gpio_num, 1);
}

// Sau khi release open-drain, XSHUT phai duoc pull-up keo HIGH. Neu van LOW thi
// sensor se khong the boot; bao loi ngay tai nguyen nhan thay vi de MODEL_ID NACK.
static esp_err_t xshut_check_released_high(int gpio_num) {
    if (gpio_num < 0) return ESP_OK;

    const int64_t deadline_us = esp_timer_get_time() +
                                (int64_t)VL53L0X_BOOT_DELAY_MS * 1000;
    while (esp_timer_get_time() < deadline_us) {
        if (gpio_get_level(gpio_num) != 0) return ESP_OK;
        vTaskDelay(1);
    }

    ESP_LOGE(TAG, "XSHUT GPIO%d van LOW sau khi release -> kiem tra pull-up XSHUT, "
                  "wiring hoac short GND", gpio_num);
    return ESP_ERR_INVALID_STATE;
}

static esp_err_t add_device_at(i2c_master_bus_handle_t bus, uint8_t addr,
                               i2c_master_dev_handle_t *out_dev) {
    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = s_scl_speed_hz,
    };
    return i2c_master_bus_add_device(bus, &dev_cfg, out_dev);
}

// -----------------------------------------------------------------------------
// identify_foreign_device() — "không phải VL53L0X" thì NÓ LÀ CÁI GÌ?
// -----------------------------------------------------------------------------
// Báo "MODEL_ID sai" rồi dừng là bỏ người debug lại giữa đường: họ vẫn không
// biết nên sửa dây, đổi địa chỉ, hay viết driver khác. Ba khả năng có thật khi
// một thiết bị ACK ở 0x29 mà 0xC0 != 0xEE, và chúng cần ba hành động KHÁC HẲN
// nhau — nên phải tách được chúng ra ngay tại đây.
//
// Cách phân biệt (đã đối chiếu datasheet ST + thư viện Pololu):
//   VL53L0X : địa chỉ thanh ghi 8-BIT.  0xC0/0xC1/0xC2 = 0xEE / 0xAA / 0x10
//   VL53L1X : địa chỉ thanh ghi 16-BIT. 0x010F/0x0110 = 0xEA / 0xCC (= 0xEACC)
//             (VL53L4CD dùng chung họ L1X, cũng trả 0xEACC)
//
// Đây chính là lý do một VL53L1X cắm nhầm vào driver này KHÔNG bao giờ chạy:
// nó ACK ở 0x29 y hệt, nhưng driver gửi MỘT byte địa chỉ (0xC0) trong khi chip
// đợi HAI byte — chip hiểu 0xC0 là byte cao của một index 16-bit và chờ tiếp,
// nên số đọc về là rác. Không có tham số nào chỉnh được chuyện đó; phải là
// driver khác.
static void identify_foreign_device(i2c_master_bus_handle_t bus, uint8_t addr,
                                     esp_err_t l0x_err, uint8_t l0x_model) {
    i2c_master_dev_handle_t dev = NULL;
    if (add_device_at(bus, addr, &dev) != ESP_OK || dev == NULL) return;

    // Thử đọc ID của họ VL53L1X: index 16-bit 0x010F, đọc 2 byte.
    const uint8_t idx16[2] = {0x01, 0x0F};
    uint8_t id[2] = {0, 0};
    const esp_err_t l1x_err = i2c_master_transmit_receive(dev, idx16, sizeof(idx16),
                                                           id, sizeof(id), s_io_timeout_ms);
    i2c_master_bus_rm_device(dev);

    if (l1x_err == ESP_OK && id[0] == 0xEA && id[1] == 0xCC) {
        ESP_LOGE(TAG, "=> THIET BI TAI 0x%02X LA VL53L1X (hoac VL53L4CD), KHONG PHAI VL53L0X.", addr);
        ESP_LOGE(TAG, "   ID 16-bit 0x010F = 0xEACC. Driver nay CHI ho tro VL53L0X (thanh ghi");
        ESP_LOGE(TAG, "   8-bit, ID 0xC0=0xEE) nen KHONG the dung — khong phai loi day/nguon,");
        ESP_LOGE(TAG, "   va khong co tham so nao chinh duoc. Can driver VL53L1X rieng, hoac");
        ESP_LOGE(TAG, "   thay bang module VL53L0X that.");
        return;
    }

    ESP_LOGE(TAG, "=> Thiet bi tai 0x%02X KHONG nhan dang duoc: L0X(0xC0)=0x%02X (err=%s), "
                  "L1X(0x010F)=0x%02X%02X (err=%s).",
             addr, l0x_model, esp_err_to_name(l0x_err), id[0], id[1], esp_err_to_name(l1x_err));
    ESP_LOGE(TAG, "   Doc ra 0x00/0xFF toan bo = chip chua boot xong hoac nguon khong on;");
    ESP_LOGE(TAG, "   so khac hoan toan = day KHONG phai cam bien ToF ho VL53.");
}

// -----------------------------------------------------------------------------
// find_sensor_addr() — chip THẬT SỰ đang trả lời ở địa chỉ nào?
// -----------------------------------------------------------------------------
// Bản trước bỏ hẳn bước này, và đó là lỗi thiết kế thứ hai:
// i2c_master_bus_add_device() KHÔNG chạm bus (nó chỉ cấp phát handle), nên nó
// LUÔN trả ESP_OK kể cả khi trên bus không có gì. Giao dịch thật đầu tiên là
// lệnh đổi địa chỉ hoặc lệnh đọc MODEL_ID — hỏng ở đó thì log chỉ nói "that
// bai", không nói được chip có mặt hay không, và nếu có thì ở đâu.
//
// Ngoài ra có một trạng thái thực tế mà bản trước KHÔNG THỂ tự phục hồi: địa
// chỉ I2C của VL53L0X nằm trong RAM của chip, chỉ mất khi XSHUT xuống LOW hoặc
// MẤT NGUỒN. Nạp lại firmware / nhấn RESET ESP32 KHÔNG cắt nguồn ToF. Nếu dây
// XSHUT không được hàn (rất phổ biến với module GY-530 mua rời) thì:
//   boot 1: chip ở 0x29 -> driver đổi sang 0x2A -> chạy OK
//   boot 2: driver "kéo" XSHUT (không nối) -> chip VẪN ở 0x2A, nhưng driver
//           nói chuyện ở 0x29 -> NACK -> driver_ok=0 MÃI cho tới khi rút điện.
// Dò cả hai địa chỉ làm trạng thái đó tự lành.
//
// Thử lại nhiều lần vì t_boot có thể chưa xong đúng lúc probe đầu tiên.
static bool find_sensor_addr(i2c_master_bus_handle_t bus, uint8_t wanted_addr,
                             uint8_t *out_addr) {
    if (!bus || !out_addr) return false;

    // Sau mot hardware reset dung, VL53L0X LUON tro ve 0x29. Vi vay probe 0x29
    // truoc; wanted_addr chi la fallback cho truong hop XSHUT khong thuc su reset
    // duoc sensor va chip con giu dia chi RAM tu lan chay truoc.
    const uint8_t candidates[2] = {
        VL53L0X_DEFAULT_I2C_ADDR,
        wanted_addr,
    };

    for (int attempt = 0; attempt < VL53L0X_PROBE_ATTEMPTS; ++attempt) {
        for (int i = 0; i < 2; ++i) {
            const uint8_t addr = candidates[i];
            if (i == 1 && addr == VL53L0X_DEFAULT_I2C_ADDR) continue;

            if (i2c_master_probe(bus, addr, VL53L0X_IO_TIMEOUT_MS) != ESP_OK) {
                continue;
            }

            // ACK chua du de ket luan day la VL53L0X. Tao handle tam va doc
            // MODEL_ID truoc khi cho phep driver ghi register 0x8A doi dia chi.
            i2c_master_dev_handle_t probe_dev = NULL;
            if (add_device_at(bus, addr, &probe_dev) != ESP_OK || probe_dev == NULL) {
                continue;
            }

            uint8_t model = 0;
            const esp_err_t id_err = read_reg8(
                probe_dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &model);
            i2c_master_bus_rm_device(probe_dev);

            if (id_err == ESP_OK && model == VL53L0X_MODEL_ID_VALUE) {
                *out_addr = addr;
                return true;
            }

            // CHỈ nhận diện sâu ở lần thử CUỐI. Các lần trước có thể trượt đơn
            // giản vì chip chưa boot xong (t_boot), và in đầy đủ 5 lần sẽ đẩy
            // kết luận thật ra khỏi màn hình — log boot trên USB-Serial-JTAG
            // còn xen với log wifi, rất dễ bị cắt mất đúng dòng cần đọc.
            if (attempt == VL53L0X_PROBE_ATTEMPTS - 1) {
                ESP_LOGE(TAG, "I2C 0x%02X ACK nhung MODEL_ID khong phai VL53L0X "
                              "(doc 0x%02X, ky vong 0xEE, err=%s)",
                         addr, model, esp_err_to_name(id_err));
                identify_foreign_device(bus, addr, id_err, model);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(VL53L0X_PROBE_RETRY_MS));
    }
    return false;
}

// -----------------------------------------------------------------------------
// Public API: bring-up MỘT sensor.
// -----------------------------------------------------------------------------
// ĐÃ ĐƠN GIẢN HOÁ TỪ 2 SENSOR VỀ 1. Bản trước có tof_driver_init_dual() +
// tof_driver_read_aux() + s_sensors[2] + slot_for_addr() để hỗ trợ một con ToF
// thứ hai (forward/dự phòng) mà THỰC TẾ CHƯA BAO GIỜ được fuse vào control loop
// (SENSOR_TOF2_ENABLED luôn = 0). Toàn bộ đường đó giờ đã xoá: nó mang theo
// những ràng buộc CHỈ có nghĩa khi có 2 chip (addr1 không được giữ 0x29, cả 2
// XSHUT phải điều khiển được, thứ tự bring-up bắt buộc) và những ràng buộc đó
// đã gây ra lỗi thật cho cấu hình 1 chip.
//
// Với MỘT sensor, mọi thứ đơn giản hơn hẳn:
//   - Địa chỉ mặc định 0x29 dùng luôn được, KHÔNG cần đổi (không ai tranh).
//   - Dây XSHUT là TÙY CHỌN (module có pull-up 10k, chip tự chạy).
//   - Không có thứ tự bring-up nào cần tuân theo.
//
// xshut_gpio < 0 = không nối, hoàn toàn hợp lệ.
esp_err_t tof_driver_init(i2c_master_bus_handle_t bus, int xshut_gpio,
                           uint8_t i2c_addr, uint32_t scl_speed_hz) {
    if (!bus) return ESP_ERR_INVALID_ARG;
    if (i2c_addr == 0 || i2c_addr >= 0x80) return ESP_ERR_INVALID_ARG;

    if (scl_speed_hz > 0) s_scl_speed_hz = scl_speed_hz;

    // ---- XSHUT: BEST-EFFORT, không bao giờ chặn bring-up ----
    // Chân không nối / không hợp lệ vẫn init được vì module tự pull-up. Đây là
    // khác biệt then chốt so với cấu hình 2 chip (nơi XSHUT là ĐIỀU KIỆN CẦN).
    const bool xshut_ok = (xshut_gpio >= 0) &&
                          (configure_xshut_gpio(xshut_gpio) == ESP_OK);

    ESP_LOGI(TAG, "VL53L0X bring-up: addr muc tieu 0x%02X, SCL %lu Hz, XSHUT=%s",
             i2c_addr, (unsigned long)s_scl_speed_hz,
             xshut_ok ? "co (open-drain)" : "khong noi (dung pull-up tren module)");

    if (xshut_gpio >= 0 && !xshut_ok) {
        ESP_LOGW(TAG, "XSHUT GPIO%d KHONG dung duoc -> khong reset cung duoc chip, "
                      "van thu bring-up qua pull-up hardware", xshut_gpio);
    }

    // ---- Reset cứng (nếu điều khiển được XSHUT) ----
    // Xoá state trong RAM chip, kể cả địa chỉ I2C đã đổi ở lần boot trước —
    // đó là lý do DUY NHẤT còn lại để cần XSHUT khi chỉ có 1 sensor.
    if (xshut_ok) {
        xshut_hold(xshut_gpio);
        vTaskDelay(pdMS_TO_TICKS(VL53L0X_XSHUT_HOLD_MS));
        xshut_release(xshut_gpio);

        // XÁC NHẬN pull-up thật sự kéo chân lên sau khi nhả. Lái open-drain
        // nghĩa là "nhả" = high-Z — nếu KHÔNG có pull-up (hoặc chân bị short
        // GND) thì chân ở LƯNG CHỪNG/LOW, chip không bao giờ boot, và triệu
        // chứng duy nhất sẽ là MODEL_ID NACK ở dưới — sai nguyên nhân.
        //
        // ⚠ Với GPIO0 (chân strapping BOOT của ESP32-S3) lỗi này còn nghiêm
        // trọng hơn một bậc: chân đọc LOW ở đây nghĩa là nó CŨNG sẽ LOW ở lần
        // reset kế tiếp -> bo vào DOWNLOAD MODE thay vì boot firmware. Nên báo
        // riêng, không gộp vào cảnh báo chung.
        if (xshut_check_released_high(xshut_gpio) != ESP_OK) {
            if (xshut_gpio == 0) {
                ESP_LOGE(TAG, "XSHUT dang o GPIO0 = chan STRAPPING 'BOOT'. Chan nay KHONG len "
                              "HIGH duoc -> lan RESET tiep theo bo se vao DOWNLOAD MODE (khong "
                              "chay firmware). SUA DAY NGAY, hoac dat BOARD_TOF_XSHUT_GPIO = -1 "
                              "(dia chi 0x29 mac dinh khong can XSHUT).");
            }
            // KHÔNG return: vẫn thử dò bus bên dưới. Có thể module tự chạy bằng
            // pull-up riêng của nó và chỉ có đường ĐỌC chân là sai.
        }

        vTaskDelay(pdMS_TO_TICKS(VL53L0X_BOOT_DELAY_MS));
    }

    memset(&s_sensor, 0, sizeof(s_sensor));

    // ---- Tìm chip THẬT SỰ đang ở đâu ----
    // Dò cả địa chỉ mong muốn LẪN 0x29: sau reset chip luôn ở 0x29, nhưng nếu
    // XSHUT không nối thì nó có thể CÒN GIỮ địa chỉ đã đổi từ lần boot trước
    // (địa chỉ nằm trong RAM chip, RESET ESP32 không xoá — xem find_sensor_addr).
    uint8_t found_at = 0;
    if (!find_sensor_addr(bus, i2c_addr, &found_at)) {
        ESP_LOGE(TAG, "KHONG thiet bi nao tra loi tren I2C o 0x%02X hay 0x%02X.",
                 i2c_addr, VL53L0X_DEFAULT_I2C_ADDR);
        ESP_LOGE(TAG, "Kiem tra theo thu tu -- (1) VIN/GND cua module, (2) SDA/SCL co dung "
                      "2 chan trong board_config.h khong, (3) chay lenh console 'i2c_scan' "
                      "de xem bus thuc su co gi.");
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "tim thay VL53L0X tai 0x%02X", found_at);

    i2c_master_dev_handle_t dev = NULL;
    esp_err_t err = add_device_at(bus, found_at, &dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "them device 0x%02X vao bus that bai: %s",
                 found_at, esp_err_to_name(err));
        return err;
    }

    // ---- Đổi địa chỉ CHỈ KHI cần, và hỏng thì KHÔNG chết ----
    // Với 1 sensor, đổi địa chỉ thuần là "cho gọn" — không ai tranh 0x29. Nên
    // hỏng thì ở lại địa chỉ hiện tại và vẫn bay được. (Ở cấu hình 2 chip đây
    // từng là lỗi CỨNG vì con thứ hai bắt buộc cần 0x29 trống.)
    uint8_t final_addr = found_at;
    if (found_at != i2c_addr) {
        err = vl53l0x_write_i2c_address(dev, i2c_addr);
        if (err == ESP_OK) {
            final_addr = i2c_addr;
        } else {
            ESP_LOGW(TAG, "doi dia chi 0x%02X -> 0x%02X that bai, DUNG TIEP o 0x%02X "
                          "(chi co 1 ToF nen khong ai tranh dia chi -- van bay duoc)",
                     found_at, i2c_addr, found_at);
        }
    } else if (found_at != VL53L0X_DEFAULT_I2C_ADDR) {
        // Chip đã ở sẵn địa chỉ đích => còn giữ state từ lần boot trước, tức
        // XSHUT không thật sự reset được nó. Không phải lỗi, nhưng phải nói ra
        // vì nó giải thích mọi hành vi "lạ" về sau.
        ESP_LOGW(TAG, "chip DA o 0x%02X san (giu state tu lan boot truoc) -> XSHUT khong "
                      "reset duoc chip. Van init lai ranging tu dau ben duoi.", found_at);
    }

    s_sensor.dev = dev;
    s_sensor.addr = final_addr;

    err = vl53l0x_sensor_init(&s_sensor);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init ranging tai 0x%02X that bai: %s -- thiet bi CO tra loi ACK nen "
                      "day/dia chi OK; nghi nguon 2.8V khong on, hoac chip khong phai VL53L0X",
                 final_addr, esp_err_to_name(err));
        i2c_master_bus_rm_device(dev);
        memset(&s_sensor, 0, sizeof(s_sensor));
        return err;
    }

    // Từ đây mọi transaction là runtime (tof_driver_read() trong sensor_hub).
    s_io_timeout_ms = VL53L0X_IO_RUNTIME_TIMEOUT_MS;
    ESP_LOGI(TAG, "VL53L0X SAN SANG tai 0x%02X (huong xuong, nguon correction cho alt_estimator, "
                  "I2C timeout runtime=%dms)", final_addr, VL53L0X_IO_RUNTIME_TIMEOUT_MS);
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// Convert raw device-range-status bits to a lightweight PAL-like status.
// Full ST PAL status additionally evaluates sigma / limit checks; here we keep
// only the device status mapping so existing tof_reading_t::range_status remains
// meaningful without importing the whole ST PAL.
// -----------------------------------------------------------------------------
static uint8_t map_device_range_status(uint8_t result_range_status) {
    uint8_t s = (uint8_t)((result_range_status & 0x78) >> 3);

    switch (s) {
        case 11: return 0;   // Range valid
        case 4:  return 2;   // Signal fail
        case 8:
        case 10: return 3;   // Min range fail
        case 6:
        case 9:  return 4;   // Phase fail
        case 1:
        case 2:
        case 3:  return 5;   // Hardware fail
        case 0:
        case 5:
        case 7:
        case 12:
        case 13:
        case 14:
        case 15:
        default: return 255; // No update / unknown
    }
}

static esp_err_t poll_sensor(void) {
    tof_sensor_state_t *s = &s_sensor;
    if (!s->dev || !s->ranging_started) return ESP_ERR_INVALID_STATE;

    // Only consume a NEW measurement. Do not repeatedly read/fuse the same one.
    uint8_t int_status = 0;
    esp_err_t err = read_reg8(s->dev, VL53L0X_REG_RESULT_INTERRUPT_STATUS, &int_status);
    if (err != ESP_OK) return err;
    if ((int_status & 0x07) == 0) {
        // ---- STALL WATCHDOG (xem tof_driver.h) ----
        // Không có mẫu mới là BÌNH THƯỜNG ở phần lớn các vòng: hub poll ~31Hz
        // còn chip đo ~30Hz nên hai nhịp trượt nhau. Nhưng KHÔNG có mẫu nào
        // trong 300ms (9 chu kỳ đo) nghĩa là chip đã ngừng hẳn việc đo — trạng
        // thái treo được báo cáo rộng rãi với VL53L0X ở continuous mode, trong
        // đó I2C vẫn ACK bình thường nên không có cách nào phát hiện khác.
        const int64_t now_us = esp_timer_get_time();
        if (s->last_sample_us != 0 &&
            (now_us - s->last_sample_us) > (int64_t)TOF_STALL_RESTART_MS * 1000) {
            // Phục hồi NHẸ: clear cờ ngắt + phát lại lệnh continuous. Hai lệnh
            // ghi (~1ms), KHÔNG phải full re-init.
            //
            // VÌ SAO KHÔNG full re-init ở đây: bring-up đầy đủ block tới ~1s (2
            // lần ref calibration, mỗi lần timeout 500ms) và nó chạy TRONG
            // sensor_hub — tức là treo luôn nguồn cấp mẫu IMU của vòng bay
            // trong 1 giây. Đó là đổi một cảm biến phụ lấy toàn bộ khả năng ổn
            // định. Phục hồi nhẹ đủ cho trường hợp chip chỉ ngừng ranging mà
            // vẫn giữ cấu hình; mất cấu hình thật thì cần 'tof_reinit' tay lúc
            // DISARMED.
            (void)write_reg8(s->dev, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
            const esp_err_t rerr = write_reg8(s->dev, VL53L0X_REG_SYSRANGE_START,
                                               VL53L0X_MODE_BACKTOBACK);
            s_stall_restarts++;
            // Dời mốc BẤT KỂ thành công hay không: nếu không, mỗi vòng poll kế
            // tiếp lại thử restart và biến watchdog thành vòng lặp ghi bus.
            s->last_sample_us = now_us;
            ESP_LOGW(TAG, "ToF ngung do %dms -> khoi dong lai ranging (lan thu %u, ghi=%s). "
                          "So nay TANG DEU = phan cung that su co van de: sut ap luc motor rut "
                          "dong (VCSEL peak 40mA) hoac nhieu I2C, KHONG phai loi phan mem",
                     TOF_STALL_RESTART_MS, (unsigned)s_stall_restarts, esp_err_to_name(rerr));
        }
        return ESP_OK; // no new sample; keep previous reading unchanged
    }

    // Có mẫu mới = chip còn sống. Ghi mốc TRƯỚC khi xét range_status: ngoài
    // tầm vẫn là một lần đo thành công về mặt phần cứng.
    s->last_sample_us = esp_timer_get_time();

    // RESULT_RANGE_STATUS block is 12 bytes; range_mm is at offset +10.
    uint8_t result[12] = {0};
    err = read_multi(s->dev, VL53L0X_REG_RESULT_RANGE_STATUS, result, sizeof(result));
    if (err != ESP_OK) return err;

    // Clear interrupt so the next ranging result can be generated.
    err = write_reg8(s->dev, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    if (err != ESP_OK) return err;

    const uint16_t dist_mm = (uint16_t)(((uint16_t)result[10] << 8) | result[11]);
    const uint8_t mapped = map_device_range_status(result[0]);

    s->last.range_status = mapped;

    // VL53L0X nominal max is ~2 m; use a broad physical gate and require device
    // status Range Valid. Bottom-ToF fusion can impose a tighter confidence gate.
    if (mapped == 0 && dist_mm > 0 && dist_mm <= 2000 && dist_mm != 0xFFFF) {
        s->last.distance_m = (float)dist_mm * 0.001f;
        s->last.valid = true;
        s->last_good_us = esp_timer_get_time();
    } else {
        s->last.valid = false;
    }

    return ESP_OK;
}

uint32_t tof_driver_stall_restarts(void) {
    return s_stall_restarts;
}

esp_err_t tof_driver_read(tof_reading_t *out) {
    if (!out) return ESP_ERR_INVALID_ARG;
    if (!s_sensor.dev) {
        memset(out, 0, sizeof(*out));
        out->valid = false;
        out->range_status = 255;
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = poll_sensor();
    tof_sensor_state_t *s = &s_sensor;

    if (err != ESP_OK) {
        // Keep the last good reading briefly across transient I2C errors.
        *out = s->last;
        if (out->valid && s->last_good_us > 0 &&
            (esp_timer_get_time() - s->last_good_us) >
                (int64_t)TOF_STALE_TIMEOUT_MS * 1000) {
            out->valid = false;
        }
        return err;
    }

    // If there has never been a good sample, keep invalid.
    if (s->last_good_us == 0) {
        s->last.valid = false;
    } else if ((esp_timer_get_time() - s->last_good_us) >
               (int64_t)TOF_STALE_TIMEOUT_MS * 1000) {
        s->last.valid = false;
        s->last.distance_m = 0.0f;
        s->last.range_status = 255;
    }

    *out = s->last;
    return ESP_OK;
}

#else   // !FC_FEATURE_TOF
// Cảm biến bị tắt lúc biên dịch. Một translation unit hoàn toàn rỗng là
// undefined behaviour theo ISO C, nên để lại đúng một khai báo vô hại.
typedef int tof_driver_disabled_at_compile_time_t;
#endif  // FC_FEATURE_TOF
