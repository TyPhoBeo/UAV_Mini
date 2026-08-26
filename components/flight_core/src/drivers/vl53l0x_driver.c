// vl53l0x_driver.c — backend cho DÒNG VL53L0X (thanh ghi 8-BIT, ID 0xC0=0xEE).
//
// Toàn bộ nội dung file này TRƯỚC ĐÂY nằm trong tof_driver.c. Nó được tách ra
// nguyên vẹn khi thêm hỗ trợ VL53L1X — xem tof_backend.h để biết vì sao hai họ
// chip KHÔNG thể dùng chung một file.
//
// Nguồn tham chiếu: datasheet ST VL53L0X + ST API, theo cách hiện thực của
// thư viện Pololu (bản rút gọn, chỉ giữ phần cần cho ranging liên tục).
//
// ⚠ FILE NÀY CHỈ ĐƯỢC BIÊN DỊCH KHI BOARD_TOF_CHIP == TOF_CHIP_VL53L0X.
// Chọn chip ở main/board_config.h.

#include "tof_backend.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "flight_core/fc_features.h"

#if FC_FEATURE_TOF && (BOARD_TOF_CHIP == TOF_CHIP_VL53L0X)

static const char *TAG = "vl53l0x";

#define VL53L0X_MODEL_ID_VALUE               0xEE
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

// stop_variable: state RIÊNG của chip L0X, đọc lúc DataInit và ghi lại trước
// khi vào ranging. Ở đây chứ không trong tof_sensor_state_t vì L1X không có
// khái niệm này — xem ghi chú "phần RIÊNG của từng chip" trong tof_backend.h.
static uint8_t s_stop_variable;

// -----------------------------------------------------------------------------
// Low-level I2C helpers — VL53L0X uses ONE-BYTE register addresses.
// -----------------------------------------------------------------------------
static esp_err_t write_reg8(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(dev, buf, sizeof(buf), tof_io_timeout_ms);
}

static esp_err_t write_reg16(i2c_master_dev_handle_t dev, uint8_t reg, uint16_t val) {
    uint8_t buf[3] = {
        reg,
        (uint8_t)(val >> 8),
        (uint8_t)(val & 0xFF),
    };
    return i2c_master_transmit(dev, buf, sizeof(buf), tof_io_timeout_ms);
}

static esp_err_t read_reg8(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *val) {
    return i2c_master_transmit_receive(dev, &reg, 1, val, 1, tof_io_timeout_ms);
}

static esp_err_t read_reg16(i2c_master_dev_handle_t dev, uint8_t reg, uint16_t *val) {
    uint8_t data[2] = {0};
    esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1, data, sizeof(data),
                                                tof_io_timeout_ms);
    if (err != ESP_OK) return err;
    *val = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    return ESP_OK;
}

static esp_err_t read_multi(i2c_master_dev_handle_t dev, uint8_t reg,
                            uint8_t *buf, size_t len) {
    return i2c_master_transmit_receive(dev, &reg, 1, buf, len, tof_io_timeout_ms);
}

static esp_err_t write_multi(i2c_master_dev_handle_t dev, uint8_t reg,
                             const uint8_t *data, size_t len) {
    if (len > 16) return ESP_ERR_INVALID_SIZE;
    uint8_t buf[17];
    buf[0] = reg;
    memcpy(&buf[1], data, len);
    return i2c_master_transmit(dev, buf, len + 1, tof_io_timeout_ms);
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

    int64_t deadline_us = esp_timer_get_time() + (int64_t)TOF_INIT_WAIT_TIMEOUT_MS * 1000;
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

    int64_t deadline_us = esp_timer_get_time() + (int64_t)TOF_INIT_WAIT_TIMEOUT_MS * 1000;
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

// =============================================================================
// GIAO DIỆN BACKEND (tof_backend.h)
// =============================================================================

// Địa chỉ I2C của VL53L0X: thanh ghi 0x8A nhận THẲNG địa chỉ 7-bit.
esp_err_t tof_backend_set_addr_l0x(i2c_master_dev_handle_t dev, uint8_t new_addr) {
    return write_reg8(dev, VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS,
                      (uint8_t)(new_addr & 0x7F));
}

// Chip CÓ ở địa chỉ này không? CHỈ ĐỌC — không ghi gì, vì facade còn dùng hàm
// này để dò trước khi được phép đổi địa chỉ.
bool tof_backend_probe_l0x(i2c_master_bus_handle_t bus, uint8_t addr,
                           uint32_t scl_hz, char *id_desc, size_t id_desc_len) {
    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = scl_hz,
    };
    i2c_master_dev_handle_t dev = NULL;
    if (i2c_master_bus_add_device(bus, &dev_cfg, &dev) != ESP_OK || dev == NULL) {
        return false;
    }

    uint8_t model = 0, revision = 0;
    const esp_err_t err = read_reg8(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &model);
    (void)read_reg8(dev, VL53L0X_REG_IDENTIFICATION_REVISION_ID, &revision);
    i2c_master_bus_rm_device(dev);

    if (id_desc && id_desc_len) {
        snprintf(id_desc, id_desc_len, "MODEL_ID=0x%02X REV=0x%02X (err=%s)",
                 model, revision, esp_err_to_name(err));
    }
    return (err == ESP_OK) && (model == VL53L0X_MODEL_ID_VALUE);
}

esp_err_t tof_backend_setup_l0x(tof_sensor_state_t *s) {
    if (!s || !s->dev) return ESP_ERR_INVALID_ARG;

    i2c_master_dev_handle_t dev = s->dev;
    esp_err_t err;
    uint8_t tmp = 0;

#define CHECK(expr_) do { err = (expr_); if (err != ESP_OK) { \
    ESP_LOGE(TAG, "VL53L0X init failed line %d: %s", __LINE__, esp_err_to_name(err)); \
    return err; } } while (0)

    // DataInit: configure 2V8 I/O mode and standard I2C mode.
    CHECK(read_reg8(dev, VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, &tmp));
    CHECK(write_reg8(dev, VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV,
                     (uint8_t)(tmp | 0x01)));
    CHECK(write_reg8(dev, 0x88, 0x00));

    // Read stop variable from private page.
    CHECK(write_reg8(dev, 0x80, 0x01));
    CHECK(write_reg8(dev, 0xFF, 0x01));
    CHECK(write_reg8(dev, 0x00, 0x00));
    CHECK(read_reg8(dev, 0x91, &s_stop_variable));
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
    CHECK(write_reg8(dev, 0x91, s_stop_variable));
    CHECK(write_reg8(dev, 0x00, 0x01));
    CHECK(write_reg8(dev, 0xFF, 0x00));
    CHECK(write_reg8(dev, 0x80, 0x00));
    CHECK(write_reg8(dev, VL53L0X_REG_SYSRANGE_START, VL53L0X_MODE_BACKTOBACK));

    s->ranging_started = true;

    ESP_LOGI(TAG, "VL53L0X ranging START tai 0x%02X (timing budget %lu us, tam TIN CAY ~1.2m)",
             s->addr, (unsigned long)VL53L0X_DEFAULT_TIMING_BUDGET_US);

#undef CHECK
    return ESP_OK;
}

// Phục hồi NHẸ khi watchdog thấy chip ngừng đo: clear cờ ngắt + phát lại lệnh
// đo liên tục. Hai lệnh ghi (~1ms), KHÔNG phải full re-init — xem lý do trong
// tof_driver.c.
esp_err_t tof_backend_restart_l0x(tof_sensor_state_t *s) {
    if (!s || !s->dev) return ESP_ERR_INVALID_STATE;
    (void)write_reg8(s->dev, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    return write_reg8(s->dev, VL53L0X_REG_SYSRANGE_START, VL53L0X_MODE_BACKTOBACK);
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

esp_err_t tof_backend_poll_l0x(tof_sensor_state_t *s) {
    if (!s || !s->dev || !s->ranging_started) return ESP_ERR_INVALID_STATE;

    // Only consume a NEW measurement. Do not repeatedly read/fuse the same one.
    uint8_t int_status = 0;
    esp_err_t err = read_reg8(s->dev, VL53L0X_REG_RESULT_INTERRUPT_STATUS, &int_status);
    if (err != ESP_OK) return err;
    if ((int_status & 0x07) == 0) {
        // Không có mẫu mới KHÔNG phải lỗi — facade lo watchdog.
        return ESP_OK;
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

#else   // backend không được chọn
// Một translation unit hoàn toàn rỗng là undefined behaviour theo ISO C.
typedef int vl53l0x_backend_not_selected_t;
#endif
