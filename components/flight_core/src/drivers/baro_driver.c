// BMP280 register map + compensation formula THẬT theo datasheet Bosch
// BMP280 (Rev 1.23), mục 3.11.3 "Compensation formulas". Giữ công thức
// double-precision theo Bosch — ESP32-S3 chỉ có hardware FPU cho single
// precision; double được xử lý bằng software (soft-float) nhưng barometer
// chạy ở tần số thấp (~25-50Hz, throttle ở flight_core.c) nên chi phí tính
// toán hiện tại chấp nhận được.
#include "flight_core/drivers/baro_driver.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "flight_core/tuning.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Toàn bộ thân driver nằm sau cờ biên dịch này. Cờ = 0 (từ
// SENSOR_BARO_ENABLED trong main/app_config.h, xem fc_features.h) thì file
// này biên dịch ra một object RỖNG — driver không tốn byte flash nào.
#if FC_FEATURE_BARO

static const char *TAG = "baro_driver";

#define BMP280_REG_CALIB_START   0x88   // 24 byte: dig_T1..T3, dig_P1..P9 (little-endian)
#define BMP280_REG_CHIP_ID       0xD0
#define BMP280_REG_RESET         0xE0
#define BMP280_REG_STATUS        0xF3
#define BMP280_REG_CTRL_MEAS     0xF4
#define BMP280_REG_CONFIG        0xF5
#define BMP280_REG_PRESS_MSB     0xF7   // burst 6 byte: press(3) + temp(3), big-endian 20-bit

#define BMP280_CHIP_ID_VALUE     0x58
#define BMP280_SOFT_RESET_VALUE  0xB6
#define BMP280_STATUS_IM_UPDATE_BIT   0x01   // bit0: 1 = đang copy NVM->register, CHƯA đọc calib được

// Custom UAV configuration — KHÔNG phải nguyên bản preset "drop detection"
// của Bosch, lựa chọn riêng cân bằng noise/latency cho project này (xem
// datasheet mục 3.3.3 nếu muốn đổi profile khác):
//   temperature oversampling x1, pressure oversampling x8, normal mode,
//   standby 0.5ms, IIR filter coefficient 8.
//
// NÂNG từ (T x1, P x4, IIR x4) — estimator giờ accel-primary (KHÔNG có ToF
// kéo lại, xem alt_estimator.h), baro là anchor DUY NHẤT chống trôi dài hạn,
// nên đáng đánh đổi latency lấy noise thấp hơn: đo lâu hơn (~22.5ms/mẫu ở P
// x8, so với ~11ms ở P x4 — vẫn nằm trong khoảng 25-50Hz baro dự kiến của
// kiến trúc này) nhưng IIRx8 giảm noise đáng kể so với IIRx4. LPF phần mềm
// trong alt_estimator.h (ALT_EST_BARO_LPF_HZ) + alpha/beta CỰC nhỏ đã lọc
// thêm 1 lớp nữa, nhưng lọc CÀNG NHIỀU TỪ PHẦN CỨNG càng tốt (cùng nguyên
// tắc DLPF MPU6050 — xem tuning.h mục 11) vì innovation gate + alpha nhỏ chỉ
// hoạt động tốt khi tín hiệu vào đã tương đối sạch.
#define BMP280_CTRL_MEAS_VALUE   0x33   // osrs_t(001)<<5 | osrs_p(100=x8)<<2 | mode(11=normal)
#define BMP280_CONFIG_VALUE      0x0C   // t_sb(000)<<5 | filter(011=IIRx8)<<2 | spi3w_en(0)

#define BMP280_NVM_READY_TIMEOUT_MS   100
#define BMP280_NVM_POLL_INTERVAL_MS   5

// Chờ pressure/IIR filter ổn định trước khi lấy mốc 0m — chip vừa reset (hoặc
// gọi lại calibrate_ground() sau 1 khoảng dài) cần thời gian để output hết
// transient, xem baro_driver.h.
#define BARO_GROUND_SETTLE_DELAY_MS   300
#define BARO_GROUND_REF_SAMPLES       32
#define BARO_GROUND_REF_DELAY_MS      20
#define BARO_GROUND_REF_MIN_VALID     (BARO_GROUND_REF_SAMPLES / 2)

// Dải áp suất hợp lý theo datasheet BMP280 (300-1100 hPa) — mẫu ngoài dải này
// chắc chắn là dữ liệu rác (bus lỗi thoáng qua nhưng transaction vẫn "OK").
#define BARO_PRESSURE_MIN_PA   30000.0
#define BARO_PRESSURE_MAX_PA   110000.0

// Tốc độ SCL đến từ tham số scl_speed_hz của baro_driver_init() (nguồn:
// BOARD_I2C_FREQ_HZ) — KHÔNG #define ở đây nữa.

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
} bmp280_calib_t;

static i2c_master_dev_handle_t s_dev = NULL;
static bmp280_calib_t          s_calib;
static double                   s_ground_pressure_pa = 0.0;
static float                    s_ground_noise_std_pa = 0.0f;
static bool                     s_ready = false;             // chip vận hành được (init xong)
static bool                     s_ground_ref_ready = false;   // calibrate_ground() thành công

// Timeout I2C: INIT rộng (chạy 1 lần lúc boot + đọc blob calib), RUNTIME hẹp
// (chạy trong sensor_hub). Lý do đầy đủ: xem khối cùng tên ở đầu imu_driver.c.
// baro_driver_calibrate_ground() chạy sau init nên dùng mức RUNTIME — đúng, vì
// phần chờ LÂU của nó là vTaskDelay giữa các mẫu, không phải một transaction.
#define BARO_I2C_INIT_TIMEOUT_MS      50
#define BARO_I2C_RUNTIME_TIMEOUT_MS   8

static int s_io_timeout_ms = BARO_I2C_INIT_TIMEOUT_MS;

static esp_err_t write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_dev, buf, sizeof(buf), s_io_timeout_ms);
}

static esp_err_t read_regs(uint8_t reg, uint8_t *buf, size_t len) {
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, len, s_io_timeout_ms);
}

static inline int16_t le16_signed(const uint8_t *p) {
    return (int16_t)(p[0] | (p[1] << 8));
}
static inline uint16_t le16_unsigned(const uint8_t *p) {
    return (uint16_t)(p[0] | (p[1] << 8));
}

// compensate_temperature() — trả nhiệt độ °C VÀ ghi t_fine ra *t_fine_out (bắt
// buộc cho compensate_pressure() dùng đúng theo datasheet, coupling này là
// THIẾT KẾ GỐC của Bosch, không phải bug).
static double compensate_temperature(int32_t adc_T, const bmp280_calib_t *c, double *t_fine_out) {
    double var1 = (((double)adc_T) / 16384.0 - ((double)c->dig_T1) / 1024.0) * ((double)c->dig_T2);
    double var2 = ((((double)adc_T) / 131072.0 - ((double)c->dig_T1) / 8192.0) *
                   (((double)adc_T) / 131072.0 - ((double)c->dig_T1) / 8192.0)) * ((double)c->dig_T3);
    double t_fine = var1 + var2;
    *t_fine_out = t_fine;
    return t_fine / 5120.0;
}

static double compensate_pressure(int32_t adc_P, const bmp280_calib_t *c, double t_fine) {
    double var1 = (t_fine / 2.0) - 64000.0;
    double var2 = var1 * var1 * ((double)c->dig_P6) / 32768.0;
    var2 = var2 + var1 * ((double)c->dig_P5) * 2.0;
    var2 = (var2 / 4.0) + (((double)c->dig_P4) * 65536.0);
    var1 = (((double)c->dig_P3) * var1 * var1 / 524288.0 + ((double)c->dig_P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)c->dig_P1);
    if (var1 == 0.0) return 0.0;   // tránh chia 0 (datasheet khuyến cáo, thực tế hiếm xảy ra)

    double p = 1048576.0 - (double)adc_P;
    p = (p - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((double)c->dig_P9) * p * p / 2147483648.0;
    var2 = p * ((double)c->dig_P8) / 32768.0;
    p = p + (var1 + var2 + ((double)c->dig_P7)) / 16.0;
    return p;   // Pa
}

// wait_nvm_ready() — poll STATUS.im_update tới khi = 0 (NVM đã copy xong vào
// register, đọc calib block mới có nghĩa) hoặc timeout — KHÔNG dựa vào delay
// cố định sau soft-reset (thời gian NVM copy không đảm bảo cố định).
static esp_err_t wait_nvm_ready(void) {
    const int max_polls = BMP280_NVM_READY_TIMEOUT_MS / BMP280_NVM_POLL_INTERVAL_MS;
    for (int i = 0; i < max_polls; i++) {
        uint8_t status = 0;
        if (read_regs(BMP280_REG_STATUS, &status, 1) == ESP_OK &&
            !(status & BMP280_STATUS_IM_UPDATE_BIT)) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(BMP280_NVM_POLL_INTERVAL_MS));
    }
    ESP_LOGE(TAG, "cho NVM copy (STATUS.im_update) qua %dms ma chua xong", BMP280_NVM_READY_TIMEOUT_MS);
    return ESP_ERR_TIMEOUT;
}

// read_raw_press_temp() — burst-read 6 byte + ghép raw 20-bit adc_P/adc_T,
// dùng chung cho cả calibrate_ground() lẫn baro_driver_read().
static esp_err_t read_raw_press_temp(int32_t *adc_P, int32_t *adc_T) {
    uint8_t raw[6];
    esp_err_t err = read_regs(BMP280_REG_PRESS_MSB, raw, sizeof(raw));
    if (err != ESP_OK) return err;
    *adc_P = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);
    *adc_T = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | (raw[5] >> 4);
    return ESP_OK;
}

esp_err_t baro_driver_init(i2c_master_bus_handle_t bus, uint8_t i2c_addr,
                            uint32_t scl_speed_hz) {
    s_ready = false;
    s_ground_ref_ready = false;
    s_dev = NULL;

    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_addr,
        .scl_speed_hz = scl_speed_hz,
    };
    ESP_LOGI(TAG, "BMP280 @0x%02X, SCL %lu Hz", i2c_addr, (unsigned long)scl_speed_hz);
    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "them device BMP280 tai 0x%02X vao bus that bai: %s",
                  i2c_addr, esp_err_to_name(err));
        s_dev = NULL;
        return err;
    }

    uint8_t chip_id = 0;
    err = read_regs(BMP280_REG_CHIP_ID, &chip_id, 1);
    if (err != ESP_OK) {
        // Lỗi giao dịch I2C THẬT (timeout/nack/...) -> propagate NGUYÊN VẸN,
        // KHÔNG ép thành ESP_ERR_NOT_FOUND (giá trị đó CHỈ dành cho "chip sai"
        // ở nhánh dưới) — quan trọng để debug đúng nguyên nhân khi 1 sensor
        // làm hỏng cả bus, không nhầm với "board không hàn chip".
        ESP_LOGE(TAG, "doc CHIP_ID that bai tai 0x%02X: %s", i2c_addr, esp_err_to_name(err));
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        return err;
    }
    if (chip_id != BMP280_CHIP_ID_VALUE) {
        ESP_LOGW(TAG, "CHIP_ID sai: doc 0x%02X, ky vong 0x%02X tai 0x%02X -> coi nhu KHONG co baro "
                      "(kiem tra lai dia chi SDO trong board_config.h)",
                  chip_id, BMP280_CHIP_ID_VALUE, i2c_addr);
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        return ESP_ERR_NOT_FOUND;
    }

    err = write_reg(BMP280_REG_RESET, BMP280_SOFT_RESET_VALUE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "soft-reset that bai: %s", esp_err_to_name(err));
        return err;
    }

    err = wait_nvm_ready();
    if (err != ESP_OK) return err;

    uint8_t calib_raw[24];
    err = read_regs(BMP280_REG_CALIB_START, calib_raw, sizeof(calib_raw));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "doc bang hieu chinh that bai: %s", esp_err_to_name(err));
        return err;
    }

    bmp280_calib_t calib;
    calib.dig_T1 = le16_unsigned(&calib_raw[0]);
    calib.dig_T2 = le16_signed(&calib_raw[2]);
    calib.dig_T3 = le16_signed(&calib_raw[4]);
    calib.dig_P1 = le16_unsigned(&calib_raw[6]);
    calib.dig_P2 = le16_signed(&calib_raw[8]);
    calib.dig_P3 = le16_signed(&calib_raw[10]);
    calib.dig_P4 = le16_signed(&calib_raw[12]);
    calib.dig_P5 = le16_signed(&calib_raw[14]);
    calib.dig_P6 = le16_signed(&calib_raw[16]);
    calib.dig_P7 = le16_signed(&calib_raw[18]);
    calib.dig_P8 = le16_signed(&calib_raw[20]);
    calib.dig_P9 = le16_signed(&calib_raw[22]);

    // Validate: dig_P1==0 chắc chắn sai theo datasheet (dùng làm mẫu số ở
    // compensate_pressure()); toàn 0x00 hoặc toàn 0xFF trong 24 byte thô là
    // dấu hiệu kinh điển của bus lỗi/chip chưa thật sự sẵn sàng dù transaction
    // "ACK thành công".
    bool all_zero = true, all_ff = true;
    for (size_t i = 0; i < sizeof(calib_raw); i++) {
        if (calib_raw[i] != 0x00) all_zero = false;
        if (calib_raw[i] != 0xFF) all_ff = false;
    }
    if (calib.dig_P1 == 0 || all_zero || all_ff) {
        ESP_LOGE(TAG, "bang hieu chinh khong hop le (dig_P1=%u all_zero=%d all_ff=%d) -> KHONG dung",
                  calib.dig_P1, (int)all_zero, (int)all_ff);
        return ESP_ERR_INVALID_RESPONSE;
    }
    s_calib = calib;

    err = write_reg(BMP280_REG_CONFIG, BMP280_CONFIG_VALUE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ghi CONFIG that bai: %s", esp_err_to_name(err));
        return err;
    }
    err = write_reg(BMP280_REG_CTRL_MEAS, BMP280_CTRL_MEAS_VALUE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ghi CTRL_MEAS that bai: %s", esp_err_to_name(err));
        return err;
    }

    // Read-back verify — CHỈ set s_ready khi CONFIG/CTRL_MEAS đọc lại đúng
    // giá trị vừa ghi (chống trường hợp ACK "thành công" nhưng chip/bus không
    // thật sự nhận cấu hình).
    uint8_t config_rb = 0, ctrl_meas_rb = 0;
    err = read_regs(BMP280_REG_CONFIG, &config_rb, 1);
    if (err != ESP_OK) { ESP_LOGE(TAG, "doc lai CONFIG that bai: %s", esp_err_to_name(err)); return err; }
    err = read_regs(BMP280_REG_CTRL_MEAS, &ctrl_meas_rb, 1);
    if (err != ESP_OK) { ESP_LOGE(TAG, "doc lai CTRL_MEAS that bai: %s", esp_err_to_name(err)); return err; }

    if (config_rb != BMP280_CONFIG_VALUE || ctrl_meas_rb != BMP280_CTRL_MEAS_VALUE) {
        ESP_LOGE(TAG, "read-back sai: CONFIG=0x%02X (ky vong 0x%02X) CTRL_MEAS=0x%02X (ky vong 0x%02X) "
                      "-> KHONG bat s_ready", config_rb, BMP280_CONFIG_VALUE, ctrl_meas_rb, BMP280_CTRL_MEAS_VALUE);
        return ESP_ERR_INVALID_RESPONSE;
    }

    s_ready = true;
    // Từ đây mọi transaction là runtime (baro_driver_read() trong sensor_hub).
    s_io_timeout_ms = BARO_I2C_RUNTIME_TIMEOUT_MS;
    ESP_LOGI(TAG, "BMP280 init OK tai 0x%02X (CONFIG=0x%02X CTRL_MEAS=0x%02X, I2C timeout "
                  "runtime=%dms) -- CHUA co moc 0m, goi baro_driver_calibrate_ground() truoc "
                  "khi dung alt_m", i2c_addr, config_rb, ctrl_meas_rb, BARO_I2C_RUNTIME_TIMEOUT_MS);
    return ESP_OK;
}

esp_err_t baro_driver_calibrate_ground(void) {
    if (!s_ready) {
        ESP_LOGE(TAG, "calibrate_ground() goi truoc khi init thanh cong");
        return ESP_ERR_INVALID_STATE;
    }

    s_ground_ref_ready = false;

    // Chờ pressure/IIR filter ổn định trước khi lấy mẫu — xem baro_driver.h.
    vTaskDelay(pdMS_TO_TICKS(BARO_GROUND_SETTLE_DELAY_MS));

    // Lưu mẫu hợp lệ vào mảng (thay vì chỉ cộng dồn sum) để tính std-dev
    // 2-pass CHÍNH XÁC (tránh mất số do bình phương giá trị lớn ~100000 Pa
    // trong công thức 1-pass E[X^2]-E[X]^2) — BARO_GROUND_REF_SAMPLES nhỏ
    // (32), tốn stack không đáng kể.
    double samples_pa[BARO_GROUND_REF_SAMPLES];
    double sum_pa = 0.0;
    int    got = 0;
    for (int i = 0; i < BARO_GROUND_REF_SAMPLES; i++) {
        vTaskDelay(pdMS_TO_TICKS(BARO_GROUND_REF_DELAY_MS));

        int32_t adc_P = 0, adc_T = 0;
        if (read_raw_press_temp(&adc_P, &adc_T) != ESP_OK) continue;   // bỏ mẫu lỗi I2C

        double t_fine = 0.0;
        (void)compensate_temperature(adc_T, &s_calib, &t_fine);
        const double press_pa = compensate_pressure(adc_P, &s_calib, t_fine);

        if (!isfinite(press_pa) || press_pa < BARO_PRESSURE_MIN_PA || press_pa > BARO_PRESSURE_MAX_PA) {
            continue;   // bỏ mẫu ngoài dải hợp lý (dữ liệu rác)
        }
        samples_pa[got] = press_pa;
        sum_pa += press_pa;
        got++;
    }
    if (got < BARO_GROUND_REF_MIN_VALID) {
        ESP_LOGE(TAG, "lay ap suat nen that bai (chi %d/%d mau hop le, can toi thieu %d) -> KHONG co moc 0m",
                  got, BARO_GROUND_REF_SAMPLES, BARO_GROUND_REF_MIN_VALID);
        return ESP_ERR_INVALID_RESPONSE;
    }
    s_ground_pressure_pa = sum_pa / (double)got;

    double sum_sq_dev = 0.0;
    for (int i = 0; i < got; i++) {
        const double dev = samples_pa[i] - s_ground_pressure_pa;
        sum_sq_dev += dev * dev;
    }
    s_ground_noise_std_pa = (float)sqrt(sum_sq_dev / (double)got);
    s_ground_ref_ready = true;

    const bool healthy = s_ground_noise_std_pa <= BARO_GROUND_NOISE_STD_MAX_PA;
    if (healthy) {
        ESP_LOGI(TAG, "BMP280 calibrate_ground OK (ap suat nen=%.1f Pa, std=%.2f Pa, %d/%d mau hop le)",
                  s_ground_pressure_pa, (double)s_ground_noise_std_pa, got, BARO_GROUND_REF_SAMPLES);
    } else {
        // KHÔNG trả lỗi (vẫn có mốc 0m dùng được) — chỉ cảnh báo UNHEALTHY,
        // caller (flight_core.c) tự quyết định có tin baro cho alt_hold/
        // takeoff hay không (xem baro_driver_ground_healthy()).
        ESP_LOGW(TAG, "BMP280 calibrate_ground: std=%.2f Pa VUOT nguong %.2f Pa -> UNHEALTHY "
                      "(drone dang rung/gio/di chuyen luc calib? lam lai khi that su dung yen)",
                  (double)s_ground_noise_std_pa, (double)BARO_GROUND_NOISE_STD_MAX_PA);
    }
    return ESP_OK;
}

bool baro_driver_ground_ready(void) { return s_ground_ref_ready; }

float baro_driver_ground_noise_std_pa(void) { return s_ground_noise_std_pa; }

bool baro_driver_ground_healthy(void) {
    return s_ground_ref_ready && (s_ground_noise_std_pa <= BARO_GROUND_NOISE_STD_MAX_PA);
}

esp_err_t baro_driver_read(baro_sample_t *out) {
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_ready || !s_ground_ref_ready) {
        memset(out, 0, sizeof(*out));
        out->ok = false;
        return ESP_ERR_NOT_FOUND;
    }

    int32_t adc_P = 0, adc_T = 0;
    esp_err_t err = read_raw_press_temp(&adc_P, &adc_T);
    if (err != ESP_OK) {
        memset(out, 0, sizeof(*out));
        out->ok = false;
        return err;
    }

    double t_fine = 0.0;
    const double temp_c = compensate_temperature(adc_T, &s_calib, &t_fine);
    const double press_pa = compensate_pressure(adc_P, &s_calib, t_fine);

    if (!isfinite(press_pa) || press_pa < BARO_PRESSURE_MIN_PA || press_pa > BARO_PRESSURE_MAX_PA) {
        memset(out, 0, sizeof(*out));
        out->ok = false;
        return ESP_ERR_INVALID_RESPONSE;
    }

    out->pressure_pa = (float)press_pa;
    out->temperature_c = (float)temp_c;
    // Công thức khí áp chuẩn (barometric formula), tương đối so với mốc
    // calibrate_ground() — xem baro_driver.h. KHÔNG phải độ cao MSL tuyệt đối.
    // BARO_ALT_SCALE (tuning.h mục 9) — hệ số hiệu chỉnh sai số hệ thống so
    // với độ cao thật đo bằng thước, mặc định 1.0 (chưa hiệu chỉnh).
    out->alt_m = 44330.0f * (1.0f - powf((float)(press_pa / s_ground_pressure_pa), 0.190294957f))
                 * BARO_ALT_SCALE;
    out->ok = true;
    return ESP_OK;
}

#else   // !FC_FEATURE_BARO
// Cảm biến bị tắt lúc biên dịch. Một translation unit hoàn toàn rỗng là
// undefined behaviour theo ISO C, nên để lại đúng một khai báo vô hại.
typedef int baro_driver_disabled_at_compile_time_t;
#endif  // FC_FEATURE_BARO
