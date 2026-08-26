// MPU6050 register map thật — WHO_AM_I probe, DEVICE_RESET, wake+clock-source,
// DLPF, full-scale range, đọc burst 14 byte (accel+temp+gyro), READ-BACK
// VERIFY toàn bộ config sau khi ghi. Công thức hiệu chỉnh (scale + trừ bias)
// GIỮ NGUYÊN từ MPUDriver::convert() (UAV-Mini, đã đúng).
//
// CHƯA remap trục sensor->body — hướng lắp IMU thật trên khung CHƯA xác nhận
// (giống vấn đề mapping motor CH1..CH4, xem README.md). Map THẲNG x/y/z tạm
// thời; nếu Mahony hội tụ sai chiều roll/pitch khi test trên bàn (nghiêng bo
// mà số đọc lệch trục), sửa dấu/hoán trục ở NGAY dưới đây (đánh dấu TODO).
// LƯU Ý nếu remap sau này: accel và gyro PHẢI dùng CÙNG MỘT rotation mapping.
#include "flight_core/drivers/imu_driver.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "flight_core/tuning.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "imu_driver";

#define MPU6050_REG_SMPLRT_DIV    0x19
#define MPU6050_REG_CONFIG        0x1A
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_INT_PIN_CFG   0x37
#define MPU6050_REG_INT_ENABLE    0x38
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_WHO_AM_I      0x75
#define MPU6050_WHO_AM_I_VALUE    0x68   // cố định theo chip, ĐỘC LẬP với AD0/địa chỉ I2C

#define MPU6050_PWR1_DEVICE_RESET   0x80
#define MPU6050_PWR1_WAKE_PLL_XGYRO 0x01   // SLEEP=0, CLKSEL=1 (PLL X-gyro reference)

// DLPF_CFG lấy từ tuning.h (IMU_ACCEL_DLPF_CFG mục 11) — TUNE Ở ĐÓ, không sửa
// giá trị ở đây. Xem bảng bandwidth/delay đầy đủ trong tuning.h — đây là LỚP
// LỌC RUNG ĐỘNG CƠ CHÍNH cho cả accel (tích phân Vz, estimator giờ
// accel-primary — xem alt_estimator.h) LẪN gyro (attitude PID).
#if (IMU_ACCEL_DLPF_CFG) < 0 || (IMU_ACCEL_DLPF_CFG) > 6
#error "IMU_ACCEL_DLPF_CFG (tuning.h) phai trong [0,6] -- 7 la gia tri du phong khong dinh nghia theo datasheet MPU6050"
#endif
#define MPU6050_CONFIG_DLPF_CFG_VALUE   ((uint8_t)(IMU_ACCEL_DLPF_CFG))
// FS_SEL=3 (±2000dps, 16.4 LSB/dps) khớp IMU_GYRO_RAW_PER_DPS.
#define MPU6050_GYRO_CONFIG_FS2000 0x18
// AFS_SEL=3 (±16g, 2048 LSB/g) khớp IMU_ACCEL_RAW_PER_G.
#define MPU6050_ACCEL_CONFIG_FS16G 0x18
// Khi DLPF bật, gyro output rate gốc là 1kHz (KHÔNG phải 8kHz) ->
// sample_rate = 1000/(1+SMPLRT_DIV). sensor_hub đọc ở đúng nhịp này, gom
// IMU_SAMPLES_PER_CONTROL mẫu rồi mới đánh thức stabilize_task 250Hz. Quan hệ
// hai nhịp được _Static_assert trong flight_core.c.
//
// Khi rơi về polling (INT hỏng), hub vẫn đọc 1kHz theo FreeRTOS tick 1ms và áp
// dụng cùng batch; timestamp/stale watchdog vẫn hoạt động như đường ngắt.
#define MPU6050_SMPLRT_DIV_VALUE   ((uint8_t)((1000 / IMU_SAMPLE_RATE_HZ) - 1))

// INT_PIN_CFG: INT_LEVEL=0 (active high), INT_OPEN=0 (push-pull),
// LATCH_INT_EN=0 (xung 50us, tự hạ — hợp với ngắt theo CẠNH LÊN, không cần
// clear thủ công), INT_RD_CLEAR=1 (bit4: đọc bất kỳ thanh ghi nào cũng clear
// INT_STATUS — lớp bảo hiểm để không bao giờ kẹt cờ ngắt).
#define MPU6050_INT_PIN_CFG_VALUE   0x10
// INT_ENABLE bit0 = DATA_RDY_EN.
#define MPU6050_INT_ENABLE_DATA_RDY 0x01

#define MPU6050_RESET_DELAY_MS       100   // datasheet: chờ reset xong trước khi ghi tiếp
#define MPU6050_CLOCK_SETTLE_DELAY_MS 10    // chờ PLL clock ổn định sau khi thoát sleep

// Tốc độ SCL KHÔNG còn #define ở đây — nó đến từ tham số scl_speed_hz của
// imu_driver_init() (nguồn: BOARD_I2C_FREQ_HZ, xem imu_driver.h).

// ============================================================================
// TIMEOUT I2C — HAI GIÁ TRỊ, HAI MỤC ĐÍCH
// ============================================================================
// INIT (50ms): chạy MỘT LẦN lúc boot, không ai đang chờ. Rộng rãi để chip chậm
// khởi động (reset 100ms, PLL settle) không bị kết luận nhầm là "không có".
//
// RUNTIME (8ms): chạy 250 LẦN MỖI GIÂY trong sensor_hub, và mỗi ms chờ ở đây
// là một ms vòng bay không có mẫu mới. Một transaction 14 byte ở 400kHz mất
// ~0.4ms — 8ms đã là 20 lần dư. Giữ 50ms ở runtime nghĩa là một lần bus glitch
// làm hub đứng 12 chu kỳ điều khiển liên tiếp (và với timeout 8ms là 2 chu kỳ,
// hub kịp publish mark_err để tầng bay thấy stale ĐÚNG LÚC nó xảy ra thay vì
// 50ms sau).
//
// Đây KHÔNG phải retry: quá hạn -> trả lỗi -> hub mark_err -> vòng sau đọc lại.
// Bus thật sự chết thì consecutive_errors vượt ngưỡng và healthy=false, đúng
// đường đã có sẵn.
#define IMU_I2C_INIT_TIMEOUT_MS      50
#define IMU_I2C_RUNTIME_TIMEOUT_MS   8

static i2c_master_dev_handle_t s_dev = NULL;
static bool                     s_ready = false;
// Đổi sang RUNTIME ở CUỐI imu_driver_init() (sau khi mọi ghi/verify đã xong).
static int                      s_io_timeout_ms = IMU_I2C_INIT_TIMEOUT_MS;

// Cấu hình ĐỌC LẠI từ chip + scale suy ra từ chính nó. imu_driver_read() dùng
// s_cfg.gyro_lsb_per_dps / s_cfg.accel_lsb_per_g, KHÔNG dùng hằng số biên dịch.
static imu_cfg_readback_t       s_cfg;

// Bảng tra scale theo datasheet MPU6050 (Register Map mục 4.4 / 4.5).
// Index = FS_SEL / AFS_SEL (2 bit).
static const float k_gyro_lsb_per_dps[4]  = { 131.0f, 65.5f, 32.8f, 16.4f };
static const float k_accel_lsb_per_g[4]   = { 16384.0f, 8192.0f, 4096.0f, 2048.0f };
static const int   k_gyro_fs_dps[4]       = { 250, 500, 1000, 2000 };
static const int   k_accel_fs_g[4]        = { 2, 4, 8, 16 };

// Một transform dùng chung cho CẢ gyro lẫn accel. Hướng lắp hiện chưa được
// xác nhận nên giữ identity; khi có dữ liệu bench chỉ sửa helper này, không
// chạm calibration (bias vẫn luôn được đo/trừ trong SENSOR frame).
static vec3f_t sensor_to_body(vec3f_t sensor) {
    return sensor;
}

static esp_err_t write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_dev, buf, sizeof(buf), s_io_timeout_ms);
}

static esp_err_t read_regs(uint8_t reg, uint8_t *buf, size_t len) {
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, len, s_io_timeout_ms);
}

// write_verify_reg() — ghi 1 thanh ghi rồi đọc lại NGAY, so khớp giá trị vừa
// ghi. Dùng cho toàn bộ config MPU6050 (CONFIG/GYRO_CONFIG/ACCEL_CONFIG/
// SMPLRT_DIV) — 1 thanh ghi verify sai là init_failed, KHÔNG chỉ log rồi bỏ
// qua (chống trường hợp ACK "thành công" nhưng chip/bus không thật sự nhận).
static esp_err_t write_verify_reg(uint8_t reg, uint8_t val, const char *name) {
    esp_err_t err = write_reg(reg, val);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ghi %s (reg 0x%02X = 0x%02X) that bai: %s", name, reg, val, esp_err_to_name(err));
        return err;
    }
    uint8_t readback = 0;
    err = read_regs(reg, &readback, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "doc lai %s (reg 0x%02X) that bai: %s", name, reg, esp_err_to_name(err));
        return err;
    }
    if (readback != val) {
        ESP_LOGE(TAG, "read-back %s sai: ghi 0x%02X, doc lai 0x%02X tai reg 0x%02X",
                  name, val, readback, reg);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

esp_err_t imu_driver_init(i2c_master_bus_handle_t bus, uint8_t i2c_addr,
                           uint32_t scl_speed_hz) {
    s_ready = false;
    s_dev = NULL;

    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_addr,
        .scl_speed_hz = scl_speed_hz,
    };
    ESP_LOGI(TAG, "MPU6050 @0x%02X, SCL %lu Hz", i2c_addr, (unsigned long)scl_speed_hz);
    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "them device MPU6050 tai 0x%02X vao bus that bai: %s",
                  i2c_addr, esp_err_to_name(err));
        s_dev = NULL;
        return err;
    }

    uint8_t who = 0;
    err = read_regs(MPU6050_REG_WHO_AM_I, &who, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "doc WHO_AM_I that bai tai 0x%02X: %s", i2c_addr, esp_err_to_name(err));
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        return err;
    }
    if (who != MPU6050_WHO_AM_I_VALUE) {
        ESP_LOGE(TAG, "WHO_AM_I sai: doc 0x%02X, ky vong 0x%02X -> khong phai MPU6050 tai 0x%02X",
                  who, MPU6050_WHO_AM_I_VALUE, i2c_addr);
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        return ESP_ERR_NOT_FOUND;
    }

    // DEVICE_RESET: đưa toàn bộ thanh ghi về default (kể cả PWR_MGMT_1 tự về
    // SLEEP=1) trước khi cấu hình lại — tránh kế thừa state lạ từ lần chạy
    // trước hoặc bootloader/power-glitch.
    err = write_reg(MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "DEVICE_RESET that bai: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(MPU6050_RESET_DELAY_MS));

    // Thoát sleep + chọn PLL X-gyro làm clock reference (chính xác hơn dao
    // động nội RC).
    err = write_reg(MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_WAKE_PLL_XGYRO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "wake+PLL X-gyro that bai: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(MPU6050_CLOCK_SETTLE_DELAY_MS));

    // Read-back verify PWR_MGMT_1 riêng (không dùng write_verify_reg() vì đã
    // ghi + delay ở trên, chỉ cần xác nhận lại).
    uint8_t pwr1_rb = 0;
    err = read_regs(MPU6050_REG_PWR_MGMT_1, &pwr1_rb, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "doc lai PWR_MGMT_1 that bai: %s", esp_err_to_name(err));
        return err;
    }
    if (pwr1_rb != MPU6050_PWR1_WAKE_PLL_XGYRO) {
        ESP_LOGE(TAG, "read-back PWR_MGMT_1 sai: ghi 0x%02X, doc lai 0x%02X",
                  MPU6050_PWR1_WAKE_PLL_XGYRO, pwr1_rb);
        return ESP_ERR_INVALID_RESPONSE;
    }

    if ((err = write_verify_reg(MPU6050_REG_CONFIG, MPU6050_CONFIG_DLPF_CFG_VALUE, "CONFIG")) != ESP_OK) return err;
    if ((err = write_verify_reg(MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_CONFIG_FS2000, "GYRO_CONFIG")) != ESP_OK) return err;
    if ((err = write_verify_reg(MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_CONFIG_FS16G, "ACCEL_CONFIG")) != ESP_OK) return err;
    if ((err = write_verify_reg(MPU6050_REG_SMPLRT_DIV, MPU6050_SMPLRT_DIV_VALUE, "SMPLRT_DIV")) != ESP_OK) return err;

    // ================= READ-BACK TOÀN BỘ CONFIG + SUY RA SCALE =================
    // write_verify_reg() ở trên đã so từng thanh ghi ngay lúc ghi, nhưng đó là
    // "ghi có dính không". Bước này trả lời câu KHÁC: "chip ĐANG chạy ở scale
    // nào" — và scale đó (không phải hằng số biên dịch) mới là thứ
    // imu_driver_read() dùng để convert. Đọc LẠI một lượt cuối, sau khi mọi
    // thanh ghi đã ổn định, rồi decode từ chính các bit vừa đọc.
    memset(&s_cfg, 0, sizeof(s_cfg));
    s_cfg.who_am_i = who;

    struct { uint8_t reg; uint8_t *dst; const char *name; } rb[] = {
        { MPU6050_REG_CONFIG,       &s_cfg.config,       "CONFIG"       },
        { MPU6050_REG_GYRO_CONFIG,  &s_cfg.gyro_config,  "GYRO_CONFIG"  },
        { MPU6050_REG_ACCEL_CONFIG, &s_cfg.accel_config, "ACCEL_CONFIG" },
        { MPU6050_REG_SMPLRT_DIV,   &s_cfg.smplrt_div,   "SMPLRT_DIV"   },
        { MPU6050_REG_PWR_MGMT_1,   &s_cfg.pwr_mgmt_1,   "PWR_MGMT_1"   },
    };
    for (size_t i = 0; i < sizeof(rb) / sizeof(rb[0]); i++) {
        err = read_regs(rb[i].reg, rb[i].dst, 1);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "read-back cuoi %s (reg 0x%02X) that bai: %s -> config KHONG TIN DUOC",
                      rb[i].name, rb[i].reg, esp_err_to_name(err));
            return err;
        }
    }

    // Decode bit -> scale. FS_SEL = GYRO_CONFIG[4:3], AFS_SEL = ACCEL_CONFIG[4:3],
    // DLPF_CFG = CONFIG[2:0]. Mask 0x03 sau khi dịch nên index luôn trong [0,3]
    // -> không thể tràn bảng tra dù thanh ghi đọc về giá trị lạ.
    s_cfg.fs_sel   = (uint8_t)((s_cfg.gyro_config  >> 3) & 0x03);
    s_cfg.afs_sel  = (uint8_t)((s_cfg.accel_config >> 3) & 0x03);
    s_cfg.dlpf_cfg = (uint8_t)(s_cfg.config & 0x07);
    s_cfg.gyro_lsb_per_dps = k_gyro_lsb_per_dps[s_cfg.fs_sel];
    s_cfg.accel_lsb_per_g  = k_accel_lsb_per_g[s_cfg.afs_sel];
    s_cfg.sample_rate_hz   = 1000 / (1 + (int)s_cfg.smplrt_div);

    ESP_LOGI(TAG, "MPU6050 CFG (READ-BACK tu chip, KHONG phai hang so bien dich):");
    ESP_LOGI(TAG, "  WHO_AM_I=0x%02X  PWR_MGMT_1=0x%02X", s_cfg.who_am_i, s_cfg.pwr_mgmt_1);
    ESP_LOGI(TAG, "  GYRO_CONFIG =0x%02X  FS_SEL=%u  -> +/-%ddps  scale=%.1f LSB/dps",
              s_cfg.gyro_config, (unsigned)s_cfg.fs_sel,
              k_gyro_fs_dps[s_cfg.fs_sel], (double)s_cfg.gyro_lsb_per_dps);
    ESP_LOGI(TAG, "  ACCEL_CONFIG=0x%02X  AFS_SEL=%u -> +/-%dg    scale=%.0f LSB/g",
              s_cfg.accel_config, (unsigned)s_cfg.afs_sel,
              k_accel_fs_g[s_cfg.afs_sel], (double)s_cfg.accel_lsb_per_g);
    ESP_LOGI(TAG, "  CONFIG      =0x%02X  DLPF_CFG=%u", s_cfg.config, (unsigned)s_cfg.dlpf_cfg);
    ESP_LOGI(TAG, "  SMPLRT_DIV  =0x%02X  -> RATE=%dHz", s_cfg.smplrt_div, s_cfg.sample_rate_hz);

    // So với cấu hình DỰ ĐỊNH. Lệch = KHÔNG set valid -> flight_core chặn ARM.
    // Không "tự thích nghi" theo scale lạ: nếu chip không nhận đúng cấu hình ta
    // yêu cầu thì có gì đó sai ở tầng dưới, và bay với chip ở trạng thái không
    // hiểu được nguy hiểm hơn hẳn việc từ chối cất cánh.
    bool cfg_ok = true;
    if (s_cfg.gyro_config != MPU6050_GYRO_CONFIG_FS2000) {
        ESP_LOGE(TAG, "GYRO_CONFIG doc lai 0x%02X != du dinh 0x%02X", s_cfg.gyro_config, MPU6050_GYRO_CONFIG_FS2000);
        cfg_ok = false;
    }
    if (s_cfg.accel_config != MPU6050_ACCEL_CONFIG_FS16G) {
        ESP_LOGE(TAG, "ACCEL_CONFIG doc lai 0x%02X != du dinh 0x%02X", s_cfg.accel_config, MPU6050_ACCEL_CONFIG_FS16G);
        cfg_ok = false;
    }
    if (s_cfg.dlpf_cfg != MPU6050_CONFIG_DLPF_CFG_VALUE) {
        ESP_LOGE(TAG, "DLPF_CFG doc lai %u != du dinh %u", (unsigned)s_cfg.dlpf_cfg, (unsigned)MPU6050_CONFIG_DLPF_CFG_VALUE);
        cfg_ok = false;
    }
    if (s_cfg.smplrt_div != MPU6050_SMPLRT_DIV_VALUE) {
        ESP_LOGE(TAG, "SMPLRT_DIV doc lai %u != du dinh %u", (unsigned)s_cfg.smplrt_div, (unsigned)MPU6050_SMPLRT_DIV_VALUE);
        cfg_ok = false;
    }
    if (s_cfg.pwr_mgmt_1 != MPU6050_PWR1_WAKE_PLL_XGYRO) {
        ESP_LOGE(TAG, "PWR_MGMT_1 doc lai 0x%02X != du dinh 0x%02X (chip ngu lai? clock source doi?)",
                  s_cfg.pwr_mgmt_1, MPU6050_PWR1_WAKE_PLL_XGYRO);
        cfg_ok = false;
    }
    // Bảo hiểm cuối: scale suy ra phải khớp hằng số mà phần còn lại của
    // firmware giả định (tuning/telemetry). Lệch nghĩa là ai đó đã đổi
    // MPU6050_*_CONFIG_* mà quên đổi IMU_*_RAW_PER_* — bắt tại đây thay vì để
    // mọi số liệu sai lặng lẽ theo tỷ lệ.
    if (s_cfg.gyro_lsb_per_dps != IMU_GYRO_RAW_PER_DPS) {
        ESP_LOGE(TAG, "scale gyro suy ra %.1f != IMU_GYRO_RAW_PER_DPS %.1f (imu_driver.h) -> SUA HANG SO CHO KHOP",
                  (double)s_cfg.gyro_lsb_per_dps, (double)IMU_GYRO_RAW_PER_DPS);
        cfg_ok = false;
    }
    if (s_cfg.accel_lsb_per_g != IMU_ACCEL_RAW_PER_G) {
        ESP_LOGE(TAG, "scale accel suy ra %.0f != IMU_ACCEL_RAW_PER_G %.0f (imu_driver.h) -> SUA HANG SO CHO KHOP",
                  (double)s_cfg.accel_lsb_per_g, (double)IMU_ACCEL_RAW_PER_G);
        cfg_ok = false;
    }

    s_cfg.valid = cfg_ok;
    if (!cfg_ok) {
        ESP_LOGE(TAG, "MPU6050 CONFIG KHONG HOP LE -> imu_cfg valid=false, ARM se bi CHAN. "
                      "Khong tin scale nao ca cho toi khi sua xong.");
        return ESP_ERR_INVALID_RESPONSE;
    }

    s_ready = true;
    // TỪ ĐÂY TRỞ ĐI mọi transaction là runtime (imu_driver_read() trong
    // sensor_hub, 250Hz) -> siết timeout xuống, xem IMU_I2C_RUNTIME_TIMEOUT_MS.
    s_io_timeout_ms = IMU_I2C_RUNTIME_TIMEOUT_MS;
    ESP_LOGI(TAG, "MPU6050 init OK tai 0x%02X (toan bo config da read-back verify + scale suy ra "
                  "tu thanh ghi that, I2C timeout runtime=%dms)",
              i2c_addr, IMU_I2C_RUNTIME_TIMEOUT_MS);
    return ESP_OK;
}

void imu_driver_get_config(imu_cfg_readback_t *out) {
    if (out == NULL) return;
    *out = s_cfg;
}

// ================= data-ready interrupt =================

static TaskHandle_t s_notify_task = NULL;
static bool         s_int_configured = false;
static uint32_t     s_isr_count = 0;

// ISR: KHÔNG làm gì ngoài đánh thức task. Mọi thứ khác (đọc I2C, PID, log)
// PHẢI ở task — I2C block được, không bao giờ gọi trong ngắt.
static void imu_int_isr(void *arg) {
    (void)arg;
    s_isr_count++;
    BaseType_t hpw = pdFALSE;
    if (s_notify_task != NULL) {
        vTaskNotifyGiveFromISR(s_notify_task, &hpw);
    }
    if (hpw == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

esp_err_t imu_driver_enable_data_ready_int(int int_gpio, TaskHandle_t task_to_notify) {
    if (!s_ready) {
        ESP_LOGE(TAG, "enable_data_ready_int: IMU chua init OK -> bo qua");
        return ESP_ERR_INVALID_STATE;
    }
    if (task_to_notify == NULL || !GPIO_IS_VALID_GPIO(int_gpio)) {
        ESP_LOGE(TAG, "enable_data_ready_int: tham so sai (gpio=%d task=%p)", int_gpio, task_to_notify);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = write_verify_reg(MPU6050_REG_INT_PIN_CFG, MPU6050_INT_PIN_CFG_VALUE, "INT_PIN_CFG");
    if (err != ESP_OK) return err;
    err = write_verify_reg(MPU6050_REG_INT_ENABLE, MPU6050_INT_ENABLE_DATA_RDY, "INT_ENABLE");
    if (err != ESP_OK) return err;

    // pull-down BẬT: nếu dây INT đứt/chưa hàn thì chân đọc LOW ổn định thay vì
    // thả nổi. Chân thả nổi sinh cạnh ngẫu nhiên -> ngắt giả -> vòng điều khiển
    // chạy sai nhịp mà nhìn log vẫn thấy "có interrupt", cực khó lần ra.
    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << int_gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,   // khớp INT_LEVEL=0 (active high, xung 50us)
    };
    err = gpio_config(&io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config(INT=%d) that bai: %s", int_gpio, esp_err_to_name(err));
        return err;
    }

    // ESP_ERR_INVALID_STATE = service đã được cài bởi driver khác -> KHÔNG phải
    // lỗi, dùng chung được.
    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "gpio_install_isr_service that bai: %s", esp_err_to_name(err));
        return err;
    }

    s_notify_task = task_to_notify;   // gán TRƯỚC khi add handler (ISR đọc biến này)
    err = gpio_isr_handler_add(int_gpio, imu_int_isr, NULL);
    if (err != ESP_OK) {
        s_notify_task = NULL;
        ESP_LOGE(TAG, "gpio_isr_handler_add(INT=%d) that bai: %s", int_gpio, esp_err_to_name(err));
        return err;
    }

    s_int_configured = true;
    ESP_LOGI(TAG, "MPU6050 data-ready INT BAT tai GPIO%d (canh len, %dHz) -> stabilize_task chay "
                  "theo nhip cam bien thay vi dong ho FreeRTOS",
              int_gpio, IMU_SAMPLE_RATE_HZ);
    return ESP_OK;
}

bool imu_driver_int_configured(void) {
    return s_int_configured;
}

uint32_t imu_driver_int_isr_count(void) {
    return s_isr_count;
}

esp_err_t imu_driver_read(const imu_calib_t *calib, imu_sample_t *out) {
    if (calib == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_ready) {
        memset(out, 0, sizeof(*out));
        out->ok = false;
        return ESP_ERR_INVALID_STATE;
    }

    // Burst 14 byte từ ACCEL_XOUT_H: accel x/y/z, temp, gyro x/y/z (big-endian
    // per thanh ghi, MSB trước). KHÔNG log lỗi ở đây — gọi ở 250Hz, spam log
    // mỗi lần lỗi bus thoáng qua vô ích; caller (flight_core.c) đã tự đếm qua
    // s_imu_ok_driver, tầng trên chịu trách nhiệm rate-limit nếu cần thêm.
    uint8_t raw[14];
    esp_err_t err = read_regs(MPU6050_REG_ACCEL_XOUT_H, raw, sizeof(raw));
    if (err != ESP_OK) {
        memset(out, 0, sizeof(*out));
        out->ok = false;
        return err;
    }

    const int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
    const int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);
    const int16_t az = (int16_t)((raw[4] << 8) | raw[5]);
    const int16_t temp_raw = (int16_t)((raw[6] << 8) | raw[7]);   // TEMP_OUT
    const int16_t gx = (int16_t)((raw[8] << 8) | raw[9]);
    const int16_t gy = (int16_t)((raw[10] << 8) | raw[11]);
    const int16_t gz = (int16_t)((raw[12] << 8) | raw[13]);

    // ================= CHUỖI BIẾN ĐỔI, TỪNG TẦNG MỘT =================
    //   int16 LSB
    //     -> chia scale ĐỌC LẠI TỪ THANH GHI  -> *_raw_dps / *_raw_g
    //     -> trừ gyro_bias_dps (CÙNG frame)   -> gyro_dps (corrected)
    //     -> remap sensor->body               -> (hiện là IDENTITY, xem dưới)
    //
    // Scale lấy từ s_cfg (suy ra từ FS_SEL/AFS_SEL thật) chứ KHÔNG phải hằng số
    // biên dịch — init đã bảo đảm hai thứ khớp nhau, nhưng đường nóng đọc từ
    // nguồn sự thật để việc "đổi cấu hình mà quên đổi hằng số" không thể tạo ra
    // dữ liệu sai lặng lẽ.
    const float gyro_lsb_per_dps = s_cfg.gyro_lsb_per_dps;
    const float accel_lsb_per_g  = s_cfg.accel_lsb_per_g;

    // ---- tầng 1: RAW (chưa hiệu chỉnh gì) ----
    out->accel_raw_g.x = (float)ax / accel_lsb_per_g;
    out->accel_raw_g.y = (float)ay / accel_lsb_per_g;
    out->accel_raw_g.z = (float)az / accel_lsb_per_g;

    out->gyro_raw_dps.x = (float)gx / gyro_lsb_per_dps;
    out->gyro_raw_dps.y = (float)gy / gyro_lsb_per_dps;
    out->gyro_raw_dps.z = (float)gz / gyro_lsb_per_dps;

    // ---- tầng 2: CORRECTED ----
    // Gyro bias được ĐO và được TRỪ trong CÙNG MỘT FRAME (sensor frame), đúng
    // yêu cầu "không calib ở frame này rồi trừ ở frame khác". Remap sensor->body
    // hiện là IDENTITY (hướng lắp IMU chưa xác nhận — xem đầu file), nên
    // sensor frame == body frame. NẾU sau này thêm remap thật: đặt nó SAU phép
    // trừ bias này, và bias vẫn phải được đo ở sensor frame.
    out->gyro_corrected_sensor_dps.x = out->gyro_raw_dps.x - calib->gyro_bias_dps.x;
    out->gyro_corrected_sensor_dps.y = out->gyro_raw_dps.y - calib->gyro_bias_dps.y;
    out->gyro_corrected_sensor_dps.z = out->gyro_raw_dps.z - calib->gyro_bias_dps.z;
    out->gyro_dps = sensor_to_body(out->gyro_corrected_sensor_dps);

    // Accel: driver KHÔNG áp bias/scale. Việc đó ở tầng fusion (flight_core.c
    // bước 1c) để một chỗ duy nhất phục vụ mọi consumer. Giữ nguyên kiến trúc
    // cũ — chỉ đổi TÊN cho rõ đây là bản chưa hiệu chỉnh.
    out->accel_g = sensor_to_body(out->accel_raw_g);

    out->temp_c = (float)temp_raw / IMU_TEMP_LSB_PER_DEGC + IMU_TEMP_OFFSET_DEGC;

    out->ok = true;
    return ESP_OK;
}
