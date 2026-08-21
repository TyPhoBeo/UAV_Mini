// QMC5883P register map thật (KHÁC HẲN QMC5883L cũ — chip ID, địa chỉ, layout
// thanh ghi đều khác, xem mag_driver.h). CHIP_ID @ 0x00 = 0x80, data burst 6
// byte @ 0x01 (little-endian), STATUS @ 0x09 (DRDY/OVFL), CTRL1 @ 0x0A,
// CTRL2 @ 0x0B.
//
// ---- MỘT NGƯỜI ĐỌC DUY NHẤT (quan trọng nhất trong file này) ----
// Đọc STATUS @ 0x09 CLEAR luôn bit DRDY. Vì vậy chỉ được có ĐÚNG MỘT nơi gọi
// mag_driver_read(): sensor task (stabilize_task, xem flight_core.c bước 1).
// Bất kỳ ai khác cần mẫu mag (Mahony, calibration, telemetry) PHẢI đọc mẫu đã
// PUBLISH qua mag_driver_get_latest() — KHÔNG được gọi mag_driver_read() lần
// hai, vì lần đọc thứ hai sẽ "ăn trộm" DRDY của lần đọc thứ nhất và cả hai
// bên đều thấy dữ liệu chập chờn/không có mẫu.
//
//   QMC5883P
//       -> sensor task gọi mag_driver_read()   (DUY NHẤT, clear DRDY)
//       -> publish s_latest {mag_body, seq, timestamp_us}
//       -> Mahony + calibration cùng đọc mag_driver_get_latest()
//          (calibration chỉ nhận mẫu khi seq ĐỔI -> không đếm trùng)
//
// ---- CTRL2 KHÔNG READBACK ĐƯỢC (đã đo trên phần cứng thật) ----
// Log `mag_test` thật: ghi CTRL2=0x08 xong đọc lại ra 0x00, NHƯNG chip đo
// hoàn hảo — DRDY lên đều, vòng đọc 20ms đạt ok=97%, XYZ ổn định
// (-2164, -97, 310). Kết luận: trên con chip này thanh ghi 0x0B không trả lại
// đúng thứ vừa ghi (write-only hoặc layout khác mô tả mình đang dựa vào).
//
// Vì vậy read-back CTRL1/CTRL2 CHỈ ĐỂ LOG, KHÔNG phải điều kiện init. Điều
// kiện DUY NHẤT để bật s_ready là đọc được một mẫu XYZ hợp lệ (bước 6) —
// verify bằng HÀNH VI, không bằng giả định về ngữ nghĩa thanh ghi. Trước đây
// readback là điều kiện cứng và nó đã TẮT một cảm biến đang chạy tốt suốt
// nhiều phiên debug.
//
// ---- REG 0x29 (SIGN) ----
// Datasheet QST chính thức yêu cầu ghi 0x29 = 0x06 sau soft-reset. Phần cứng
// này: chạy `mag_test` CÓ ghi 0x29 -> chip đo tốt (ok=97%). CHƯA có số liệu
// đối chứng cho trường hợp KHÔNG ghi 0x29 trên cùng build này. Firmware CHÍNH
// hiện KHÔNG ghi 0x29 — vì gate init giờ là "đọc được mẫu thật", nếu thiếu
// 0x29 mà chip không đo thì init sẽ FAIL to tiếng ở bước 6 chứ không âm thầm
// sai. Chốt bằng cách chạy `mag_test 0` rồi `mag_test 1` và so 2 log.
// Xem mag_driver_selftest() cuối file.
//
// CHƯA remap trục sensor->body — giống imu_driver.c, hướng lắp mag thật trên
// khung CHƯA xác nhận. Map THẲNG x/y/z tạm thời.
//
// Raw counts có thể đưa THẲNG vào Mahony vì Mahony tự normalize vector, nên
// CHƯA cần đổi sang Gauss. Nhưng raw counts CHƯA có hard-iron/soft-iron
// calibration (xem calibration.h) — offset (hard-iron) và méo trục (soft-iron)
// làm SAI HƯỚNG vector, mà normalization KHÔNG loại bỏ được sai hướng, chỉ
// chuẩn hóa độ dài. Trên UAV còn thêm nhiễu từ trường động (motor/ESC/dòng
// pin) thay đổi theo throttle. Vì vậy: mag hiện chỉ dùng cho bring-up/debug,
// TODO trước khi tin heading tuyệt đối: (1) sensor->body remap, (2) hard-iron
// calib, (3) soft-iron calib, (4) magnetic-field validity gating (xem
// mag_driver.h đầu file) trước khi tin MAG cho yaw.
#include "flight_core/drivers/mag_driver.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Toàn bộ thân driver nằm sau cờ biên dịch này. Cờ = 0 (từ
// SENSOR_MAG_ENABLED trong main/app_config.h, xem fc_features.h) thì file
// này biên dịch ra một object RỖNG — driver không tốn byte flash nào.
#if FC_FEATURE_MAG

static const char *TAG = "mag_driver";

#define QMC5883P_REG_CHIP_ID   0x00
#define QMC5883P_REG_X_LSB     0x01   // burst 6 byte: X,Y,Z LSB-first (little-endian)
#define QMC5883P_REG_STATUS    0x09
#define QMC5883P_REG_CTRL1     0x0A
#define QMC5883P_REG_CTRL2     0x0B

#define QMC5883P_CHIP_ID_VALUE 0x80

#define QMC5883P_STATUS_DRDY_BIT   0x01   // bit0: 1 = có mẫu mới
#define QMC5883P_STATUS_OVFL_BIT   0x02   // bit1: 1 = mẫu tràn, không hợp lệ

#define QMC5883P_CTRL2_SOFT_RST             0x80
// CTRL2: RNG[3:2] | SET/RESET mode[1:0]. RNG=10b (±8 Gauss), SET/RESET=00b (on).
#define QMC5883P_CTRL2_RNG_8G   (0x2 << 2)
#define QMC5883P_CTRL2_SETRESET_ON   0x00
#define QMC5883P_CTRL2_VALUE  (QMC5883P_CTRL2_RNG_8G | QMC5883P_CTRL2_SETRESET_ON)   // = 0x08
// CTRL2 có bit RFU/self-test ngoài RNG/SET-RESET (SOFT_RST tự clear sau reset,
// không nằm trong readback này) — read-back chỉ verify đúng phần cấu hình
// mình ghi, không đòi khớp NGUYÊN byte (xem mag_driver_init()).
#define QMC5883P_CTRL2_CONFIG_MASK   0x0F

// CTRL1: OSR2[7:6] | OSR1[5:4] | ODR[3:2] | MODE[1:0].
// MODE: 00=Suspend 01=Normal 10=Single 11=Continuous (datasheet QMC5883P).
// ĐANG DEBUG: dùng ĐÚNG MỘT cấu hình 0xC9 (Normal, ODR=100Hz) — KHÔNG có
// fallback tự động sang giá trị khác. Một cấu hình duy nhất thì log boot mới
// nói lên được điều gì; nhiều đường cấu hình chỉ làm mờ nguyên nhân thật.
#define QMC5883P_CTRL1_OSR2_FIELD   (0x3 << 6)   // theo datasheet application example
#define QMC5883P_CTRL1_OSR1_FIELD   (0x0 << 4)
#define QMC5883P_CTRL1_ODR_100HZ    (0x2 << 2)   // ODR=10b -> 100Hz
#define QMC5883P_CTRL1_MODE_NORMAL      0x01     // MODE=01b
#define QMC5883P_CTRL1_VALUE  (QMC5883P_CTRL1_OSR2_FIELD | QMC5883P_CTRL1_OSR1_FIELD | \
                                QMC5883P_CTRL1_ODR_100HZ | QMC5883P_CTRL1_MODE_NORMAL)   // = 0xC9
// MODE=00 — mọi lần ĐỔI MODE phải đi qua Suspend, KHÔNG ghi đè trực tiếp mode
// này sang mode khác (xem write_ctrl1_via_suspend()).
#define QMC5883P_CTRL1_SUSPEND      0x00

// Tốc độ SCL đến từ tham số scl_speed_hz của mag_driver_init() (nguồn:
// BOARD_I2C_FREQ_HZ) — KHÔNG #define ở đây nữa. GHI LẠI vào s_scl_speed_hz vì
// mag_driver_selftest() phải add lại device sau khi reset chip, và nó phải dùng
// ĐÚNG tốc độ đã dùng lúc init (lệch nhau = selftest chạy ở tốc độ khác với lúc
// bay, đúng kiểu bug làm "selftest pass nhưng bay lỗi").
static uint32_t s_scl_speed_hz = 0;

// SIGN register — xem đoạn "REG 0x29" đầu file. CHỈ mag_driver_selftest() ghi
// (khi được yêu cầu tường minh), init bình thường KHÔNG ghi.
#define QMC5883P_REG_SIGN     0x29
#define QMC5883P_SIGN_VALUE   0x06

#define QMC5883P_RESET_SETTLE_DELAY_MS   10
#define QMC5883P_MODE_SETTLE_MS          5    // sau khi đổi MODE (qua Suspend)

// Chờ DRDY: ODR=100Hz -> mẫu đầu phải có sau ~10ms. Cho 150ms (15 chu kỳ) để
// không fail oan vì I2C/scheduler chậm lúc boot.
#define MAG_DRDY_TIMEOUT_MS    150
#define MAG_DRDY_POLL_MS       10

// pdMS_TO_TICKS() làm TRÒN XUỐNG: với CONFIG_FREERTOS_HZ thấp (100Hz) thì
// pdMS_TO_TICKS(5) = 0 -> vTaskDelay(0) KHÔNG delay gì cả, biến vòng chờ
// thành busy-loop và làm timeout tính bằng "số vòng" sai hoàn toàn. Sàn 1
// tick + deadline đo bằng esp_timer_get_time() (đồng hồ THẬT, không phụ thuộc
// tick rate) là 2 lớp bảo vệ cho đúng lỗi này.
#define MAG_TICKS_AT_LEAST_1(ms)   ((pdMS_TO_TICKS(ms) > 0) ? pdMS_TO_TICKS(ms) : 1)

static i2c_master_dev_handle_t s_dev = NULL;
static bool                     s_ready = false;

// Mẫu ĐÃ PUBLISH — nguồn dữ liệu DUY NHẤT cho mọi consumer ngoài sensor task
// (xem đầu file). Chỉ mag_driver_read() ghi (từ 1 task duy nhất), consumer
// chỉ đọc -> không cần mutex, giống pattern telemetry snapshot.
static mag_published_t s_latest = { .mag_body = {0, 0, 0}, .seq = 0, .timestamp_us = 0 };

// Timeout I2C: INIT rộng (chạy 1 lần lúc boot), RUNTIME hẹp (chạy trong
// sensor_hub, mỗi ms chờ là một ms vòng bay không được cấp mẫu). Lý do đầy đủ:
// xem khối cùng tên ở đầu imu_driver.c. 8ms vs ~0.3ms transaction thật.
// LƯU Ý: mag_driver_selftest() cũng dùng timeout RUNTIME này (nó chạy sau
// init) — không sao, vì phần chờ LÂU của selftest nằm ở vòng wait_drdy()
// (deadline riêng MAG_DRDY_TIMEOUT_MS), không nằm trong một transaction đơn lẻ.
#define MAG_I2C_INIT_TIMEOUT_MS      50
#define MAG_I2C_RUNTIME_TIMEOUT_MS   8

static int s_io_timeout_ms = MAG_I2C_INIT_TIMEOUT_MS;

static esp_err_t write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_dev, buf, sizeof(buf), s_io_timeout_ms);
}

static esp_err_t read_regs(uint8_t reg, uint8_t *buf, size_t len) {
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, len, s_io_timeout_ms);
}

// cleanup_device() — dọn s_dev/s_ready cho MỌI đường lỗi sau khi
// i2c_master_bus_add_device() đã thành công. Thiếu bước này ở 1 nhánh lỗi thì
// gọi mag_driver_init() lại lần sau sẽ add_device() 2 lần cùng địa chỉ (leak
// handle cũ) — xem mag_driver_deinit().
static void cleanup_device(void) {
    s_ready = false;
    // Trả timeout về mức INIT: lần init KẾ TIẾP phải được chờ rộng rãi như
    // lần đầu (chip vừa bị reset/mất điện thì nó chậm, không phải "hỏng").
    s_io_timeout_ms = MAG_I2C_INIT_TIMEOUT_MS;
    if (s_dev != NULL) {
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
    }
}

esp_err_t mag_driver_deinit(void) {
    cleanup_device();
    return ESP_OK;
}

bool mag_driver_is_ready(void) {
    return s_ready;
}

// write_ctrl1_via_suspend() — ĐỔI MODE luôn đi đường Suspend: ghi CTRL1=0x00,
// đợi, rồi mới ghi mode đích. Ghi đè thẳng mode này sang mode khác (vd Normal
// -> Continuous) là chuyển trạng thái KHÔNG được datasheet bảo đảm; chip có
// thể "nhận" byte (readback đúng) mà state machine bên trong kẹt ở trạng thái
// không đo -> DRDY không bao giờ lên trong khi mọi readback đều đẹp. Đi qua
// Suspend là đường DUY NHẤT chắc chắn về được trạng thái xác định.
//
// Ngay sau soft-reset chip đã ở Suspend (MODE=00), nên bước ghi Suspend lúc
// init là thừa về mặt trạng thái — vẫn giữ để quy tắc này là CẤU TRÚC của
// code chứ không phải điều phải nhớ mỗi lần sửa.
static esp_err_t write_ctrl1_via_suspend(uint8_t ctrl1_value) {
    esp_err_t err = write_reg(QMC5883P_REG_CTRL1, QMC5883P_CTRL1_SUSPEND);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(MAG_TICKS_AT_LEAST_1(QMC5883P_MODE_SETTLE_MS));
    err = write_reg(QMC5883P_REG_CTRL1, ctrl1_value);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(MAG_TICKS_AT_LEAST_1(QMC5883P_MODE_SETTLE_MS));
    return ESP_OK;
}

// wait_drdy() — chờ tới khi STATUS.DRDY=1, deadline tính bằng
// esp_timer_get_time() (micro giây thật). KHÔNG đếm số vòng lặp nhân với
// "số ms mỗi vòng": vTaskDelay có thể ngắn hơn/dài hơn yêu cầu tuỳ tick rate
// và tải hệ thống, nên phép nhân đó không phải thời gian thật.
//
// Trả ESP_OK (DRDY đã lên), ESP_ERR_TIMEOUT (hết hạn, DRDY vẫn 0), hoặc lỗi
// I2C nguyên vẹn. status_out (có thể NULL) nhận STATUS lần đọc cuối để log.
static esp_err_t wait_drdy(int timeout_ms, uint8_t *status_out) {
    const int64_t deadline_us = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    for (;;) {
        uint8_t status = 0;
        const esp_err_t err = read_regs(QMC5883P_REG_STATUS, &status, 1);
        if (err != ESP_OK) {
            return err;
        }
        if (status_out != NULL) {
            *status_out = status;
        }
        if (status & QMC5883P_STATUS_DRDY_BIT) {
            return ESP_OK;
        }
        if (esp_timer_get_time() >= deadline_us) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(MAG_TICKS_AT_LEAST_1(MAG_DRDY_POLL_MS));
    }
}

// read_xyz_raw() — đọc burst 6 byte data và giải mã little-endian. KHÔNG đụng
// STATUS (caller tự quyết định đã có mẫu mới hay chưa).
static esp_err_t read_xyz_raw(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t raw[6];
    const esp_err_t err = read_regs(QMC5883P_REG_X_LSB, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }
    // Little-endian: LSB tại địa chỉ thấp hơn (KHÁC MPU6050 big-endian). Ép
    // qua uint16_t trước khi OR để tránh integer-promotion ambiguity.
    *x = (int16_t)((uint16_t)raw[0] | ((uint16_t)raw[1] << 8));
    *y = (int16_t)((uint16_t)raw[2] | ((uint16_t)raw[3] << 8));
    *z = (int16_t)((uint16_t)raw[4] | ((uint16_t)raw[5] << 8));
    return ESP_OK;
}

esp_err_t mag_driver_init(i2c_master_bus_handle_t bus, uint8_t i2c_addr,
                           uint32_t scl_speed_hz) {
    s_scl_speed_hz = scl_speed_hz;   // selftest add lại device -> phải dùng cùng tốc độ
    // KHÔNG gán s_dev = NULL trực tiếp: nếu lần init trước đã add_device thành
    // công (hoặc init này được gọi lại để restart sensor), gán NULL sẽ VỨT
    // handle cũ mà không remove khỏi bus -> leak handle + device trùng địa chỉ
    // trên bus. Phải cleanup_device() tử tế trước.
    if (s_dev != NULL) {
        ESP_LOGW(TAG, "init: da co device handle cu -> cleanup truoc khi add lai");
        cleanup_device();
    }
    s_ready = false;

    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_addr,
        .scl_speed_hz = scl_speed_hz,
    };
    ESP_LOGI(TAG, "QMC5883P @0x%02X, SCL %lu Hz", i2c_addr, (unsigned long)scl_speed_hz);
    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "them device QMC5883P tai 0x%02X vao bus that bai: %s",
                  i2c_addr, esp_err_to_name(err));
        s_dev = NULL;   // add_device thất bại -> không có handle nào để remove
        return err;
    }

    // ---- Bước 1: CHIP_ID ----
    uint8_t chip_id = 0;
    err = read_regs(QMC5883P_REG_CHIP_ID, &chip_id, 1);
    if (err != ESP_OK) {
        // Lỗi giao dịch I2C THẬT (timeout/nack/...) -> propagate NGUYÊN VẸN,
        // KHÔNG ép thành mã khác — quan trọng để debug đúng nguyên nhân khi 1
        // sensor làm hỏng cả bus, không nhầm với "chip sai địa chỉ/model".
        ESP_LOGE(TAG, "[1/6] doc CHIP_ID that bai tai 0x%02X: %s", i2c_addr, esp_err_to_name(err));
        cleanup_device();
        return err;
    }
    ESP_LOGI(TAG, "[1/6] CHIP_ID @0x00 = 0x%02X (ky vong 0x%02X)", chip_id, QMC5883P_CHIP_ID_VALUE);
    if (chip_id != QMC5883P_CHIP_ID_VALUE) {
        // Transaction THÀNH CÔNG (bus/slave đã ACK + trả byte) nhưng CHIP_ID
        // sai -> KHÔNG phải "không có sensor" theo nghĩa transport, mà là
        // "có thiết bị nhưng không đúng QMC5883P" (hàn nhầm QMC5883L cũ, chip
        // khác, hoặc lệch địa chỉ trùng 1 thiết bị khác trên bus). Trả
        // ESP_ERR_INVALID_RESPONSE để phân biệt rõ 2 tình huống này khi debug.
        ESP_LOGE(TAG, "device ACK tai 0x%02X nhung CHIP_ID khong phai QMC5883P "
                      "(dung QMC5883L cu? register map da doi sang QMC5883P, xem mag_driver.c)",
                  i2c_addr);
        cleanup_device();
        return ESP_ERR_INVALID_RESPONSE;
    }

    // ---- Bước 2: soft-reset ----
    err = write_reg(QMC5883P_REG_CTRL2, QMC5883P_CTRL2_SOFT_RST);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[2/6] soft-reset that bai: %s", esp_err_to_name(err));
        cleanup_device();
        return err;
    }
    vTaskDelay(MAG_TICKS_AT_LEAST_1(QMC5883P_RESET_SETTLE_DELAY_MS));
    ESP_LOGI(TAG, "[2/6] soft-reset OK (CTRL2=0x%02X), doi %dms", QMC5883P_CTRL2_SOFT_RST,
              QMC5883P_RESET_SETTLE_DELAY_MS);

    // REG 0x29 (SIGN): KHÔNG ghi ở đây — xem đoạn "REG 0x29" đầu file. Muốn
    // thử trên phần cứng thật thì dùng mag_driver_selftest(write_sign_reg=true).

    // ---- Bước 3: CTRL2 (range) ----
    err = write_reg(QMC5883P_REG_CTRL2, QMC5883P_CTRL2_VALUE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[3/6] ghi CTRL2 that bai: %s", esp_err_to_name(err));
        cleanup_device();
        return err;
    }

    // ---- Bước 4: CTRL1 (mode/ODR) qua Suspend ----
    err = write_ctrl1_via_suspend(QMC5883P_CTRL1_VALUE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[4/6] ghi CTRL1 (qua Suspend) that bai: %s", esp_err_to_name(err));
        cleanup_device();
        return err;
    }

    // ---- Bước 5: read-back CTRL1/CTRL2 — CHỈ ĐỂ LOG, KHÔNG phải điều kiện ----
    // ĐÃ TỪNG là điều kiện cứng và ĐÓ LÀ LỖI. Log 'mag_test' trên phần cứng
    // thật cho thấy: CTRL2 readback = 0x00 (không phải 0x08 vừa ghi) TRONG KHI
    // chip đo hoàn hảo — DRDY lên đều, vòng đọc 20ms đạt ok=97%, XYZ ổn định.
    // Tức là trên con chip này thanh ghi 0x0B không đọc lại được đúng thứ vừa
    // ghi (write-only, hoặc layout khác mô tả mình đang dựa vào). Bắt init
    // FAIL vì readback lệch = vứt bỏ một cảm biến ĐANG CHẠY TỐT chỉ vì nó
    // không khớp một giả định của mình.
    //
    // Nguyên tắc thay thế: verify bằng HÀNH VI, không bằng giả định. Điều kiện
    // DUY NHẤT để bật s_ready là bước 6 — chip phải đưa được một mẫu XYZ hợp lệ
    // ra tay. Đó là bằng chứng trực tiếp và không thể giả mạo cho đúng thứ
    // mình cần: chip đang đo. Readback vẫn đọc + log để không mất thông tin
    // (lệch thì cảnh báo, đọc lỗi I2C thì vẫn fail vì đó là lỗi bus thật).
    uint8_t ctrl1_rb = 0, ctrl2_rb = 0;
    err = read_regs(QMC5883P_REG_CTRL1, &ctrl1_rb, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[5/6] doc lai CTRL1 that bai: %s", esp_err_to_name(err));
        cleanup_device();
        return err;
    }
    err = read_regs(QMC5883P_REG_CTRL2, &ctrl2_rb, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[5/6] doc lai CTRL2 that bai: %s", esp_err_to_name(err));
        cleanup_device();
        return err;
    }
    ESP_LOGI(TAG, "[5/6] readback CTRL1=0x%02X (ky vong 0x%02X) CTRL2=0x%02X (ky vong 0x%02X, mask 0x%02X)",
              ctrl1_rb, QMC5883P_CTRL1_VALUE, ctrl2_rb, QMC5883P_CTRL2_VALUE, QMC5883P_CTRL2_CONFIG_MASK);

    const bool ctrl1_ok = (ctrl1_rb == QMC5883P_CTRL1_VALUE);
    const bool ctrl2_ok = ((ctrl2_rb & QMC5883P_CTRL2_CONFIG_MASK) ==
                           (QMC5883P_CTRL2_VALUE & QMC5883P_CTRL2_CONFIG_MASK));
    if (!ctrl1_ok || !ctrl2_ok) {
        // CẢNH BÁO, KHÔNG fail — xem lý do dài ở trên. Bước 6 mới là điều kiện.
        // CTRL2 lệch nghĩa là KHÔNG chắc range ±8G đã áp dụng: ảnh hưởng duy
        // nhất là HỆ SỐ TỶ LỆ của raw counts. Mahony tự normalize vector, còn
        // hard/soft-iron calib đo trực tiếp trên raw counts thực tế — nên tỷ lệ
        // sai KHÔNG làm hỏng heading. Chỉ đừng diễn giải raw counts thành Gauss
        // bằng hằng số cứng khi chưa xác nhận được range thật.
        ESP_LOGW(TAG, "[5/6] read-back CTRL lech (%s%s) -> VAN DI TIEP, de buoc 6 (doc mau that) "
                      "quyet dinh. %s",
                  ctrl1_ok ? "" : "CTRL1 ", ctrl2_ok ? "" : "CTRL2 ",
                  ctrl2_ok ? "" : "CTRL2 lech: chua chac range +-8G da ap dung, raw counts co the "
                                  "khac ty le mong doi (khong anh huong huong vector)");
    }

    // ---- Bước 6: chờ DRDY THẬT + đọc mẫu XYZ đầu tiên ----
    // Read-back CTRL chỉ chứng minh "ghi thanh ghi dính", KHÔNG chứng minh
    // chip ĐANG ĐO. Đã trả giá thật cho khác biệt này: 1 phiên calib_mag chạy
    // đủ 60s rồi báo "qua it mau (0 < 200) -> xoay lau hon", đổ lỗi cho người
    // dùng trong khi driver không hề nhận được mẫu nào. Chỉ bật s_ready khi
    // đã cầm trên tay MỘT mẫu XYZ hợp lệ.
    uint8_t status = 0;
    err = wait_drdy(MAG_DRDY_TIMEOUT_MS, &status);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[6/6] KHONG co DRDY sau %dms (STATUS=0x%02X, %s) -> chip KHONG do du da "
                      "nhan cau hinh. KHONG bat s_ready (mag TAT, yaw se troi). Chay 'mag_test' "
                      "de do tung buoc, hoac kiem tra nguon/day/dia chi 0x%02X",
                  MAG_DRDY_TIMEOUT_MS, status, esp_err_to_name(err), i2c_addr);
        cleanup_device();
        return err;
    }

    int16_t mx = 0, my = 0, mz = 0;
    err = read_xyz_raw(&mx, &my, &mz);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[6/6] DRDY da len (STATUS=0x%02X) nhung doc XYZ that bai: %s",
                  status, esp_err_to_name(err));
        cleanup_device();
        return err;
    }
    ESP_LOGI(TAG, "[6/6] STATUS=0x%02X, mau XYZ dau tien = (%d, %d, %d)", status, mx, my, mz);
    if (mx == 0 && my == 0 && mz == 0) {
        // Earth field thật không bao giờ cho vector 0 tuyệt đối trên cả 3 trục.
        ESP_LOGE(TAG, "[6/6] mau dau tien TOAN 0 tren ca 3 truc -> du lieu rac, KHONG bat s_ready");
        cleanup_device();
        return ESP_ERR_INVALID_RESPONSE;
    }

    s_latest.mag_body.x = (float)mx;
    s_latest.mag_body.y = (float)my;
    s_latest.mag_body.z = (float)mz;
    s_latest.timestamp_us = esp_timer_get_time();
    s_latest.seq = 1;

    s_ready = true;
    // Từ đây mọi transaction là runtime (mag_driver_read() trong sensor_hub).
    s_io_timeout_ms = MAG_I2C_RUNTIME_TIMEOUT_MS;
    ESP_LOGI(TAG, "QMC5883P INIT OK tai 0x%02X: Normal ODR=100Hz RNG=8G "
                  "(CTRL1=0x%02X CTRL2=0x%02X), da doc duoc mau that -> chip DANG DO, "
                  "I2C timeout runtime=%dms",
              i2c_addr, ctrl1_rb, ctrl2_rb, MAG_I2C_RUNTIME_TIMEOUT_MS);
    return ESP_OK;
}

static void invalidate_sample(mag_sample_t *out) {
    out->mag_body = vec3f_zero();
    out->ok = false;
}

esp_err_t mag_driver_read(mag_sample_t *out) {
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_ready) {
        invalidate_sample(out);
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t status = 0;
    esp_err_t err = read_regs(QMC5883P_REG_STATUS, &status, 1);
    if (err != ESP_OK) {
        invalidate_sample(out);
        return err;
    }

    // DRDY=0 -> chưa có mẫu mới (KHÔNG dùng data cũ như mẫu mới). OVFL=1 ->
    // mẫu tràn, không hợp lệ. Cả 2 trường hợp: ok=false nhưng KHÔNG phải lỗi
    // I2C (trả ESP_OK) — Mahony tự bỏ qua mag tick này, xem flight_core.c.
    // Đọc STATUS CLEAR DRDY -> đây PHẢI là nơi duy nhất đọc STATUS (xem đầu
    // file): consumer khác lấy mẫu qua mag_driver_get_latest().
    if (!(status & QMC5883P_STATUS_DRDY_BIT) || (status & QMC5883P_STATUS_OVFL_BIT)) {
        invalidate_sample(out);
        return ESP_OK;
    }

    int16_t mx = 0, my = 0, mz = 0;
    err = read_xyz_raw(&mx, &my, &mz);
    if (err != ESP_OK) {
        invalidate_sample(out);
        return err;
    }

    // Cả 3 trục = 0 đồng thời gần như chắc chắn là dữ liệu rác (chip lỗi/bus
    // glitch thoáng qua nhưng transaction vẫn "OK") — Earth field thật không
    // bao giờ cho vector 0 tuyệt đối trên cả 3 trục cùng lúc. OVFL đã xử lý
    // saturation lớn ở trên; đây là gate riêng cho trường hợp "toàn 0" bất
    // thường. KHÔNG cố gate theo norm Earth-field ở đây — đó là việc của tầng
    // estimator (xem TODO đầu mag_driver.h), driver chỉ lọc sample rác rõ ràng.
    if (mx == 0 && my == 0 && mz == 0) {
        invalidate_sample(out);
        return ESP_OK;
    }

    // TODO remap trục: map THẲNG x/y/z (chưa xác nhận hướng lắp mag thật, xem
    // đầu file). Raw counts THẲNG, chưa hard-iron/soft-iron calibration.
    out->mag_body.x = (float)mx;
    out->mag_body.y = (float)my;
    out->mag_body.z = (float)mz;
    out->ok = true;

    // PUBLISH — seq tăng ĐÚNG 1 lần cho mỗi mẫu MỚI. Consumer (calibration)
    // so seq với lần trước để biết "đây có phải mẫu chưa từng thấy không",
    // KHÔNG cần và KHÔNG được đọc phần cứng lần hai.
    s_latest.mag_body = out->mag_body;
    s_latest.timestamp_us = esp_timer_get_time();
    s_latest.seq++;
    return ESP_OK;
}

esp_err_t mag_driver_get_latest(mag_published_t *out) {
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = s_latest;
    // seq==0 nghĩa là chưa từng publish mẫu nào (init cũng chưa xong) — caller
    // phân biệt bằng seq, không cần cờ ok riêng.
    return (s_latest.seq == 0) ? ESP_ERR_NOT_FOUND : ESP_OK;
}

// ================= standalone selftest (bench/debug only) =================

// Log 1 bước của selftest theo cùng 1 định dạng để đọc log dễ so sánh.
#define MAG_TEST_LOG(step, fmt, ...)   ESP_LOGW(TAG, "[mag_test %s] " fmt, step, ##__VA_ARGS__)

// sweep_ctrl1_candidates() — CHỈ chạy trong selftest, khi cấu hình chính
// KHÔNG cho DRDY. Thử lần lượt vài giá trị CTRL1 và báo cái nào làm chip THẬT
// SỰ phát mẫu.
//
// Vì sao cần: MODE/OSR/ODR của QMC5883P được suy ra từ mô tả field, và mình
// đã có bằng chứng thực nghiệm rằng suy luận đó có thể sai (0 mẫu suốt 60s
// dù readback CTRL1 khớp hoàn hảo). Đoán tiếp một giá trị nữa rồi nạp vào
// firmware bay là lặp lại đúng sai lầm cũ. Thay vào đó: hỏi thẳng con chip,
// mỗi giá trị đều đi qua Suspend đúng quy tắc đổi mode, và CHỈ log — KHÔNG
// tự động đổi cấu hình của driver. Người đọc log quyết định.
static void sweep_ctrl1_candidates(void) {
    static const struct { uint8_t value; const char *desc; } CANDIDATES[] = {
        { 0xC9, "Normal ODR=100Hz (gia tri firmware dang dung)" },
        { 0xCB, "Continuous ODR=100Hz (cung field, chi doi MODE)" },
        { 0xC3, "Continuous, nguyen van application example QST" },
        { 0x03, "Continuous, OSR/ODR = 00 (toi gian)" },
        { 0x01, "Normal, OSR/ODR = 00 (toi gian)" },
        { 0x1D, "OSR2=00 OSR1=01 ODR=11 MODE=01 (bien the OSR thap)" },
    };
    MAG_TEST_LOG("11 quet CTRL1", "cau hinh chinh khong cho DRDY -> thu %d gia tri CTRL1 khac "
                                   "(CHI DO va LOG, KHONG tu doi cau hinh driver)",
                 (int)(sizeof(CANDIDATES) / sizeof(CANDIDATES[0])));
    int n_worked = 0;
    for (size_t i = 0; i < sizeof(CANDIDATES) / sizeof(CANDIDATES[0]); i++) {
        const uint8_t value = CANDIDATES[i].value;
        esp_err_t err = write_ctrl1_via_suspend(value);
        if (err != ESP_OK) {
            MAG_TEST_LOG("11 quet CTRL1", "  0x%02X: ghi that bai (%s) - %s",
                         value, esp_err_to_name(err), CANDIDATES[i].desc);
            continue;
        }
        uint8_t rb = 0;
        (void)read_regs(QMC5883P_REG_CTRL1, &rb, 1);
        uint8_t status = 0;
        err = wait_drdy(MAG_DRDY_TIMEOUT_MS, &status);
        if (err != ESP_OK) {
            MAG_TEST_LOG("11 quet CTRL1", "  0x%02X: readback=0x%02X KHONG co DRDY (STATUS=0x%02X) - %s",
                         value, rb, status, CANDIDATES[i].desc);
            continue;
        }
        int16_t x = 0, y = 0, z = 0;
        err = read_xyz_raw(&x, &y, &z);
        if (err != ESP_OK) {
            MAG_TEST_LOG("11 quet CTRL1", "  0x%02X: readback=0x%02X co DRDY nhung doc XYZ loi (%s) - %s",
                         value, rb, esp_err_to_name(err), CANDIDATES[i].desc);
            continue;
        }
        n_worked++;
        MAG_TEST_LOG("11 quet CTRL1", "  0x%02X: readback=0x%02X CO DRDY, XYZ=(%d, %d, %d) <== CHAY DUOC - %s",
                     value, rb, x, y, z, CANDIDATES[i].desc);
    }
    if (n_worked == 0) {
        MAG_TEST_LOG("11 quet CTRL1", "KHONG gia tri nao cho DRDY -> van de KHONG nam o CTRL1. "
                                       "Nghi nguon/day SDA-SCL/pull-up/dia chi, hoac chip loi. "
                                       "Thu 'mag_test 1' (co ghi REG 0x29) neu chua thu");
    } else {
        MAG_TEST_LOG("11 quet CTRL1", "%d gia tri chay duoc (xem cac dong '<== CHAY DUOC' o tren). "
                                       "Doi QMC5883P_CTRL1_VALUE trong mag_driver.c sang gia tri do "
                                       "roi build lai", n_worked);
    }
}

esp_err_t mag_driver_selftest(i2c_master_bus_handle_t bus, uint8_t i2c_addr,
                              bool write_sign_reg, int read_loop_ms) {
    // Chiếm trọn driver trong lúc test: hạ s_ready xuống để không có ai đọc
    // STATUS song song (nếu có consumer chạy task khác) — vẫn giữ nguyên
    // nguyên tắc "một người đọc duy nhất" ở đầu file.
    cleanup_device();

    MAG_TEST_LOG("BAT DAU", "addr=0x%02X write_sign_reg(0x29=0x06)=%s read_loop=%dms",
                 i2c_addr, write_sign_reg ? "CO" : "KHONG", read_loop_ms);

    // Dùng ĐÚNG tốc độ mag_driver_init() đã dùng (s_scl_speed_hz). Fallback
    // 400000 chỉ cho trường hợp selftest được gọi khi init CHƯA từng chạy —
    // đúng ca mà selftest tồn tại để chẩn đoán (init thất bại).
    const uint32_t speed = (s_scl_speed_hz > 0) ? s_scl_speed_hz : 400000;
    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_addr,
        .scl_speed_hz = speed,
    };
    MAG_TEST_LOG("add_device", "SCL %lu Hz", (unsigned long)speed);
    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) {
        MAG_TEST_LOG("add_device", "THAT BAI: %s", esp_err_to_name(err));
        s_dev = NULL;
        return err;
    }

    uint8_t chip_id = 0, ctrl1_rb = 0, ctrl2_rb = 0, status = 0;

    err = read_regs(QMC5883P_REG_CHIP_ID, &chip_id, 1);
    MAG_TEST_LOG("1 CHIP_ID", "err=%s value=0x%02X (ky vong 0x%02X)",
                 esp_err_to_name(err), chip_id, QMC5883P_CHIP_ID_VALUE);
    if (err != ESP_OK) { goto done; }

    err = write_reg(QMC5883P_REG_CTRL2, QMC5883P_CTRL2_SOFT_RST);
    MAG_TEST_LOG("2 soft-reset", "ghi CTRL2=0x%02X err=%s", QMC5883P_CTRL2_SOFT_RST, esp_err_to_name(err));
    if (err != ESP_OK) { goto done; }
    vTaskDelay(MAG_TICKS_AT_LEAST_1(QMC5883P_RESET_SETTLE_DELAY_MS));

    // Đây là CÂU HỎI cần trả lời bằng phần cứng thật: datasheet QST nói phải
    // ghi 0x29=0x06, nhưng log boot thật từng cho thấy ghi xong thì CTRL2
    // readback về 0x00. Chạy selftest 2 lần (write_sign_reg=false rồi =true)
    // và so 2 log là biết chip này thuộc phe nào.
    if (write_sign_reg) {
        err = write_reg(QMC5883P_REG_SIGN, QMC5883P_SIGN_VALUE);
        MAG_TEST_LOG("3 SIGN", "ghi 0x29=0x%02X err=%s", QMC5883P_SIGN_VALUE, esp_err_to_name(err));
        if (err != ESP_OK) { goto done; }
        vTaskDelay(MAG_TICKS_AT_LEAST_1(QMC5883P_MODE_SETTLE_MS));
    } else {
        MAG_TEST_LOG("3 SIGN", "BO QUA (khong ghi 0x29)");
    }

    err = write_reg(QMC5883P_REG_CTRL2, QMC5883P_CTRL2_VALUE);
    MAG_TEST_LOG("4 CTRL2 ghi", "0x%02X err=%s", QMC5883P_CTRL2_VALUE, esp_err_to_name(err));
    if (err != ESP_OK) { goto done; }

    err = read_regs(QMC5883P_REG_CTRL2, &ctrl2_rb, 1);
    MAG_TEST_LOG("5 CTRL2 doc", "err=%s value=0x%02X (ky vong 0x%02X) -> %s",
                 esp_err_to_name(err), ctrl2_rb, QMC5883P_CTRL2_VALUE,
                 ((ctrl2_rb & QMC5883P_CTRL2_CONFIG_MASK) ==
                  (QMC5883P_CTRL2_VALUE & QMC5883P_CTRL2_CONFIG_MASK)) ? "KHOP" : "LECH");
    if (err != ESP_OK) { goto done; }

    err = write_ctrl1_via_suspend(QMC5883P_CTRL1_VALUE);
    MAG_TEST_LOG("6 CTRL1 ghi", "0x00(suspend) -> 0x%02X err=%s", QMC5883P_CTRL1_VALUE, esp_err_to_name(err));
    if (err != ESP_OK) { goto done; }

    err = read_regs(QMC5883P_REG_CTRL1, &ctrl1_rb, 1);
    if (err == ESP_OK) { err = read_regs(QMC5883P_REG_CTRL2, &ctrl2_rb, 1); }
    MAG_TEST_LOG("7 CTRL doc lai", "err=%s CTRL1=0x%02X (ky vong 0x%02X) CTRL2=0x%02X",
                 esp_err_to_name(err), ctrl1_rb, QMC5883P_CTRL1_VALUE, ctrl2_rb);
    if (err != ESP_OK) { goto done; }

    err = wait_drdy(MAG_DRDY_TIMEOUT_MS, &status);
    MAG_TEST_LOG("8 DRDY", "err=%s STATUS=0x%02X (DRDY=%d OVFL=%d) sau toi da %dms",
                 esp_err_to_name(err), status,
                 (status & QMC5883P_STATUS_DRDY_BIT) ? 1 : 0,
                 (status & QMC5883P_STATUS_OVFL_BIT) ? 1 : 0, MAG_DRDY_TIMEOUT_MS);
    if (err != ESP_OK) {
        // Đây CHÍNH LÀ trường hợp đang gặp: thanh ghi ghi/đọc đẹp nhưng chip
        // không đo. Quét thử các giá trị CTRL1 khác để biết chip cần gì.
        sweep_ctrl1_candidates();
        goto done;
    }

    {
        int16_t mx = 0, my = 0, mz = 0;
        err = read_xyz_raw(&mx, &my, &mz);
        MAG_TEST_LOG("9 XYZ dau", "err=%s (%d, %d, %d)", esp_err_to_name(err), mx, my, mz);
        if (err != ESP_OK) { goto done; }
    }

    // ---- Vòng đọc 20ms: đây là bài kiểm tra THẬT SỰ quan trọng ----
    // Với ODR=100Hz, đọc mỗi 20ms (50Hz) thì gần như MỌI lần đọc phải có mẫu
    // mới. ok_pct thấp/bằng 0 = chip không phát mẫu ổn định, và khi đó
    // calibration có sửa kiểu gì cũng vô nghĩa.
    {
        int n_ok = 0, n_drdy0 = 0, n_ovfl = 0, n_zero = 0, n_err = 0;
        int16_t last_x = 0, last_y = 0, last_z = 0;
        const int64_t loop_deadline_us = esp_timer_get_time() + (int64_t)read_loop_ms * 1000;
        while (esp_timer_get_time() < loop_deadline_us) {
            uint8_t st = 0;
            if (read_regs(QMC5883P_REG_STATUS, &st, 1) != ESP_OK) {
                n_err++;
            } else if (st & QMC5883P_STATUS_OVFL_BIT) {
                n_ovfl++;
            } else if (!(st & QMC5883P_STATUS_DRDY_BIT)) {
                n_drdy0++;
            } else {
                int16_t x = 0, y = 0, z = 0;
                if (read_xyz_raw(&x, &y, &z) != ESP_OK) {
                    n_err++;
                } else if (x == 0 && y == 0 && z == 0) {
                    n_zero++;
                } else {
                    n_ok++;
                    last_x = x; last_y = y; last_z = z;
                }
            }
            vTaskDelay(MAG_TICKS_AT_LEAST_1(20));
        }
        const int total = n_ok + n_drdy0 + n_ovfl + n_zero + n_err;
        MAG_TEST_LOG("10 vong doc", "%dms @20ms: ok=%d drdy0=%d ovfl=%d all_zero=%d i2c_err=%d "
                                     "(tong=%d, ok=%d%%) mau cuoi=(%d, %d, %d)",
                     read_loop_ms, n_ok, n_drdy0, n_ovfl, n_zero, n_err, total,
                     (total > 0) ? (n_ok * 100 / total) : 0, last_x, last_y, last_z);
        err = (n_ok > 0) ? ESP_OK : ESP_ERR_TIMEOUT;
    }
    if (err != ESP_OK) {
        // Có DRDY lần đầu nhưng vòng đọc không thu được mẫu nào -> chip chỉ đo
        // 1 lần rồi dừng (nghi one-shot/Single mode thay vì đo liên tục).
        MAG_TEST_LOG("10 vong doc", "co DRDY luc init nhung vong doc 0 mau -> nghi chip do MOT LAN "
                                     "roi dung (Single mode?) thay vi do lien tuc");
        sweep_ctrl1_candidates();
    }

done:
    MAG_TEST_LOG("KET THUC", "ket qua = %s -> khoi phuc driver bang mag_driver_init() binh thuong",
                 esp_err_to_name(err));
    cleanup_device();
    // Khôi phục cấu hình bình thường để mag chạy lại ngay, không cần reboot.
    // KHÔNG ghi đè err của bài test bằng err của lần init khôi phục — cái
    // caller cần biết là bài test nói gì.
    // Khôi phục ở ĐÚNG tốc độ đã dùng trước test (s_scl_speed_hz vẫn còn giữ —
    // selftest không chạm nó). Truyền `speed` đã tính ở trên để nhánh "init
    // chưa từng chạy" cũng khôi phục được thay vì add device ở 0 Hz.
    const esp_err_t restore_err = mag_driver_init(bus, i2c_addr, speed);
    if (restore_err != ESP_OK) {
        MAG_TEST_LOG("KHOI PHUC", "mag_driver_init() sau test THAT BAI: %s (mag se TAT toi khi reboot)",
                     esp_err_to_name(restore_err));
    }
    return err;
}

#else   // !FC_FEATURE_MAG
// Cảm biến bị tắt lúc biên dịch. Một translation unit hoàn toàn rỗng là
// undefined behaviour theo ISO C, nên để lại đúng một khai báo vô hại.
typedef int mag_driver_disabled_at_compile_time_t;
#endif  // FC_FEATURE_MAG
