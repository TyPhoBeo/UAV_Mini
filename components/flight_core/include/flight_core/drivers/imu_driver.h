// IMU driver (MPU6050, new I2C master API — dùng chung
// i2c_master_bus_handle_t với mag_driver/baro_driver/tof_driver, xem
// flight_core.c::init_i2c_bus()).
//
// Công thức hiệu chỉnh (scale + trừ bias) GIỮ NGUYÊN từ MPUDriver::convert()
// (UAV-Mini, đã đúng).
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"       // TaskHandle_t — xem imu_driver_enable_data_ready_int()
#include "flight_core/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    vec3f_t gyro_bias_dps;    // từ calib tĩnh (đứng yên), nạp từ NVS lúc boot — xem calibration.h
} imu_calib_t;

typedef struct {
    vec3f_t gyro_dps;   // đã trừ bias, đã remap trục sensor->body (TODO ở driver)
    vec3f_t accel_g;    // đơn vị g, đã remap trục
    float   temp_c;      // nhiệt độ die MPU6050 (°C, công thức datasheet) — dùng cho calib
                          // quality/thermal logging (xem calibration.c), KHÔNG phải OAT chính xác
    bool    ok;
} imu_sample_t;

#define IMU_GYRO_RAW_PER_DPS   16.4f    // FS_SEL=3 (±2000dps) — xem imu_driver.c
#define IMU_ACCEL_RAW_PER_G    2048.0f  // AFS_SEL=3 (±16g) — xem imu_driver.c
// Công thức nhiệt độ MPU6050/MPU6000 THEO DATASHEET (Register Map, mục
// TEMP_OUT): Temp_degC = TEMP_OUT/340 + 36.53 — CỐ ĐỊNH, không phụ thuộc
// FS_SEL/AFS_SEL, không cần calib riêng.
#define IMU_TEMP_LSB_PER_DEGC  340.0f
#define IMU_TEMP_OFFSET_DEGC   36.53f

// Sample rate cấu hình vào SMPLRT_DIV. PHẢI BẰNG ĐÚNG CONTROL_TASK_HZ vì chân
// INT của MPU6050 chính là đồng hồ nhịp của stabilize_task khi chạy chế độ
// interrupt-driven — lệch 2 số này là vòng điều khiển chạy sai tần số trong
// khi hằng số dt của PID vẫn giữ nguyên (PID phản ứng sai hệ số).
// flight_core.c có _Static_assert canh đúng điều kiện này lúc biên dịch.
// 1000/(1+SMPLRT_DIV) phải chia hết -> chỉ dùng ước của 1000 (125/200/250/500/1000).
#define IMU_SAMPLE_RATE_HZ     250

// imu_driver_init() — thêm device MPU6050 vào bus dùng chung (đã tạo sẵn ở
// flight_core.c::init_i2c_bus(), KHÔNG tự tạo bus riêng). Sequence: probe
// WHO_AM_I (0x75=0x68) -> DEVICE_RESET -> wake+PLL X-gyro clock -> ghi
// DLPF/FS_SEL/AFS_SEL/SMPLRT_DIV -> READ-BACK VERIFY từng thanh ghi vừa ghi.
// s_ready CHỈ true khi TẤT CẢ bước verify khớp — 1 bước sai là driver coi như
// KHÔNG init được (an toàn mặc định: attitude không bao giờ valid -> không
// arm được), xem chi tiết trong imu_driver.c.
// scl_speed_hz: tần số SCL cho ĐÚNG device này. Trong API I2C master mới của
// ESP-IDF, tần số là thuộc tính CỦA DEVICE (i2c_device_config_t.scl_speed_hz),
// KHÔNG phải của bus — i2c_master_bus_config_t không có field tần số nào cả.
// Truyền từ board_cfg->i2c_freq_hz (= BOARD_I2C_FREQ_HZ) để CHỈ CÓ MỘT nơi
// định nghĩa tốc độ bus. Trước đây mỗi driver tự #define riêng một hằng số
// 400000 -> 4 bản sao + BOARD_I2C_FREQ_HZ là bản thứ 5 và KHÔNG ĐƯỢC DÙNG,
// nên sửa nó không có tác dụng gì (đúng loại bẫy im lặng).
esp_err_t imu_driver_init(i2c_master_bus_handle_t bus, uint8_t i2c_addr,
                           uint32_t scl_speed_hz);

// imu_driver_read() — đọc burst 14 byte raw 6 trục, convert bằng scale ở
// trên + trừ calib->gyro_bias_dps, remap trục theo hướng lắp board thật (xem
// board_config.h phần "TODO: remap trục"). Trả ESP_ERR_INVALID_ARG nếu calib
// hoặc out là NULL (không dereference con trỏ NULL).
esp_err_t imu_driver_read(const imu_calib_t *calib, imu_sample_t *out);

// imu_driver_enable_data_ready_int() — bật DATA_RDY interrupt của MPU6050 ra
// chân INT (board_config.h BOARD_MPU_INT_GPIO) và nối vào ISR đánh thức
// task_to_notify bằng vTaskNotifyGiveFromISR(). Gọi SAU imu_driver_init() và
// SAU khi task đã được tạo (cần TaskHandle_t thật).
//
// Đổi gì so với polling: stabilize_task không còn tự hẹn giờ bằng
// vTaskDelayUntil() mà ngủ chờ chính con cảm biến báo "có mẫu mới". Mẫu được
// đọc ngay sau khi sinh ra -> độ trễ nhỏ và ỔN ĐỊNH, không còn jitter 0..4ms
// do lệch pha giữa đồng hồ FreeRTOS và đồng hồ nội của MPU6050. Với vòng
// điều khiển thì jitter độ trễ đúng nghĩa là nhiễu bơm thẳng vào D-term.
//
// AN TOÀN: caller PHẢI có đường dự phòng khi ngắt ngừng đến (dây đứt, chip
// treo). stabilize_task chờ notify KÈM TIMEOUT rồi tự rơi về đồng hồ
// FreeRTOS — vòng điều khiển KHÔNG BAO GIỜ được phép đứng chờ ngắt vô hạn,
// vì lúc đó motor vẫn giữ nguyên duty lệnh cuối cùng.
//
// Lỗi ghi/verify thanh ghi hoặc cấu hình GPIO -> trả lỗi, KHÔNG bật cờ; caller
// cứ chạy tiếp bằng polling như cũ (giảm chất lượng, không mất an toàn).
esp_err_t imu_driver_enable_data_ready_int(int int_gpio, TaskHandle_t task_to_notify);

// true khi INT đã cấu hình xong (cả thanh ghi MPU lẫn GPIO/ISR).
bool imu_driver_int_configured(void);

// Số lần ISR đã chạy từ lúc boot — so với số vòng lặp thực tế để biết ngắt có
// đến đủ nhịp không (xem telemetry imu_int_*). CHỈ để chẩn đoán.
uint32_t imu_driver_int_isr_count(void);

#ifdef __cplusplus
}
#endif
