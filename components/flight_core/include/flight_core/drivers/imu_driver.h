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
    vec3f_t gyro_bias_dps;    // mean raw sensor-frame dps; fresh-calibrated moi boot
} imu_calib_t;

// imu_sample_t — MỘT mẫu IMU với CẢ HAI tầng dữ liệu, đơn vị ghi rõ trong tên.
//
// VÌ SAO giữ cả raw lẫn corrected: chẩn đoán bias cần nhìn ĐỒNG THỜI cả hai
// (xem telemetry GRAW*/GBIAS*/GCORR*). Nếu chỉ publish corrected thì khi drone
// vẫn trôi yaw ta không phân biệt được "bias sai" với "bias đúng nhưng chưa
// được áp" — đúng loại mơ hồ đã làm mất thời gian trước đây.
typedef struct {
    // ---- tầng RAW: sau scale vật lý, TRƯỚC mọi hiệu chỉnh calib ----
    vec3f_t gyro_raw_dps;    // = raw_lsb / gyro_lsb_per_dps (từ FS_SEL đọc lại)
    vec3f_t accel_raw_g;     // = raw_lsb / accel_lsb_per_g  (từ AFS_SEL đọc lại)

    // ---- corrected SENSOR frame, trước remap ----
    vec3f_t gyro_corrected_sensor_dps; // = gyro_raw_dps - gyro_bias_dps

    // ---- BODY frame authoritative cho estimator/controller ----
    vec3f_t gyro_dps;   // sensor_to_body(gyro_corrected_sensor_dps); Mahony + rate PID dùng
    vec3f_t accel_g;    // sensor_to_body(accel_raw_g); accel calib áp ở flight_core

    float   temp_c;      // nhiệt độ die MPU6050 (°C, công thức datasheet) — dùng cho calib
                          // quality/thermal logging (xem calibration.c), KHÔNG phải OAT chính xác
    bool    ok;
} imu_sample_t;

// ---- Scale MẶC ĐỊNH theo cấu hình DỰ ĐỊNH ghi vào chip ----
// KHÔNG dùng thẳng ở đường nóng: imu_driver_read() dùng scale SUY RA TỪ
// READ-BACK thanh ghi thật (xem imu_driver_get_config()). Hai hằng số này chỉ
// là giá trị kỳ vọng để so khớp lúc init — lệch = init fail.
#define IMU_GYRO_RAW_PER_DPS   16.4f    // FS_SEL=3 (±2000dps) — xem imu_driver.c
#define IMU_ACCEL_RAW_PER_G    2048.0f  // AFS_SEL=3 (±16g) — xem imu_driver.c
// Công thức nhiệt độ MPU6050/MPU6000 THEO DATASHEET (Register Map, mục
// TEMP_OUT): Temp_degC = TEMP_OUT/340 + 36.53 — CỐ ĐỊNH, không phụ thuộc
// FS_SEL/AFS_SEL, không cần calib riêng.
#define IMU_TEMP_LSB_PER_DEGC  340.0f
#define IMU_TEMP_OFFSET_DEGC   36.53f

// MPU6050 chạy 1kHz khi DLPF được bật. sensor_hub gom đúng 4 mẫu liên tiếp,
// lấy trung bình rồi mới publish một mẫu 250Hz cho stabilize_task. Vì vậy PID
// và estimator vẫn giữ dt=4ms, trong khi nhiễu IMU được giảm bằng oversampling.
// 1000/(1+SMPLRT_DIV) phải chia hết -> chỉ dùng ước của 1000.
//
// ⚠ BA HẰNG SỐ NÀY MẮC NỐI TIẾP, KHÔNG ĐỔI RIÊNG ĐƯỢC MỘT CÁI:
//     IMU_SAMPLE_RATE_HZ == CONTROL_TASK_HZ * IMU_SAMPLES_PER_CONTROL
//   (_Static_assert ở flight_core.c + sensor_hub.c bắt lúc BIÊN DỊCH — nếu
//   không có chúng thì dt dùng trong mọi tích phân sẽ LỆCH so với nhịp thật, và
//   triệu chứng là PID hành xử khác hẳn giá trị đã tune mà KHÔNG có gì báo.)
//
//   IMU_SAMPLE_RATE_HZ còn phải là ƯỚC CỦA 1000 (ràng buộc phần cứng:
//   SMPLRT_DIV nguyên). Mà 1000 = 2^3 * 5^3 — KHÔNG có thừa số 3 — nên
//   IMU_SAMPLES_PER_CONTROL = 3 là BẤT KHẢ THI về mặt số học, không phải
//   chuyện chỉnh ngưỡng. Các tổ hợp hợp lệ:
//       batch 1 -> CONTROL 1000Hz    batch 2 -> CONTROL 500Hz
//       batch 4 -> CONTROL  250Hz    batch 5 -> CONTROL 200Hz
//       batch 8 -> CONTROL  125Hz
//   Đổi batch thì PHẢI đổi CONTROL_TASK_HZ (flight_core.c) cho khớp.
//
// ⚠ ĐỪNG NHẦM với IMU_ACCEL_DLPF_CFG (tuning.h mục 11) — đó là bộ lọc PHẦN
//   CỨNG trong chip, cũng hay nhận giá trị 3-4 nhưng KHÔNG liên quan gì tới
//   nhịp gộp mẫu ở đây.
#define IMU_SAMPLE_RATE_HZ          1000
#define IMU_SAMPLES_PER_CONTROL     4

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

// imu_cfg_readback_t — cấu hình MPU6050 ĐỌC LẠI TỪ CHIP sau init, cùng scale
// SUY RA từ chính các bit đó (KHÔNG phải hằng số biên dịch). Đây là nguồn sự
// thật cho mọi phép convert LSB->đơn vị vật lý ở runtime.
//
// VÌ SAO: một hằng số scale cố định độc lập với thanh ghi là bẫy im lặng —
// chip nhận sai FS_SEL (bus glitch, chip khác đời, ai đó sửa #define mà quên
// sửa hằng số kia) thì mọi số dps/g đều sai theo TỶ LỆ, gyro bias đo được cũng
// sai đúng tỷ lệ đó, và không có triệu chứng nào ngoài "drone bay lạ".
typedef struct {
    uint8_t who_am_i;
    uint8_t config;          // 0x1A
    uint8_t gyro_config;     // 0x1B
    uint8_t accel_config;    // 0x1C
    uint8_t smplrt_div;      // 0x19
    uint8_t pwr_mgmt_1;      // 0x6B

    uint8_t fs_sel;          // GYRO_CONFIG bit[4:3]
    uint8_t afs_sel;         // ACCEL_CONFIG bit[4:3]
    uint8_t dlpf_cfg;        // CONFIG bit[2:0]

    float   gyro_lsb_per_dps;   // suy ra từ fs_sel  (131/65.5/32.8/16.4)
    float   accel_lsb_per_g;    // suy ra từ afs_sel (16384/8192/4096/2048)
    int     sample_rate_hz;     // 1000/(1+smplrt_div) khi DLPF bật

    bool    valid;           // true CHỈ khi mọi thanh ghi đọc lại khớp giá trị dự định
} imu_cfg_readback_t;

// imu_driver_get_config() — cấu hình đã read-back lúc init. valid=false nghĩa
// là KHÔNG được tin scale nào cả (và flight_core chặn ARM). An toàn gọi bất kỳ
// lúc nào sau init; trước init trả struct rỗng valid=false.
void imu_driver_get_config(imu_cfg_readback_t *out);

// true khi INT đã cấu hình xong (cả thanh ghi MPU lẫn GPIO/ISR).
bool imu_driver_int_configured(void);

// Số lần ISR đã chạy từ lúc boot — so với số vòng lặp thực tế để biết ngắt có
// đến đủ nhịp không (xem telemetry imu_int_*). CHỈ để chẩn đoán.
uint32_t imu_driver_int_isr_count(void);

#ifdef __cplusplus
}
#endif
