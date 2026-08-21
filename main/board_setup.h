// board_setup.h — DUY NHẤT một chỗ dựng flight_core_board_config_t từ
// board_config.h (pin/địa chỉ) + app_config.h (cờ bật/tắt).
//
// VÌ SAO tách ra: trước đây đoạn gán 20 dòng này được CHÉP NGUYÊN VĂN ở hai
// nơi — src/main.c (firmware console USB) và micropython_module/fc/fc_bridge.c
// (module Python). Đã kiểm: hai bản giống nhau từng ký tự. Nhưng
// micropython_module/ KHÔNG nằm trong build hiện tại, nên nếu sửa bản ở main.c
// mà quên bản kia thì trình biên dịch không hề báo gì — bản Python sẽ lặng lẽ
// giữ pin/mixer CŨ cho tới lúc ai đó build nó và bay. Riêng mấy dòng
// motor_gpio[] gán CHÉO ở dưới, lệch một dòng là drone lật ngay khi arm.
//
// Đây là ĐỊNH NGHĨA, không phải bản sao: sửa ở đây là cả hai nơi cùng đổi.
//
// static inline trong header (thay vì một file .c riêng) là có chủ đích: hàm
// chỉ được gọi đúng một lần lúc khởi động, và cách này KHÔNG cần nối dây thêm
// vào CMake — quan trọng vì micropython_module/ hiện không có build system nào
// trong repo này.
#pragma once

#include "flight_core/flight_core.h"

#include "app_config.h"
#include "board_config.h"

// board_config_fill() — điền TOÀN BỘ cấu hình phần cứng vào *cfg.
// Gọi ngay trước flight_core_start(). Không đụng phần cứng, không cấp phát.
static inline void board_config_fill(flight_core_board_config_t *cfg) {
    *cfg = (flight_core_board_config_t){0};

    cfg->i2c_sda_gpio = BOARD_I2C_SDA_GPIO;
    cfg->i2c_scl_gpio = BOARD_I2C_SCL_GPIO;
    cfg->i2c_freq_hz = BOARD_I2C_FREQ_HZ;
    cfg->imu_addr = BOARD_IMU_I2C_ADDR;
    cfg->imu_int_gpio = BOARD_MPU_INT_GPIO;   // data-ready INT = nhip stabilize_task

    cfg->mag_enabled = SENSOR_MAG_ENABLED;
    cfg->mag_addr = BOARD_MAG_I2C_ADDR;

    cfg->tof_enabled = SENSOR_TOF_ENABLED;
    cfg->tof_addr = BOARD_TOF_I2C_ADDR;
    cfg->tof_xshut_gpio = BOARD_TOF_XSHUT_GPIO;
    // Toc do SCL RIENG cho ToF (0 = dung chung i2c_freq_hz). Xem
    // BOARD_TOF_I2C_FREQ_HZ trong board_config.h: ToF thuong noi bang day roi
    // nen can bien thoi gian rong hon cac chip nam san tren PCB.
    cfg->tof_freq_hz = BOARD_TOF_I2C_FREQ_HZ;

    cfg->baro_enabled = SENSOR_BARO_ENABLED;
    cfg->baro_addr = BOARD_BARO_I2C_ADDR;

    // Vị trí vật lý ĐÃ XÁC NHẬN (sơ đồ người dùng đo trên khung thật, đầu drone
    // = cạnh M3-M2): CH3=front-left, CH2=front-right, CH1=back-right,
    // CH4=back-left. Mixer Quad-X (attitude_control.h) giả định thứ tự
    // motor_gpio[0..3] = M1=front-left/M2=front-right/M3=back-right/
    // M4=back-left — vì vậy gán CHÉO theo vị trí vật lý, KHÔNG theo số thứ tự
    // CH. Xem app_config.h mục "XÁC NHẬN VỊ TRÍ VẬT LÝ ĐỘNG CƠ" — chiều quay
    // CW/CCW mỗi góc VẪN CHƯA xác nhận, tự kiểm bằng test_motor trước khi bay.
    cfg->motor_gpio[0] = MOTOR_CH3_PIN;   // mixer M1 (front-left) <- CH3
    cfg->motor_gpio[1] = MOTOR_CH2_PIN;   // mixer M2 (front-right) <- CH2
    cfg->motor_gpio[2] = MOTOR_CH1_PIN;   // mixer M3 (back-right) <- CH1
    cfg->motor_gpio[3] = MOTOR_CH4_PIN;   // mixer M4 (back-left) <- CH4

    cfg->battery_enabled = SENSOR_BATTERY_ENABLED;
    cfg->battery_adc1_channel = BOARD_BATTERY_ADC1_CHANNEL;
}
