// Barometer driver (BMP280, I2C) — PORT THẬT theo datasheet Bosch BMP280
// (chip-ID probe, đọc bảng hiệu chỉnh NVM, formula compensation chuẩn
// double-precision từ tài liệu Bosch). Board KHÔNG gắn ToF — baro là NGUỒN
// DUY NHẤT cho alt_estimator (alt_estimator.h luôn chạy ở chế độ "PRIMARY"),
// KHÔNG chỉ còn vai trò "sửa trôi dài hạn" như thiết kế ban đầu khi có ToF.
// Độ phân giải thô hơn ToF nhiều (~0.3-1m, xem README) — chấp nhận cho
// alt_hold ở tầm trung/cao, KHÔNG đủ chính xác để bay sát đất/hạ mượt.
//
// alt_m trả về là ĐỘ CAO TƯƠNG ĐỐI so với áp suất nền lúc
// baro_driver_calibrate_ground() thành công (= 0m) — KHÔNG phải độ cao tuyệt
// đối so với mực nước biển (không biết QNH hiện tại). Đủ dùng cho việc sửa
// trôi vz/alt trong 1 chuyến bay, KHÔNG dùng để so sánh giữa các lần calibrate
// khác nhau.
//
// 2 BƯỚC TÁCH RIÊNG có chủ đích (KHÔNG gộp lại):
//   1) baro_driver_init()            — chip vận hành được (probe/config/verify),
//                                       CHƯA có mốc 0m, baro_driver_read() vẫn
//                                       trả ok=false cho tới bước 2.
//   2) baro_driver_calibrate_ground() — xác lập mốc 0m MỚI. Gọi lại được bất
//                                       cứ lúc nào drone đang đứng yên trên mặt
//                                       đất (vd ngay trước arm) để reset mốc,
//                                       KHÔNG chỉ gọi 1 lần lúc boot — lúc boot
//                                       sensor vừa reset/IIR chưa settle/drone
//                                       có thể còn đang bị cầm hay di chuyển.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "flight_core/fc_features.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float pressure_pa;     // áp suất tuyệt đối đo được (Pa)
    float temperature_c;   // nhiệt độ chip (°C) — dùng nội bộ cho compensation, không phải OAT chính xác
    float alt_m;            // độ cao TƯƠNG ĐỐI so với mốc calibrate_ground() (m), dương = lên cao
    bool  ok;                // false nếu chip chưa sẵn sàng HOẶC chưa calibrate_ground() thành công
} baro_sample_t;

// ================= CỜ BIÊN DỊCH =================
// Toàn bộ khai báo hàm dưới đây nằm sau `#if FC_FEATURE_BARO`. Cờ = 0 (đặt qua
// SENSOR_BARO_ENABLED trong main/app_config.h) thì driver KHÔNG được biên
// dịch vào firmware, và mọi lời gọi còn sót lại sẽ là LỖI BIÊN DỊCH — cố ý:
// lỗi lúc build tốt hơn nhiều so với một hàm rỗng trả về giá trị giả lúc bay.
//
// Các KIỂU dữ liệu (struct mẫu đo) vẫn được khai báo bình thường vì
// sensor_snapshot_t/telemetry vẫn có ô cho cảm biến này — chúng chỉ luôn ở
// trạng thái "chưa từng có mẫu".
#if FC_FEATURE_BARO

// baro_driver_init() — thêm device BMP280 vào bus dùng chung (đã tạo sẵn ở
// flight_core.c::init_i2c_bus(), KHÔNG tự tạo bus riêng). Sequence: probe
// CHIP_ID (0x58) -> soft-reset -> poll STATUS.im_update tới khi NVM copy xong
// -> đọc + VALIDATE 24 byte bảng hiệu chỉnh -> ghi CONFIG/CTRL_MEAS -> READ-BACK
// VERIFY. CHƯA lấy mốc 0m (xem baro_driver_calibrate_ground()) — baro_driver_read()
// sau bước này trả ok=false tới khi calibrate_ground() thành công.
// Trả lỗi THẬT của giao dịch I2C (KHÔNG ép thành ESP_ERR_NOT_FOUND) nếu
// transaction thất bại — CHỈ trả ESP_ERR_NOT_FOUND khi transaction THÀNH CÔNG
// nhưng CHIP_ID sai (board không hàn BMP280, hoặc SDO nối khác địa chỉ khai
// báo trong board_config.h) — caller (flight_core.c) coi như "không có baro".
// scl_speed_hz: xem imu_driver.h (tần số là thuộc tính CỦA DEVICE trong API I2C
// master mới, không phải của bus). Nguồn: BOARD_I2C_FREQ_HZ.
esp_err_t baro_driver_init(i2c_master_bus_handle_t bus, uint8_t i2c_addr,
                            uint32_t scl_speed_hz);

// baro_driver_calibrate_ground() — lấy trung bình BARO_GROUND_REF_SAMPLES mẫu
// áp suất (bỏ mẫu lỗi I2C/ngoài dải hợp lý) làm mốc 0m MỚI, ghi đè mốc cũ nếu
// có, ĐỒNG THỜI tính độ lệch chuẩn (std-dev) của các mẫu đó — xem
// baro_driver_ground_noise_std_pa()/baro_driver_ground_healthy(). Trả
// ESP_ERR_INVALID_STATE nếu gọi trước khi baro_driver_init() thành công;
// ESP_ERR_INVALID_RESPONSE nếu không đủ mẫu hợp lệ (bus lỗi liên tục).
// Drone PHẢI đứng yên trên mặt đất khi gọi — gió/rung khi đang cầm/di chuyển
// làm mốc 0m sai VÀ std-dev cao giả (baro bị coi nhầm là unhealthy).
//
// Gọi được NHIỀU LẦN (vd 1 lần lúc boot cho có mốc sẵn — xem flight_core.c —
// rồi gọi lại gần lúc ARM để mốc mới nhất, xem CMD_CALIB_BARO_GROUND trong
// command.h) — mỗi lần ghi đè hoàn toàn mốc + std-dev cũ, KHÔNG cộng dồn.
esp_err_t baro_driver_calibrate_ground(void);

// baro_driver_ground_ready() — true SAU khi baro_driver_calibrate_ground()
// thành công ít nhất 1 lần (bất kể healthy hay không) — dùng để phân biệt
// "chưa từng calib" (telemetry baro_calibrated=false) với "đã calib nhưng
// noisy" (baro_calibrated=true, baro_healthy=false).
bool baro_driver_ground_ready(void);

// baro_driver_ground_noise_std_pa() — độ lệch chuẩn (Pa) của
// BARO_GROUND_REF_SAMPLES mẫu áp suất nền lúc calibrate_ground() gần nhất.
// 0.0f nếu chưa từng calib thành công.
float baro_driver_ground_noise_std_pa(void);

// baro_driver_ground_healthy() — false nếu std-dev vượt
// BARO_GROUND_NOISE_STD_MAX_PA (baro noisy bất thường lúc calib — rung/gió/
// vị trí đặt không ổn định) HOẶC chưa từng calib. Caller (flight_core.c) nên
// dùng field này để CẢNH BÁO, KHÔNG BẮT BUỘC chặn ARM (attitude vẫn quyết
// định chính qua fsm_arm_guard_ok() — xem README mục Calibration) nhưng PHẢI
// chặn cho phép ALTITUDE HOLD/AUTO TAKEOFF dựa vào baro khi board không có
// ToF (xem "10. Pre-arm ground calibration").
bool baro_driver_ground_healthy(void);

// Ngưỡng std-dev áp suất nền (Pa) coi là "ồn bất thường" — điểm khởi đầu,
// CHƯA đo trên phần cứng thật. Gradient khí áp chuẩn gần mặt đất ~-12 Pa/m
// (dP/dz ≈ -ρg, ρ~1.225kg/m³) -> 3 Pa std-dev ứng ~0.25m noise — khớp cỡ độ
// phân giải BMP280 đã ghi ở baro_driver.h (~0.3-1m) — TODO xác nhận lại bằng
// Test A (README mục Test bắt buộc trước flight).
#define BARO_GROUND_NOISE_STD_MAX_PA   3.0f

// baro_driver_read() — đọc 1 mẫu áp suất+nhiệt độ mới (burst 6 byte), tính
// compensation + alt_m tương đối, VALIDATE (finite, trong dải 30000-110000 Pa)
// trước khi trả ok=true. Gọi được ở nhịp cao dù ODR vật lý BMP280 thấp hơn —
// đọc trùng giữa 2 lần convert vẫn trả dữ liệu hợp lệ (cùng giá trị), không
// sai, chỉ dư — NHƯNG caller (flight_core.c) nên throttle xuống ~25-50Hz,
// không cần gọi mỗi tick 250Hz của stabilize_task (baro không cần nhanh vậy).
esp_err_t baro_driver_read(baro_sample_t *out);

#endif  // FC_FEATURE_BARO
#ifdef __cplusplus
}
#endif
