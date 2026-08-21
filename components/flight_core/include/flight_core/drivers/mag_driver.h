// Magnetometer driver (QMC5883P, new I2C master API — dùng chung
// i2c_master_bus_handle_t với imu_driver/baro_driver/tof_driver, xem
// flight_core.c::init_i2c_bus()).
//
// QUAN TRỌNG: nếu bật mag (mag_valid=true truyền vào mahony_update()), yaw có
// chuẩn tuyệt đối và bias gyro-Z trở nên "observable" (Mahony tự sửa qua
// integral feedback) — đây là hạn chế đã xác nhận trên UAV-Mini: KHÔNG có mag
// -> yaw luôn trôi chậm không cách nào sửa bằng heading-hold, vì heading-hold
// tự tham chiếu vào chính gyro đang bias. BẬT mag ở đây là cách dứt điểm nhất,
// không phải để "cho có".
//
// TODO (tầng estimator, KHÔNG phải driver này): magnetic validity gating —
// so |mag_body| (sau hard-iron/soft-iron calib, xem calibration.h) với norm
// tham chiếu đo lúc DISARMED, hoặc phát hiện thay đổi mạnh theo throttle
// (nhiễu dây nguồn/ESC/motor) — mag_driver.c chỉ lọc DRDY/OVFL/all-zero ở
// mức 1 mẫu (xem mag_driver.c), KHÔNG biết gì về Earth-field reference hay
// throttle. Khi mag invalid theo gate này, Mahony vẫn PHẢI chạy tiếp với
// accel+gyro (mag_ok=false CHỈ cho tick đó) — không được làm cả attitude
// estimator invalid chỉ vì mất 1 mẫu mag.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "flight_core/types.h"
#include "flight_core/fc_features.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    vec3f_t mag_body;   // raw counts THẲNG — CHƯA hard-iron/soft-iron calib, xem mag_driver.c
    bool    ok;
} mag_sample_t;

// Mẫu ĐÃ PUBLISH bởi sensor task — nguồn dữ liệu cho MỌI consumer khác
// (Mahony, calibration, telemetry). Xem mag_driver_get_latest().
typedef struct {
    vec3f_t  mag_body;      // mẫu hợp lệ GẦN NHẤT (raw counts, chưa calib)
    uint32_t seq;           // tăng ĐÚNG 1 mỗi mẫu MỚI; 0 = chưa từng có mẫu nào
    int64_t  timestamp_us;  // esp_timer_get_time() lúc publish
} mag_published_t;

// ================= CỜ BIÊN DỊCH =================
// Toàn bộ khai báo hàm dưới đây nằm sau `#if FC_FEATURE_MAG`. Cờ = 0 (đặt qua
// SENSOR_MAG_ENABLED trong main/app_config.h) thì driver KHÔNG được biên
// dịch vào firmware, và mọi lời gọi còn sót lại sẽ là LỖI BIÊN DỊCH — cố ý:
// lỗi lúc build tốt hơn nhiều so với một hàm rỗng trả về giá trị giả lúc bay.
//
// Các KIỂU dữ liệu (struct mẫu đo) vẫn được khai báo bình thường vì
// sensor_snapshot_t/telemetry vẫn có ô cho cảm biến này — chúng chỉ luôn ở
// trạng thái "chưa từng có mẫu".
#if FC_FEATURE_MAG

// mag_driver_init() — thêm device QMC5883P vào bus dùng chung (đã tạo sẵn ở
// flight_core.c::init_i2c_bus(), KHÔNG tự tạo bus riêng). Trình tự 6 bước, log
// từng bước: (1) CHIP_ID 0x00 kỳ vọng 0x80 — KHÁC HẲN QMC5883L cũ, (2)
// soft-reset, (3) CTRL2=0x08 (±8G), (4) CTRL1=0xC9 (Normal 100Hz) ghi QUA
// Suspend, (5) read-back CTRL1/CTRL2 — CHỈ LOG, (6) CHỜ DRDY THẬT + đọc 1 mẫu
// XYZ hợp lệ — ĐÂY là điều kiện DUY NHẤT để bật ready.
//
// Verify bằng HÀNH VI, không bằng giả định: phần cứng thật cho thấy CTRL2
// readback về 0x00 (không phải 0x08 vừa ghi) trong khi chip đo hoàn hảo
// (ok=97% ở vòng đọc 20ms). Bắt init fail vì readback lệch = vứt bỏ cảm biến
// đang chạy tốt. Một mẫu XYZ thật là bằng chứng trực tiếp và không thể giả
// mạo cho đúng thứ ta cần biết: chip đang đo.
//
// HỆ QUẢ của việc CTRL2 không readback được: chưa xác nhận range ±8G đã áp
// dụng. Chỉ ảnh hưởng TỶ LỆ raw counts — Mahony tự normalize, hard/soft-iron
// calib đo trực tiếp trên raw counts thật, nên heading KHÔNG bị ảnh hưởng.
// Đừng quy đổi raw counts sang Gauss bằng hằng số cứng khi chưa xác nhận range.
//
// KHÔNG ghi REG 0x29 (SIGN) dù datasheet QST yêu cầu — phần cứng này từng cho
// thấy ghi 0x29 làm CTRL2 readback sai. Muốn thử: mag_driver_selftest().
//
// Lỗi giao dịch I2C thật (bus/timeout/NACK) -> propagate nguyên vẹn.
// Transaction OK nhưng CHIP_ID sai / mẫu đầu toàn 0 -> ESP_ERR_INVALID_RESPONSE
// (KHÁC "không có sensor" — bus/slave đã trả lời, chỉ là không đúng chip).
// DRDY không lên trong MAG_DRDY_TIMEOUT_MS -> ESP_ERR_TIMEOUT (chạy `mag_test`
// để biết CTRL1 nào làm chip đo được).
// Mọi lỗi -> Mahony tự chạy chế độ "không mag" (yaw rate-only), khớp hành vi
// mặc định của UAV-Mini (UAV_MAHONY_USE_MAG=0).
// scl_speed_hz: xem imu_driver.h (tần số là thuộc tính CỦA DEVICE trong API I2C
// master mới, không phải của bus). Được GHI LẠI trong driver để
// mag_driver_selftest() add lại device ở ĐÚNG tốc độ đó sau khi reset chip.
esp_err_t mag_driver_init(i2c_master_bus_handle_t bus, uint8_t i2c_addr,
                           uint32_t scl_speed_hz);

// mag_driver_deinit() — remove device handle khỏi bus (KHÔNG đụng bus dùng
// chung), reset s_ready=false. Gọi trước khi mag_driver_init() lại nếu cần
// restart sensor — tránh add_device() 2 lần cùng địa chỉ (leak handle).
esp_err_t mag_driver_deinit(void);

// mag_driver_is_ready() — true khi init đã qua HẾT 6 bước (kể cả đọc được mẫu
// XYZ thật). Dùng để đồng bộ lại trạng thái sau mag_driver_selftest(), vì
// selftest tự init lại và lần init đó có thể thành công hoặc thất bại.
bool mag_driver_is_ready(void);

// mag_driver_read() — đọc STATUS trước, chỉ lấy mẫu khi DRDY=1 và OVFL=0
// (xem mag_driver.c) — KHÔNG bao giờ trả data cũ như mẫu mới. Trả
// ESP_ERR_INVALID_ARG nếu out là NULL (không dereference con trỏ NULL).
//
// CHỈ ĐƯỢC GỌI TỪ ĐÚNG MỘT NƠI (sensor task = stabilize_task, flight_core.c
// bước 1). Đọc STATUS CLEAR bit DRDY, nên nếu có nơi thứ hai cũng gọi hàm
// này, hai bên sẽ tranh nhau DRDY và cả hai đều thấy mẫu chập chờn/không có
// mẫu. Consumer khác (Mahony, calibration) dùng mag_driver_get_latest().
esp_err_t mag_driver_read(mag_sample_t *out);

// mag_driver_get_latest() — lấy mẫu ĐÃ PUBLISH gần nhất, KHÔNG đụng phần
// cứng (an toàn để gọi từ nhiều nơi, không cướp DRDY của sensor task).
// Trả ESP_ERR_NOT_FOUND khi chưa từng có mẫu nào (seq==0), ESP_OK khi có.
//
// Consumer muốn "chỉ xử lý mẫu MỚI" (calibration) so out->seq với seq lần
// trước mình đã xử lý — seq tăng đúng 1 cho mỗi mẫu, nên không bao giờ đếm
// trùng một mẫu hai lần dù gọi hàm này nhanh hơn ODR của chip.
esp_err_t mag_driver_get_latest(mag_published_t *out);

// mag_driver_selftest() — BENCH/DEBUG ONLY, chạy tuần tự toàn bộ trình tự
// khởi tạo và LOG TỪNG BƯỚC (CHIP_ID -> reset -> [SIGN] -> CTRL2 ghi/đọc ->
// CTRL1 ghi/đọc -> chờ DRDY -> XYZ đầu -> vòng đọc 20ms), để xác định bằng
// dữ liệu thật chỗ nào hỏng thay vì suy đoán.
//
// write_sign_reg: có ghi REG 0x29=0x06 (datasheet QST yêu cầu) hay không.
//   Chạy 2 lần, false rồi true, và SO 2 LOG — đó là cách dứt điểm câu hỏi
//   "chip này có cần 0x29 không", vì phần cứng đã từng mâu thuẫn với datasheet.
// read_loop_ms: thời lượng vòng đọc 20ms cuối bài test (0 = bỏ qua). Với
//   ODR=100Hz, đọc 50Hz thì ok% phải cao; ok=0 nghĩa là chip không phát mẫu
//   và mọi thứ ở tầng trên (calibration/Mahony) đều vô nghĩa.
//
// BLOCK caller trong suốt bài test (~read_loop_ms + ~0.3s). Chỉ gọi khi
// DISARMED. Tự khôi phục driver bằng mag_driver_init() khi xong, không cần
// reboot. Trả ESP_OK nếu vòng đọc thu được >=1 mẫu hợp lệ.
esp_err_t mag_driver_selftest(i2c_master_bus_handle_t bus, uint8_t i2c_addr,
                              bool write_sign_reg, int read_loop_ms);

#endif  // FC_FEATURE_MAG
#ifdef __cplusplus
}
#endif
