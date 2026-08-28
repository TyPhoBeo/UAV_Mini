#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "flight_core/fc_features.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================
// ToF driver — MỘT sensor, hướng xuống. HỖ TRỢ HAI DÒNG CHIP.
// ============================================================
//
// Vai trò trong hệ: nguồn CORRECTION có điều kiện cho alt_estimator.
// KHÔNG phải nguồn chính — z/vz đến từ tích phân accel (accel-primary), ToF chỉ
// kéo z về đúng khi nó thật sự đang nhìn MẶT SÀN đã khoá (xem
// alt_estimator.h mục "ToF surface-gated correction"). Bay qua bàn/ghế thì ToF
// bị GATE, KHÔNG được sửa z — đó là thiết kế, không phải lỗi.
//
// ============================================================
// CHỌN CHIP — BOARD_TOF_CHIP trong main/board_config.h
// ============================================================
//   TOF_CHIP_VL53L0X : thanh ghi 8-bit,  tầm tin cậy ~1.2m
//   TOF_CHIP_VL53L1X : thanh ghi 16-bit, tầm tin cậy ~1.3m (SHORT) / ~2.6m (LONG)
//
// API DƯỚI ĐÂY KHÔNG ĐỔI theo chip. Người dùng driver (sensor_hub, flight_core,
// main.c) không cần biết chip nào đang chạy — dùng tof_driver_chip_name() nếu
// cần in ra cho người đọc log.
//
// Hiện thực chia làm ba file trong src/drivers/:
//   tof_driver.c    — facade: XSHUT, dò địa chỉ, watchdog, staleness (file này)
//   vl53l0x_driver.c — backend dòng L0X
//   vl53l1x_driver.c — backend dòng L1X
// Giao diện giữa chúng: src/drivers/tof_backend.h (NỘI BỘ, không dùng từ ngoài).
//
// VÌ SAO tách file thay vì if/else: địa chỉ thanh ghi khác ĐỘ RỘNG (8 vs 16
// bit), nên MỌI hàm read/write đều khác chữ ký. Không có một dòng nào ở tầng
// thấp dùng chung được. Chi tiết đầy đủ: tof_backend.h.
//
// ⚠ CẮM NHẦM CHIP KHÔNG BAO GIỜ IM LẶNG: chip lạ vẫn ACK ở 0x29 nhưng ID đọc
// ra sai, và driver in ra ĐÚNG dòng cấu hình cần sửa (xem
// identify_foreign_device() trong tof_driver.c).
//
// ============================================================
// ĐÃ ĐƠN GIẢN HOÁ TỪ 2 SENSOR VỀ 1
// ============================================================
// Bản trước có tof_driver_init_dual() + tof_driver_read_aux() + s_sensors[2] để
// hỗ trợ một con ToF thứ hai (forward/dự phòng). Con đó CHƯA BAO GIỜ được fuse
// vào control loop (SENSOR_TOF2_ENABLED luôn = 0), nhưng nó bắt cả file phải
// mang những ràng buộc CHỈ có nghĩa khi có 2 chip:
//   - addr1 KHÔNG được giữ 0x29 (phải nhường cho con 2 boot)
//   - CẢ HAI dây XSHUT phải điều khiển được
//   - thứ tự bring-up bắt buộc (con 1 phải rời 0x29 trước khi nhả con 2)
// Cả ba đều đã gây lỗi thật cho cấu hình 1 chip. Toàn bộ đường đó giờ đã xoá.
//
// Với MỘT sensor:
//   - Địa chỉ mặc định 0x29 dùng LUÔN được (không ai tranh) -> KHÔNG cần đổi.
//   - Dây XSHUT là TÙY CHỌN (module có pull-up 10k, chip tự chạy).
//   - Không có thứ tự bring-up nào cần tuân theo.
//
// ============================================================
// XSHUT — nếu có nối
// ============================================================
// Là chân 2.8V, KHÔNG level-shift, KHÔNG chịu 3.3V đẩy vào (datasheet ST +
// trang sản phẩm Pololu). Driver lái OPEN-DRAIN: LOW = kéo xuống thật,
// "HIGH" = THẢ VỀ HIGH-Z cho pull-up 10k trên module kéo lên 2.8V.
// ĐỪNG đổi sang GPIO_MODE_OUTPUT — xem configure_xshut_gpio() trong .c.
//
// Lý do DUY NHẤT còn lại để cần XSHUT khi chỉ có 1 sensor: reset cứng để xoá
// state trong RAM chip (kể cả địa chỉ I2C đã đổi ở lần boot trước). Không nối
// cũng chạy bình thường.
// ============================================================

// ============================================================
// Stale timeout
// ============================================================
//
// Không nhận được measurement hợp lệ mới quá thời gian này -> mẫu cũ không còn
// được coi là valid (tof_driver.c dùng macro này).
#ifndef TOF_STALE_TIMEOUT_MS
#define TOF_STALE_TIMEOUT_MS 200
#endif


// ============================================================
// Reading
// ============================================================

typedef struct {
    // Khoảng cách tính bằng mét (0.350f = 350mm).
    float distance_m;

    // Range status nội bộ của driver:
    //   0   = valid
    //   !=0 = measurement không hợp lệ / lỗi ranging
    //   255 = unknown / chưa có mẫu mới
    uint8_t range_status;

    // true khi distance_m dùng được.
    bool valid;
} tof_reading_t;


// ================= CỜ BIÊN DỊCH =================
// Toàn bộ khai báo hàm dưới đây nằm sau `#if FC_FEATURE_TOF`. Cờ = 0 (đặt qua
// SENSOR_TOF_ENABLED trong main/app_config.h) thì driver KHÔNG được biên
// dịch vào firmware, và mọi lời gọi còn sót lại sẽ là LỖI BIÊN DỊCH — cố ý:
// lỗi lúc build tốt hơn nhiều so với một hàm rỗng trả về giá trị giả lúc bay.
//
// Các KIỂU dữ liệu (struct mẫu đo) vẫn được khai báo bình thường vì
// sensor_snapshot_t/telemetry vẫn có ô cho cảm biến này — chúng chỉ luôn ở
// trạng thái "chưa từng có mẫu".
#if FC_FEATURE_TOF

// ============================================================
// Init
// ============================================================
//
// bus          : I2C master bus đã tạo bởi flight_core (init_i2c_bus()).
// xshut_gpio   : chân XSHUT. **< 0 = KHÔNG NỐI, hoàn toàn hợp lệ** — lúc đó
//                driver bỏ qua bước reset cứng và dựa vào pull-up trên module.
// i2c_addr     : địa chỉ 7-bit MONG MUỐN. Driver tự dò cả địa chỉ này LẪN
//                default 0x29, rồi đổi nếu cần. Đổi thất bại KHÔNG phải lỗi
//                cứng (chỉ 1 sensor, không ai tranh địa chỉ) — driver ở lại
//                địa chỉ tìm thấy và vẫn chạy.
// scl_speed_hz : tốc độ SCL của device handle (nguồn: BOARD_I2C_FREQ_HZ).
//
// Trả ESP_ERR_NOT_FOUND nếu KHÔNG có gì trả lời ACK ở cả hai địa chỉ — lúc đó
// log chỉ rõ thứ tự cần kiểm (nguồn -> dây SDA/SCL -> lệnh console 'i2c_scan').
esp_err_t tof_driver_init(i2c_master_bus_handle_t bus, int xshut_gpio,
                           uint8_t i2c_addr, uint32_t scl_speed_hz);


// ============================================================
// Read — non-blocking
// ============================================================
//
// ESP_OK = giao tiếp I2C thành công. Measurement mới chưa hợp lệ -> out->valid
// = false. Driver giữ mẫu cũ qua các lỗi bus thoáng qua, tối đa
// TOF_STALE_TIMEOUT_MS.
//
// Gọi TỪ sensor_hub task (chủ sở hữu DUY NHẤT bus I2C) — KHÔNG gọi từ
// stabilize_task.
esp_err_t tof_driver_read(tof_reading_t *out);

// ============================================================================
// STALL WATCHDOG — VL53L0X TREO GIỮA CHỪNG LÀ CHUYỆN CÓ THẬT
// ============================================================================
// Chế độ continuous back-to-back: chip tự đo liên tục và bật cờ ngắt mỗi lần có
// mẫu mới; driver chỉ poll RESULT_INTERRUPT_STATUS rồi clear. Có một trạng thái
// hỏng ĐƯỢC BÁO CÁO RỘNG RÃI: chip ngừng hẳn việc bật cờ đó — I2C vẫn ACK, vẫn
// đọc được thanh ghi, MODEL_ID vẫn đúng, nhưng KHÔNG BAO GIỜ có mẫu mới nữa.
//
// Nhìn từ ngoài nó y hệt "vật thể ngoài tầm", nên nếu chỉ nhìn `valid` thì
// không phân biệt được "không có gì để đo" với "cảm biến đã chết". Bản trước
// của driver này rơi đúng vào đó: poll_sensor() trả ESP_OK mãi mãi khi không có
// mẫu mới, không có đường phục hồi nào — cảm biến chết là chết luôn tới khi
// reboot.
//
// Nguyên nhân thường gặp (xem README mục ToF): sụt áp lúc motor rút dòng
// (VCSEL peak 40mA theo datasheet ST), nhiễu I2C từ dây motor, hoặc xung đột
// lịch truy cập bus.
//
// tof_driver_stall_restarts() = số lần watchdog đã phải khởi động lại ranging.
// Ở phần cứng lành lặn con số này phải ĐỨNG YÊN. Tăng đều = phần cứng đang
// hỏng thật (nguồn/nhiễu) và cần sửa mạch, KHÔNG phải sửa phần mềm — watchdog
// chỉ mua thêm thời gian, nó không làm cảm biến đáng tin trở lại.
uint32_t tof_driver_stall_restarts(void);

// tof_driver_poll_stats() — số liệu THÔ của đường poll, để phân biệt ba tình
// huống mà bề ngoài giống hệt nhau (valid=0, range_status=255, stall=0):
//
//   calls=0, not_ready>0  -> hub CÓ gọi nhưng driver chưa sẵn sàng
//                            (dev=NULL hoặc ranging_started=0)
//   calls=0, not_ready=0  -> hub KHÔNG hề gọi (lịch poll sai / tof_present=0)
//   calls>0, io_errors≈calls -> chip KHÔNG trả lời trên I2C (last_err chỉ rõ)
//   calls>0, io_errors=0     -> bus tốt, chip chỉ chưa sinh mẫu nào
//
// VÌ SAO CẦN: khi backend lỗi I2C, poll_with_watchdog() return NGAY, trước cả
// khối stall watchdog — nên stall_restarts đứng yên và trông như "chip khoẻ".
// Không có bộ đếm này thì không cách nào phân biệt bằng quan sát.
//
// Con trỏ nào không cần thì truyền NULL. Chỉ đọc, an toàn khi đang bay.
void tof_driver_poll_stats(uint32_t *calls, uint32_t *io_errors,
                           uint32_t *not_ready, int *last_err);

// Tên dòng chip ĐANG được biên dịch vào ("VL53L0X" / "VL53L1X"). Hằng chuỗi
// tĩnh, luôn khác NULL. Dùng cho log/telemetry để người đọc biết firmware này
// build cho chip nào — KHÔNG phải kết quả dò lúc chạy.
const char *tof_driver_chip_name(void);


#endif  // FC_FEATURE_TOF
#ifdef __cplusplus
}
#endif
