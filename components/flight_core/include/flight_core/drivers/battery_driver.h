// Battery voltage driver — ADC1 oneshot + calibration (esp_adc), ĐÃ implement
// thật (không phải TODO stub, khác các driver cảm biến khác trong thư mục
// này). BẮT BUỘC dùng ADC1 — ADC2 xung đột WiFi trên ESP32-S3 (xem
// board_config.h). Kênh (channel) là số kênh ADC1 (0..9), KHÔNG phải số GPIO
// — tra bảng mapping GPIO<->kênh ADC1 trong board_config.h.
//
// ============================================================================
// CHIA ÁP LÀ HẰNG SỐ PHẦN CỨNG — KHÔNG PHẢI THAM SỐ CỦA CALLER
// ============================================================================
// Schematic miniUav rev1.0:
//
//     VBAT ──[ R4 = 22k ]──┬── ADC_PIN (GPIO6 / ADC1_CH5)
//                          │
//                        [ R5 = 10k ]
//                          │
//                         GND
//
//   Vadc = VBAT * R_BOTTOM / (R_TOP + R_BOTTOM) = VBAT * 10/32
//   VBAT = Vadc * (R_TOP + R_BOTTOM) / R_BOTTOM = Vadc * 3.2
//
// Trước đây ratio này là THAM SỐ truyền từ tầng trên (board_config.h ->
// flight_core_board_config_t -> sensor_hub_start() -> battery_driver_read_v()).
// Sai về mặt thiết kế: đây là hằng số VẬT LÝ của bo mạch, không phải lựa chọn
// runtime — truyền qua 4 tầng chỉ tạo ra 4 chỗ có thể gán nhầm (và ĐÃ có 1 lần
// tầng trên quên gán -> ratio = 1.0 -> điện áp báo về sai 3.2 lần mà không có
// gì báo lỗi). Giờ định nghĩa TẠI ĐÂY, cạnh chính công thức dùng nó.
#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "flight_core/fc_features.h"

#ifdef __cplusplus
extern "C" {
#endif

// Giá trị điện trở THẬT theo schematic — sửa 2 số này (và CHỈ 2 số này) nếu
// đổi bo. BATTERY_DIVIDER_RATIO tự suy ra, không gõ tay 3.2f để không bao giờ
// lệch giữa "điện trở ghi trong comment" và "hệ số thực dùng".
#define BATTERY_R_TOP_OHM       10000.0f
#define BATTERY_R_BOTTOM_OHM    10000.0f
#define BATTERY_DIVIDER_RATIO \
    ((BATTERY_R_TOP_OHM + BATTERY_R_BOTTOM_OHM) / BATTERY_R_BOTTOM_OHM)   // = 3.2f

// ---- Dải hợp lệ cho pin 1S LiPo (xem tuning.h mục 5/10) ----
// Rộng hơn dải làm việc thật (3.0-4.2V) để KHÔNG loại nhầm pin cạn sâu hay
// vừa sạc đầy hơi cao, nhưng vẫn đủ chặt để bắt các lỗi thang đo kinh điển:
// ratio sai (báo ~1.3V nếu ratio=1.0, hoặc ~13V nếu ratio=10), đứt divider
// (~0V hoặc ~3.3V kẹp trần), hoặc lắp nhầm pin 2S (~6-8.4V).
//
// LƯU Ý QUAN TRỌNG: ngưỡng này KHÔNG "sửa" được điện áp sai — nó chỉ làm cho
// số sai TRỞ NÊN NHÌN THẤY ĐƯỢC (valid=false) thay vì âm thầm chảy vào
// commander/battery-compensation. Đọc ra 6.07V trên bo cắm pin 1S nghĩa là
// phần cứng có vấn đề THẬT (pin 2S cắm nhầm, hoặc R4/R5 khác schematic) —
// PHẢI đo bằng vôn kế, không được nới ngưỡng này cho "hết báo lỗi".
#define BATTERY_MIN_VALID_V     2.0f
#define BATTERY_MAX_VALID_V     4.5f

// Một mẫu pin ĐẦY ĐỦ — gồm cả số liệu thô để chẩn đoán thang đo từ xa mà
// không cần cắm dây debug (xem telemetry BAT_RAW/BAT_MV trong
// telemetry_format.c). Trước đây driver CHỈ trả điện áp cuối, nên khi số sai
// không có cách nào biết sai ở khâu nào (ADC? calibration? ratio?).
typedef struct {
    int   raw;          // giá trị ADC thô (0..4095 @ 12-bit)
    int   adc_mv;       // điện áp TẠI CHÂN ADC (mV) — sau adc_cali nếu có
    float voltage_v;    // = adc_mv/1000 * BATTERY_DIVIDER_RATIO (điện áp pin)
    bool  calibrated;   // true = qua adc_cali_raw_to_voltage(); false = fallback tuyến tính THÔ
    bool  valid;        // true CHỈ khi calibrated VÀ voltage_v trong [MIN,MAX]_VALID_V
} battery_sample_t;

// ================= CỜ BIÊN DỊCH =================
// Toàn bộ khai báo hàm dưới đây nằm sau `#if FC_FEATURE_BATTERY`. Cờ = 0 (đặt qua
// SENSOR_BATTERY_ENABLED trong main/app_config.h) thì driver KHÔNG được biên
// dịch vào firmware, và mọi lời gọi còn sót lại sẽ là LỖI BIÊN DỊCH — cố ý:
// lỗi lúc build tốt hơn nhiều so với một hàm rỗng trả về giá trị giả lúc bay.
//
// Các KIỂU dữ liệu (struct mẫu đo) vẫn được khai báo bình thường vì
// sensor_snapshot_t/telemetry vẫn có ô cho cảm biến này — chúng chỉ luôn ở
// trạng thái "chưa từng có mẫu".
#if FC_FEATURE_BATTERY

// battery_driver_init() — tạo ADC1 oneshot unit + config channel (12-bit,
// ADC_ATTEN_DB_12 ~0-3.3V) + calibration scheme (curve-fitting). Scheme không
// khả dụng -> VẪN trả ESP_OK và vẫn đọc được (fallback tuyến tính thô) nhưng
// mọi mẫu sẽ có calibrated=false VÀ valid=false — xem battery_driver_read().
esp_err_t battery_driver_init(int adc1_channel);

// battery_driver_read() — đọc 1 mẫu ĐẦY ĐỦ (raw + mV + volt + cờ hợp lệ).
//
// valid=false có nghĩa "KHÔNG ĐƯỢC dùng cho quyết định bay" (low-battery
// failsafe, battery compensation) — caller PHẢI coi như chưa có mẫu, KHÔNG
// được dùng voltage_v trong nhánh đó. voltage_v vẫn được điền để hiện lên
// telemetry phục vụ chẩn đoán (xem BAT_RAW/BAT_MV/BAT_VALID) — nhìn thấy số
// sai quan trọng hơn nhiều so với giấu nó đi.
//
// Trả ESP_ERR_INVALID_STATE nếu chưa init, hoặc lỗi ADC nguyên vẹn từ
// adc_oneshot_read()/adc_cali_raw_to_voltage().
esp_err_t battery_driver_read(battery_sample_t *out);

// battery_driver_read_v() — tiện ích cho caller CHỈ cần điện áp hợp lệ.
// Mẫu không hợp lệ (xem battery_driver_read()) -> trả ESP_ERR_INVALID_RESPONSE
// và KHÔNG ghi *out_voltage, để caller không vô tình dùng số rác.
esp_err_t battery_driver_read_v(float *out_voltage);

#endif  // FC_FEATURE_BATTERY
#ifdef __cplusplus
}
#endif
