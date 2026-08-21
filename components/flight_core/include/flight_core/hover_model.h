// hover_model.h — ước lượng ga hover từ ĐIỆN ÁP PIN, chốt MỘT LẦN lúc ARM.
//
// ============================================================================
// VẤN ĐỀ NÓ GIẢI QUYẾT
// ============================================================================
// Cascade Vz (alt_hold.h) tính throttle = hover_ff + I. hover_ff TRƯỚC ĐÂY là
// hằng số biên dịch ALT_HOLD_HOVER_NOMINAL = 1000 duty, KHÔNG phụ thuộc pin.
// Nhưng hover THẬT của con drone này phụ thuộc pin rất mạnh (số đo bench-ramp
// của người dùng):
//
//        pin 4.2V -> hover ~ 900 duty
//        pin 3.6V -> hover ~ 1300..1400 duty
//
// Bay với pin 3.6V thì hover_ff=1000 hụt ~350 duty, và toàn bộ khoảng hụt đó
// phải do thành phần I của vòng Vz tự bò lên mà bù (ALT_HOLD_VZ_ILIMIT=500 nên
// nó bù ĐƯỢC, chỉ là RẤT CHẬM: dI/dt = Ki*vz_err). Đó chính là triệu chứng
// "arm xong drone nằm ì hơn chục giây rồi mới nhúc nhích" — không phải lỗi
// tune Ki, mà là feedforward sai điểm neo ngay từ đầu.
//
// ============================================================================
// VÌ SAO LATCH MỘT LẦN, KHÔNG CẬP NHẬT LIÊN TỤC — ĐỌC KỸ TRƯỚC KHI "CẢI TIẾN"
// ============================================================================
// Firmware này ĐÃ TỪNG có bù pin liên tục (throttle *= NOMINAL_V/vbat) và nó
// đã bị GỠ BỎ có chủ đích — xem flight_core.c bước 9b. Lý do, đo được từ log
// bench thật: VBAT không chỉ phản ánh "pin còn bao nhiêu", nó TỤT THEO TẢI
// TỨC THÌ. Nhấc drone lên -> 4 motor rút dòng -> sụt áp nội trở -> VBAT đo
// giảm dù pin còn đầy. Nhân throttle theo con số đó tạo VÒNG PHẢN HỒI DƯƠNG
// ký sinh nằm NGOÀI mọi vòng PID đã tune:
//
//        ga lên -> dòng tăng -> VBAT giảm -> bù tăng -> ga lên nữa -> ...
//
// Bằng chứng: BATV dao động 3.51..3.82V trong VÀI GIÂY ở tải gần như không
// đổi -> hệ số bù nhảy 1.10..1.20, tức nhiễu áp bị KHUẾCH ĐẠI thẳng vào ga.
//
// LATCH KHÔNG DÍNH LỖI ĐÓ, vì hai lý do ĐỘC LẬP:
//   1. Nó đọc vbat ĐÚNG MỘT LẦN, lúc ARM, khi motor CHƯA QUAY (throttle bị
//      khoá 0 ở DISARMED) -> đo được điện áp KHÔNG TẢI, đúng đại lượng mà
//      model cần. Sau đó giá trị ĐÓNG BĂNG: không có đường phản hồi nào từ
//      ga trở lại hover_ff, nên không có vòng lặp nào để mà dương.
//   2. Nó neo vào hover_ff (một SỐ HẠNG cộng), KHÔNG nhân vào output cuối.
//      PID vẫn xuất ra đúng duty nó tính; không có hệ số lạ nhân sau lưng làm
//      sai lệch mọi gain đã tune ở bench.
//
// Pin tụt DẦN trong lúc bay: để I của vòng Vz tự bù, đúng như nó vẫn luôn làm.
// Phần I chỉ còn phải bù độ trôi chậm vài chục duty thay vì cả hố 350 duty.
//
// ============================================================================
// MODEL
// ============================================================================
//        hover(V) = HOVER_MODEL_REF_DUTY * (V_REF / V) ^ HOVER_MODEL_EXP
//
// Fit từ ĐÚNG 2 điểm đo bench-ramp ở trên. Kiểm chứng:
//        900 * (4.2/3.6)^2.6 = 1344 duty  -> nằm trong dải đo 1300..1400 ✓
// Exponent khớp chính xác cho điểm giữa 1350 là 2.630, nên 2.6 là fit đúng.
//
// ⚠ HAI ĐIỂM ĐO = model 2 tham số. Nó KHÔNG có bậc tự do nào để tự phát hiện
// mình sai. Ngoài dải 3.4..4.2V nó là NGOẠI SUY thuần tuý -> clamp cứng
// [MIN,MAX]_DUTY là bắt buộc, không phải phòng xa.
//
// ⚠ Đo lại 2 điểm này nếu đổi motor/cánh/pin/khối lượng khung. Chúng là số đo
// của MỘT con drone cụ thể, không phải hằng số vật lý.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "flight_core/drivers/motor_driver.h"   // MOTOR_SAFE_MAX_DUTY
#include "flight_core/fc_features.h"
#include "flight_core/tuning.h"                 // ATT_MAX_COLLECTIVE_FRACTION,
                                                 // TAKEOFF_PRIME_HOVER_FRAC

#ifdef __cplusplus
extern "C" {
#endif

// ---- Hai điểm đo bench-ramp (xem đầu file) ----
#define HOVER_MODEL_REF_DUTY      900.0f   // hover tại V_REF
#define HOVER_MODEL_V_REF         4.2f
#define HOVER_MODEL_EXP           2.6f     // fit từ (4.2,900) và (3.6,1350)

// ---- Clamp cứng ----
// MAX **SUY RA**, KHÔNG gõ tay 1700: nó CHÍNH LÀ trần collective đã có sẵn
// trong tuning.h. Vượt qua đó thì mixer hết dải tạo mô-men roll/pitch/yaw ->
// "đủ ga" nhưng MẤT LÁI. Một đại lượng vật lý -> một hằng số: gõ 1700 ở đây
// sẽ tạo bản sao thứ hai và bảo đảm có ngày hai số lệch nhau khi ai đó chỉnh
// ATT_MAX_COLLECTIVE_FRACTION mà quên file này.
#define HOVER_MODEL_MAX_DUTY \
    ((float)MOTOR_SAFE_MAX_DUTY * ATT_MAX_COLLECTIVE_FRACTION)   // = 1700

// MIN là đại lượng ĐỘC LẬP (biên dưới vùng hợp lệ của model), không phải bản
// sao của hằng số nào — định nghĩa tại đây là đúng chỗ.
#define HOVER_MODEL_MIN_DUTY      800.0f

// Dưới ngưỡng này TỪ CHỐI ARM. Không phải "pin yếu thì bay dè" — mà là model
// đã hết vùng hợp lệ: 900*(4.2/3.4)^2.6 = 1559 duty, còn 3.2V cho ra 1825 tức
// ĐÃ vượt clamp MAX. Bay tiếp nghĩa là hover_ff bị clamp thấp hơn hover thật
// và cascade lại rơi về đúng cái hố mà module này sinh ra để lấp.
#define HOVER_MODEL_MIN_LATCH_V   3.4f

// Tỷ lệ PRIME/hover KHÔNG định nghĩa ở đây — nó là một NÚM TUNE, sống trong
// tuning.h dưới tên TAKEOFF_PRIME_HOVER_FRAC, và được dùng ở CẢ HAI nơi:
//   - tuning.h : TAKEOFF_PRIME_DUTY (giá trị khởi tạo, khi chưa latch)
//   - file này : hover_model_prime_duty() (giá trị thật, sau khi latch)
// Nếu định nghĩa riêng một bản ở đây thì ga PRIME lúc chưa latch và sau khi
// latch sẽ theo hai tỷ lệ khác nhau mà không ai nhận ra.

// Số mẫu vbat gộp lại khi latch. Pin lấy mẫu ở 10Hz (sensor_hub 250Hz /
// SENSOR_BATTERY_DIVISOR 25) -> 5 mẫu = ~500ms lịch sử.
//
// TRUNG VỊ chứ không phải trung bình: ADC pin thỉnh thoảng nhả một mẫu lệch
// hẳn (nhiễu chuyển mạch). Trung bình để mẫu đó kéo kết quả đi; trung vị thì
// không. Và trung vị của 5 mẫu KHÔNG cần bộ lọc chạy nền — chỉ cần lịch sử,
// nên không thêm state động nào vào control loop.
#define HOVER_MODEL_SAMPLES       5
#define HOVER_MODEL_MIN_SAMPLES   3   // ít hơn -> từ chối latch (từ chối ARM)

// hover_model_from_voltage() — hàm THUẦN. vbat <= 0 trả HOVER_MODEL_MIN_DUTY
// (caller PHẢI tự kiểm valid trước; đây chỉ là chặn chia-0, không phải guard).
// Kết quả LUÔN nằm trong [MIN_DUTY, MAX_DUTY].
float hover_model_from_voltage(float vbat_v);

// hover_model_prime_duty() — ga PRIME suy ra từ hover đã latch. Tách hàm riêng
// để chỉ có MỘT nơi định nghĩa quan hệ prime<->hover.
int hover_model_prime_duty(float hover_duty);

// ---- Vòng đệm vbat: chỉ nhận mẫu MỚI (theo seq của sensor_health_t) ----
// Ghi ở control loop 250Hz nhưng chỉ nạp khi seq đổi (10Hz), nên không có
// chuyện một mẫu bị đếm 25 lần và làm trung vị mất ý nghĩa.
typedef struct {
    float    v[HOVER_MODEL_SAMPLES];
    int      count;      // số ô đã dùng (bão hoà ở HOVER_MODEL_SAMPLES)
    int      head;       // ô ghi tiếp theo
    uint32_t last_seq;   // seq của mẫu đã nạp gần nhất; 0 = chưa nạp gì
} hover_vbat_ring_t;

void hover_vbat_reset(hover_vbat_ring_t *r);

// hover_vbat_push() — nạp nếu (valid && seq != last_seq && seq != 0).
// Mẫu KHÔNG hợp lệ bị BỎ QUA HOÀN TOÀN (không nạp, không xoá lịch sử cũ):
// battery_driver.h đã định nghĩa valid=false là "không được dùng cho quyết
// định bay", mà latch hover chính là một quyết định bay.
void hover_vbat_push(hover_vbat_ring_t *r, float v, uint32_t seq, bool valid);

// hover_vbat_median() — trung vị. Trả false (KHÔNG ghi *out) nếu chưa đủ
// HOVER_MODEL_MIN_SAMPLES mẫu.
bool hover_vbat_median(const hover_vbat_ring_t *r, float *out_v);

#ifdef __cplusplus
}
#endif
