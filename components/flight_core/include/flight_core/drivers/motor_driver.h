// Motor driver — LEDC PWM 4 kênh. PORT THẬT (không stub) từ MotorOutput::init()/
// write_channel() (UAV-Mini) — cấu hình LEDC không phụ thuộc board cụ thể,
// chỉ 4 số GPIO là TODO (board_config.h). LEDC API giống hệt trên ESP32-S3.
#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// 24kHz khớp schematic miniUav (N-MOSFET KAO3400, ~24kHz — xem board_config.h),
// 11-bit (max vật lý 2047 — 80MHz APB / 24kHz ~= 3333 >= 2^11, vẫn đủ chia hết
// dù 2^12=4096 thì không), SAFE_MAX_DUTY=2000 để chừa lề (~2.3%,
// khớp tỷ lệ chừa lề của bản 10-bit cũ 1000/1023). TOÀN BỘ gain PID + hằng số
// duty-domain trong flight_core/tuning.h đã NHÂN ĐÔI theo thang mới này —
// ĐỪNG đổi SAFE_MAX_DUTY/độ phân giải một mình ở đây mà không soát lại
// tuning.h (mọi giá trị "đơn vị duty" phải tỷ lệ cùng nhau, xem tuning.h mục 1-4).
#define MOTOR_PWM_FREQ_HZ    24000
#define MOTOR_PWM_MAX_DUTY   2047
#define MOTOR_SAFE_MAX_DUTY  2000

// motor_driver_init() — motor_gpio[4] lấy từ board_config.h (main/, TODO điền
// theo schematic), thứ tự M1..M4 khớp mixer Quad-X trong attitude_control.h.
// Cấu hình LEDC timer 0 (low-speed, 11-bit, 24kHz) + 4 channel.
esp_err_t motor_driver_init(const int motor_gpio[4]);

void motor_driver_arm(void);       // reset_pid tương ứng do caller (attitude_state_reset) tự gọi
void motor_driver_disarm(void);
bool motor_driver_is_armed(void);

// ============================================================================
// HARDWARE ARMED GATE — lớp phòng thủ CUỐI CÙNG cho motor output.
// ============================================================================
// Khi armed=false, motor_driver_set_duties() BỊ ÉP về 0 NGAY TRONG DRIVER,
// bất kể caller truyền duty gì. Đây là bảo đảm cấu trúc, KHÔNG phải quy ước:
// dù tầng trên có bug/đường code nào lỡ gọi set_duties() sau khi đã kill, phần
// cứng vẫn không quay.
//
// VÌ SAO CẦN: trước đây motor_driver_disarm() chỉ ghi 0 MỘT LẦN rồi hạ cờ
// s_armed, nhưng set_duties() KHÔNG hề đọc cờ đó — nên bất kỳ lệnh ghi duty
// nào sau đó trong CÙNG tick (attitude cascade + mixer chạy tiếp ở bước 10)
// đều bật motor trở lại. Cắt motor rồi để code chạy tiếp là một lỗi an toàn,
// không phải chi tiết cài đặt.
//
// motor_driver_arm() tự set armed=true; motor_driver_disarm() tự set false.
// Hàm này để tầng trên (kill latch trong flight_core.c) đóng gate ĐỘC LẬP với
// trạng thái arm/disarm logic.
void motor_driver_set_armed(bool armed);

// Ghi duty 4 motor (đã clamp [0, MOTOR_SAFE_MAX_DUTY] bởi caller — driver chỉ
// clamp lại lần cuối vào [0, MOTOR_PWM_MAX_DUTY] cho chắc). BỊ ÉP VỀ 0 nếu
// armed gate đang đóng (xem motor_driver_set_armed()).
void motor_driver_set_duties(int m1, int m2, int m3, int m4);
void motor_driver_stop_all(void);   // set cả 4 về 0, KHÔNG đổi armed state

// motor_driver_all_off() — cắt motor + ĐÓNG LUÔN armed gate (khác
// stop_all() vốn chỉ ghi 0 và để gate mở). Dùng cho mọi đường KILL/hard fault:
// sau lệnh này KHÔNG đường code nào ghi được duty khác 0 cho tới khi
// motor_driver_arm()/set_armed(true) được gọi lại tường minh.
void motor_driver_all_off(void);

// motor_id: 0=M1 .. 3=M4.
int motor_driver_get_last_duty(int motor_id);

#ifdef __cplusplus
}
#endif
