// telemetry_format — build dòng STATUS text từ telemetry_snapshot_t, GIỮ
// NGUYÊN format lõi của UAV-Mini flight_control.cpp::flight_pipeline_status_line()
// (khớp STATUS_RE/PLOT_RE/ALT_PLOT_RE/ACC_PLOT_RE/MOTOR_RE trong
// tools/uav_udp_console.py) rồi NỐI THÊM field UAV-S3-specific ở cuối dòng
// (an toàn — STATUS_RE của Python đã cập nhật để chấp nhận đuôi mở rộng, xem
// README). KHÔNG parse/tính toán gì — chỉ snprintf từ snapshot đã có.
#pragma once

#include <stddef.h>

#include "flight_core/flight_core.h"

#ifdef __cplusplus
extern "C" {
#endif

// telemetry_format_status_line() — đọc flight_core_read_telemetry() rồi
// snprintf 1 dòng (kết thúc \n) vào out. Gọi từ net_task (xem src/main.c),
// KHÔNG gọi từ stabilize_task.
void telemetry_format_status_line(char *out, size_t out_size);

// telemetry_format_alt_mode() — derive pseudo alt_mode (0=OFF 1=LOG_ONLY
// 2=HOLD 3=TAKEOFF 4=LANDING) từ FSM state THẬT, dùng CHUNG giữa STATUS line
// (MODE=) và @ALT GET (mode=) — xem command_parser.c mục @ALT cho giải thích
// đầy đủ vì sao UAV-S3 không có "alt_mode" độc lập như UAV-Mini.
int telemetry_format_alt_mode(fsm_state_t state);

#ifdef __cplusplus
}
#endif
