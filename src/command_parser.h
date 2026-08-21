// command_parser — diễn giải giao thức text ground-station (KHỚP
// tools/uav_udp_console.py + PORT tinh thần từ UAV-Mini pid_link.cpp/
// mahony_link.cpp/flight_control.cpp phần xử lý lệnh @SP/@TRIM/@ALT/@TKO/@LAND)
// thành command_t rồi đẩy vào flight_core qua flight_core_push_command().
//
// KHÔNG chạy ở đây: PID/mixer/estimator (đó là việc của flight_core.c, chạy
// TRONG stabilize_task 250Hz) — module này CHỈ parse text -> command_t/getter
// calls, KHÔNG tính toán điều khiển gì. Gọi từ 1 task DUY NHẤT (net_task, xem
// src/main.c) — KHÔNG gọi từ stabilize_task (giữ đúng ranh giới "không parse
// string trong control loop").
//
// Giao thức (xem README mục "Ground station qua WiFi/UDP" để biết đầy đủ):
//   Lệnh 1 ký tự tức thời: r=ARM k=KILL d=DISARM(chỉ từ ARMED, xem CMD_DISARM)
//     t=TAKEOFF l=LAND >/<=alt step p=heartbeat("PONG", xem CMD_HEARTBEAT —
//     ĐƯỜNG DUY NHẤT qua UDP feed Commander watchdog, xem command_parser.c
//     case 'p' cho lý do QUAN TRỌNG phải gửi định kỳ khi đang bay)
//     f=VÀO flight mode (bật streaming STATUS định kỳ) q=THOÁT flight mode
//     (tắt streaming — xem command_parser_telemetry_enabled()) s=status h/?=help
//   Lệnh nhiều ký tự bắt đầu bằng '@', kết thúc '\n':
//     @PID GET | SET <ANGLE|RATE> <ROLL|PITCH|YAW> kp ki kd ilim outlim
//     @MAH GET | SET <kp> <ki>
//     @SP  GET | SET <roll_deg> <pitch_deg> <yaw_rate_dps>
//     @TRIM GET | SET <roll_deg> <pitch_deg>
//     @ALT GET | SET <kp> <vzkp> <vzki> <vzilim> | TGT <m> | MODE <0-4> | TAKEOFF
//     @TKO GET | SET <hover> <spool_duty> <spool_ms>
//     @LAND (bare, kích landing) | GET | SET <dvz> <flarealt> <fvz> <tdalt>
//     @HOVER <sec> | @MOVE <dir> <pct 0-100> <sec> | @YAW <deg>  — timed-command
//           (xem command.h CMD_HOVER/CMD_MOVE/CMD_SET_YAW), KHÔNG GET
//     @TEST <motor 1-4> <duty_pct 0-100> — spin 1 động cơ ~500ms, CHỈ DISARMED
//           + đã THÁO CÁNH QUẠT (xem command.h CMD_TEST_MOTOR), KHÔNG GET
//     @CMDR GET | SET <altmin_m> <altmax_m> <battfloor_v> <hbtimeout_ms>
//           <hardtilt_deg> <motorsat_ms>  — Commander (geofence + failsafe),
//           chạy BẤT KỲ LÚC NÀO kể cả đang bay, xem commander.h
#pragma once

#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// command_parser_feed_byte() — nạp 1 byte từ net_link_read() vào state machine
// nội bộ (chế độ dòng '@...' hoặc lệnh 1 ký tự tức thời, xem đầu file). Nếu
// byte này hoàn tất 1 lệnh CÓ REPLY, ghi reply (kết thúc bằng \n, có thể
// NHIỀU dòng — vd "PID GET") vào out (null-terminated) và trả true — caller
// (net_task) tự net_link_write(out, strlen(out)). Trả false nếu byte này
// chưa hoàn tất lệnh nào (đang tích lũy) hoặc lệnh không cần reply.
bool command_parser_feed_byte(int c, char *out, size_t out_size);

// command_parser_telemetry_enabled() — true SAU KHI người dùng bấm phím 'f'
// (vào "flight mode", nút "Flight" trên GUI), false SAU KHI bấm 'q' (thoát
// flight mode, nút "Exit"). Một chiều bật/tắt (KHÔNG phải toggle trên cùng 1
// phím) — bấm lại 'f' khi đã bật hoặc 'q' khi đã tắt chỉ xác nhận lại trạng
// thái, không đổi gì (idempotent). net_task() (main.c) CHỈ gửi STATUS định kỳ
// khi cờ này true. Mặc định false lúc mới boot/kết nối — UDP peer KHÔNG tự
// bật streaming. KHÔNG ảnh hưởng reply trực tiếp của lệnh khác (@CAL/@PID/../
// single-char) — những cái đó LUÔN hoạt động bất kể cờ này, chỉ STATUS định
// kỳ bị gate.
bool command_parser_telemetry_enabled(void);

#ifdef __cplusplus
}
#endif
