// fc_bridge — lớp phiên dịch DUY NHẤT giữa MicroPython (mp_obj_t) và
// flight_core (command_t / telemetry_snapshot_t). KHÔNG có logic điều khiển ở
// đây — chỉ marshal tham số + gọi 2 hàm biên giới của flight_core.h.
#pragma once

#include "flight_core/command.h"
#include "flight_core/telemetry.h"

#ifdef __cplusplus
extern "C" {
#endif

// fc_bridge_init() — đọc board_config.h, dựng flight_core_board_config_t, gọi
// flight_core_start(). Gọi MỘT LẦN lúc module fc được import lần đầu (xem
// fc_module.c: mp_module fc, hàm nào gọi cái này thì xem MP_REGISTER_MODULE +
// hook init trong fc_module.c).
void fc_bridge_init(void);

// Trả false nếu command queue đầy — fc_module.c PHẢI raise exception Python
// tương ứng (KHÔNG được nuốt lỗi).
bool fc_bridge_push(const command_t *cmd);

void fc_bridge_read_telemetry(telemetry_snapshot_t *out);

// dir dạng chuỗi ("forward"/"back"/"left"/"right"/"up"/"down"/"cw"/"ccw") ->
// move_dir_t. Trả false nếu chuỗi không hợp lệ (fc_module.c raise ValueError).
bool fc_bridge_parse_move_dir(const char *s, move_dir_t *out);

// fc_bridge_get_control_input() — đọc lại rol/pit/yaw/thr (-100..100) từ
// CMD_CONTROL gần nhất, dùng cho fc.get_states(). Xem flight_core_get_control_input().
void fc_bridge_get_control_input(float *rol_pct, float *pit_pct, float *yaw_pct, float *thr_pct);

#ifdef __cplusplus
}
#endif
