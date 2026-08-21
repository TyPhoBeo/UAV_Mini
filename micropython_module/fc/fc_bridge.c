#include "fc_bridge.h"

#include <string.h>

#include "flight_core/flight_core.h"

// board_setup.h nằm ở main/ (gốc repo) cạnh board_config.h/app_config.h — nó
// tự kéo theo cả hai file đó. CHỈ include TỪ ĐÂY, không rải include ở nơi khác.
// Nếu checkout MicroPython thật của anh đặt 3 file này chỗ khác, sửa DUY NHẤT
// đường dẫn include này.
#include "../../main/board_setup.h"

static bool s_started = false;

void fc_bridge_init(void) {
    if (s_started) return;

    // Cấu hình phần cứng dựng ở MỘT chỗ duy nhất (main/board_setup.h) và dùng
    // chung với src/main.c — trước đây 20 dòng gán này bị chép ra hai bản, mà
    // file này KHÔNG nằm trong build nên bản ở đây có thể trôi đi rất lâu mà
    // không ai biết.
    flight_core_board_config_t cfg;
    board_config_fill(&cfg);

    flight_core_start(&cfg);
    s_started = true;
}

bool fc_bridge_push(const command_t *cmd) {
    fc_bridge_init();   // an toàn nếu Python gọi lệnh trước khi import hook chạy
    return flight_core_push_command(cmd);
}

void fc_bridge_read_telemetry(telemetry_snapshot_t *out) {
    fc_bridge_init();
    flight_core_read_telemetry(out);
}

void fc_bridge_get_control_input(float *rol_pct, float *pit_pct, float *yaw_pct, float *thr_pct) {
    fc_bridge_init();
    flight_core_get_control_input(rol_pct, pit_pct, yaw_pct, thr_pct);
}

bool fc_bridge_parse_move_dir(const char *s, move_dir_t *out) {
    if (strcmp(s, "forward") == 0) { *out = MOVE_FORWARD; return true; }
    if (strcmp(s, "back") == 0)    { *out = MOVE_BACK;    return true; }
    if (strcmp(s, "left") == 0)    { *out = MOVE_LEFT;    return true; }
    if (strcmp(s, "right") == 0)   { *out = MOVE_RIGHT;   return true; }
    if (strcmp(s, "up") == 0)      { *out = MOVE_UP;      return true; }
    if (strcmp(s, "down") == 0)    { *out = MOVE_DOWN;    return true; }
    if (strcmp(s, "cw") == 0)      { *out = MOVE_CW;      return true; }
    if (strcmp(s, "ccw") == 0)     { *out = MOVE_CCW;     return true; }
    return false;
}
