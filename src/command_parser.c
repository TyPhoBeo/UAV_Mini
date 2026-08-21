// command_parser.c — xem command_parser.h cho tổng quan giao thức. PORT tinh
// thần từ UAV-Mini pid_link.cpp/mahony_link.cpp/flight_control.cpp (các hàm
// handle_setpoint_line/handle_trim_line/handle_altitude_line/handle_takeoff_line/
// handle_landing_line) — giữ NGUYÊN VĂN cú pháp lệnh + format reply để
// tools/uav_udp_console.py không cần sửa gì phía parse.
#include "command_parser.h"

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "flight_core/flight_core.h"
#include "telemetry_format.h"

// ================= '@'-line buffering + single-char state machine =================
// PORT từ handle_pid_line_char() (UAV-Mini flight_control.cpp).

#define LINE_BUF_SIZE 128
static char s_line_buf[LINE_BUF_SIZE];
static size_t s_line_len = 0;
static bool s_line_mode = false;

// Mặc định TẮT — học được UDP peer (net_link.c, xem main.c::net_task()) KHÔNG
// tự bật streaming STATUS định kỳ nữa. Bấm 'f' (nút "Flight" GUI) để bật —
// xem command_parser_telemetry_enabled() ở command_parser.h.
static bool s_telemetry_enabled = false;

bool command_parser_telemetry_enabled(void) { return s_telemetry_enabled; }

static void to_upper_inplace(char *s) {
    for (; *s != '\0'; ++s) *s = (char)toupper((unsigned char)*s);
}

static void append(char *out, size_t out_size, size_t *pos, const char *fmt, ...) {
    if (*pos >= out_size) return;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(out + *pos, out_size - *pos, fmt, ap);
    va_end(ap);
    if (n > 0) *pos += (size_t)n;
}

// ================= @PID =================

static void dump_pid_line(char *out, size_t out_size, size_t *pos,
                           const char *loop, const char *axis, const pid_gains_t *g) {
    append(out, out_size, pos, "PID %s %s %.4f %.4f %.4f %.4f %.4f\n",
           loop, axis, g->kp, g->ki, g->kd, g->integrator_limit, g->output_limit);
}

static bool handle_pid(const char *upper, char *out, size_t out_size) {
    if (strncmp(upper, "PID", 3) != 0) return false;
    size_t pos = 0;

    if (strcmp(upper, "PID GET") == 0) {
        // Đảo nhãn ROLL<->PITCH khi dump — KHỚP đúng đảo-ở-biên của CMD_SET_PID
        // (flight_core.c: NGOÀI "ROLL" ghi vào bộ gain angle_pitch/rate_pitch
        // nội bộ), nếu không "PID GET" sẽ đọc lại SAI bộ so với "PID SET" vừa
        // gửi (roll/pitch tréo ngoe giữa GET và SET).
        attitude_gains_t g;
        flight_core_get_attitude_gains(&g);
        dump_pid_line(out, out_size, &pos, "ANGLE", "ROLL", &g.angle_pitch);
        dump_pid_line(out, out_size, &pos, "ANGLE", "PITCH", &g.angle_roll);
        dump_pid_line(out, out_size, &pos, "ANGLE", "YAW", &g.angle_yaw);
        dump_pid_line(out, out_size, &pos, "RATE", "ROLL", &g.rate_pitch);
        dump_pid_line(out, out_size, &pos, "RATE", "PITCH", &g.rate_roll);
        dump_pid_line(out, out_size, &pos, "RATE", "YAW", &g.rate_yaw);
        append(out, out_size, &pos, "PID END\n");
        return true;
    }

    char loop_tok[16] = {0}, axis_tok[16] = {0};
    float kp = 0, ki = 0, kd = 0, ilim = 0, olim = 0;
    int n = sscanf(upper, "PID SET %15s %15s %f %f %f %f %f",
                   loop_tok, axis_tok, &kp, &ki, &kd, &ilim, &olim);
    if (n != 7) {
        append(out, out_size, &pos,
               "PID ERR usage: PID SET <ANGLE|RATE> <ROLL|PITCH|YAW> kp ki kd ilimit outlim\n");
        return true;
    }

    pid_loop_t loop;
    if (strcmp(loop_tok, "ANGLE") == 0) loop = PID_LOOP_ANGLE;
    else if (strcmp(loop_tok, "RATE") == 0) loop = PID_LOOP_RATE;
    else { append(out, out_size, &pos, "PID ERR unknown loop: %s\n", loop_tok); return true; }

    pid_axis_t axis;
    if (strcmp(axis_tok, "ROLL") == 0) axis = PID_AXIS_ROLL;
    else if (strcmp(axis_tok, "PITCH") == 0) axis = PID_AXIS_PITCH;
    else if (strcmp(axis_tok, "YAW") == 0) axis = PID_AXIS_YAW;
    else { append(out, out_size, &pos, "PID ERR unknown axis: %s\n", axis_tok); return true; }

    // Validate TRƯỚC khi queue — giá trị âm không hợp lệ cho gain PID (xem
    // yêu cầu "PID gains >= 0"). ERR + GIỮ NGUYÊN giá trị cũ (không push lệnh).
    if (kp < 0.0f || ki < 0.0f || kd < 0.0f || ilim < 0.0f || olim < 0.0f) {
        append(out, out_size, &pos, "PID ERR gains phai >= 0 (nhan: kp=%.4f ki=%.4f kd=%.4f ilim=%.4f olim=%.4f)\n",
               kp, ki, kd, ilim, olim);
        return true;
    }

    command_t cmd = {0};
    cmd.type = CMD_SET_PID;
    cmd.as.set_pid.loop = loop;
    cmd.as.set_pid.axis = axis;
    cmd.as.set_pid.kp = kp; cmd.as.set_pid.ki = ki; cmd.as.set_pid.kd = kd;
    cmd.as.set_pid.ilimit = ilim; cmd.as.set_pid.outlimit = olim;
    if (!flight_core_push_command(&cmd)) {
        append(out, out_size, &pos, "PID ERR command queue day, thu lai\n");
        return true;
    }

    append(out, out_size, &pos, "PID OK %s %s %.4f %.4f %.4f %.4f %.4f\n",
           loop_tok, axis_tok, kp, ki, kd, ilim, olim);
    return true;
}

// ================= @MAH =================

static bool handle_mah(const char *upper, char *out, size_t out_size) {
    if (strncmp(upper, "MAH", 3) != 0) return false;

    if (strcmp(upper, "MAH GET") == 0) {
        mahony_config_t mc;
        flight_core_get_mahony_config(&mc);
        snprintf(out, out_size, "MAH KP=%.4f KI=%.4f\n", mc.kp, mc.ki);
        return true;
    }

    float kp = 0, ki = 0;
    if (sscanf(upper, "MAH SET %f %f", &kp, &ki) != 2) {
        snprintf(out, out_size, "MAH ERR usage: MAH SET <kp> <ki>\n");
        return true;
    }
    if (kp < 0.0f || ki < 0.0f) {
        snprintf(out, out_size, "MAH ERR kp/ki phai >= 0\n");
        return true;
    }

    command_t cmd = {0};
    cmd.type = CMD_SET_MAHONY;
    cmd.as.set_mahony.kp = kp;
    cmd.as.set_mahony.ki = ki;
    if (!flight_core_push_command(&cmd)) {
        snprintf(out, out_size, "MAH ERR command queue day\n");
        return true;
    }

    // Clamp giống apply_command() (kp:[0,10] ki:[0,2], xem flight_core.c) để
    // reply đúng giá trị SẼ áp dụng.
    const float kp_c = clampf(kp, 0.0f, 10.0f);
    const float ki_c = clampf(ki, 0.0f, 2.0f);
    snprintf(out, out_size, "MAH OK KP=%.4f KI=%.4f\n", kp_c, ki_c);
    return true;
}

// ================= @SP (setpoint bay tay persistent) =================

static bool handle_sp(const char *upper, char *out, size_t out_size) {
    if (strncmp(upper, "SP", 2) != 0) return false;

    if (strcmp(upper, "SP GET") == 0) {
        float r, p, y;
        flight_core_get_setpoint(&r, &p, &y);
        snprintf(out, out_size, "SP R=%.2f P=%.2f Y=%.2f\n", r, p, y);
        return true;
    }

    float r = 0, p = 0, y = 0;
    if (sscanf(upper, "SP SET %f %f %f", &r, &p, &y) != 3) {
        snprintf(out, out_size, "SP ERR usage: SP SET <roll_deg> <pitch_deg> <yaw_rate_dps>\n");
        return true;
    }

    command_t cmd = {0};
    cmd.type = CMD_SET_ATTITUDE;
    cmd.as.set_attitude.roll_deg = r;
    cmd.as.set_attitude.pitch_deg = p;
    cmd.as.set_attitude.yaw_rate_dps = y;
    if (!flight_core_push_command(&cmd)) {
        snprintf(out, out_size, "SP ERR command queue day\n");
        return true;
    }

    const float r_c = clampf(r, -SP_TILT_MAX_DEG, SP_TILT_MAX_DEG);
    const float p_c = clampf(p, -SP_TILT_MAX_DEG, SP_TILT_MAX_DEG);
    const float y_c = clampf(y, -SP_YAW_RATE_MAX_DPS, SP_YAW_RATE_MAX_DPS);
    snprintf(out, out_size, "SP OK R=%.2f P=%.2f Y=%.2f\n", r_c, p_c, y_c);
    return true;
}

// ================= @TRIM =================

static bool handle_trim(const char *upper, char *out, size_t out_size) {
    if (strncmp(upper, "TRIM", 4) != 0) return false;

    if (strcmp(upper, "TRIM GET") == 0) {
        float r, p;
        flight_core_get_trim(&r, &p);
        snprintf(out, out_size, "TRIM roll=%.2f pitch=%.2f\n", r, p);
        return true;
    }

    float r = 0, p = 0;
    if (sscanf(upper, "TRIM SET %f %f", &r, &p) != 2) {
        snprintf(out, out_size, "TRIM ERR usage: TRIM GET | SET <roll_deg> <pitch_deg>\n");
        return true;
    }

    command_t cmd = {0};
    cmd.type = CMD_SET_TRIM;
    cmd.as.set_trim.roll_deg = r;
    cmd.as.set_trim.pitch_deg = p;
    if (!flight_core_push_command(&cmd)) {
        snprintf(out, out_size, "TRIM ERR command queue day\n");
        return true;
    }

    const float r_c = clampf(r, -TRIM_MAX_DEG, TRIM_MAX_DEG);
    const float p_c = clampf(p, -TRIM_MAX_DEG, TRIM_MAX_DEG);
    snprintf(out, out_size, "TRIM SET OK roll=%.2f pitch=%.2f\n", r_c, p_c);
    return true;
}

// ================= @ALT =================
// LƯU Ý @ALT MODE: UAV-S3 KHÔNG có "alt_mode" độc lập như UAV-Mini — alt_hold
// LUÔN chạy khi FSM đang HOLDING/FLYING (do state machine quyết định, không
// phải cờ riêng). MODE 3 (TAKEOFF)/4 (LANDING) ánh xạ sang đúng lệnh FSM
// tương ứng; MODE 0/1/2 chỉ trả OK (không có tác dụng thật — KHÔNG thể "tắt"
// alt_hold giữa chừng đang bay mà không hạ, xem an toàn FSM). Xem
// telemetry_format.c::alt_mode_from_state() cho chiều ngược lại (GET).
static bool handle_alt(const char *upper, char *out, size_t out_size) {
    if (strncmp(upper, "ALT", 3) != 0) return false;

    if (strcmp(upper, "ALT GET") == 0) {
        alt_hold_tune_t t;
        flight_core_get_alt_hold_tune(&t);
        telemetry_snapshot_t sn;
        flight_core_read_telemetry(&sn);
        snprintf(out, out_size, "ALT kp=%.3f vzkp=%.1f vzki=%.1f vzilim=%.1f tgt=%.2f mode=%d\n",
                 t.alt_kp, t.vz_kp, t.vz_ki, t.vz_ilimit, sn.alt_target_m,
                 telemetry_format_alt_mode(sn.state));
        return true;
    }

    {
        float kp = 0, vzkp = 0, vzki = 0, vzil = 0;
        if (sscanf(upper, "ALT SET %f %f %f %f", &kp, &vzkp, &vzki, &vzil) == 4) {
            command_t cmd = {0};
            cmd.type = CMD_SET_ALT_TUNE;
            cmd.as.set_alt_tune.alt_kp = kp;
            cmd.as.set_alt_tune.vz_kp = vzkp;
            cmd.as.set_alt_tune.vz_ki = vzki;
            cmd.as.set_alt_tune.vz_ilimit = vzil;
            if (!flight_core_push_command(&cmd)) {
                snprintf(out, out_size, "ALT ERR command queue day\n");
                return true;
            }
            snprintf(out, out_size, "ALT SET OK kp=%.3f vzkp=%.1f vzki=%.1f vzilim=%.1f\n",
                     kp, vzkp, vzki, vzil);
            return true;
        }
    }

    {
        float tgt = 0;
        if (sscanf(upper, "ALT TGT %f", &tgt) == 1) {
            command_t cmd = {0};
            cmd.type = CMD_SET_ALTITUDE;
            cmd.as.set_altitude.alt_mm = (int32_t)(tgt * 1000.0f);
            if (!flight_core_push_command(&cmd)) {
                snprintf(out, out_size, "ALT ERR command queue day\n");
                return true;
            }
            snprintf(out, out_size, "ALT TGT=%.2f m\n", tgt);
            return true;
        }
    }

    {
        int mode = 0;
        if (sscanf(upper, "ALT MODE %d", &mode) == 1) {
            command_t cmd = {0};
            if (mode == 3) {
                cmd.type = CMD_TAKEOFF;
                // Cùng lý do như 'ALT TAKEOFF'/'t': alt_mm giờ CÓ tác dụng nên
                // không được để 0. Lấy alt_target_m hiện tại làm đích.
                telemetry_snapshot_t sn;
                flight_core_read_telemetry(&sn);
                cmd.as.takeoff.alt_mm = (int32_t)(sn.alt_target_m * 1000.0f);
                if (!flight_core_push_command(&cmd)) {
                    snprintf(out, out_size, "ALT MODE=3 ERR command queue day, lenh BI BO\n");
                    return true;
                }
            } else if (mode == 4) {
                cmd.type = CMD_LAND;
                if (!flight_core_push_command(&cmd)) {
                    snprintf(out, out_size, "ALT MODE=4 ERR command queue day, lenh BI BO\n");
                    return true;
                }
            }
            // mode 0/1/2: khong co lenh tuong duong, chi ACK (xem comment dau ham).
            snprintf(out, out_size, "ALT MODE=%d (0=OFF 1=LOG 2=HOLD 3=TAKEOFF->CMD_TAKEOFF 4=LANDING->CMD_LAND)\n", mode);
            return true;
        }
    }

    // "ALT TAKEOFF [m]" — độ cao mục tiêu LÀ THAM SỐ CỦA LỆNH (mét, tuỳ chọn).
    //
    // ĐỔI QUAN TRỌNG: chuỗi cất cánh giờ leo tới target rồi settle (xem
    // takeoff_land.h), nên alt_mm KHÔNG còn bị bỏ qua. Trước đây mọi đường
    // console đẩy `command_t cmd = {0}` -> alt_mm=0, vô hại hồi đó nhưng giờ có
    // nghĩa là "cất cánh lên đúng sàn tối thiểu" — gần như chắc chắn không phải
    // ý người bấm.
    //
    // Bỏ trống -> dùng alt_target_m HIỆN TẠI (đặt trước bằng @ALT SET/TGT), và
    // nếu cái đó cũng 0 thì firmware áp sàn + log rõ. Ưu tiên số người dùng đã
    // nói ra thay vì tự chọn hộ một con số.
    if (strncmp(upper, "ALT TAKEOFF", 11) == 0) {
        telemetry_snapshot_t sn;
        flight_core_read_telemetry(&sn);
        const fsm_state_t predicted = fsm_on_takeoff_request(sn.state);
        const bool ok = (predicted != sn.state);

        float tgt_m = 0.0f;
        const bool has_arg = (sscanf(upper, "ALT TAKEOFF %f", &tgt_m) == 1);
        if (!has_arg || tgt_m <= 0.0f) {
            tgt_m = sn.alt_target_m;   // 0 -> firmware áp sàn, xem CMD_TAKEOFF
        }

        command_t cmd = {0};
        cmd.type = CMD_TAKEOFF;
        cmd.as.takeoff.alt_mm = (int32_t)(tgt_m * 1000.0f);
        // KIEM ket qua push. Truoc day bo qua -> queue day thi lenh BIEN MAT
        // hoan toan im lang trong khi GUI van duoc bao "sent". Moi handler khac
        // trong file nay deu kiem; rieng duong takeoff thi khong.
        if (!flight_core_push_command(&cmd)) {
            snprintf(out, out_size, "ALT TAKEOFF ERR command queue day, lenh BI BO\n");
            return true;
        }
        // KHÔNG nói "started". `ok` ở trên chỉ kiểm ĐƯỢC MỘT điều kiện (FSM
        // state); CMD_TAKEOFF trong flight_core.c còn từ chối vì thiếu nguồn
        // correction (baro TẮT + ToF hỏng) hoặc estimator không hợp lệ — và
        // những cái đó KHÔNG kiểm được từ đây mà không nhân đôi logic điều kiện
        // (hai bộ song song sẽ lệch nhau, đúng thứ takeoff_land.h cấm).
        //
        // Nên reply chỉ nói "đã gửi", và chỉ ra chỗ có câu trả lời THẬT:
        // TKOREJ= trong STATUS (GUI tự hiện, xem TAKEOFF_REJECT_NAMES).
        if (ok) {
            snprintf(out, out_size,
                     "ALT TAKEOFF sent tgt=%.2fm — ket qua THAT o TKOREJ= trong STATUS "
                     "(0=chay: PRIME -> CLIMB -> HOLD -> HOLDING)\n",
                     (double)tgt_m);
        } else {
            snprintf(out, out_size, "ALT TAKEOFF REJECTED: can state=ARMED (hien tai=%s)\n",
                     fsm_state_name(sn.state));
        }
        return true;
    }

    snprintf(out, out_size,
             "ALT ERR usage: ALT GET | SET <kp> <vzkp> <vzki> <vzilim> | TGT <m> | MODE <0-4> | TAKEOFF\n");
    return true;
}

// ================= @THR =================

// "THR STEP <delta>" — nấc throttle bench với delta TUỲ Ý.
//
// VÌ SAO CẦN: các phím ký tự đơn ('+'/'-'/']'/'[') hard-code delta ±20/±5. Muốn
// một bước khác (vd ±100 cho phím W/S ở GUI) thì phải gửi lặp nhiều lần — vừa
// tốn gói UDP vừa làm hành vi phụ thuộc số lần gói tới nơi.
//
// GIỚI HẠN GIỮ NGUYÊN: flight_core chỉ nhận CMD_BENCH_THROTTLE_STEP khi
// FSM_BENCH_RAMP. Lúc HOLDING/FLYING thì alt_hold (PID) sở hữu throttle — ghi
// đè bằng tay ở đó sẽ bị PID xoá sau đúng 1 tick (4ms). Reply nói RÕ điều này
// thay vì im lặng bỏ qua.
static bool handle_thr(const char *upper, char *out, size_t out_size) {
    if (strncmp(upper, "THR", 3) != 0) return false;

    int delta = 0;
    if (sscanf(upper, "THR STEP %d", &delta) == 1) {
        telemetry_snapshot_t sn;
        flight_core_read_telemetry(&sn);
        const bool ok = (sn.state == FSM_BENCH_RAMP);

        command_t cmd = {0};
        cmd.type = CMD_BENCH_THROTTLE_STEP;
        cmd.as.bench_step.delta_duty = (int32_t)delta;
        flight_core_push_command(&cmd);

        if (ok) {
            snprintf(out, out_size, "THR STEP %+d\n", delta);
        } else {
            snprintf(out, out_size,
                     "THR STEP BO QUA: can state=BENCH_RAMP (hien tai=%s). "
                     "Khi dang bay, throttle do alt_hold PID so huu — dung ALT TGT de len/xuong\n",
                     fsm_state_name(sn.state));
        }
        return true;
    }

    // "THR OFFSET <duty>" -- offset TAM THOI cho phim GIU (W/S o GUI).
    // Gia tri TUYET DOI, 0 = nha phim. GUI gui lai dinh ky trong luc con giu;
    // mat goi / ground-station chet -> firmware tu ve 0 sau BENCH_OFFSET_STALE_US.
    // KHONG reply: phim giu gui ~10Hz, reply moi lan se ngap log.
    int offset = 0;
    if (sscanf(upper, "THR OFFSET %d", &offset) == 1) {
        command_t cmd = {0};
        cmd.type = CMD_BENCH_THROTTLE_OFFSET;
        cmd.as.bench_offset.offset_duty = (int32_t)offset;
        flight_core_push_command(&cmd);
        out[0] = 0;   // reply RONG: im lang co chu dich (phim giu gui ~10Hz)
        return true;
    }

    snprintf(out, out_size, "THR ERR usage: THR STEP <delta> | THR OFFSET <duty>\n");
    return true;
}

// ================= @TKO =================

static bool handle_tko(const char *upper, char *out, size_t out_size) {
    if (strncmp(upper, "TKO", 3) != 0) return false;

    if (strcmp(upper, "TKO GET") == 0) {
        alt_hold_tune_t ht;
        takeoff_tune_t tt;
        flight_core_get_alt_hold_tune(&ht);
        flight_core_get_takeoff_tune(&tt);
        snprintf(out, out_size, "TKO hover=%.0f prime=%d ms=%d climb=%.2f\n",
                 ht.hover, tt.prime_duty, tt.prime_ms, (double)tt.max_climb_ms);
        return true;
    }

    // max_climb là THAM SỐ THỨ TƯ, TUỲ CHỌN: nó là thứ quyết định "leo nhanh
    // hay chậm" trong kiến trúc slew (xem takeoff_land.h), nhưng để tuỳ chọn
    // thì ground-station cũ gửi 3 tham số vẫn chạy — firmware giữ nguyên giá
    // trị đang dùng khi không nhận được field này (xem CMD_SET_TKO_TUNE).
    float hov = 0, climb = 0;
    int prime = 0, ms = 0;
    const int n = sscanf(upper, "TKO SET %f %d %d %f", &hov, &prime, &ms, &climb);
    if (n >= 3) {
        command_t cmd = {0};
        cmd.type = CMD_SET_TKO_TUNE;
        cmd.as.set_tko_tune.hover = hov;
        cmd.as.set_tko_tune.prime_duty = prime;
        cmd.as.set_tko_tune.prime_ms = ms;
        cmd.as.set_tko_tune.max_climb_ms = (n >= 4) ? climb : 0.0f;   // 0 = giữ nguyên
        if (!flight_core_push_command(&cmd)) {
            snprintf(out, out_size, "TKO ERR command queue day\n");
            return true;
        }
        if (n >= 4) {
            snprintf(out, out_size, "TKO SET OK hover=%.0f prime=%d ms=%d climb=%.2f\n",
                     hov, prime, ms, (double)climb);
        } else {
            snprintf(out, out_size, "TKO SET OK hover=%.0f prime=%d ms=%d (climb giu nguyen)\n",
                     hov, prime, ms);
        }
        return true;
    }

    snprintf(out, out_size,
             "TKO ERR usage: TKO GET | SET <hover> <prime_duty> <prime_ms> [max_climb_ms]\n");
    return true;
}

// ================= @LAND =================

static bool handle_land(const char *upper, char *out, size_t out_size) {
    if (strncmp(upper, "LAND", 4) != 0) return false;

    if (strcmp(upper, "LAND GET") == 0) {
        landing_tune_t lt;
        flight_core_get_landing_tune(&lt);
        snprintf(out, out_size, "LAND dvz=%.2f flarealt=%.2f fvz=%.2f tdalt=%.2f\n",
                 lt.descent_vz, lt.flare_alt_m, lt.flare_vz, lt.touchdown_alt_m);
        return true;
    }

    {
        float dvz = 0, fa = 0, fvz = 0, td = 0;
        if (sscanf(upper, "LAND SET %f %f %f %f", &dvz, &fa, &fvz, &td) == 4) {
            command_t cmd = {0};
            cmd.type = CMD_SET_LAND_TUNE;
            cmd.as.set_land_tune.descent_vz = dvz;
            cmd.as.set_land_tune.flare_alt_m = fa;
            cmd.as.set_land_tune.flare_vz = fvz;
            cmd.as.set_land_tune.touchdown_alt_m = td;
            if (!flight_core_push_command(&cmd)) {
                snprintf(out, out_size, "LAND ERR command queue day\n");
                return true;
            }
            snprintf(out, out_size, "LAND SET OK dvz=%.2f flarealt=%.2f fvz=%.2f tdalt=%.2f\n",
                     dvz, fa, fvz, td);
            return true;
        }
    }

    if (strcmp(upper, "LAND") == 0) {   // bare -> kích landing
        telemetry_snapshot_t sn;
        flight_core_read_telemetry(&sn);
        const fsm_state_t predicted = fsm_on_land_request(sn.state);
        const bool ok = (predicted != sn.state);
        command_t cmd = {0};
        cmd.type = CMD_LAND;
        flight_core_push_command(&cmd);
        snprintf(out, out_size, ok ? "LAND started (ha tu dong)\n"
                                   : "LAND REJECTED: can state=HOLDING|FLYING (hien tai=%s)\n",
                 fsm_state_name(sn.state));
        return true;
    }

    snprintf(out, out_size,
             "LAND ERR usage: LAND (kich) | GET | SET <dvz> <flarealt> <fvz> <tdalt>\n");
    return true;
}

// ================= @HOVER / @MOVE / @YAW (timed-command, PORT y hệt console
// USB "hover"/"move"/"yaw" — xem command.h CMD_HOVER/CMD_MOVE/CMD_SET_YAW,
// TRƯỚC ĐÂY chỉ gọi được qua USB/MicroPython, giờ thêm đường UDP) =================

static bool handle_hover(const char *upper, char *out, size_t out_size) {
    if (strncmp(upper, "HOVER", 5) != 0) return false;
    // sec CHỈ để tương thích chữ ký CMD_HOVER — flight_core.c hiện KHÔNG dùng
    // tới (hủy timed-command đang chạy rồi về HOLDING ngay lập tức, xem
    // apply_command() case CMD_HOVER), nhận tham số cho khớp cú pháp console.
    float sec = 0.0f;
    sscanf(upper, "HOVER %f", &sec);
    command_t cmd = {0};
    cmd.type = CMD_HOVER;
    cmd.as.hover.sec = sec;
    if (!flight_core_push_command(&cmd)) {
        snprintf(out, out_size, "HOVER ERR command queue day\n");
        return true;
    }
    snprintf(out, out_size, "HOVER OK -- huy timed-command dang chay, ve HOLDING\n");
    return true;
}

// parse_move_dir_upper() — PORT parse_move_dir() (src/main.c) sang chữ HOA
// (command_parser.c luôn upper-case cả dòng TRƯỚC khi match, xem handle_line()).
static bool parse_move_dir_upper(const char *s, move_dir_t *out) {
    if (strcmp(s, "FORWARD") == 0) { *out = MOVE_FORWARD; return true; }
    if (strcmp(s, "BACK") == 0)    { *out = MOVE_BACK;    return true; }
    if (strcmp(s, "LEFT") == 0)    { *out = MOVE_LEFT;    return true; }
    if (strcmp(s, "RIGHT") == 0)   { *out = MOVE_RIGHT;   return true; }
    if (strcmp(s, "UP") == 0)      { *out = MOVE_UP;      return true; }
    if (strcmp(s, "DOWN") == 0)    { *out = MOVE_DOWN;    return true; }
    if (strcmp(s, "CW") == 0)      { *out = MOVE_CW;      return true; }
    if (strcmp(s, "CCW") == 0)     { *out = MOVE_CCW;     return true; }
    return false;
}

static bool handle_move(const char *upper, char *out, size_t out_size) {
    if (strncmp(upper, "MOVE", 4) != 0) return false;

    char dir_str[16] = {0};
    int pct = 0;
    float sec = 0.0f;
    if (sscanf(upper, "MOVE %15s %d %f", dir_str, &pct, &sec) == 3) {
        move_dir_t dir;
        if (!parse_move_dir_upper(dir_str, &dir)) {
            snprintf(out, out_size,
                     "MOVE ERR dir sai: %s (dung forward/back/left/right/up/down/cw/ccw)\n", dir_str);
            return true;
        }
        command_t cmd = {0};
        cmd.type = CMD_MOVE;
        cmd.as.move.dir = dir;
        cmd.as.move.pct = pct;
        cmd.as.move.sec = sec;
        if (!flight_core_push_command(&cmd)) {
            snprintf(out, out_size, "MOVE ERR command queue day\n");
            return true;
        }
        snprintf(out, out_size, "MOVE OK %s %d%% %.1fs\n", dir_str, pct, (double)sec);
        return true;
    }

    snprintf(out, out_size,
             "MOVE ERR usage: MOVE <forward|back|left|right|up|down|cw|ccw> <pct 0-100> <sec>\n");
    return true;
}

static bool handle_yaw(const char *upper, char *out, size_t out_size) {
    if (strncmp(upper, "YAW", 3) != 0) return false;

    float deg = 0.0f;
    if (sscanf(upper, "YAW %f", &deg) == 1) {
        command_t cmd = {0};
        cmd.type = CMD_SET_YAW;
        cmd.as.set_yaw.yaw_deg = deg;
        if (!flight_core_push_command(&cmd)) {
            snprintf(out, out_size, "YAW ERR command queue day\n");
            return true;
        }
        snprintf(out, out_size, "YAW OK %.1fdeg (tuong doi, open-loop theo thoi gian)\n", (double)deg);
        return true;
    }

    snprintf(out, out_size, "YAW ERR usage: YAW <deg> (am = trai, duong = phai)\n");
    return true;
}

// ================= @TEST (test 1 động cơ riêng lẻ — xem command.h
// CMD_TEST_MOTOR, PORT y hệt lệnh console USB "test_motor") =================
// KHÔNG có GET (không có gì để đọc lại — đây là 1 xung, không phải tham số
// tune). CHỈ chạy khi DISARMED (chặn cứng trong flight_core.c::apply_command(),
// KHÔNG phụ thuộc client kiểm tra trước) + tự dừng sau ~500ms
// (TEST_MOTOR_PULSE_MS) + duty client gửi CÒN BỊ CLAMP THÊM xuống tối đa
// TEST_MOTOR_MAX_DUTY_PCT=25% ở flight_core.c dù client gửi cao hơn — reply
// dưới đây echo NGUYÊN VĂN giá trị client gửi (giống @TKO/@LAND SET), KHÔNG
// biết trước firmware có clamp thêm hay không, xem log firmware ('status'
// console/`mag_test`-style ESP_LOGW) để biết duty THẬT SỰ đã áp dụng.
static bool handle_test(const char *upper, char *out, size_t out_size) {
    if (strncmp(upper, "TEST", 4) != 0) return false;

    int idx = 0, pct = 0;
    if (sscanf(upper, "TEST %d %d", &idx, &pct) == 2) {
        if (idx < 1 || idx > 4) {
            snprintf(out, out_size, "TEST ERR motor phai 1-4 (nhan %d)\n", idx);
            return true;
        }
        command_t cmd = {0};
        cmd.type = CMD_TEST_MOTOR;
        cmd.as.test_motor.motor_idx = idx;
        cmd.as.test_motor.duty_pct = pct;
        if (!flight_core_push_command(&cmd)) {
            snprintf(out, out_size, "TEST ERR command queue day\n");
            return true;
        }
        snprintf(out, out_size,
                 "TEST OK M%d @ %d%% da gui -- CHI chay neu dang DISARMED va da THAO CANH QUAT, "
                 "tu dong dung sau ~500ms, xem log firmware de xac nhan\n", idx, pct);
        return true;
    }

    snprintf(out, out_size, "TEST ERR usage: TEST <motor 1-4> <duty_pct 0-100>\n");
    return true;
}

// ================= @CMDR (Commander — geofence + ngưỡng fault, xem
// commander.h + CMD_SET_COMMANDER_CFG) =================
// KHÁC @TKO/@LAND/@ALT: chạy được BẤT KỲ LÚC NÀO (kể cả đang bay) — geofence/
// failsafe PHẢI chỉnh live, không chờ DISARMED.
static bool handle_cmdr(const char *upper, char *out, size_t out_size) {
    if (strncmp(upper, "CMDR", 4) != 0) return false;

    if (strcmp(upper, "CMDR GET") == 0) {
        commander_config_t cc;
        flight_core_get_commander_cfg(&cc);
        snprintf(out, out_size,
                 "CMDR altmin=%.2f altmax=%.2f battfloor=%.2f hbms=%d tilt=%.1f motorsatms=%d\n",
                 cc.alt_min_m, cc.alt_max_m, cc.battery_floor_v,
                 cc.heartbeat_timeout_ms, cc.hard_tilt_deg, cc.motor_sat_hard_ms);
        return true;
    }

    float altmin = 0, altmax = 0, battfloor = 0, tilt = 0;
    int hbms = 0, motorsatms = 0;
    if (sscanf(upper, "CMDR SET %f %f %f %d %f %d",
               &altmin, &altmax, &battfloor, &hbms, &tilt, &motorsatms) == 6) {
        command_t cmd = {0};
        cmd.type = CMD_SET_COMMANDER_CFG;
        cmd.as.set_commander_cfg.alt_min_m = altmin;
        cmd.as.set_commander_cfg.alt_max_m = altmax;
        cmd.as.set_commander_cfg.battery_floor_v = battfloor;
        cmd.as.set_commander_cfg.heartbeat_timeout_ms = hbms;
        cmd.as.set_commander_cfg.hard_tilt_deg = tilt;
        cmd.as.set_commander_cfg.motor_sat_hard_ms = motorsatms;
        if (!flight_core_push_command(&cmd)) {
            snprintf(out, out_size, "CMDR ERR command queue day\n");
            return true;
        }
        // Echo lại giá trị GỬI (KHÔNG đọc lại flight_core_get_commander_cfg()
        // ngay — lệnh vừa push còn nằm trong queue, chưa chắc apply_command()
        // đã xử lý xong tick này). Giống pattern reply của @TKO/@LAND SET.
        snprintf(out, out_size,
                 "CMDR SET OK altmin=%.2f altmax=%.2f battfloor=%.2f hbms=%d tilt=%.1f motorsatms=%d\n",
                 altmin, altmax, battfloor, hbms, tilt, motorsatms);
        return true;
    }

    snprintf(out, out_size,
             "CMDR ERR usage: CMDR GET | SET <altmin_m> <altmax_m> <battfloor_v> "
             "<hbtimeout_ms> <hardtilt_deg> <motorsat_ms>\n");
    return true;
}

// ================= @CAL (calibration qua UDP — xem calibration.h +
// flight_core.c CMD_CALIB_*, ported 1-1 từ bộ lệnh console calib_*) =================
// KHÁC MỌI handler khác ở trên: bộ lệnh này KHÔNG có "SET giá trị" — chỉ
// kích các bước calib (đã CHỈ chạy khi FSM DISARMED, tự chặn trong
// flight_core.c, xem đó) và đọc tiến trình qua CAL STATUS.
static bool handle_cal(const char *upper, char *out, size_t out_size) {
    if (strncmp(upper, "CAL", 3) != 0) return false;

    if (strcmp(upper, "CAL STATUS") == 0) {
        telemetry_snapshot_t sn;
        flight_core_read_telemetry(&sn);
        // valid_* = ĐÃ CÓ giá trị calib hợp lệ (RAM/NVS) — KHÔNG phải "phiên
        // gần nhất thành công": phiên thất bại KHÔNG xóa calib cũ, nên
        // valid_mag=1 vẫn xuất hiện ngay sau 1 lần CAL MAG hỏng. mag_ok = driver
        // QMC5883P init được lúc boot (0 -> CAL MAG START sẽ bị từ chối ngay).
        int n = snprintf(out, out_size,
                 "CAL uncalibrated=%d valid_gyro=%d valid_accel=%d valid_mag=%d mag_ok=%d "
                 "gyro_active=%d accel_capturing=%d accel_faces=%d/%d "
                 "mag_active=%d mag_samples=%u baro_cal=%d baro_healthy=%d "
                 "gyro_std=(%.3f,%.3f,%.3f) accel_residual=%.3f imu_temp=%.1f\n",
                 (int)sn.uncalibrated, (int)sn.calib_gyro_valid, (int)sn.calib_accel_valid,
                 (int)sn.calib_mag_valid, (int)sn.mag_ok_driver,
                 (int)sn.calib_gyro_active, (int)sn.calib_accel_capturing,
                 sn.calib_accel_faces_done, CALIB_ACCEL_FACES_NEEDED,
                 (int)sn.calib_mag_active, (unsigned)sn.calib_mag_sample_count,
                 (int)sn.baro_calibrated, (int)sn.baro_healthy,
                 sn.gyro_calib_std_x_dps, sn.gyro_calib_std_y_dps, sn.gyro_calib_std_z_dps,
                 sn.accel_calib_residual_g, sn.imu_temp_c);
        // Dòng 2 CHỈ khi đang chạy phiên mag — range 3 trục là thứ cần nhìn
        // trong lúc xoay (nên GẦN BẰNG NHAU + tăng đều), loi_i2c/chua_sansang
        // cho biết mau=0 là do người xoay ít hay do chip không phát mẫu.
        if (sn.calib_mag_active && n > 0 && (size_t)n < out_size) {
            snprintf(out + n, out_size - (size_t)n,
                     "CAL MAG left=%ds samples=%u range=(%.0f,%.0f,%.0f) i2c_err=%u notready=%u\n",
                     sn.calib_mag_seconds_left, (unsigned)sn.calib_mag_sample_count,
                     sn.calib_mag_range_x, sn.calib_mag_range_y, sn.calib_mag_range_z,
                     (unsigned)sn.calib_mag_read_err_count, (unsigned)sn.calib_mag_notready_count);
        }
        return true;
    }

    // Bảng lệnh 1-1 với command.h CMD_CALIB_* — KHÔNG tham số, chỉ kích.
    static const struct { const char *text; command_type_t type; const char *reply; } CAL_TABLE[] = {
        { "CAL GYRO START", CMD_CALIB_GYRO,        "CAL GYRO START OK -- DUNG YEN drone, xem CAL STATUS\n" },
        { "CAL GYRO ABORT", CMD_CALIB_GYRO_ABORT,  "CAL GYRO ABORT OK\n" },
        { "CAL ACC START",  CMD_CALIB_ACCEL_FACE,  "CAL ACC START OK -- bat 1 mat, GIU YEN ~0.5s, xem CAL STATUS\n" },
        { "CAL ACC NEXT",   CMD_CALIB_ACCEL_FACE,  "CAL ACC NEXT OK -- bat 1 mat, GIU YEN ~0.5s, xem CAL STATUS\n" },
        { "CAL ACC ABORT",  CMD_CALIB_ACCEL_RESET, "CAL ACC ABORT OK -- huy tien trinh 6-face, lam lai tu dau\n" },
        { "CAL MAG START",  CMD_CALIB_MAG_START,   "CAL MAG START OK -- TU DONG chay 60s, XOAY hinh so 8 SUOT thoi gian nay "
                                                    "(CAL MAG STOP van dung duoc de ket thuc SOM)\n" },
        { "CAL MAG STOP",   CMD_CALIB_MAG_STOP,    "CAL MAG STOP OK -- ket thuc SOM, xem CAL STATUS de biet ket qua\n" },
        { "CAL MAG ABORT",  CMD_CALIB_MAG_ABORT,   "CAL MAG ABORT OK\n" },
        { "CAL BARO START", CMD_CALIB_BARO_GROUND, "CAL BARO START OK -- DUNG YEN ~1s, xem CAL STATUS\n" },
        { "CAL RESET",      CMD_CALIB_ERASE,       "CAL RESET OK -- da xoa TOAN BO calib, can calib lai truoc khi arm\n" },
    };
    for (size_t i = 0; i < sizeof(CAL_TABLE) / sizeof(CAL_TABLE[0]); i++) {
        if (strcmp(upper, CAL_TABLE[i].text) == 0) {
            command_t cmd = {0};
            cmd.type = CAL_TABLE[i].type;
            if (!flight_core_push_command(&cmd)) {
                snprintf(out, out_size, "CAL ERR command queue day\n");
                return true;
            }
            snprintf(out, out_size, "%s", CAL_TABLE[i].reply);
            return true;
        }
    }

    if (strcmp(upper, "CAL SAVE") == 0 || strcmp(upper, "CAL LOAD") == 0) {
        // Kiến trúc này KHÔNG có bước "commit" riêng — mỗi giai đoạn calib
        // (gyro/accel 6-face/mag) TỰ ĐỘNG lưu NVS ngay khi hoàn tất
        // (calibration_save_*(), xem calibration.c), và flight_core_start()
        // TỰ ĐỘNG nạp lại lúc boot — không có gì để SAVE/LOAD thủ công thêm.
        // Trả OK kèm giải thích thay vì ERR, tránh GUI tưởng lệnh không tồn tại.
        snprintf(out, out_size,
                 "CAL %s OK (khong can lam gi them -- calib TU DONG luu NVS ngay khi xong tung "
                 "giai doan, va TU DONG nap lai luc boot, xem README muc Calibration)\n",
                 (upper[4] == 'S') ? "SAVE" : "LOAD");
        return true;
    }

    snprintf(out, out_size,
             "CAL ERR usage: CAL STATUS | GYRO START|ABORT | ACC START|NEXT|ABORT | "
             "MAG START|STOP|ABORT | BARO START | SAVE | LOAD | RESET\n");
    return true;
}

// ================= dispatch dòng '@...' =================

static void handle_line(const char *line_in, char *out, size_t out_size) {
    out[0] = '\0';

    char upper[LINE_BUF_SIZE];
    strncpy(upper, line_in, sizeof(upper) - 1);
    upper[sizeof(upper) - 1] = '\0';
    to_upper_inplace(upper);

    if (handle_pid(upper, out, out_size)) return;
    if (handle_mah(upper, out, out_size)) return;
    if (handle_sp(upper, out, out_size)) return;
    if (handle_trim(upper, out, out_size)) return;
    if (handle_alt(upper, out, out_size)) return;
    if (handle_thr(upper, out, out_size)) return;
    if (handle_tko(upper, out, out_size)) return;
    if (handle_land(upper, out, out_size)) return;
    if (handle_hover(upper, out, out_size)) return;
    if (handle_move(upper, out, out_size)) return;
    if (handle_yaw(upper, out, out_size)) return;
    if (handle_test(upper, out, out_size)) return;
    if (handle_cmdr(upper, out, out_size)) return;
    if (handle_cal(upper, out, out_size)) return;

    snprintf(out, out_size, "ERR unknown line cmd: %s\n", line_in);
}

// ================= lệnh 1 ký tự tức thời =================
// PORT tinh thần console_task switch (UAV-Mini flight_control.cpp) — bộ lệnh
// KHÁC (không có +/-/[]/0/z/x throttle-tay/alt-mode-toggle: kiến trúc UAV-S3
// luôn đi qua alt_hold/attitude controller, không có "throttle tay trực
// tiếp" — xem yêu cầu "PC không tính PID/gửi duty").
static void handle_single_char(int c, char *out, size_t out_size) {
    out[0] = '\0';

    switch (c) {
        case 'r': case 'R': {
            // KHÔNG được reply "ARMED" ở đây. Lệnh chỉ vừa được ĐẨY VÀO QUEUE —
            // quyết định thật xảy ra sau đó trong stabilize_task
            // (apply_command/prearm_check) và có THÊM ~12 lý do từ chối khác
            // (pin, baro, IMU stale, chưa calib, FSM không ở DISARMED...).
            //
            // Bản cũ reply "ARMED" chỉ dựa trên attitude guard — một PHỎNG ĐOÁN.
            // Hệ quả thực tế: người dùng bấm ARM, thấy "ARMED", nhưng drone
            // KHÔNG arm và không có cách nào biết vì sao (log lý do chỉ ra
            // console USB, không tới GUI qua UDP). Giờ nói đúng những gì đã xảy
            // ra và chỉ chỗ xem kết quả THẬT.
            telemetry_snapshot_t sn;
            flight_core_read_telemetry(&sn);
            const bool guard = fsm_arm_guard_ok(sn.attitude_valid, sn.roll_deg, sn.pitch_deg);
            command_t cmd = {0};
            cmd.type = CMD_ARM;
            flight_core_push_command(&cmd);
            if (!guard) {
                snprintf(out, out_size,
                         "ARM REJECTED (attitude/tilt): valid=%d R=%.1f P=%.1f — dat drone bang phang\n",
                         sn.attitude_valid ? 1 : 0, (double)sn.roll_deg, (double)sn.pitch_deg);
            } else if (sn.state != FSM_DISARMED) {
                snprintf(out, out_size,
                         "ARM REJECTED: dang o %s, chi ARM duoc tu DISARMED. Bam 'k' (KILL) roi ARM lai\n",
                         fsm_state_name(sn.state));
            } else {
                snprintf(out, out_size,
                         "ARM da gui — xem ARM=/ARMREJ= trong STATUS de biet ket qua THAT\n");
            }
            break;
        }
        case 'k': case 'K': {
            // KILL — KHÁC disarm thường, bypass moi FSM guard (xem CMD_KILL
            // trong command.h). Thắng mọi state, không chờ landing/PID.
            command_t cmd = {0};
            cmd.type = CMD_KILL;
            flight_core_push_command(&cmd);
            snprintf(out, out_size, "KILLED (motor cat ngay, DISARMED cuong buc)\n");
            break;
        }
        case 'd': case 'D': {
            // DISARM thường — KHÁC KILL: CHỈ hợp lệ từ FSM_ARMED/FSM_BENCH_RAMP
            // (xem fsm_on_disarm_request()), no-op nếu không đúng state (không
            // có gì để "hủy" nếu chưa ARM hoặc đang bay dở — bay dở PHẢI qua
            // LAND/KILL, không có đường "disarm êm" giữa không trung).
            telemetry_snapshot_t sn;
            flight_core_read_telemetry(&sn);
            const bool ok = (sn.state == FSM_ARMED || sn.state == FSM_BENCH_RAMP);
            command_t cmd = {0};
            cmd.type = CMD_DISARM;
            flight_core_push_command(&cmd);
            snprintf(out, out_size, ok ? "DISARMED\n"
                                       : "DISARM REJECTED: can state=ARMED|BENCH_RAMP (hien tai=%s, "
                                         "dang bay thi dung KILL hoac LAND)\n",
                     fsm_state_name(sn.state));
            break;
        }
        case 'p': case 'P': {
            // "Ping" Commander heartbeat watchdog (xem commander.h
            // heartbeat_timeout_ms, CMD_HEARTBEAT trong command.h). QUAN
            // TRỌNG: đây là con đường DUY NHẤT qua UDP ground-station để
            // commander_heartbeat() được gọi — thiếu nó, s_cmd_state.
            // last_heartbeat_us đứng yên từ lúc boot, và Commander sẽ SOFT
            // FAULT (-> LANDING) NGAY tick đầu tiên sau khi vào HOLDING/FLYING
            // (heartbeat_age vượt xa heartbeat_timeout_ms mặc định 1000ms).
            // tools/uav_udp_console.py tự gửi lệnh này định kỳ khi đã kết nối
            // (xem PidTunerApp._start_commander_heartbeat()) — KHÔNG chỉ dựa
            // vào người dùng bấm tay.
            command_t cmd = {0};
            cmd.type = CMD_HEARTBEAT;
            flight_core_push_command(&cmd);
            snprintf(out, out_size, "PONG\n");
            break;
        }
        case 't': case 'T': {
            telemetry_snapshot_t sn;
            flight_core_read_telemetry(&sn);
            const fsm_state_t predicted = fsm_on_takeoff_request(sn.state);
            const bool ok = (predicted != sn.state);
            command_t cmd = {0};
            cmd.type = CMD_TAKEOFF;
            // Phím 't' không có tham số -> lấy alt_target_m HIỆN TẠI làm đích
            // (đặt trước bằng @ALT SET/TGT hoặc '>'/'<'). Trước đây để alt_mm=0
            // vì chuỗi cũ bỏ qua nó; giờ 0 nghĩa là chỉ lên sàn tối thiểu.
            cmd.as.takeoff.alt_mm = (int32_t)(sn.alt_target_m * 1000.0f);
            if (!flight_core_push_command(&cmd)) {
                snprintf(out, out_size, "TAKEOFF ERR command queue day, lenh BI BO\n");
                break;
            }
            // KHÔNG dùng một snprintf với format chọn bằng ternary: hai format
            // có danh sách tham số KHÁC NHAU (%.2f vs %s), varargs cố định sẽ
            // đọc sai kiểu ở một trong hai nhánh.
            if (ok) {
                // "sent", KHÔNG phải "started" — xem lý do đầy đủ ở nhánh
                // "ALT TAKEOFF" phía trên (điều kiện thật nằm trong flight_core).
                snprintf(out, out_size, "TAKEOFF sent tgt=%.2fm (ket qua o TKOREJ= trong STATUS)\n",
                         (double)sn.alt_target_m);
            } else {
                snprintf(out, out_size, "TAKEOFF REJECTED: can state=ARMED (hien tai=%s)\n",
                         fsm_state_name(sn.state));
            }
            break;
        }
        case 'l': case 'L': {
            telemetry_snapshot_t sn;
            flight_core_read_telemetry(&sn);
            const fsm_state_t predicted = fsm_on_land_request(sn.state);
            const bool ok = (predicted != sn.state);
            command_t cmd = {0};
            cmd.type = CMD_LAND;
            flight_core_push_command(&cmd);
            snprintf(out, out_size, ok ? "LANDING started\n"
                                       : "LAND REJECTED: can state=HOLDING|FLYING (hien tai=%s)\n",
                     fsm_state_name(sn.state));
            break;
        }
        case '>': case '<': {
            // Alt target step +-0.1m (W/S tren GUI) — doc target hien tai roi
            // cong/tru, KHONG dieu chinh throttle truc tiep (xem yeu cau muc 5).
            telemetry_snapshot_t sn;
            flight_core_read_telemetry(&sn);
            const float step = (c == '>') ? 0.10f : -0.10f;
            const float new_tgt = sn.alt_target_m + step;
            command_t cmd = {0};
            cmd.type = CMD_SET_ALTITUDE;
            cmd.as.set_altitude.alt_mm = (int32_t)(new_tgt * 1000.0f);
            flight_core_push_command(&cmd);
            snprintf(out, out_size, "ALT TGT step -> %.2f m\n", new_tgt);
            break;
        }
        case 'b': case 'B': {
            // Vào bench-test tăng ga tay để tune PID — xem GHI CHÚ 3
            // flight_state_machine.h + CMD_BENCH_RAMP_START (command.h). CHỈ
            // hợp lệ từ ARMED. Drone PHẢI được giữ chặt/kẹp trên giá đỡ — đây
            // KHÔNG phải chế độ bay.
            telemetry_snapshot_t sn;
            flight_core_read_telemetry(&sn);
            const bool ok = (sn.state == FSM_ARMED);
            command_t cmd = {0};
            cmd.type = CMD_BENCH_RAMP_START;
            flight_core_push_command(&cmd);
            snprintf(out, out_size, ok
                     ? "BENCH_RAMP started -- throttle=0. XAC NHAN drone da GIU CHAT/KEP TREN GIA DO. "
                       "Dung +/-/]/[ de chinh ga, 0 de ve 0, n de dung.\n"
                     : "BENCH_RAMP REJECTED: can state=ARMED (hien tai=%s)\n",
                     fsm_state_name(sn.state));
            break;
        }
        case 'n': case 'N': {
            // Dừng bench-test NGAY (cắt máy, không ramp xuống), về ARMED —
            // KHÔNG latch (khác 'd'/'k'), bấm 'b' lại được luôn để tune tiếp.
            command_t cmd = {0};
            cmd.type = CMD_BENCH_RAMP_STOP;
            flight_core_push_command(&cmd);
            snprintf(out, out_size, "BENCH_RAMP stopped -- motor cat ngay, ve ARMED\n");
            break;
        }
        case '+': case '-': case ']': case '[': case '0': {
            // Nấc throttle bench-test (CHỈ hợp lệ khi đang BENCH_RAMP, xem 'b'
            // ở trên) — +/-=20 (thô), ]/[=5 (tinh), 0=về hẳn 0 (delta âm rất
            // lớn, clamp sàn 0 trong flight_core.c — KHÔNG thoát BENCH_RAMP,
            // khác 'n').
            telemetry_snapshot_t sn;
            flight_core_read_telemetry(&sn);
            const bool ok = (sn.state == FSM_BENCH_RAMP);
            int32_t delta = 0;
            switch (c) {
                case '+': delta = 20; break;
                case '-': delta = -20; break;
                case ']': delta = 5; break;
                case '[': delta = -5; break;
                case '0': delta = -1000000; break;
            }
            command_t cmd = {0};
            cmd.type = CMD_BENCH_THROTTLE_STEP;
            cmd.as.bench_step.delta_duty = delta;
            flight_core_push_command(&cmd);
            if (ok) {
                snprintf(out, out_size, "BENCH throttle step %+d\n", (int)delta);
            } else {
                snprintf(out, out_size, "BENCH throttle step IGNORED: can state=BENCH_RAMP (hien tai=%s, bam 'b' truoc)\n",
                         fsm_state_name(sn.state));
            }
            break;
        }
        case 's': case 'S':
            telemetry_format_status_line(out, out_size);
            break;
        case 'f': case 'F': {
            // VÀO "flight mode" — bật STATUS streaming định kỳ (xem net_task()
            // trong main.c). Idempotent (bấm lại khi đã bật không đổi gì, chỉ
            // xác nhận lại) — thoát bằng 'q' (case riêng bên dưới), KHÔNG phải
            // bấm lại 'f' lần 2. KHÔNG ảnh hưởng lệnh khác — @CAL/@PID/../
            // single-char khác vẫn luôn hoạt động + trả reply trực tiếp bất kể
            // cờ này.
            s_telemetry_enabled = true;
            snprintf(out, out_size, "FLIGHT MODE ON -- STATUS streaming bat dau (~20Hz), bam 'q' de thoat\n");
            break;
        }
        case 'q': case 'Q': {
            // THOÁT "flight mode" — tắt STATUS streaming (xem case 'f' ở
            // trên). Idempotent (bấm khi đã tắt không lỗi, chỉ xác nhận lại).
            s_telemetry_enabled = false;
            snprintf(out, out_size, "FLIGHT MODE OFF -- STATUS streaming dung (lenh khac van hoat dong binh thuong)\n");
            break;
        }
        case 'h': case 'H': case '?':
            snprintf(out, out_size,
                     "r=ARM k=KILL d=DISARM(chi tu ARMED) t=TAKEOFF l=LAND >/<=alt+-0.1m "
                     "p=heartbeat(PONG) f=vao flight mode(bat STATUS) q=thoat flight mode(tat STATUS) "
                     "s=status\n"
                     "b=BENCH_RAMP start(chi tu ARMED) n=BENCH_RAMP stop(khong latch) "
                     "+/-=throttle bench +-20 ]/[=+-5 0=throttle bench ve 0 (CHI hop le khi dang BENCH_RAMP)\n"
                     "@PID GET|SET <ANGLE|RATE> <ROLL|PITCH|YAW> kp ki kd ilim outlim\n"
                     "@MAH GET|SET <kp> <ki>\n"
                     "@SP GET|SET <roll_deg> <pitch_deg> <yaw_rate_dps>\n"
                     "@TRIM GET|SET <roll_deg> <pitch_deg>\n"
                     "@ALT GET|SET <kp> <vzkp> <vzki> <vzilim>|TGT <m>|MODE <0-4>|TAKEOFF\n"
                     "@TKO GET|SET <hover> <prime_duty> <prime_ms> [max_climb_ms]\n"
                     "@LAND (kich)|GET|SET <dvz> <flarealt> <fvz> <tdalt>\n"
                     "@HOVER <sec> | @MOVE <dir> <pct> <sec> | @YAW <deg>\n"
                     "@TEST <motor 1-4> <duty_pct 0-100> -- CHI khi DISARMED, THAO CANH QUAT truoc\n"
                     "@CMDR GET|SET <altmin> <altmax> <battfloor> <hbms> <tilt> <motorsatms>\n"
                     "@CAL STATUS|GYRO START|ABORT|ACC START|NEXT|ABORT|MAG START|STOP|ABORT|BARO START|SAVE|LOAD|RESET\n");
            break;
        case '\r': case '\n': case ' ':
            break;   // im lang, khong reply
        default:
            snprintf(out, out_size, "ERR unknown: %c (go 'h' de xem help)\n", (char)c);
            break;
    }
}

// ================= public API =================

bool command_parser_feed_byte(int c, char *out, size_t out_size) {
    if (c < 0) return false;

    if (!s_line_mode) {
        if (c == '@') {
            s_line_mode = true;
            s_line_len = 0;
            return false;
        }
        handle_single_char(c, out, out_size);
        return out[0] != '\0';
    }

    if (c == '\n' || c == '\r') {
        s_line_buf[s_line_len] = '\0';
        handle_line(s_line_buf, out, out_size);
        s_line_mode = false;
        s_line_len = 0;
        return out[0] != '\0';
    }

    if (s_line_len + 1 < sizeof(s_line_buf)) {
        s_line_buf[s_line_len++] = (char)c;
    }
    return false;
}
