// app_main() THẬT — firmware điều khiển UAV qua console USB (D+/D-,
// USB-Serial-JTAG, KHÔNG qua UART0 — xem board_config.h). KHÔNG cần
// MicroPython: link thẳng vào components/flight_core/ (TẦNG DƯỚI, C thuần)
// giống hệt fc_bridge.c, chỉ khác nguồn lệnh là console thay vì Python VM.
// Build/nạp bằng platformio.ini ở gốc repo — pio run -e esp32s3_flight_core_verify -t upload.
//
// Đọc THẲNG main/board_config.h + main/app_config.h (đường dẫn tương đối —
// src/ và main/ nằm cạnh nhau ở gốc repo), khớp nguyên tắc "2 file đó là\n// nguồn DUY NHẤT" (giống fc_bridge_init()).
#include "../main/app_config.h"
#include "../main/board_config.h"
#include "../main/board_setup.h"   // board_config_fill() -- dung chung voi fc_bridge.c
#include "command_parser.h"
#include "flight_core/drivers/imu_driver.h"   // IMU_SAMPLE_RATE_HZ (nhip ngat ky vong, imu_int_test)
#include "flight_core/drivers/tof_driver.h"   // tof_driver_stall_restarts() trong tof_test
#include "flight_core/flight_core.h"
#include "flight_core/mahony_filter.h"   // MAHONY_DEFAULT_MAG_ERROR_GATE (in kem 'status')
#include "net_link.h"
#include "telemetry_format.h"

#include "esp_console.h"
#include "esp_log.h"
#include "esp_system.h"   // esp_get_free_heap_size() trong lenh 'tasks'
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>      // sqrtf() trong cmd_tof_test
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "uav_main";

// ============================================================================
// KHÔNG CÒN heartbeat_task — ĐÃ XOÁ CÓ CHỦ ĐÍCH
// ============================================================================
// Trước đây file này có một task riêng đẩy CMD_HEARTBEAT mỗi 300ms từ BÊN
// TRONG firmware. Điều đó làm Commander heartbeat watchdog
// (commander.h heartbeat_timeout_ms, mặc định 1000ms) trở nên VÔ NGHĨA: nguồn
// nuôi nó nằm trên chính con chip mà nó phải giám sát, nên nó chỉ còn "chứng
// minh" một điều duy nhất — firmware vẫn đang chạy. Mà nếu firmware KHÔNG chạy
// thì cả Commander lẫn task heartbeat đều đã chết cùng nhau, không ai còn để
// phát hiện gì.
//
// Cái watchdog này sinh ra để bắt: MẤT NGUỒN ĐIỀU KHIỂN BÊN NGOÀI (script
// Python treo, GUI đóng, WiFi rớt, người vận hành rút cáp). Đó là sự kiện xảy
// ra NGOÀI chip, nên bằng chứng "còn sống" BẮT BUỘC phải đến từ ngoài chip.
// Với heartbeat nội bộ, drone mất hoàn toàn liên lạc với PC vẫn tiếp tục treo
// lơ lửng chờ lệnh vô thời hạn, và log thì báo heartbeat_age ~0ms.
//
// GIỜ CMD_HEARTBEAT CHỈ ĐẾN TỪ NGUỒN NGOÀI (không đường nào khác):
//   - net_task()          : BẤT KỲ byte UDP nào nhận được (kể cả keepalive "\n")
//   - command_parser 'p'  : ping của GUI/console (tools/uav_udp_console.py tự gửi
//                            mỗi 400ms suốt phiên kết nối)
//   - console `heartbeat` : gõ tay khi bay bằng console USB
//   - MicroPython fc.heartbeat() : python/fc_api.py Heartbeat thread
//
// HỆ QUẢ PHẢI BIẾT TRƯỚC KHI BAY: bay bằng console USB mà không có gì gửi
// heartbeat thì Commander sẽ SOFT FAULT sau 1000ms và tự hạ cánh. Đó là hành
// vi ĐÚNG của watchdog này. Console dùng để bench/kiểm tra, còn bay thật đi
// qua GUI/UDP vốn đã tự ping.
#define POLL_INTERVAL_MS        100
#define ALTITUDE_TOLERANCE_MM   50

// Core cho MỌI task do firmware này tạo. Core 1 dành RIÊNG cho vòng bay
// (stabilize prio 23 + sensor_hub prio 22, xem sensor_hub.h) — mọi thứ thuộc
// tầng application/network phải nằm ở core 0 cùng WiFi/lwIP.
//
// KHÔNG dùng xTaskCreate() trần: trên build dual-core nó tương đương
// tskNO_AFFINITY, tức scheduler ĐƯỢC PHÉP đặt task lên core 1. Priority thấp
// chỉ đảm bảo chúng không PREEMPT vòng bay — nó KHÔNG ngăn chúng chạy trong
// khe rảnh của core 1, làm bẩn cache và thêm jitter vào dt của vòng 250Hz.
#define APP_TASK_CORE   0

// ĐƠN VỊ: BYTE. Đặt tên hằng thay vì viết thẳng 6144 vào xTaskCreatePinnedToCore()
// để lệnh `tasks` in được "còn trống / tổng" từ CÙNG một nguồn số — hai chỗ chép
// tay rồi lệch nhau sẽ biến công cụ đo thành công cụ nói dối.
#define NET_TASK_STACK_BYTES  6144

// Handle của net_task — CHỈ dùng để đọc high-water-mark trong lệnh `tasks`.
static TaskHandle_t s_net_task = NULL;

// ================= ground station qua WiFi/UDP =================
// net_task: KHÔNG chạy PID/estimator/mixer — chỉ transport (net_link) + parse
// text (command_parser) + format telemetry (telemetry_format). Toàn bộ điều
// khiển THẬT vẫn chạy trong stabilize_task (flight_core.c, core 1, 250Hz),
// net_task chỉ đẩy command_t vào queue giống hệt console/MicroPython — xem
// README mục "Ground station qua WiFi/UDP".
#define NET_TELEMETRY_INTERVAL_MS   50   // ~20Hz STATUS — đủ cho tune/plot, không nghẽn UDP/GUI
#define NET_POLL_INTERVAL_MS        5

static void net_task(void *arg) {
    (void)arg;

    esp_err_t err = net_link_init(WIFI_HOSTNAME);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "net_link_init() THAT BAI: %s -- dieu khien qua WiFi/UDP se KHONG hoat "
"dong (console USB van dung binh thuong). Kiem tra WIFI_STA_SSID/PASS "
"trong app_config.h da dien dung chua.", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "WiFi/UDP san sang tai %s:%d -- chay 'python tools/uav_udp_console.py %s' tren PC",
              net_link_ip_string(), (int)WIFI_UDP_PORT, net_link_ip_string());

    char rx_buf[256];

    // ---- STATUS_LINE_BUF: 512 la QUA NHO, va no da lam GUI MU HOAN TOAN ----
    // Dong STATUS thuc te ~1010 byte (113 field — do bang chinh chuoi mau trong
    // python/test_status_parse_offline.py). Voi buffer 512, snprintf() cat dong
    // o khoang "FAULT=/BATV=" va NEM DI toan bo phan duoi: KILL, IAGE, MSAT,
    // AIRB, BAT*, TKO*, ARMREJ va TKOREJ.
    //
    // Hau qua KHONG phai "mat vai field cuoi". STATUS_RE trong
    // tools/uav_udp_console.py dung .match() tren CA dong, nen dong bi cat
    // KHONG MATCH GI CA -> moi nhanh cap nhat nhan/trang thai cua GUI khong bao
    // gio chay. Bieu do van ve duoc (PLOT_RE/ALT_PLOT_RE dung .search() va chi
    // can phan DAU dong) nen nhin qua tuong GUI van song. Day chinh la ly do
    // "log bao san sang" trong khi ly do tu choi TAKEOFF (TKOREJ=) khong bao
    // gio hien ra duoc.
    //
    // 1536 cho bien rong khi cac bo dem (%u/%d: TERR, DLM, BACC, BREJ, BCREJ,
    // BSEQ, LDT, LMX) len nhieu chu so sau thoi gian chay dai. Do dai THUC
    // (~1010-1100 byte) van duoi gioi han payload UDP 1472 byte cua MTU 1500
    // nen khong sinh phan manh IP. Phia GUI da doc 4096 (RECV_BUF_SIZE).
    #define STATUS_LINE_BUF 2048

    // ---- static, KHONG phai bien cuc bo tren stack ----
    // net_task la SINGLETON (tao dung mot lan trong app_main) nen static an
    // toan, va no giu 2x1536 byte NAM NGOAI stack.
    //
    // ⚠ DE TREN STACK LA MOT LOI THAT DA XAY RA: khi hai buffer nay con la
    // bien cuc bo, net_task giu 256+1536+1536 = 3328 byte tren stack 4096 byte,
    // chi con <800 byte cho net_link_init() -> esp_netif_init() -> tcpip_init()
    // -> lwip_init(). Tran stack, de len mang gpio_isr_func[] (cap phat tren
    // heap boi gpio_install_isr_service), va ISR data-ready cua IMU chay
    // 250Hz nhay vao con tro rac -> "Guru Meditation InstrFetchProhibited,
    // PC=0x00000003" ngay giua esp_netif_init, KEM theo boot loop.
    //
    // Canary cua FreeRTOS (CONFIG_FREERTOS_CHECK_STACKOVERFLOW_CANARY) KHONG
    // bat duoc vi no chi kiem tra luc chuyen ngu canh — ngat da no truoc do.
    // Vi vay panic hien ra o mot noi HOAN TOAN khong lien quan toi nguyen nhan.
    static char reply[STATUS_LINE_BUF];
    static char status_line[STATUS_LINE_BUF];
    int64_t last_telemetry_us = 0;

    while (1) {
        const int n = net_link_read(rx_buf, sizeof(rx_buf));
        if (n > 0) {
            // BẤT KỲ byte nào nhận được (kể cả gói keepalive "\n" rỗng của GUI)
            // = "PC còn sống" -> nuôi Commander heartbeat watchdog. Mất hẳn
            // (không còn gói nào) -> watchdog hết hạn -> SOFT FAULT -> LANDING
            // tự động (xem commander.c) — đây là lớp failsafe "mất PC" mục 12.
            command_t hb = {0};
            hb.type = CMD_HEARTBEAT;
            flight_core_push_command(&hb);

            for (int i = 0; i < n; i++) {
                if (command_parser_feed_byte((unsigned char)rx_buf[i], reply, sizeof(reply))) {
                    net_link_write(reply, strlen(reply));
                }
            }
        }

        const int64_t now_us = esp_timer_get_time();
        // STATUS định kỳ CHỈ gửi sau khi user bấm 'f' (nút "Flight" GUI) —
        // xem command_parser_telemetry_enabled(). Học được UDP peer KHÔNG tự
        // bật streaming nữa; lệnh khác (@CAL/@PID/../single-char) vẫn LUÔN
        // trả reply trực tiếp bất kể cờ này, không bị ảnh hưởng.
        if (command_parser_telemetry_enabled() &&
            (now_us - last_telemetry_us) > (int64_t)NET_TELEMETRY_INTERVAL_MS * 1000) {
            last_telemetry_us = now_us;
            telemetry_format_status_line(status_line, sizeof(status_line));
            net_link_write(status_line, strlen(status_line));
        }

        vTaskDelay(pdMS_TO_TICKS(NET_POLL_INTERVAL_MS));
    }
}

static bool parse_move_dir(const char *s, move_dir_t *out) {
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

// ================= console commands =================
// Ngữ nghĩa BLOCKING (poll telemetry tới khi xong/timeout) — PORT trực tiếp
// từ python/fc_api.py (cùng hằng số timeout mặc định) để hành vi console này
// và REPL MicroPython (khi có) giống hệt nhau.

static int cmd_status(int argc, char **argv) {
    (void)argc; (void)argv;
    telemetry_snapshot_t t;
    flight_core_read_telemetry(&t);
    printf("state=%s armed=%d\n", fsm_state_name(t.state), (int)t.armed);
    printf("attitude: roll=%.1f pitch=%.1f yaw=%.1f valid=%d gyro=(%.1f,%.1f,%.1f)dps\n",
           t.roll_deg, t.pitch_deg, t.yaw_deg, (int)t.attitude_valid,
           t.gyro_roll_dps, t.gyro_pitch_dps, t.gyro_yaw_dps);
    // sensor_err_count tang nhanh trong luc dung yen = I2C dang loi that su
    // (bus/day/pull-up) -> lam sai ca gyro/accel calib (xem flight_core.c
    // buoc 1b: mau loi bi loai khoi trung binh, nhung loi qua nhieu se lam
    // calib THAT BAI thay vi tinh ra so sai).
    // mag_ok=0 => mag_driver_init() THAT BAI luc boot (CHIP_ID sai / read-back
    // CTRL sai / chip khong phat mau -- xem log boot 'mag_driver'). Khi do
    // calib_mag_start se bi TU CHOI ngay thay vi chay 60s vo ich.
    printf("imu_ok=%d mag_ok=%d baro_ok=%d sensor_err_count=%u loop_dt=%.2fms\n",
           (int)t.imu_ok_driver, (int)t.mag_ok_driver, (int)t.baro_ok_driver,
           (unsigned)t.sensor_err_count, t.loop_dt_ms);
    // Nhip vong dieu khien. int_active=1 => chay theo data-ready INT cua MPU6050
    // (do tre mau on dinh). =0 => dong ho FreeRTOS. timeout>0 nghia la ngat da
    // tung ngung den -> kiem tra day INT. isr vs wake chenh nhieu = ngat den
    // nhung task khong kip tieu thu, hoac co canh gia (day tha noi).
    // no_new_sample = so tick vong bay chay MA KHONG co mau IMU moi (seq khong
    // doi hoac mau qua han). O nhip binh thuong phai DUNG YEN: hub publish dung
    // 1 mau roi moi notify. Tang deu = vong bay quay nhanh hon nguon mau -> cac
    // buoc tich phan (Mahony/Az/Vz/Z/I-term) bi BO QUA o dung nhung tick do,
    // xem imu_updated trong flight_core.c.
    printf("loop_clock: imu_int_active=%d isr=%u wake=%u timeout=%u no_new_sample=%u\n",
           (int)t.imu_int_active, (unsigned)t.imu_int_isr_count,
           (unsigned)t.imu_int_wake_count, (unsigned)t.imu_int_timeout_count,
           (unsigned)t.imu_no_new_sample_count);
    // mag_ok=1 chi la "doc I2C duoc". Cai QUYET DINH yaw co het troi hay khong
    // la used=1 + used_count TANG DEU. used_count dung yen => mag dang bi chan:
    //  - valid_mag=0 (chua calib) -> chay 'calib_mag_start'
    //  - valid_mag=1 ma rejected_count tang nhanh -> mag_err vuot gate 0.90,
    //    nghi LECH TRUC mag vs IMU (mag_driver.c chua remap truc) hoac calib sai
    // norm nen GAN NHU KHONG DOI khi xoay drone -- norm nhay manh = con hard-iron.
    printf("mag_fusion: used=%d rejected=%d ref_valid=%d norm=%.0f err=%.3f (gate %.2f) "
"used_count=%u rejected_count=%u\n",
           (int)t.mag_used, (int)t.mag_rejected, (int)t.mag_reference_valid,
           t.mag_norm, t.mag_error_norm, (double)MAHONY_DEFAULT_MAG_ERROR_GATE,
           (unsigned)t.mag_used_count, (unsigned)t.mag_rejected_count);
    // airborne=0 VA candidate=0 (GROUND) -> alt/vz KHOA CUNG 0, chua tich phan
    // gi (xem alt_estimator.h "BA PHA") -- binh thuong khi dung yen/dang ARMED
    // cho lenh takeoff, KHONG phai loi. candidate=1 = dang cho phep inertial
    // chay de TAO bang chung liftoff (chua confirmed).
    printf("alt: %.2fm vz=%.2fm/s target=%.2fm valid=%d airborne=%d candidate=%d\n",
           t.alt_m, t.vz_ms, t.alt_target_m, (int)t.alt_valid,
           (int)t.alt_airborne, (int)t.alt_liftoff_candidate);
    printf("alt_inertial: vz_accel_only=%.3fm/s z_inertial=%.2fm\n",
           t.vz_accel_only_ms, t.z_inertial_m);
    // seq phai TANG deu (~50/s) neu BMP280 con song; dt~0.020s = dung nhip
    // baro that (KHONG phai 0.004s cua vong dieu khien).
    printf("baro: raw=%.2fm filtered=%.2fm innov=%.2fm seq=%u dt=%.3fs accept=%u reject=%u "
"reject_lientuc=%u reacquire=%d fusion_init=%d | ground calibrated=%d healthy=%d std=%.2fPa\n",
           t.baro_alt_m, t.baro_filtered_alt_m, t.baro_innovation_m,
           (unsigned)t.baro_seq, (double)t.baro_dt_s,
           (unsigned)t.baro_accept_count, (unsigned)t.baro_reject_count,
           (unsigned)t.baro_reject_consecutive, (int)t.baro_reacquire_active,
           (int)t.baro_fusion_initialized,
           (int)t.baro_calibrated, (int)t.baro_healthy, t.baro_ground_noise_std_pa);
    // ---- ToF huong xuong (VL53L0X) ----
    // age=-1 nghia la sensor_hub CHUA TUNG lay duoc mau nao — phan biet han voi
    // "sensor doc 0m". Do la thu dau tien phai nhin khi ToF "khong hoat dong".
    //
    // surface: 0=UNKNOWN 1=FLOOR 2=OTHER(ban/ghe). corr=1 nghia la ToF DANG
    // duoc phep sua world-Z. Tren mat dat (ground lock) corr LUON =0 va reject
    // tang deu — DUNG THIET KE, khong phai loi: Z=0 luc con o dat la ground
    // truth chac chan hon moi cam bien.
    printf("tof: driver_ok=%d raw=%.3fm valid=%d age=%dms | vertical=%.3fm innov=%.3fm\n",
           (int)t.tof_ok_driver, t.tof_range_m, (int)t.tof_valid, (int)t.tof_age_ms,
           (double)t.tof_vertical_m, (double)t.tof_innovation_m);
    printf("tof_surface: state=%d(%s) corr=%d surface_z=%.2fm floor_z=%.2fm ground_ref=%.3fm "
"accept=%u reject=%u | landing_z=%.2fm valid=%d\n",
           (int)t.tof_surface_state,
           (t.tof_surface_state == 1) ? "FLOOR" : ((t.tof_surface_state == 2) ? "OTHER" : "UNKNOWN"),
           (int)t.tof_correction_enabled,
           (double)t.tof_surface_z_m, (double)t.floor_plane_z_m,
           (double)t.tof_ground_range_m,
           (unsigned)t.tof_accept_count, (unsigned)t.tof_reject_count,
           (double)t.landing_surface_z_m, (int)t.landing_surface_valid);
    // degraded=1 nghia la Z VAN hop le nhung da qua lau khong co correction nao
    // (ToF lan baro) -> dang dead-reckon thuan accel, sai so tang BAC HAI.
    printf("alt_health: valid=%d degraded=%d no_correction=%dms\n",
           (int)t.alt_valid, (int)t.alt_degraded, (int)t.alt_no_correction_ms);
    printf("vert_accel: raw=%.3f lpf=%.3f corrected=%.3f bias=%.4f (tat ca m/s2)\n",
           t.vert_accel_raw_ms2, t.vert_accel_ms2, t.az_corrected_ms2, (double)t.accel_bias_ms2);
    printf("throttle=%d motors=(%d,%d,%d,%d)\n", t.throttle_duty, t.m1, t.m2, t.m3, t.m4);
    // comp LUON = 1.000x: bu throttle theo pin DA BO HAN (xem tuning.h muc 10b).
    // Giu field chi de khong doi format dong STATUS. Dien ap pin gio CHI dung
    // cho failsafe (san pin -> LANDING) va prearm_check().
    // battery=0.00V nghia la KHONG CO mau dung duoc (ngoai dai 1S hoac thieu
    // ADC calibration) -> commander TU BO QUA. Nhin dong battery_adc ben duoi
    // de biet sai o dau: raw kep 0/4095 = ADC/day; mV dung nhung V sai = chia
    // ap sai; cali=0 = thieu eFuse calibration.
    printf("battery=%.2fV comp=%.3fx (comp da bo, luon 1.000)\n",
           t.battery_v, t.battery_comp);
    printf("battery_adc: raw=%d mv=%d ratio=%.2f v_raw=%.3f valid=%d cali=%d age=%dms\n",
           t.bat_adc_raw, t.bat_adc_mv, (double)t.bat_divider_ratio,
           (double)t.bat_voltage_raw_v, (int)t.bat_valid, (int)t.bat_calibrated,
           (int)t.bat_age_ms);
    // valid=1 nghia la CO gia tri calib hop le trong RAM/NVS -- KHONG co nghia
    // la phien calib GAN NHAT thanh cong: 1 phien that bai KHONG xoa calib CU,
    // nen mag=1 hoan toan co the ton tai song song voi 1 lan calib_mag vua hong.
    printf("calib: uncalibrated=%d | valid: gyro=%d accel=%d mag=%d",
           (int)t.uncalibrated, (int)t.calib_gyro_valid,
           (int)t.calib_accel_valid, (int)t.calib_mag_valid);
    if (t.calib_gyro_active) printf(" | GYRO dang do...");
    if (t.calib_accel_capturing) printf(" | ACCEL dang bat mat %d/6...", t.calib_accel_faces_done + 1);
    else if (t.calib_accel_faces_done > 0) printf(" | ACCEL da bat %d/6 mat", t.calib_accel_faces_done);
    printf("\n");
    if (t.calib_mag_active) {
        // range 3 truc nen GAN BANG NHAU va tang deu khi xoay. mau=0 keo dai
        // = chip khong phat mau (xoay them vo ich) -- xem loi_i2c/chua_sansang.
        printf("calib_mag: con %ds | mau=%u loi_i2c=%u chua_sansang=%u | range=(%.0f,%.0f,%.0f)\n",
               t.calib_mag_seconds_left, (unsigned)t.calib_mag_sample_count,
               (unsigned)t.calib_mag_read_err_count, (unsigned)t.calib_mag_notready_count,
               t.calib_mag_range_x, t.calib_mag_range_y, t.calib_mag_range_z);
    }
    // Quality metric lan calib GAN NHAT (0 neu chua tung calib) -- xem
    // "6. Gyro calibration output" / "13. Accel calibration verification".
    printf("calib_quality: imu_temp=%.1fC gyro_std=(%.3f,%.3f,%.3f)dps accel_residual=%.3fg\n",
           t.imu_temp_c, t.gyro_calib_std_x_dps, t.gyro_calib_std_y_dps, t.gyro_calib_std_z_dps,
           t.accel_calib_residual_g);
    return 0;
}

static int cmd_arm(int argc, char **argv) {
    (void)argc; (void)argv;
    command_t cmd = {0};
    cmd.type = CMD_ARM;
    if (!flight_core_push_command(&cmd)) { printf("loi: command queue day\n"); return 1; }
    printf("da gui ARM -- go 'status' de xac nhan (co the bi tu choi neu attitude invalid "
"hoac nghieng qua %.0f do, xem log firmware)\n", FSM_ARM_MAX_TILT_DEG);
    return 0;
}

static int cmd_disarm(int argc, char **argv) {
    (void)argc; (void)argv;
    command_t cmd = {0};
    cmd.type = CMD_DISARM;
    flight_core_push_command(&cmd);
    printf("da gui DISARM\n");
    return 0;
}

static int cmd_takeoff(int argc, char **argv) {
    if (argc < 2) { printf("usage: takeoff <mm> [timeout_s=5] [climb_timeout_s=8]\n"); return 1; }
    const int32_t alt_mm = atoi(argv[1]);
    const float timeout_s = (argc >= 3) ? (float)atof(argv[2]) : 5.0f;
    const float climb_timeout_s = (argc >= 4) ? (float)atof(argv[3]) : 8.0f;

    command_t cmd = {0};
    cmd.type = CMD_TAKEOFF;
    cmd.as.takeoff.alt_mm = alt_mm;
    if (!flight_core_push_command(&cmd)) { printf("loi: command queue day\n"); return 1; }

    // Pha 1: đợi HOLDING (đã rời đất an toàn) — xem GHI CHÚ 1 trong
    // flight_state_machine.h: spool KHÔNG tự leo tới alt_mm.
    printf("takeoff: doi HOLDING...\n");
    telemetry_snapshot_t t;
    int64_t t0 = esp_timer_get_time();
    while (1) {
        flight_core_read_telemetry(&t);
        if (t.state == FSM_HOLDING) break;
        if (t.state == FSM_EMERGENCY) { printf("loi: EMERGENCY trong luc takeoff\n"); return 1; }
        if (t.state == FSM_DISARMED) { printf("loi: takeoff bi tu choi hoac ABORT (xem log firmware)\n"); return 1; }
        if ((esp_timer_get_time() - t0) > (int64_t)(timeout_s * 1e6f)) {
            command_t land_cmd = {0};
            land_cmd.type = CMD_LAND;
            flight_core_push_command(&land_cmd);
            printf("loi: qua %.1fs ma chua HOLDING -> da goi land() an toan\n", timeout_s);
            return 1;
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }

    // Pha 2: leo tới alt_mm mong muốn (lệnh riêng, xem takeoff_land.h).
    command_t alt_cmd = {0};
    alt_cmd.type = CMD_SET_ALTITUDE;
    alt_cmd.as.set_altitude.alt_mm = alt_mm;
    flight_core_push_command(&alt_cmd);

    t0 = esp_timer_get_time();
    while (1) {
        flight_core_read_telemetry(&t);
        const int32_t cur_mm = (int32_t)(t.alt_m * 1000.0f);
        const int32_t diff = cur_mm - alt_mm;
        if ((diff >= -ALTITUDE_TOLERANCE_MM) && (diff <= ALTITUDE_TOLERANCE_MM)) break;
        if (t.state == FSM_EMERGENCY || t.state == FSM_DISARMED || t.state == FSM_LANDING) break;
        if ((esp_timer_get_time() - t0) > (int64_t)(climb_timeout_s * 1e6f)) break;
        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
    printf("takeoff xong: state=%s alt=%.2fm\n", fsm_state_name(t.state), t.alt_m);
    return 0;
}

static int cmd_land(int argc, char **argv) {
    const float timeout_s = (argc >= 2) ? (float)atof(argv[1]) : 15.0f;
    command_t cmd = {0};
    cmd.type = CMD_LAND;
    if (!flight_core_push_command(&cmd)) { printf("loi: command queue day\n"); return 1; }

    printf("land: doi DISARMED...\n");
    telemetry_snapshot_t t;
    int64_t t0 = esp_timer_get_time();
    while (1) {
        flight_core_read_telemetry(&t);
        if (t.state == FSM_DISARMED) break;
        if ((esp_timer_get_time() - t0) > (int64_t)(timeout_s * 1e6f)) {
            printf("loi: qua %.1fs ma chua DISARMED\n", timeout_s);
            return 1;
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
    printf("da DISARMED\n");
    return 0;
}

static int cmd_hover(int argc, char **argv) {
    const float sec = (argc >= 2) ? (float)atof(argv[1]) : 3.0f;
    command_t cmd = {0};
    cmd.type = CMD_HOVER;
    flight_core_push_command(&cmd);

    telemetry_snapshot_t t;
    const int64_t t0 = esp_timer_get_time();
    while ((esp_timer_get_time() - t0) < (int64_t)(sec * 1e6f)) {
        flight_core_read_telemetry(&t);
        if (t.state != FSM_HOLDING && t.state != FSM_FLYING) break;   // fault -> dừng chờ sớm, an toàn
        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
    return 0;
}

static int cmd_move(int argc, char **argv) {
    if (argc < 4) {
        printf("usage: move <forward|back|left|right|up|down|cw|ccw> <pct 0-100> <sec>\n");
        return 1;
    }
    move_dir_t dir;
    if (!parse_move_dir(argv[1], &dir)) { printf("huong khong hop le: %s\n", argv[1]); return 1; }

    command_t cmd = {0};
    cmd.type = CMD_MOVE;
    cmd.as.move.dir = dir;
    cmd.as.move.pct = atoi(argv[2]);
    cmd.as.move.sec = (float)atof(argv[3]);
    if (!flight_core_push_command(&cmd)) { printf("loi: command queue day\n"); return 1; }

    vTaskDelay(pdMS_TO_TICKS((int)(cmd.as.move.sec * 1000.0f)));   // khớp timed-command tự trả HOLDING (flight_core.c)
    return 0;
}

static int cmd_yaw(int argc, char **argv) {
    if (argc < 2) { printf("usage: yaw <deg>\n"); return 1; }
    command_t cmd = {0};
    cmd.type = CMD_SET_YAW;
    cmd.as.set_yaw.yaw_deg = (float)atof(argv[1]);
    flight_core_push_command(&cmd);
    return 0;
}

static int cmd_alt(int argc, char **argv) {
    if (argc < 2) { printf("usage: alt <mm>\n"); return 1; }
    command_t cmd = {0};
    cmd.type = CMD_SET_ALTITUDE;
    cmd.as.set_altitude.alt_mm = atoi(argv[1]);
    flight_core_push_command(&cmd);
    return 0;
}

static int cmd_heartbeat(int argc, char **argv) {
    (void)argc; (void)argv;
    command_t cmd = {0};
    cmd.type = CMD_HEARTBEAT;
    flight_core_push_command(&cmd);
    telemetry_snapshot_t t;
    flight_core_read_telemetry(&t);
    printf("heartbeat da gui. LUU Y: firmware KHONG con tu sinh heartbeat trong nen -- "
"watchdog Commander (timeout mac dinh 1000ms) chi duoc nuoi boi nguon dieu khien "
"BEN NGOAI (UDP/GUI/MicroPython) hoac chinh lenh nay. Bay bang console USB thi phai "
"go lai truoc moi 1s, neu khong Commander se SOFT FAULT -> LANDING. "
"heartbeat_age truoc lenh nay = %dms\n", (int)t.heartbeat_age_ms);
    return 0;
}

static int cmd_test_motor(int argc, char **argv) {
    if (argc < 3) { printf("usage: test_motor <1-4> <duty_pct 0-100>\n"); return 1; }
    printf("!!! XAC NHAN DA THAO HET CANH QUAT TRUOC KHI TIEP TUC !!!\n");
    printf("(chi chay neu dang DISARMED -- xem app_config.h muc \"XAC NHAN VI TRI VAT LY DONG CO\")\n");

    command_t cmd = {0};
    cmd.type = CMD_TEST_MOTOR;
    cmd.as.test_motor.motor_idx = atoi(argv[1]);
    cmd.as.test_motor.duty_pct = atoi(argv[2]);
    if (!flight_core_push_command(&cmd)) { printf("loi: command queue day\n"); return 1; }
    printf("da gui test_motor M%d @ %s%% -- xem log firmware de xac nhan chay hay bi tu choi\n",
           (int)cmd.as.test_motor.motor_idx, argv[2]);
    return 0;
}

static int cmd_test_motor_all(int argc, char **argv) {
    if (argc < 2) { printf("usage: test_motor_all <duty_pct 0-100>\n"); return 1; }
    printf("!!! XAC NHAN DA THAO HET CANH QUAT TRUOC KHI TIEP TUC !!!\n");
    printf("(chi chay neu dang DISARMED -- ca 4 dong co quay CUNG LUC, DONG duty -- "
"chi de sanity-check du 4 con quay, KHONG dung de xac nhan vi tri/chieu, "
"xem app_config.h muc \"XAC NHAN VI TRI VAT LY DONG CO\")\n");

    command_t cmd = {0};
    cmd.type = CMD_TEST_MOTOR;
    cmd.as.test_motor.motor_idx = 0;   // 0 = ca 4 cung luc, xem command.h
    cmd.as.test_motor.duty_pct = atoi(argv[1]);
    if (!flight_core_push_command(&cmd)) { printf("loi: command queue day\n"); return 1; }
    printf("da gui test_motor_all @ %s%% -- xem log firmware de xac nhan chay hay bi tu choi\n", argv[1]);
    return 0;
}

// ================= bench-test PID tuning (xem GHI CHÚ 3 flight_state_machine.h) =================
// Throttle đi thẳng qua CMD_BENCH_THROTTLE_STEP, KHÔNG qua alt_hold/takeoff —
// drone PHẢI được giữ chặt/kẹp trên giá đỡ, đây KHÔNG phải chế độ bay. Bước
// mỗi lần bấm "+"/"-" — ĐỔI Ở ĐÂY nếu muốn nấc khác (thang duty 0..2000, xem
// motor_driver.h MOTOR_SAFE_MAX_DUTY), KHÔNG cần sửa flight_core.c.
#define BENCH_THROTTLE_STEP_DUTY   20

static int cmd_bench_start(int argc, char **argv) {
    (void)argc; (void)argv;
    printf("!!! XAC NHAN DRONE DA DUOC GIU CHAT / KEP TREN GIA DO TRUOC KHI TIEP TUC !!!\n");
    printf("(day KHONG phai che do bay -- throttle se tang dan qua lenh '+'/'-', "
"attitude PID chay binh thuong tu throttle >= %d, xem tuning.h ATT_MIN_THROTTLE_DUTY)\n",
           ATT_MIN_THROTTLE_DUTY);
    command_t cmd = {0};
    cmd.type = CMD_BENCH_RAMP_START;
    if (!flight_core_push_command(&cmd)) { printf("loi: command queue day\n"); return 1; }
    printf("da gui bench_start -- xem log firmware de xac nhan, roi dung '+'/'-' de chinh ga\n");
    return 0;
}

static int cmd_bench_throttle_up(int argc, char **argv) {
    (void)argc; (void)argv;
    command_t cmd = {0};
    cmd.type = CMD_BENCH_THROTTLE_STEP;
    cmd.as.bench_step.delta_duty = BENCH_THROTTLE_STEP_DUTY;
    if (!flight_core_push_command(&cmd)) { printf("loi: command queue day\n"); return 1; }
    printf("+%d duty -- go 'status' de xem throttle hien tai\n", BENCH_THROTTLE_STEP_DUTY);
    return 0;
}

static int cmd_bench_throttle_down(int argc, char **argv) {
    (void)argc; (void)argv;
    command_t cmd = {0};
    cmd.type = CMD_BENCH_THROTTLE_STEP;
    cmd.as.bench_step.delta_duty = -BENCH_THROTTLE_STEP_DUTY;
    if (!flight_core_push_command(&cmd)) { printf("loi: command queue day\n"); return 1; }
    printf("-%d duty -- go 'status' de xem throttle hien tai\n", BENCH_THROTTLE_STEP_DUTY);
    return 0;
}

static int cmd_bench_stop(int argc, char **argv) {
    (void)argc; (void)argv;
    command_t cmd = {0};
    cmd.type = CMD_BENCH_RAMP_STOP;
    if (!flight_core_push_command(&cmd)) { printf("loi: command queue day\n"); return 1; }
    printf("da gui bench_stop -- motor cat ngay, ve ARMED\n");
    return 0;
}

// ================= calibration (bench-only, CHỈ khi DISARMED) =================
// Console là NGUỒN DUY NHẤT chạy được calib trên firmware này nếu không build
// MicroPython port (xem platformio.ini) — PHẢI có parity đầy đủ với fc.calibrate_*()
// (micropython_module/fc/fc_module.c), nếu không firmware console/UDP (bản
// THẬT SỰ được nạp — xem platformio.ini) sẽ KHÔNG BAO GIỜ arm được sau khi
// gate calib mới thêm vào (CMD_ARM từ chối nếu s_uncalibrated, xem flight_core.c).

static int cmd_calib_gyro(int argc, char **argv) {
    (void)argc; (void)argv;
    command_t cmd = {0};
    cmd.type = CMD_CALIB_GYRO;
    flight_core_push_command(&cmd);
    printf("calib_gyro: da gui -- DUNG YEN drone; settle 1.5s + collect 4s + validate 1.5s, xem cal_status\n");
    return 0;
}

// cmd_cal_status() - mot man hinh DUY NHAT tra loi "calibration co an khong".
//
// VI SAO can lenh rieng: `status` da rat dai va khong cho nhin ky gyro. Cau hoi
// "bias co thuc su duoc khu khong" can 3 dong so DAT CANH NHAU (raw/bias/corr)
// de doc gia tri va TU KIEM CHUNG raw - bias == corr. Roi rac trong status thi
// khong ai ghep lai duoc.
static int cmd_cal_status(int argc, char **argv) {
    (void)argc; (void)argv;
    telemetry_snapshot_t t;
    flight_core_read_telemetry(&t);

    static const char *GCAL_NAMES[] = {
        "IDLE", "SETTLING", "WAIT_STATIONARY", "COLLECT", "VALIDATE", "PASS", "FAIL"
    };
    const int gs = t.gyro_cal_state;
    const char *gname = (gs >= 0 && gs < (int)(sizeof(GCAL_NAMES)/sizeof(GCAL_NAMES[0])))
                         ? GCAL_NAMES[gs] : "?";

    printf("=== MPU6050 CONFIG (read-back tu chip) ===\n");
    printf("  valid=%d  GYRO_CONFIG=0x%02X FS_SEL=%u scale=%.1f LSB/dps\n",
           (int)t.imu_cfg_valid, t.imu_gyro_config, (unsigned)t.imu_fs_sel,
           (double)t.imu_gyro_lsb_per_dps);
    printf("           ACCEL_CONFIG=0x%02X AFS_SEL=%u scale=%.0f LSB/g\n",
           t.imu_accel_config, (unsigned)t.imu_afs_sel, (double)t.imu_accel_lsb_per_g);
    if (!t.imu_cfg_valid) printf("  ** CONFIG KHONG HOP LE -> ARM BI CHAN **\n");

    printf("=== GYRO (dps) ===  state=%s bad_samples=%d fail=%d\n",
           gname, t.gyro_cal_bad_samples, t.gyro_cal_fail);
    printf("  GRAW  = %8.3f %8.3f %8.3f\n",
           (double)t.gyro_raw_dps.x, (double)t.gyro_raw_dps.y, (double)t.gyro_raw_dps.z);
    printf("  GBIAS = %8.3f %8.3f %8.3f\n",
           (double)t.gyro_bias_dps.x, (double)t.gyro_bias_dps.y, (double)t.gyro_bias_dps.z);
    printf("  GCORR = %8.3f %8.3f %8.3f   <- Mahony/PID dung dong nay\n",
           (double)t.gyro_corr_dps.x, (double)t.gyro_corr_dps.y, (double)t.gyro_corr_dps.z);
    printf("  GRAWmean = %8.3f %8.3f %8.3f\n",
           (double)t.gyro_raw_mean_dps.x, (double)t.gyro_raw_mean_dps.y,
           (double)t.gyro_raw_mean_dps.z);
    printf("  GCORRmean= %8.3f %8.3f %8.3f   (independent validation)\n",
           (double)t.gyro_corr_mean_dps.x, (double)t.gyro_corr_mean_dps.y,
           (double)t.gyro_corr_mean_dps.z);
    printf("  GSTDRAW  = %8.3f %8.3f %8.3f\n",
           (double)t.gyro_raw_std_dps.x, (double)t.gyro_raw_std_dps.y,
           (double)t.gyro_raw_std_dps.z);
    printf("  GSTDCORR = %8.3f %8.3f %8.3f\n",
           (double)t.gyro_corr_std_dps.x, (double)t.gyro_corr_std_dps.y,
           (double)t.gyro_corr_std_dps.z);
    printf("  kiem tra: GRAW - GBIAS phai == GCORR; drone dung yen => GCORR ~ 0\n");
    printf("  gyro_valid=%d (NVS co bias cu=%d)  temp calib=%.1fC hien tai=%.1fC%s\n",
           (int)t.calib_gyro_valid, (int)t.gyro_valid_from_nvs,
           (double)t.gyro_cal_temp_c, (double)t.imu_temp_c,
           t.gyro_cal_temp_warn ? "  ** LECH NHIET DO LON **" : "");

    printf("=== ACCEL (g) ===  accel_valid=%d (diagonal offset+scale, 6-face)\n",
           (int)t.calib_accel_valid);
    printf("  ARAW  = %8.4f %8.4f %8.4f\n",
           (double)t.accel_raw_g.x, (double)t.accel_raw_g.y, (double)t.accel_raw_g.z);
    printf("  ACORR = %8.4f %8.4f %8.4f   ANORM=%.4f (nen ~1.0 khi dung yen)\n",
           (double)t.accel_corr_g.x, (double)t.accel_corr_g.y, (double)t.accel_corr_g.z,
           (double)t.accel_norm_g);
    printf("  faces_done=%d/6\n", t.calib_accel_faces_done);
    return 0;
}

static int cmd_calib_gyro_abort(int argc, char **argv) {
    (void)argc; (void)argv;
    command_t cmd = {0};
    cmd.type = CMD_CALIB_GYRO_ABORT;
    flight_core_push_command(&cmd);
    printf("calib_gyro_abort: da gui\n");
    return 0;
}

static int cmd_calib_accel_face(int argc, char **argv) {
    (void)argc; (void)argv;
    command_t cmd = {0};
    cmd.type = CMD_CALIB_ACCEL_FACE;
    flight_core_push_command(&cmd);
    printf("calib_accel_face: da gui -- GIU YEN drone o 1 huong ~0.5s. Goi lai lenh nay 6 lan,\n"
"doi huong (mat) khac nhau moi lan (vd: nam ngua, up, nghieng 4 canh) de bao phu +-g\n"
"ca 3 truc. Xem log firmware 'mat N/6' de theo doi tien do.\n");
    return 0;
}

static int cmd_calib_accel_reset(int argc, char **argv) {
    (void)argc; (void)argv;
    command_t cmd = {0};
    cmd.type = CMD_CALIB_ACCEL_RESET;
    flight_core_push_command(&cmd);
    printf("calib_accel_reset: da huy tien trinh 6-face dang do (neu co), lam lai tu dau\n");
    return 0;
}

static int cmd_calib_mag_start(int argc, char **argv) {
    (void)argc; (void)argv;
    command_t cmd = {0};
    cmd.type = CMD_CALIB_MAG_START;
    flight_core_push_command(&cmd);
    printf("calib_mag_start: da gui -- TU DONG chay 60s, XOAY drone hinh so 8 LIEN TUC suot thoi gian nay. "
"Tu dong tinh ket qua khi het gio (khong can 'calib_mag_stop' -- lenh do van dung duoc de ket thuc SOM)\n");
    return 0;
}

static int cmd_calib_mag_stop(int argc, char **argv) {
    (void)argc; (void)argv;
    command_t cmd = {0};
    cmd.type = CMD_CALIB_MAG_STOP;
    flight_core_push_command(&cmd);
    printf("calib_mag_stop: da gui -- KET THUC SOM (truoc khi het 60s tu dong), tinh ngay voi mau da co\n");
    return 0;
}

static int cmd_calib_mag_abort(int argc, char **argv) {
    (void)argc; (void)argv;
    command_t cmd = {0};
    cmd.type = CMD_CALIB_MAG_ABORT;
    flight_core_push_command(&cmd);
    printf("calib_mag_abort: da gui\n");
    return 0;
}

static int cmd_calib_baro_ground(int argc, char **argv) {
    (void)argc; (void)argv;
    command_t cmd = {0};
    cmd.type = CMD_CALIB_BARO_GROUND;
    flight_core_push_command(&cmd);
    printf("calib_baro_ground: da gui -- DUNG YEN drone tren mat dat ~1s, xem log firmware "
"de biet ket qua (khuyen nghi goi lenh nay NGAY TRUOC arm de co moc moi nhat)\n");
    return 0;
}

// mag_test [sign] [loop_ms] -- KHONG phai calibration: chay lai trinh tu init
// QMC5883P va log TUNG BUOC de biet chip hong o dau. Chay 2 lan (mag_test 0 va
// mag_test 1) roi SO 2 LOG de chot xem chip nay co can REG 0x29=0x06 hay khong.
static int cmd_mag_test(int argc, char **argv) {
    command_t cmd = {0};
    cmd.type = CMD_MAG_SELFTEST;
    cmd.as.mag_selftest.write_sign_reg = (argc > 1) ? atoi(argv[1]) : 0;
    cmd.as.mag_selftest.read_loop_ms = (argc > 2) ? atoi(argv[2]) : 1000;
    if (cmd.as.mag_selftest.read_loop_ms < 0) cmd.as.mag_selftest.read_loop_ms = 0;
    if (cmd.as.mag_selftest.read_loop_ms > 5000) cmd.as.mag_selftest.read_loop_ms = 5000;
    if (!flight_core_push_command(&cmd)) { printf("loi: command queue day\n"); return 1; }
    printf("mag_test: da gui (sign=%d loop=%dms) -- xem cac dong '[mag_test ...]' trong log firmware. "
"Vong doc cuoi la quan trong nhat: ok%% cao = chip phat mau on dinh\n",
           (int)cmd.as.mag_selftest.write_sign_reg, (int)cmd.as.mag_selftest.read_loop_ms);
    return 0;
}

static int cmd_calib_erase(int argc, char **argv) {
    (void)argc; (void)argv;
    printf("!!! XOA TOAN BO calib gyro/accel/mag trong NVS -- se KHONG the ARM cho toi khi calib lai !!!\n");
    command_t cmd = {0};
    cmd.type = CMD_CALIB_ERASE;
    flight_core_push_command(&cmd);
    return 0;
}


// ================= tof_test — kiem tra ToF co THUC SU chay khong =================
// VI SAO CAN LENH RIENG: `status` chi cho anh chup MOT thoi diem. Cau hoi that
// su la "sensor co dang cap mau DEU khong, va so co on dinh khong" — phai lay
// mau lien tuc mot khoang thoi gian moi tra loi duoc.
//
// DOC TU SNAPSHOT cua sensor_hub, KHONG doc I2C truc tiep. Co chu dich: no test
// DUNG con duong ma vong dieu khien dung (driver -> hub -> snapshot). Chinh cho
// nay tung hong: driver init OK nhung hub KHONG he doc ToF, va doc I2C truc
// tiep se bao "OK" trong khi bay van khong co du lieu.
static int cmd_tof_test(int argc, char **argv) {
    const float sec = (argc >= 2) ? (float)atof(argv[1]) : 3.0f;
    const int duration_ms = (int)(sec * 1000.0f);
    if (duration_ms < 200 || duration_ms > 30000) {
        printf("tof_test: thoi gian phai trong 0.2..30s\n");
        return 1;
    }

    telemetry_snapshot_t t0;
    flight_core_read_telemetry(&t0);
    if (!t0.tof_ok_driver) {
        printf("tof_test: FAIL -- driver KHONG init duoc luc boot.\n");
        printf("  => xem log boot 'tof_driver' (MODEL_ID sai / khong ACK / XSHUT).\n");
        printf("  => chay 'i2c_scan' TRUOC: no tra loi dut diem chip co ACK khong.\n");
        return 1;
    }

    printf("tof_test: lay mau %.1fs (doc tu snapshot sensor_hub)...\n", (double)sec);

    int    n_new = 0, n_valid = 0, n_invalid = 0;
    float  vmin = 1e9f, vmax = -1e9f, vsum = 0.0f, vsum_sq = 0.0f;
    int    age_max = -1;
    int32_t last_age = -1;
    int    n_poll = 0;

    const int64_t t_end_us = esp_timer_get_time() + (int64_t)duration_ms * 1000;
    while (esp_timer_get_time() < t_end_us) {
        telemetry_snapshot_t t;
        flight_core_read_telemetry(&t);
        n_poll++;

        // Mau MOI nhan biet bang age GIAM (hub vua publish lai). Khong dung
        // tof_range_m doi gia tri: hai mau lien tiep hoan toan co the bang nhau.
        if (t.tof_age_ms >= 0 && (last_age < 0 || t.tof_age_ms < last_age)) {
            n_new++;
            if (t.tof_valid) {
                const float d = t.tof_range_m;
                n_valid++;
                if (d < vmin) vmin = d;
                if (d > vmax) vmax = d;
                vsum += d;
                vsum_sq += d * d;
            } else {
                n_invalid++;
            }
        }
        if (t.tof_age_ms > age_max) age_max = t.tof_age_ms;
        last_age = t.tof_age_ms;

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    const float hz = (float)n_new * 1000.0f / (float)duration_ms;
    printf("  mau moi   : %d  (~%.1f Hz)  poll=%d\n", n_new, (double)hz, n_poll);
    printf("  hop le    : %d   khong hop le: %d\n", n_valid, n_invalid);
    printf("  age lon nhat: %dms\n", age_max);
    // Watchdog treo (xem tof_driver.h). O phan cung lanh con so nay DUNG YEN.
    // Tang trong luc test = chip THAT SU ngung do va da bi da day lai.
#if SENSOR_TOF_ENABLED
    // tof_driver_stall_restarts() chi ton tai khi ToF duoc bien dich vao
    // firmware (xem fc_features.h). Khong gate o day = tat ToF trong
    // app_config.h se vo BUILD -- tuc la co mot cau hinh hop le ma khong build duoc.
    printf("  stall_restarts: %u  (phai DUNG YEN; tang = ToF treo that -> nghi nguon/nhieu)\n",
           (unsigned)tof_driver_stall_restarts());
#endif

    if (n_valid > 0) {
        const float mean = vsum / (float)n_valid;
        const float var = (vsum_sq / (float)n_valid) - (mean * mean);
        const float sd = (var > 0.0f) ? sqrtf(var) : 0.0f;
        printf("  khoang cach: min=%.3fm max=%.3fm mean=%.3fm std=%.4fm\n",
               (double)vmin, (double)vmax, (double)mean, (double)sd);
    }

    // ---- Ket luan ----
    if (n_new == 0) {
        printf("tof_test: FAIL -- driver OK nhung sensor_hub KHONG cap mau nao.\n");
        printf("  => hub khong doc ToF (tof_present=0 luc sensor_hub_start?).\n");
        return 1;
    }
    // Ky vong = nhip CHIP (1000/INTER_MEASUREMENT_MS), khong phai nhip poll cua
    // hub. Hub poll 16ms (SENSOR_TOF_DIVISOR=4) tuc NHANH HON chip, nen tran
    // thuc te la chip chu khong phai lich poll.
    if (hz < 10.0f) {
        printf("tof_test: FAIL -- nhip qua thap (%.1fHz, ky vong ~%dHz).\n",
               (double)hz, 1000 / BOARD_TOF_L1X_INTER_MEASUREMENT_MS);
        // KHONG doan "bus I2C nghen" nua: mot chu ky doc day du chi ton ~2.3ms
        // @100kHz (1 byte co ngat + 17 byte ket qua + 1 byte clear), tuc rieng
        // I2C cho phep toi ~430Hz. Nghen bus KHONG the keo nhip xuong 10Hz.
        printf("  => nghi theo thu tu: (1) cua so cam bien bi che/con mang bao ve\n");
        printf("     (do ra 1-4mm la dau hieu ro), (2) nguon 2.8V sut khi VCSEL ban,\n");
        printf("     (3) INTER_MEASUREMENT/timing budget dat sai trong board_config.h.\n");
        return 1;
    }
    if (n_valid == 0) {
        printf("tof_test: FAIL -- co mau nhung KHONG mau nao hop le.\n");
        printf("  => range_status luon bao loi: ngoai tam, be mat hap thu, nang chieu truc tiep.\n");
        printf("  => thu chia mot to giay trang cach sensor ~10cm.\n");
        return 1;
    }
    if (n_valid * 2 < n_new) {
        printf("tof_test: CANH BAO -- chi %d/%d mau hop le (<50%%).\n", n_valid, n_new);
        printf("  => ToF chay nhung khong on dinh; correction se bi ngat quang.\n");
        return 1;
    }

    printf("tof_test: OK -- ToF chay tot (%d/%d mau hop le, ~%.1fHz).\n",
           n_valid, n_new, (double)hz);
    printf("  Buoc tiep: dat drone tren san phang roi 'arm' -- luc do ground_ref\n");
    printf("  duoc chot. Xem 'status' dong tof_surface: ground_ref phai khac 0.000.\n");
    return 0;
}
// ================= i2c_scan — tren bus THUC SU co gi =================
// VI SAO CAN: khi mot driver bao "init that bai", co dung ba kha nang — chip
// khong co dien, chip noi sai chan, hoac chip o dia chi khac du kien. Ca ba
// deu hien ra y het nhau trong log driver. Quet bus tach duoc chung ra ngay
// lap tuc, va no la buoc DAU TIEN nen lam khi tof_test bao "driver khong init".
// cmd_tof_reinit() — chay lai bring-up ToF NGAY LUC GOI.
//
// VI SAO can lenh nay: tof_driver_init() chi chay trong ~1s dau sau reset.
// Console di qua USB-Serial-JTAG, ma reset chip = USB RE-ENUMERATE -> terminal
// rot va noi lai SAU khi app_main() xong, nen toan bo log init (ke ca dong
// "VL53L0X init failed line NNN" chi dung cho hong) da troi mat. Lenh nay in
// lai chuoi do luc terminal DANG cam, va cho phep sua day/nguon roi thu lai
// ngay ma khong can nap lai firmware.
static int cmd_tof_reinit(int argc, char **argv) {
    (void)argc; (void)argv;

    printf("tof_reinit: dang chay lai bring-up VL53L0X (co the mat ~1s)...\n");
    printf("  => doc cac dong log 'tof_driver' ngay ben duoi, KHONG phai dong ket qua o cuoi.\n");

    const int rc = flight_core_tof_reinit();

    if (rc == 0) {
        printf("tof_reinit: OK -- driver_ok=1, sensor_hub da bat dau doc ToF.\n");
        printf("  => chay 'tof_test 5' de kiem tra nhip mau + do on dinh.\n");
        return 0;
    }
    if (rc == -1) {
        printf("tof_reinit: ToF dang TAT trong app_config.h (SENSOR_TOF_ENABLED=0), "
"hoac bus I2C chua init.\n");
        return 1;
    }
    if (rc == -2) {
        printf("tof_reinit: khong muon duoc bus tu sensor_hub -> KHONG chay de len bus dang ban.\n");
        return 1;
    }
    if (rc == -3) {
        printf("tof_reinit: chi chay duoc khi DISARMED (bring-up giu bus toi ~1s).\n");
        return 1;
    }

    printf("tof_reinit: THAT BAI. Dong 'VL53L0X init failed line NNN' o tren la cho hong THAT:\n");
    printf("  line 464 (get_spad_info) / 503 / 505 (ref calibration) + ESP_ERR_TIMEOUT\n");
    printf("      => chip ACK I2C nhung loi analog khong chay: AVDD/2V8 yeu hoac thieu dong.\n");
    printf("  line 499 (timing budget) + ESP_ERR_INVALID_ARG/INVALID_STATE\n");
    printf("      => thanh ghi timeout doc ra rac, bang tuning khong 'an'.\n");
    printf("  bat ky line nao + loi I2C (TIMEOUT/INVALID_STATE tu bus)\n");
    printf("      => chip rot giua chung: nguon chap chon hoac SDA/SCL nhieu.\n");
    return 1;
}

static int cmd_i2c_scan(int argc, char **argv) {
    (void)argc; (void)argv;

    uint8_t found[16];
    const int n = flight_core_i2c_scan(found, (int)(sizeof(found) / sizeof(found[0])));

    if (n == -1) {
        printf("i2c_scan: bus I2C chua init (xem log boot 'I2C bus init that bai').\n");
        return 1;
    }
    if (n == -2) {
        printf("i2c_scan: khong muon duoc bus tu sensor_hub -> KHONG quet (ket qua se sai).\n");
        printf("  => thu lai; neu lap lai mai thi sensor_hub dang ket trong mot transaction.\n");
        return 1;
    }

    printf("i2c_scan: tim thay %d thiet bi tren SDA=GPIO%d SCL=GPIO%d @ %d Hz\n",
           n, BOARD_I2C_SDA_GPIO, BOARD_I2C_SCL_GPIO, (int)BOARD_I2C_FREQ_HZ);

    for (int i = 0; i < n; ++i) {
        const uint8_t a = found[i];
        const char *who = "?";
        if (a == BOARD_IMU_I2C_ADDR)       who = "MPU6050 (IMU)";
        else if (a == BOARD_MAG_I2C_ADDR)  who = "QMC5883P (mag)";
        else if (a == BOARD_BARO_I2C_ADDR) who = "BMP280 (baro)";
        // Ten chip lay tu driver dang build (BOARD_TOF_CHIP), khong hardcode:
        // in nham dong chip o day se dan nguoi debug di sai huong ngay tu buoc
        // dau tien. Ca hai dong VL53 deu mac dinh o 0x29.
        else if (a == BOARD_TOF_I2C_ADDR || a == 0x29) {
            static char tof_who[48];
            snprintf(tof_who, sizeof(tof_who), "%s ToF (huong xuong%s)",
                     tof_driver_chip_name(),
                     (a == 0x29 && BOARD_TOF_I2C_ADDR != 0x29) ? ", dia chi MAC DINH" : "");
            who = tof_who;
        }
        printf("  0x%02X  %s\n", a, who);
    }

    // Ket luan RIENG cho ToF, vi do la cai dang hong.
    bool tof_seen = false;
    for (int i = 0; i < n; ++i) {
        if (found[i] == 0x29 || found[i] == BOARD_TOF_I2C_ADDR) tof_seen = true;
    }
    if (!tof_seen) {
        printf("i2c_scan: KHONG thay ToF o 0x29 hay 0x%02X.\n", BOARD_TOF_I2C_ADDR);
        printf("  => chip khong co dien hoac khong noi vao dung bus.\n");
        printf("  => kiem tra VIN/GND, roi den SDA=GPIO%d / SCL=GPIO%d.\n",
               BOARD_I2C_SDA_GPIO, BOARD_I2C_SCL_GPIO);
        printf("  => neu day XSHUT (GPIO%d) co noi: rut dien HAN TOAN roi cap lai,\n",
               BOARD_TOF_XSHUT_GPIO);
        printf("     vi dia chi ToF nam trong RAM chip, RESET ESP32 khong xoa no.\n");
    } else {
        printf("i2c_scan: ToF CO tra loi ACK -> day va nguon OK; loi (neu con) nam o init ranging.\n");
    }
    return 0;
}

// ================= imu_int_test — IMU co THUC SU chay theo ngat khong =================
// VI SAO KHONG TIN CO imu_int_active MOT MINH: co do chi noi "da ghi register
// INT_ENABLE + cai ISR thanh cong". No van bang 1 khi day INT dut, cho toi khi
// du 25 lan cho qua han lien tiep. Cau hoi that su la "xung ngat co ve DEU o
// dung 250Hz khong" -- chi do duoc bang cach lay hai moc dem cach nhau mot
// khoang thoi gian, dung cach `tof_test` do nhip ToF.
static int cmd_imu_int_test(int argc, char **argv) {
    const float sec = (argc >= 2) ? (float)atof(argv[1]) : 2.0f;
    const int duration_ms = (int)(sec * 1000.0f);
    if (duration_ms < 200 || duration_ms > 30000) {
        printf("imu_int_test: thoi gian phai trong 0.2..30s\n");
        return 1;
    }

    telemetry_snapshot_t a, b;
    flight_core_read_telemetry(&a);
    const int64_t t0 = esp_timer_get_time();
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    flight_core_read_telemetry(&b);
    const int64_t t1 = esp_timer_get_time();

    const float dt_s = (float)(t1 - t0) * 1e-6f;
    const float isr_hz  = (float)(b.imu_int_isr_count  - a.imu_int_isr_count)  / dt_s;
    const float wake_hz = (float)(b.imu_int_wake_count - a.imu_int_wake_count) / dt_s;
    const uint32_t timeouts = b.imu_int_timeout_count - a.imu_int_timeout_count;

    printf("imu_int_test: do trong %.2fs (INT = GPIO%d)\n", (double)dt_s, BOARD_MPU_INT_GPIO);
    printf("  ngat ISR  : %.1f Hz   (ky vong ~%d Hz = SMPLRT cua MPU6050)\n",
           (double)isr_hz, IMU_SAMPLE_RATE_HZ);
    printf("  hub thuc day: %.1f Hz\n", (double)wake_hz);
    printf("  cho qua han: %u lan\n", (unsigned)timeouts);
    printf("  imu_int_active=%d\n", (int)b.imu_int_active);

    if (!b.imu_int_active) {
        printf("imu_int_test: FAIL -- hub DANG chay bang dong ho FreeRTOS, KHONG theo ngat.\n");
        if (isr_hz < 1.0f) {
            printf("  => khong co xung nao tren GPIO%d: day INT chua han, hoac sai chan.\n",
                   BOARD_MPU_INT_GPIO);
            printf("  => cung co the IMU init that bai (xem log boot 'imu_driver').\n");
        } else {
            printf("  => co xung nhung hub da tu roi ve polling truoc do; reset de thu lai.\n");
        }
        return 1;
    }
    if (isr_hz < 1.0f) {
        printf("imu_int_test: FAIL -- ngat da bat nhung KHONG co xung nao den.\n");
        printf("  => day INT dut/chua han, hoac GPIO%d bi peripheral khac chiem.\n",
               BOARD_MPU_INT_GPIO);
        return 1;
    }
    // +-20%: du rong de bo qua sai so lay mau, du chat de bat truong hop
    // SMPLRT_DIV sai (se ra 1000Hz) hay ngat gia do chan tha noi (ra bat ky so nao).
    if (isr_hz < IMU_SAMPLE_RATE_HZ * 0.8f || isr_hz > IMU_SAMPLE_RATE_HZ * 1.2f) {
        printf("imu_int_test: FAIL -- nhip ngat %.1fHz lech xa %dHz.\n",
               (double)isr_hz, IMU_SAMPLE_RATE_HZ);
        printf("  => cao gap ~4 lan: SMPLRT_DIV khong duoc ghi (chip chay 1kHz).\n");
        printf("  => so la lung tung: chan INT tha noi -> ngat gia.\n");
        return 1;
    }
    if (timeouts > 0) {
        printf("imu_int_test: CANH BAO -- %u lan cho qua han trong %.1fs (ngat co luc mat).\n",
               (unsigned)timeouts, (double)dt_s);
        printf("  => tiep xuc day INT chap chon, hoac nhieu tu dong co.\n");
        return 1;
    }

    printf("imu_int_test: OK -- IMU dang lam DONG HO cho ca vong dieu khien (%.1fHz, 0 truot).\n",
           (double)isr_hz);
    return 0;
}

static void print_stack_row(const char *name, int core, const char *prio,
                            uint32_t total_bytes, uint32_t free_bytes) {
    if (free_bytes == 0) {
        // free==0 = task chua duoc tao (getter tra 0), KHONG phai "da tran het".
        printf("  %-10s core=%-2d prio=%-3s %6u B   (task chua chay)\n",
               name, core, prio, (unsigned)total_bytes);
        return;
    }
    if (total_bytes == 0) {
        // Khong biet tong (stack do IDF tu quyet dinh) -- chi in phan con trong.
        printf("  %-10s core=%-2d prio=%-3s      ?      %25s con %5u B\n",
               name, core, prio, "", (unsigned)free_bytes);
        return;
    }
    const uint32_t used = (free_bytes < total_bytes) ? (total_bytes - free_bytes) : total_bytes;
    const int used_pct = (int)((used * 100u) / total_bytes);
    // Nguong canh bao: duoi 512B la mong, duoi 256B la sap tran that.
    const char *flag = (free_bytes < 256u) ? "  <<< NGUY HIEM"
                     : (free_bytes < 512u) ? "  <<< sat nguong" : "";
    printf("  %-10s core=%-2d prio=%-3s %6u B   dung %5u B (%2d%%)  con %5u B%s\n",
           name, core, prio, (unsigned)total_bytes, (unsigned)used, used_pct,
           (unsigned)free_bytes, flag);
}

// `tasks` -- CHI DOC, an toan ca khi dang bay. Khong dung vao control path,
// khong lay mutex nao, chay tren core 0 (task console).
// uxTaskGetStackHighWaterMark() chi quet vung stack cua task khac de dem byte
// 0xA5 chua bi cham -> khong ghi gi, khong dung lich, khong chan task do.
static int cmd_tasks(int argc, char **argv) {
    (void)argc; (void)argv;

    flight_core_task_stats_t st;
    flight_core_get_task_stats(&st);

    telemetry_snapshot_t t;
    flight_core_read_telemetry(&t);

    char prio_buf[2][8];
    snprintf(prio_buf[0], sizeof(prio_buf[0]), "%u", (unsigned)st.stabilize_priority);
    snprintf(prio_buf[1], sizeof(prio_buf[1]), "%u", (unsigned)st.sensor_hub_priority);

    printf("=== STACK: cho trong THAP NHAT tung do duoc, tinh bang BYTE ===\n");
    printf("  (ESP-IDF tra BYTE chu khong phai word vi portSTACK_TYPE=uint8_t,\n");
    printf("   nen so nay so sanh TRUC TIEP duoc voi so khai bao luc tao task)\n");
    print_stack_row("stabilize", st.stabilize_core, prio_buf[0],
                    st.stabilize_stack_total_bytes, st.stabilize_stack_free_bytes);
    print_stack_row("sensor_hub", st.sensor_hub_core, prio_buf[1],
                    st.sensor_hub_stack_total_bytes, st.sensor_hub_stack_free_bytes);
    print_stack_row("net", APP_TASK_CORE, "5", NET_TASK_STACK_BYTES,
                    (s_net_task != NULL) ? (uint32_t)uxTaskGetStackHighWaterMark(s_net_task) : 0u);
    print_stack_row("udp_rx", APP_TASK_CORE, "3",
                    net_link_udp_rx_stack_total_bytes(),
                    net_link_udp_rx_stack_free_bytes());
    // Lenh nay chay TRONG chinh task console -> uxTaskGetStackHighWaterMark(NULL)
    // la watermark cua no, khong can luu handle rieng. Tong stack do
    // ESP_CONSOLE_REPL_CONFIG_DEFAULT() quyet dinh (khong set tay) nen bao 0.
    print_stack_row("console", APP_TASK_CORE, "-", 0u,
                    (uint32_t)uxTaskGetStackHighWaterMark(NULL));

    printf("=== NHIP VONG BAY ===\n");
    // loop_dt phai bam quanh 1000/IMU_SAMPLE_RATE_HZ. deadline_miss TANG DEU =
    // vong bay khong kip han -> xem lai priority/core/stack, khong phai tune PID.
    printf("  loop_dt=%.3f ms (ky vong %.3f ms @ %d Hz)  deadline_miss=%u\n",
           t.loop_dt_ms, 1000.0 / (double)IMU_SAMPLE_RATE_HZ, IMU_SAMPLE_RATE_HZ,
           (unsigned)t.deadline_miss_count);
    printf("  imu_int: active=%d isr=%u wake=%u timeout=%u no_new_sample=%u\n",
           (int)t.imu_int_active, (unsigned)t.imu_int_isr_count,
           (unsigned)t.imu_int_wake_count, (unsigned)t.imu_int_timeout_count,
           (unsigned)t.imu_no_new_sample_count);
    printf("  heap free=%u B (thap nhat tung thay=%u B)\n",
           (unsigned)esp_get_free_heap_size(), (unsigned)esp_get_minimum_free_heap_size());
    return 0;
}

static void register_commands(void) {
    esp_console_register_help_command();

    const esp_console_cmd_t cmds[] = {
        { .command = "status", .help = "In telemetry hien tai (state/attitude/alt/motor/battery)", .func = cmd_status },
        { .command = "tasks", .help = "Do stack con trong (BYTE) tung task + nhip vong bay + heap -- CHI DOC, an toan khi dang bay", .func = cmd_tasks },
        { .command = "tof_test", .help = "tof_test [sec] -- kiem tra ToF co cap mau deu + on dinh khong", .func = cmd_tof_test },
        { .command = "i2c_scan", .help = "Quet bus I2C 0x08..0x77 -- buoc DAU TIEN khi mot cam bien khong init duoc", .func = cmd_i2c_scan },
        { .command = "tof_reinit", .help = "Chay lai bring-up ToF NGAY (de doc log init ma khong can reboot)", .func = cmd_tof_reinit },
        { .command = "imu_int_test", .help = "imu_int_test [sec] -- do nhip ngat data-ready MPU6050 (vong dieu khien co chay theo ngat that khong)", .func = cmd_imu_int_test },
        { .command = "arm", .help = "Arm dong co (can attitude valid + khong nghieng qua nguong)", .func = cmd_arm },
        { .command = "disarm", .help = "Disarm ngay lap tuc", .func = cmd_disarm },
        { .command = "takeoff", .help = "takeoff <mm> [timeout_s] [climb_timeout_s] -- blocking", .func = cmd_takeoff },
        { .command = "land", .help = "land [timeout_s] -- blocking toi khi DISARMED", .func = cmd_land },
        { .command = "hover", .help = "hover <sec> -- giu nguyen vi tri trong sec giay", .func = cmd_hover },
        { .command = "move", .help = "move <dir> <pct> <sec> -- dir=forward/back/left/right/up/down/cw/ccw", .func = cmd_move },
        { .command = "yaw", .help = "yaw <deg> -- xoay tuong doi (open-loop theo thoi gian)", .func = cmd_yaw },
        { .command = "alt", .help = "alt <mm> -- doi altitude target (co geofence clamp)", .func = cmd_alt },
        { .command = "heartbeat", .help = "Nuoi watchdog Commander (BAT BUOC moi <1s khi bay bang console USB)", .func = cmd_heartbeat },
        { .command = "test_motor", .help = "test_motor <1-4> <duty_pct> -- CHI khi DISARMED, THAO CANH QUAT truoc", .func = cmd_test_motor },
        { .command = "test_motor_all", .help = "test_motor_all <duty_pct> -- quay CA 4 dong co cung luc, dong duty (sanity-check, KHONG dung de xac nhan vi tri/chieu) -- CHI khi DISARMED, THAO CANH QUAT truoc", .func = cmd_test_motor_all },
        { .command = "bench_start", .help = "Vao che do bench-test tang ga tay de tune PID -- CHI tu ARMED, drone PHAI duoc giu chat/kep tren gia do", .func = cmd_bench_start },
        { .command = "+", .help = "Tang throttle bench-test len 1 nac (mac dinh +20 duty) -- CHI khi dang bench-test (bench_start truoc)", .func = cmd_bench_throttle_up },
        { .command = "-", .help = "Giam throttle bench-test 1 nac (mac dinh -20 duty) -- CHI khi dang bench-test (bench_start truoc)", .func = cmd_bench_throttle_down },
        { .command = "bench_stop", .help = "Cat throttle bench-test NGAY, ve ARMED (khong latch, lap lai duoc)", .func = cmd_bench_stop },
        { .command = "calib_gyro", .help = "Fresh gyro calib: settle+RAW collect+independent validate (~7s stationary) -- DISARMED", .func = cmd_calib_gyro },
        { .command = "calib_gyro_abort", .help = "Huy phien calib_gyro dang do (khong luu)", .func = cmd_calib_gyro_abort },
        { .command = "cal_status", .help = "Xem GRAW/GBIAS/GCORR/GSTD + config MPU doc lai -- kiem tra calib co an khong", .func = cmd_cal_status },
        { .command = "calib_accel_face", .help = "Bat 1 mat trong quy trinh accel 6-face -- goi 6 lan, doi huong moi lan", .func = cmd_calib_accel_face },
        { .command = "calib_accel_reset", .help = "Huy tien trinh accel 6-face dang do, lam lai tu dau", .func = cmd_calib_accel_reset },
        { .command = "calib_mag_start", .help = "Bat dau thu mau mag, TU DONG chay 60s -- xoay hinh so 8 suot thoi gian nay", .func = cmd_calib_mag_start },
        { .command = "calib_mag_stop", .help = "Ket thuc SOM (truoc 60s) + tinh hard/soft-iron + luu NVS ngay", .func = cmd_calib_mag_stop },
        { .command = "calib_mag_abort", .help = "Huy phien calib_mag dang thu (khong tinh/luu)", .func = cmd_calib_mag_abort },
        { .command = "calib_baro_ground", .help = "Lay lai moc 0m + std-dev baro NGAY LUC GOI -- khuyen nghi ngay truoc arm", .func = cmd_calib_baro_ground },
        { .command = "calib_erase", .help = "XOA TOAN BO calib NVS (gyro+accel+mag) -- bench/test only", .func = cmd_calib_erase },
        { .command = "mag_test", .help = "mag_test [sign 0|1] [loop_ms] -- do QMC5883P tung buoc (CHI khi DISARMED)", .func = cmd_mag_test },
    };
    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "UAV-S3 flight control firmware (console USB D+/D-) boot");

#if !MOTOR_POSITIONS_CONFIRMED
    ESP_LOGW(TAG, "!!! app_config.h: MOTOR_POSITIONS_CONFIRMED=0 -- vi tri vat ly CH1..CH4 CHUA "
"xac nhan. THAO HET CANH QUAT roi dung lenh 'test_motor <1-4> <pct>' de xac "
"nhan TRUOC khi lap canh/bay that (xem app_config.h de biet quy trinh day du).");
#endif

    // Cấu hình phần cứng dựng ở MỘT chỗ duy nhất (main/board_setup.h) và dùng
    // chung với fc_bridge_init() — trước đây 20 dòng gán này bị chép ra hai bản.
    flight_core_board_config_t cfg;
    board_config_fill(&cfg);

    esp_err_t err = flight_core_start(&cfg);
    ESP_LOGI(TAG, "flight_core_start() -> %s", esp_err_to_name(err));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "flight_core_start() THAT BAI -- KHONG the dieu khien duoc (kiem tra "
"motor_gpio trong board_config.h, day dan dong co)");
        return;
    }

    {
        telemetry_snapshot_t t0;
        flight_core_read_telemetry(&t0);
        if (t0.uncalibrated) {
            // BAT BUOC de bay: CHI gyro + accel. Mag KHONG con bat buoc (che do
            // bay dung yaw-RATE — xem recompute_uncalibrated() trong flight_core.c).
            ESP_LOGW(TAG, "!!! CHUA CALIBRATE (thieu accel hop le trong NVS) -- lenh 'arm' se bi TU CHOI. "
"Go 'calib_accel_face' 6 lan + 'calib_gyro' truoc khi bay (xem 'help'). "
"Mag KHONG bat buoc.");
        }
    }

    // Stack tính bằng BYTE (xTaskCreate* của ESP-IDF nhận byte, KHÔNG phải
    // word như FreeRTOS vanilla) -> 4096 = 4KB. PIN CORE 0: core 1 dành riêng
    // cho vòng bay, xem APP_TASK_CORE.
    // Stack 6144 byte (khong phai 4096): net_link_init() goi ca chuoi
    // esp_netif_init() -> tcpip_init() -> lwip_init() + esp_wifi_init(),
    // deu an stack sau. Hai buffer STATUS 1536 byte da duoc chuyen sang
    // static (xem net_task) nen khong con nam o day, nhung van de bien
    // rong: tran stack o task nay bieu hien thanh panic o cho khac han
    // (ISR GPIO nhay vao con tro rac), rat kho lan nguoc.
    xTaskCreatePinnedToCore(net_task, "net", NET_TASK_STACK_BYTES, NULL, 5, &s_net_task, APP_TASK_CORE);

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "uav>";
    repl_config.max_cmdline_length = 256;
    // Mặc định của IDF là tskNO_AFFINITY -> REPL task được phép chạy trên core
    // 1. Nó BLOCK trong đọc console phần lớn thời gian, nhưng handler lệnh thì
    // không (vd `calib_accel_face` chạy hàng trăm ms, `tof_test` chạy vài giây)
    // — đúng loại tải không được phép rơi vào core của vòng bay.
    repl_config.task_core_id = APP_TASK_CORE;

    esp_console_dev_usb_serial_jtag_config_t hw_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl));

    register_commands();

    ESP_LOGI(TAG, "console san sang -- go 'help' de xem danh sach lenh (arm/takeoff/land/...)");
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
