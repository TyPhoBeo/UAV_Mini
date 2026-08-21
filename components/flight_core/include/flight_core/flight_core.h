// flight_core — API CÔNG KHAI DUY NHẤT của TẦNG DƯỚI. Đây là toàn bộ bề mặt
// mà micropython_module/fc (TẦNG TRÊN) được phép chạm vào.
//
// BIÊN GIỚI CHỈ GỒM 2 KÊNH (không con trỏ chung vào control state):
//   XUỐNG : flight_core_push_command()  — đẩy command_t vào FreeRTOS queue.
//   LÊN   : flight_core_read_telemetry()— đọc bản sao telemetry_snapshot_t.
// CHỈ stabilize task (bên trong flight_core.c, core 1) ghi control state và
// telemetry snapshot -> 1 writer, không race, không cần khóa nặng (mutex nhẹ
// chỉ để tránh torn-read khi copy struct, không phải để đồng bộ logic).
//
// flight_core KHÔNG include gì từ main/ hay micropython_module/ — component
// độc lập, build/test riêng được (xem README + platformio.ini ở gốc repo).
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "flight_core/alt_hold.h"
#include "flight_core/attitude_control.h"
#include "flight_core/command.h"
#include "flight_core/commander.h"
#include "flight_core/mahony_filter.h"
#include "flight_core/takeoff_land.h"
#include "flight_core/telemetry.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // I2C dùng chung cho IMU + mag (nếu có) + ToF + baro.
    int      i2c_sda_gpio;
    int      i2c_scl_gpio;
    int      i2c_freq_hz;
    uint8_t  imu_addr;   // IMU BẮT BUỘC — không có cờ enable (xem SENSOR_IMU_ENABLED trong app_config.h)

    // Chân DATA_RDY interrupt của MPU6050. >=0 -> stabilize_task chạy theo
    // NHỊP CẢM BIẾN (ngủ chờ ngắt) thay vì tự hẹn giờ bằng đồng hồ FreeRTOS;
    // <0 -> polling như cũ. Cấu hình ngắt thất bại KHÔNG làm flight_core_start()
    // fail — chỉ cảnh báo rồi chạy tiếp bằng polling (giảm chất lượng, không
    // mất an toàn). Xem imu_driver_enable_data_ready_int().
    int      imu_int_gpio;

    // Cờ mag_enabled/tof_enabled/baro_enabled/battery_enabled
    // ánh xạ 1-1 từ SENSOR_*_ENABLED trong main/app_config.h (caller —
    // fc_bridge.c/src/main.c — set các cờ này TRƯỚC khi gọi flight_core_start()).
    // =false -> flight_core BỎ QUA HẲN việc init driver tương ứng, KHÔNG thử
    // dò I2C, coi như "chưa có cảm biến" vĩnh viễn (an toàn mặc định, giống
    // hệt hành vi khi init thật sự thất bại).
    bool     mag_enabled;
    uint8_t  mag_addr;

    // ToF (VL53L0X, MỘT con, hướng xuống, dùng chung bus I2C ở trên) — nguồn
    // CORRECTION có điều kiện cho alt_estimator, KHÔNG phải nguồn chính (xem
    // tof_driver.h + alt_estimator.h).
    //
    // ĐÃ BỎ tof2_* (con thứ hai forward/dự phòng): nó chưa bao giờ được fuse
    // vào control loop nhưng bắt driver phải mang những ràng buộc chỉ có nghĩa
    // khi có 2 chip — và chính những ràng buộc đó đã gây lỗi thật cho cấu hình
    // 1 chip. Xem tof_driver.h mục "ĐÃ ĐƠN GIẢN HOÁ TỪ 2 SENSOR VỀ 1".
    //
    // tof_xshut_gpio < 0 = KHÔNG NỐI, hoàn toàn hợp lệ (module tự pull-up).
    bool     tof_enabled;
    uint8_t  tof_addr;
    int      tof_xshut_gpio;
    // Tốc độ SCL RIÊNG cho ToF. 0 = dùng chung i2c_freq_hz (mặc định cũ, nên
    // caller không set field này vẫn chạy y như trước).
    //
    // Tốc độ là thuộc tính của DEVICE trong API I2C master mới của ESP-IDF, nên
    // ToF chạy 100kHz trong khi IMU/mag/baro chạy 400kHz trên CÙNG bus là hợp
    // lệ. Lý do cần: ToF thường nối bằng dây rời ra mép khung (điện dung lớn,
    // gần dây motor) trong khi các chip kia nằm sẵn trên PCB — xem
    // BOARD_TOF_I2C_FREQ_HZ trong board_config.h.
    uint32_t tof_freq_hz;

    // Baro (BMP280) — sửa trôi dài hạn cho alt_estimator, KHÔNG phải nguồn
    // chính (xem baro_driver.h + alt_estimator.h).
    bool     baro_enabled;
    uint8_t  baro_addr;

    // Thứ tự M1..M4 khớp mixer Quad-X (attitude_control.h).
    int motor_gpio[4];

    // Battery ADC — PHẢI dùng ADC1 (ADC2 xung đột WiFi trên S3).
    // KHÔNG có divider_ratio ở đây: chia áp là hằng số phần cứng cố định,
    // định nghĩa tại battery_driver.h (BATTERY_DIVIDER_RATIO, suy ra từ
    // R_TOP/R_BOTTOM) — truyền qua nhiều tầng chỉ tạo thêm chỗ gán nhầm.
    bool  battery_enabled;
    int   battery_adc1_channel;
} flight_core_board_config_t;

// flight_core_start() — gọi MỘT LẦN từ main/main.c (sau khi NVS/PSRAM init).
// Init driver (lỗi driver KHÔNG chặn boot — log cảnh báo rồi chạy tiếp ở trạng
// thái "chưa có cảm biến", an toàn vì fsm_arm_guard_ok() cần attitude_valid;
// chưa cắm IMU thì không bao giờ ARM được, đúng ý muốn). Spawn stabilize task
// 250Hz PIN CORE 1, tách hẳn khỏi MicroPython VM (chạy core 0).
esp_err_t flight_core_start(const flight_core_board_config_t *board_cfg);

// flight_core_push_command() — thread-safe, gọi từ bất kỳ core/task nào (bridge
// MicroPython gọi từ core 0). Trả false nếu queue đầy (COMMAND_QUEUE_DEPTH) —
// caller (fc_bridge.c) PHẢI coi là lỗi, không được nuốt lệnh.
bool flight_core_push_command(const command_t *cmd);

// flight_core_kill_now() — ĐƯỜNG KILL ƯU TIÊN CAO NHẤT, BYPASS command queue.
//
// Khác flight_core_push_command(CMD_KILL): hàm này KHÔNG dùng queue nên KHÔNG
// BAO GIỜ fail vì queue đầy, và cắt motor NGAY trong ngữ cảnh caller (không
// đợi stabilize_task tới tick sau — nếu task đó đang trễ/treo thì "đợi tick
// sau" nghĩa là motor tiếp tục quay vô thời hạn).
//
// Tác dụng: đóng armed gate phần cứng (motor_driver_all_off()) + bật kill
// latch. Sau đó KHÔNG đường code nào ghi được duty khác 0 cho tới khi có
// CMD_ARM tường minh — latch KHÔNG tự hạ theo thời gian hay khi fault hết.
//
// Thread-safe, gọi được từ bất kỳ task/core nào (net_task, MicroPython bridge,
// console). An toàn gọi lại nhiều lần (idempotent).
//
// FSM vẫn được ép về DISARMED, nhưng ở tick kế tiếp của stabilize_task —
// motor thì đã cắt ngay lập tức, không chờ.
void flight_core_kill_now(const char *reason);

// flight_core_read_telemetry() — thread-safe, không block lâu (mutex giữ rất
// ngắn, chỉ để memcpy). An toàn gọi tần suất cao từ Python polling loop.
void flight_core_read_telemetry(telemetry_snapshot_t *out);

// ================= Live-tuning getters (ground-station GET, xem command_parser.c) =================
// Đọc bản COPY dưới mutex nhẹ (s_tuning_mtx trong flight_core.c) — CHỈ
// stabilize_task ghi (khi xử lý CMD_SET_PID/CMD_SET_MAHONY/CMD_SET_TRIM/
// CMD_SET_ALT_TUNE/CMD_SET_TKO_TUNE/CMD_SET_LAND_TUNE/CMD_SET_ATTITUDE/
// CMD_SET_COMMANDER_CFG trong apply_command()), net_link/command_parser task
// chỉ ĐỌC — cùng nguyên tắc 1 writer như flight_core_read_telemetry(). An
// toàn gọi từ bất kỳ task nào.
void flight_core_get_attitude_gains(attitude_gains_t *out);
void flight_core_get_mahony_config(mahony_config_t *out);
void flight_core_get_trim(float *roll_deg, float *pitch_deg);
void flight_core_get_setpoint(float *roll_deg, float *pitch_deg, float *yaw_rate_dps);
void flight_core_get_alt_hold_tune(alt_hold_tune_t *out);
void flight_core_get_takeoff_tune(takeoff_tune_t *out);
void flight_core_get_landing_tune(landing_tune_t *out);

// flight_core_get_commander_cfg() — geofence + ngưỡng fault ĐANG áp dụng (xem
// commander.h + CMD_SET_COMMANDER_CFG). Cùng nguyên tắc 1 writer/mutex nhẹ
// như các getter trên — ghi CHỈ qua apply_command() (stabilize_task), đọc từ
// bất kỳ task nào.
void flight_core_get_commander_cfg(commander_config_t *out);

// flight_core_get_control_input() — đọc lại rol/pit/yaw/thr (-100..100) từ
// CMD_CONTROL gần nhất (xem fc.get_states() ở micropython_module). Cùng
// nguyên tắc 1 writer/mutex nhẹ như các getter trên. Trả 0 hết nếu chưa từng
// có CMD_CONTROL hoặc đã stale (xem SP_STALE_TIMEOUT_US).
void flight_core_get_control_input(float *rol_pct, float *pit_pct, float *yaw_pct, float *thr_pct);

// ================= Chẩn đoán bus I2C =================
// flight_core_i2c_scan() — quét toàn bộ dải địa chỉ 7-bit hợp lệ (0x08..0x77)
// và ghi các địa chỉ CÓ ACK vào found[].
//
// VÌ SAO cần: khi một driver báo "init that bai", có đúng ba khả năng — chip
// không có điện, chip nối sai chân, hoặc chip ở địa chỉ khác dự kiến. Không có
// lệnh quét thì cả ba đều hiện ra y hệt nhau và chỉ còn cách đoán. Quét trả lời
// dứt điểm câu hỏi "trên bus THẬT SỰ có gì".
//
// Bus được MƯỢN khỏi sensor_hub trong lúc quét (sensor_hub_suspend), vì quét
// ~112 transaction xen vào giữa vòng đọc cảm biến sẽ làm nhịp IMU trượt.
//
// Trả: số địa chỉ tìm được (>=0), -1 nếu tham số sai/bus chưa init,
//      -2 nếu không mượn được bus (hub không nhả kịp).
int flight_core_i2c_scan(uint8_t *found, int max_found);

// flight_core_tof_reinit() — CHẠY LẠI toàn bộ bring-up ToF, ngay lúc gọi.
//
// VÌ SAO cần: `tof_driver_init()` chỉ chạy MỘT LẦN trong flight_core_start(),
// tức là trong ~1 giây đầu sau reset. Trên board này console đi qua
// USB-Serial-JTAG: reset chip = USB device RE-ENUMERATE, nên terminal bị rớt
// và nối lại SAU khi app_main() đã chạy xong — toàn bộ log init (kể cả dòng
// "VL53L0X init failed line NNN" chỉ đúng chỗ hỏng) đã trôi mất. Không có lệnh
// này thì cách duy nhất để đọc log init là canh đúng thời điểm cắm lại cổng,
// và đó không phải một quy trình chẩn đoán.
//
// Nó cũng cho phép LẶP: sửa dây/nguồn rồi gọi lại ngay, không cần nạp lại
// firmware hay reboot.
//
// CHỈ chạy khi DISARMED. Bring-up ToF giữ bus và có thể block tới ~1s (2 lần
// ref calibration, mỗi lần timeout 500ms) — chạy lúc đang bay sẽ làm snapshot
// đứng và Commander trip hard fault.
//
// Trả:  0 = thành công (driver_ok chuyển sang 1, hub bắt đầu đọc ToF)
//      -1 = ToF bị TẮT trong app_config.h, hoặc bus chưa init
//      -2 = không mượn được bus khỏi sensor_hub
//      -3 = không phải DISARMED
//      >0 = mã esp_err_t của lần init hỏng (xem log để biết line + lý do)
int flight_core_tof_reinit(void);

// ================= Chẩn đoán RTOS (stack thực đo) =================
// flight_core_task_stats_t — chỗ trống CÒN LẠI ít nhất từng đo được trên stack
// của 2 task core-1, tính bằng BYTE.
//
// VÌ SAO cần: stack size khai báo lúc xTaskCreate*() chỉ là con số ta ĐOÁN.
// Không đo thì có đúng hai kiểu sai và cả hai đều tốn: cấp thừa (phí DRAM) hoặc
// cấp thiếu (tràn stack -> ghi đè vùng nhớ kề bên -> crash ở chỗ KHÔNG liên
// quan gì tới task thủ phạm, đúng loại lỗi đã làm firmware này boot-loop một
// lần rồi). Canary của FreeRTOS chỉ kiểm lúc chuyển ngữ cảnh nên bắt được rất
// muộn; high-water-mark cho biết TRƯỚC khi tràn còn cách bao xa.
//
// ĐƠN VỊ: BYTE — xem giải thích đầy đủ ở sensor_hub_stack_free_bytes().
// free = 0 nghĩa là task chưa tồn tại (chưa gọi flight_core_start()).
//
// Đọc được từ BẤT KỲ task nào, không mutex: uxTaskGetStackHighWaterMark() chỉ
// quét vùng stack của task khác để đếm byte 0xA5 chưa bị chạm, không ghi gì.
// core/priority đi kèm luôn trong struct này thay vì để bên gọi tự viết lại
// hằng số: STABILIZE_TASK_PRIORITY/CORE là #define RIÊNG TƯ trong flight_core.c
// và đang được _Static_assert khoá. Nếu console tự chép "23"/"1" vào lệnh in
// thì lần nào đó đổi priority sẽ có một bảng chẩn đoán in ra số cũ — tức là
// công cụ đo nói dối, đúng thứ nguy hiểm nhất ở một công cụ đo.
typedef struct {
    uint32_t stabilize_stack_total_bytes;
    uint32_t stabilize_stack_free_bytes;
    uint8_t  stabilize_core;
    uint8_t  stabilize_priority;
    uint32_t sensor_hub_stack_total_bytes;
    uint32_t sensor_hub_stack_free_bytes;
    uint8_t  sensor_hub_core;
    uint8_t  sensor_hub_priority;
} flight_core_task_stats_t;

void flight_core_get_task_stats(flight_core_task_stats_t *out);

#ifdef __cplusplus
}
#endif
