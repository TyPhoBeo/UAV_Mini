// sensor_hub — TẦNG THU MẪU CẢM BIẾN, tách hẳn khỏi vòng điều khiển.
//
// ============================================================================
// VẤN ĐỀ NÓ SINH RA ĐỂ GIẢI QUYẾT
// ============================================================================
// Trước đây stabilize_task (250Hz, ngân sách ~4ms/tick) tự gọi
// imu_driver_read()/mag_driver_read()/baro_driver_read() TRỰC TIẾP. Mọi
// transaction I2C trong các driver đó có timeout 50ms. Một con cảm biến treo
// hoặc SDA/SCL bị kéo thấp = stabilize_task đứng 50ms = MẤT 12 CHU KỲ ĐIỀU
// KHIỂN LIÊN TIẾP, trong lúc motor vẫn giữ nguyên duty của lệnh cuối cùng. Ba
// cảm biến cùng lỗi = 150ms. Đó là kiểu hỏng tệ nhất: không crash, không báo
// lỗi, chỉ đơn giản là mất điều khiển trong lúc bay.
//
// GIỜ có HAI lớp bảo vệ, không phải một:
//   1. Tách task — bus treo thì HUB bị chặn, không phải vòng bay.
//   2. Timeout RUNTIME riêng (8ms) tách khỏi timeout INIT (50ms) — xem khối
//      "TIMEOUT I2C" ở đầu imu_driver.c. Kể cả hub cũng không còn đứng 50ms
//      cho một transaction lẽ ra chỉ mất ~0.4ms.
//
// ============================================================================
// KIẾN TRÚC
// ============================================================================
//   MPU6050 INT (IO36) ──ISR──> sensor_task (core 1, prio 22)
//                                    │  SỞ HỮU DUY NHẤT bus I2C
//                                    │  đọc IMU mỗi lần có ngắt
//                                    │  publish snapshot IMU
//                                    │  xTaskNotifyGive(stabilize) ──> PREEMPT NGAY
//                                    ▼                                     │
//                            sensor_snapshot_t (mutex, 1 writer)           │
//                                    │  value + seq + timestamp + valid    │
//                                    ▼                                     ▼
//                            stabilize_task (core 1, prio 23) — CHỈ COPY, KHÔNG I2C
//                                    │  Mahony -> Az/Vz/Z -> safety/FSM -> PID -> mixer -> motor
//                                    ▼
//                            stabilize BLOCK lại
//                                    │
//                            hub chạy tiếp MAG/BARO/ToF/battery
//
// Vì sao sensor_task ưu tiên THẤP HƠN stabilize_task (23 > 22):
// mục tiêu của cả kiến trúc này là mẫu IMU đi tới MOTOR càng nhanh càng tốt,
// và bên tính ra duty là stabilize_task chứ không phải hub. Đặt hub CAO HƠN
// (bản cũ: hub 23, stabilize 20) làm đúng điều ngược lại: sau
// xTaskNotifyGive() hub VẪN GIỮ CPU và chạy tiếp MAG/BARO/ToF — vòng điều
// khiển phải xếp hàng sau 1-3 transaction I2C (mỗi cái tới 50ms timeout) mới
// được chạy, dù nó đã có đủ mẫu IMU cần thiết từ trước đó.
//
// Với thứ tự đúng, xTaskNotifyGive() trong sensor_task PREEMPT ngay chính nó:
// stabilize chạy trọn một tick điều khiển rồi block lại, hub mới quay về đọc
// nốt các cảm biến chậm. Hub KHÔNG bị bỏ đói vì stabilize luôn block trong
// ulTaskNotifyTake() ở cuối mỗi tick.
//
// Lập luận "hub kẹt I2C thì vòng điều khiển vẫn chạy" KHÔNG hề dựa vào việc
// hub ưu tiên cao hơn: nó dựa vào chỗ hub BLOCKED (không spin) khi bus treo,
// và vào timeout dự phòng của stabilize_task. Đổi thứ tự ưu tiên không đụng
// tới lập luận đó.
//
// ============================================================================
// AI SỞ HỮU PHẦN CỨNG
// ============================================================================
// CHỈ sensor_task được gọi *_driver_read(). Lý do không chỉ là kiến trúc đẹp:
// QMC5883P xoá cờ DRDY ngay khi đọc thanh ghi STATUS, nên hai nơi cùng đọc sẽ
// ăn trộm mẫu của nhau và cả hai đều thấy "lúc có lúc không". Mahony,
// calibration, telemetry ĐỀU đọc snapshot đã publish, không đụng phần cứng.
//
// Lệnh bench cần độc chiếm bus (mag_selftest, baro calibrate_ground) dùng
// sensor_hub_suspend()/resume() — xem bên dưới.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "flight_core/drivers/baro_driver.h"
#include "flight_core/drivers/battery_driver.h"
#include "flight_core/drivers/imu_driver.h"
#include "flight_core/drivers/mag_driver.h"
#include "flight_core/drivers/tof_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// SENSOR_HUB_TASK_PRIORITY — NGAY DƯỚI vòng điều khiển
// ============================================================================
// Ở HEADER (không phải .c) để flight_core.c canh được bất biến kiến trúc
// "stabilize_task > hub" bằng _Static_assert. Hai số nằm ở hai file mà không
// có gì bắt buộc chúng khớp nhau là đúng loại lỗi chỉ lộ ra khi đang bay.
//
// 22 = ngay dưới stabilize_task (23). esp_timer cũng ở 22 nhưng PIN CPU0
// (CONFIG_ESP_TIMER_TASK_AFFINITY_CPU0=y) nên không bao giờ tranh CPU với hub
// trên core 1. Bảng priority đầy đủ: xem đầu sensor_hub.c.
#define SENSOR_HUB_TASK_PRIORITY  22

// Core của hub — ở header CÙNG LÝ DO với priority ngay trên: flight_core.c
// _Static_assert "cả hai task bay phải cùng core 1" (preemption chỉ có ý nghĩa
// TRONG một core; hai task khác core thì priority không quyết định gì cả), và
// lệnh console `tasks` in ra bảng chẩn đoán từ đúng nguồn này thay vì chép tay.
#define SENSOR_HUB_TASK_CORE      1

// Sức khoẻ 1 cảm biến. Tách `valid` (mẫu gần nhất dùng được) khỏi `healthy`
// (cảm biến nhìn chung còn tin được) có chủ đích: MỘT lỗi I2C lẻ tẻ KHÔNG
// được kết luận cảm biến chết — chỉ hạ `healthy` khi lỗi LIÊN TIẾP vượt
// ngưỡng. Bus manager lo retry/reset; tầng bay chỉ quan tâm stale/mất/degraded.
typedef struct {
    bool     valid;              // mẫu gần nhất đọc thành công
    bool     healthy;            // consecutive_errors dưới ngưỡng
    uint32_t seq;                // tăng ĐÚNG 1 mỗi mẫu MỚI publish; 0 = chưa có mẫu nào
    int64_t  timestamp_us;       // esp_timer_get_time() lúc publish mẫu gần nhất
    uint32_t consecutive_errors; // reset về 0 mỗi lần đọc thành công
    uint32_t total_errors;       // cộng dồn từ boot (chẩn đoán bus)
} sensor_health_t;

// Ảnh chụp NHẤT QUÁN của toàn bộ cảm biến tại một thời điểm. Copy nguyên khối
// dưới mutex — consumer không bao giờ thấy nửa mẫu cũ nửa mẫu mới.
typedef struct {
    imu_sample_t    imu;
    sensor_health_t imu_h;

    mag_sample_t    mag;
    sensor_health_t mag_h;

    baro_sample_t   baro;
    sensor_health_t baro_h;

    // Mẫu pin ĐẦY ĐỦ (raw ADC + mV + volt + cờ calibrated/valid) — xem
    // battery_driver.h. battery_h.valid CHỈ true khi battery.valid true (mẫu
    // ngoài dải 1S hoặc thiếu ADC calibration bị coi như KHÔNG có mẫu), giữ
    // đúng quy ước sẵn có của commander_evaluate(): không có mẫu -> KHÔNG trip
    // fault pin, thay vì trip nhầm theo một con số sai.
    battery_sample_t battery;
    sensor_health_t  battery_h;

    // ToF hướng xuống (VL53L0X). TRƯỚC ĐÂY KHÔNG CÓ Ở ĐÂY: hub không hề đọc
    // ToF, và flight_core.c dùng `tof_reading_t tof = {0}` hardcode — nghĩa là
    // cảm biến được init lúc boot rồi KHÔNG BAO GIỜ được lấy mẫu. Giờ hub sở
    // hữu nó như mọi cảm biến I2C khác, và tof_h.seq là thứ alt_estimator dùng
    // để chỉ correction MỘT LẦN cho mỗi mẫu (ToF ~30Hz vs estimator 250Hz).
    tof_reading_t    tof;
    sensor_health_t  tof_h;
} sensor_snapshot_t;

// Ngưỡng lỗi LIÊN TIẾP trước khi hạ healthy. Đủ lớn để nhiễu bus lẻ tẻ không
// đánh sập cảm biến, đủ nhỏ để phát hiện chết thật trong vài chục ms.
#define SENSOR_UNHEALTHY_ERROR_STREAK   5

// Ngưỡng "stale" (us) cho từng cảm biến — tính bằng timestamp, KHÔNG bằng cờ
// ok của lần đọc cuối. Vòng điều khiển tự quyết định stale bằng cách so
// now_us với timestamp_us (xem sensor_hub_age_us()).
#define SENSOR_IMU_STALE_US    ((int64_t)20000)    // 20ms = 5 chu kỳ 250Hz
#define SENSOR_MAG_STALE_US    ((int64_t)300000)   // 300ms — mag ODR thấp, mất vài mẫu là bình thường
#define SENSOR_BARO_STALE_US   ((int64_t)500000)   // 500ms — khớp ALT_EST_BARO_TIMEOUT_MS
// ToF VL53L0X ~30Hz (timing budget 33ms). 200ms = ~6 mẫu bị mất mới coi là
// stale — đủ rộng cho vài lần range_status lỗi (bề mặt hấp thụ/ngoài tầm là
// chuyện BÌNH THƯỜNG với ToF, không phải hỏng cảm biến).
#define SENSOR_TOF_STALE_US    ((int64_t)200000)

// sensor_hub_start() — tạo sensor_task. Gọi SAU khi các *_driver_init() đã
// chạy xong (hub không tự init driver — nó chỉ đọc cái đã init được).
//   notify_task : task sẽ được đánh thức mỗi lần có mẫu IMU mới (stabilize_task).
//                  NULL = không đánh thức ai (hub vẫn chạy, dùng cho bench).
//   imu_int_gpio: chân INT của MPU6050; <0 = không dùng ngắt, hub tự chạy theo
//                  đồng hồ FreeRTOS.
// Cờ *_present nói cảm biến nào ĐÃ init thành công — hub bỏ qua hẳn cảm biến
// không có, không đọc rồi đếm lỗi vô ích.
// battery_divider_ratio KHÔNG còn là tham số: chia áp là hằng số phần cứng,
// định nghĩa tại battery_driver.h (BATTERY_DIVIDER_RATIO) — xem lý do ở đó.
esp_err_t sensor_hub_start(TaskHandle_t notify_task, int imu_int_gpio,
                            const imu_calib_t *imu_calib,
                            bool imu_present, bool mag_present, bool baro_present,
                            bool battery_present, bool tof_present);

// sensor_hub_read() — copy snapshot mới nhất. KHÔNG chạm phần cứng, không
// block lâu (chỉ giữ mutex đủ để memcpy). Gọi được từ bất kỳ task nào.
void sensor_hub_read(sensor_snapshot_t *out);

// sensor_hub_imu_int_active() — CÓ ĐANG chạy theo ngắt data-ready THẬT của
// MPU6050 hay không (đã bật thành công VÀ chưa rơi về polling do mất ngắt
// liên tục — xem IMU_INT_MISS_STREAK_MAX trong sensor_hub.c). KHÁC HẲN việc
// stabilize_task có đang được sensor_hub đánh thức đều đặn hay không (cái đó
// LUÔN đúng một khi hub chạy, kể cả khi hub tự rơi về đồng hồ FreeRTOS nội bộ
// — xem sensor_task(): xTaskNotifyGive() gọi KHÔNG ĐIỀU KIỆN mỗi vòng). Dùng
// hàm này cho telemetry.imu_int_active — KHÔNG dùng cờ "đã start hub" của
// flight_core.c, nếu không field này luôn báo 1 dù ngắt phần cứng chưa bao
// giờ chạy (imu_int_isr_count vẫn là 0), gây hiểu nhầm khi debug dây INT.
bool sensor_hub_imu_int_active(void);

// sensor_hub_age_us() — tuổi của một mẫu tính từ now_us. Trả INT64_MAX nếu
// chưa từng có mẫu (seq==0) — luôn lớn hơn mọi ngưỡng stale, nên caller không
// cần trường hợp đặc biệt cho "chưa bao giờ có dữ liệu".
static inline int64_t sensor_hub_age_us(const sensor_health_t *h, int64_t now_us) {
    if (h->seq == 0) return INT64_MAX;
    return now_us - h->timestamp_us;
}

// sensor_hub_set_imu_calib() — đổi gyro bias đang dùng (sau calib gyro). Hub
// giữ BẢN SAO riêng vì nó là bên gọi imu_driver_read(); truyền con trỏ vào
// task khác sẽ thành dữ liệu chia sẻ không khoá.
void sensor_hub_set_imu_calib(const imu_calib_t *calib);

// sensor_hub_set_tof_present() — bật/tắt việc hub ĐỌC ToF sau khi hub đã chạy.
//
// Cờ *_present được chốt lúc sensor_hub_start(). Với ToF cần đổi được lúc
// runtime vì bring-up của nó CHẠY LẠI ĐƯỢC (flight_core_tof_reinit(), dùng khi
// sửa dây/nguồn tại bàn): init lại thành công mà hub vẫn giữ present=false thì
// cảm biến sống nhưng KHÔNG AI ĐỌC — đúng cái lỗi "driver init OK nhưng hub
// không hề đọc ToF" mà kiến trúc snapshot sinh ra để chặn.
//
// Ghi 1 bool 32-bit là atomic trên Xtensa; hub đọc nó ở đầu mỗi vòng nên hiệu
// lực ngay từ vòng kế. CHỈ gọi khi bus đang được mượn (sensor_hub_suspend) hoặc
// khi chắc chắn driver ở trạng thái dùng được.
void sensor_hub_set_tof_present(bool present);

// ---- Bus lease cho lệnh bench độc chiếm phần cứng ----
// sensor_hub_suspend() chờ tới khi sensor_task thật sự NGỪNG chạm bus rồi mới
// trả về (timeout_ms hết mà chưa dừng được -> trả false, caller PHẢI huỷ lệnh
// bench chứ không được chạy đè). resume() cho hub chạy lại.
//
// Dùng cho mag_driver_selftest() và baro_driver_calibrate_ground() — hai lệnh
// này nói chuyện trực tiếp với chip và sẽ ăn trộm DRDY/mẫu của hub nếu chạy
// song song.
// sensor_hub_stack_free_bytes() / _total_bytes() — chỗ trống CÒN LẠI ít nhất
// từng đo được trên stack của sensor_task, tính bằng BYTE.
//
// ĐƠN VỊ: BYTE (không phải word). uxTaskGetStackHighWaterMark() của FreeRTOS
// vanilla trả về WORD vì nó chia cho sizeof(StackType_t); trên port Xtensa của
// ESP-IDF `#define portSTACK_TYPE uint8_t` (portmacro.h:78) nên phép chia đó là
// chia cho 1 -> kết quả ra BYTE. Cùng đơn vị với SENSOR_TASK_STACK_BYTES, so
// sánh trực tiếp được.
//
// Đây là mức THẤP NHẤT trong toàn bộ vòng đời task (FreeRTOS tô stack bằng
// 0xA5 lúc tạo rồi đếm ngược từ đáy), KHÔNG phải mức tức thời — nên đọc lúc
// nào cũng có ý nghĩa. Trả 0 nếu hub chưa chạy.
uint32_t sensor_hub_stack_free_bytes(void);
uint32_t sensor_hub_stack_total_bytes(void);

bool sensor_hub_suspend(int timeout_ms);
void sensor_hub_resume(void);

#ifdef __cplusplus
}
#endif
