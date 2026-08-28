#include "flight_core/sensor_hub.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/semphr.h"

#include "flight_core/drivers/battery_driver.h"

static const char *TAG = "sensor_hub";

// ============================================================================
// SENSOR_TASK_PRIORITY = 22 — NGAY DƯỚI stabilize_task (23)
// ============================================================================
// Lý do hub phải THẤP HƠN vòng điều khiển: xem sensor_hub.h phần "Vì sao
// sensor_task ưu tiên THẤP HƠN". Ở đây là bảng số thật.
//
// Bức tranh priority THẬT trên bản build này (configMAX_PRIORITIES = 25 ->
// dải hợp lệ 0..24, tra từ FreeRTOSConfig.h + esp_task.h + sdkconfig):
//   24  ipc0 / ipc1   <- IPC_MAX_PRIORITY, MỖI CORE MỘT CON. ipc1 Ở TRÊN CORE 1.
//   23  stabilize_task            <- CORE 1, cao nhất firmware được dùng
//   22  <- sensor_hub (đây)       <- CORE 1
//   22  esp_timer     <- ESP_TASK_TIMER_PRIO, PIN CPU0 nên KHÔNG tranh với hub
//   18  lwIP TCP/IP   <- CONFIG_LWIP_TCPIP_TASK_PRIO, PIN CPU0 (xem sdkconfig)
//    5  net           <- PIN CPU0
//    3  udp_rx        <- PIN CPU0
//    2  console REPL  <- PIN CPU0 (repl_config.task_core_id)
//    1  main          <- PIN CPU0 (CONFIG_ESP_MAIN_TASK_AFFINITY_CPU0)
//
// KHÔNG dùng 24 cho stabilize: sẽ NGANG với ipc1 (task IPC của core 1). Cùng
// priority nghĩa là round-robin với nó, mà `esp_ipc_call*` là đường ĐỒNG BỘ
// giữa 2 core do IDF dùng nội bộ (vd cấp phát ngắt cho core kia) — task gọi nó
// BLOCK tới khi ipc task chạy xong. 23 là mức cao nhất KHÔNG đụng task hệ thống.
//
// Không task nào bị bỏ đói: hub luôn BLOCK trong ulTaskNotifyTake()/
// vTaskDelayUntil() giữa các vòng và stabilize_task cũng vậy, nên idle task của
// core 1 vẫn được chạy — điều kiện để Task Watchdog
// (CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1=y, timeout 5s) không panic.
// Giá trị THẬT ở sensor_hub.h (SENSOR_HUB_TASK_PRIORITY) để flight_core.c canh
// được bất biến "stabilize > hub" lúc biên dịch — xem _Static_assert ở đó.
#define SENSOR_TASK_PRIORITY     SENSOR_HUB_TASK_PRIORITY
#define SENSOR_TASK_CORE         SENSOR_HUB_TASK_CORE
// ĐƠN VỊ: BYTE. xTaskCreate*() của ESP-IDF nhận stack theo BYTE (khác FreeRTOS
// vanilla vốn nhận theo WORD) — 4096 ở đây là 4KB, KHÔNG phải 16KB.
#define SENSOR_TASK_STACK_BYTES  4096

// Canh lúc BIÊN DỊCH: hub PHẢI dưới IPC (configMAX_PRIORITIES-1).
_Static_assert(SENSOR_TASK_PRIORITY < configMAX_PRIORITIES - 1,
               "sensor_hub KHONG duoc ngang/tren IPC task (configMAX_PRIORITIES-1)");

// Hub đọc MPU6050 ở 1kHz rồi gom IMU_SAMPLES_PER_CONTROL mẫu thành một mẫu
// điều khiển. Các cảm biến chậm vẫn được xếp lịch theo nhịp publish 250Hz.
#define SENSOR_HUB_HZ            IMU_SAMPLE_RATE_HZ
#define SENSOR_CONTROL_HZ        (SENSOR_HUB_HZ / IMU_SAMPLES_PER_CONTROL)
_Static_assert((SENSOR_HUB_HZ % IMU_SAMPLES_PER_CONTROL) == 0,
               "IMU sample rate phai chia het cho kich thuoc batch");

// MAG/BARO đọc xen kẽ theo bội số của control_round 250Hz (sau mỗi batch IMU),
// không phải raw round 1kHz. Nhờ vậy đổi oversampling không đổi ODR cảm biến:
//   MAG  : mỗi 5 control_round -> ~50Hz
//   BARO : mỗi 5 control_round -> ~50Hz
// Lệch pha (mag ở vòng chẵn, baro ở vòng lệch 2) để 2 cái không rơi cùng vòng.
#define SENSOR_MAG_DIVISOR       5
#define SENSOR_BARO_DIVISOR      5
#define SENSOR_BARO_PHASE        2

// ~10Hz — ADC, không qua I2C, rẻ nhưng không cần nhanh. Nâng từ 5Hz: sụt áp
// dưới tải lúc drone giật ga xảy ra trong ~100ms, lấy mẫu 200ms/lần có thể
// bỏ lỡ hẳn đáy sụt mà failsafe pin cần thấy. 10Hz vẫn thừa chậm để không
// tốn CPU đáng kể.
#define SENSOR_BATTERY_DIVISOR   25

// ToF: divisor 8 = poll mỗi 32ms (~31Hz), phase 4.
//
// ĐÃ THỬ 4/1 (poll 16ms) rồi HOÀN NGUYÊN: giả thuyết lúc đó là hub 32ms và chip
// 40ms trượt pha nên mất mẫu. Poll dày hơn KHÔNG cải thiện gì trên phần cứng
// thật — lần đo sau khi đổi cho 0 mẫu, tức nút thắt nằm ở chỗ khác (chip không
// sinh mẫu), không phải ở lịch poll. Giữ 8/4 là cấu hình đã từng chạy được.
//
// PHASE 4 giảm va chạm với mag(phase 0)/baro(phase 2) nhưng KHÔNG loại bỏ được.
// Đừng tin điều ngược lại: với divisor 5/5/8 thì
//     ToF ∩ baro = vòng 12, 52, 92, ...   (mỗi 40 vòng = 160ms)
//     ToF ∩ mag  = vòng 20, 60, 100, ...  (mỗi 40 vòng = 160ms)
// (mag ∩ baro thì thật sự rỗng — hai cái đó cùng divisor 5, khác phase.)
//
// Một vòng có 2 transaction là chấp nhận được (~1ms tổng, trong ngân sách 4ms)
// và KHÔNG chặn vòng bay vì hub đã notify stabilize TRƯỚC khi đọc mag/baro/ToF.
#define SENSOR_TOF_DIVISOR       8
#define SENSOR_TOF_PHASE         4

// PHASE phải NHỎ HƠN DIVISOR, nếu không (round % DIVISOR) == PHASE không bao
// giờ đúng và cảm biến đó ngừng được đọc HOÀN TOÀN mà không có lấy một dòng
// log — kiểu hỏng tệ nhất, vì mọi thứ khác vẫn chạy bình thường. Bắt lúc biên
// dịch thay vì lúc bay.
_Static_assert(SENSOR_TOF_PHASE < SENSOR_TOF_DIVISOR,
    "SENSOR_TOF_PHASE >= SENSOR_TOF_DIVISOR -> ToF khong bao gio duoc poll");
_Static_assert(SENSOR_BARO_PHASE < SENSOR_BARO_DIVISOR,
    "SENSOR_BARO_PHASE >= SENSOR_BARO_DIVISOR -> baro khong bao gio duoc poll");

// Timeout chờ ngắt IMU = 2 chu kỳ (giống logic cũ trong stabilize_task): ngắt
// bình thường luôn tới trước hạn; hết hạn nghĩa là ngắt THẬT SỰ ngừng đến.
#define IMU_INT_WAIT_TIMEOUT_MS   (2 * 1000 / SENSOR_HUB_HZ)
#define IMU_INT_WAIT_TIMEOUT_TICKS \
    ((pdMS_TO_TICKS(IMU_INT_WAIT_TIMEOUT_MS) > 0) ? pdMS_TO_TICKS(IMU_INT_WAIT_TIMEOUT_MS) : 1)
#define IMU_INT_MISS_STREAK_MAX   (SENSOR_HUB_HZ / 10)   // ~100ms

// ---- state (1 instance) ----
static SemaphoreHandle_t   s_mtx = NULL;
static sensor_snapshot_t   s_snap;
static TaskHandle_t        s_task = NULL;
static TaskHandle_t        s_notify_task = NULL;

static imu_calib_t         s_imu_calib;
static SemaphoreHandle_t   s_calib_mtx = NULL;

static bool  s_imu_present = false, s_mag_present = false;
static bool  s_baro_present = false, s_battery_present = false;
// volatile: ĐỔI ĐƯỢC LÚC RUNTIME từ task khác (sensor_hub_set_tof_present() sau
// flight_core_tof_reinit()), khác 4 cờ trên vốn chỉ ghi 1 lần lúc start.
static volatile bool s_tof_present = false;

static bool     s_imu_int_active = false;
static uint32_t s_imu_int_wake_count = 0;
static uint32_t s_imu_int_timeout_count = 0;
static int      s_imu_int_miss_streak = 0;

// Bus lease: request = "xin hub dừng chạm bus", acked = "hub đã dừng thật".
// Hai cờ RIÊNG BIỆT vì giữa lúc xin và lúc hub thực sự dừng, hub có thể đang
// ở GIỮA một transaction I2C — chỉ khi hub tự xác nhận mới an toàn cho bench
// command bắt đầu.
static volatile bool s_suspend_request = false;
static volatile bool s_suspend_acked = false;

// mark_ok()/mark_err() — cập nhật sức khoẻ 1 cảm biến. Gọi TRONG mutex.
static void mark_ok(sensor_health_t *h, int64_t now_us) {
    h->valid = true;
    h->healthy = true;
    h->seq++;
    h->timestamp_us = now_us;
    h->consecutive_errors = 0;
}

static void mark_err(sensor_health_t *h) {
    // KHÔNG đụng seq/timestamp: mẫu cũ vẫn là mẫu cũ, tuổi của nó phải tiếp
    // tục tăng để consumer phát hiện stale. Ghi đè timestamp ở đây sẽ làm dữ
    // liệu chết trông như vừa mới — đúng loại lỗi khiến estimator tin vào số
    // đã hỏng.
    h->valid = false;
    h->total_errors++;
    if (h->consecutive_errors < UINT32_MAX) h->consecutive_errors++;
    if (h->consecutive_errors >= SENSOR_UNHEALTHY_ERROR_STREAK) {
        h->healthy = false;
    }
}

static void publish_imu(const imu_sample_t *s, bool ok, int64_t now_us) {
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    if (ok) { s_snap.imu = *s; mark_ok(&s_snap.imu_h, now_us); }
    else    { mark_err(&s_snap.imu_h); }
    xSemaphoreGive(s_mtx);
}

#if FC_FEATURE_MAG   // cảm biến tắt lúc biên dịch -> hàm này không còn ai gọi
static void publish_mag(const mag_sample_t *s, bool ok, int64_t now_us) {
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    if (ok) { s_snap.mag = *s; mark_ok(&s_snap.mag_h, now_us); }
    else    { mark_err(&s_snap.mag_h); }
    xSemaphoreGive(s_mtx);
}
#endif  // FC_FEATURE_MAG

#if FC_FEATURE_BARO
static void publish_baro(const baro_sample_t *s, bool ok, int64_t now_us) {
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    if (ok) { s_snap.baro = *s; mark_ok(&s_snap.baro_h, now_us); }
    else    { mark_err(&s_snap.baro_h); }
    xSemaphoreGive(s_mtx);
}
#endif  // FC_FEATURE_BARO

// publish_battery() — LUÔN lưu mẫu (kể cả không hợp lệ) để telemetry chẩn
// đoán thấy được raw/mV/volt sai là bao nhiêu, nhưng CHỈ mark_ok khi mẫu hợp
// lệ. Tách 2 việc này có chủ đích: "có số để nhìn" khác hẳn "số dùng được để
// bay" (xem battery_driver.h).
#if FC_FEATURE_BATTERY
static void publish_battery(const battery_sample_t *s, bool ok, int64_t now_us) {
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    s_snap.battery = *s;
    if (ok) { mark_ok(&s_snap.battery_h, now_us); }
    else    { mark_err(&s_snap.battery_h); }
    xSemaphoreGive(s_mtx);
}
#endif  // FC_FEATURE_BATTERY

// publish_tof() — LUÔN lưu mẫu (kể cả invalid) để telemetry chẩn đoán nhìn
// được range_status, nhưng CHỈ mark_ok khi measurement dùng được. Cùng quy ước
// publish_battery(): "có số để nhìn" KHÁC HẲN "số dùng được để bay".
//
// QUAN TRỌNG: mark_ok() tăng seq, và alt_estimator dùng ĐÚNG seq đó để nhận
// biết mẫu MỚI (ToF ~30Hz vs estimator 250Hz). Nếu tăng seq cả khi đọc lỗi thì
// estimator sẽ correction lại trên một mẫu CŨ — đúng loại lỗi làm Z bị kéo
// nhiều lần bởi cùng một measurement.
#if FC_FEATURE_TOF
static void publish_tof(const tof_reading_t *s, bool ok, int64_t now_us) {
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    s_snap.tof = *s;
    if (ok) { mark_ok(&s_snap.tof_h, now_us); }
    else    { mark_err(&s_snap.tof_h); }
    xSemaphoreGive(s_mtx);
}
#endif  // FC_FEATURE_TOF

// wait_next_sample() — nguồn nhịp của hub: ngắt data-ready của MPU6050, dự
// phòng bằng đồng hồ FreeRTOS.
//
// Quy tắc không được phá: KHÔNG BAO GIỜ chờ ngắt vô hạn. Ngắt ngừng đến mà hub
// đứng im nghĩa là snapshot ngừng cập nhật — nhưng stabilize_task vẫn chạy và
// vẫn thấy dữ liệu "có vẻ hợp lệ" với timestamp cũ dần. Có timeout thì hub vẫn
// quay, vẫn đọc, vẫn publish (hoặc publish lỗi) — trạng thái thật luôn tới
// được tầng bay.
static void wait_next_sample(TickType_t *last_wake, TickType_t period) {
    if (s_imu_int_active) {
        if (ulTaskNotifyTake(pdTRUE, IMU_INT_WAIT_TIMEOUT_TICKS) > 0) {
            *last_wake = xTaskGetTickCount();
            s_imu_int_miss_streak = 0;
            s_imu_int_wake_count++;
            return;
        }
        s_imu_int_timeout_count++;
        if (++s_imu_int_miss_streak >= IMU_INT_MISS_STREAK_MAX) {
            s_imu_int_active = false;
            ESP_LOGE(TAG, "MPU6050 INT NGUNG DEN (%d lan lien tiep) -> ROI VE POLLING dong ho FreeRTOS",
                      IMU_INT_MISS_STREAK_MAX);
        }
    }
    vTaskDelayUntil(last_wake, period);
}

static void sensor_task(void *arg) {
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(1000 / SENSOR_HUB_HZ);
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t control_round = 0;
    uint32_t batch_count = 0;
    bool batch_ok = true;
    imu_sample_t batch_sum = {0};

    while (1) {
        wait_next_sample(&last_wake, period);

        // ---- Bus lease: nhả bus cho lệnh bench ----
        // Xác nhận ĐÃ dừng rồi mới ngủ. Trong lúc treo cờ này hub KHÔNG chạm
        // phần cứng, nhưng snapshot vẫn giữ nguyên mẫu cuối — timestamp không
        // được làm mới nên consumer sẽ tự thấy stale, đúng như thực tế.
        if (s_suspend_request) {
            batch_count = 0;
            batch_ok = true;
            memset(&batch_sum, 0, sizeof(batch_sum));
            s_suspend_acked = true;
            while (s_suspend_request) {
                vTaskDelay(pdMS_TO_TICKS(5));
            }
            s_suspend_acked = false;
            last_wake = xTaskGetTickCount();   // tránh quay bù một loạt vòng sau khi resume
            continue;
        }

        const int64_t now_us = esp_timer_get_time();

        // ---- IMU: đọc 1kHz, gom 4 mẫu thành một mẫu 250Hz ----
        if (s_imu_present) {
            imu_calib_t calib;
            xSemaphoreTake(s_calib_mtx, portMAX_DELAY);
            calib = s_imu_calib;
            xSemaphoreGive(s_calib_mtx);

            imu_sample_t imu = {0};
            const esp_err_t err = imu_driver_read(&calib, &imu);
            const bool ok = (err == ESP_OK) && imu.ok;
            batch_ok = batch_ok && ok;
            if (ok) {
                batch_sum.gyro_dps.x += imu.gyro_dps.x;
                batch_sum.gyro_dps.y += imu.gyro_dps.y;
                batch_sum.gyro_dps.z += imu.gyro_dps.z;
                batch_sum.accel_g.x += imu.accel_g.x;
                batch_sum.accel_g.y += imu.accel_g.y;
                batch_sum.accel_g.z += imu.accel_g.z;
                // RAW cũng phải gom: telemetry GRAW*/ARAW* đọc từ mẫu đã
                // publish. Bỏ qua đây thì GRAW luôn = 0 và việc so
                // GRAW - GBIAS = GCORR (thứ dùng để CHỨNG MINH calib đúng)
                // trở thành vô nghĩa.
                batch_sum.gyro_raw_dps.x += imu.gyro_raw_dps.x;
                batch_sum.gyro_raw_dps.y += imu.gyro_raw_dps.y;
                batch_sum.gyro_raw_dps.z += imu.gyro_raw_dps.z;
                batch_sum.gyro_corrected_sensor_dps.x += imu.gyro_corrected_sensor_dps.x;
                batch_sum.gyro_corrected_sensor_dps.y += imu.gyro_corrected_sensor_dps.y;
                batch_sum.gyro_corrected_sensor_dps.z += imu.gyro_corrected_sensor_dps.z;
                batch_sum.accel_raw_g.x += imu.accel_raw_g.x;
                batch_sum.accel_raw_g.y += imu.accel_raw_g.y;
                batch_sum.accel_raw_g.z += imu.accel_raw_g.z;
                batch_sum.temp_c += imu.temp_c;
            }
        } else {
            batch_ok = false;
        }

        batch_count++;
        if (batch_count < IMU_SAMPLES_PER_CONTROL) continue;

        control_round++;
        imu_sample_t averaged = {0};
        if (batch_ok) {
            const float inv_n = 1.0f / (float)IMU_SAMPLES_PER_CONTROL;
            averaged.gyro_dps.x = batch_sum.gyro_dps.x * inv_n;
            averaged.gyro_dps.y = batch_sum.gyro_dps.y * inv_n;
            averaged.gyro_dps.z = batch_sum.gyro_dps.z * inv_n;
            averaged.accel_g.x = batch_sum.accel_g.x * inv_n;
            averaged.accel_g.y = batch_sum.accel_g.y * inv_n;
            averaged.accel_g.z = batch_sum.accel_g.z * inv_n;
            averaged.gyro_raw_dps.x = batch_sum.gyro_raw_dps.x * inv_n;
            averaged.gyro_raw_dps.y = batch_sum.gyro_raw_dps.y * inv_n;
            averaged.gyro_raw_dps.z = batch_sum.gyro_raw_dps.z * inv_n;
            averaged.gyro_corrected_sensor_dps.x = batch_sum.gyro_corrected_sensor_dps.x * inv_n;
            averaged.gyro_corrected_sensor_dps.y = batch_sum.gyro_corrected_sensor_dps.y * inv_n;
            averaged.gyro_corrected_sensor_dps.z = batch_sum.gyro_corrected_sensor_dps.z * inv_n;
            averaged.accel_raw_g.x = batch_sum.accel_raw_g.x * inv_n;
            averaged.accel_raw_g.y = batch_sum.accel_raw_g.y * inv_n;
            averaged.accel_raw_g.z = batch_sum.accel_raw_g.z * inv_n;
            averaged.temp_c = batch_sum.temp_c * inv_n;
            averaged.ok = true;
        }
        publish_imu(&averaged, batch_ok, now_us);
        batch_count = 0;
        batch_ok = true;
        memset(&batch_sum, 0, sizeof(batch_sum));

        // ---- Đánh thức vòng điều khiển NGAY sau khi có mẫu IMU mới ----
        // Trước khi đọc mag/baro có chủ đích: vòng điều khiển chỉ cần IMU để
        // chạy, không nên đợi thêm 2-3 transaction I2C nữa mới được chạy.
        //
        // stabilize_task ưu tiên CAO HƠN hub (23 > 22, CÙNG core 1), nên dòng
        // này KHÔNG chỉ "đặt cờ rồi tính sau": nó PREEMPT hub ngay tại đây.
        // Toàn bộ tick điều khiển (Mahony -> Az/Vz/Z -> safety/FSM -> PID ->
        // mixer -> motor) chạy XONG và stabilize block lại, RỒI hub mới quay về
        // dòng dưới để đọc MAG/BARO/ToF/battery. Đây là điểm mấu chốt: thời
        // gian từ mẫu IMU tới lệnh motor không còn bị các cảm biến chậm chen vào.
        //
        // Gọi KHÔNG ĐIỀU KIỆN, kể cả khi đọc IMU lỗi: hub là đồng hồ nhịp của
        // vòng bay, và vòng bay PHẢI chạy để safety/failsafe/Commander chạy.
        // Mẫu lỗi KHÔNG tăng seq (mark_err), nên stabilize_task tự thấy "không
        // có mẫu mới" và bỏ qua bước tích phân — xem imu_updated trong
        // stabilize_task().
        if (s_notify_task != NULL) {
            xTaskNotifyGive(s_notify_task);
        }

        // ---- MAG: xen kẽ ----
        // s_mag_present đã luôn = false khi cảm biến bị tắt, nên #if ở đây chỉ
        // gỡ đi phần code vốn KHÔNG BAO GIỜ chạy — hành vi không đổi một chút nào.
#if FC_FEATURE_MAG
        if (s_mag_present && (control_round % SENSOR_MAG_DIVISOR) == 0) {
            mag_sample_t mag = {0};
            const esp_err_t err = mag_driver_read(&mag);
            // mag.ok=false khi DRDY chưa lên = CHƯA CÓ MẪU MỚI, KHÔNG phải lỗi
            // bus. Chỉ đếm lỗi khi transaction I2C thật sự hỏng — nếu không,
            // ODR thấp của chip sẽ bị hiểu nhầm thành "cảm biến chết".
            if (err != ESP_OK) {
                publish_mag(&mag, false, now_us);
            } else if (mag.ok) {
                publish_mag(&mag, true, now_us);
            }
        }
#endif  // FC_FEATURE_MAG

        // ---- BARO: xen kẽ, lệch pha với mag ----
#if FC_FEATURE_BARO
        if (s_baro_present && (control_round % SENSOR_BARO_DIVISOR) == SENSOR_BARO_PHASE) {
            baro_sample_t baro = {0};
            const esp_err_t err = baro_driver_read(&baro);
            if (err != ESP_OK) {
                publish_baro(&baro, false, now_us);
            } else if (baro.ok) {
                publish_baro(&baro, true, now_us);
            }
            // err==ESP_OK && !baro.ok = chưa calibrate_ground() -> không phải
            // lỗi bus, không đếm.
        }
#endif  // FC_FEATURE_BARO

        // ---- Battery: ADC, không qua I2C ----
        // ok = đọc ADC thành công VÀ mẫu qua được sanity 1S + có calibration
        // thật (battery_driver.h). Mẫu hỏng vẫn được publish để nhìn thấy,
        // nhưng health sẽ báo invalid -> flight_core publish 0.0f ra ngoài ->
        // commander/battery-compensation tự bỏ qua (quy ước sẵn có).
#if FC_FEATURE_BATTERY
        if (s_battery_present && (control_round % SENSOR_BATTERY_DIVISOR) == 0) {
            battery_sample_t bat = {0};
            const esp_err_t err = battery_driver_read(&bat);
            publish_battery(&bat, (err == ESP_OK) && bat.valid, now_us);
        }
#endif  // FC_FEATURE_BATTERY

        // ---- ToF hướng xuống: xen kẽ, lệch pha với mag/baro ----
        // ok = đọc I2C thành công VÀ measurement hợp lệ. range_status != 0 là
        // chuyện BÌNH THƯỜNG với ToF (ngoài tầm, bề mặt hấp thụ, ánh nắng) —
        // publish mẫu để nhìn được range_status, nhưng KHÔNG mark_ok nên seq
        // không tăng và estimator không correction trên nó.
#if FC_FEATURE_TOF
        if (s_tof_present && (control_round % SENSOR_TOF_DIVISOR) == SENSOR_TOF_PHASE) {
            tof_reading_t tof = {0};
            const esp_err_t err = tof_driver_read(&tof);
            publish_tof(&tof, (err == ESP_OK) && tof.valid, now_us);
        }
#endif  // FC_FEATURE_TOF
    }
}

esp_err_t sensor_hub_start(TaskHandle_t notify_task, int imu_int_gpio,
                            const imu_calib_t *imu_calib,
                            bool imu_present, bool mag_present, bool baro_present,
                            bool battery_present, bool tof_present) {
    if (s_task != NULL) return ESP_OK;   // idempotent

    s_mtx = xSemaphoreCreateMutex();
    if (s_mtx == NULL) return ESP_ERR_NO_MEM;
    s_calib_mtx = xSemaphoreCreateMutex();
    if (s_calib_mtx == NULL) return ESP_ERR_NO_MEM;

    memset(&s_snap, 0, sizeof(s_snap));
    if (imu_calib != NULL) s_imu_calib = *imu_calib;

    s_notify_task = notify_task;
    s_imu_present = imu_present;
    s_mag_present = mag_present;
    s_baro_present = baro_present;
    s_battery_present = battery_present;
    s_tof_present = tof_present;

    if (xTaskCreatePinnedToCore(sensor_task, "sensor_hub", SENSOR_TASK_STACK_BYTES,
                                 NULL, SENSOR_TASK_PRIORITY, &s_task,
                                 SENSOR_TASK_CORE) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    // Ngắt phải trỏ vào SENSOR_TASK (không phải stabilize_task nữa) — hub mới
    // là bên đọc phần cứng. Bật SAU khi task đã tồn tại vì cần TaskHandle_t thật.
    if (imu_present && imu_int_gpio >= 0) {
        if (imu_driver_enable_data_ready_int(imu_int_gpio, s_task) == ESP_OK) {
            s_imu_int_active = true;
            ESP_LOGI(TAG, "sensor_hub chay theo NGAT data-ready MPU6050 (GPIO%d)", imu_int_gpio);
        } else {
            ESP_LOGW(TAG, "khong bat duoc INT MPU6050 -> hub chay bang dong ho FreeRTOS %dHz",
                      SENSOR_HUB_HZ);
        }
    }

    ESP_LOGI(TAG, "sensor_hub start: imu_present=%d imu=%dHz batch=%d -> control=%dHz; mag=%d baro=%d batt=%d tof=%d (core %d, prio %d)",
              imu_present,
              SENSOR_HUB_HZ, IMU_SAMPLES_PER_CONTROL, SENSOR_CONTROL_HZ,
              mag_present, baro_present, battery_present, tof_present,
              SENSOR_TASK_CORE, SENSOR_TASK_PRIORITY);
    return ESP_OK;
}

bool sensor_hub_imu_int_active(void) {
    return s_imu_int_active;
}

void sensor_hub_read(sensor_snapshot_t *out) {
    if (out == NULL) return;
    if (s_mtx == NULL) { memset(out, 0, sizeof(*out)); return; }
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    *out = s_snap;
    xSemaphoreGive(s_mtx);
}

void sensor_hub_set_tof_present(bool present) {
    s_tof_present = present;
}

void sensor_hub_set_imu_calib(const imu_calib_t *calib) {
    if (calib == NULL || s_calib_mtx == NULL) return;
    xSemaphoreTake(s_calib_mtx, portMAX_DELAY);
    s_imu_calib = *calib;
    xSemaphoreGive(s_calib_mtx);
}

uint32_t sensor_hub_stack_free_bytes(void) {
    if (s_task == NULL) return 0;
    return (uint32_t)uxTaskGetStackHighWaterMark(s_task);
}

uint32_t sensor_hub_stack_total_bytes(void) {
    return (uint32_t)SENSOR_TASK_STACK_BYTES;
}

bool sensor_hub_suspend(int timeout_ms) {
    if (s_task == NULL) return true;   // hub chưa chạy -> bus vốn đã rảnh
    s_suspend_request = true;
    const int64_t deadline_us = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (!s_suspend_acked) {
        if (esp_timer_get_time() > deadline_us) {
            // KHÔNG nuốt lỗi: caller phải huỷ lệnh bench. Chạy đè lên hub
            // đang dùng bus sẽ cho ra kết quả sai mà trông như thật.
            s_suspend_request = false;
            ESP_LOGE(TAG, "suspend() qua %dms ma sensor_task chua nha bus", timeout_ms);
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return true;
}

void sensor_hub_resume(void) {
    s_suspend_request = false;
}
