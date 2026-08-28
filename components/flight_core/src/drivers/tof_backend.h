// tof_backend.h — GIAO DIỆN NỘI BỘ giữa facade tof_driver.c và driver của TỪNG
// dòng chip (vl53l0x_driver.c / vl53l1x_driver.c).
//
// ⚠ ĐÂY LÀ HEADER NỘI BỘ CỦA src/drivers/. KHÔNG nằm trong include/ và KHÔNG
// được include từ ngoài component. Người dùng driver (sensor_hub, flight_core,
// main.c) chỉ thấy tof_driver.h — API đó KHÔNG ĐỔI khi tách chip.
//
// ============================================================================
// VÌ SAO TÁCH — hai họ chip KHÔNG chỉ khác vài hằng số
// ============================================================================
// Cái này đã được ghi sẵn trong identify_foreign_device() của bản cũ, giờ nó
// thành cấu trúc thật:
//
//   VL53L0X : địa chỉ thanh ghi 8-BIT.  ID tại 0xC0   = 0xEE
//   VL53L1X : địa chỉ thanh ghi 16-BIT. ID tại 0x010F = 0xEACC
//
// Khác biệt đó nằm ở TẦNG THẤP NHẤT — mọi hàm read/write đều khác chữ ký. Nhét
// cả hai vào một file bằng if/else sẽ bắt MỌI helper (encode_timeout,
// get_spad_info, load_tuning, ...) phải mang thêm một tham số "chip nào" trong
// khi thân hàm không dùng chung được một dòng nào. Hai file, mỗi file một họ,
// là cách duy nhất giữ được từng driver đọc thẳng theo datasheet của nó.
//
// Khác biệt vận hành đáng kể (ảnh hưởng tới alt_estimator, KHÔNG chỉ tới driver):
//
//   |                        | VL53L0X        | VL53L1X                    |
//   |------------------------|----------------|----------------------------|
//   | Tầm tối đa danh nghĩa  | ~2.0 m         | ~4.0 m (Long mode)         |
//   | Tầm TIN CẬY trong nhà  | ~1.2 m         | ~2.6 m (Long, ánh sáng yếu)|
//   | Register width         | 8-bit          | 16-bit                     |
//   | Distance mode          | không có       | Short / Medium / Long      |
//   | Timing budget          | 33 ms mặc định | 20..1000 ms, đặt được      |
//   | Kết quả                | RESULT 12 byte | RESULT block riêng         |
//
// Cột "tầm TIN CẬY" là cột quan trọng nhất cho dự án này: nguyên nhân đã phân
// tích được của lần rơi trước là ToF mất mẫu khi vượt ~1.2m của L0X. L1X ở
// Short mode giữ được ~1.3m dưới nắng và Long mode tới ~2.6m trong nhà — đó
// chính là lý do đổi chip.
//
// ============================================================================
// AI CHỌN CHIP
// ============================================================================
// BOARD_TOF_CHIP trong main/board_config.h (TOF_CHIP_VL53L0X / TOF_CHIP_VL53L1X).
// Facade tof_driver.c dispatch tĩnh theo cờ đó — KHÔNG có con trỏ hàm, KHÔNG có
// dò chip lúc chạy: chip nào không được chọn thì KHÔNG được biên dịch vào, và
// gọi nhầm là lỗi LINK chứ không phải lỗi lúc bay.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "flight_core/drivers/tof_driver.h"

// ---------------------------------------------------------------------------
// CHỌN CHIP — BOARD_TOF_CHIP (main/board_config.h)
// ---------------------------------------------------------------------------
// Dùng __has_include vì components/flight_core/ phải build ĐỘC LẬP được, không
// có main/ bên cạnh (bất biến ghi ở đầu components/flight_core/CMakeLists.txt).
// CÙNG cơ chế mà fc_features.h dùng cho app_config.h, và cùng lý do: "dùng nếu
// có, mặc định an toàn nếu không có".
//
// KHÔNG đọc giá trị này từ CMake rồi truyền -D: đã thử và HỎNG (xem fc_features.h).
// Đi qua #include thì trình biên dịch tự sinh depfile, ninja buộc phải dịch lại
// đúng những file bị ảnh hưởng khi đổi chip.
#if defined(__has_include)
#  if __has_include("board_config.h")
#    include "board_config.h"
#  endif
#endif

// Mã dòng chip. NGUỒN THẬT là main/board_config.h (nó tự định nghĩa để đọc
// được độc lập). Ở đây chỉ điền vào khi build ĐỘC LẬP, không có board_config.h.
// Số phải TRÙNG hai bên — đó là lý do chúng được viết ra ở cả hai chỗ chứ không
// include chéo: một file mô tả phần cứng không nên phụ thuộc header nội bộ của
// driver, và ngược lại.
#ifndef TOF_CHIP_VL53L0X
#define TOF_CHIP_VL53L0X   0
#endif
#ifndef TOF_CHIP_VL53L1X
#define TOF_CHIP_VL53L1X   1
#endif
#if (TOF_CHIP_VL53L0X != 0) || (TOF_CHIP_VL53L1X != 1)
#error "TOF_CHIP_* trong board_config.h khong khop voi tof_backend.h -- hai ben phai cung so"
#endif

// Mặc định khi build ĐỘC LẬP, không thấy board_config.h. Chọn L0X vì đó là chip
// đã chạy thật trên bo này trước đây — mặc định phải là cấu hình ĐÃ ĐƯỢC KIỂM
// CHỨNG, không phải cấu hình mới nhất.
#ifndef BOARD_TOF_CHIP
#  define BOARD_TOF_CHIP   TOF_CHIP_VL53L0X
#endif

#if (BOARD_TOF_CHIP != TOF_CHIP_VL53L0X) && (BOARD_TOF_CHIP != TOF_CHIP_VL53L1X)
#error "BOARD_TOF_CHIP chi duoc la TOF_CHIP_VL53L0X hoac TOF_CHIP_VL53L1X -- xem main/board_config.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Hằng số DÙNG CHUNG cho cả hai backend
// ============================================================================
// Đặt ở đây thay vì lặp lại trong hai file: chúng là chính sách của HỆ (bao lâu
// thì coi là treo, timeout bus bao nhiêu), không phải thông số của chip.

// Timeout I2C: INIT rộng (chạy 1 lần lúc boot — probe địa chỉ, hàng chục tới
// hàng trăm lần ghi cấu hình), RUNTIME hẹp (tof_driver_read() trong sensor_hub,
// nằm trên đường nhịp cảm biến nên không được phép block lâu).
#define TOF_IO_INIT_TIMEOUT_MS       50
#define TOF_IO_RUNTIME_TIMEOUT_MS    8
// Timeout cho các vòng chờ có deadline bên trong init (ref calibration, chờ
// boot xong). Khác hai cái trên: đây là "chờ CHIP làm xong việc", không phải
// "chờ BUS trả lời".
#define TOF_INIT_WAIT_TIMEOUT_MS     500

// Dò địa chỉ: thử lại vì chip có thể chưa boot xong đúng lúc probe đầu tiên, và
// vì bus có thể đang bận với cảm biến khác.
#define TOF_PROBE_ATTEMPTS           5
#define TOF_PROBE_RETRY_MS           10

// STALL WATCHDOG — xem khối cùng tên trong tof_driver.h.
//
// Bao lâu KHÔNG có mẫu MỚI thì coi là chip đã treo. Với L0X timing budget 33ms
// (~30Hz) thì 300ms = 9 chu kỳ đo bị mất liên tiếp. Với L1X ở timing budget
// 50ms (~20Hz) thì 300ms = 6 chu kỳ — vẫn đủ rộng để nhiễu lẻ tẻ không kích
// nhầm, đủ hẹp để không kéo dài tình trạng "không có độ cao" quá lâu.
//
// KHÁC TOF_STALE_TIMEOUT_MS (200ms, trong tof_driver.h): cái đó nói "số đo đã
// cũ, đừng dùng cho điều khiển" (an toàn cho consumer). Cái này nói "phần cứng
// đã ngừng đo, phải đá nó dậy" (phục hồi). Hai câu hỏi khác nhau, hai ngưỡng
// riêng.
#define TOF_STALL_RESTART_MS         300

// ============================================================================
// Trạng thái sensor — DÙNG CHUNG, backend tự điền
// ============================================================================
// Cả hai họ chip đều cần đúng những ô này. Phần RIÊNG của từng chip (ví dụ
// stop_variable của L0X) nằm trong static của file backend tương ứng, KHÔNG
// nhét vào đây — nếu để chung thì một nửa struct luôn vô nghĩa với mỗi backend.
typedef struct {
    i2c_master_dev_handle_t dev;   // NULL = chưa init
    uint8_t  addr;                 // địa chỉ 7-bit THẬT SỰ đang dùng
    bool     ranging_started;
    int64_t  last_good_us;         // lần cuối có số đo HỢP LỆ
    // Lần cuối chip báo có mẫu mới = lần cuối nó CÓ ĐO. Tách hẳn khỏi
    // last_good_us: ngoài tầm / bề mặt hấp thụ là range_status != 0 nhưng chip
    // VẪN SỐNG và vẫn đo. Gộp hai cái làm một sẽ khiến watchdog restart chip
    // mỗi lần drone bay qua chỗ không có gì để đo — biến một tình huống bình
    // thường thành một chuỗi reset vô nghĩa.
    int64_t  last_sample_us;

    // Lần cuối một kết quả được TIÊU THỤ TRỌN VẸN (đọc xong khối 17 byte VÀ
    // clear interrupt OK), bất kể hợp lệ hay không.
    //
    // ⚠ VÌ SAO KHÔNG DÙNG LUÔN last_sample_us: stall-watchdog trong tof_driver.c
    // DỜI last_sample_us tới hiện tại mỗi lần nó restart, để bản thân nó không
    // lặp vô hạn. Hệ quả là last_sample_us KHÔNG còn trả lời được câu "chip có
    // còn đo không" — một con ToF chết hẳn vẫn làm nó nhích mỗi 300ms.
    //
    // last_consumed_us CHỈ backend đặt, watchdog không chạm. Đây là số DUY NHẤT
    // dùng được để kết luận "mất sensor" (xem tof_driver_last_sample_us()).
    int64_t  last_consumed_us;

    // Lần cuối chip GIƠ CỜ "có mẫu mới" — KHÁC last_sample_us.
    // last_sample_us chỉ nhích khi đã ĐỌC XONG kết quả thành công; cái này nhích
    // ngay khi thấy cờ. Hai số bằng nhau = đường I2C lành. last_ready_us chạy mà
    // last_sample_us đứng = chip vẫn đo nhưng MCU không lấy được kết quả (bus
    // lỗi). Không có số này thì hai ca đó nhìn giống hệt nhau trong log.
    int64_t  last_ready_us;

    // Cực tính ngắt ĐỌC TỪ CHIP lúc setup. Để PER-INSTANCE chứ không phải static
    // toàn cục: hai sensor trên cùng bus có thể khác cực tính, và một biến chung
    // sẽ khiến con thứ hai đọc cờ "có mẫu" bằng cực tính của con thứ nhất —
    // vòng poll hoặc không bao giờ thấy mẫu, hoặc thấy mẫu ở mọi vòng.
    uint8_t  interrupt_polarity;

    tof_reading_t last;
} tof_sensor_state_t;

// ============================================================================
// Giao diện mà MỖI backend phải hiện thực
// ============================================================================
// Facade gọi đúng bốn hàm này. Tên có tiền tố chip nên hai backend có thể cùng
// nằm trong build mà không đụng ký hiệu (dù bình thường chỉ một cái được biên
// dịch vào — xem CMakeLists.txt).
//
// XXX_probe()  : chip CÓ ở địa chỉ này không? Đọc ID, KHÔNG ghi gì.
//                Trả true + điền *out_id_desc (chuỗi ngắn cho log).
// XXX_setup()  : init đầy đủ + vào chế độ đo liên tục. s->dev phải đã có.
// XXX_poll()   : tiêu thụ MỘT mẫu mới nếu có, cập nhật s->last / các mốc thời
//                gian. KHÔNG có mẫu mới KHÔNG phải lỗi -> trả ESP_OK.
// XXX_restart(): phục hồi NHẸ khi watchdog phát hiện treo (clear cờ + phát lại
//                lệnh đo liên tục). KHÔNG phải full re-init — xem lý do trong
//                tof_driver.c.

esp_err_t tof_backend_set_addr_l0x(i2c_master_dev_handle_t dev, uint8_t new_addr);
bool      tof_backend_probe_l0x(i2c_master_bus_handle_t bus, uint8_t addr,
                                uint32_t scl_hz, char *id_desc, size_t id_desc_len);
esp_err_t tof_backend_setup_l0x(tof_sensor_state_t *s);
esp_err_t tof_backend_poll_l0x(tof_sensor_state_t *s);
esp_err_t tof_backend_restart_l0x(tof_sensor_state_t *s);

esp_err_t tof_backend_set_addr_l1x(i2c_master_dev_handle_t dev, uint8_t new_addr);
bool      tof_backend_probe_l1x(i2c_master_bus_handle_t bus, uint8_t addr,
                                uint32_t scl_hz, char *id_desc, size_t id_desc_len);
esp_err_t tof_backend_setup_l1x(tof_sensor_state_t *s);
esp_err_t tof_backend_poll_l1x(tof_sensor_state_t *s);
esp_err_t tof_backend_restart_l1x(tof_sensor_state_t *s);

// Timeout hiện hành cho MỌI transaction của backend. Facade hạ nó từ INIT
// xuống RUNTIME khi bring-up xong (xem cuối tof_driver_init()). Định nghĩa
// trong tof_driver.c, dùng ở cả hai backend.
extern int tof_io_timeout_ms;

#ifdef __cplusplus
}
#endif
