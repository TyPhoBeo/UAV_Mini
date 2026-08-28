// tof_driver.c — FACADE chung cho ToF, KHÔNG chứa code riêng của chip nào.
//
// Ở đây là những thứ ĐÚNG VỚI MỌI chip ToF của dự án này:
//   - XSHUT sequencing (reset cứng, kiểm tra pull-up, cảnh báo strapping pin)
//   - Dò địa chỉ trên bus (0x29 mặc định + địa chỉ mong muốn) và đổi nếu cần
//   - Nhận diện "chip lạ" và nói RÕ nó là họ nào
//   - Stall watchdog (chip ngừng đo nhưng I2C vẫn ACK)
//   - Giữ mẫu cũ qua lỗi bus thoáng qua, tối đa TOF_STALE_TIMEOUT_MS
//
// Code RIÊNG của từng dòng chip nằm ở vl53l0x_driver.c / vl53l1x_driver.c, sau
// giao diện trong tof_backend.h (header NỘI BỘ của thư mục này).
//
// ============================================================================
// DISPATCH — TĨNH, theo BOARD_TOF_CHIP
// ============================================================================
// KHÔNG có con trỏ hàm và KHÔNG dò chip lúc chạy. Lý do: cả hai đều biến "cắm
// nhầm chip" từ một lỗi BUILD thành một hành vi lúc bay. Với dispatch tĩnh,
// backend không được chọn thì không được biên dịch vào, và các hằng số phụ
// thuộc chip (tầm đo, nhịp mẫu) được _Static_assert kiểm ngay lúc build.
//
// Đổi chip = đổi BOARD_TOF_CHIP trong main/board_config.h rồi build lại.
#include "flight_core/drivers/tof_driver.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "tof_backend.h"

// Toàn bộ thân driver nằm sau cờ biên dịch này. Cờ = 0 (từ SENSOR_TOF_ENABLED
// trong main/app_config.h, xem fc_features.h) thì file này biên dịch ra một
// object RỖNG — driver không tốn byte flash nào.
#if FC_FEATURE_TOF

static const char *TAG = "tof_driver";

// ---------------------------------------------------------------------------
// Dispatch tĩnh sang backend đã chọn.
// ---------------------------------------------------------------------------
#if BOARD_TOF_CHIP == TOF_CHIP_VL53L0X
#  define TOF_CHIP_NAME       "VL53L0X"
#  define tof_be_set_addr     tof_backend_set_addr_l0x
#  define tof_be_probe        tof_backend_probe_l0x
#  define tof_be_setup        tof_backend_setup_l0x
#  define tof_be_poll         tof_backend_poll_l0x
#  define tof_be_restart      tof_backend_restart_l0x
#elif BOARD_TOF_CHIP == TOF_CHIP_VL53L1X
#  define TOF_CHIP_NAME       "VL53L1X"
#  define tof_be_set_addr     tof_backend_set_addr_l1x
#  define tof_be_probe        tof_backend_probe_l1x
#  define tof_be_setup        tof_backend_setup_l1x
#  define tof_be_poll         tof_backend_poll_l1x
#  define tof_be_restart      tof_backend_restart_l1x
#else
#  error "BOARD_TOF_CHIP khong hop le -- xem main/board_config.h (TOF_CHIP_VL53L0X / TOF_CHIP_VL53L1X)"
#endif

// Địa chỉ I2C mặc định sau power-on. GIỐNG NHAU cho cả hai họ VL53 (ST dùng
// chung 0x29 cho toàn dòng), nên nó ở facade chứ không ở backend.
#define TOF_DEFAULT_I2C_ADDR   0x29

// Datasheet ST, "Power-up sequence": t_boot = 1.2ms MAX (thời gian firmware
// trong chip boot xong sau khi XSHUT được nhả).
//
// Giá trị cũ ở đây là 2 và nó VI PHẠM spec trong thực tế, không phải trên lý
// thuyết: vTaskDelay(pdMS_TO_TICKS(2)) với tick 1000Hz chỉ bảo đảm "ít nhất 1
// tick trọn vẹn" — tick đầu tiên bị cắt cụt theo pha, nên thời gian thật có thể
// chỉ ~1.0ms, tức NGẮN HƠN t_boot max. Kết quả là hỏng KHÔNG ĐỀU: có lần boot
// kịp, có lần không, và lần không kịp thì đọc ID lỗi -> driver_ok=0.
// 10ms là con số mọi thư viện có uy tín (Pololu, Adafruit, ST ULD) đang dùng;
// 8ms thừa ra chỉ tốn một lần duy nhất lúc boot.
#define TOF_BOOT_DELAY_MS      10
// Giữ XSHUT ở LOW bao lâu trước khi nhả. Datasheet không cho số tối thiểu, nên
// dùng cùng bậc với t_boot thay vì đoán nhỏ hơn — đây là đường boot 1 lần,
// không phải vòng nóng.
#define TOF_XSHUT_HOLD_MS      10

// Driver bus frequency supplied by board/application.
static uint32_t s_scl_speed_hz = 100000;

// MỘT sensor duy nhất (hướng xuống, nguồn correction cho alt_estimator).
static tof_sensor_state_t s_sensor;

// Số lần watchdog phải khởi động lại ranging (xem tof_driver.h).
static uint32_t s_stall_restarts = 0;

// Timeout dùng cho MỌI transaction của backend. Bắt đầu ở mức INIT, hạ xuống
// RUNTIME khi tof_driver_init() thành công (xem cuối hàm đó).
int tof_io_timeout_ms = TOF_IO_INIT_TIMEOUT_MS;

// -----------------------------------------------------------------------------
// XSHUT — active LOW. Dung open-drain de LOW = shutdown, HIGH = release.
// -----------------------------------------------------------------------------
// Theo datasheet VL53, XSHUT can co muc logic xac dinh; neu host co trang thai
// boot khong xac dinh thi nen co pull-up (ST khuyen nghi 10k). Voi carrier co
// pull-up XSHUT ve AVDD/IOVDD, open-drain la cach sach nhat:
//   gpio_set_level(..., 0) -> keo LOW, hardware standby
//   gpio_set_level(..., 1) -> nha high-Z, pull-up hardware keo HIGH
//
// Datasheet cung ghi cac I/O la failsafe; khong dua vao gia thuyet co diode ESD
// ve AVDD. Trong 2V8 mode, muc HIGH cua XSHUT/SDA/SCL/GPIO1 nen bang AVDD.
//
// gpio < 0 = chan XSHUT khong duoc dieu khien boi firmware, hoan toan hop le
// voi MOT sensor o dia chi mac dinh.
static esp_err_t configure_xshut_gpio(int gpio_num) {
    if (gpio_num < 0) return ESP_ERR_INVALID_ARG;
    if (!GPIO_IS_VALID_OUTPUT_GPIO(gpio_num)) {
        ESP_LOGE(TAG, "XSHUT GPIO%d khong phai chan output hop le tren chip nay", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    gpio_config_t cfg = {0};
    cfg.pin_bit_mask = 1ULL << gpio_num;
    // INPUT_OUTPUT_OD, KHÔNG phải OUTPUT_OD.
    //
    // Cả hai lái open-drain giống hệt nhau; khác biệt DUY NHẤT là bit INPUT.
    // gpio_config() với GPIO_MODE_OUTPUT_OD (= OUTPUT|OD, KHÔNG có bit INPUT)
    // gọi gpio_input_disable() -> TẮT input buffer của pad -> gpio_get_level()
    // trên chân đó trả về 0 VĨNH VIỄN, bất kể điện áp thật trên chân.
    //
    // Hậu quả với chính file này: xshut_check_released_high() đọc bằng
    // gpio_get_level() nên nó LUÔN LUÔN thấy LOW và LUÔN LUÔN báo lỗi
    // "XSHUT vẫn LOW sau khi release" — kể cả khi dây và pull-up hoàn toàn
    // đúng. Với GPIO0 nó còn in nguyên cảnh báo strapping/DOWNLOAD MODE. Đó là
    // một báo động giả chỉ vào đúng chỗ KHÔNG hỏng, trong khi lỗi thật (chip
    // không ACK / sai ID) nằm ở dưới.
    //
    // KHÔNG đổi sang GPIO_MODE_OUTPUT/GPIO_MODE_INPUT_OUTPUT (push-pull): xem
    // khối comment ở trên — XSHUT phải lái open-drain.
    cfg.mode = GPIO_MODE_INPUT_OUTPUT_OD;
    cfg.pull_up_en = GPIO_PULLUP_DISABLE;   // dung pull-up hardware ngoai/onboard theo mach
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;

    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) return err;

    // Sau gpio_config(), dua XSHUT ve LOW ngay de startup co trang thai xac dinh.
    return gpio_set_level(gpio_num, 0);
}

// Giữ chip trong reset (kéo LOW thật).
static void xshut_hold(int gpio_num) {
    if (gpio_num < 0) return;
    gpio_set_level(gpio_num, 0);
}

// Nha chip cho boot (HIGH-Z; pull-up hardware keo XSHUT len).
static void xshut_release(int gpio_num) {
    if (gpio_num < 0) return;
    gpio_set_level(gpio_num, 1);
}

// Sau khi release open-drain, XSHUT phai duoc pull-up keo HIGH. Neu van LOW thi
// sensor se khong the boot; bao loi ngay tai nguyen nhan thay vi de ID NACK.
static esp_err_t xshut_check_released_high(int gpio_num) {
    if (gpio_num < 0) return ESP_OK;

    const int64_t deadline_us = esp_timer_get_time() +
                                (int64_t)TOF_BOOT_DELAY_MS * 1000;
    while (esp_timer_get_time() < deadline_us) {
        if (gpio_get_level(gpio_num) != 0) return ESP_OK;
        vTaskDelay(1);
    }

    ESP_LOGE(TAG, "XSHUT GPIO%d van LOW sau khi release -> kiem tra pull-up XSHUT, "
                  "wiring hoac short GND", gpio_num);
    return ESP_ERR_INVALID_STATE;
}

static esp_err_t add_device_at(i2c_master_bus_handle_t bus, uint8_t addr,
                               i2c_master_dev_handle_t *out_dev) {
    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = s_scl_speed_hz,
    };
    return i2c_master_bus_add_device(bus, &dev_cfg, out_dev);
}

// -----------------------------------------------------------------------------
// identify_foreign_device() — "không phải chip đã chọn" thì NÓ LÀ CÁI GÌ?
// -----------------------------------------------------------------------------
// Báo "ID sai" rồi dừng là bỏ người debug lại giữa đường: họ vẫn không biết nên
// sửa dây, đổi địa chỉ, hay đổi BOARD_TOF_CHIP. Ba khả năng có thật khi một
// thiết bị ACK ở 0x29 mà ID không khớp, và chúng cần ba hành động KHÁC HẲN
// nhau — nên phải tách được chúng ra ngay tại đây.
//
// Cách phân biệt (đã đối chiếu datasheet ST + thư viện Pololu + ST ULD):
//   VL53L0X : địa chỉ thanh ghi 8-BIT.  0xC0/0xC1/0xC2 = 0xEE / 0xAA / 0x10
//   VL53L1X : địa chỉ thanh ghi 16-BIT. 0x010F/0x0110 = 0xEA / 0xCC (= 0xEACC)
//             (VL53L4CD dùng chung họ L1X, cũng trả 0xEACC)
//
// GIỜ HAI HỌ ĐỀU CÓ DRIVER trong repo, nên kết luận không còn là "phải thay
// module" mà là "đổi MỘT dòng cấu hình" — thông điệp dưới đây nói đúng điều đó.
static void identify_foreign_device(i2c_master_bus_handle_t bus, uint8_t addr) {
    i2c_master_dev_handle_t dev = NULL;
    if (add_device_at(bus, addr, &dev) != ESP_OK || dev == NULL) return;

    // ID họ L0X: thanh ghi 8-bit 0xC0.
    const uint8_t idx8 = 0xC0;
    uint8_t id_l0x = 0;
    const esp_err_t l0x_err = i2c_master_transmit_receive(dev, &idx8, 1, &id_l0x, 1,
                                                          tof_io_timeout_ms);
    // ID họ L1X: index 16-bit 0x010F, đọc 2 byte.
    const uint8_t idx16[2] = {0x01, 0x0F};
    uint8_t id_l1x[2] = {0, 0};
    const esp_err_t l1x_err = i2c_master_transmit_receive(dev, idx16, sizeof(idx16),
                                                          id_l1x, sizeof(id_l1x),
                                                          tof_io_timeout_ms);
    i2c_master_bus_rm_device(dev);

    if (l0x_err == ESP_OK && id_l0x == 0xEE) {
        ESP_LOGE(TAG, "=> THIET BI TAI 0x%02X LA VL53L0X, nhung firmware dang build cho %s.",
                 addr, TOF_CHIP_NAME);
        ESP_LOGE(TAG, "   SUA: dat BOARD_TOF_CHIP = TOF_CHIP_VL53L0X trong main/board_config.h");
        ESP_LOGE(TAG, "   roi build lai. Driver VL53L0X CO SAN trong repo (vl53l0x_driver.c).");
        return;
    }
    if (l1x_err == ESP_OK && id_l1x[0] == 0xEA && id_l1x[1] == 0xCC) {
        ESP_LOGE(TAG, "=> THIET BI TAI 0x%02X LA VL53L1X (hoac VL53L4CD), nhung firmware "
                      "dang build cho %s.", addr, TOF_CHIP_NAME);
        ESP_LOGE(TAG, "   SUA: dat BOARD_TOF_CHIP = TOF_CHIP_VL53L1X trong main/board_config.h");
        ESP_LOGE(TAG, "   roi build lai. Driver VL53L1X CO SAN trong repo (vl53l1x_driver.c).");
        return;
    }

    ESP_LOGE(TAG, "=> Thiet bi tai 0x%02X KHONG nhan dang duoc: L0X(0xC0)=0x%02X (err=%s), "
                  "L1X(0x010F)=0x%02X%02X (err=%s).",
             addr, id_l0x, esp_err_to_name(l0x_err),
             id_l1x[0], id_l1x[1], esp_err_to_name(l1x_err));
    ESP_LOGE(TAG, "   Doc ra 0x00/0xFF toan bo = chip chua boot xong hoac nguon khong on;");
    ESP_LOGE(TAG, "   so khac hoan toan = day KHONG phai cam bien ToF ho VL53.");
}

// -----------------------------------------------------------------------------
// find_sensor_addr() — chip THẬT SỰ đang trả lời ở địa chỉ nào?
// -----------------------------------------------------------------------------
// i2c_master_bus_add_device() KHÔNG chạm bus (nó chỉ cấp phát handle), nên nó
// LUÔN trả ESP_OK kể cả khi trên bus không có gì. Giao dịch thật đầu tiên là
// lệnh đổi địa chỉ hoặc lệnh đọc ID — hỏng ở đó thì log chỉ nói "that bai",
// không nói được chip có mặt hay không, và nếu có thì ở đâu.
//
// Ngoài ra có một trạng thái thực tế cần tự phục hồi: địa chỉ I2C của VL53 nằm
// trong RAM của chip, chỉ mất khi XSHUT xuống LOW hoặc MẤT NGUỒN. Nạp lại
// firmware / nhấn RESET ESP32 KHÔNG cắt nguồn ToF. Nếu dây XSHUT không được hàn
// (rất phổ biến với module mua rời) thì:
//   boot 1: chip ở 0x29 -> driver đổi sang 0x2A -> chạy OK
//   boot 2: driver "kéo" XSHUT (không nối) -> chip VẪN ở 0x2A, nhưng driver
//           nói chuyện ở 0x29 -> NACK -> driver_ok=0 MÃI cho tới khi rút điện.
// Dò cả hai địa chỉ làm trạng thái đó tự lành.
static bool find_sensor_addr(i2c_master_bus_handle_t bus, uint8_t wanted_addr,
                             uint8_t *out_addr) {
    if (!bus || !out_addr) return false;

    // Sau mot hardware reset dung, chip LUON tro ve 0x29. Vi vay probe 0x29
    // truoc; wanted_addr chi la fallback cho truong hop XSHUT khong thuc su
    // reset duoc sensor va chip con giu dia chi RAM tu lan chay truoc.
    const uint8_t candidates[2] = {
        TOF_DEFAULT_I2C_ADDR,
        wanted_addr,
    };

    for (int attempt = 0; attempt < TOF_PROBE_ATTEMPTS; ++attempt) {
        for (int i = 0; i < 2; ++i) {
            const uint8_t addr = candidates[i];
            if (i == 1 && addr == TOF_DEFAULT_I2C_ADDR) continue;

            if (i2c_master_probe(bus, addr, TOF_IO_INIT_TIMEOUT_MS) != ESP_OK) {
                continue;
            }

            // ACK chua du de ket luan day la dung chip. Doc ID qua backend
            // truoc khi cho phep driver ghi thanh ghi doi dia chi.
            char id_desc[64] = {0};
            if (tof_be_probe(bus, addr, s_scl_speed_hz, id_desc, sizeof(id_desc))) {
                ESP_LOGI(TAG, "%s detected tai 0x%02X: %s", TOF_CHIP_NAME, addr, id_desc);
                *out_addr = addr;
                return true;
            }

            // CHỈ nhận diện sâu ở lần thử CUỐI. Các lần trước có thể trượt đơn
            // giản vì chip chưa boot xong (t_boot), và in đầy đủ 5 lần sẽ đẩy
            // kết luận thật ra khỏi màn hình — log boot trên USB-Serial-JTAG
            // còn xen với log wifi, rất dễ bị cắt mất đúng dòng cần đọc.
            if (attempt == TOF_PROBE_ATTEMPTS - 1) {
                ESP_LOGE(TAG, "I2C 0x%02X ACK nhung ID khong phai %s (%s)",
                         addr, TOF_CHIP_NAME, id_desc);
                identify_foreign_device(bus, addr);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(TOF_PROBE_RETRY_MS));
    }
    return false;
}

// Đổi địa chỉ + XÁC NHẬN giao tiếp THỰC SỰ tại địa chỉ mới.
// i2c_master_device_change_address() chi cap nhat device handle phia ESP-IDF;
// doc lai ID de dam bao ca sensor va handle da cung chuyen sang new_addr.
static esp_err_t change_i2c_address(i2c_master_bus_handle_t bus,
                                     i2c_master_dev_handle_t dev,
                                     uint8_t new_addr) {
    if (new_addr == 0 || new_addr >= 0x80) return ESP_ERR_INVALID_ARG;

    esp_err_t err = tof_be_set_addr(dev, new_addr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s ghi lenh doi I2C -> 0x%02X that bai: %s",
                 TOF_CHIP_NAME, new_addr, esp_err_to_name(err));
        return err;
    }

    err = i2c_master_device_change_address(dev, new_addr, TOF_IO_INIT_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "device handle doi sang 0x%02X that bai: %s",
                 new_addr, esp_err_to_name(err));
        return err;
    }

    char id_desc[64] = {0};
    if (!tof_be_probe(bus, new_addr, s_scl_speed_hz, id_desc, sizeof(id_desc))) {
        ESP_LOGE(TAG, "%s verify dia chi moi 0x%02X that bai: %s",
                 TOF_CHIP_NAME, new_addr, id_desc);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "%s doi dia chi I2C -> 0x%02X OK (%s)",
             TOF_CHIP_NAME, new_addr, id_desc);
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// Public API: bring-up MỘT sensor.
// -----------------------------------------------------------------------------
// Với MỘT sensor:
//   - Địa chỉ mặc định 0x29 dùng LUÔN được (không ai tranh) -> KHÔNG cần đổi.
//   - Dây XSHUT là TÙY CHỌN (module có pull-up 10k, chip tự chạy).
//   - Không có thứ tự bring-up nào cần tuân theo.
//
// xshut_gpio < 0 = không nối, hoàn toàn hợp lệ.
esp_err_t tof_driver_init(i2c_master_bus_handle_t bus, int xshut_gpio,
                           uint8_t i2c_addr, uint32_t scl_speed_hz) {
    if (!bus) return ESP_ERR_INVALID_ARG;
    if (i2c_addr == 0 || i2c_addr >= 0x80) return ESP_ERR_INVALID_ARG;

    if (scl_speed_hz > 0) s_scl_speed_hz = scl_speed_hz;

    // Bring-up ghi hàng chục tới hàng trăm thanh ghi -> cần timeout rộng. Hạ về
    // RUNTIME ở cuối hàm. Đặt lại ở ĐẦU (không chỉ ở khai báo) để lần gọi thứ
    // hai — flight_core_tof_reinit() — cũng có timeout rộng như lần đầu.
    tof_io_timeout_ms = TOF_IO_INIT_TIMEOUT_MS;

    // ---- XSHUT: BEST-EFFORT, không bao giờ chặn bring-up ----
    // Chân không nối / không hợp lệ vẫn init được vì module tự pull-up.
    const bool xshut_ok = (xshut_gpio >= 0) &&
                          (configure_xshut_gpio(xshut_gpio) == ESP_OK);

    ESP_LOGI(TAG, "%s bring-up: addr muc tieu 0x%02X, SCL %lu Hz, XSHUT=%s",
             TOF_CHIP_NAME, i2c_addr, (unsigned long)s_scl_speed_hz,
             xshut_ok ? "co (open-drain)" : "khong noi (dung pull-up tren module)");

    if (xshut_gpio >= 0 && !xshut_ok) {
        ESP_LOGW(TAG, "XSHUT GPIO%d KHONG dung duoc -> khong reset cung duoc chip, "
                      "van thu bring-up qua pull-up hardware", xshut_gpio);
    }

    // ---- Reset cứng (nếu điều khiển được XSHUT) ----
    // Xoá state trong RAM chip, kể cả địa chỉ I2C đã đổi ở lần boot trước —
    // đó là lý do DUY NHẤT còn lại để cần XSHUT khi chỉ có 1 sensor.
    if (xshut_ok) {
        xshut_hold(xshut_gpio);
        vTaskDelay(pdMS_TO_TICKS(TOF_XSHUT_HOLD_MS));
        xshut_release(xshut_gpio);

        // XÁC NHẬN pull-up thật sự kéo chân lên sau khi nhả. Lái open-drain
        // nghĩa là "nhả" = high-Z — nếu KHÔNG có pull-up (hoặc chân bị short
        // GND) thì chân ở LƯNG CHỪNG/LOW, chip không bao giờ boot, và triệu
        // chứng duy nhất sẽ là ID NACK ở dưới — sai nguyên nhân.
        //
        // ⚠ Với GPIO0 (chân strapping BOOT của ESP32-S3) lỗi này còn nghiêm
        // trọng hơn một bậc: chân đọc LOW ở đây nghĩa là nó CŨNG sẽ LOW ở lần
        // reset kế tiếp -> bo vào DOWNLOAD MODE thay vì boot firmware. Nên báo
        // riêng, không gộp vào cảnh báo chung.
        if (xshut_check_released_high(xshut_gpio) != ESP_OK) {
            if (xshut_gpio == 0) {
                ESP_LOGE(TAG, "XSHUT dang o GPIO0 = chan STRAPPING 'BOOT'. Chan nay KHONG len "
                              "HIGH duoc -> lan RESET tiep theo bo se vao DOWNLOAD MODE (khong "
                              "chay firmware). SUA DAY NGAY, hoac dat BOARD_TOF_XSHUT_GPIO = -1 "
                              "(dia chi 0x29 mac dinh khong can XSHUT).");
            }
            // KHÔNG return: vẫn thử dò bus bên dưới. Có thể module tự chạy bằng
            // pull-up riêng của nó và chỉ có đường ĐỌC chân là sai.
        }

        vTaskDelay(pdMS_TO_TICKS(TOF_BOOT_DELAY_MS));
    }

    memset(&s_sensor, 0, sizeof(s_sensor));

    // ---- Tìm chip THẬT SỰ đang ở đâu ----
    uint8_t found_at = 0;
    if (!find_sensor_addr(bus, i2c_addr, &found_at)) {
        ESP_LOGE(TAG, "KHONG thiet bi %s nao tra loi tren I2C o 0x%02X hay 0x%02X.",
                 TOF_CHIP_NAME, i2c_addr, TOF_DEFAULT_I2C_ADDR);
        ESP_LOGE(TAG, "Kiem tra theo thu tu -- (1) VIN/GND cua module, (2) SDA/SCL co dung "
                      "2 chan trong board_config.h khong, (3) chay lenh console 'i2c_scan' "
                      "de xem bus thuc su co gi, (4) BOARD_TOF_CHIP co dung dong chip khong.");
        return ESP_ERR_NOT_FOUND;
    }

    i2c_master_dev_handle_t dev = NULL;
    esp_err_t err = add_device_at(bus, found_at, &dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "them device 0x%02X vao bus that bai: %s",
                 found_at, esp_err_to_name(err));
        return err;
    }

    // ---- Đổi địa chỉ CHỈ KHI cần, và hỏng thì KHÔNG chết ----
    // Với 1 sensor, đổi địa chỉ thuần là "cho gọn" — không ai tranh 0x29. Nên
    // hỏng thì ở lại địa chỉ hiện tại và vẫn bay được.
    uint8_t final_addr = found_at;
    if (found_at != i2c_addr) {
        if (change_i2c_address(bus, dev, i2c_addr) == ESP_OK) {
            final_addr = i2c_addr;
        } else {
            ESP_LOGW(TAG, "doi dia chi 0x%02X -> 0x%02X that bai, DUNG TIEP o 0x%02X "
                          "(chi co 1 ToF nen khong ai tranh dia chi -- van bay duoc)",
                     found_at, i2c_addr, found_at);
        }
    } else if (found_at != TOF_DEFAULT_I2C_ADDR) {
        // Chip đã ở sẵn địa chỉ đích => còn giữ state từ lần boot trước, tức
        // XSHUT không thật sự reset được nó. Không phải lỗi, nhưng phải nói ra
        // vì nó giải thích mọi hành vi "lạ" về sau.
        ESP_LOGW(TAG, "chip DA o 0x%02X san (giu state tu lan boot truoc) -> XSHUT khong "
                      "reset duoc chip. Van init lai ranging tu dau ben duoi.", found_at);
    }

    s_sensor.dev = dev;
    s_sensor.addr = final_addr;
    s_sensor.last.range_status = 255;

    err = tof_be_setup(&s_sensor);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init ranging tai 0x%02X that bai: %s -- thiet bi CO tra loi ACK nen "
                      "day/dia chi OK; nghi nguon 2.8V khong on, hoac chip khong phai %s",
                 final_addr, esp_err_to_name(err), TOF_CHIP_NAME);
        i2c_master_bus_rm_device(dev);
        memset(&s_sensor, 0, sizeof(s_sensor));
        return err;
    }

    s_sensor.last_good_us = 0;
    // Mốc watchdog bắt đầu TỪ ĐÂY: chip vừa được lệnh đo, nên TOF_STALL_RESTART_MS
    // tới phải có mẫu đầu tiên. Để 0 thì watchdog không bao giờ chạy (xem điều
    // kiện last_sample_us != 0 trong poll_with_watchdog).
    s_sensor.last_sample_us = esp_timer_get_time();
    memset(&s_sensor.last, 0, sizeof(s_sensor.last));
    s_sensor.last.range_status = 255;

    // Từ đây mọi transaction là runtime (tof_driver_read() trong sensor_hub).
    tof_io_timeout_ms = TOF_IO_RUNTIME_TIMEOUT_MS;
    ESP_LOGI(TAG, "%s SAN SANG tai 0x%02X (huong xuong, nguon correction cho alt_estimator, "
                  "I2C timeout runtime=%dms)",
             TOF_CHIP_NAME, final_addr, TOF_IO_RUNTIME_TIMEOUT_MS);
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// poll_with_watchdog() — gọi backend, rồi xử lý trạng thái "chip ngừng đo".
// -----------------------------------------------------------------------------
// Backend chỉ trả lời "có mẫu mới hay không". Việc quyết định KHÔNG CÓ MẪU bao
// lâu thì là TREO nằm ở đây vì nó giống nhau cho mọi chip — và vì backend
// không nên tự ý reset phần cứng.
static esp_err_t poll_with_watchdog(void) {
    tof_sensor_state_t *s = &s_sensor;
    if (!s->dev || !s->ranging_started) return ESP_ERR_INVALID_STATE;

    const int64_t before_us = s->last_sample_us;
    const esp_err_t err = tof_be_poll(s);
    if (err != ESP_OK) return err;

    // Backend cập nhật last_sample_us khi VÀ CHỈ KHI nó tiêu thụ một mẫu mới.
    // Mốc không đổi = vòng này không có mẫu.
    if (s->last_sample_us != before_us) return ESP_OK;

    // ---- STALL WATCHDOG (xem tof_driver.h) ----
    // Không có mẫu mới là BÌNH THƯỜNG ở phần lớn các vòng: hub poll và chip đo
    // ở hai nhịp khác nhau nên chúng trượt nhau. Nhưng KHÔNG có mẫu nào trong
    // TOF_STALL_RESTART_MS nghĩa là chip đã ngừng hẳn việc đo — trạng thái treo
    // được báo cáo rộng rãi với dòng VL53 ở continuous mode, trong đó I2C vẫn
    // ACK bình thường nên không có cách nào phát hiện khác.
    const int64_t now_us = esp_timer_get_time();
    if (s->last_sample_us != 0 &&
        (now_us - s->last_sample_us) > (int64_t)TOF_STALL_RESTART_MS * 1000) {
        // Phục hồi NHẸ: clear cờ ngắt + phát lại lệnh continuous (backend lo
        // chi tiết). Hai lệnh ghi (~1ms), KHÔNG phải full re-init.
        //
        // VÌ SAO KHÔNG full re-init ở đây: bring-up đầy đủ block tới ~1s và nó
        // chạy TRONG sensor_hub — tức là treo luôn nguồn cấp mẫu IMU của vòng
        // bay trong 1 giây. Đó là đổi một cảm biến phụ lấy toàn bộ khả năng ổn
        // định. Phục hồi nhẹ đủ cho trường hợp chip chỉ ngừng ranging mà vẫn
        // giữ cấu hình; mất cấu hình thật thì cần 'tof_reinit' tay lúc DISARMED.
        const esp_err_t rerr = tof_be_restart(s);
        s_stall_restarts++;
        // Dời mốc BẤT KỂ thành công hay không: nếu không, mỗi vòng poll kế tiếp
        // lại thử restart và biến watchdog thành vòng lặp ghi bus.
        s->last_sample_us = now_us;
        ESP_LOGW(TAG, "ToF ngung do %dms -> khoi dong lai ranging (lan thu %u, ghi=%s). "
                      "So nay TANG DEU = phan cung that su co van de: sut ap luc motor rut "
                      "dong (VCSEL peak 40mA) hoac nhieu I2C, KHONG phai loi phan mem",
                 TOF_STALL_RESTART_MS, (unsigned)s_stall_restarts, esp_err_to_name(rerr));
    }
    return ESP_OK;
}

uint32_t tof_driver_stall_restarts(void) {
    return s_stall_restarts;
}

int64_t tof_driver_last_sample_us(void) {
    // ⚠ TRẢ VỀ last_consumed_us, KHÔNG PHẢI last_sample_us.
    // last_sample_us bị stall-watchdog DỜI TỚI HIỆN TẠI mỗi lần restart (xem
    // poll_with_watchdog), để nó không restart lặp vô hạn. Dùng nó ở đây thì
    // một con ToF đã chết hẳn vẫn trông như "vừa đo xong" mỗi 300ms -> điều
    // kiện mất-sensor KHÔNG BAO GIỜ đúng, tức là phá đúng cái đang cần bảo vệ.
    //
    // last_consumed_us CHỈ do backend đặt, và chỉ khi một kết quả đã thực sự
    // được đọc trọn vẹn. Watchdog không chạm vào.
    return s_sensor.last_consumed_us;
}

esp_err_t tof_driver_read(tof_reading_t *out) {
    if (!out) return ESP_ERR_INVALID_ARG;
    if (!s_sensor.dev) {
        memset(out, 0, sizeof(*out));
        out->valid = false;
        out->range_status = 255;
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = poll_with_watchdog();
    tof_sensor_state_t *s = &s_sensor;

    if (err != ESP_OK) {
        // Keep the last good reading briefly across transient I2C errors.
        *out = s->last;
        if (out->valid && s->last_good_us > 0 &&
            (esp_timer_get_time() - s->last_good_us) >
                (int64_t)TOF_STALE_TIMEOUT_MS * 1000) {
            out->valid = false;
        }
        return err;
    }

    // If there has never been a good sample, keep invalid.
    if (s->last_good_us == 0) {
        s->last.valid = false;
    } else if ((esp_timer_get_time() - s->last_good_us) >
               (int64_t)TOF_STALE_TIMEOUT_MS * 1000) {
        s->last.valid = false;
        s->last.distance_m = 0.0f;
        s->last.range_status = 255;
    }

    *out = s->last;
    return ESP_OK;
}

const char *tof_driver_chip_name(void) {
    return TOF_CHIP_NAME;
}

#else   // !FC_FEATURE_TOF
// Cảm biến bị tắt lúc biên dịch. Một translation unit hoàn toàn rỗng là
// undefined behaviour theo ISO C, nên để lại đúng một khai báo vô hại.
typedef int tof_driver_disabled_at_compile_time_t;
#endif  // FC_FEATURE_TOF
