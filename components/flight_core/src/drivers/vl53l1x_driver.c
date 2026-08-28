// vl53l1x_driver.c — backend cho VL53L1X (thanh ghi 16-BIT, ID 0x010F=0xEACC).
//
// ⚠ BACKEND NÀY LÀ CHO VL53L1X. Không tuyên bố con nào khác "giống hệt".
// Bản trước ghi "cùng dòng: VL53L4CD (cũng trả 0xEACC)" — câu đó nguy hiểm ngay
// cả khi ID có trùng: L4CD là chip KHÁC, chuỗi init và bảng cấu hình ULD của nó
// riêng. Ai đọc dòng đó rồi cắm L4CD vào sẽ thấy chip nhận diện "thành công" và
// đo ra số, nhưng là số của một cấu hình sai. Trùng ID không phải bằng chứng
// tương thích — muốn hỗ trợ chip khác thì thêm backend riêng như đã làm với L0X.
//
// ⚠ FILE NÀY CHỈ ĐƯỢC BIÊN DỊCH KHI BOARD_TOF_CHIP == TOF_CHIP_VL53L1X.
// Chọn chip ở main/board_config.h.
//
// ============================================================================
// VÌ SAO KHÔNG DÙNG LẠI ĐƯỢC CODE CỦA VL53L0X
// ============================================================================
// Địa chỉ thanh ghi là 16-BIT. Mọi transaction đều gửi HAI byte index trước dữ
// liệu. Một VL53L1X nối vào driver L0X sẽ ACK bình thường ở 0x29 nhưng đọc ra
// rác: chip hiểu byte 0xC0 (MODEL_ID của L0X) là byte CAO của một index 16-bit
// rồi chờ byte tiếp theo. Không có tham số nào sửa được — phải là driver khác.
// Đó chính là chẩn đoán mà identify_foreign_device() trong tof_driver.c in ra.
//
// ============================================================================
// CÁCH INIT — ST "ultra lite driver" (ULD), KHÔNG phải full ST API
// ============================================================================
// ST phát hành hai thứ cho L1X:
//   1. VL53L1_API đầy đủ (~10k dòng, cần RAM lớn, nhiều lớp trừu tượng)
//   2. VL53L1X ULD — nạp MỘT bảng cấu hình mặc định 91 byte vào 0x2D..0x87,
//      rồi chỉ chỉnh vài thanh ghi (distance mode, timing budget).
//
// Ở đây dùng (2), theo ĐÚNG thứ tự và ĐÚNG giá trị của ULD. Đây là cùng cách
// tiếp cận mà backend L0X đang dùng với VL53L0X_TUNING[].
//
// ============================================================================
// DISTANCE MODE + TIMING BUDGET — HAI THAM SỐ QUYẾT ĐỊNH TẦM ĐO
// ============================================================================
// Đây là thứ L0X KHÔNG CÓ, và là lý do đổi sang chip này. Xem BOARD_TOF_L1X_*
// trong board_config.h để chỉnh; ở đây chỉ giải thích ý nghĩa:
//
//   SHORT  (1): tới ~1.3m, MIỄN NHIỄM ánh sáng nền tốt nhất, cho phép timing
//               budget ngắn nhất (15ms).
//   LONG   (2): tới ~4m danh nghĩa, ~2.6m thực tế trong nhà; nhạy ánh sáng nền
//               hơn (ngoài nắng tụt về ~73cm theo datasheet ST).
//
// Timing budget dài = ít nhiễu hơn + xa hơn, nhưng nhịp mẫu chậm hơn. Nhịp mẫu
// đi THẲNG vào chất lượng vz của alt_estimator (alpha-beta lấy vi phân từ
// range), nên KHÔNG được đặt tuỳ tiện — xem ràng buộc _Static_assert bên dưới.
#include "tof_backend.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "flight_core/fc_features.h"

#if FC_FEATURE_TOF && (BOARD_TOF_CHIP == TOF_CHIP_VL53L1X)

static const char *TAG = "vl53l1x";

// -----------------------------------------------------------------------------
// Register map — index 16-BIT.
// -----------------------------------------------------------------------------
#define L1X_VHV_CONFIG_TIMEOUT_MACROP                    0x0008
#define L1X_VHV_CONFIG_INIT                              0x000B

#define L1X_I2C_SLAVE_DEVICE_ADDRESS                     0x0001

#define L1X_GPIO_HV_MUX_CTRL                             0x0030
#define L1X_GPIO_TIO_HV_STATUS                           0x0031

#define L1X_PHASECAL_CONFIG_TIMEOUT_MACROP               0x004B

#define L1X_RANGE_TIMEOUT_A_HI                           0x005E
#define L1X_RANGE_VCSEL_PERIOD_A                         0x0060
#define L1X_RANGE_TIMEOUT_B_HI                           0x0061
#define L1X_RANGE_VCSEL_PERIOD_B                         0x0063
#define L1X_RANGE_VALID_PHASE_HIGH                       0x0069

#define L1X_SYSTEM_INTERMEASUREMENT_PERIOD               0x006C

#define L1X_SD_CONFIG_WOI_SD0                            0x0078
#define L1X_SD_CONFIG_INITIAL_PHASE_SD0                  0x007A

#define L1X_SYSTEM_INTERRUPT_CLEAR                       0x0086
#define L1X_SYSTEM_MODE_START                            0x0087

#define L1X_RESULT_RANGE_STATUS                          0x0089

// ⚠ THANH GHI OSC CALIBRATION LÀ 0x00DE, KHÔNG PHẢI 0x0022.
// 0x0022 là một thanh ghi khác hoàn toàn. Đọc nhầm nó -> hệ số quy đổi sai ->
// inter-measurement period sai (thường là quá NGẮN) -> chip bỏ mẫu im lặng.
// Triệu chứng sẽ là "ToF lúc được lúc không" và người debug đi tìm dây/nguồn.
#define L1X_RESULT_OSC_CALIBRATE_VAL                     0x00DE
#define L1X_FIRMWARE_SYSTEM_STATUS                       0x00E5

#define L1X_IDENTIFICATION_MODEL_ID                      0x010F

// Bảng cấu hình ULD nạp liên tiếp vào dải này.
#define L1X_CONFIG_START_REG                             0x002D
#define L1X_CONFIG_END_REG                               0x0087

// ID của VL53L1X: đọc 16-bit tại 0x010F -> 0xEACC
//   0x010F = 0xEA  (model ID byte cao)
//   0x0110 = 0xCC  (model ID byte thấp)
// KHÔNG dùng giá trị này để kết luận "chip nào cũng được miễn ra 0xEACC".
#define L1X_MODEL_ID_VALUE                               0xEACC
// Module type @ 0x0111 — đọc thêm để in ra lúc probe. KHÔNG dùng để chấp
// nhận/từ chối chip: nó khác nhau giữa các lô module và biến nó thành điều kiện
// sẽ loại nhầm phần cứng tốt. Chỉ để log khi cần đối chiếu.
#define L1X_IDENTIFICATION_MODULE_TYPE                   0x0111

// SYSTEM_MODE_START
#define L1X_MODE_START_RANGING                           0x40   // timed, back-to-back
#define L1X_MODE_STOP_RANGING                            0x00

// Distance mode (mã nội bộ, KHÔNG phải giá trị thanh ghi — L1X đặt distance
// mode bằng cách ghi 6 thanh ghi khác nhau).
#define L1X_DIST_SHORT   1
#define L1X_DIST_LONG    2

// Chặn giá trị VÔ LÝ ở tầng driver — KHÔNG phải giới hạn bay.
// Đây là biên vật lý danh nghĩa của chip: lớn hơn nó thì kết quả là rác/tràn số
// chứ không phải "đo được xa". Giới hạn TIN CẬY để điều khiển độ cao là
// ALT_EST_TOF_MAX_RANGE_M (alt_estimator.h) và phải đo thực nghiệm.
#define L1X_PHYSICAL_MAX_MM  4000

// ---- Cấu hình lấy từ board_config.h, có mặc định an toàn nếu thiếu ----
#ifndef BOARD_TOF_L1X_DISTANCE_MODE
#  define BOARD_TOF_L1X_DISTANCE_MODE   L1X_DIST_SHORT
#endif
#ifndef BOARD_TOF_L1X_TIMING_BUDGET_MS
#  define BOARD_TOF_L1X_TIMING_BUDGET_MS 33
#endif
// Inter-measurement period: chu kỳ giữa hai lần BẮT ĐẦU đo ở chế độ timed.
//
// ⚠ PHẢI LỚN HƠN timing budget, KHÔNG được bằng. ST ULD ghi rõ IM phải >= TB,
// nhưng đặt BẰNG NHAU không chừa chỗ cho overhead nội bộ giữa hai lần đo — chip
// sẽ trượt nhịp và bỏ mẫu. Khoảng cách ~20% là mức mọi hiện thực ULD dùng.
#ifndef BOARD_TOF_L1X_INTER_MEASUREMENT_MS
#  define BOARD_TOF_L1X_INTER_MEASUREMENT_MS 40
#endif

// Board này dùng sensor-side pull-up ở AVDD ~= 2.8V.
// Nếu sau này đổi sang board có I/O 1.8V thì đặt = 0.
#ifndef BOARD_TOF_L1X_IO_2V8
#  define BOARD_TOF_L1X_IO_2V8   1
#endif

// Chờ chip boot xong firmware. Nhanh (vài ms) — 200ms là biên rộng.
#define L1X_BOOT_TIMEOUT_MS        200
// Chờ lần đo ĐẦU TIÊN lúc init (VHV calibration). Chậm hơn nhiều một lần đo
// bình thường vì chip còn phải chạy calibration, nên KHÔNG dùng chung timeout
// với vòng chờ boot.
#define L1X_INIT_RANGE_TIMEOUT_MS  1000

// ============================================================================
// RÀNG BUỘC BẮT LÚC BIÊN DỊCH
// ============================================================================
// (1) inter-measurement > timing budget. Bằng nhau hoặc nhỏ hơn thì chip trượt
//     nhịp và bỏ mẫu IM LẶNG — triệu chứng sẽ là "ToF lúc được lúc không" và
//     người debug sẽ đi tìm dây/nguồn.
_Static_assert(BOARD_TOF_L1X_INTER_MEASUREMENT_MS > BOARD_TOF_L1X_TIMING_BUDGET_MS,
    "BOARD_TOF_L1X_INTER_MEASUREMENT_MS <= TIMING_BUDGET: khong chua cho overhead "
    "giua hai lan do -> chip truot nhip va bo mau im lang");
// (2) Nhịp mẫu phải NHANH HƠN ngưỡng "mẫu đã cũ" của consumer. IM lớn hơn
//     TOF_STALE_TIMEOUT_MS nghĩa là MỌI mẫu đều tới sau khi mẫu trước đã bị
//     đánh dấu stale -> alt_estimator không bao giờ có correction liên tục, và
//     triệu chứng là "ToF có số nhưng estimator không dùng". Chặn ở đây thay vì
//     để phát hiện lúc bay.
_Static_assert(BOARD_TOF_L1X_INTER_MEASUREMENT_MS * 2 <= TOF_STALE_TIMEOUT_MS,
    "Nhip mau L1X qua cham so voi TOF_STALE_TIMEOUT_MS: moi mau se toi khi mau "
    "truoc da stale -> alt_estimator mat correction. Giam INTER_MEASUREMENT "
    "hoac nang TOF_STALE_TIMEOUT_MS");
_Static_assert(BOARD_TOF_L1X_DISTANCE_MODE == L1X_DIST_SHORT ||
               BOARD_TOF_L1X_DISTANCE_MODE == L1X_DIST_LONG,
    "BOARD_TOF_L1X_DISTANCE_MODE chi duoc la 1 (SHORT) hoac 2 (LONG)");
// (3) LONG mode KHÔNG hỗ trợ timing budget 15ms (bảng ST không có hàng đó).
_Static_assert(!(BOARD_TOF_L1X_DISTANCE_MODE == L1X_DIST_LONG &&
                 BOARD_TOF_L1X_TIMING_BUDGET_MS < 20),
    "LONG mode khong ho tro timing budget < 20ms -- bang cua ST khong co hang do");

// -----------------------------------------------------------------------------
// Low-level I2C — index 16-BIT (khác biệt CỐT LÕI so với L0X).
// -----------------------------------------------------------------------------
static esp_err_t l1x_write8(i2c_master_dev_handle_t dev, uint16_t idx, uint8_t val) {
    const uint8_t buf[3] = {(uint8_t)(idx >> 8), (uint8_t)(idx & 0xFF), val};
    return i2c_master_transmit(dev, buf, sizeof(buf), tof_io_timeout_ms);
}

static esp_err_t l1x_write16(i2c_master_dev_handle_t dev, uint16_t idx, uint16_t val) {
    const uint8_t buf[4] = {(uint8_t)(idx >> 8), (uint8_t)(idx & 0xFF),
                            (uint8_t)(val >> 8), (uint8_t)(val & 0xFF)};
    return i2c_master_transmit(dev, buf, sizeof(buf), tof_io_timeout_ms);
}

static esp_err_t l1x_write32(i2c_master_dev_handle_t dev, uint16_t idx, uint32_t val) {
    const uint8_t buf[6] = {(uint8_t)(idx >> 8), (uint8_t)(idx & 0xFF),
                            (uint8_t)(val >> 24), (uint8_t)(val >> 16),
                            (uint8_t)(val >> 8),  (uint8_t)(val)};
    return i2c_master_transmit(dev, buf, sizeof(buf), tof_io_timeout_ms);
}

static esp_err_t l1x_read(i2c_master_dev_handle_t dev, uint16_t idx,
                          uint8_t *buf, size_t len) {
    const uint8_t iw[2] = {(uint8_t)(idx >> 8), (uint8_t)(idx & 0xFF)};
    return i2c_master_transmit_receive(dev, iw, sizeof(iw), buf, len,
                                       tof_io_timeout_ms);
}

static esp_err_t l1x_read8(i2c_master_dev_handle_t dev, uint16_t idx, uint8_t *val) {
    return l1x_read(dev, idx, val, 1);
}

static esp_err_t l1x_read16(i2c_master_dev_handle_t dev, uint16_t idx, uint16_t *val) {
    uint8_t d[2] = {0};
    const esp_err_t err = l1x_read(dev, idx, d, sizeof(d));
    if (err != ESP_OK) return err;
    *val = (uint16_t)(((uint16_t)d[0] << 8) | d[1]);
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// Bảng cấu hình mặc định của ST ULD: ghi liên tiếp vào 0x2D..0x87.
// Chỉ số 0 của mảng ứng với index 0x2D.
// -----------------------------------------------------------------------------
static const uint8_t L1X_DEFAULT_CONFIG[] = {
    0x00, /* 0x2D */

    // 0x2E — I2C pull-up rail. 1 = pull-up ở AVDD (2.8V), 0 = 1.8V.
    // KHÔNG phải hằng số vô hại: sai rail thì mức logic I2C không đạt ngưỡng và
    // biểu hiện là NACK ngẫu nhiên, giống hệt lỗi dây.
#if BOARD_TOF_L1X_IO_2V8
    0x01, /* 0x2E : I2C pull-up tai AVDD */
#else
    0x00, /* 0x2E : I2C 1V8 */
#endif

    // 0x2F — GPIO1 pull-up rail. Board này KHÔNG dùng GPIO1 (driver đọc cờ
    // "có mẫu mới" qua I2C, không qua chân ngắt) nên để 0. Nếu sau này nối
    // GPIO1 và nó được pull-up ở AVDD thì đổi thành 0x01.
    0x00, /* 0x2F */

    0x01, /* 0x30 */
    0x02, /* 0x31 */
    0x00, /* 0x32 */
    0x02, /* 0x33 */
    0x08, /* 0x34 */
    0x00, /* 0x35 */
    0x08, /* 0x36 */
    0x10, /* 0x37 */
    0x01, /* 0x38 */
    0x01, /* 0x39 */
    0x00, /* 0x3A */
    0x00, /* 0x3B */
    0x00, /* 0x3C */
    0x00, /* 0x3D */
    0xFF, /* 0x3E */
    0x00, /* 0x3F */
    0x0F, /* 0x40 */
    0x00, /* 0x41 */
    0x00, /* 0x42 */
    0x00, /* 0x43 */
    0x00, /* 0x44 */
    0x00, /* 0x45 */
    0x20, /* 0x46 : interrupt mode = new sample ready */
    0x0B, /* 0x47 */
    0x00, /* 0x48 */
    0x00, /* 0x49 */
    0x02, /* 0x4A */
    0x0A, /* 0x4B */
    0x21, /* 0x4C */
    0x00, /* 0x4D */
    0x00, /* 0x4E */
    0x05, /* 0x4F */
    0x00, /* 0x50 */
    0x00, /* 0x51 */
    0x00, /* 0x52 */
    0x00, /* 0x53 */
    0xC8, /* 0x54 */
    0x00, /* 0x55 */
    0x00, /* 0x56 */
    0x38, /* 0x57 */
    0xFF, /* 0x58 */
    0x01, /* 0x59 */
    0x00, /* 0x5A */
    0x08, /* 0x5B */
    0x00, /* 0x5C */
    0x00, /* 0x5D */
    0x01, /* 0x5E */
    0xCC, /* 0x5F : ST ULD = 0xCC */
    0x0F, /* 0x60 */
    0x01, /* 0x61 */
    0xF1, /* 0x62 */
    0x0D, /* 0x63 */
    0x01, /* 0x64 : sigma threshold MSB (mm, FP14.2) */
    0x68, /* 0x65 : sigma threshold LSB */
    0x00, /* 0x66 : min count rate MSB (MCPS, FP9.7) */
    0x80, /* 0x67 : min count rate LSB */
    0x08, /* 0x68 */
    0xB8, /* 0x69 */
    0x00, /* 0x6A */
    0x00, /* 0x6B */
    0x00, /* 0x6C : intermeasurement period (32-bit, 0x6C..0x6F) */
    0x00, /* 0x6D */
    0x0F, /* 0x6E */
    0x89, /* 0x6F */
    0x00, /* 0x70 */
    0x00, /* 0x71 */
    0x00, /* 0x72 : distance threshold high (mm) */
    0x00, /* 0x73 */
    0x00, /* 0x74 : distance threshold low (mm) */
    0x00, /* 0x75 */
    0x00, /* 0x76 */
    0x01, /* 0x77 */
    0x0F, /* 0x78 */
    0x0D, /* 0x79 */
    0x0E, /* 0x7A */
    0x0E, /* 0x7B */
    0x00, /* 0x7C */
    0x00, /* 0x7D */
    0x02, /* 0x7E */
    0xC7, /* 0x7F */
    0xFF, /* 0x80 */
    0x9B, /* 0x81 */
    0x00, /* 0x82 */
    0x00, /* 0x83 */
    0x00, /* 0x84 */
    0x01, /* 0x85 */

    // 0x86/0x87 = 0x00: KHÔNG clear-interrupt và KHÔNG start-ranging trong lúc
    // nạp cấu hình. Cả hai được phát TƯỜNG MINH ở dưới, theo đúng thứ tự. Để
    // khác 0 ở đây nghĩa là chip bắt đầu đo giữa chừng lúc bảng cấu hình mới
    // ghi được một nửa.
    0x00, /* 0x86 */
    0x00, /* 0x87 */
};

_Static_assert(sizeof(L1X_DEFAULT_CONFIG) ==
               (L1X_CONFIG_END_REG - L1X_CONFIG_START_REG + 1),
    "Bang cau hinh ULD sai kich thuoc: phai phu dung dai 0x2D..0x87");

// -----------------------------------------------------------------------------
// Distance mode. L1X không có một thanh ghi "mode" — phải ghi 6 thanh ghi.
// Giá trị lấy từ VL53L1X_SetDistanceMode() của ULD.
// -----------------------------------------------------------------------------
static esp_err_t l1x_set_distance_mode(i2c_master_dev_handle_t dev, uint8_t mode) {
    esp_err_t err;
#define W(f_, i_, v_) do { err = f_(dev, (i_), (v_)); if (err != ESP_OK) return err; } while (0)
    if (mode == L1X_DIST_SHORT) {
        W(l1x_write8,  L1X_PHASECAL_CONFIG_TIMEOUT_MACROP, 0x14);
        W(l1x_write8,  L1X_RANGE_VCSEL_PERIOD_A,           0x07);
        W(l1x_write8,  L1X_RANGE_VCSEL_PERIOD_B,           0x05);
        W(l1x_write8,  L1X_RANGE_VALID_PHASE_HIGH,         0x38);
        W(l1x_write16, L1X_SD_CONFIG_WOI_SD0,              0x0705);
        W(l1x_write16, L1X_SD_CONFIG_INITIAL_PHASE_SD0,    0x0606);
    } else {
        W(l1x_write8,  L1X_PHASECAL_CONFIG_TIMEOUT_MACROP, 0x0A);
        W(l1x_write8,  L1X_RANGE_VCSEL_PERIOD_A,           0x0F);
        W(l1x_write8,  L1X_RANGE_VCSEL_PERIOD_B,           0x0D);
        W(l1x_write8,  L1X_RANGE_VALID_PHASE_HIGH,         0xB8);
        W(l1x_write16, L1X_SD_CONFIG_WOI_SD0,              0x0F0D);
        W(l1x_write16, L1X_SD_CONFIG_INITIAL_PHASE_SD0,    0x0E0E);
    }
#undef W
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// Timing budget. Bảng hằng số lấy nguyên từ VL53L1X_SetTimingBudgetInMs() của
// ULD — ST tra bảng theo (distance mode, budget), KHÔNG tính công thức.
//
// KHÔNG có làm tròn: budget không nằm trong bảng là LỖI, không phải "gần đúng".
// Lý do — nhịp mẫu là thứ mà _Static_assert (2) ở trên đang bảo vệ; im lặng
// dùng một budget khác cái người ta viết ra sẽ vô hiệu hoá đúng cái assert đó.
// Bảng chỉ có 15/20/33/50/100/200/500ms (15 chỉ ở SHORT), và cấu hình đã được
// _Static_assert (3) chặn trường hợp LONG+15ms.
// -----------------------------------------------------------------------------
static esp_err_t l1x_set_timing_budget(i2c_master_dev_handle_t dev,
                                        uint8_t mode, uint16_t budget_ms) {
    uint16_t timeout_a = 0, timeout_b = 0;

    if (mode == L1X_DIST_SHORT) {
        switch (budget_ms) {
            case 15:  timeout_a = 0x001D; timeout_b = 0x0027; break;
            case 20:  timeout_a = 0x0051; timeout_b = 0x006E; break;
            case 33:  timeout_a = 0x00D6; timeout_b = 0x006E; break;
            case 50:  timeout_a = 0x01AE; timeout_b = 0x01E8; break;
            case 100: timeout_a = 0x02E1; timeout_b = 0x0388; break;
            case 200: timeout_a = 0x03E1; timeout_b = 0x0496; break;
            case 500: timeout_a = 0x0591; timeout_b = 0x05C1; break;
            default:
                ESP_LOGE(TAG, "timing budget %ums khong co trong bang ST (SHORT chi "
                              "co 15/20/33/50/100/200/500)", (unsigned)budget_ms);
                return ESP_ERR_INVALID_ARG;
        }
    } else {
        // LONG mode: 15ms KHÔNG được hỗ trợ.
        switch (budget_ms) {
            case 20:  timeout_a = 0x001E; timeout_b = 0x0022; break;
            case 33:  timeout_a = 0x0060; timeout_b = 0x006E; break;
            case 50:  timeout_a = 0x00AD; timeout_b = 0x00C6; break;
            case 100: timeout_a = 0x01CC; timeout_b = 0x01EA; break;
            case 200: timeout_a = 0x02D9; timeout_b = 0x02F8; break;
            case 500: timeout_a = 0x048F; timeout_b = 0x04A4; break;
            default:
                ESP_LOGE(TAG, "timing budget %ums khong co trong bang ST (LONG chi "
                              "co 20/33/50/100/200/500)", (unsigned)budget_ms);
                return ESP_ERR_INVALID_ARG;
        }
    }

    esp_err_t err = l1x_write16(dev, L1X_RANGE_TIMEOUT_A_HI, timeout_a);
    if (err != ESP_OK) return err;
    return l1x_write16(dev, L1X_RANGE_TIMEOUT_B_HI, timeout_b);
}

// Inter-measurement period. Đơn vị thanh ghi là clock tick của chip, và hệ số
// quy đổi phụ thuộc osc calibration ĐỌC TỪ CHÍNH CHIP (0x00DE) — hằng số cứng
// sẽ sai theo từng con. Công thức lấy từ VL53L1X_SetInterMeasurementInMs():
//
//     period = ClockPLL * IM_ms * 1.075
//
// Hiện thực bằng SỐ NGUYÊN (nhân 1075 rồi chia 1000, có làm tròn) thay vì float:
// vòng init không có lý do gì phải kéo FPU vào, và số nguyên cho kết quả xác
// định trên mọi toolchain.
static esp_err_t l1x_set_inter_measurement(i2c_master_dev_handle_t dev,
                                            uint16_t period_ms) {
    if (period_ms == 0) return ESP_ERR_INVALID_ARG;

    uint16_t pll = 0;
    esp_err_t err = l1x_read16(dev, L1X_RESULT_OSC_CALIBRATE_VAL, &pll);
    if (err != ESP_OK) return err;
    pll &= 0x03FF;

    if (pll == 0) {
        // Chip trả 0 = chưa boot xong hoặc đọc lỗi. Nhân với 0 sẽ cho chu kỳ 0
        // = "đo lại ngay lập tức", tức bỏ qua ràng buộc IM > TB một cách im
        // lặng. Báo lỗi thay vì đặt một giá trị vô nghĩa.
        ESP_LOGE(TAG, "osc calibration (0x%04X) doc ra 0 -> khong quy doi duoc "
                      "inter-measurement, chip chua san sang",
                 L1X_RESULT_OSC_CALIBRATE_VAL);
        return ESP_ERR_INVALID_RESPONSE;
    }

    const uint64_t tmp = (uint64_t)pll * (uint64_t)period_ms * 1075ULL;
    const uint32_t period = (uint32_t)((tmp + 500ULL) / 1000ULL);

    return l1x_write32(dev, L1X_SYSTEM_INTERMEASUREMENT_PERIOD, period);
}

// Chip đã boot xong firmware chưa. Phải chờ TRƯỚC khi đọc ID hay ghi cấu hình —
// ghi sớm thì chip nhận nhưng firmware sẽ đè lại lúc boot xong, và triệu chứng
// là "cấu hình không có tác dụng" chứ không phải lỗi.
// ⚠ NACK THOÁNG QUA TRONG CỬA SỔ BOOT KHÔNG PHẢI LỖI.
// Ngay sau power-up / nhả XSHUT / reset, chip chưa kịp dựng xong I2C slave nên
// nó NACK vài lần đầu. Bản trước `return err` ngay lần đọc hỏng ĐẦU TIÊN —
// nghĩa là init thất bại ngẫu nhiên tuỳ vào việc MCU hỏi sớm hay muộn vài trăm
// micro giây. Triệu chứng là "thỉnh thoảng boot không thấy ToF", và người debug
// sẽ đi đo lại dây.
//
// Ở đây: lỗi đọc -> NHỚ lại rồi thử tiếp cho tới deadline. Chỉ khi hết hạn mới
// báo hỏng, và báo kèm lỗi I2C cuối cùng để phân biệt "chip im" với "chip NACK".
// KHÔNG retry vô hạn — vẫn đúng một deadline như cũ.
//
// Chính sách này CHỈ áp cho cửa sổ boot. Lỗi I2C lúc chạy vẫn nổi lên ngay.
static esp_err_t l1x_wait_boot(i2c_master_dev_handle_t dev, uint8_t addr) {
    const int64_t deadline_us = esp_timer_get_time() +
                                (int64_t)L1X_BOOT_TIMEOUT_MS * 1000;
    esp_err_t last_err = ESP_ERR_TIMEOUT;
    while (esp_timer_get_time() < deadline_us) {
        uint8_t st = 0;
        const esp_err_t err = l1x_read8(dev, L1X_FIRMWARE_SYSTEM_STATUS, &st);
        if (err == ESP_OK) {
            if (st & 0x01) return ESP_OK;
        } else {
            last_err = err;      // NACK lúc boot: ghi nhớ, thử lại
        }
        vTaskDelay(1);   // >= 1 tick RTOS
    }
    ESP_LOGE(TAG, "VL53L1X addr=0x%02X: FIRMWARE_SYSTEM_STATUS (0x%04X) khong bao "
                  "boot xong sau %dms, loi I2C cuoi=%s",
             addr, L1X_FIRMWARE_SYSTEM_STATUS, L1X_BOOT_TIMEOUT_MS,
             esp_err_to_name(last_err));
    return ESP_ERR_TIMEOUT;
}

// -----------------------------------------------------------------------------
// Cực tính ngắt — PHẢI ĐỌC TỪ CHIP, không được giả định.
// -----------------------------------------------------------------------------
// ST ULD: polarity = !(bit4 cua GPIO_HV_MUX_CTRL). Cờ "có mẫu mới" là
// (GPIO_TIO_HV_STATUS & 1) == polarity. Giả định sai cực tính thì vòng chờ mẫu
// KHÔNG BAO GIỜ thoát (hoặc tệ hơn: thoát ngay mỗi vòng và đọc lại cùng một mẫu).
//
// CHỐT MỘT LẦN lúc init rồi dùng lại: cực tính không đổi trong lúc chạy, và đọc
// lại nó ở mỗi lần poll là thêm một transaction I2C vào đường nhịp cảm biến.
//
// LƯU Ở tof_sensor_state_t::interrupt_polarity, KHÔNG phải static toàn cục.
// Bản trước dùng biến static: đúng khi có ĐÚNG MỘT sensor, nhưng thêm con thứ
// hai là nó im lặng ghi đè cực tính của con thứ nhất — vòng poll của một trong
// hai sẽ hoặc không bao giờ thấy mẫu, hoặc thấy mẫu ở MỌI vòng (đọc lại cùng
// một kết quả cũ). Không có lỗi nào được in ra.
static esp_err_t l1x_read_interrupt_polarity(i2c_master_dev_handle_t dev,
                                              uint8_t *polarity) {
    uint8_t mux = 0;
    const esp_err_t err = l1x_read8(dev, L1X_GPIO_HV_MUX_CTRL, &mux);
    if (err != ESP_OK) return err;
    *polarity = (uint8_t)(!((mux >> 4) & 0x01));
    return ESP_OK;
}

// Có mẫu mới chưa? KHÔNG chờ — caller quyết định.
static esp_err_t l1x_check_ready(i2c_master_dev_handle_t dev, uint8_t polarity,
                                  bool *ready) {
    uint8_t status = 0;
    const esp_err_t err = l1x_read8(dev, L1X_GPIO_TIO_HV_STATUS, &status);
    if (err != ESP_OK) return err;
    *ready = ((status & 0x01) == polarity);
    return ESP_OK;
}

// Chờ CÓ CHẶN cho tới khi có mẫu. CHỈ dùng trong init (VHV calibration).
// TUYỆT ĐỐI không gọi từ đường runtime — sensor_hub không được phép block.
static esp_err_t l1x_wait_ready_init(i2c_master_dev_handle_t dev, uint8_t polarity,
                                      uint32_t timeout_ms) {
    const int64_t deadline_us = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (esp_timer_get_time() < deadline_us) {
        bool ready = false;
        const esp_err_t err = l1x_check_ready(dev, polarity, &ready);
        if (err != ESP_OK) return err;
        if (ready) return ESP_OK;
        vTaskDelay(1);
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t l1x_clear_interrupt(i2c_master_dev_handle_t dev) {
    return l1x_write8(dev, L1X_SYSTEM_INTERRUPT_CLEAR, 0x01);
}

// =============================================================================
// GIAO DIỆN BACKEND (tof_backend.h)
// =============================================================================

// Địa chỉ I2C của L1X: index 16-bit 0x0001, nhận địa chỉ 7-bit.
esp_err_t tof_backend_set_addr_l1x(i2c_master_dev_handle_t dev, uint8_t new_addr) {
    return l1x_write8(dev, L1X_I2C_SLAVE_DEVICE_ADDRESS, (uint8_t)(new_addr & 0x7F));
}

// Chip CÓ ở địa chỉ này không? CHỈ ĐỌC — không ghi gì, vì facade còn dùng hàm
// này để dò trước khi được phép đổi địa chỉ.
//
// CHỜ BOOT trước khi đọc ID: chip chưa boot xong vẫn ACK nhưng trả rác, và một
// lần "rác" ở đây sẽ bị facade hiểu là "chip lạ" -> in ra chẩn đoán sai hoàn
// toàn. Facade còn thử lại TOF_PROBE_ATTEMPTS lần nữa, nên tệ nhất chỉ là chậm.
bool tof_backend_probe_l1x(i2c_master_bus_handle_t bus, uint8_t addr,
                           uint32_t scl_hz, char *id_desc, size_t id_desc_len) {
    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = scl_hz,
    };
    i2c_master_dev_handle_t dev = NULL;
    if (i2c_master_bus_add_device(bus, &dev_cfg, &dev) != ESP_OK || dev == NULL) {
        return false;
    }

    const esp_err_t boot_err = l1x_wait_boot(dev, addr);
    uint16_t id = 0;
    uint8_t  module_type = 0;
    esp_err_t err = boot_err;
    if (boot_err == ESP_OK) {
        err = l1x_read16(dev, L1X_IDENTIFICATION_MODEL_ID, &id);
        // Module type CHỈ để in ra. Lỗi đọc nó KHÔNG làm probe thất bại —
        // chấp nhận/từ chối chip chỉ dựa trên MODEL_ID.
        (void)l1x_read8(dev, L1X_IDENTIFICATION_MODULE_TYPE, &module_type);
    }
    i2c_master_bus_rm_device(dev);

    if (id_desc && id_desc_len) {
        snprintf(id_desc, id_desc_len,
                 "addr=0x%02X MODEL_ID(0x010F)=0x%04X MODULE_TYPE(0x0111)=0x%02X (err=%s)",
                 (unsigned)addr, (unsigned)id, (unsigned)module_type,
                 esp_err_to_name(err));
    }
    return (err == ESP_OK) && (id == L1X_MODEL_ID_VALUE);
}

esp_err_t tof_backend_setup_l1x(tof_sensor_state_t *s) {
    if (!s || !s->dev) return ESP_ERR_INVALID_ARG;

    i2c_master_dev_handle_t dev = s->dev;
    esp_err_t err;

#define CHECK(expr_) do { err = (expr_); if (err != ESP_OK) { \
    ESP_LOGE(TAG, "VL53L1X init failed line %d: %s", __LINE__, esp_err_to_name(err)); \
    return err; } } while (0)

    // ---- 1. Chờ firmware trong chip boot xong ----
    // Lặp lại dù probe đã chờ: giữa probe và đây có thể đã có một lần reset
    // (facade kéo XSHUT), và bước này rẻ.
    CHECK(l1x_wait_boot(dev, s->addr));

    // ---- 2. Nạp bảng cấu hình mặc định của ST ULD (0x2D..0x87) ----
    // Ghi từng byte thay vì một burst dài: burst 91 byte ở 100kHz chiếm bus
    // ~9ms liên tục, và i2c_master_transmit() cần một buffer tạm 93 byte trên
    // stack của sensor_hub. Đây là đường boot chạy MỘT LẦN, không phải vòng
    // nóng — đổi 91 transaction lấy stack và độ chiếm bus là đúng chiều.
    for (size_t i = 0; i < sizeof(L1X_DEFAULT_CONFIG); ++i) {
        const uint16_t reg = (uint16_t)(L1X_CONFIG_START_REG + i);
        err = l1x_write8(dev, reg, L1X_DEFAULT_CONFIG[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ghi cau hinh 0x%04X that bai: %s",
                     reg, esp_err_to_name(err));
            return err;
        }
    }

    // ---- 3. Chốt cực tính ngắt (ĐỌC từ chip, không giả định) ----
    // Phải sau khi nạp cấu hình, vì chính bảng đó đặt bit cực tính.
    CHECK(l1x_read_interrupt_polarity(dev, &s->interrupt_polarity));

    // ---- 4. Một lần đo để chip tự chạy VHV/phase calibration ----
    // ST bắt buộc bước này sau khi nạp config: nếu bỏ qua, mẫu đầu tiên sẽ có
    // sai số lớn và KHÔNG có gì báo hiệu.
    CHECK(l1x_write8(dev, L1X_SYSTEM_MODE_START, L1X_MODE_START_RANGING));

    err = l1x_wait_ready_init(dev, s->interrupt_polarity, L1X_INIT_RANGE_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "khong nhan duoc mau dau tien sau %dms -> chip khong bat dau do "
                      "(nghi nguon 2.8V khong on hoac cau hinh bi tu choi)",
                 L1X_INIT_RANGE_TIMEOUT_MS);
        (void)l1x_write8(dev, L1X_SYSTEM_MODE_START, L1X_MODE_STOP_RANGING);
        return err;
    }

    CHECK(l1x_clear_interrupt(dev));
    CHECK(l1x_write8(dev, L1X_SYSTEM_MODE_START, L1X_MODE_STOP_RANGING));

    // ST ULD: hai vòng VHV, và bắt đầu VHV từ nhiệt độ lần trước.
    CHECK(l1x_write8(dev, L1X_VHV_CONFIG_TIMEOUT_MACROP, 0x09));
    CHECK(l1x_write8(dev, L1X_VHV_CONFIG_INIT, 0x00));

    // ---- 5. Distance mode + timing budget + inter-measurement ----
    // THỨ TỰ BẮT BUỘC: timing budget tra bảng THEO distance mode, nên mode phải
    // được đặt trước. Đảo lại thì budget lấy từ bảng của mode cũ.
    CHECK(l1x_set_distance_mode(dev, BOARD_TOF_L1X_DISTANCE_MODE));
    CHECK(l1x_set_timing_budget(dev, BOARD_TOF_L1X_DISTANCE_MODE,
                                BOARD_TOF_L1X_TIMING_BUDGET_MS));
    CHECK(l1x_set_inter_measurement(dev, BOARD_TOF_L1X_INTER_MEASUREMENT_MS));

    // ---- 6. Vào chế độ đo liên tục ----
    CHECK(l1x_clear_interrupt(dev));
    CHECK(l1x_write8(dev, L1X_SYSTEM_MODE_START, L1X_MODE_START_RANGING));

    s->ranging_started = true;

    ESP_LOGI(TAG, "VL53L1X ranging START tai 0x%02X (mode=%s, TB=%dms, IM=%dms, "
                  "~%dHz, tam TIN CAY ~%s)",
             s->addr,
             (BOARD_TOF_L1X_DISTANCE_MODE == L1X_DIST_SHORT) ? "SHORT" : "LONG",
             BOARD_TOF_L1X_TIMING_BUDGET_MS, BOARD_TOF_L1X_INTER_MEASUREMENT_MS,
             1000 / BOARD_TOF_L1X_INTER_MEASUREMENT_MS,
             (BOARD_TOF_L1X_DISTANCE_MODE == L1X_DIST_SHORT) ? "1.3m" : "2.6m trong nha");

#undef CHECK
    return ESP_OK;
}

// Khởi động lại vòng đo bằng ĐƯỜNG MỀM (không đụng XSHUT).
//
// PHẢI là STOP -> CLEAR -> START, đủ ba bước.
// Bản trước chỉ clear rồi ghi START đè lên trạng thái đang chạy. Ghi START khi
// chip đã ở trạng thái ranging không buộc nó bắt đầu một chu kỳ mới — nên đúng
// cái ca mà restart sinh ra để cứu (chip kẹt, không sinh mẫu nữa) lại là ca nó
// không cứu được. STOP trước mới đưa được chip về trạng thái xác định.
//
// Lỗi được TRẢ VỀ THẬT, không nuốt: nếu soft restart hỏng, facade còn đường
// mạnh hơn (kéo XSHUT -> chờ boot -> setup lại). Nuốt lỗi ở đây sẽ khiến facade
// tưởng đã cứu xong và không bao giờ leo thang.
esp_err_t tof_backend_restart_l1x(tof_sensor_state_t *s) {
    if (!s || !s->dev) return ESP_ERR_INVALID_STATE;

    // Từ đây tới lúc START lại, chip KHÔNG ở trạng thái đo được.
    s->ranging_started = false;

    esp_err_t err = l1x_write8(s->dev, L1X_SYSTEM_MODE_START, L1X_MODE_STOP_RANGING);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "VL53L1X addr=0x%02X restart: STOP that bai: %s",
                 s->addr, esp_err_to_name(err));
        return err;
    }

    err = l1x_clear_interrupt(s->dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "VL53L1X addr=0x%02X restart: CLEAR that bai: %s",
                 s->addr, esp_err_to_name(err));
        return err;
    }

    err = l1x_write8(s->dev, L1X_SYSTEM_MODE_START, L1X_MODE_START_RANGING);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "VL53L1X addr=0x%02X restart: START that bai: %s",
                 s->addr, esp_err_to_name(err));
        return err;
    }

    s->ranging_started = true;
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// RangeStatus của L1X -> thang PAL mà tof_reading_t::range_status đang dùng.
//
// Bảng tra lấy nguyên từ VL53L1X_GetRangeStatus() của ST ULD: chỉ số là
// device status thô (5 bit), giá trị là PAL status. 0 = mẫu HỢP LỆ.
//
// Dùng BẢNG chứ không switch: đây là ánh xạ của ST, và viết lại nó thành
// switch/case là mở đường cho một ca bị dịch sai mà không ai đối chiếu được
// với source gốc.
// -----------------------------------------------------------------------------
static const uint8_t L1X_STATUS_MAP[24] = {
    255, 255, 255, 5,
    2,   4,   1,   7,
    3,   0,   255, 255,
    9,   13,  255, 255,
    255, 255, 10,  6,
    255, 255, 11,  12
};

esp_err_t tof_backend_poll_l1x(tof_sensor_state_t *s) {
    if (!s || !s->dev || !s->ranging_started) return ESP_ERR_INVALID_STATE;

    bool ready = false;
    esp_err_t err = l1x_check_ready(s->dev, s->interrupt_polarity, &ready);
    if (err != ESP_OK) return err;

    // ⚠ KHÔNG BAO GIỜ CHỜ Ở ĐÂY. Hàm này chạy trong sensor_hub, trên đường nhịp
    // cảm biến của vòng bay. Không có mẫu mới KHÔNG phải lỗi — facade lo
    // watchdog (nó biết bao lâu không mẫu thì là treo).
    if (!ready) return ESP_OK;

    // Chip GIƠ CỜ. Chưa phải "đã có mẫu" — mới chỉ là "chip bảo có". Ghi vào
    // last_ready_us (thuần chẩn đoán), KHÔNG đụng last_sample_us.
    s->last_ready_us = esp_timer_get_time();

    // Đọc CẢ KHỐI kết quả trong MỘT transaction: 0x0089..0x0099 = 17 byte.
    //   [0]      RESULT_RANGE_STATUS                  (0x0089)
    //   [3..4]   ambient count rate MCPS              (0x008C, FP9.7)
    //   [13..14] range mm, crosstalk-corrected        (0x0096)
    //   [15..16] peak signal count rate MCPS          (0x0098, FP9.7)
    // Đọc nhiều lần riêng mở ra cửa sổ để chip cập nhật giữa chừng -> ghép
    // status của mẫu này với khoảng cách của mẫu kia. Rất hiếm, và vì hiếm nên
    // sẽ không bao giờ tìm ra được nếu nó xảy ra lúc bay.
    uint8_t result[17] = {0};
    err = l1x_read(s->dev, L1X_RESULT_RANGE_STATUS, result, sizeof(result));
    // ⚠ ĐỌC HỎNG -> KHÔNG ĐỘNG last_sample_us.
    // Bản trước ghi mốc TRƯỚC lần đọc này, nên bus chết vẫn làm watchdog tin là
    // "vừa có mẫu tươi" — đúng cái mà watchdog sinh ra để phát hiện. Giờ mốc chỉ
    // nhích sau khi kết quả đã thực sự nằm trong tay MCU.
    if (err != ESP_OK) return err;

    const uint8_t raw_status = (uint8_t)(result[0] & 0x1F);
    const uint16_t dist_mm = (uint16_t)(((uint16_t)result[13] << 8) | result[14]);
    const uint16_t amb_raw = (uint16_t)(((uint16_t)result[3]  << 8) | result[4]);
    const uint16_t sig_raw = (uint16_t)(((uint16_t)result[15] << 8) | result[16]);

    // Clear interrupt để chip sinh mẫu kế tiếp. Luôn làm SAU khi đã tiêu thụ
    // xong dữ liệu.
    //
    // ⚠ CLEAR HỎNG = COI NHƯ CẢ NHỊP HỎNG. Không clear được thì chip KHÔNG sinh
    // mẫu kế tiếp, và mọi vòng poll sau sẽ đọc lại đúng khối kết quả cũ. Nhận
    // nó làm mẫu hợp lệ nghĩa là bơm một giá trị ĐỨNG YÊN vào alt_estimator
    // trong khi drone đang đổi độ cao — nguy hiểm hơn hẳn việc mất mẫu, vì mất
    // mẫu thì watchdog thấy còn số đông cứng thì không ai thấy.
    err = l1x_clear_interrupt(s->dev);
    if (err != ESP_OK) return err;   // last_sample_us KHÔNG nhích

    // ---- Từ đây trở xuống: mẫu đã ĐƯỢC TIÊU THỤ TRỌN VẸN ----
    // MỘT mốc thời gian dùng chung cho cả hai trường: chúng mô tả CÙNG một mẫu
    // vật lý, nên gọi esp_timer_get_time() hai lần là tạo ra hai thời điểm khác
    // nhau cho cùng một sự kiện.
    const int64_t sample_now_us = esp_timer_get_time();
    s->last_sample_us = sample_now_us;
    // Mốc "chip còn đo được" — watchdog KHÔNG chạm vào cái này, nên nó là số
    // duy nhất phân biệt được "không có gì để đo" với "mất sensor".
    s->last_consumed_us = sample_now_us;

    s->last.signal_mcps  = sig_raw;
    s->last.ambient_mcps = amb_raw;
    s->last.range_status_raw = raw_status;

    uint8_t mapped = 255;
    if (raw_status < sizeof(L1X_STATUS_MAP)) {
        mapped = L1X_STATUS_MAP[raw_status];
    }
    s->last.range_status = mapped;

    // Gate vật lý rộng theo TẦM CỦA CHIP (khác L0X: 2000mm -> 4000mm). Đây CHỈ
    // chặn giá trị vô lý/hỏng ở tầng driver — KHÔNG phải "độ cao bay an toàn".
    // Giới hạn TIN CẬY để bay là ALT_EST_TOF_MAX_RANGE_M bên alt_estimator, và
    // nó phải được xác định bằng ĐO THỰC TẾ trên bề mặt/ánh sáng thật, không
    // phải lấy con số datasheet. Hai tầng khác nhau, cố ý.
    if (mapped == 0 && dist_mm > 0 && dist_mm <= L1X_PHYSICAL_MAX_MM) {
        s->last.distance_m = (float)dist_mm * 0.001f;
        s->last.valid = true;
        s->last_good_us = sample_now_us;   // CÙNG mốc với last_sample_us
    } else {
        // Ngoài tầm / bề mặt hấp thụ: chip VẪN SỐNG và vẫn đo.
        // last_sample_us ĐÃ nhích ở trên, last_good_us thì KHÔNG. Đó chính là
        // thứ phân biệt "không có gì để đo" với "sensor chết" ở tầng trên.
        s->last.valid = false;
    }

    return ESP_OK;
}

#else   // backend không được chọn
typedef int vl53l1x_backend_not_selected_t;
#endif
