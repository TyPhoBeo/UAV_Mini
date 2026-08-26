// calibration.h — persist gyro/accel/mag calibration qua NVS. Module NÀY chỉ
// làm 2 việc: (1) định nghĩa struct kết quả calib, (2) load/save từng phần
// (gyro/accel/mag ĐỘC LẬP nhau — calib lại 1 cảm biến không xóa 2 cảm biến
// kia) qua nvs_flash. KHÔNG có phép toán ước lượng calib ở đây (trung bình
// mẫu, min/max 6-face, hard/soft-iron...) — toàn bộ nằm trong flight_core.c
// (apply_command() các case CMD_CALIB_*) vì cần đọc mẫu cảm biến LIÊN TỤC
// nhiều tick không-block (xem comment trong flight_core.c), calibration.c chỉ
// là lớp NVS thuần, không đụng vào command queue/stabilize task.
//
// TOÀN VẸN DỮ LIỆU: mỗi field (gyro / accel [bias+scale] / mag [hard+soft])
// lưu KÈM CRC32 riêng — corrupt 1 field (ghi dở dang lúc mất điện, bit-rot
// flash) chỉ làm field ĐÓ bị coi invalid (*_valid=false, đòi calib lại), KHÔNG
// kéo theo 2 field còn lại (giữ đúng tinh thần "độc lập nhau" ở trên). Ngoài
// ra có 1 "format version" chung — đổi CALIBRATION_NVS_FORMAT_VERSION khi đổi
// layout struct (thêm/bớt field) để firmware mới KHÔNG đọc nhầm blob cũ sai
// layout (coi như UNCALIBRATED, an toàn mặc định, thay vì đọc rác).
//
// NGUYÊN TẮC BOOT (xem flight_core.c::flight_core_start()): accel/mag được nạp
// persistent như trước; gyro NVS chỉ là history/backup chẩn đoán. Gyro luôn đo
// fresh + validation độc lập mỗi boot, và fresh fail thì ARM bị chặn.
#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "flight_core/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    vec3f_t gyro_bias_dps;
    bool    gyro_valid;
    float   gyro_cal_temp_c;
    bool    gyro_cal_temp_valid_from_nvs;

    // gyro_valid_from_nvs — NVS CO bias gyro hop le luc boot hay khong.
    //
    // KHAC gyro_valid: tu khi co startup gyro calibration (tuning.h muc 8b),
    // gyro_valid nghia la "bias DUNG DE BAY hien tai hop le", va no CHI bat
    // sau khi cua so VALIDATE dat. Con co nay chi ghi lai "luc boot NVS co gi"
    // — dung de log/so sanh/chan doan, TUYET DOI KHONG dung de quyet dinh co
    // cho ARM hay khong. Gop hai y nghia vao mot co chinh la cach de bias cu
    // trong NVS lang le duoc dung de bay lai.
    bool    gyro_valid_from_nvs;

    vec3f_t accel_bias_g;      // (max+min)/2 mỗi trục, từ quy trình 6-face
    vec3f_t accel_scale;       // 2/(max-min) mỗi trục, 1.0 = không sửa
    bool    accel_valid;

    vec3f_t mag_hard_iron;         // offset (max+min)/2, đơn vị raw count
    vec3f_t mag_soft_iron_scale;   // scale per-axis (xấp xỉ chéo, không full 3x3)
    bool    mag_valid;

    // ---- Trim roll/pitch (độ) — bù lệch cơ khí/CG của khung ----
    // NẰM Ở ĐÂY, không phải chỗ khác, vì nó CÙNG BẢN CHẤT với 3 khối trên: một
    // hằng số đo được từ CHÍNH con drone này, đúng cho tới khi phần cứng đổi.
    // Trước đây trim chỉ sống trong RAM (s_trim_*_deg của flight_core.c) nên
    // MẤT SẠCH sau mỗi lần cắm lại điện — người dùng phải dò lại bằng tay từ
    // đầu mỗi buổi bay, và tệ hơn là dễ quên rồi bay với drone lệch.
    //
    // Lưu theo QUY ƯỚC NGOÀI (đúng như @TRIM SET nhận và @TRIM GET trả), KHÔNG
    // phải theo biến nội bộ đã hoán trục của flight_core.c — để dump NVS đọc
    // được mà không cần biết quy ước trong lõi (xem case CMD_SET_TRIM).
    float   trim_roll_deg;
    float   trim_pitch_deg;
    bool    trim_valid;
} calibration_params_t;

// calibration_nvs_init() — nvs_flash_init() + erase-retry nếu cần (khớp hệt
// pattern net_link.c::init_nvs_flash_safe(), NHƯNG file này KHÔNG include/phụ
// thuộc net_link.c — flight_core là component độc lập, có thể build cùng WiFi
// firmware (src/main.c, đã tự gọi nvs_flash_init() cho esp_wifi) HOẶC cùng
// MicroPython firmware (không có WiFi). nvs_flash_init() bản thân IDEMPOTENT
// (gọi 2 lần từ 2 nơi an toàn, lần sau trả ESP_OK ngay) nên KHÔNG cần biết ai
// gọi trước. An toàn gọi nhiều lần/nhiều task.
esp_err_t calibration_nvs_init(void);

// calibration_load() — nạp TOÀN BỘ những gì có trong NVS vào *out (đã
// memset 0 trước). Phần nào chưa từng lưu -> giữ *_valid=false, KHÔNG coi là
// lỗi (trả ESP_OK) — đây là trạng thái bình thường lúc mới flash lần đầu.
esp_err_t calibration_load(calibration_params_t *out);

esp_err_t calibration_save_gyro(const vec3f_t *bias_dps, float temperature_c);
esp_err_t calibration_save_accel(const vec3f_t *bias_g, const vec3f_t *scale);
esp_err_t calibration_save_mag(const vec3f_t *hard_iron, const vec3f_t *soft_iron_scale);

// calibration_save_trim() — lưu trim roll/pitch (độ, QUY ƯỚC NGOÀI). Gọi mỗi
// lần CMD_SET_TRIM được áp dụng. Ghi NVS ~mỗi lần bấm nút nghe có vẻ nhiều,
// nhưng trim chỉ đổi khi NGƯỜI DÙNG bấm (nút nudge/slider) — vài chục lần một
// buổi, không phải mỗi tick — nên hoàn toàn trong ngân sách wear của NVS.
esp_err_t calibration_save_trim(float roll_deg, float pitch_deg);

// calibration_erase_all() — xóa TOÀN BỘ namespace calib trong NVS (gyro+accel
// +mag). Dùng cho bench/test (nghiệm thu "xóa NVS calib -> UNCALIBRATED") —
// KHÔNG phải thao tác bay, chỉ expose qua lệnh console/MicroPython riêng biệt
// rõ ràng (xem CMD_CALIB_ERASE), không lẫn vào luồng calib bình thường.
esp_err_t calibration_erase_all(void);

#ifdef __cplusplus
}
#endif
