// calibration.c — lớp NVS thuần cho calibration_params_t. Xem calibration.h
// cho nguyên tắc tổng thể (KHÔNG có phép toán ước lượng calib ở đây).
#include "flight_core/calibration.h"

#include <string.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "calibration";

#define CALIB_NVS_NAMESPACE   "fc_calib"
#define CALIB_KEY_VERSION     "ver"
#define CALIB_KEY_GYRO        "gyro"
#define CALIB_KEY_GYRO_CRC    "gyro_crc"
#define CALIB_KEY_ACCEL_BIAS  "accel_b"
#define CALIB_KEY_ACCEL_SCALE "accel_s"
#define CALIB_KEY_ACCEL_CRC   "accel_crc"
#define CALIB_KEY_MAG_HARD    "mag_h"
#define CALIB_KEY_MAG_SOFT    "mag_s"
#define CALIB_KEY_MAG_CRC     "mag_crc"
#define CALIB_KEY_TRIM        "trim"
#define CALIB_KEY_TRIM_CRC    "trim_crc"

// Đổi khi layout struct persist thay đổi (thêm/bớt field trong vec3f_t đôi
// bias/scale...) — firmware mới gặp version cũ/lạ sẽ coi TOÀN BỘ NVS như
// chưa từng calib (an toàn mặc định) thay vì đọc rác theo layout sai.
#define CALIB_NVS_FORMAT_VERSION   1u

static bool s_nvs_ready = false;

esp_err_t calibration_nvs_init(void) {
    if (s_nvs_ready) return ESP_OK;

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS can xoa (%s) -> erase + init lai", esp_err_to_name(err));
        esp_err_t erase_err = nvs_flash_erase();
        if (erase_err != ESP_OK) return erase_err;
        err = nvs_flash_init();
    }
    if (err != ESP_OK) return err;

    s_nvs_ready = true;
    return ESP_OK;
}

// crc32_compute() — CRC-32 chuẩn (polynomial 0xEDB88320, init/final XOR
// 0xFFFFFFFF, giống zlib/PNG) — tự triển khai bit-by-bit thay vì phụ thuộc
// header esp_rom_crc.h/mbedtls (tránh lệ thuộc API nội bộ có thể đổi tên giữa
// các bản ESP-IDF) — payload ở đây rất nhỏ (<=24 byte/field), tốc độ không
// quan trọng, chỉ chạy lúc save/load (hiếm, không phải đường nóng 250Hz).
static uint32_t crc32_compute(const void *data, size_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    const uint8_t *p = (const uint8_t *)data;
    for (size_t i = 0; i < len; i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1u) ? (crc >> 1) ^ 0xEDB88320u : (crc >> 1);
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

// read_blob_vec3() — đọc đúng sizeof(vec3f_t) byte, KHÔNG coi ESP_ERR_NVS_NOT_FOUND
// là lỗi (key chưa từng lưu = bình thường lúc mới flash) — trả ESP_ERR_NVS_NOT_FOUND
// nguyên vẹn cho caller tự quyết định *_valid, các lỗi khác (blob sai size,
// I/O lỗi...) propagate nguyên vẹn để log rõ nguyên nhân.
static esp_err_t read_blob_vec3(nvs_handle_t h, const char *key, vec3f_t *out) {
    size_t len = sizeof(vec3f_t);
    esp_err_t err = nvs_get_blob(h, key, out, &len);
    if (err == ESP_OK && len != sizeof(vec3f_t)) {
        ESP_LOGW(TAG, "key '%s' sai kich thuoc (%u != %u) -> coi nhu chua co", key, (unsigned)len, (unsigned)sizeof(vec3f_t));
        return ESP_ERR_NVS_NOT_FOUND;
    }
    return err;
}

static esp_err_t write_blob_vec3(nvs_handle_t h, const char *key, const vec3f_t *v) {
    return nvs_set_blob(h, key, v, sizeof(vec3f_t));
}

// crc_matches() — đọc CRC lưu ở `crc_key`, so với CRC32 tính lại trên
// `payload`/`payload_len` hiện có. false nếu key chưa từng lưu HOẶC không khớp
// (2 trường hợp gộp chung: "không tin được field này", caller đều xử lý như
// nhau — *_valid=false, không phân biệt "chưa calib" với "hỏng dữ liệu" ở
// mức field, chỉ log khác nhau để debug).
static bool crc_matches(nvs_handle_t h, const char *crc_key, const void *payload, size_t payload_len) {
    uint32_t stored_crc = 0;
    if (nvs_get_u32(h, crc_key, &stored_crc) != ESP_OK) return false;
    return stored_crc == crc32_compute(payload, payload_len);
}

esp_err_t calibration_load(calibration_params_t *out) {
    memset(out, 0, sizeof(*out));
    if (out == NULL) return ESP_ERR_INVALID_ARG;

    esp_err_t init_err = calibration_nvs_init();
    if (init_err != ESP_OK) {
        ESP_LOGE(TAG, "calibration_nvs_init that bai: %s -> khong the nap calib", esp_err_to_name(init_err));
        return init_err;
    }

    nvs_handle_t h;
    esp_err_t err = nvs_open(CALIB_NVS_NAMESPACE, NVS_READONLY, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        // Namespace chưa từng tạo (chưa calib lần nào) — bình thường, KHÔNG lỗi.
        ESP_LOGI(TAG, "chua co namespace '%s' trong NVS -> chua tung calib, tat ca *_valid=false", CALIB_NVS_NAMESPACE);
        return ESP_OK;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open('%s') that bai: %s", CALIB_NVS_NAMESPACE, esp_err_to_name(err));
        return err;
    }

    uint32_t stored_version = 0;
    const bool version_ok = (nvs_get_u32(h, CALIB_KEY_VERSION, &stored_version) == ESP_OK) &&
                             (stored_version == CALIB_NVS_FORMAT_VERSION);
    if (!version_ok) {
        // Version thiếu (NVS cũ từ trước khi có version) hoặc khác — coi TOÀN
        // BỘ như chưa calib, KHÔNG cố đọc field theo layout có thể đã đổi.
        ESP_LOGW(TAG, "calib NVS version=%u (ky vong %u) hoac chua co -> coi TOAN BO nhu CHUA CALIB "
                      "(an toan mac dinh, phai calib lai)", (unsigned)stored_version, CALIB_NVS_FORMAT_VERSION);
        nvs_close(h);
        return ESP_OK;
    }

    vec3f_t gyro = vec3f_zero();
    if (read_blob_vec3(h, CALIB_KEY_GYRO, &gyro) == ESP_OK) {
        if (crc_matches(h, CALIB_KEY_GYRO_CRC, &gyro, sizeof(gyro))) {
            out->gyro_bias_dps = gyro;
            out->gyro_valid = true;
        } else {
            ESP_LOGW(TAG, "gyro calib CRC KHONG KHOP -> coi nhu HONG (mat dien giua luc ghi? bit-rot flash?), can calib lai");
        }
    }

    struct { vec3f_t bias; vec3f_t scale; } accel_pair = {
        vec3f_zero(), { 1.0f, 1.0f, 1.0f }
    };
    const bool accel_b_ok = (read_blob_vec3(h, CALIB_KEY_ACCEL_BIAS, &accel_pair.bias) == ESP_OK);
    const bool accel_s_ok = (read_blob_vec3(h, CALIB_KEY_ACCEL_SCALE, &accel_pair.scale) == ESP_OK);
    if (accel_b_ok && accel_s_ok) {
        if (crc_matches(h, CALIB_KEY_ACCEL_CRC, &accel_pair, sizeof(accel_pair))) {
            out->accel_bias_g = accel_pair.bias;
            out->accel_scale = accel_pair.scale;
            out->accel_valid = true;
        } else {
            ESP_LOGW(TAG, "accel calib CRC KHONG KHOP -> coi nhu HONG, can calib lai");
        }
    }

    struct { vec3f_t hard; vec3f_t soft; } mag_pair = {
        vec3f_zero(), { 1.0f, 1.0f, 1.0f }
    };
    const bool mag_h_ok = (read_blob_vec3(h, CALIB_KEY_MAG_HARD, &mag_pair.hard) == ESP_OK);
    const bool mag_s_ok = (read_blob_vec3(h, CALIB_KEY_MAG_SOFT, &mag_pair.soft) == ESP_OK);
    if (mag_h_ok && mag_s_ok) {
        if (crc_matches(h, CALIB_KEY_MAG_CRC, &mag_pair, sizeof(mag_pair))) {
            out->mag_hard_iron = mag_pair.hard;
            out->mag_soft_iron_scale = mag_pair.soft;
            out->mag_valid = true;
        } else {
            ESP_LOGW(TAG, "mag calib CRC KHONG KHOP -> coi nhu HONG, can calib lai");
        }
    }

    // ---- Trim roll/pitch ----
    // Trim CHUA TUNG luu = 0/0 va trim_valid=false -- day la TRANG THAI BINH
    // THUONG (khung chua duoc do), KHONG phai loi va KHONG chan ARM. Khac han
    // gyro/accel: thieu nhung cai do thi khong bay duoc, thieu trim thi bay
    // duoc nhung hoi dat.
    struct { float roll; float pitch; } trim_pair = { 0.0f, 0.0f };
    size_t trim_len = sizeof(trim_pair);
    if (nvs_get_blob(h, CALIB_KEY_TRIM, &trim_pair, &trim_len) == ESP_OK &&
        trim_len == sizeof(trim_pair)) {
        if (crc_matches(h, CALIB_KEY_TRIM_CRC, &trim_pair, sizeof(trim_pair))) {
            out->trim_roll_deg = trim_pair.roll;
            out->trim_pitch_deg = trim_pair.pitch;
            out->trim_valid = true;
        } else {
            ESP_LOGW(TAG, "trim CRC KHONG KHOP -> bo qua, dung trim 0/0");
        }
    }

    nvs_close(h);
    ESP_LOGI(TAG, "calib load: gyro=%d accel=%d mag=%d trim=%d(%.2f/%.2f)",
             (int)out->gyro_valid, (int)out->accel_valid, (int)out->mag_valid,
             (int)out->trim_valid, (double)out->trim_roll_deg, (double)out->trim_pitch_deg);
    return ESP_OK;
}

esp_err_t calibration_save_gyro(const vec3f_t *bias_dps) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(CALIB_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) { ESP_LOGE(TAG, "nvs_open ghi gyro that bai: %s", esp_err_to_name(err)); return err; }
    err = write_blob_vec3(h, CALIB_KEY_GYRO, bias_dps);
    if (err == ESP_OK) err = nvs_set_u32(h, CALIB_KEY_GYRO_CRC, crc32_compute(bias_dps, sizeof(*bias_dps)));
    if (err == ESP_OK) err = nvs_set_u32(h, CALIB_KEY_VERSION, CALIB_NVS_FORMAT_VERSION);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    if (err != ESP_OK) ESP_LOGE(TAG, "luu gyro calib that bai: %s", esp_err_to_name(err));
    return err;
}

esp_err_t calibration_save_accel(const vec3f_t *bias_g, const vec3f_t *scale) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(CALIB_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) { ESP_LOGE(TAG, "nvs_open ghi accel that bai: %s", esp_err_to_name(err)); return err; }
    err = write_blob_vec3(h, CALIB_KEY_ACCEL_BIAS, bias_g);
    if (err == ESP_OK) err = write_blob_vec3(h, CALIB_KEY_ACCEL_SCALE, scale);
    if (err == ESP_OK) {
        const struct { vec3f_t bias; vec3f_t scale; } pair = { *bias_g, *scale };
        err = nvs_set_u32(h, CALIB_KEY_ACCEL_CRC, crc32_compute(&pair, sizeof(pair)));
    }
    if (err == ESP_OK) err = nvs_set_u32(h, CALIB_KEY_VERSION, CALIB_NVS_FORMAT_VERSION);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    if (err != ESP_OK) ESP_LOGE(TAG, "luu accel calib that bai: %s", esp_err_to_name(err));
    return err;
}

esp_err_t calibration_save_mag(const vec3f_t *hard_iron, const vec3f_t *soft_iron_scale) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(CALIB_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) { ESP_LOGE(TAG, "nvs_open ghi mag that bai: %s", esp_err_to_name(err)); return err; }
    err = write_blob_vec3(h, CALIB_KEY_MAG_HARD, hard_iron);
    if (err == ESP_OK) err = write_blob_vec3(h, CALIB_KEY_MAG_SOFT, soft_iron_scale);
    if (err == ESP_OK) {
        const struct { vec3f_t hard; vec3f_t soft; } pair = { *hard_iron, *soft_iron_scale };
        err = nvs_set_u32(h, CALIB_KEY_MAG_CRC, crc32_compute(&pair, sizeof(pair)));
    }
    if (err == ESP_OK) err = nvs_set_u32(h, CALIB_KEY_VERSION, CALIB_NVS_FORMAT_VERSION);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    if (err != ESP_OK) ESP_LOGE(TAG, "luu mag calib that bai: %s", esp_err_to_name(err));
    return err;
}

esp_err_t calibration_save_trim(float roll_deg, float pitch_deg) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(CALIB_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) { ESP_LOGE(TAG, "nvs_open ghi trim that bai: %s", esp_err_to_name(err)); return err; }
    const struct { float roll; float pitch; } pair = { roll_deg, pitch_deg };
    err = nvs_set_blob(h, CALIB_KEY_TRIM, &pair, sizeof(pair));
    if (err == ESP_OK) err = nvs_set_u32(h, CALIB_KEY_TRIM_CRC, crc32_compute(&pair, sizeof(pair)));
    if (err == ESP_OK) err = nvs_set_u32(h, CALIB_KEY_VERSION, CALIB_NVS_FORMAT_VERSION);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    if (err != ESP_OK) ESP_LOGE(TAG, "luu trim that bai: %s", esp_err_to_name(err));
    return err;
}

esp_err_t calibration_erase_all(void) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(CALIB_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) return ESP_OK;   // chưa từng tạo -> coi như đã "xóa"
    if (err != ESP_OK) { ESP_LOGE(TAG, "nvs_open xoa calib that bai: %s", esp_err_to_name(err)); return err; }
    err = nvs_erase_all(h);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    if (err == ESP_OK) ESP_LOGW(TAG, "da XOA TOAN BO calib trong NVS (gyro+accel+mag+trim+version)");
    else ESP_LOGE(TAG, "xoa calib that bai: %s", esp_err_to_name(err));
    return err;
}
