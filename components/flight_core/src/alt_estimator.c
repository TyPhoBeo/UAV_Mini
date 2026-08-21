#include "flight_core/alt_estimator.h"

#include <math.h>

static const float GRAVITY_MS2 = 9.81f;

// reset_flight_state() — phần state phụ thuộc MỐC TOẠ ĐỘ (Z/Vz/shadow/fusion
// baro). Dùng chung bởi reset() và reanchor() để hai đường không bao giờ lệch
// nhau khi thêm field mới.
static void reset_flight_state(alt_estimator_t *e) {
    e->alt_m = 0.0f;
    e->vz_ms = 0.0f;
    e->valid = false;
    e->degraded = true;              // chưa có correction nào -> chưa đáng tin
    e->last_correction_us = 0;
    e->no_correction_ms = 0;

    e->vz_accel_only_ms = 0.0f;
    e->z_inertial_m = 0.0f;

    // Nguon dang giu Z + luong correction gan nhat: deu gan lien voi MOC toa do
    // vua bi doi, nen phai xoa cung luc. Bo sot thi sau reanchor() telemetry se
    // bao mot nguon/luong sua cua he toa do CU.
    e->last_tof_accept_us = 0;
    e->last_baro_accept_us = 0;
    e->active_source = ALT_SRC_NONE;
    e->tof_corr_z_m = 0.0f;
    e->tof_corr_vz_ms = 0.0f;
    e->baro_corr_z_m = 0.0f;
    e->baro_corr_vz_ms = 0.0f;

    // Residual lai bias: doi moc = residual cu vo nghia. KHONG dung toi
    // accel_bias_ms2 o day -- bias la thuoc tinh CUA CAM BIEN, khong phai cua
    // he toa do, va no la thu dat nhat da hoc duoc (alt_estimator_reset() moi
    // xoa han).
    e->bias_residual_m = 0.0f;
    e->bias_residual_init = false;

    e->baro_med_hist[0] = e->baro_med_hist[1] = e->baro_med_hist[2] = 0.0f;
    e->baro_med_count = 0;
    e->baro_med_idx = 0;
    e->baro_lpf_alt_m = 0.0f;
    e->baro_lpf_init = false;

    e->last_processed_baro_seq = 0;
    e->baro_seq_init = false;
    e->last_baro_timestamp_us = 0;
    e->last_baro_received_us = 0;
    e->baro_fusion_initialized = false;
    e->baro_new_sample = false;
    e->baro_innovation_m = 0.0f;
    e->baro_dt_s = 0.0f;
    e->baro_accept_count = 0;
    e->baro_reject_count = 0;

    e->baro_reject_consecutive = 0;
    e->baro_disagreement_active = false;
    e->baro_disagreement_since_us = 0;
    e->baro_disagreement_sign = 0.0f;
    e->baro_reacquire_active = false;

    // ToF: phần phụ thuộc MỐC TOẠ ĐỘ. floor_plane_z_m và tof_ground_range_m
    // KHÔNG nằm ở đây — chúng là "cấu hình vật lý của chuyến bay" (chốt lúc
    // ARM/takeoff), không phải state trôi theo thời gian. Xoá chúng ở reset sẽ
    // làm mất mốc sàn giữa chừng và ToF không còn phân biệt được sàn với bàn.
    e->tof_vertical_m = 0.0f;
    e->tof_innovation_m = 0.0f;
    e->tof_surface_z_m = 0.0f;
    e->tof_surface_state = ALT_EST_TOF_SURFACE_UNKNOWN;
    e->tof_floor_ticks = 0;
    e->tof_other_ticks = 0;
    e->tof_correction_enabled = false;
    e->last_processed_tof_seq = 0;
    e->tof_seq_init = false;
    e->last_tof_timestamp_us = 0;
    e->last_tof_received_us = 0;
    e->tof_new_sample = false;
    e->tof_dt_s = 0.0f;
    e->tof_accept_count = 0;
    e->tof_reject_count = 0;
    e->landing_surface_z_m = 0.0f;
    e->landing_surface_valid = false;
}

void alt_estimator_reset(alt_estimator_t *e) {
    reset_flight_state(e);

    // Chỉ reset() mới xoá bias + filter accel — reanchor() KHÔNG (xem
    // alt_estimator.h): đổi mốc baro không làm sai bias gia tốc đã học.
    e->accel_bias_ms2 = 0.0f;
    e->bias_adapt_count = 0;   // bo dem tron doi, chi xoa o reset TOAN PHAN
    e->az_raw_ms2 = 0.0f;
    e->az_lpf_ms2 = 0.0f;
    e->az_corrected_ms2 = 0.0f;
    e->az_lpf_init = false;

    e->was_inertial_enabled = false;

    // CHỈ reset() mới xoá mốc sàn + ground ref ToF (reset = "quên hết, bắt đầu
    // lại"), reanchor()/prepare_takeoff() thì KHÔNG.
    e->floor_plane_z_m = 0.0f;
    e->tof_ground_range_m = 0.0f;
    e->tof_ground_ref_valid = false;
    e->tof_raw_m = 0.0f;
    e->tof_tilt_cos = 1.0f;
}

void alt_estimator_prepare_takeoff(alt_estimator_t *e) {
    // Zero phần phụ thuộc MỐC (Z/Vz + 2 shadow). Trong SPOOL ground lock cũng
    // ép các số này về 0 mỗi tick, nên phần này là belt-and-braces — GIÁ TRỊ
    // THẬT của hàm nằm ở khối dưới.
    e->alt_m = 0.0f;
    e->vz_ms = 0.0f;
    e->z_inertial_m = 0.0f;
    e->vz_accel_only_ms = 0.0f;

    // Xoá trạng thái BẤT ĐỒNG/REACQUIRE cũ. Ground lock KHÔNG chạm mấy field
    // này, nên nếu chuyến trước kết thúc giữa lúc đang REACQUIRE thì chuyến sau
    // sẽ khởi động với alpha/beta LỚN ngay từ mẫu baro đầu tiên sau liftoff —
    // đúng lúc sát đất baro nhiễu nhất. Đó là một cú kéo Z mạnh không ai đặt.
    e->baro_reject_consecutive = 0;
    e->baro_disagreement_active = false;
    e->baro_disagreement_since_us = 0;
    e->baro_disagreement_sign = 0.0f;
    e->baro_reacquire_active = false;

    // CỐ Ý GIỮ (khác hẳn reset()/reanchor()):
    //   valid                   -> mốc baro KHÔNG đổi ở đây (calib đã chạy lúc
    //                              ARM), nên không có lý do gì tuyên bố mất
    //                              estimator. Đặt false ở đây sẽ làm chính
    //                              guard "alt_estimator hợp lệ" của CMD_TAKEOFF
    //                              thất bại — tự chặn takeoff của mình.
    //   accel_bias_ms2          -> đã học cả lúc ARMED đứng yên, đúng cái ta cần
    //                              nhất ngay trước khi tích phân.
    //   az_raw/az_lpf/az_lpf_init -> filter accel không liên quan mốc toạ độ.
    //   baro prefilter + seq + fusion_initialized -> mốc baro không đổi.
}

void alt_estimator_reanchor(alt_estimator_t *e) {
    reset_flight_state(e);
    // was_inertial_enabled KHÔNG reset: reanchor chỉ được gọi lúc còn trên đất
    // (CMD_TAKEOFF/CMD_CALIB_BARO_GROUND, xem flight_core.c) nên nó vốn đã
    // false; ép lại cũng không sai nhưng để nguyên giữ đúng ngữ nghĩa "hàm này
    // chỉ đụng tới thứ phụ thuộc mốc toạ độ".
}

// lpf_alpha() — hệ số LPF bậc 1 chuẩn (khớp công thức PID_D_LPF_HZ trong
// pid.c): alpha = dt/(dt + RC), RC = 1/(2*pi*fc).
static float lpf_alpha(float dt, float cutoff_hz) {
    const float rc = 1.0f / (2.0f * 3.14159265f * cutoff_hz);
    return dt / (dt + rc);
}

// median3() — median-of-3, chống đúng 1 mẫu spike mà không thêm latency đáng kể.
static float median3(float a, float b, float c) {
    if (a > b) { const float t = a; a = b; b = t; }
    if (b > c) { const float t = b; b = c; c = t; }
    if (a > b) { const float t = a; a = b; b = t; }
    return b;
}

// baro_prefilter() — median-of-3 rồi LPF. dt_baro_s PHẢI là chu kỳ THẬT giữa 2
// mẫu baro (~20ms), KHÔNG phải dt tick điều khiển (~4ms). Chạy cả khi GROUND
// để telemetry BALT/BFILT sống và bộ lọc luôn "nóng" khi cần dùng thật.
static float baro_prefilter(alt_estimator_t *e, float baro_alt_m, float dt_baro_s) {
    e->baro_med_hist[e->baro_med_idx] = baro_alt_m;
    e->baro_med_idx = (e->baro_med_idx + 1) % 3;
    if (e->baro_med_count < 3) e->baro_med_count++;

    const float med = (e->baro_med_count < 3)
        ? baro_alt_m   // chưa đủ 3 mẫu -> dùng thẳng, tránh trễ khởi động
        : median3(e->baro_med_hist[0], e->baro_med_hist[1], e->baro_med_hist[2]);

    if (!e->baro_lpf_init) {
        e->baro_lpf_alt_m = med;
        e->baro_lpf_init = true;
    } else {
        e->baro_lpf_alt_m += lpf_alpha(dt_baro_s, ALT_EST_BARO_LPF_HZ) * (med - e->baro_lpf_alt_m);
    }
    return e->baro_lpf_alt_m;
}

// baro_correct_airborne() — toàn bộ nhánh fusion baro, CHỈ gọi khi airborne VÀ
// đã xác nhận có mẫu MỚI. Tách hàm để nhánh phân loại (a)/(b)/(c)/(d) đọc
// được liền mạch, xem alt_estimator.h "CORRECT".
// apply_vz_correction() — phần sửa Vz của alpha-beta, ĐÃ CHẶN TRẦN.
// beta*innovation/dt_baro khuếch đại innovation ~50 lần (dt_baro~0.02s) nên
// phải có trần cứng, đặc biệt ở REACQUIRE nơi innovation vượt gate theo định
// nghĩa — xem ALT_EST_BARO_VZ_CORRECTION_MAX_MS.
static void apply_vz_correction(alt_estimator_t *e, float beta, float innovation,
                                 float dt_baro_s) {
    const float dvz = clampf(beta * innovation / dt_baro_s,
                              -ALT_EST_BARO_VZ_CORRECTION_MAX_MS,
                              ALT_EST_BARO_VZ_CORRECTION_MAX_MS);
    e->vz_ms += dvz;
    e->baro_corr_vz_ms = dvz;   // telemetry: DA SUA bao nhieu (khac innovation)
}

static void baro_correct_airborne(alt_estimator_t *e, float baro_filtered,
                                   float dt_baro_s, int64_t now_us) {
    const float innovation = baro_filtered - e->alt_m;
    e->baro_innovation_m = innovation;

    if (!e->baro_fusion_initialized) {
        // (a) Cold start THẬT (sau alt_estimator_reset() giữa không trung do
        // attitude mất hợp lệ) — chưa có gốc toạ độ chung nào để so, snap.
        // Đây KHÔNG phải đường đi của takeoff bình thường: ở đó fusion đã được
        // đánh dấu initialized ngay tại cạnh ground->inertial, nơi Z=0 và baro
        // ref=0 vốn đã trùng nhau (xem alt_estimator.h).
        e->alt_m = baro_filtered;
        e->baro_fusion_initialized = true;
        e->baro_accept_count++;
        e->last_correction_us = now_us;
        e->last_baro_accept_us = now_us;   // -> alt_source_t (xem alt_estimator.h)
        e->baro_reject_consecutive = 0;
        e->baro_disagreement_active = false;
        e->baro_reacquire_active = false;
        return;
    }

    if (fabsf(innovation) <= ALT_EST_BARO_INNOVATION_GATE_M) {
        // (b) Trong gate: correction thường. Estimator và baro đang đồng
        // thuận -> xoá sạch mọi trạng thái bất đồng.
        e->baro_corr_z_m = ALT_EST_BARO_ALPHA * innovation;
        e->alt_m += e->baro_corr_z_m;
        apply_vz_correction(e, ALT_EST_BARO_BETA, innovation, dt_baro_s);
        e->baro_accept_count++;
        e->last_correction_us = now_us;
        e->last_baro_accept_us = now_us;   // -> alt_source_t (xem alt_estimator.h)
        e->baro_reject_consecutive = 0;
        e->baro_disagreement_active = false;
        e->baro_reacquire_active = false;
        return;
    }

    // Ngoài gate — phân biệt spike NGẮN với bất đồng DAI DẲNG cùng dấu.
    const float sign = (innovation > 0.0f) ? 1.0f : -1.0f;
    if (!e->baro_disagreement_active || sign != e->baro_disagreement_sign) {
        e->baro_disagreement_active = true;
        e->baro_disagreement_since_us = now_us;
        e->baro_disagreement_sign = sign;
    }
    const int64_t disagreement_ms = (now_us - e->baro_disagreement_since_us) / 1000;

    if (disagreement_ms >= (int64_t)ALT_EST_BARO_REACQUIRE_DISAGREEMENT_MS) {
        // (d) REACQUIRE — nghi INERTIAL DRIFT chứ không phải baro lỗi. Chấp
        // nhận correction dù ngoài gate, alpha/beta lớn hơn nhưng vẫn không
        // snap. Đây là lối thoát khỏi deadlock "trôi -> reject -> trôi thêm".
        e->baro_reacquire_active = true;
        e->baro_corr_z_m = ALT_EST_BARO_REACQUIRE_ALPHA * innovation;
        e->alt_m += e->baro_corr_z_m;
        apply_vz_correction(e, ALT_EST_BARO_REACQUIRE_BETA, innovation, dt_baro_s);
        e->baro_accept_count++;
        e->last_correction_us = now_us;
        e->last_baro_accept_us = now_us;   // -> alt_source_t (xem alt_estimator.h)
        e->baro_reject_consecutive = 0;
    } else {
        // (c) Spike ngắn / chưa đủ lâu để tin — reject, dead-reckon tiếp.
        e->baro_reject_count++;
        e->baro_reject_consecutive++;
    }
}


// ============================================================================
// ToF — CORRECTION CO DIEU KIEN THEO BE MAT
// ============================================================================
// Bai toan: VL53L0X do range toi BE MAT DANG O DUOI. No khong biet do la san,
// ban hay ghe. Neu coi no la world-Z thi bay qua ban cao 40cm se khien
// controller tuong UAV vua tut 40cm va day ga — UAV tu boc len dung 40cm.
//
// Cach giai: KHONG so ToF voi mot nguong co dinh. So no voi RANGE DU DOAN toi
// MAT SAN DA KHOA, tinh tu chinh world-Z quan tinh:
//
//   predicted_floor_range = (z_uav - floor_plane_z) + tof_ground_range
//   innovation            = tof_vertical - predicted_floor_range
//
// Dieu nay lam case "UAV ha do cao that" TU DUNG ma khong can logic rieng:
// khi UAV tut 40cm that thi z_uav tut 40cm (quan tinh thay), predicted_range
// tut 40cm, ToF cung tut 40cm -> innovation ~ 0 -> van FLOOR.
// Con khi bay ngang len mat ban: z_uav KHONG doi, predicted_range KHONG doi,
// nhung ToF tut 40cm -> innovation ~ -40cm -> OTHER.
// Cung mot con so ToF, hai ket luan khac nhau, phan biet bang chuyen dong
// quan tinh. Do la ly do so voi DU DOAN chu khong so voi NGUONG.

// tof_vertical_from_range() — chieu range theo truc sensor ve phuong thang dung.
// tilt_cos = phan tu (2,2) cua ma tran xoay body->world = chieu body -Z len
// world -Z. DUNG LAI DUNG he so da dung cho az (1 - 2x^2 - 2y^2) de hai cho
// khong bao gio lech quy uoc he truc.
static float tof_vertical_from_range(float range_m, float tilt_cos) {
    return range_m * tilt_cos;
}

// tof_classify_surface() — cap nhat may trang thai UNKNOWN/FLOOR/OTHER.
//
// KHONG CO DUONG NAO DOI TRANG THAI THEO THOI GIAN. Day la yeu cau cot loi:
// mot cai ban co the dai 30 giay, 60 giay, hay ca chuyen bay — no van la ban.
// Chi co BANG CHUNG VAT LY (innovation khop lai mat san da khoa) moi dua duoc
// trang thai ve FLOOR. So sanh voi baro REACQUIRE (CO yeu to thoi gian): baro
// do world-Z nen "bat dong keo dai" ham y estimator troi; ToF do BE MAT nen
// "bat dong keo dai" chi ham y be mat do van con o do.
static void tof_classify_surface(alt_estimator_t *e, float innovation) {
    const bool matches_floor = fabsf(innovation) <= ALT_EST_TOF_FLOOR_MATCH_GATE_M;
    const bool higher_surface = (innovation <= -ALT_EST_TOF_SURFACE_STEP_GATE_M);

    if (e->tof_surface_state == ALT_EST_TOF_SURFACE_OTHER) {
        // Quay lai FLOOR KHO HON vao FLOOR lan dau: gate CHAT hon + can nhieu
        // mau lien tiep hon (hysteresis). Khong co cai nay thi bay doc mep ban
        // se nhap nhay FLOOR/OTHER va ToF correction bat/tat lien tuc.
        if (fabsf(innovation) <= ALT_EST_TOF_REACQUIRE_GATE_M) {
            if (e->tof_floor_ticks < ALT_EST_TOF_REACQUIRE_TICKS) e->tof_floor_ticks++;
        } else {
            e->tof_floor_ticks = 0;
        }
        if (e->tof_floor_ticks >= ALT_EST_TOF_REACQUIRE_TICKS) {
            e->tof_surface_state = ALT_EST_TOF_SURFACE_FLOOR;
            e->tof_other_ticks = 0;
        }
        return;
    }

    // UNKNOWN hoac FLOOR.
    if (higher_surface) {
        if (e->tof_other_ticks < ALT_EST_TOF_OTHER_TICKS) e->tof_other_ticks++;
    } else {
        e->tof_other_ticks = 0;
    }
    if (e->tof_other_ticks >= ALT_EST_TOF_OTHER_TICKS) {
        e->tof_surface_state = ALT_EST_TOF_SURFACE_OTHER;
        e->tof_floor_ticks = 0;
        return;
    }

    if (matches_floor) {
        if (e->tof_floor_ticks < ALT_EST_TOF_FLOOR_TICKS) e->tof_floor_ticks++;
    } else {
        e->tof_floor_ticks = 0;
    }
    if (e->tof_floor_ticks >= ALT_EST_TOF_FLOOR_TICKS) {
        e->tof_surface_state = ALT_EST_TOF_SURFACE_FLOOR;
    }
}

// correct_from_tof() — mot mau ToF MOI. Moi duong thoat deu de
// tof_correction_enabled=false (mac dinh an toan: tha khong sua con hon sua
// theo mot be mat sai).
static void correct_from_tof(alt_estimator_t *e, float range_m, float tilt_cos,
                              float dt_tof_s, bool inertial_enabled, int64_t now_us) {
    e->tof_raw_m = range_m;
    e->tof_tilt_cos = tilt_cos;
    e->tof_correction_enabled = false;

    // Ngoai dai tin duoc / nghieng qua / chua co ground ref -> KHONG phan loai,
    // KHONG sua. Giu nguyen surface_state cu: mat vai mau KHONG phai bang chung
    // de ket luan da roi khoi ban.
    if (range_m < ALT_EST_TOF_MIN_RANGE_M || range_m > ALT_EST_TOF_MAX_RANGE_M ||
        tilt_cos < ALT_EST_TOF_TILT_MIN_COS || !e->tof_ground_ref_valid) {
        e->tof_reject_count++;
        return;
    }

    const float vertical = tof_vertical_from_range(range_m, tilt_cos);
    e->tof_vertical_m = vertical;

    // world-Z cua be mat DANG nhin thay — luon tinh duoc, ke ca khi la ban.
    // Day la so dung cho telemetry + chon be mat ha canh, KHONG BAO GIO vao PID.
    e->tof_surface_z_m = (e->alt_m + e->tof_ground_range_m) - vertical;

    // Tren mat dat (ground lock) thi z_uav bi ep 0 va chua co gi de sua — chi
    // cap nhat so cho telemetry roi thoat.
    if (!inertial_enabled) {
        e->tof_innovation_m = 0.0f;
        return;
    }

    const float predicted_floor_range =
        (e->alt_m - e->floor_plane_z_m) + e->tof_ground_range_m;
    const float innovation = vertical - predicted_floor_range;
    e->tof_innovation_m = innovation;

    tof_classify_surface(e, innovation);

    // HAI dieu kien, CA HAI deu bat buoc:
    //   (1) surface_state == FLOOR  -> phan loai BEN VUNG noi day la mat san
    //   (2) |innovation| trong gate -> CHINH MAU NAY dong y voi mat san do
    //
    // (2) KHONG duoc bo. Bug da gap khi thieu no: trong cua so cho xac nhan
    // OTHER (OTHER_TICKS mau), state VAN con la FLOOR, nen correction van chay
    // voi innovation -0.40m. Voi alpha=0.25 va 3 mau, Z tut 0.80 -> 0.625
    // TRUOC KHI cai ban duoc nhan ra — dung kieu mat do cao ma ca task nay sinh
    // ra de ngan. Do la ly do phan loai (ben vung) va gate (tung mau) phai la
    // hai kiem tra RIENG BIET.
    if (e->tof_surface_state != ALT_EST_TOF_SURFACE_FLOOR ||
        fabsf(innovation) > ALT_EST_TOF_FLOOR_MATCH_GATE_M) {
        e->tof_reject_count++;
        return;
    }

    // Dang nhin DUNG mat san da khoa -> correction alpha-beta.
    // KHONG set thang z = do duoc: alpha < 1 de nhieu ToF khong thanh buoc
    // nhay Z, va de mot mau sai le khong keo duoc estimator di xa.
    e->tof_corr_z_m = ALT_EST_TOF_ALPHA * innovation;
    e->alt_m += e->tof_corr_z_m;
    const float dvz = clampf(ALT_EST_TOF_BETA * innovation / dt_tof_s,
                              -ALT_EST_TOF_VZ_CORRECTION_MAX_MS,
                              ALT_EST_TOF_VZ_CORRECTION_MAX_MS);
    e->vz_ms += dvz;
    e->tof_corr_vz_ms = dvz;   // telemetry
    e->tof_accept_count++;
    e->tof_correction_enabled = true;
    e->last_correction_us = now_us;   // nuôi watchdog degraded (xem VALIDITY)
    e->last_tof_accept_us = now_us;   // -> alt_source_t (xem alt_estimator.h)
}

void alt_estimator_set_tof_ground_ref(alt_estimator_t *e, float ground_range_m) {
    if (ground_range_m < ALT_EST_TOF_MIN_RANGE_M ||
        ground_range_m > ALT_EST_TOF_MAX_RANGE_M) {
        e->tof_ground_ref_valid = false;
        return;
    }
    e->tof_ground_range_m = ground_range_m;
    e->tof_ground_ref_valid = true;
}

void alt_estimator_lock_floor(alt_estimator_t *e) {
    // Mat san = world-Z HIEN TAI (0 luc cat canh, vi ground lock dang giu Z=0).
    e->floor_plane_z_m = e->alt_m;
    // Bat dau chuyen bay o UNKNOWN, KHONG phai FLOOR: de vai mau ToF dau tien
    // tu chung minh la dang nhin san. Neu UAV cat canh TU TREN mot cai ban thi
    // chinh cai ban do LA mat san cua chuyen bay nay — dung theo dinh nghia.
    e->tof_surface_state = ALT_EST_TOF_SURFACE_UNKNOWN;
    e->tof_floor_ticks = 0;
    e->tof_other_ticks = 0;
    e->landing_surface_valid = false;
}

bool alt_estimator_select_landing_surface(alt_estimator_t *e) {
    if (!e->tof_ground_ref_valid || e->tof_vertical_m <= 0.0f) {
        e->landing_surface_valid = false;
        return false;
    }
    // CO Y KHONG dung floor_plane_z_m: ha xuong mot cai ban khong duoc ghi de
    // moc san cua ca chuyen bay (floor != landing surface).
    e->landing_surface_z_m = e->tof_surface_z_m;
    e->landing_surface_valid = true;
    return true;
}

float alt_estimator_height_above_landing_surface(const alt_estimator_t *e) {
    if (!e->landing_surface_valid) return e->alt_m;
    return e->alt_m - e->landing_surface_z_m;
}

void alt_estimator_update(alt_estimator_t *e,
                           vec3f_t a, quat_t q,
                           bool baro_healthy, uint32_t baro_seq,
                           int64_t baro_timestamp_us, float baro_alt_m,
                           bool tof_healthy, uint32_t tof_seq,
                           int64_t tof_timestamp_us, float tof_range_m,
                           bool stationary, bool liftoff_candidate, bool airborne,
                           int64_t now_us, float dt) {
    // ---- PREDICT 1: accel body -> world Z, bỏ trọng lực ----
    // Hàng Z của ma trận xoay body->world (quaternion [w,x,y,z]) chấm accel.
    // Quy ước KHỚP mahony_filter.c (world "up" = {0,0,1}, accel body khi nằm
    // phẳng đọc +1g trên Z) -> world Z dương = LÊN. Trừ đúng +1.0.
    const float w = q.w, x = q.x, y = q.y, z = q.z;
    const float az_world_g =
        2.0f * (x * z - w * y) * a.x +
        2.0f * (y * z + w * x) * a.y +
        (1.0f - 2.0f * x * x - 2.0f * y * y) * a.z;

    e->az_raw_ms2 = (az_world_g - 1.0f) * GRAVITY_MS2;   // m/s²

    // ---- PREDICT 2: LPF (lớp phòng thủ thứ 2 sau DLPF phần cứng MPU6050).
    // LUÔN chạy kể cả GROUND — nuôi bộ học bias và giữ filter nóng. ----
    if (!e->az_lpf_init) {
        e->az_lpf_ms2 = e->az_raw_ms2;
        e->az_lpf_init = true;
    } else {
        e->az_lpf_ms2 += lpf_alpha(dt, ALT_EST_VERT_ACC_LPF_HZ) * (e->az_raw_ms2 - e->az_lpf_ms2);
    }

    // ---- PREDICT 3: HỌC BIAS TỪ az_lpf (TRƯỚC deadband!) ----
    // Thứ tự này là bản sửa lỗi: residual tĩnh thật nhỏ hơn deadband, nếu học
    // sau deadband thì bias luôn hội tụ về 0 và không bao giờ bù được gì (xem
    // alt_estimator.h "THỨ TỰ HỌC BIAS"). Chỉ học khi đứng yên thật; caller
    // đảm bảo stationary => chưa inertial-enabled.
    if (stationary) {
        e->accel_bias_ms2 += lpf_alpha(dt, ALT_EST_ACCEL_BIAS_LEARN_HZ) *
                             (e->az_lpf_ms2 - e->accel_bias_ms2);
        e->accel_bias_ms2 = clampf(e->accel_bias_ms2,
                                    -ALT_EST_ACCEL_BIAS_MAX_MS2, ALT_EST_ACCEL_BIAS_MAX_MS2);
    }

    // ---- PREDICT 4: trừ bias RỒI MỚI deadband ----
    float az_ms2 = e->az_lpf_ms2 - e->accel_bias_ms2;
    if (fabsf(az_ms2) < ALT_EST_VERT_ACC_DEADBAND_MS2) {
        az_ms2 = 0.0f;
    }
    e->az_corrected_ms2 = az_ms2;

    // ---- PHA: GROUND vs (CANDIDATE | AIRBORNE) ----
    const bool inertial_enabled = airborne || liftoff_candidate;

    if (inertial_enabled && !e->was_inertial_enabled) {
        // Cạnh GROUND -> inertial: ground lock vừa nhả. Ngay lúc này Z=0 là
        // ĐÚNG (drone còn trên đất) và mốc 0m của baro cũng là mặt đất đó
        // (calibrate_ground() chạy ngay trước takeoff) -> hai nguồn CÙNG gốc
        // toạ độ, fusion coi như đã khởi tạo, KHÔNG cần snap theo mẫu baro
        // tức thời (snap ở đây sẽ nuốt luôn nhiễu baro vào làm gốc).
        e->baro_fusion_initialized = true;
        e->baro_reject_consecutive = 0;
        e->baro_disagreement_active = false;
        e->baro_reacquire_active = false;
    }
    e->was_inertial_enabled = inertial_enabled;

    if (inertial_enabled) {
        // Tích phân "constant-acceleration" (chính xác hơn Euler thường).
        // KHÔNG reset gì ở cạnh CANDIDATE -> AIRBORNE: Policy B (continuity),
        // xem alt_estimator.h — Z/Vz lúc confirmed đã là độ cao/vận tốc THẬT.
        const float vz_old = e->vz_ms;
        e->vz_ms += az_ms2 * dt;
        e->alt_m += vz_old * dt + 0.5f * az_ms2 * dt * dt;

        const float vz_ao_old = e->vz_accel_only_ms;
        e->vz_accel_only_ms += az_ms2 * dt;
        e->z_inertial_m += vz_ao_old * dt + 0.5f * az_ms2 * dt * dt;
    } else {
        // GROUND: biết chắc chưa rời đất -> khoá cứng, KHÔNG tích phân gì.
        // Kể cả shadow tracker: mục đích của nó là lộ ra trôi KHI ĐANG BAY mà
        // không có baro, không phải trôi lúc còn nằm trên bàn.
        e->alt_m = 0.0f;
        e->vz_ms = 0.0f;
        e->z_inertial_m = 0.0f;
        e->vz_accel_only_ms = 0.0f;
    }

    // ---- CORRECT 1: ToF — CHỈ MỘT LẦN cho mỗi SEQUENCE mới ----
    // Chạy TRƯỚC baro có chủ đích: ToF chính xác cm và alpha mạnh hơn hẳn, nên
    // để nó đặt Z trước rồi baro chỉ tinh chỉnh phần còn lại. Đảo thứ tự thì
    // baro (nhiễu dm) kéo Z đi rồi ToF phải kéo về — thêm dao động vô ích.
    //
    // Cùng cơ chế seq như baro: estimator 250Hz, ToF ~30Hz -> một mẫu xuất hiện
    // ~8 tick liên tiếp trong snapshot. Correction 8 lần cho 1 mẫu = kéo Z về
    // ToF mạnh gấp 8 lần dự kiến.
    const bool tof_new = tof_healthy &&
                          (!e->tof_seq_init || tof_seq != e->last_processed_tof_seq) &&
                          (tof_seq != 0);
    e->tof_new_sample = tof_new;

    if (tof_new) {
        e->last_processed_tof_seq = tof_seq;
        e->tof_seq_init = true;
        e->last_tof_received_us = now_us;

        const float dt_tof_s = (e->last_tof_timestamp_us == 0)
            ? (1.0f / 30.0f)
            : clampf((float)(tof_timestamp_us - e->last_tof_timestamp_us) / 1e6f, 0.005f, 0.5f);
        e->last_tof_timestamp_us = tof_timestamp_us;
        e->tof_dt_s = dt_tof_s;

        // tilt_cos = phần tử (2,2) của ma trận xoay body->world — CÙNG hệ số đã
        // dùng cho az ở PREDICT 1, nên hai chỗ không bao giờ lệch quy ước trục.
        const float tilt_cos = 1.0f - 2.0f * x * x - 2.0f * y * y;
        correct_from_tof(e, tof_range_m, tilt_cos, dt_tof_s, inertial_enabled, now_us);
    } else if (e->last_tof_received_us != 0 &&
               (now_us - e->last_tof_received_us) > (int64_t)ALT_EST_TOF_TIMEOUT_MS * 1000) {
        // Mất ToF quá lâu -> ngừng coi là đang correction. KHÔNG hạ e->valid:
        // IMU mới là nguồn prediction chính, baro vẫn correction được (spec
        // "SENSOR FALLBACK"). Cũng KHÔNG đổi surface_state: mất tín hiệu không
        // phải bằng chứng đã rời khỏi bàn.
        e->tof_correction_enabled = false;
    }

    // ---- CORRECT: baro — CHỈ MỘT LẦN cho mỗi SEQUENCE mới ----
    // Nhận biết mẫu mới bằng seq (không bằng cờ healthy): estimator 250Hz,
    // baro ~50Hz -> cùng một mẫu sẽ xuất hiện ~5 tick liên tiếp trong
    // snapshot. Correction 5 lần cho 1 mẫu = kéo Z về baro mạnh gấp 5 lần dự
    // kiến (xem alt_estimator.h).
    const bool new_sample = baro_healthy &&
                             (!e->baro_seq_init || baro_seq != e->last_processed_baro_seq) &&
                             (baro_seq != 0);
    e->baro_new_sample = new_sample;

    if (new_sample) {
        e->last_processed_baro_seq = baro_seq;
        e->baro_seq_init = true;
        e->last_baro_received_us = now_us;   // "cảm biến còn sống" — KỂ CẢ nếu bị gate reject bên dưới

        // dt_baro từ HIỆU 2 TIMESTAMP BARO THẬT (~20ms @50Hz), KHÔNG phải dt
        // vòng điều khiển (~4ms) — beta/dt_baro sẽ sai 5 lần nếu dùng nhầm.
        const float dt_baro_s = (e->last_baro_timestamp_us == 0)
            ? (1.0f / 50.0f)
            : clampf((float)(baro_timestamp_us - e->last_baro_timestamp_us) / 1e6f, 0.005f, 0.5f);
        e->last_baro_timestamp_us = baro_timestamp_us;
        e->baro_dt_s = dt_baro_s;

        const float baro_filtered = baro_prefilter(e, baro_alt_m, dt_baro_s);

        if (airborne) {
            baro_correct_airborne(e, baro_filtered, dt_baro_s, now_us);
        } else {
            // GROUND/CANDIDATE: KHÔNG correction.
            //  - GROUND: ground truth Z=0 đã biết chắc, không cần baro "tranh cãi".
            //  - CANDIDATE: Z/Vz đang được tích phân CHÍNH ĐỂ chứng minh drone
            //    rời đất; kéo nó về baro (vốn còn đang đọc ~0 vì drone mới chỉ
            //    nhấc vài cm và baro rất nhiễu ở sát đất) sẽ triệt tiêu đúng
            //    tín hiệu mà detector cần -> quay lại đúng bug vòng tròn.
            // Vẫn tính innovation để telemetry nhìn được baro lệch bao nhiêu.
            e->baro_innovation_m = baro_filtered - e->alt_m;
        }
    }

    // ========================================================================
    // VALIDITY (state dùng được) vs DEGRADED (state còn đáng tin)
    // ========================================================================
    // valid KHÔNG phụ thuộc baro/ToF nữa. IMU là nguồn prediction chính; caller
    // đã gọi alt_estimator_reset() khi attitude mất hợp lệ, nên tới được đây
    // nghĩa là pipeline quán tính đang chạy. Chỉ còn phải canh số có hữu hạn
    // không — NaN/Inf lọt vào Z/Vz sẽ lan thẳng vào PID rồi ra mixer.
    const bool finite_state = isfinite(e->alt_m) && isfinite(e->vz_ms) &&
                               isfinite(e->accel_bias_ms2);
    if (!finite_state) {
        // Số hỏng thật -> vứt, KHÔNG để lan xuống controller.
        e->alt_m = 0.0f;
        e->vz_ms = 0.0f;
        e->accel_bias_ms2 = 0.0f;
        e->valid = false;
    } else {
        e->valid = true;
    }

    // DEGRADED: đã bao lâu không có correction nào (ToF HOẶC baro). Ground lock
    // đang giữ Z=0 thì không cần correction để tin -> không degraded.
    if (!inertial_enabled) {
        e->last_correction_us = now_us;   // giữ watchdog "no" trong lúc còn ở đất
        e->no_correction_ms = 0;
        e->degraded = false;
    } else if (e->last_correction_us == 0) {
        // Vừa mở dynamics mà chưa có correction nào — bắt đầu đếm từ đây thay
        // vì tính từ mốc 0 (sẽ ra một con số khổng lồ vô nghĩa ngay tick đầu).
        e->last_correction_us = now_us;
        e->no_correction_ms = 0;
        e->degraded = false;
    } else {
        const int64_t age_us = now_us - e->last_correction_us;
        e->no_correction_ms = (int32_t)(age_us / 1000);
        e->degraded = age_us > (int64_t)ALT_EST_NO_CORRECTION_DEGRADED_MS * 1000;
    }

    // ---- HOC BIAS ACCEL TU RESIDUAL DO CAO (duong THU HAI) ----
    // Nhanh `stationary` o dau ham chi hoc duoc khi drone DUNG YEN, tuc la chi
    // luc con tren dat. Bias accel Z troi theo NHIET DO, ma nhiet do doi nhieu
    // nhat CHINH LUC DANG BAY. Khong co duong nay thi sai so DC cua Az khong
    // bao gio duoc sua luc bay, va Z troi deu mot chieu suot chuyen bay.
    //
    // Nguon residual: innovation cua mau ABSOLUTE VUA DUOC CHAP NHAN o tick
    // nay. Dung mau bi reject la hoc theo outlier -- dung thu phai tranh nhat.
    {
        float residual = 0.0f;
        bool have_residual = false;
        // ToF duoc uu tien khi con trong tam: no do TRUC TIEP toi be mat, khong
        // troi theo thoi tiet nhu baro.
        if (e->tof_new_sample && e->last_tof_accept_us == now_us) {
            residual = e->tof_innovation_m;
            have_residual = true;
        } else if (e->baro_new_sample && e->last_baro_accept_us == now_us) {
            residual = e->baro_innovation_m;
            have_residual = true;
        }

        // cos goc nghieng lay tu hang 3 cua ma tran xoay (cung cong thuc voi
        // phep chieu az o dau ham) -- KHONG tinh lai bang atan2.
        const float tilt_cos = 1.0f - 2.0f * x * x - 2.0f * y * y;

        const bool gate_ok =
            inertial_enabled &&                                        // dang bay
            have_residual &&                                           // co mau THAT, da qua gate
            (fabsf(residual) <= ALT_EST_BIAS_ADAPT_MAX_RESIDUAL_M) &&  // khong phai va cham/doi be mat
            (fabsf(e->az_corrected_ms2) <= ALT_EST_BIAS_ADAPT_MAX_AZ_MS2) &&  // khong dang tang/giam toc manh
            (tilt_cos >= ALT_EST_BIAS_ADAPT_MIN_TILT_COS);             // khong nghieng nhieu

        if (!inertial_enabled) {
            // Con tren dat: xoa bo loc de lan bay sau bat dau sach, khong mang
            // theo residual cu cua lan truoc.
            e->bias_residual_m = 0.0f;
            e->bias_residual_init = false;
        } else if (gate_ok) {
            if (!e->bias_residual_init) {
                e->bias_residual_m = residual;
                e->bias_residual_init = true;
            } else {
                e->bias_residual_m += lpf_alpha(dt, ALT_EST_BIAS_RESIDUAL_LPF_HZ) *
                                       (residual - e->bias_residual_m);
            }
            // DAU TRU: bias qua cao -> az qua thap -> Z tut lai -> innovation
            // DUONG -> phai GIAM bias. Xem giai thich day du trong alt_estimator.h.
            e->accel_bias_ms2 -= ALT_EST_BIAS_FROM_RESIDUAL_K * e->bias_residual_m * dt;
            e->accel_bias_ms2 = clampf(e->accel_bias_ms2,
                                        -ALT_EST_ACCEL_BIAS_MAX_MS2,
                                        ALT_EST_ACCEL_BIAS_MAX_MS2);
            if (e->bias_adapt_count < UINT32_MAX) e->bias_adapt_count++;
        }
        // gate_ok=false MA dang bay: GIU NGUYEN bo loc, khong xoa. Mot doan
        // nghieng/tang toc ngan khong duoc lam mat toan bo phan da hoc.
    }

    // ---- NGUON NAO DANG GIU Z (xem alt_source_t trong alt_estimator.h) ----
    // Dua tren thoi diem ACCEPT gan nhat cua TUNG nguon, KHONG dua tren health:
    // mot ToF van tra mau deu nhung bi innovation gate loai sach thi dong gop
    // cua no cho Z bang 0, du tof_healthy van = 1. Day chinh la truong hop can
    // nhin thay nhat.
    //
    // Con tren mat dat (ground lock) thi Z=0 la su that da biet, khong can
    // correction nao -> bao NONE moi dung, thay vi bao mot nguon "dang giu".
    {
        const int64_t win_us = (int64_t)ALT_EST_SOURCE_ACTIVE_MS * 1000;
        const bool tof_live = inertial_enabled && (e->last_tof_accept_us != 0) &&
                               ((now_us - e->last_tof_accept_us) <= win_us);
        const bool baro_live = inertial_enabled && (e->last_baro_accept_us != 0) &&
                                ((now_us - e->last_baro_accept_us) <= win_us);
        e->active_source = tof_live ? (baro_live ? ALT_SRC_BOTH : ALT_SRC_TOF)
                                     : (baro_live ? ALT_SRC_BARO : ALT_SRC_NONE);
    }
}

void alt_estimator_correct_velocity(alt_estimator_t *e, float vz_measured_ms, bool measurement_valid, float dt) {
    // CHỪA INTERFACE cho nguồn correction vận tốc tương lai (optical flow —
    // xem alt_estimator.h). No-op có chủ đích: KHÔNG được âm thầm sửa e->vz_ms
    // bằng công thức phỏng đoán khi chưa có nguồn đo thật đứng sau.
    (void)e;
    (void)vz_measured_ms;
    (void)measurement_valid;
    (void)dt;
}
