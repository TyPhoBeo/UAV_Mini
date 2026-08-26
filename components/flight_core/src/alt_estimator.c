#include "flight_core/alt_estimator.h"

#include <math.h>
#include <string.h>

#include "flight_core/fc_features.h"   // FC_FEATURE_FLOOR_GATE

static const float GRAVITY_MS2 = 9.80665f;
static const float TWO_PI = 6.28318530718f;

static float lpf_alpha(float hz, float dt) {
    const float x = TWO_PI * hz * dt;
    return clampf(x / (1.0f + x), 0.0f, 1.0f);
}

static void clear_runtime(alt_estimator_t *e, bool keep_floor) {
    alt_estimator_t old = *e;
    memset(e, 0, sizeof(*e));
    e->tof_tilt_cos = 1.0f;
    e->tof_track_state = ALT_TOF_LOST;
    e->active_source = ALT_SRC_GROUND_LOCK;
    e->tof_surface_state = ALT_EST_TOF_SURFACE_UNKNOWN;
    // terrain_off_m/terr_* CO Y ve 0 o day (memset da lam) va KHONG nam trong
    // keep_floor: keep_floor giu MAT SAN (thu do luc nam dat), con terrain la
    // trang thai BAY. clear_runtime() chi chay khi Z/Vz da hong hoac khi
    // unlock — ca hai deu la luc moi tham chieu terrain da mat nghia.
    if (keep_floor) {
        e->floor_mean_m = old.floor_mean_m;
        e->floor_m2_m2 = old.floor_m2_m2;
        e->floor_std_m = old.floor_std_m;
        e->floor_sample_count = old.floor_sample_count;
        e->tof_ground_range_m = old.tof_ground_range_m;
        e->tof_ground_ref_valid = old.tof_ground_ref_valid;
        e->floor_locked = old.floor_locked;
    }
}

void alt_estimator_reset(alt_estimator_t *e) { if (e) clear_runtime(e, false); }

bool alt_estimator_floor_ready(const alt_estimator_t *e) {
    return e && e->tof_ground_ref_valid &&
           e->floor_sample_count >= ALT_EST_FLOOR_MIN_SAMPLES &&
           e->floor_std_m <= ALT_EST_FLOOR_MAX_STD_M;
}

// lock_floor_at() — phan THAN chung: chot goc toa do tai `ground_range_m`.
// Tach ra de duong chuan (trung binh cua so on dinh) va duong fallback (mau
// ToF hien tai) dung DUNG cung mot logic reset — hai ban sao se lech nhau.
static void lock_floor_at(alt_estimator_t *e, float ground_range_m) {
    e->floor_locked = true;
    e->tof_ground_range_m = ground_range_m;
    e->floor_plane_z_m = e->alt_m = e->vz_ms = 0.0f;
    e->active_source = ALT_SRC_GROUND_LOCK;
    // Chot datum TRUOC khi cat canh (B3): mat san vua khoa CHINH LA terrain
    // hien tai -> offset = 0 theo dinh nghia. Xoa sach ung vien/pending cua
    // chuyen bay truoc, neu khong mot bac dia hinh cu se song sot qua lan
    // takeoff moi va lam lech alt ngay tu tick dau.
    e->terrain_off_m = e->terr_cand_offset_m = e->terr_residual_m = 0.0f;
    e->terr_pending = false;
    e->terr_confirm_cnt = 0;
    e->agl_m = 0.0f;
    e->prev_tof_vertical_valid = false;
}

bool alt_estimator_lock_floor(alt_estimator_t *e) {
    if (!alt_estimator_floor_ready(e)) return false;
    lock_floor_at(e, e->floor_mean_m);
    return true;
}

// alt_estimator_lock_floor_fallback() — chot goc toa do bang mau ToF hop le
// HIEN TAI thay vi trung binh cua so on dinh.
//
// CHI dung khi cong floor da tat (FC_FEATURE_FLOOR_GATE=0, xem fc_features.h).
// Kem chinh xac hon duong chuan — mot mau don le mang ca nhieu cua cam bien —
// nhung van HON HAN viec giu nguyen tof_ground_range_m cu: gia tri cu la goc
// toa do cua LAN BAY TRUOC (hoac 0 neu chua tung bay), tuc alt_m se sai ngay
// tu tick dau tien va sai mot luong khong ai biet truoc.
//
// Tra false khi khong co ca mau ToF nao dung duoc -> caller phai tu choi lenh:
// khong co goc toa do nghia la khong biet minh dang o do cao nao.
bool alt_estimator_lock_floor_fallback(alt_estimator_t *e) {
    if (!e || !e->tof_ground_ref_valid) return false;
    // tof_vertical_m = range da bu nghieng (xem update()). >0 moi co nghia.
    if (!(e->tof_vertical_m > 0.0f) || !isfinite(e->tof_vertical_m)) return false;
    lock_floor_at(e, e->tof_vertical_m);
    return true;
}

void alt_estimator_unlock_floor(alt_estimator_t *e) {
    if (e) clear_runtime(e, false);
}

void alt_estimator_prepare_takeoff(alt_estimator_t *e) {
    if (!e) return;
    e->alt_m = e->vz_ms = e->z_inertial_m = e->vz_accel_only_ms = 0.0f;
    e->landing_surface_valid = false;
    e->active_source = ALT_SRC_GROUND_LOCK;
    e->terrain_off_m = e->terr_cand_offset_m = e->terr_residual_m = 0.0f;
    e->terr_pending = false;
    e->terr_confirm_cnt = 0;
    e->agl_m = 0.0f;
    e->prev_tof_vertical_valid = false;
}

void alt_estimator_confirm_liftoff(alt_estimator_t *e) {
    if (!e || !e->floor_locked || !e->tof_fusable) return;
    e->alt_m = fmaxf(0.0f, e->tof_z_m);
    e->agl_m = e->alt_m - e->terrain_off_m;
    e->vz_ms = e->tof_vz_valid ? e->tof_vz_lpf_ms : 0.0f;
    e->z_inertial_m = e->alt_m;
    e->vz_accel_only_ms = e->vz_ms;
    e->active_source = ALT_SRC_TOF_FUSED;
    e->valid = true;
    e->degraded = false;
}

bool alt_estimator_select_landing_surface(alt_estimator_t *e) {
    if (!e || !e->floor_locked) return false;
    e->landing_surface_z_m = 0.0f;
    e->landing_surface_valid = true;
    return true;
}

// LANDING chay tren AGL, KHONG chay tren datum — xem docstring o header.
// terrain_off_m = 0 (co bien dich tat, hoac dang o tren dung mat san) thi ham
// nay tra ve dung alt_m nhu ban cu.
float alt_estimator_height_above_landing_surface(const alt_estimator_t *e) {
    return alt_estimator_agl_m(e);
}

float alt_estimator_agl_m(const alt_estimator_t *e) {
    return e ? fmaxf(0.0f, e->alt_m - e->terrain_off_m) : 0.0f;
}

float alt_estimator_tof_agl_m(const alt_estimator_t *e) {
    return e ? fmaxf(0.0f, e->tof_z_m - e->terrain_off_m) : 0.0f;
}

bool alt_estimator_terrain_rebase(alt_estimator_t *e) {
#if FC_FEATURE_TERRAIN_OFFSET
    if (!e || !e->floor_locked || !e->tof_ground_ref_valid) return false;
    // Chi rebase tu mot mau ToF con dung duoc ve HINH HOC (khong doi
    // tof_fusable: dung luc guard ban thi fusable thuong DANG bi tat).
    if (!(e->tof_vertical_m >= ALT_EST_TOF_MIN_RANGE_M &&
          e->tof_vertical_m <= ALT_EST_TOF_MAX_RANGE_M)) return false;
    const float raw_agl = e->tof_vertical_m - e->tof_ground_range_m;
    e->terrain_off_m = e->alt_m - raw_agl;
    e->terr_pending = false;
    e->terr_confirm_cnt = 0;
    e->terr_cand_offset_m = e->terrain_off_m;
    e->terr_commit_count++;
    e->agl_m = raw_agl;
    return true;
#else
    (void)e;
    return false;
#endif
}

static void floor_add(alt_estimator_t *e, float v) {
    // Che do tam thoi chot mau hop le dau tien. Sau khi valid, giu nguyen floor
    // toi event unlock/reset de ToF reset/seq dut khong lam hoc lai mat san.
    if (e->tof_ground_ref_valid || e->floor_sample_count >= ALT_EST_FLOOR_MAX_SAMPLES) return;
    if (e->floor_sample_count && fabsf(v - e->floor_mean_m) > ALT_EST_FLOOR_RESTART_DELTA_M) {
        e->floor_sample_count = 0;
        e->floor_mean_m = e->floor_m2_m2 = 0.0f;
        e->tof_ground_ref_valid = false;
    }
    e->floor_sample_count++;
    const float d = v - e->floor_mean_m;
    e->floor_mean_m += d / (float)e->floor_sample_count;
    e->floor_m2_m2 += d * (v - e->floor_mean_m);
    e->floor_std_m = e->floor_sample_count > 1
        ? sqrtf(fmaxf(0.0f, e->floor_m2_m2 / (float)(e->floor_sample_count - 1))) : 0.0f;
    e->tof_ground_range_m = e->floor_mean_m;
    e->tof_ground_ref_valid = e->floor_sample_count >= ALT_EST_FLOOR_MIN_SAMPLES &&
                              e->floor_std_m <= ALT_EST_FLOOR_MAX_STD_M;
}

static void update_age(alt_estimator_t *e, bool airborne, int64_t now_us) {
    if (!airborne) {
        e->tof_track_state = e->tof_fusable ? ALT_TOF_TRACKING : ALT_TOF_LOST;
        return;
    }
    if (!e->last_tof_accept_us) {
        e->tof_track_state = ALT_TOF_LOST;
        e->tof_fusable = e->valid = false;
        e->degraded = true;
        e->active_source = ALT_SRC_TOF_LOST;
        return;
    }
    const int age = (int)((now_us - e->last_tof_accept_us) / 1000);
    e->no_correction_ms = age;
    if (age <= ALT_EST_TOF_TRACK_MAX_AGE_MS) {
        e->tof_track_state = ALT_TOF_TRACKING;
        e->valid = true; e->degraded = false; e->active_source = ALT_SRC_TOF_FUSED;
    } else if (age <= ALT_EST_TOF_BRIDGE_MS) {
        e->tof_track_state = ALT_TOF_BRIDGE;
        e->tof_fusable = false;
        e->valid = true; e->degraded = false; e->active_source = ALT_SRC_TOF_SHORT_BRIDGE;
    } else if (age < ALT_EST_TOF_LOST_MS) {
        e->tof_track_state = ALT_TOF_BRIDGE;
        e->tof_fusable = false;
        e->valid = true; e->degraded = true; e->active_source = ALT_SRC_IMU_PREDICT_ONLY;
    } else {
        e->tof_track_state = ALT_TOF_LOST;
        e->tof_fusable = e->valid = false;
        e->degraded = true; e->active_source = ALT_SRC_TOF_LOST;
    }
}

void alt_estimator_update(alt_estimator_t *e, vec3f_t a, quat_t q,
#if FC_FEATURE_BARO
                          bool baro_healthy, uint32_t baro_seq,
                          int64_t baro_timestamp_us, float baro_alt_m,
#endif
                          bool tof_healthy, uint32_t tof_seq,
                          int64_t tof_timestamp_us, float tof_range_m,
                          bool stationary, bool liftoff_candidate, bool airborne,
                          int64_t now_us, float dt) {
    if (!e || !(dt > 0.0f) || !isfinite(dt)) return;
#if FC_FEATURE_BARO
    (void)baro_healthy; (void)baro_seq; (void)baro_timestamp_us; (void)baro_alt_m;
    e->baro_corr_z_m = e->baro_corr_vz_ms = 0.0f;
    e->baro_accept_count = 0; e->baro_fusion_initialized = false;
#endif
    (void)liftoff_candidate;

    const float w=q.w, x=q.x, y=q.y, z=q.z;
    const float rzz = 1.0f - 2.0f*x*x - 2.0f*y*y;
    const float ezg = 2.0f*(x*z-w*y)*a.x + 2.0f*(y*z+w*x)*a.y + rzz*a.z;
    e->az_body_z_g = a.z;
    e->az_earth_raw_ms2 = ezg * GRAVITY_MS2;
    e->az_after_gravity_ms2 = (ezg - 1.0f) * GRAVITY_MS2;
    e->az_raw_ms2 = e->az_after_gravity_ms2;

    const bool tof_new = tof_healthy && tof_seq &&
        (!e->tof_seq_init || tof_seq != e->last_processed_tof_seq);
    e->tof_new_sample = e->tof_correction_enabled = false;
    e->tof_corr_z_m = e->tof_corr_vz_ms = 0.0f;
    if (tof_new) {
        e->tof_seq_init = true;
        e->last_processed_tof_seq = tof_seq;
        e->last_tof_received_us = now_us;
        e->tof_new_sample = true;
        e->tof_raw_m = tof_range_m;
        e->tof_tilt_cos = rzz;
        e->tof_dt_s = e->last_tof_timestamp_us
            ? (float)(tof_timestamp_us - e->last_tof_timestamp_us) / 1e6f : 0.0f;
        e->last_tof_timestamp_us = tof_timestamp_us;
        const bool geometry_ok = isfinite(tof_range_m) &&
            tof_range_m >= ALT_EST_TOF_MIN_RANGE_M && tof_range_m <= ALT_EST_TOF_MAX_RANGE_M &&
            rzz >= ALT_EST_TOF_TILT_MIN_COS;
        if (geometry_ok) {
            e->tof_vertical_m = tof_range_m * rzz;
            if (!e->floor_locked && stationary) floor_add(e, e->tof_vertical_m);
            if (e->tof_ground_ref_valid) {
                // AGL THO: do cao tren BE MAT dang nhin. Dai luong nay KHONG
                // phu thuoc terrain — no la thu cam bien thuc su do duoc.
                float raw_agl = e->tof_vertical_m - e->tof_ground_range_m;

#if FC_FEATURE_TERRAIN_OFFSET
                // ---- PHAT HIEN BAC TERRAIN BANG RESIDUAL (B4) ----
                // residual = phan thay doi range KHONG giai thich duoc bang
                // chuyen dong cua drone. Chi co nghia khi DANG BAY va hai mau
                // ke nhau du gan (gap lon thi vz*dt khong con la du doan tot).
                if (airborne && e->prev_tof_vertical_valid &&
                    e->tof_dt_s > 0.005f &&
                    e->tof_dt_s <= (float)ALT_EST_TOF_GAP_DERIV_MAX_MS / 1000.0f) {
                    const float d_range  = e->tof_vertical_m - e->prev_tof_vertical_m;
                    const float expected = e->vz_ms * e->tof_dt_s;   // leo -> range tang
                    const float residual = d_range - expected;
                    e->terr_residual_m = residual;
                    if (fabsf(residual) > TERR_JUMP_THRESH_M) {
                        e->terr_pending = true;
                        // Offset ung vien giu alt_datum LIEN TUC: bac lam
                        // raw_agl tut xuong bao nhieu thi offset bu len bay
                        // nhieu.
                        e->terr_cand_offset_m = e->terrain_off_m - residual;
                        e->terr_confirm_cnt = 0;
                    }
                }

                // ---- XAC NHAN N MAU TRUOC KHI COMMIT (B5) ----
                // So sanh do cao datum SE CO neu commit voi datum dang coast
                // (alt_m luc nay chua an correction cua mau nay). Mep ban lam
                // range nhay qua nhay lai -> mot mau lech la dem lai tu dau.
                if (e->terr_pending) {
                    const float cand_z = raw_agl + e->terr_cand_offset_m;
                    if (fabsf(cand_z - e->alt_m) < TERR_CONFIRM_TOL_M) {
                        if (++e->terr_confirm_cnt >= TERR_CONFIRM_N) {
                            e->terrain_off_m = e->terr_cand_offset_m;   // COMMIT
                            e->terr_pending = false;
                            e->terr_confirm_cnt = 0;
                            e->terr_commit_count++;
                        }
                    } else {
                        e->terr_confirm_cnt = 0;
                    }
                }
#endif  // FC_FEATURE_TERRAIN_OFFSET

                e->prev_tof_vertical_m = e->tof_vertical_m;
                e->prev_tof_vertical_valid = true;

                // Z tren SAN = AGL + terrain. terrain_off_m = 0 -> dung cong
                // thuc cu.
                float zt = raw_agl + e->terrain_off_m;
                if (zt < ALT_EST_GROUND_ZERO_BAND_M) zt = 0.0f;
                e->tof_z_m = e->tof_surface_z_m = zt;
                const bool deriv_ok = e->prev_tof_z_valid && e->tof_dt_s > 0.005f &&
                    e->tof_dt_s <= ALT_EST_TOF_GAP_DERIV_MAX_MS/1000.0f &&
                    fabsf(zt - e->prev_tof_z_m) <= ALT_EST_TOF_DERIV_JUMP_M;
                if (deriv_ok) {
                    e->tof_vz_ms = (zt - e->prev_tof_z_m) / e->tof_dt_s;
                    const float k = lpf_alpha(ALT_EST_TOF_VZ_LPF_HZ, e->tof_dt_s);
                    e->tof_vz_lpf_ms += k * (e->tof_vz_ms - e->tof_vz_lpf_ms);
                    e->tof_vz_valid = true;
                } else e->tof_vz_valid = false;
                e->prev_tof_z_m = zt;
                e->prev_tof_z_timestamp_us = tof_timestamp_us;
                e->prev_tof_z_valid = true;
                e->tof_innovation_m = zt - e->alt_m;
                e->tof_surface_gate_ok = !airborne || fabsf(e->tof_innovation_m) <= ALT_EST_TOF_INNOV_GATE_M;
#if FC_FEATURE_TERRAIN_OFFSET
                if (e->terr_pending) {
                    // ---- B6: TRONG luc nghi ngo ----
                    // KHONG an range (alt_m COAST bang tich phan accel), nhung
                    // cung KHONG dung tof_reacquire_count: commit xong la fuse
                    // lai NGAY, khong phai cho them 3 mau reacquire nua.
                    // I-term cua vong Vz do tang tren FREEZE (flight_core.c).
                    e->tof_fusable = false;
                    e->tof_surface_state = ALT_EST_TOF_SURFACE_OTHER;
                } else
#endif
                if (e->tof_surface_gate_ok) {
                    if (e->tof_track_state == ALT_TOF_LOST && airborne) {
                        if (e->tof_reacquire_count < UINT8_MAX) e->tof_reacquire_count++;
                    } else e->tof_reacquire_count = ALT_EST_TOF_REACQUIRE_SAMPLES;
                    e->tof_fusable = e->tof_reacquire_count >= ALT_EST_TOF_REACQUIRE_SAMPLES;
                    e->tof_surface_state = ALT_EST_TOF_SURFACE_FLOOR;
                } else {
                    e->tof_reacquire_count = 0; e->tof_fusable = false;
                    e->tof_surface_state = ALT_EST_TOF_SURFACE_OTHER;
                }
            }
        } else {
            e->tof_fusable = e->tof_vz_valid = e->prev_tof_z_valid = false;
            e->tof_reacquire_count = 0; e->tof_reject_count++;
            // Mau hong -> KHONG duoc lam moc cho residual cua mau ke tiep:
            // d_range tinh qua mot khoang trong se cho residual gia.
            e->prev_tof_vertical_valid = false;
        }
    }

    const bool bias_ok = (!airborne && stationary) ||
        (airborne && e->tof_fusable && e->tof_vz_valid &&
         fabsf(e->tof_vz_lpf_ms) < 0.15f && fabsf(e->tof_innovation_m) < 0.08f);
    if (bias_ok) {
        const float kb = clampf(ALT_EST_ACCEL_BIAS_RATE_HZ * dt, 0.0f, 0.01f);
        e->accel_bias_ms2 += kb * (e->az_after_gravity_ms2 - e->accel_bias_ms2);
        e->accel_bias_ms2 = clampf(e->accel_bias_ms2, -ALT_EST_ACCEL_BIAS_LIMIT_MS2,
                                   ALT_EST_ACCEL_BIAS_LIMIT_MS2);
        e->bias_adapt_count++;
    }
    e->az_after_bias_ms2 = e->az_after_gravity_ms2 - e->accel_bias_ms2;
    float az = fabsf(e->az_after_bias_ms2) < ALT_EST_ACCEL_DEADBAND_MS2 ? 0.0f : e->az_after_bias_ms2;
    if (!e->az_lpf_init) { e->az_lpf_ms2 = az; e->az_lpf_init = true; }
    else e->az_lpf_ms2 += lpf_alpha(ALT_EST_ACCEL_LPF_HZ, dt) * (az - e->az_lpf_ms2);
    e->az_corrected_ms2 = e->az_lpf_ms2;

    if (!airborne) {
        e->alt_m = e->vz_ms = e->z_inertial_m = e->vz_accel_only_ms = 0.0f;
        // Nam dat = dang o tren CHINH mat san da khoa: terrain phai la 0,
        // khong duoc mang bac cua chuyen bay truoc sang.
        e->terrain_off_m = 0.0f;
        e->terr_pending = false;
        e->terr_confirm_cnt = 0;
        e->agl_m = 0.0f;
        e->active_source = ALT_SRC_GROUND_LOCK;

        // Moc "lan cuoi co correction dung duoc" — refresh moi tick chung nao
        // mau ToF gan nhat con fusable. Phai chay TRUOC khi tinh valid ben duoi
        // vi valid gio doc chinh moc nay.
        if (e->tof_fusable) e->last_tof_accept_us = e->last_correction_us = now_us;

        // ====================================================================
        // LUOI DO TREN MAT DAT — CUNG NGUONG voi tren khong (xem update_age)
        // ====================================================================
        // TRUOC DAY: valid = tof_healthy, danh gia LAI moi tick, khong co do
        // tre nao. Nghia la MOT mau ToF xau duy nhat -> valid=false ->
        // commander thay alt_estimator_lost -> SOFT FAULT NGAY.
        //
        // Do la mot bat doi xung khong bien minh duoc: khi DANG BAY, cung su
        // kien do duoc cho 100ms TRACKING + 220ms BRIDGE + 300ms LOST truoc khi
        // bi coi la mat (update_age). Khi NAM DAT — dung luc motor vua len ga,
        // rung manh nhat, va ToF dang o cu ly ngan nhat/kho doc nhat — thi
        // KHONG duoc cho mot mili-giay nao.
        //
        // Hau qua da quan sat duoc: TAKEOFF vao PRIME, mot mau ToF loi giua
        // chung, abort ngay. `range_status != 0` la chuyen BINH THUONG voi ToF
        // (be mat hap thu, ngoai tam, anh nang) — chinh sensor_hub.h cung viet
        // vay — nen bat mot mau lam huy ca lan cat canh la sai ban chat.
        //
        // Gio dung DUNG nguong ALT_EST_TOF_LOST_MS cua tren khong. KHONG noi
        // hon: het 300ms ma van khong co mau fusable thi that su la mat ToF, va
        // luc do tu choi cat canh moi la dung.
        //
        // last_tof_accept_us == 0 (chua TUNG co mau nao dung duoc) -> KHONG hop
        // le, khong co gi de gia han. Do la truong hop ToF chet/khong hàn, phai
        // chan cat canh.
        const bool ground_tof_alive =
            e->last_tof_accept_us != 0 &&
            ((now_us - e->last_tof_accept_us) / 1000) < (int64_t)ALT_EST_TOF_LOST_MS;

        // tof_healthy VAN BAT BUOC o ca hai che do — khong co ToF thi khong co
        // gi de goi la do cao. Chi khac: gio no duoc phep TAM thoi false trong
        // cua so tren ma van con valid.
        //
        // FC_FEATURE_FLOOR_GATE=0 -> bo ve floor_ready. Neu giu lai thi cong
        // floor van chan gian tiep qua duong nay: TAKEOFF kiem !s_alt_est.valid
        // va se tu choi y het nhu cu, tat cong o flight_core.c thanh vo nghia.
#if FC_FEATURE_FLOOR_GATE
        e->valid = alt_estimator_floor_ready(e) && (tof_healthy || ground_tof_alive);
#else
        e->valid = tof_healthy || ground_tof_alive;
#endif
        // degraded = "con valid nhung dang song bang gia han, khong phai bang
        // mau tuoi". Commander CHI xet degraded khi airborne nen no khong gay
        // fault o day — nhung telemetry doc duoc, va do la thu can nhin thay
        // khi di tim vi sao mot lan cat canh bi tu choi.
        e->degraded = !e->valid || !tof_healthy;
        e->no_correction_ms = e->last_tof_accept_us
            ? (int)((now_us - e->last_tof_accept_us) / 1000) : 0;
        update_age(e, false, now_us);
        return;
    }

    e->alt_m += e->vz_ms*dt + 0.5f*e->az_corrected_ms2*dt*dt;
    e->vz_ms += e->az_corrected_ms2*dt;
    e->z_inertial_m += e->vz_accel_only_ms*dt + 0.5f*e->az_corrected_ms2*dt*dt;
    e->vz_accel_only_ms += e->az_corrected_ms2*dt;
    if (tof_new && e->tof_fusable) {
        const float iz = e->tof_z_m - e->alt_m;
        e->tof_corr_z_m = ALT_EST_TOF_Z_GAIN * iz;
        e->alt_m += e->tof_corr_z_m;
        float dv = e->tof_vz_valid ? ALT_EST_TOF_VZ_GAIN*(e->tof_vz_lpf_ms-e->vz_ms) : 0.0f;
        if (e->tof_dt_s > 0.005f)
            dv += clampf(ALT_EST_TOF_INNOV_VZ_GAIN*iz/e->tof_dt_s, -0.20f, 0.20f);
        e->tof_corr_vz_ms = dv;
        e->vz_ms += dv;
        e->tof_accept_count++;
        e->tof_correction_enabled = true;
        e->last_tof_accept_us = e->last_correction_us = now_us;
        e->tof_track_state = ALT_TOF_TRACKING;
    } else if (tof_new) e->tof_reject_count++;

    if (!isfinite(e->alt_m) || !isfinite(e->vz_ms)) {
        clear_runtime(e, true);
        e->active_source = ALT_SRC_TOF_LOST;
        return;
    }
    // AGL loc — dan xuat DUY NHAT tu (alt_m, terrain_off_m). Moi noi can "cao
    // bao nhieu so voi be mat" phai doc field nay, khong tu tru lai.
    e->agl_m = e->alt_m - e->terrain_off_m;
    update_age(e, true, now_us);
}

void alt_estimator_correct_velocity(alt_estimator_t *e, float v, bool ok, float dt) {
    if (!e || !ok || !(dt > 0.0f) || !isfinite(v)) return;
    e->vz_ms += clampf(0.10f*(v-e->vz_ms), -0.15f, 0.15f);
}
