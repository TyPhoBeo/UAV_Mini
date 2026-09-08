#include "flight_core/alt_estimator.h"

#include <math.h>
#include <string.h>

#include "flight_core/fc_features.h"   // FC_FEATURE_FLOOR_GATE

// terr_pending ep tof_fusable=false -> khong co correction. Neu no keo dai
// qua ALT_EST_NO_CORRECTION_DEGRADED_MS thi commander SOFT FAULT -> LANDING.
// Vi vay timeout cua pending PHAI ngan hon ca hai nguong duoi.
_Static_assert(TERR_PENDING_TIMEOUT_MS < ALT_EST_NO_CORRECTION_DEGRADED_MS,
    "TERR_PENDING_TIMEOUT_MS >= ALT_EST_NO_CORRECTION_DEGRADED_MS: nghi ngo terrain "
    "keo dai qua nguong 'khong co correction' -> degraded=true -> commander SOFT FAULT "
    "-> LANDING giua chuyen, TRUOC khi loi thoat kip chay.");
_Static_assert(TERR_PENDING_TIMEOUT_MS < ALT_EST_TOF_LOST_MS,
    "TERR_PENDING_TIMEOUT_MS >= ALT_EST_TOF_LOST_MS: nghi ngo terrain keo dai qua "
    "nguong mat ToF.");

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
    e->tilt_cos = 1.0f;
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

// LUON TRA TRUE. ToF do tuyet doi nen "da chot duoc mat san chua" khong con
// la cau hoi co nghia. Giu ham vi 4 cho trong flight_core.c va telemetry goi.
bool alt_estimator_floor_ready(const alt_estimator_t *e) {
    return e != NULL;
}

// lock_floor_at() — RESET trang thai bay truoc khi cat canh.
// Ten con giu chu "floor" vi 3 ham public goi no van mang ten do (va duoc goi
// tu flight_core.c), nhung no KHONG con chot goc toa do nao: ToF do tuyet doi.
static void lock_floor_at(alt_estimator_t *e) {
    e->floor_locked = true;
    // tof_ground_range_m khong con duoc tru vao dau ca (ToF do tuyet doi).
    // Van xoa ve 0 de telemetry khong hien mot so cu gay hieu nham.
    e->tof_ground_range_m = 0.0f;
    e->floor_plane_z_m = e->alt_m = e->vz_ms = 0.0f;
    e->active_source = ALT_SRC_GROUND_LOCK;
    e->terrain_off_m = e->terr_cand_offset_m = e->terr_residual_m = 0.0f;
    e->terr_anchor_raw_m = 0.0f;
    e->terr_offset_stale = false;
    e->terr_pending = false;
    e->terr_pending_since_us = 0;
    e->terr_confirm_cnt = 0;
    e->agl_m = 0.0f;
    e->prev_tof_vertical_valid = false;
    // Xoa moc slew/derivative: gia tri cua lan bay truoc se lam mau dau tien
    // cua lan nay bi gioi han sai huong.
    e->prev_tof_z_valid = false;
}

// Ba ham duoi day lam CUNG mot viec (reset trang thai bay truoc khi cat canh).
// Giu ba cua vao vi chung duoc goi tu 3 cho khac nhau voi y nghia khac nhau.
bool alt_estimator_lock_floor(alt_estimator_t *e) {
    if (!e) return false;
    lock_floor_at(e);
    return true;
}

bool alt_estimator_lock_floor_fallback(alt_estimator_t *e) {
    if (!e) return false;
    lock_floor_at(e);
    return true;
}

void alt_estimator_lock_floor_at_zero(alt_estimator_t *e) {
    if (!e) return;
    lock_floor_at(e);
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
    e->terr_anchor_raw_m = 0.0f;
    e->terr_offset_stale = false;
    e->terr_pending = false;
    e->terr_pending_since_us = 0;
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

// BA DAI LUONG, DUNG TRON:
//   tof_vertical_m : do THO toi be mat duoi (da bu tilt). Khong qua terrain/fusion.
//   terrain_off_m  : cao do be mat do so voi SAN cat canh.
//   alt_m          : Z TUYET DOI = khoang_ho + terrain_off.
//
// agl_m() uu tien SO DO THO. Cong thuc cu (alt_m - terrain_off) SAI khi offset
// hong: alt_m DA chua terrain_off nen sai so khong tu triet tieu. Do duoc:
// TOFF_cu 0.693, khoang ho that 0.019m -> cong thuc cu ra 0.712m -> landing
// tuong con 70cm de ha trong khi drone sap cham.
float alt_estimator_agl_m(const alt_estimator_t *e) {
    if (!e) return 0.0f;
    // So do THO con dung duoc ve hinh hoc -> dung no, bat ke terrain_off.
    if (e->tof_vertical_m >= ALT_EST_TOF_MIN_RANGE_M &&
        e->tof_vertical_m <= ALT_EST_TOF_MAX_RANGE_M) {
        return fmaxf(0.0f, e->tof_vertical_m);
    }
    return fmaxf(0.0f, e->alt_m - e->terrain_off_m);
}

float alt_estimator_tof_agl_m(const alt_estimator_t *e) {
    if (!e) return 0.0f;
    if (e->tof_vertical_m >= ALT_EST_TOF_MIN_RANGE_M &&
        e->tof_vertical_m <= ALT_EST_TOF_MAX_RANGE_M) {
        return fmaxf(0.0f, e->tof_vertical_m);
    }
    return fmaxf(0.0f, e->tof_z_m - e->terrain_off_m);
}

bool alt_estimator_terrain_rebase(alt_estimator_t *e) {
#if FC_FEATURE_TERRAIN_OFFSET
    // Rebase = do lai truc tiep tu mot mau ToF hop le hinh hoc. KHONG doi tof_fusable
    // (dung luc guard ban thi fusable thuong dang bi tat).
    if (!e || !e->floor_locked) return false;
    // Chi rebase tu mot mau ToF con dung duoc ve HINH HOC (khong doi
    // tof_fusable: dung luc guard ban thi fusable thuong DANG bi tat).
    if (!(e->tof_vertical_m >= ALT_EST_TOF_MIN_RANGE_M &&
          e->tof_vertical_m <= ALT_EST_TOF_MAX_RANGE_M)) return false;
    const float raw_agl = e->tof_vertical_m;
    e->terrain_off_m = e->alt_m - raw_agl;
    e->terr_pending = false;
    e->terr_pending_since_us = 0;
    e->terr_confirm_cnt = 0;
    e->terr_cand_offset_m = e->terrain_off_m;
    e->terr_anchor_raw_m = raw_agl;
    e->terr_commit_count++;
    e->terr_offset_stale = false;   // rebase = do lai truc tiep, tin duoc
    e->agl_m = raw_agl;
    return true;
#else
    (void)e;
    return false;
#endif
}

// tof_hw_alive = "chip VAN DANG DO", KHAC "co mau dung duoc".
//
// TUOI MAU KHONG CON LA DIEU KIEN. Truoc day bac thang 4 muc theo tuoi mau lam
// moi su kien binh thuong (bay qua vat the, nam sat san, be mat hap thu) ket
// thuc bang TU DONG HA CANH. Gio chi con MOT cau hoi: chip co con do khong.
//
// Danh doi co y: coast bang accel khong con gioi han thoi gian. Bu lai bang
// ALT_EST_COAST_SNAP_MS (ep alt_m ve so do that sau 400ms).
static void update_age(alt_estimator_t *e, bool airborne, bool tof_hw_alive,
                       int64_t now_us) {
    e->no_correction_ms = e->last_tof_accept_us
        ? (int)((now_us - e->last_tof_accept_us) / 1000) : 0;

    if (!airborne) {
        e->tof_track_state = e->tof_fusable ? ALT_TOF_TRACKING : ALT_TOF_LOST;
        return;
    }

    if (!tof_hw_alive) {
        // Chip im — bus dut hoac cam bien chet. Duong soft-fault DUY NHAT.
        e->tof_track_state = ALT_TOF_LOST;
        e->tof_fusable = e->valid = false;
        e->degraded = true;
        e->active_source = ALT_SRC_TOF_LOST;
        return;
    }

    // Chip con do. Co mau vua duoc fuse o tick nay khong?
    if (e->tof_fusable) {
        e->tof_track_state = ALT_TOF_TRACKING;
        e->valid = true;
        e->degraded = false;
        e->active_source = ALT_SRC_TOF_FUSED;
    } else {
        // Chip do nhung mau khong dung duoc (ngoai tam / hap thu / sat san /
        // dang nghi co bac terrain). Coast bang IMU. VAN valid — khong phai loi.
        e->tof_track_state = ALT_TOF_BRIDGE;
        e->valid = true;
        e->active_source = ALT_SRC_IMU_PREDICT_ONLY;

        // degraded = "QUA LAU khong co correction", KHONG phai "tick nay khong co".
        // Commander fault NGAY tick dau khi thay degraded (khong debounce), nen dat
        // degraded=true o mau dau khong fusable = mot mau ToF xau -> LANDING.
        //
        // ⚠ Doc last_tof_geom_ok_us, KHONG phai last_tof_accept_us: cau hoi la "cam
        // bien con cho so do khong", khong phai "ta co dang dung so do do khong".
        // Hai cai khac nhau dung luc terr_pending co Y ngung fuse. Ban cu doc moc
        // accept nen moi lan roi ban deu dem nguoc toi FAULT sau 300ms du ToF khoe.
        const int64_t since_geom_ms = e->last_tof_geom_ok_us
            ? (now_us - e->last_tof_geom_ok_us) / 1000
            : (int64_t)ALT_EST_NO_CORRECTION_DEGRADED_MS;
        e->degraded = since_geom_ms >= (int64_t)ALT_EST_NO_CORRECTION_DEGRADED_MS;
    }
}

void alt_estimator_update(alt_estimator_t *e, vec3f_t a, quat_t q,
#if FC_FEATURE_BARO
                          bool baro_healthy, uint32_t baro_seq,
                          int64_t baro_timestamp_us, float baro_alt_m,
#endif
                          bool tof_healthy, uint32_t tof_seq,
                          int64_t tof_timestamp_us, float tof_range_m,
                          bool tof_hw_alive,
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
    // Ghi MOI TICK (khong nam trong if(tof_new)): bu throttle theo nghieng
    // can gia tri tuoi o nhip dieu khien, khong phai nhip ToF.
    e->tilt_cos = rzz;
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
            {
                // ToF do TUYET DOI: alt_m = khoang cach toi BE MAT dang nhin, KHONG phai
                // "cao hon diem cat canh". May do san (Welford) da bo han vi nam sat san
                // VL53L1X doc 0.000m -> cua so khong bao gio dong -> chan ca viec cat canh.
                float raw_agl = e->tof_vertical_m;

                // Dat true tai DUNG mau commit offset moi. Xem khoi
                // "BAC DIA HINH LAM MOC SLEW/DAO HAM MAT NGHIA" ben duoi.
                bool terr_step_now = false;
                (void)terr_step_now;

#if FC_FEATURE_TERRAIN_OFFSET
                // ---- PHAT HIEN BAC TERRAIN BANG RESIDUAL (B4) ----
                // residual = phan thay doi range KHONG giai thich duoc bang
                // chuyen dong cua drone. Chi co nghia khi DANG BAY va hai mau
                // ke nhau du gan (gap lon thi vz*dt khong con la du doan tot).
                if (airborne && e->prev_tof_vertical_valid &&
                    e->tof_dt_s > 0.005f &&
                    e->tof_dt_s <= (float)ALT_EST_TOF_GAP_DERIV_MAX_MS / 1000.0f) {
                    const float d_range  = e->tof_vertical_m - e->prev_tof_vertical_m;
                    // KHONG tru chuyen dong cua drone. Can do tren log: so hang vz*dt sua duoc
                    // 0.018m nhung bom vao 0.082m sai so, tren tin hieu bac ban 0.75-1.20m.
                    // TRES tren SAN PHANG voi cong thuc cu: +0.078 trung binh / dinh +0.108.
                    const float residual = d_range;
                    e->terr_residual_m = residual;

                    // MOC NEO, khong cong don. Bac dia hinh la HIEU CUA HAI MUC.
                    // Ban cu `cand -= d_range` chi chay khi vuot nguong -> mau duoi nguong bi bo
                    // im lang -> bat doi xung -> TOFF ket o +0.165 thay vi 0 khi quay lai san.
                    if (!e->terr_pending) {
                        if (fabsf(residual) > TERR_JUMP_THRESH_M) {
                            e->terr_pending = true;
                            // MUC TRUOC BAC. prev_tof_vertical_m van la mau
                            // lien truoc (no chi duoc cap nhat cuoi khoi nay).
                            e->terr_anchor_raw_m = e->prev_tof_vertical_m;
                            // Dong ho loi thoat: chi dat o CANH LEN cua nghi
                            // ngo. Dat lai moi mau se lam timeout khong bao
                            // gio het han.
                            e->terr_pending_since_us = now_us;
                            e->terr_samples_seen = 0;
                            e->terr_confirm_cnt = 0;
                        }
                    }

                    if (e->terr_pending) {
                        // Ung vien duoc tinh LAI moi mau tu moc neo -- khong
                        // tich luy, nen khong co sai so nao dong lai.
                        // terrain_off_m KHONG doi trong suot pending (chi doi
                        // luc COMMIT) nen no la goc on dinh de cong vao.
                        e->terr_cand_offset_m =
                            e->terrain_off_m + (e->terr_anchor_raw_m - raw_agl);
                        // ⚠ KHONG dat terr_confirm_cnt = 0 o day: khoi nay chay
                        // MOI MAU, xoa o day thi bo dem khong bao gio vuot 1 va
                        // COMMIT khong bao gio xay ra. Viec reset thuoc ve khoi
                        // confirm ben duoi (chi xoa khi range CON nhay).
                    }

                // Tieu chi confirm chi dung SO DO THO: hoi "range da dung yen chua".
                // Ban cu so cand_z voi alt_m (tron so tho voi so da loc) -- ma alt_m dang
                // COAST trong pending nen no troi xa dan, cang pending lau cang kho confirm.
                if (e->terr_pending) {
                    // Dem MAU da di qua confirm -- timeout doc con so nay
                    // (xem TERR_MIN_SAMPLES_BEFORE_TIMEOUT).
                    if (e->terr_samples_seen < UINT8_MAX) e->terr_samples_seen++;
                    if (fabsf(residual) <= TERR_JUMP_THRESH_M) {
                        if (++e->terr_confirm_cnt >= TERR_CONFIRM_N) {
                            // ---- B3: SANITY CHECK ----
                            // Buoc terrain qua lon = do sai, khong phai bac
                            // that. Tu choi va giu offset cu; van thoat pending
                            // (neu khong thi lai ket dung vong luan quan cu).
                            if (fabsf(e->terr_cand_offset_m - e->terrain_off_m)
                                    <= TERR_MAX_STEP_M) {
                                e->terrain_off_m = e->terr_cand_offset_m;   // COMMIT
                                e->terr_commit_count++;
                                terr_step_now = true;   // moc slew/dao ham phai bo
                                // Offset MOI da duoc xac nhan -> tin lai duoc.
                                e->terr_offset_stale = false;
                            } else if (e->terr_reject_count < UINT16_MAX) {
                                e->terr_reject_count++;
                            }
                            e->terr_pending = false;
                            e->terr_pending_since_us = 0;
                            e->terr_confirm_cnt = 0;
                            e->terr_samples_seen = 0;
                        }
                    } else {
                        // Van con nhay -> khoi cong don o tren da xu ly mau nay
                        // va da dat confirm_cnt = 0. Giu lai cho ro y dinh.
                        e->terr_confirm_cnt = 0;
                    }
                }
                }   // dong khoi `if (airborne && prev_valid && dt hop le)`
#endif  // FC_FEATURE_TERRAIN_OFFSET

                e->prev_tof_vertical_m = e->tof_vertical_m;
                e->prev_tof_vertical_valid = true;

                // Z tren SAN = AGL + terrain. terrain_off_m = 0 -> dung cong
                // thuc cu.
                float zt = raw_agl + e->terrain_off_m;
                if (zt < ALT_EST_GROUND_ZERO_BAND_M) zt = 0.0f;

#if FC_FEATURE_TERRAIN_OFFSET
                // Sanity: bac qua lon = do sai, khong phai bac that. Tu choi nhung VAN thoat
                // pending (khong thi ket lai dung vong luan quan cu).
                if (e->terr_pending || terr_step_now) {
                    e->prev_tof_z_valid = false;
                }
#endif

                // SLEW-RATE LIMIT thay cho innovation gate: gioi han zt doi bao nhieu MOI GIAY
                // thay vi LOAI BO mau. Gate cu vut mau -> mat nguon do cao -> auto-land.
                // Chay tren tof_dt_s THAT (nhip ToF khac nhau giua L0X 33ms va L1X 40ms).
                if (e->prev_tof_z_valid && e->tof_dt_s > 0.0f &&
                    e->tof_dt_s <= ALT_EST_TOF_GAP_DERIV_MAX_MS / 1000.0f) {
                    const float max_step = ALT_EST_TOF_MAX_SLEW_MS * e->tof_dt_s;
                    const float d = zt - e->prev_tof_z_m;
                    if (d >  max_step) zt = e->prev_tof_z_m + max_step;
                    if (d < -max_step) zt = e->prev_tof_z_m - max_step;
                }

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

                // INNOVATION GATE DA BO: bay qua vat 0.5m -> innovation vuot nguong -> mau bi
                // loai -> valid=false -> soft-fault. Tuc "bay qua vat the" == "tu ha canh".
                // He qua da chon: alt_hold BAM THEO be mat, terrain offset lo phan bu lai.
                e->tof_surface_gate_ok = true;
                e->tof_surface_state = ALT_EST_TOF_SURFACE_FLOOR;

                // Moc "chip con do duoc" -- ghi TRUOC moi nhanh terrain. Neu di theo
                // tof_fusable thi no dung yen dung luc nghi ngo dia hinh, tai tao lai loi cu.
                e->last_tof_geom_ok_us = now_us;

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
                {
                    // Reacquire chi con y nghia sau mot lan MAT HAN ToF (het
                    // tam, mat mau) — khong con lien quan toi "nhin thay be mat
                    // khac" nua vi gate do da bo.
                    if (e->tof_track_state == ALT_TOF_LOST && airborne) {
                        if (e->tof_reacquire_count < UINT8_MAX) e->tof_reacquire_count++;
                    } else e->tof_reacquire_count = ALT_EST_TOF_REACQUIRE_SAMPLES;
                    e->tof_fusable = e->tof_reacquire_count >= ALT_EST_TOF_REACQUIRE_SAMPLES;
#if FC_FEATURE_TERRAIN_OFFSET
                    // ⚠ OFFSET DA MAT TIN CAY -> KHONG duoc fuse Z tuyet doi.
                    // Xem terr_offset_stale trong alt_estimator.h. Fuse bang
                    // mot offset cu = bom buoc nhay gia 0.5m vao alt_m (da do
                    // duoc: TOFF_cu 0.693 + raw 1.01 = 1.70 vs alt_m 1.09).
                    // Raw range VAN dung duoc cho agl/clearance -- chi rieng Z
                    // TUYET DOI la khong con tin.
                    if (e->terr_offset_stale) {
                        e->tof_fusable = false;
                        e->tof_surface_state = ALT_EST_TOF_SURFACE_OTHER;
                    }
#endif
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

#if FC_FEATURE_TERRAIN_OFFSET
    // Bac dia hinh lam moc slew/dao ham mat nghia. Slew chay bat ke pending se
    // keo prev_tof_z_m tut dan roi phai bo len lai -> dao ham cua doc NHAN TAO do
    // = +-1.5 m/s di thang vao fusion -> drone bay BANG ma estimator bao leo 0.7.
    if (e->terr_pending && e->terr_pending_since_us &&
        e->terr_samples_seen >= TERR_MIN_SAMPLES_BEFORE_TIMEOUT &&
        (now_us - e->terr_pending_since_us) >
            (int64_t)TERR_PENDING_TIMEOUT_MS * 1000) {
        e->terr_pending = false;
        e->terr_pending_since_us = 0;
        e->terr_confirm_cnt = 0;
        e->terr_samples_seen = 0;
        e->terr_cand_offset_m = e->terrain_off_m;   // vut ung vien
        e->terr_anchor_raw_m = 0.0f;
        if (e->terr_timeout_count < UINT16_MAX) e->terr_timeout_count++;

        // LOI THOAT BAT BUOC cho terr_pending -- PHAI nam ngoai if(tof_new), vi trong
        // pending khong co mau nao di qua confirm nen no khong bao gio tu huy duoc.
        // Het han = HUY nghi ngo, KHONG commit mot ung vien chua duoc xac nhan.
        e->terr_offset_stale = true;
    }
#endif

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
        e->terr_pending_since_us = 0;
        e->terr_confirm_cnt = 0;
        e->agl_m = 0.0f;
        e->active_source = ALT_SRC_GROUND_LOCK;

        // Moc "lan cuoi co correction dung duoc" — refresh moi tick chung nao
        // mau ToF gan nhat con fusable. Phai chay TRUOC khi tinh valid ben duoi
        // vi valid gio doc chinh moc nay.
        if (e->tof_fusable) e->last_tof_accept_us = e->last_correction_us = now_us;

        // Nam sat san thi driver ngung cap mau hop le (TOFAGE bo len hang chuc giay
        // trong khi TOFALIVE van dao). Khong xoa thi tof_vz_lpf_ms dong bang, va
        // confirm_liftoff() tiem no vao vz -> do duoc -1.414 m/s ngay khi roi dat.
        // KHONG dung toi tof_fusable: no giu ground_tof_alive -> cong cat canh.
        e->tof_vz_valid = false;
        e->tof_vz_ms = e->tof_vz_lpf_ms = 0.0f;
        // Khong co dao ham nao bac qua ranh gioi dat/khong: mau cuoi cung trong
        // luc nam dat va mau dau tien khi da bay cach nhau ca giay.
        e->prev_tof_vertical_valid = false;
        e->prev_tof_z_valid = false;

        // Moc hinh hoc refresh khi nam dat: tren mat dat Z bi khoa = 0 nen khong co
        // tich phan nao chay, cau hoi degraded vo nghia. Khong refresh thi nam cho
        // lau roi cat canh se SOFT FAULT ngay tick dau.
        e->last_tof_geom_ok_us = now_us;

        // Luoi do tren mat dat dung CUNG nguong ALT_EST_TOF_LOST_MS voi tren khong.
        // Truoc day mot mau ToF xau la valid=false ngay -> huy ca lan cat canh, trong
        // khi range_status != 0 la chuyen BINH THUONG voi ToF.
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
        update_age(e, false, tof_hw_alive, now_us);
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

    // TRAN COAST: alt_m la trang thai co tri nho, moi lan ToF ngung sua no thi no
    // coast bang accel va GIU LAI sai so. Nang nhat la terr_offset_stale ket
    // vinh vien -> alt_m troi tu do het chuyen bay. Xem ALT_EST_COAST_SNAP_MS.
    if (e->last_tof_accept_us && e->last_tof_geom_ok_us) {
        const int64_t coast_ms = (now_us - e->last_tof_accept_us) / 1000;
        const int64_t geom_ms  = (now_us - e->last_tof_geom_ok_us) / 1000;
        if (coast_ms > (int64_t)ALT_EST_COAST_SNAP_MS &&
            geom_ms  < (int64_t)ALT_EST_TOF_LOST_MS) {
#if FC_FEATURE_TERRAIN_OFFSET
            if (e->terr_pending || e->terr_offset_stale) {
                // KHONG ep alt_m = tof_z_m khi dang nghi ngo: tof_z_m = raw + terrain_off, ma
                // terrain_off CHINH LA thu dang bi nghi ngo. Rebase thay vi snap -- sau 400ms
                // coast sai so tich phan chi ~8mm, con offset cu co the sai ca nua met.
                e->terrain_off_m = e->alt_m - e->tof_vertical_m;
                e->terr_cand_offset_m = e->terrain_off_m;
                e->terr_anchor_raw_m = e->tof_vertical_m;
                e->terr_pending = false;
                e->terr_pending_since_us = 0;
                e->terr_confirm_cnt = 0;
                e->terr_samples_seen = 0;
                e->terr_offset_stale = false;
                e->tof_z_m = e->tof_surface_z_m = e->alt_m;
                e->prev_tof_z_valid = false;   // moc slew/dao ham cu het nghia
            } else
#endif
            {
                // terrain_off_m dang dung -> tof_z_m la so do THAT. Ep alt_m ve
                // no, dung nhu cau truc mong muon: khong de tich phan troi.
                e->alt_m = e->tof_z_m;
            }
            e->vz_ms = e->tof_vz_valid ? e->tof_vz_lpf_ms : 0.0f;
            e->last_tof_accept_us = e->last_correction_us = now_us;
            if (e->coast_snap_count < UINT16_MAX) e->coast_snap_count++;
        }
    }

    if (!isfinite(e->alt_m) || !isfinite(e->vz_ms)) {
        clear_runtime(e, true);
        e->active_source = ALT_SRC_TOF_LOST;
        return;
    }
    // AGL loc — dan xuat DUY NHAT tu (alt_m, terrain_off_m). Moi noi can "cao
    // bao nhieu so voi be mat" phai doc field nay, khong tu tru lai.
    e->agl_m = e->alt_m - e->terrain_off_m;
    update_age(e, true, tof_hw_alive, now_us);
}

void alt_estimator_correct_velocity(alt_estimator_t *e, float v, bool ok, float dt) {
    if (!e || !ok || !(dt > 0.0f) || !isfinite(v)) return;
    e->vz_ms += clampf(0.10f*(v-e->vz_ms), -0.15f, 0.15f);
}
