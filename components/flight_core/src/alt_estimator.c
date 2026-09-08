#include "flight_core/alt_estimator.h"

#include <math.h>
#include <string.h>

#include "flight_core/fc_features.h"   // FC_FEATURE_FLOOR_GATE

// Nghi ngo terrain lam tof_fusable=false, tuc KHONG co correction trong suot
// thoi gian do. Hai nguong ben duoi la hai cach khac nhau de mot khoang
// "khong correction" bien thanh SOFT FAULT -> LANDING:
//
//   ALT_EST_NO_CORRECTION_DEGRADED_MS : degraded=true -> commander fault
//   ALT_EST_TOF_LOST_MS               : (chi khi chip IM han, khong ap o day)
//
// terr_pending PHAI tu huy TRUOC ca hai, neu khong thi bay qua vat the =
// tu dong ha canh. Do dung la trieu chung da lam TERRAIN_OFFSET_ENABLED bi
// tat, va cung la lo hong cua chinh ban va nay o phien ban dau (assert cu chi
// so voi ALT_EST_TOF_LOST_MS trong khi nguong THAT SU chan truoc la
// ALT_EST_NO_CORRECTION_DEGRADED_MS).
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

// alt_estimator_floor_ready() — GIU LAI CHU KY, LUON TRA TRUE.
//
// ToF gio do TUYET DOI (khong con goc toa do, xem update()), nen "da chot
// duoc mat san chua" khong con la cau hoi co nghia. Tra true de moi cho goi
// no — prearm, takeoff, telemetry — khong con bi chan.
//
// KHONG xoa han ham: no duoc goi tu 4 cho trong flight_core.c va telemetry.
// Xoa se lam vo build o nhung file khong lien quan gi toi thay doi nay. Giu
// mot ham tra hang so la cach re nhat de giu bien gioi module.
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

// ---- Ba ham "chot san" ben duoi: GIU CHU KY, BO PHAN THONG KE ----
// ToF gio do TUYET DOI nen khong con goc toa do de chot. Thu duy nhat con
// y nghia la RESET trang thai bay (alt/vz/terrain/prev) truoc khi cat canh —
// chinh la phan lock_floor_at() van lam. Ca ba deu thanh cong va lam DUNG
// mot viec do.
//
// KHONG hop nhat thanh mot ham: chung duoc goi tu 3 cho khac nhau trong
// flight_core.c voi y nghia khac nhau ("duong chuan", "fallback", "khong co
// mau"), va gop lai se buoc phai sua ca ba cho do trong cung mot luot — nhieu
// rui ro hon la giu ba cua vao mong.
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

// ============================================================================
// KHOANG HO (clearance) vs DO CAO TUYET DOI -- HAI THU KHAC NHAU
// ============================================================================
//   tof_vertical_m : khoang cach THO toi be mat ngay duoi (da bu cos tilt).
//                    KHONG qua terrain_off, KHONG qua fusion. Do TRUC TIEP.
//   terrain_off_m  : cao do cua be mat do so voi SAN cat canh.
//   alt_m          : do cao TUYET DOI so voi san = agl + terrain_off.
//
// ⚠ agl_m TRUOC DAY tinh bang (alt_m - terrain_off_m). Dung khi offset dung,
// nhung SAI HAN khi offset da stale: ca hai ve deu mang cung mot sai so nen no
// KHONG tu trieu tieu -- clearance thua huong nguyen cai sai cua Z tuyet doi.
//
// Do duoc tren log: TOFF_cu = 0.693 con drone thuc te cach mat ban 0.019m.
// Cong thuc cu cho ra clearance = 0.712m -> landing tuong con 70cm de ha, trong
// khi drone SAP CHAM. Touchdown khong bao gio kich hoat, motor quay mai.
//
// GIO: uu tien SO DO THO. No khong phu thuoc terrain_off nen dung ke ca khi
// offset sai hoan toan -- dung thu ma landing/guard can.
//
// Chi rot ve (alt_m - terrain_off_m) khi KHONG co so do tho dung duoc (ToF
// ngoai tam / hap thu). Luc do day la uoc luong tot nhat con lai.
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
    // ⚠ TRUOC DAY con doi them `e->tof_ground_ref_valid`, va do la mot LOI
    // CHET LANG: co do duoc set boi may do san (floor_add + Welford), ma may
    // do san DA BI XOA HAN. Grep toan repo chi con MOT cho cham vao no:
    //     clear_runtime(): e->tof_ground_ref_valid = old.tof_ground_ref_valid;
    // tuc no tu copy chinh minh, khong ai set true bao gio -> dieu kien luon
    // false -> ham nay LUON return false.
    //
    // Hau qua: guard khoang ho (flight_core.c B8) van ep leo duoc, nhung
    // NHANH REBASE cua no chet cung. Log in ra "(khong rebase duoc: ToF khong
    // dung duoc)" moi lan, dung nhu the ToF hong -- trong khi ToF hoan toan
    // binh thuong. Nguoi doc log se di tim loi o cam bien.
    //
    // Tuong tu, `tof_ground_range_m` gio CHI duoc gan 0.0f (lock_floor_at),
    // nen phep tru no la vo nghia -- ToF do TUYET DOI, raw_agl chinh la
    // tof_vertical_m (dung cong thuc voi khoi fusion o alt_estimator_update).
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

// tof_hw_alive = "chip VẪN ĐANG ĐO", KHÁC HẲN "có mẫu dùng được".
// Nó đến từ sensor_hub snapshot (tof_alive_us) và chỉ đứng yên khi chip chết
// hoặc bus đứt — nằm sát sàn (0mm, dưới tầm mù), nhìn ra khoảng không, hay bề
// mặt hấp thụ đều KHÔNG làm nó đứng.
// update_age() — quyet dinh do cao con DUNG DUOC khong.
//
// ============================================================================
// TUOI MAU KHONG CON LA DIEU KIEN (yeu cau nguoi dung)
// ============================================================================
// TRUOC DAY day la mot bac thang 4 muc theo tuoi mau hop le:
//     <=TRACK_MAX_AGE -> TRACKING
//     <=BRIDGE_MS     -> SHORT_BRIDGE
//     < LOST_MS       -> IMU_PREDICT (degraded)
//     >=LOST_MS       -> LOST  -> valid=false -> Commander soft-fault
//
// Bac cuoi la nguon goc cua ca mot chuoi loi da phai vá tung cai mot: bay qua
// vat the, nam sat san, nhin ra khoang khong, be mat hap thu — tat ca deu lam
// tuoi mau tang vo han TRONG KHI cam bien hoan toan lanh, va tat ca deu ket
// thuc bang tu dong ha canh giua chung.
//
// GIO chi con MOT cau hoi: CHIP CO CON DO KHONG (tof_hw_alive).
//   con do  -> valid = true. Co mau moi thi fuse, khong co thi coast bang IMU.
//              Do la trang thai degraded, KHONG phai loi.
//   chip im -> valid = false. Day la loi THAT, va la duong soft-fault DUY NHAT
//              con lai.
//
// ⚠ DANH DOI CO Y: coast bang tich phan accel khong con gioi han thoi gian.
// Chip song ma khong ra mau hop le trong 10s thi do cao van "valid" du no da
// troi dang ke. Bu lai: khong con tu ha canh vi mot ly do binh thuong. Nguoi
// bay nhin ALTSRC=IMU tren GUI de biet dang coast.
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

        // ====================================================================
        // degraded = "QUA LAU khong co correction", KHONG phai "tick nay khong
        // co correction". Do la HOP DONG viet trong commander.h muc
        // alt_estimator_degraded, va truoc ban va nay code KHONG giu dung no.
        // ====================================================================
        // Commander fault NGAY tick dau khi thay degraded (commander.c, nhanh
        // "ToF correction mat qua lau"), KHONG co debounce nao o do. Nen dat
        // degraded=true ngay o mau dau tien khong fusable co nghia la:
        //
        //   MOT mau ToF khong dung duoc  ->  SOFT FAULT  ->  LANDING
        //
        // Ma "mau khong dung duoc" la chuyen BINH THUONG voi ToF: ngoai tam,
        // be mat hap thu, nang manh, va — tu khi bat FC_FEATURE_TERRAIN_OFFSET
        // — moi lan terr_pending len 1 vi nghi co bac dia hinh.
        //
        // ⚠ HAU QUA CU THE DA SUYT XAY RA: bay qua ban -> terr_pending=1 ->
        // tof_fusable=false -> degraded=true -> LANDING GIUA CHUYEN. Dung
        // trieu chung da lam TERRAIN_OFFSET_ENABLED bi tat lan truoc, chi khac
        // nguyen nhan. TERR_PENDING_TIMEOUT_MS (200ms) KHONG cuu duoc vi fault
        // no o tick dau tien, rat lau truoc 200ms.
        //
        // GIO: dem tu moc correction cuoi cung. Coast vai chuc ms bang IMU la
        // an toan (sai so bac hai theo thoi gian, o 200ms van rat nho); chi khi
        // vuot ALT_EST_NO_CORRECTION_DEGRADED_MS moi that su la "Z dang troi
        // tu do" va luc do bao degraded moi dung nghia.
        //
        // ⚠ DOC last_tof_geom_ok_us, KHONG PHAI last_tof_accept_us.
        //
        // Cau hoi ma degraded phai tra loi la "CAM BIEN con cho ta so do
        // khong", chu khong phai "ta co dang dung so do do de sua Z khong".
        // Hai cai nay khac nhau dung o luc phat hien dia hinh: ta CO Y ngung
        // fuse (terr_pending / terr_offset_stale ep tof_fusable=false) de alt_m
        // coast qua bac -- do la thiet ke.
        //
        // Ban cu doc last_tof_accept_us nen moi lan roi ban deu dem nguoc:
        //     TPEND=1 -> tof_fusable=false -> moc dung -> 300ms -> SOFT FAULT
        // trong khi ToF van tra mau deu 25Hz. Ngan sach chi 300ms ke tu luc
        // nghi ngo, ma detect+confirm da an ~160-190ms -- truot mot mau la het.
        //
        // == 0 = chip CHUA TUNG tra mau hop le nao trong ca chuyen bay -> that
        // su khong co gi de tin -> degraded ngay. Do moi la ToF chet.
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
                // ============================================================
                // ToF DO TUYET DOI — KHONG con goc toa do, KHONG con do san
                // ============================================================
                // TRUOC DAY: raw_agl = tof_vertical_m - tof_ground_range_m,
                // trong do tof_ground_range_m den tu mot cua so thong ke thu
                // luc drone nam yen (floor_add + Welford + nguong std).
                //
                // May do san do da bi BO HAN theo yeu cau nguoi dung. Ly do
                // thuc te: nam sat san thi VL53L1X doc 0.000m (duoi tam mu
                // ~4cm) -> driver loai mau -> cua so khong bao gio dong duoc
                // -> tof_ground_ref_valid dung o false -> ca khoi nay khong
                // chay -> ToF khong correction gi ca, VA takeoff bi tu choi.
                // Mot co che sinh ra de lam do cao chinh xac hon lai la thu
                // chan khong cho bay.
                //
                // GIO: do cao = khoang cach toi BE MAT dang nhin, thang tu
                // cam bien. Khong tru gi ca.
                //
                // ⚠ DOI NGHIA DO CAO — biet truoc de khong ngac nhien:
                // alt_m gio la "cach be mat ben duoi bao nhieu", KHONG phai
                // "cao hon diem cat canh bao nhieu". Cat canh tu tren ban roi
                // bay ra ngoai ban thi do cao NHAY mot bac bang chieu cao ban,
                // va PID se phan ung voi buoc nhay do. Slew-rate limit ben duoi
                // lam cho buoc nhay do di TU TU thay vi tuc thi.
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
                    // ========================================================
                    // ⚠ KHONG TRU CHUYEN DONG CUA DRONE NUA — DA CAN DO LAI
                    // ========================================================
                    // BAN CU: residual = d_range - vz_accel_only_ms * tof_dt_s,
                    // y la "bo phan range doi do CHINH DRONE leo/ha".
                    //
                    // Can lai bang so do duoc tren log:
                    //     thu no SUA duoc (drone tu leo, 1 mau) : 0.018 m
                    //     thu no BOM VAO  (sai so cua VZAO)     : 0.082 m
                    //     bac ban that can phat hien            : 0.75..1.20 m
                    //
                    // So hang do chi chiem 2.4% tin hieu, ma sai so cua no gap
                    // 5 LAN chinh thu no sua. O 25Hz thi 40ms drone khong di
                    // duoc bao xa: bac ban lon hon chuyen dong cua no ~55 lan.
                    //
                    // Do TRES tren SAN PHANG voi cong thuc cu: trung binh
                    // +0.078m, dinh +0.108m / nguong 0.12 -> con 12mm la bao
                    // dong dia hinh GIA. Chinh so hang "hieu chinh" tao ra no.
                    //
                    // GIO: buoc nhay THO. Doi lai TERR_JUMP_THRESH_M phai noi
                    // 0.12 -> 0.20 (bien 2.6x so voi |d_range| lon nhat do
                    // duoc luc leo, 0.076m). Xem hang so do trong header.
                    const float residual = d_range;
                    e->terr_residual_m = residual;

                    // ========================================================
                    // ⚠ MOC NEO, KHONG CONG DON. Doi tu ban cong don vi mot
                    //   LOI DO DUOC TREN LOG BAY -- doc ky truoc khi doi lai.
                    // ========================================================
                    // BAN CU: `cand -= residual`, va CHI chay khi
                    // |residual| > TERR_JUMP_THRESH_M. Nghia la moi mau DUOI
                    // nguong bi BO IM LANG du be mat van dang doi.
                    //
                    // Bat doi xung: mep sac di len bat duoc gan het, mep thoai
                    // di xuong bat duoc it hon -> offset KHONG ve 0 khi quay
                    // lai san. Do duoc: bay qua dia hinh roi VE LAI SAN ma
                    // TOFF ket o +0.165 thay vi 0 (mo phong mep khong sac cho
                    // +0.150 -- trung khop).
                    //
                    // Va khi DI LEN: bac that +0.70 chi bat duoc +0.55 ->
                    // alt_m THAP hon that 0.15m -> PID tuong drone dang thap
                    // -> tang ga -> DRONE VOT LEN. Dung trieu chung nguoi dung
                    // bao cao.
                    //
                    // GIO: bac dia hinh la HIEU CUA HAI MUC.
                    //     offset_moi = offset_cu + (anchor_raw - raw_hien_tai)
                    // Mot phep tru. Khong phu thuoc bac trai ra may mau, cung
                    // khong phu thuoc mau nao vuot nguong. Len va xuong doi
                    // xung TUYET DOI -> TOFF luon ve dung 0 khi ve san.
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

                // ---- XAC NHAN N MAU TRUOC KHI COMMIT (B5) ----
                // ⚠ TIEU CHI DA DOI: hoi "RANGE DA DUNG YEN CHUA", khong con
                // hoi "cand_z co khop alt_m khong".
                //
                // Ve cu: cand_z = raw_agl + cand_offset   (dai luong THO)
                //        so voi alt_m                     (da FUSE + LPF + bias)
                // Tron mot so tho voi mot so da loc, roi doi chung khop nhau
                // trong 0.10m. Ma dung trong cua so pending ta CO Y ngung fuse
                // nen alt_m dang COAST -- no troi moi luc mot xa dung luc dang
                // can no lam moc. Cang pending lau cang kho confirm: mot vong
                // tu lam kho chinh minh.
                //
                // Ve moi chi dung SO DO THO. Bac dia hinh khong den trong mot
                // mau (do duoc: TOFV 0.890 -> 0.338 -> 0.126 -> 0.119 -> 0.110)
                // nen: ung vien duoc tinh lai tu MOC NEO moi mau, va khi range
                // dung yen N mau lien tiep thi mat duoi da on dinh o muc moi
                // -> COMMIT. Khong dai luong nao da qua bo loc tham gia vao
                // quyet dinh nay.
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
                // ====================================================================
                // ⚠ BAC DIA HINH LAM MOC SLEW/DAO HAM MAT NGHIA -- LOI DO DUOC
                // ====================================================================
                // Slew limit va dao ham deu chay tren prev_tof_z_m, va ca hai TRUOC
                // DAY van chay BAT KE terr_pending. Hau qua, mo phong tren so lieu
                // log that (len ban 0.78m):
                //
                //   4 mau pending: slew keo prev_tof_z_m tut 0.890 -> 0.620
                //                  (moi mau -1.5*0.045 = -0.0675m)
                //   commit:        zt DUNG phai la 0.108+0.764 = 0.872, nhung slew
                //                  chi cho bo len 1.5 m/s -> mat them 4 mau
                //   dao ham cua cai doc NHAN TAO do = +1.5 m/s, va no di THANG vao
                //   fusion:  dv = ALT_EST_TOF_VZ_GAIN * (tof_vz_lpf - vz)
                //
                //   KET QUA: drone bay BANG ma estimator ket luan dang leo +0.70 m/s
                //   (cong them innov -0.203m keo alt_m tut ngay tai mau commit).
                //   PID thay "dang leo" -> cat ga de ham -> drone chui xuong DUNG
                //   luc vua len tren mat ban. Roi ban thi nguoc dau.
                //
                // Ban chat: bac dia hinh la mot buoc NHAY THAT trong he quy chieu
                // datum. Bat no di qua slew limit la tu tao ra mot doan doc gia, roi
                // lay dao ham cua chinh doan doc do lam "van toc thang dung".
                //
                // GIO: trong luc nghi ngo VA tai dung mau commit, vut moc cu di.
                // prev_tof_z_valid = false lam ba viec cung luc:
                //   1. bo qua slew  -> zt = raw + terrain_off MOI, dung ngay lap tuc
                //   2. deriv_ok = false -> tof_vz_valid = false, KHONG bom van toc ma
                //   3. cuoi khoi, prev_tof_z_m duoc nap lai = zt (gia tri DUNG)
                // Mau ke tiep co mot cap (prev, cur) cung he quy chieu -> dao ham
                // that tro lai binh thuong.
                if (e->terr_pending || terr_step_now) {
                    e->prev_tof_z_valid = false;
                }
#endif

                // ---- BO LOC NHE: SLEW-RATE LIMIT (thay cho innovation gate) ----
                // Gioi han zt duoc phep doi bao nhieu MOI GIAY, thay vi LOAI BO
                // mau khi no nhay. Khac biet cot loi:
                //   gate cu  -> mau bi vut  -> mat nguon do cao -> auto-land
                //   slew moi -> mau duoc dung nhung DI TU TU -> khong bao gio
                //               mat nguon, chi cham hon vai tram ms
                //
                // Chay tren THOI GIAN THAT (tof_dt_s) chu khong theo so mau: nhip
                // ToF thay doi theo cau hinh chip (L0X 33ms vs L1X 40ms) va co
                // the truot mau. Tinh theo mau se cho toc do gioi han khac nhau
                // giua hai chip voi CUNG mot hang so — dung loai bug im lang.
                //
                // Bo qua o mau DAU TIEN (prev chua co) va khi dt vo ly: luc do
                // khong co moc nao de gioi han, ep vao se khoa zt o 0 mai mai.
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

                // ============================================================
                // INNOVATION GATE: DA BO (yeu cau nguoi dung)
                // ============================================================
                // TRUOC DAY:
                //     tof_surface_gate_ok = |innovation| <= 0.25m
                // Bay qua vat cao 0.5m -> range tut 0.5m trong MOT mau ->
                // innovation vuot nguong -> surface=OTHER -> tof_fusable=false
                // -> sau ALT_EST_TOF_LOST_MS thi valid=false -> Commander
                // soft-fault "altitude estimator lost mid-flight" -> LANDING.
                //
                // Tuc la "bay qua vat the" == "tu dong ha canh". Da quan sat
                // duoc tren bo, va chinh comment o alt_estimator.h muc TERRAIN
                // cung da mo ta dung kich ban nay.
                //
                // GIO: moi mau hop le ve HINH HOC deu duoc fuse. Bo loc chong
                // nhay dot ngot chuyen sang SLEW-RATE LIMIT ngay tren zt (xem
                // ALT_EST_TOF_MAX_STEP_M ben duoi) — no lam so do doi MUOT thay
                // vi LOAI BO mau, nen khong bao gio dan toi "mat nguon do cao".
                //
                // ⚠ HE QUA PHAI BIET: alt_hold gio BAM THEO be mat ben duoi.
                // Bay qua ban cao 0.5m thi drone tu nang len ~0.5m roi ha lai
                // khi qua khoi. Do la danh doi da chon: tha bam theo dia hinh
                // con hon tu ha canh giua chung.
                e->tof_surface_gate_ok = true;
                e->tof_surface_state = ALT_EST_TOF_SURFACE_FLOOR;

                // ⚠ MOC "CHIP CON DO DUOC" -- ghi o DAY, TRUOC moi nhanh
                // terrain ben duoi. Toi day ta da biet: chip tra mau moi
                // (tof_new), range trong tam, tilt trong nguong. Do la tat ca
                // y nghia cua cau hoi "cam bien con song khong".
                //
                // KHONG duoc dat sau khoi terrain: terr_pending va
                // terr_offset_stale deu ep tof_fusable=false, va neu moc nay
                // di theo tof_fusable thi no lai dung yen dung luc dang nghi
                // ngo dia hinh -- tai tao lai chinh cai loi dang sua.
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
    // ========================================================================
    // LOI THOAT BAT BUOC CHO terr_pending  (fix vong luan quan)
    // ========================================================================
    // ⚠ KHOI NAY PHAI NAM NGOAI if(tof_new). Do la TOAN BO diem cua no.
    //
    // Trong luc nghi ngo, tof_fusable bi ep false (co y: khong an range dang
    // loan). Nhung confirm-counter chi chay khi CO mau di qua nhanh fusable ->
    // khong bao gio confirm, cung khong bao gio huy -> terr_pending ket o 1
    // vinh vien -> sau 300ms khong correction thi valid=false -> ALTSRC=4 ->
    // soft-fault -> LANDING giua chuyen bay. Da do duoc tren bo:
    //     TOFF=-0.315  TPEND=1  TCMT=1  TOFFUSE=0  TOFTRACK=0
    //
    // Dat trong if(tof_new) thi loi thoat cung chi chay khi co mau moi -- tuc
    // la khong sua duoc gi trong dung cai kich ban no sinh ra de sua.
    //
    // HET HAN = HUY NGHI NGO, KHONG PHAI COMMIT. Commit mot ung vien chua duoc
    // xac nhan la ghi mot offset co the sai vao trang thai BEN VUNG -- dung cai
    // da tao ra TOFF=-0.315 roi loai het moi mau sau do. Huy thi te nhat la
    // PID thay mot buoc nhay do cao that va phan ung voi no; do la thu co the
    // phuc hoi duoc.
    // ⚠ TIMEOUT DOI HAI DIEU KIEN, khong chi het gio.
    // Khoi confirm chay o nhip ToF (25Hz) con khoi nay chay o nhip dieu khien
    // (250Hz) -- lech mot bac. Doi them "da co du co hoi" de mot ung vien HOP
    // LE khong bi vut chi vi ToF truot vai mau. Xem terr_samples_seen.
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

        // ====================================================================
        // ⚠ TIMEOUT PHAI FAIL-SAFE: KHONG duoc quay lai fuse bang offset CU
        // ====================================================================
        // BAN TRUOC chi bo rieng lan nghi ngo roi cho ToF fuse lai NGAY bang
        // terrain_off_m cu. Do la mot LOI CHET NGUOI, do duoc tren log:
        //
        //     TOFF_cu = 0.693   raw = 1.01   ->  tof_z = 1.70
        //     alt_m dang coast  = 1.09       ->  innovation +0.61m
        //     -> estimator bi keo NHAY 0.5m -> FAULT=1 -> LANDING
        //
        // Ly do sau xa: het han nghia la "ta BIET be mat da doi nhung KHONG
        // BIET doi bao nhieu". Mot offset cu trong tinh huong do khong con mo
        // ta dia hinh ben duoi -- no la RAC, khong phai "gia tri an toan".
        //
        // GIO: danh dau offset la STALE. alt_estimator_update() se khong fuse
        // ToF nua (xem cho dat tof_fusable), estimator song bang IMU bridge va
        // bao degraded -> commander ha canh CO KIEM SOAT thay vi bi mot cu
        // nhay gia lam mat kiem soat.
        //
        // ⚠ Raw range VAN dung duoc: agl_m / clearance van tinh tu tof_vertical
        // nen guard va landing van co so do that de lam viec. Chi rieng Z
        // TUYET DOI la khong con tin duoc.
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

        // ====================================================================
        // ⚠ XOA VAN TOC ToF KHI NAM DAT — LOI DA DO DUOC TREN LOG
        // ====================================================================
        // Nam sat san thi driver NGUNG CAP MAU HOP LE (do duoc: TOFAGE bo tu
        // 8415 len 9158ms trong khi TOFALIVE van dao 3..43, tuc chip VAN tra
        // loi I2C — chi la khong con range dung duoc). Khong co mau moi thi
        // tof_vz_* va prev_* GIU NGUYEN gia tri cu: dong bang, khong ai xoa.
        //
        // (Khong phai do cong hinh hoc chan: ALT_EST_TOF_MIN_RANGE_M = 0.00
        //  nen 6mm van qua. Nguyen nhan nam o tang driver/hub, ngoai file nay.)
        //
        // Do duoc: dung yen tren san 8.4 GIAY ma
        //     tof_vz_lpf_ms = -1.414 m/s  va  tof_vz_valid = 1
        // van con nguyen tu luc dat drone xuong.
        //
        // Roi alt_estimator_confirm_liftoff() lam:
        //     vz_ms = tof_vz_valid ? tof_vz_lpf_ms : 0   ->  -1.414
        //     vz_accel_only_ms = vz_ms                   ->  -1.414
        // tuc la vua roi dat da bi tiem mot van toc RUNG XUONG 1.4 m/s.
        //
        // HAU QUA DAY CHUYEN — day moi la cho dau:
        //   vz_accel_only_ms KHONG BAO GIO duoc ToF sua (co y, xem khoi B4),
        //   nen sai so do o LAI suot chuyen bay. Do tren log: VZAO ~ -1.2 m/s
        //   trong khi VZ that ~ +0.4 -> lech TRUNG BINH +1.28 m/s.
        //   Ma terrain dung CHINH no lam du doan doc lap:
        //       expected = vz_accel_only_ms * tof_dt_s
        //   dt=45ms -> expected lech 0.057m = 48% cua TERR_JUMP_THRESH_M.
        //   Do lai tren log: TRES tren SAN PHANG (TOFF=0, khong he co bac) da
        //   la +0.078m trung binh, dinh +0.108m — chi con 0.012m nua la BAO
        //   DONG DIA HINH GIA.
        //
        // GIO: nam dat thi moi dai luong DAN XUAT tu ToF deu bi xoa. Khong the
        // co "van toc thang dung" khi drone dang nam yen tren san.
        //
        // ⚠ KHONG dung toi tof_fusable: no la thu giu ground_tof_alive ->
        // valid -> cong cat canh. Xoa no o day = tu choi cat canh vinh vien
        // (dung cai bay ma khoi comment "LUOI DO TREN MAT DAT" ben duoi mo ta).
        e->tof_vz_valid = false;
        e->tof_vz_ms = e->tof_vz_lpf_ms = 0.0f;
        // Khong co dao ham nao bac qua ranh gioi dat/khong: mau cuoi cung trong
        // luc nam dat va mau dau tien khi da bay cach nhau ca giay.
        e->prev_tof_vertical_valid = false;
        e->prev_tof_z_valid = false;

        // ⚠ MOC HINH HOC: refresh khi NAM DAT, va chi o day.
        // update_age() dung moc nay de bao degraded sau
        // ALT_EST_NO_CORRECTION_DEGRADED_MS. Nhung tren mat dat Z bi KHOA = 0
        // (xem ngay dau nhanh nay) — khong co tich phan nao dang chay thi khong
        // co gi de "troi tu do", nen cau hoi degraded khong co nghia o day.
        //
        // Neu khong refresh: nam cho 8.4s -> moc cu 8400ms -> tick DAU TIEN
        // sau khi AIRB=1 da thay "qua han 300ms" -> SOFT FAULT ngay khi vua
        // roi dat, du ToF hoan toan binh thuong. Refresh o day cho moi chuyen
        // bay bat dau voi tron ven ngan sach 300ms.
        e->last_tof_geom_ok_us = now_us;

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

    // ========================================================================
    // ⚠ TRAN COAST -- alt_m KHONG duoc phep tich phan tu do vo han
    // ========================================================================
    // Xem ALT_EST_COAST_SNAP_MS trong header de biet vi sao. Tom tat: alt_m la
    // trang thai co tri nho, va moi lan ToF ngung sua no thi no coast bang accel
    // va GIU LAI sai so. Truong hop nang nhat la terr_offset_stale ket vinh vien
    // (chi xoa khi COMMIT hoac rebase) -> alt_m troi het chuyen bay.
    //
    // Chi lam gi khi CHIP CON DO DUOC (last_tof_geom_ok_us con tuoi). ToF chet
    // that thi khong co so do nao de ep ve, va duong degraded lo phan ha canh.
    if (e->last_tof_accept_us && e->last_tof_geom_ok_us) {
        const int64_t coast_ms = (now_us - e->last_tof_accept_us) / 1000;
        const int64_t geom_ms  = (now_us - e->last_tof_geom_ok_us) / 1000;
        if (coast_ms > (int64_t)ALT_EST_COAST_SNAP_MS &&
            geom_ms  < (int64_t)ALT_EST_TOF_LOST_MS) {
#if FC_FEATURE_TERRAIN_OFFSET
            if (e->terr_pending || e->terr_offset_stale) {
                // ⚠ KHONG duoc ep alt_m = tof_z_m o day.
                // tof_z_m = raw + terrain_off_m, ma dung luc nay terrain_off_m
                // CHINH LA thu dang bi nghi ngo. Ep ve no = bom vao alt_m dung
                // cu nhay gia ma terr_offset_stale sinh ra de chan (do duoc:
                // TOFF_cu 0.693 + raw 1.01 = 1.70 vs alt_m 1.09 -> +0.61m).
                //
                // Thu DUNG duoc o day la NEO LAI: dat terrain_off theo alt_m
                // hien tai. Sau vai tram ms coast, sai so tich phan chi co
                // mili-met (bias 0.1 m/s^2 -> 400ms cho 0.008m), trong khi
                // offset cu co the sai ca nua met. Tin cai chinh xac hon.
                //
                // Rebase lam tof_z_m == alt_m -> fusion chay lai NGAY va khong
                // co buoc nhay nao. Day cung la loi thoat duy nhat pha duoc
                // deadlock cua terr_offset_stale.
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
