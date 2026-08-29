// Estimator do cao toi gian: IMU prediction + VL53L0X correction.
// Barometer (neu build) chi con la input tuong thich/telemetry, tuyet doi
// khong tham gia Z, Vz, bias hay validity.
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "flight_core/fc_features.h"
#include "flight_core/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ALT_SRC_GROUND_LOCK = 0,
    ALT_SRC_IMU_PREDICT_ONLY,
    ALT_SRC_TOF_FUSED,
    ALT_SRC_TOF_SHORT_BRIDGE,
    ALT_SRC_TOF_LOST,
} alt_source_t;

static inline const char *alt_source_name(alt_source_t s) {
    switch (s) {
        case ALT_SRC_GROUND_LOCK: return "GROUND_LOCK";
        case ALT_SRC_IMU_PREDICT_ONLY: return "IMU_PREDICT";
        case ALT_SRC_TOF_FUSED: return "TOF_FUSED";
        case ALT_SRC_TOF_SHORT_BRIDGE: return "TOF_BRIDGE";
        case ALT_SRC_TOF_LOST: return "TOF_LOST";
        default: return "UNKNOWN";
    }
}

typedef enum {
    ALT_EST_TOF_SURFACE_UNKNOWN = 0,
    ALT_EST_TOF_SURFACE_FLOOR,
    ALT_EST_TOF_SURFACE_OTHER,
} alt_est_tof_surface_t;

typedef enum {
    ALT_TOF_LOST = 0,
    ALT_TOF_BRIDGE,
    ALT_TOF_TRACKING,
} alt_tof_track_state_t;

typedef struct {
    float alt_m, vz_ms;
    bool valid, degraded;
    int64_t last_correction_us;
    int32_t no_correction_ms;
    alt_source_t active_source;

    // Cac tang Az tach rieng de kiem tra dau/truc/rung tren log.
    float az_body_z_g;
    float az_earth_raw_ms2;
    float az_after_gravity_ms2;
    float az_after_bias_ms2;
    float az_raw_ms2;             // alias wire cu = after gravity
    float az_lpf_ms2;
    float az_corrected_ms2;       // sau bias + deadband + LPF
    // cos(tilt) TUOI moi tick dieu khien (250Hz) -- KHAC tof_tilt_cos von chi
    // duoc ghi khi CO MAU ToF MOI (~25Hz, va dung han neu ToF chet).
    // Dung cho bu throttle theo goc nghieng (TILT_COMP_*): bu ga phai theo kip
    // attitude, khong the tre 40ms hay dong bang o gia tri cuoi cung.
    float tilt_cos;
    bool az_lpf_init;
    float vz_accel_only_ms, z_inertial_m;
    float accel_bias_ms2, bias_residual_m;
    uint32_t bias_adapt_count;

    int64_t last_tof_accept_us;
    float tof_corr_z_m, tof_corr_vz_ms;

#if FC_FEATURE_BARO
    // Debug-only compatibility. Tat ca correction/count fusion luon bang 0.
    int64_t last_baro_accept_us;
    float baro_corr_z_m, baro_corr_vz_ms;
    float baro_lpf_alt_m, baro_innovation_m, baro_dt_s;
    uint32_t baro_accept_count, baro_reject_count, baro_reject_consecutive;
    bool baro_reacquire_active, baro_fusion_initialized;
#endif

    float tof_raw_m, tof_vertical_m, tof_z_m;
    float tof_vz_ms, tof_vz_lpf_ms;
    bool tof_vz_valid, tof_fusable, tof_surface_gate_ok;
    float tof_innovation_m, tof_surface_z_m, tof_tilt_cos;
    bool tof_correction_enabled;
    alt_tof_track_state_t tof_track_state;
    uint32_t last_processed_tof_seq;
    bool tof_seq_init;
    int64_t last_tof_timestamp_us, last_tof_received_us;
    bool tof_new_sample;
    float tof_dt_s;
    uint32_t tof_accept_count, tof_reject_count;
    float prev_tof_z_m;
    int64_t prev_tof_z_timestamp_us;
    bool prev_tof_z_valid;
    uint8_t tof_reacquire_count;

    // Ground-floor calibration: Welford, chi khi stationary va chua lock.
    float floor_plane_z_m, tof_ground_range_m;
    bool tof_ground_ref_valid, floor_locked;
    uint16_t floor_sample_count;
    float floor_mean_m, floor_m2_m2, floor_std_m;
    int tof_surface_state, tof_floor_ticks, tof_other_ticks;
    float landing_surface_z_m;
    bool landing_surface_valid;

    // ---- TERRAIN (xem khoi TERRAIN OFFSET ben duoi). Cac field nay TON TAI
    // ke ca khi FC_FEATURE_TERRAIN_OFFSET = 0 (de format telemetry khong doi
    // theo co bien dich) — luc do terrain_off_m DUNG YEN o 0 va moi cong thuc
    // thu ve dang cu.
    float    terrain_off_m;        // do cao BE MAT dang nhin so voi san da khoa
    float    agl_m;                // alt_m - terrain_off_m (do cao tren BE MAT)
    bool     terr_pending;
    // Moc bat dau nghi ngo -- de huy nghi ngo khi qua TERR_PENDING_TIMEOUT_MS.
    int64_t terr_pending_since_us;         // dang NGHI co bac, chua COMMIT
    float    terr_cand_offset_m;   // offset ung vien dang cho xac nhan
    int      terr_confirm_cnt;     // so mau lien tiep da khop ung vien
    uint32_t terr_commit_count;    // so lan COMMIT (telemetry + landing reset pha)
    // So lan nghi ngo bi HUY vi het han (TERR_PENDING_TIMEOUT_MS). Con so nay
    // la thu can nhin khi tune TERR_JUMP_THRESH_M: cao lien tuc = nguong dat
    // qua thap, dang bao dong gia tren nhieu ToF binh thuong.
    uint16_t terr_timeout_count;
    // So lan COMMIT bi TU CHOI vi vuot TERR_MAX_STEP_M (sanity fail).
    uint16_t terr_reject_count;
    float    terr_residual_m;      // residual mau gan nhat (telemetry/tune)
    float    prev_tof_vertical_m;  // range da bu tilt cua mau TRUOC
    bool     prev_tof_vertical_valid;
} alt_estimator_t;

// Mot noi duy nhat cho tham so estimator IMU + ToF.
// Gate hinh hoc CAN DUOI cho mot mau range. HA 0.03 -> 0.01 (yeu cau nguoi
// dung): 3cm loai bo mau khi drone con NAM DAT — chinh la luc ta CAN mau nhat
// (chot mat san truoc takeoff, va bang chung roi dat o vai cm dau tien).
//
// ⚠ DANH DOI: L0X duoi ~3cm doc kem tin cay (crosstalk cua chinh cua so kinh,
// min-range-fail). Mau trong dai 0.01-0.03 nen coi la "co con hon khong", KHONG
// phai so do chinh xac. Neu thay tof_z_m nhay loan sat dat thi day la nghi can
// dau tien.
#define ALT_EST_TOF_MIN_RANGE_M              0.00f
// ============================================================================
// GATE HINH HOC cho MOT mau range tho — PHAI phu duoc TRAN BAY SAU BU TILT
// ============================================================================
// Rang buoc: gate >= ALT_EST_MAX_FLIGHT_Z_M / ALT_EST_TOF_TILT_MIN_COS
//            2.90    >= 2.50 / 0.87 = 2.87                              ✓
//
// VI SAO chia cho cos: ToF do theo TRUC CAM BIEN. Drone nghieng goc t thi de
// o do cao z, chum tia phai di quang duong z/cos(t). O tran 2.50m va nghieng
// het muc cho phep (cos=0.87, ~30 do) thi range THAT la 2.87m. Gate bang dung
// tran bay se cat mau ngay tai diem lam viec cao nhat — mat correction dung luc
// can nhat, va la mot loi chi xuat hien khi vua bay cao vua nghieng.
//
// ⚠⚠ CANH BAO VAT LY — SO NAY VUOT TAM TIN CAY CUA CHIP:
// VL53L1X @LONG mode tin cay ~2.6m trong nha (4m la so danh nghia, chi dat
// duoc voi be mat phan xa tot va it anh sang nen). Gate 2.90m nghia la:
//   - Bay o 2.50m VA nghieng manh -> range 2.87m -> NGOAI tam tin cay
//     -> mau bi chip tra ve range_status != 0 -> tof_reject_count TANG.
//   - Do la GIOI HAN CAM BIEN, khong phai loi phan mem.
//
// VI SAO VAN CHAP NHAN DUOC (khac han truoc day):
//   1. Innovation gate DA BO -> mat mau khong con dan toi soft-fault/auto-land.
//   2. Cua so dung sai TOF_STALE_TIMEOUT_MS (200ms) da HOAT DONG THAT -> vai
//      mau xau lien tiep khong lam mat nguon do cao.
//   3. alt_m coast bang tich phan accel trong luc do; nghieng manh o tran bay
//      la trang thai NGAN, khong phai che do bay lau dai.
// Truoc khi co (1) va (2), cau hinh nay se tu ha canh giua chung.
//
// NEU THAY tof_reject_count tang deu khi bay cao: do la vat ly, khong phai bug.
// Cach chua THAT su la ha tran bay, khong phai noi gate them.
#define ALT_EST_TOF_MAX_RANGE_M              2.90f

// ============================================================================
// TRAN BAY (geofence max — COMMANDER_DEFAULT_ALT_MAX_M lay thang tu day)
// ============================================================================
// 2.50m theo yeu cau nguoi dung (truoc: 3.00m, va 3.00 KHONG kha thi vi no doi
// gate 3.45m — xa hon ca tam danh nghia cua chip o goc nghieng lon).
//
// ⚠ DOI SO NAY THI PHAI DOI ALT_EST_TOF_MAX_RANGE_M THEO, theo dung cong thuc
// o tren. Hai hang so nay mac noi tiep: nang tran ma quen nang gate thi mau bi
// cat dung tai tran, va trieu chung se la "bay cao thi do cao nhay loan" chu
// khong phai mot loi ro rang. test_tof_bringup_offline.py kiem rang buoc nay.
#define ALT_EST_MAX_FLIGHT_Z_M               2.50f

// cos(tilt) toi thieu de mot mau ToF con duoc coi la do duoc hinh hoc.
// 0.87 ~ 30 do. Ha so nay (cho phep nghieng hon) se lam range yeu cau tang
// theo 1/cos — xem rang buoc o ALT_EST_TOF_MAX_RANGE_M.
#define ALT_EST_TOF_TILT_MIN_COS             0.87f
// TAM THOI: ToF dang co hien tuong reset/seq dut quang, nen chot ngay mau
// geometry-hop-le dau tien. Doi lai 30/50 khi driver da on dinh.
//
// ⚠ DA THU NANG LEN 8/16 (spec B3 "trung binh N mau") VA PHAI LUI LAI:
// floor_add() co dong `if (valid || count >= MAX_SAMPLES) return;`. Voi 1/1 no
// vo hai vi mau dau tien luon chot duoc. Voi 8/16, neu cua so dau tien co
// std vuot ALT_EST_FLOOR_MAX_STD_M thi floor_sample_count NAM LI o MAX,
// floor_add() return vinh vien, tof_ground_ref_valid KHONG BAO GIO true ->
// prearm_check() TU CHOI ARM MAI MAI (ARMREJ=ALT_EST_INVALID), khong co duong
// thoat nao ngoai reboot. Muon nang N thi PHAI sua floor_add() thanh "het cua
// so ma chua dat -> bat dau cua so moi" TRUOC, khong duoc doi rieng 2 so nay.
#define ALT_EST_FLOOR_MIN_SAMPLES            1
#define ALT_EST_FLOOR_MAX_SAMPLES            1
// Voi N=1 thi std LUON = 0 nen nguong nay hien la code chet — no chi song lai
// khi MIN_SAMPLES > 1 (xem canh bao ngay tren).
#define ALT_EST_FLOOR_MAX_STD_M              0.008f
#define ALT_EST_FLOOR_RESTART_DELTA_M        0.030f
// Dai chet quanh 0: zt duoi muc nay bi EP ve dung 0.0 (alt_estimator.c) de
// drone nam dat khong bao cao mot do cao am/lat phat vai mm.
//
// ⚠ PHAI <= TAKEOFF_LIFTOFF_Z_M, KHONG DUOC LON HON. Ban dau 0.015 trong khi
// LIFTOFF_Z ha xuong 0.01 -> moi gia tri trong [0.01, 0.015) bi ep ve 0 TRUOC
// khi detector doc, nen ve `tof_z_m >= TAKEOFF_LIFTOFF_Z_M` thanh mot dieu kien
// KHONG BAO GIO dung duoc va takeoff se abort NO_LIFT_EVIDENCE mai mai. Ha
// xuong 0.005 de dai chet nam HAN duoi nguong roi dat.
#define ALT_EST_GROUND_ZERO_BAND_M           0.005f
// ALT_EST_TOF_INNOV_GATE_M: DA BO (xem alt_estimator.c, khoi "INNOVATION GATE").
// Gate nay bien "bay qua vat the" thanh "tu dong ha canh". Thay bang slew-rate
// limit ngay ben duoi: lam so do doi MUOT thay vi LOAI BO mau.

// Toc do toi da (m/s) ma tof_z_m duoc phep doi. Day la bo loc chong nhay dot
// ngot DUY NHAT con lai tren duong ToF.
//
// 1.5 m/s chon theo VAT LY, khong phai theo cam tinh:
//   - Toc do leo/ha THAT cua drone nay bi chan boi ALT_HOLD_VZ_LIMIT_MS va
//     LAND_* (deu <= ~0.5 m/s), nen 1.5 m/s KHONG BAO GIO can tro chuyen dong
//     that — no chi cat nhung buoc nhay nhanh hon moi thu drone lam duoc.
//   - Bay qua vat cao 0.5m: buoc nhay 0.5m se duoc trai ra ~0.33s. Du cham de
//     vong Vz khong giat, du nhanh de bam kip dia hinh.
//   - Mot mau ToF loi (range nhay 2m) bi cat con ~0.06m/mau @40ms -> gan nhu
//     vo hai, va mau ke tiep dung lai se keo ve ngay.
#define ALT_EST_TOF_MAX_SLEW_MS              1.5f
#define ALT_EST_TOF_REACQUIRE_SAMPLES        3
#define ALT_EST_TOF_TRACK_MAX_AGE_MS         100
#define ALT_EST_TOF_BRIDGE_MS                220
#define ALT_EST_TOF_LOST_MS                  300
#define ALT_EST_TOF_GAP_DERIV_MAX_MS         120
#define ALT_EST_TOF_DERIV_JUMP_M             0.20f
#define ALT_EST_TOF_VZ_LPF_HZ                7.0f
#define ALT_EST_TOF_Z_GAIN                   0.35f
#define ALT_EST_TOF_VZ_GAIN                  0.25f
#define ALT_EST_TOF_INNOV_VZ_GAIN            0.02f
#define ALT_EST_ACCEL_LPF_HZ                 20.0f
#define ALT_EST_ACCEL_DEADBAND_MS2           0.08f
#define ALT_EST_ACCEL_BIAS_RATE_HZ           0.05f
#define ALT_EST_ACCEL_BIAS_LIMIT_MS2         0.50f
#define ALT_EST_STATIONARY_GYRO_DPS          2.0f
#define ALT_EST_STATIONARY_ACCEL_TOL_G       0.05f
#define ALT_EST_SOURCE_ACTIVE_MS             ALT_EST_TOF_TRACK_MAX_AGE_MS
#define ALT_EST_NO_CORRECTION_DEGRADED_MS    ALT_EST_TOF_LOST_MS

// ============================================================================
// TERRAIN OFFSET (FC_FEATURE_TERRAIN_OFFSET <- app_config.h TERRAIN_OFFSET_ENABLED)
// ============================================================================
// VAN DE: alt_m la do cao tren MAT SAN DA KHOA luc cat canh. Bay qua mot cai
// ban cao 0.75m thi range tut 0.75m trong MOT mau -> innovation -0.75 vuot
// ALT_EST_TOF_INNOV_GATE_M (0.25) -> tof_surface_state = OTHER -> tof_fusable
// = false -> sau ALT_EST_TOF_LOST_MS thi valid = false -> Commander soft-fault
// -> LANDING. Tuc la "bay qua vat the" = "tu dong ha canh".
//
// CACH LAM: giu MOT so duy nhat terrain_off_m = do cao BE MAT DANG NHIN so voi
// san da khoa. Moi phep tinh Z deu di qua no:
//
//     agl_m   = tof_vertical_m - tof_ground_range_m      (do cao tren BE MAT)
//     tof_z_m = agl_m + terrain_off_m                    (do cao tren SAN)
//
// San takeoff: terrain_off_m = 0. Tren mat ban 0.75m: terrain_off_m = 0.75.
// COMMIT offset DUNG BANG buoc nhay -> tof_z_m (va qua do alt_m) LIEN TUC
// xuyen qua cu nhay -> PID khong thay gi bat thuong -> KHONG can tat PID.
//
// PHAT HIEN BANG RESIDUAL, KHONG PHAI NGUONG THO:
//     d_range  = range - range_prev
//     expected = vz_est * dt          (drone leo thi range tang)
//     residual = d_range - expected
// Co so nguong: |vz| bi tran o ALT_HOLD_VZ_LIMIT_MS (0.5 m/s) va ToF ~30Hz,
// nen MOT mau chuyen dong toi da dich ~0.017m. 0.12m KHONG THE do chuyen dong
// cua drone tao ra.
#define TERR_JUMP_THRESH_M                   0.12f
// Mep ban la cho te nhat: FoV 25deg o 1m cho vet sang ~44cm, beam nua tren ban
// nua hut xuong san -> range nhay qua nhay lai. Phai xac nhan N mau LIEN TIEP
// khop ung vien moi duoc COMMIT.
#define TERR_CONFIRM_N                       4
#define TERR_CONFIRM_TOL_M                   0.10f
// Guard khoang ho toi thieu (luoi an toan CUOI, B8). Duoi muc nay -> ep leo
// bat ke frame nao dang chon, VA rebase terrain_off_m ngay (rebase thay vi
// danh nhau voi PID). Du logic offset co sai, drone van khong cam xuong ban.
#define TERR_MIN_CLEARANCE_M                 0.25f
#define TERR_ESCAPE_VZ_MS                    0.20f

// ============================================================================
// TERR_PENDING_TIMEOUT_MS -- LOI THOAT BAT BUOC CHO terr_pending
// ============================================================================
// ⚠ DAY LA BAN VA CHO LOI DA LAM TINH NANG NAY BI TAT (app_config.h). Log that
// do duoc tren bo:
//     TOFF=-0.315  TPEND=1  TCMT=1  TOFFUSE=0  TOFTRACK=0
//     TOFR 83->95 (tang deu)   TOFA=177..178 (DUNG YEN)
//
// VONG LUAN QUAN:
//   terr_pending = true  ->  tof_fusable = false   (dung theo thiet ke: dang
//                                                   nghi ngo thi khong an range)
//   tof_fusable = false  ->  khong co mau nao de so voi ung vien
//   khong co mau         ->  terr_confirm_cnt khong bao gio tang, VA cung
//                            khong bao gio bi reset ve 0
//   =>  terr_pending KET O 1 VINH VIEN
//
// Sau ALT_EST_TOF_LOST_MS khong co correction -> valid = false -> ALTSRC=4
// (TOF_LOST) -> commander soft-fault -> LANDING giua chuyen bay.
//
// CACH SUA: nghi ngo co HAN. Het han ma chua confirm duoc thi HUY nghi ngo va
// quay lai fuse binh thuong -- chap nhan mot buoc nhay do cao con hon mat han
// nguon do cao roi tu ha canh.
//
// 200ms -- PHAI NGAN HON ALT_EST_TOF_LOST_MS (300ms), co _Static_assert bao ve.
// Neu dat dai hon thi soft-fault se no TRUOC khi loi thoat kip chay, va ban va
// nay thanh vo dung. Van du cho TERR_CONFIRM_N=4 mau o nhip ToF ~40ms (L1X)
// ke ca khi truot mot vai mau.
#define TERR_PENDING_TIMEOUT_MS              200

// B3 -- SANITY CHECK truoc khi COMMIT offset.
// Mot buoc terrain trong nha khong the lon hon nay: ban ~0.75m, ghe ~0.45m,
// tu ~1.8m nhung bay ben tren tu thi ToF het tam truoc da. Vuot nguong nay
// nghia la do sai (mau rac / phan xa gong / ToF ngoai tam), khong phai co
// mot cai bac that cao nhu vay -> TU CHOI commit, giu offset cu.
#define TERR_MAX_STEP_M                      1.50f

// Frame do cao — CHON DUOC LUC RUNTIME (fc.set_param("alt_frame", 0|1)).
typedef enum {
    // Giu do cao so voi SAN da khoa (mac dinh, hanh vi cu). Bay qua ban thi
    // khoang ho GIAM dung bang chieu cao ban.
    ALT_FRAME_DATUM = 0,
    // Giu KHOANG CACH so voi BE MAT dang nhin (terrain following): qua ban thi
    // drone LEO LEN dung bang chieu cao ban.
    ALT_FRAME_AGL,
} alt_frame_t;
#if FC_FEATURE_BARO
#define ALT_EST_BARO_LPF_HZ                  2.0f
#endif

void alt_estimator_reset(alt_estimator_t *e);
void alt_estimator_prepare_takeoff(alt_estimator_t *e);
bool alt_estimator_floor_ready(const alt_estimator_t *e);

// alt_estimator_lock_floor_fallback() — chốt gốc toạ độ bằng mẫu ToF hợp lệ
// HIỆN TẠI (thay vì trung bình cửa sổ ổn định như alt_estimator_lock_floor()).
// CHỈ dùng khi cổng floor đã tắt (FC_FEATURE_FLOOR_GATE=0, xem fc_features.h).
// Kém chính xác hơn nhưng hơn hẳn việc giữ nguyên gốc toạ độ của lần bay trước.
// false = không có mẫu ToF nào dùng được -> caller PHẢI từ chối lệnh.
bool alt_estimator_lock_floor_fallback(alt_estimator_t *e);
bool alt_estimator_lock_floor(alt_estimator_t *e);
void alt_estimator_unlock_floor(alt_estimator_t *e);
void alt_estimator_confirm_liftoff(alt_estimator_t *e);
bool alt_estimator_select_landing_surface(alt_estimator_t *e);

// DO CAO TREN BE MAT HA CANH = AGL. LANDING LUON chay tren so nay, TUYET DOI
// KHONG chay tren alt_m (datum): dang bay tren ban ma bam land thi phai ha
// xuong MAT BAN, khong phai co xuong cao do san (lam vay la DAM BAN).
// FC_FEATURE_TERRAIN_OFFSET = 0 -> terrain_off_m luon 0 -> tra ve dung alt_m
// nhu ban cu.
float alt_estimator_height_above_landing_surface(const alt_estimator_t *e);

// AGL DA LOC (= alt_m - terrain_off_m): do cao tren BE MAT dang nhin. Dung cho
// frame ALT_FRAME_AGL va guard khoang ho. Lien tuc ca khi ToF mat mau (coast
// bang tich phan accel) — KHAC alt_estimator_tof_agl_m() von la so DO THO.
float alt_estimator_agl_m(const alt_estimator_t *e);

// AGL THO tu mau ToF gan nhat (tof_z_m - terrain_off_m). Dung cho bang chung
// cham dat cua landing (can so DO, khong phai so tich phan).
float alt_estimator_tof_agl_m(const alt_estimator_t *e);

// REBASE terrain NGAY theo range ToF hien tai — KHONG cho xac nhan N mau.
// CHI dung cho guard khoang ho toi thieu (B8): luc do da co bang chung vat ly
// rang be mat gan hon nhieu so voi model, va cho them 4 mau nua la cho them
// mot cu va cham. Tra false neu khong co mau ToF dung duoc.
bool alt_estimator_terrain_rebase(alt_estimator_t *e);
// Chot goc toa do = 0 khi KHONG co mau ToF nao dung duoc (nam sat san, ToF doc
// 0.000m duoi tam mu). Duong cuoi de takeoff KHONG bi tu choi chi vi drone dang
// nam dung cho no phai nam. Xem giai thich day du trong alt_estimator.c.
void alt_estimator_lock_floor_at_zero(alt_estimator_t *e);

void alt_estimator_update(alt_estimator_t *e, vec3f_t acc_body_g, quat_t q,
#if FC_FEATURE_BARO
                          bool baro_healthy, uint32_t baro_seq,
                          int64_t baro_timestamp_us, float baro_alt_m,
#endif
                          bool tof_healthy, uint32_t tof_seq,
                          int64_t tof_timestamp_us, float tof_range_m,
                          // tof_hw_alive = chip VAN DANG DO, khac han "co mau
                          // dung duoc". Nam sat san (0mm, duoi tam mu), nhin ra
                          // khoang khong hay be mat hap thu deu cho tof_healthy
                          // = false NHUNG tof_hw_alive = true. Chi khi chip chet
                          // / bus dut thi no moi false.
                          //
                          // Dung DUY NHAT de tra loi "con cam bien khong". Cau
                          // "co so de bay khong" van la viec cua tof_healthy.
                          // Truyen tof_alive_us tu sensor_snapshot_t.
                          bool tof_hw_alive,
                          bool stationary, bool liftoff_candidate, bool airborne,
                          int64_t now_us, float dt);
void alt_estimator_correct_velocity(alt_estimator_t *e, float vz_measured_ms,
                                    bool measurement_valid, float dt);

#ifdef __cplusplus
}
#endif
