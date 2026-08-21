// Altitude estimator — INERTIAL-PRIMARY: Vz đến TỪ TÍCH PHÂN ACCELERATION,
// barometer (BMP280) CHỈ là anchor CHẬM chống trôi dài hạn, KHÔNG BAO GIỜ tạo
// Vz bằng đạo hàm d(baro)/dt. Học nguyên lý kiến trúc Z/Vz của 01Studio
// pyDrone (prediction bằng accel world-frame, correction bằng measurement
// chậm, KHÔNG set thẳng estimate = measurement mỗi mẫu) — KHÔNG copy số/
// register của pyDrone (họ dùng SPL06-001, project này dùng BMP280 qua
// baro_driver.c riêng). Nhận quaternion qua tham số (DI) — KHÔNG include
// ngược Mahony, giữ đúng ranh giới tầng.
//
// HAI NGUỒN CORRECTION, VAI TRÒ KHÁC HẲN NHAU:
//   Az (IMU) : DYNAMICS/PREDICTION chính. Vz = ∫Az, Z = ∫Vz. Không gì thay thế.
//   ToF      : correction CÓ ĐIỀU KIỆN, chỉ khi đang nhìn ĐÚNG mặt sàn đã khoá.
//   Baro     : correction CHẬM cho world-Z, chống trôi dài hạn.
// KHÔNG BAO GIỜ: z = tof, z = baro, Vz = d(tof)/dt, Vz = d(baro)/dt.
//
// ⚠ ToF KHÔNG PHẢI cảm biến world-Z. Nó đo range tới BỀ MẶT ĐANG Ở DƯỚI và
// không biết đó là sàn hay bàn. Dùng thẳng làm độ cao thì bay qua một cái bàn
// cao 40cm sẽ khiến controller tưởng UAV vừa tụt 40cm và đẩy ga lên — UAV tự
// bốc lên đúng bằng chiều cao cái bàn. Xem "MẶT SÀN ĐÃ KHOÁ" bên dưới.
//
// ============================================================================
// BA PHA: GROUND -> LIFTOFF_CANDIDATE -> AIRBORNE
// ============================================================================
// GROUND (inertial KHÔNG chạy):
//   Biết CHẮC CHẮN chưa rời đất -> khoá CỨNG alt_m=0, vz_ms=0, z_inertial_m=0,
//   vz_accel_only_ms=0 MỖI TICK. KHÔNG tích phân accel. Vẫn chạy: LPF accel,
//   học accel_bias khi đứng yên, prefilter + telemetry baro, baro health.
//   Baro correction TẮT (ground truth Z=0 đã biết, không cần innovation gate
//   "tranh cãi" với inertial).
//
// LIFTOFF_CANDIDATE (inertial CHẠY, chưa bàn giao) = ĐANG SPOOL:
//   VÌ SAO PHẢI CÓ PHA NÀY: chuỗi cất cánh (takeoff_land.h) ram ga rồi CHỐT
//   ĐỘ CAO ĐANG CÓ làm target cho alt_hold. Nếu ground lock còn khoá alt_m=0
//   suốt pha TAKING_OFF thì cái chốt được sẽ LUÔN là 0 — alt_hold nhận target
//   0 và không giữ được gì. Nên phải có một pha TRUNG GIAN cho inertial chạy
//   TRƯỚC khi tuyên bố airborne.
//   (Lịch sử: pha này ra đời để phá một bug phụ thuộc vòng tròn với bộ dò rời
//   đất cũ — ground lock khoá Z/Vz=0 trong khi bộ dò lại chấm điểm bằng chính
//   Z/Vz đó nên không bao giờ confirm được. Bộ dò đã bị bỏ, nhưng pha này VẪN
//   cần vì lý do ở trên.)
//   Điều kiện mở do flight_core.c quyết định: state == TAKING_OFF.
//   Baro fusion CHƯA chạy ở pha này — sát đất baro nhiễu nhất, kéo Z về baro
//   lúc drone mới nhấc vài cm sẽ triệt tiêu đúng phần vừa tích phân được.
//
// AIRBORNE (inertial CHẠY + baro fusion CHẠY):
//   Từ lúc bàn giao xong (FSM rời TAKING_OFF) tới khi hạ cánh xong.
//
// ---- CHÍNH SÁCH CHUYỂN CANDIDATE -> AIRBORNE: **POLICY B (CONTINUITY)** ----
// KHÔNG reset Z/Vz khi confirmed. Lý do: trong candidate, Z/Vz được tích phân
// từ đúng mốc vật lý (Z=0 lúc còn trên đất, giải phóng đúng lúc ground lock
// nhả), nên tại thời điểm confirmed chúng đã là ĐỘ CAO THẬT (~5-10cm) và VẬN
// TỐC LÊN THẬT. Reset Vz=0 ở đây sẽ đưa cho alt_hold một lời nói dối ngay
// khoảnh khắc nhạy cảm nhất (drone đang đi lên với vz thật > 0 nhưng
// controller tưởng vz=0 -> đẩy ga thêm -> vọt lố). Reset Z=0 thì mất luôn
// 5-10cm đã leo được. Cả hai đều tệ hơn hẳn việc giữ nguyên.
//   Hệ quả: mốc "Z=0" của cả chuyến bay = điểm mà ground lock nhả (mặt đất),
//   TRÙNG với mốc 0m của baro (calibrate_ground() chạy ngay trước takeoff,
//   xem flight_core.c CMD_TAKEOFF) -> hai nguồn CÙNG một gốc toạ độ, đúng
//   điều kiện để innovation có nghĩa.
//
// ---- THOÁT CANDIDATE (không cần timeout riêng trong estimator) ----
// Candidate kết thúc khi FSM rời TAKING_OFF — bình thường là bàn giao sang
// HOLDING (sau spool_ms), hoặc bị Commander ép LANDING/EMERGENCY nếu có fault.
// Cả hai trường hợp flight_core.c đều tự hạ candidate; nếu chuyển về trạng
// thái mặt đất thì ground lock đóng lại và Z/Vz/shadow tự về 0 ngay tick sau.
// CỐ Ý KHÔNG thêm timeout thứ 2 bên trong estimator: hai bộ đếm độc lập cùng
// quyết định một việc sẽ có lúc lệch nhau, và cái ở estimator có thể zero hoá
// Z/Vz ĐÚNG LÚC drone đang thật sự lên — nguy hiểm hơn lợi ích nó mang lại.
//
// ---- THIẾT KẾ: bộ lọc alpha-beta-gamma làm tay (3 state) ----
//   z              : vị trí (m) — "alpha" state
//   vz             : vận tốc (m/s) — "beta" state
//   accel_bias_ms2 : bias gia tốc world-frame Z còn sót lại SAU calib 6-face
//                    — "gamma" state, ĐƠN VỊ m/s² (CÙNG đơn vị với az, xem
//                    ALT_EST_ACCEL_BIAS_MAX_MS2 để biết vì sao bị chặn biên).
//
// PREDICT (mỗi tick 250Hz):
//   1. Xoay accel body->world bằng quaternion, trừ 1g -> az_raw_ms2 (m/s²).
//   2. LPF (ALT_EST_VERT_ACC_LPF_HZ) -> az_lpf_ms2 (m/s²).
//   3. Học bias TỪ az_lpf_ms2 (khi stationary) — xem "THỨ TỰ" bên dưới.
//   4. az_corrected = az_lpf - bias, RỒI MỚI deadband.
//   5. Tích phân (chỉ khi inertial enabled) kiểu "constant-acceleration":
//        vz_old = vz;  vz += az*dt;  z += vz_old*dt + 0.5*az*dt*dt;
//
// ---- THỨ TỰ HỌC BIAS: LPF -> HỌC -> TRỪ -> DEADBAND (đã sửa) ----
// Bản cũ làm LPF -> DEADBAND -> HỌC -> TRỪ, và nó KHÔNG BAO GIỜ hội tụ được:
// residual tĩnh thật (~0.02-0.04 m/s² đo trên bo này) NHỎ HƠN deadband (0.05)
// nên sau deadband nó thành 0 tròn, bias học mãi về 0 — đúng triệu chứng
// ABIAS≈-0.003 trong khi VACC≈-0.03 quan sát được trên log thật. Deadband
// phải là bước CUỐI (chặn nhiễu quanh 0 SAU khi đã bù bias), không được đứng
// trước bộ học.
//
// CORRECT (baro — CHỈ khi AIRBORNE và CHỈ MỘT LẦN cho mỗi mẫu MỚI):
//   Mẫu mới nhận biết bằng SEQUENCE (baro_seq đổi), KHÔNG bằng cờ "healthy":
//   estimator chạy 250Hz còn BMP280 ~50Hz, nếu correction chạy mỗi tick với
//   cùng một mẫu thì Z bị kéo về baro mạnh gấp ~5 lần dự kiến và beta/dt cũng
//   sai hệ số tương ứng — nhìn ra ngoài giống hệt "Z không chịu tăng".
//   dt_baro lấy từ HIỆU 2 TIMESTAMP BARO THẬT (~20ms), KHÔNG phải dt vòng
//   điều khiển (~4ms).
//   Phân loại innovation = baro_filtered - z:
//     (a) fusion CHƯA init (cold start thật) -> snap z = baro_filtered.
//     (b) |innovation| <= GATE -> correction thường (ALPHA/BETA nhỏ).
//     (c) ngoài GATE, bất đồng NGẮN -> reject (spike/propwash).
//     (d) ngoài GATE, bất đồng CÙNG DẤU kéo dài >= REACQUIRE_DISAGREEMENT_MS
//         -> REACQUIRE: correction với ALPHA/BETA lớn hơn (vẫn << 1, KHÔNG
//         snap) — lối thoát khỏi deadlock "trôi -> reject -> trôi thêm".
//
// ---- BARO: HEALTH vs FUSION-INIT là HAI chuyện khác nhau ----
//   last_baro_received_us   : lần CUỐI nhận được mẫu mới (kể cả mẫu bị gate
//                             reject) -> dùng cho `valid` (cảm biến còn sống).
//   baro_fusion_initialized : fusion đã có gốc toạ độ chung với baro hay chưa.
//   Bản cũ gộp làm một (`had_good_baro`) và nó được set true ngay từ lúc chỉ
//   đang MONITOR trên mặt đất, khiến nhánh "snap lần đầu" không bao giờ chạy
//   như comment mô tả. Giờ fusion được đánh dấu initialized đúng tại cạnh
//   ground->inertial (lúc đó Z=0 và baro ref=0 TRÙNG nhau theo cấu tạo, nên
//   không cần snap gì cả — snap ở đó còn có hại vì sẽ nuốt nhiễu baro tức thời
//   vào làm gốc).
//
// 🔹 KIẾN TRÚC TASK: baro_driver_read() chạy trong sensor_task RIÊNG (xem
// sensor_hub.h) — task đó sở hữu bus I2C độc quyền và publish sensor_snapshot_t
// kèm seq + timestamp mỗi cảm biến. stabilize_task (nơi gọi hàm này, ~250Hz)
// chỉ memcpy snapshot, KHÔNG chạm I2C.
#pragma once

#include <stdint.h>

#include "flight_core/types.h"

#ifdef __cplusplus
extern "C" {
#endif

// alt_source_t — nguồn correction nào đang thực sự giữ world-Z đứng yên.
//
// KHÔNG phải "cảm biến nào đang bật", cũng KHÔNG phải "cảm biến nào healthy" —
// mà là "cảm biến nào có mẫu VỪA ĐƯỢC CHẤP NHẬN để sửa Z". Ba khái niệm đó
// tách nhau ra đúng ở trường hợp đáng lo nhất: ToF còn sống, còn trả mẫu đều,
// nhưng mọi mẫu đều bị innovation gate loại -> healthy=1 mà đóng góp=0.
typedef enum {
    ALT_SRC_NONE = 0,   // KHÔNG nguồn nào — Z đang thuần dead-reckon accel (sai số bậc hai)
    ALT_SRC_TOF  = 1,   // chỉ ToF
    ALT_SRC_BARO = 2,   // chỉ baro  <- đây là trạng thái "ToF chết, baro gánh"
    ALT_SRC_BOTH = 3,   // cả hai cùng đóng góp (bình thường khi dưới tầm ToF)
} alt_source_t;

static inline const char *alt_source_name(alt_source_t s) {
    switch (s) {
        case ALT_SRC_TOF:  return "TOF";
        case ALT_SRC_BARO: return "BARO";
        case ALT_SRC_BOTH: return "TOF+BARO";
        default:           return "NONE";
    }
}

typedef struct {
    float alt_m;      // ĐỘ CAO ƯỚC LƯỢNG CUỐI (m) — 0 khi GROUND
    float vz_ms;       // VẬN TỐC ƯỚC LƯỢNG CUỐI (m/s), dương = lên — 0 khi GROUND

    // ========================================================================
    // VALIDITY vs DEGRADED — HAI câu hỏi KHÁC NHAU
    // ========================================================================
    // valid    = "state có DÙNG ĐƯỢC không" — số hữu hạn, pipeline quán tính
    //            đang chạy. KHÔNG phụ thuộc baro hay ToF.
    // degraded = "state có còn ĐÁNG TIN không" — đã bao lâu KHÔNG có correction
    //            nào (từ BẤT KỲ nguồn nào). valid=1 degraded=1 nghĩa là Z vẫn là
    //            một con số hợp lệ nhưng đang dead-reckon thuần accel.
    //
    // ĐỔI SO VỚI BẢN TRƯỚC: valid từng = "baro còn gửi mẫu". Hệ quả là tắt baro
    // (hoặc baro rớt vài trăm ms) làm estimator bị tuyên bố CHẾT ngay, dù IMU và
    // ToF vẫn hoàn toàn tốt -> Commander ép LANDING vô cớ, và không bay được
    // bằng ToF một mình. IMU mới là nguồn prediction chính; baro/ToF chỉ sửa
    // trôi, nên mất chúng là DEGRADED chứ không phải INVALID.
    //
    // ⚠ Nhưng KHÔNG được bỏ luôn cảnh báo: không có correction thì sai số Z
    // tăng BẬC HAI theo thời gian (bias residual ~0.03 m/s² đo trên bo này ->
    // 3s ≈ 0.14m, 10s ≈ 1.5m, 30s ≈ 13.5m). Vì vậy mới có `degraded` + timeout.
    bool  valid;
    bool  degraded;               // quá lâu không có correction nào
    int64_t last_correction_us;   // lần CUỐI có correction ĐƯỢC CHẤP NHẬN (ToF HOẶC baro)
    int32_t no_correction_ms;     // tuổi correction gần nhất (telemetry)

    // ---- NGUỒN NÀO ĐANG THỰC SỰ GIỮ Z ----
    // `degraded` chỉ nói "có correction hay không". Nó KHÔNG nói correction đó
    // đến từ đâu. Trước đây baro đã âm thầm gánh khi ToF chết — hành vi đúng,
    // nhưng không ai nhìn thấy được: cùng một dòng telemetry cho cả trường hợp
    // "ToF tốt" lẫn "ToF chết 10s, đang bay mù bằng baro". Với một ToF chập
    // chờn thì đó chính là thông tin quan trọng nhất trên màn hình.
    //
    // Một nguồn được coi là ĐANG HOẠT ĐỘNG nếu correction gần nhất mà nó được
    // CHẤP NHẬN (không phải chỉ "healthy") còn mới hơn
    // ALT_EST_SOURCE_ACTIVE_MS. Dùng thời điểm ACCEPT chứ không dùng health là
    // có chủ đích: một ToF vẫn trả mẫu đều nhưng bị gate loại sạch (nhìn nhầm
    // bề mặt) là ToF KHÔNG đóng góp gì cho Z, dù `tof_healthy` vẫn bằng 1.
    int64_t last_tof_accept_us;
    int64_t last_baro_accept_us;
    alt_source_t active_source;

    // ---- Lượng sửa của LẦN correction gần nhất (telemetry/bench) ----
    // Innovation cho biết hai bên BẤT ĐỒNG bao nhiêu; mấy số này cho biết ta
    // thực sự ĐÃ SỬA bao nhiêu. Hai thứ khác nhau: innovation lớn mà correction
    // nhỏ nghĩa là gain thấp (đúng thiết kế); innovation nhỏ mà correction lớn
    // nghĩa là gain đang quá tay. Không tách ra thì không tune được.
    float tof_corr_z_m,  tof_corr_vz_ms;
    float baro_corr_z_m, baro_corr_vz_ms;

    // ---- Bias accel học TỪ RESIDUAL ĐỘ CAO (khác hẳn nhánh `stationary`) ----
    // Nhánh cũ chỉ học khi drone ĐỨNG YÊN (gyro/accel norm nhỏ) — tức là chỉ
    // học được lúc còn trên đất. Nhưng bias accel Z trôi theo NHIỆT ĐỘ, và
    // nhiệt độ đổi nhiều nhất CHÍNH LÚC ĐANG BAY (motor nóng, khí lưu). Không
    // có đường học lúc bay thì sai số DC của Az chỉ được sửa gián tiếp qua
    // correction vị trí, và Z sẽ trôi đều một chiều suốt chuyến bay.
    //
    // Đường mới: residual độ cao dai dẳng một chiều = bằng chứng Az đang lệch
    // DC. Cập nhật CỰC CHẬM để nó không tranh việc với alpha-beta (correction
    // vị trí phải xử lý sai số NHANH; bias chỉ xử lý phần TRÔI).
    float bias_residual_m;        // residual đã lọc, dùng để lái bias (telemetry)
    bool  bias_residual_init;
    uint32_t bias_adapt_count;    // số tick THẬT SỰ có adapt (bench: phải tăng khi bay)

    // ---- State thứ 3 (gamma) — ĐƠN VỊ m/s², bị chặn biên
    // ±ALT_EST_ACCEL_BIAS_MAX_MS2 ----
    float accel_bias_ms2;

    // ---- Shadow tracker DEBUG: tích phân accel THUẦN, baro KHÔNG BAO GIỜ
    // chạm — so với alt_m/vz_ms để thấy baro đang kéo bao nhiêu. Cũng bị khoá
    // 0 khi GROUND. TÊN ĐƠN VỊ: vz_accel_only_ms là VẬN TỐC (m/s) — trước đây
    // tên nó là *_ms2 (gia tốc) do sao chép nhầm hậu tố, đã sửa. ----
    float vz_accel_only_ms;
    float z_inertial_m;

    // ---- Vertical accel, TẤT CẢ đơn vị m/s² ----
    float az_raw_ms2;        // world-frame, đã trừ 1g, TRƯỚC LPF
    float az_lpf_ms2;        // SAU LPF, TRƯỚC trừ bias (đây là tín hiệu bộ học bias nhìn)
    float az_corrected_ms2;  // SAU trừ bias + deadband — giá trị THẬT được tích phân
    bool  az_lpf_init;

    // ---- Pha (xem "BA PHA") — lưu lại để bắt cạnh GROUND->inertial ----
    bool was_inertial_enabled;

    // ---- Baro pre-filter: median-of-3 rồi LPF ----
    float baro_med_hist[3];
    int   baro_med_count;
    int   baro_med_idx;
    float baro_lpf_alt_m;
    bool  baro_lpf_init;

    // ---- Baro sample tracking (xem "CORRECT" + "HEALTH vs FUSION-INIT") ----
    uint32_t last_processed_baro_seq;  // seq mẫu ĐÃ xử lý gần nhất — chống xử lý lại mẫu cũ
    bool     baro_seq_init;            // đã từng thấy seq nào chưa (seq hợp lệ bắt đầu từ 1)
    int64_t  last_baro_timestamp_us;   // timestamp mẫu baro trước -> dt_baro THẬT
    int64_t  last_baro_received_us;    // lần cuối NHẬN mẫu mới (kể cả bị reject) -> `valid`
    bool     baro_fusion_initialized;  // fusion đã có gốc toạ độ chung với baro
    bool     baro_new_sample;          // tick này CÓ mẫu baro mới không (telemetry)
    float    baro_innovation_m;        // baro_filtered - alt_m lần gần nhất
    float    baro_dt_s;                // dt THẬT giữa 2 mẫu baro (~0.02s @50Hz)
    uint32_t baro_accept_count;        // mẫu ĐÃ DÙNG sửa Z/Vz (gate thường HOẶC reacquire)
    uint32_t baro_reject_count;        // mẫu ngoài gate bị bỏ, cộng dồn

    // ---- Bất đồng dai dẳng + REACQUIRE (chỉ có nghĩa khi AIRBORNE) ----
    uint32_t baro_reject_consecutive;    // reject LIÊN TỤC hiện tại, reset khi accept
    // baro_disagreement_active TÁCH RIÊNG khỏi ..._since_us thay vì dùng 0 làm
    // sentinel "không bất đồng": now_us=0 là một MỐC THỜI GIAN HỢP LỆ, và khi
    // trùng sentinel thì chuỗi bất đồng bị tính lại từ đầu MỖI MẪU -> REACQUIRE
    // không bao giờ kích hoạt. Thực tế esp_timer_get_time() không bao giờ trả 0
    // lúc estimator chạy, nhưng một hằng số canh có hai nghĩa là loại lỗi chỉ
    // chờ đổi nguồn thời gian là bung ra.
    bool     baro_disagreement_active;
    int64_t  baro_disagreement_since_us; // mốc bắt đầu chuỗi bất đồng CÙNG DẤU
    float    baro_disagreement_sign;     // dấu innovation lúc bắt đầu chuỗi
    bool     baro_reacquire_active;      // đang dùng REACQUIRE_ALPHA/BETA

    // ========================================================================
    // ToF hướng xuống (VL53L0X) — CORRECTION CÓ ĐIỀU KIỆN THEO BỀ MẶT
    // ========================================================================
    // ToF đo range tới BỀ MẶT ĐANG Ở DƯỚI, và nó KHÔNG BIẾT đó là sàn, bàn,
    // ghế hay thùng carton. Nên nó KHÔNG PHẢI cảm biến world-Z: dùng thẳng
    // `z = tof` sẽ làm UAV tự bốc lên 40cm khi bay qua một cái bàn cao 40cm
    // (controller thấy "độ cao tụt" và đẩy ga). Toàn bộ khối dưới đây tồn tại
    // để phân biệt "mặt sàn đã khoá" với "một bề mặt khác".
    float tof_raw_m;             // range thô theo trục sensor
    float tof_vertical_m;        // đã chiếu về phương thẳng đứng (bù nghiêng)
    float tof_innovation_m;      // tof_vertical - range dự đoán tới SÀN ĐÃ KHOÁ
    float tof_surface_z_m;       // world-Z ước lượng của bề mặt ĐANG nhìn thấy
    float tof_tilt_cos;          // cos góc nghiêng (1.0 = phẳng)

    // floor_plane_z_m — world-Z của MẶT SÀN được khoá lúc cất cánh. KHÔNG BAO
    // GIỜ được đổi trong HOLDING/FLYING chỉ vì ToF nhìn thấy bề mặt khác lâu.
    float floor_plane_z_m;
    // tof_ground_range_m — range ToF đo được khi UAV NẰM TRÊN SÀN (sensor cách
    // sàn một khoảng vật lý, nên số này KHÁC 0). Thiếu nó thì mọi phép so range
    // dự đoán đều lệch đúng bằng chiều cao lắp sensor.
    float tof_ground_range_m;
    bool  tof_ground_ref_valid;

    // Phân loại bề mặt — xem alt_est_tof_surface_t.
    int   tof_surface_state;
    int   tof_floor_ticks;       // mẫu LIÊN TIẾP khớp sàn đã khoá
    int   tof_other_ticks;       // mẫu LIÊN TIẾP thấy bề mặt CAO hơn sàn
    bool  tof_correction_enabled;  // ToF có đang được phép sửa world-Z không

    uint32_t last_processed_tof_seq;
    bool     tof_seq_init;
    int64_t  last_tof_timestamp_us;
    int64_t  last_tof_received_us;
    bool     tof_new_sample;
    float    tof_dt_s;
    uint32_t tof_accept_count;   // mẫu ĐÃ dùng sửa Z/Vz
    uint32_t tof_reject_count;   // mẫu bị bỏ (bề mặt khác / nghiêng / ngoài tầm)

    // landing_surface_z_m — bề mặt được CHỌN làm đích hạ cánh khi có CMD_LAND.
    // TÁCH HẲN khỏi floor_plane_z_m: hạ xuống một cái bàn KHÔNG được ghi đè mốc
    // sàn của cả chuyến bay (xem alt_estimator_select_landing_surface()).
    float landing_surface_z_m;
    bool  landing_surface_valid;
} alt_estimator_t;

// Bề mặt mà ToF đang nhìn thấy — quyết định ToF có được sửa world-Z hay không.
typedef enum {
    ALT_EST_TOF_SURFACE_UNKNOWN = 0,  // chưa đủ bằng chứng (mới bay/ToF lỗi)
    ALT_EST_TOF_SURFACE_FLOOR,        // khớp mặt sàn đã khoá -> ĐƯỢC correction
    ALT_EST_TOF_SURFACE_OTHER,        // bề mặt khác (bàn/ghế/bục) -> CẤM correction
} alt_est_tof_surface_t;

// ============================================================================
// Vertical accel filtering — LPF phần mềm là lớp phòng thủ THỨ 2 (lớp 1 là
// DLPF PHẦN CỨNG của MPU6050, xem tuning.h mục 11). CHƯA đo trên phần cứng
// thật, điểm khởi đầu — tune bằng test G (README).
// ============================================================================
#define ALT_EST_VERT_ACC_LPF_HZ         10.0f   // cutoff LPF accel world-frame (Hz)
// Deadband áp SAU khi trừ bias (xem "THỨ TỰ HỌC BIAS") — chặn nhiễu quanh 0
// khỏi bị tích phân thành trôi, KHÔNG được đặt trước bộ học bias.
#define ALT_EST_VERT_ACC_DEADBAND_MS2   0.05f

// ============================================================================
// Accel bias học chậm (state gamma) — xem "THỨ TỰ HỌC BIAS".
// ============================================================================
#define ALT_EST_ACCEL_BIAS_LEARN_HZ     0.03f   // ~5.3s time-constant
// Chặn biên bias: residual thật sau calib 6-face chỉ cỡ vài chục mm/s². Nếu bộ
// học chạy tới cả m/s² thì nó KHÔNG còn đang học bias nữa mà đang hấp thụ một
// lỗi khác (attitude sai, calib accel hỏng, trục lắp sai) — chặn lại để lỗi đó
// lộ ra ở az_corrected thay vì bị giấu vào bias.
#define ALT_EST_ACCEL_BIAS_MAX_MS2      0.50f

// ---- HỌC BIAS TỪ RESIDUAL ĐỘ CAO (đường thứ HAI, độc lập với `stationary`) ----
// accel_bias_ms2 += -K * residual * dt
//
// DẤU: az_corrected = az_lpf - bias. Bias quá CAO -> az quá THẤP -> Z tụt lại
// so với cảm biến -> innovation (đo - ước lượng) DƯƠNG. Muốn sửa thì phải GIẢM
// bias, nên dấu là TRỪ. (Kiểm ngược: sensor lệch +0.1 m/s² lúc treo -> Z ước
// lượng bò lên -> innovation ÂM -> bias TĂNG về +0.1. Đúng.)
//
// ĐƠN VỊ K: (m/s²) thêm vào bias trên mỗi (mét residual × giây).
//
// 0.02 -> 0.30 sau khi ĐO trên mô phỏng baro-only (drone treo 1.00m, bias accel
// thật +0.13 m/s², nguồn duy nhất là baro 50Hz):
//     K=0.02  sau 180s: bias học được +0.012 / 0.13  -> sai số Z còn 1.6cm
//     K=0.30  sau 180s: bias học được +0.082        -> sai số Z về 0.0cm
//     K=1.00, 3.00: không tốt hơn đáng kể, không mất ổn định
// K=0.02 chậm tới mức gần như không đóng góp gì trong một chuyến bay vài phút.
//
// ⚠ VÌ SAO ĐỘ LỢI CHỈ KHIÊM TỐN: vòng alpha-beta vị trí VỐN ĐÃ khử phần lớn sai
// số DC của accel (1.7cm với bias 0.13). Đường học bias chỉ dọn nốt phần dư đó.
// Đừng kỳ vọng nó cứu được sai số lớn — sai số lớn trong cấu hình baro-only đến
// từ PROPWASH, và không tham số nào ở đây chạm tới được (xem ghi chú dưới).
//
// HAI rủi ro của việc nâng K đã được ĐO, cả hai đều KHÔNG xảy ra:
//   1. Học nhầm propwash thành bias: cho propwash 12cm suốt 60s rồi tắt đột
//      ngột — sai số sau đó GIỐNG HỆT nhau ở mọi K (0.1156m đỉnh, 0.0045m cuối).
//      Vòng vị trí kéo Z về trước khi bias kịp gây hại.
//   2. Bias chạy tới clamp khi baro TRÔI dài (thời tiết): cho baro trôi
//      0.002 m/s trong 5 phút — |bias| lớn nhất chỉ 0.0068 (clamp 0.50). Z bám
//      theo baro nên innovation về 0, bias không có gì để tích.
#define ALT_EST_BIAS_FROM_RESIDUAL_K       0.30f

// ⚠⚠ GIỚI HẠN THẬT CỦA CẤU HÌNH BARO-ONLY — đọc trước khi tune bất kỳ số nào ở
// trên. Đo trên mô phỏng, drone treo yên ở 1.00m:
//     bias accel 0.13 m/s²      -> sai số Z  1.7cm  (alpha-beta tự xử lý)
//     nhiễu baro 25cm           -> sai số Z 19.6cm đỉnh, 5.0cm ổn định
//     PROPWASH 10cm             -> sai số Z 10.0cm  <-- ánh xạ 1:1
//     PROPWASH 30cm             -> sai số Z 30.0cm  <-- ánh xạ 1:1
//
// Propwash ánh xạ MỘT-ĐỔI-MỘT sang sai số độ cao và KHÔNG bộ lọc nào loại được
// nó: đó là lệch HỆ THỐNG của phép đo, không phải nhiễu. Estimator có nhiệm vụ
// bám theo phép đo, nên nó bám theo cả phần sai.
//
// Cách sửa DUY NHẤT là VẬT LÝ: dán một miếng xốp/mút che lỗ thông của BMP280.
// Mọi flight controller thương mại đều làm vậy. Không có tham số phần mềm nào
// thay thế được miếng xốp đó.

// LPF trên residual TRƯỚC khi lái bias. Bias chỉ được phép phản ứng với phần
// DC; không lọc thì mỗi mẫu ToF/baro nhiễu sẽ lắc bias, mà bias lại nằm trong
// vòng tích phân -> nhiễu bị tích phân hai lần thành trôi vị trí.
#define ALT_EST_BIAS_RESIDUAL_LPF_HZ       0.20f

// Chỉ adapt khi gia tốc thẳng đứng ĐANG NHỎ. Lúc drone đang tăng/giảm tốc mạnh,
// residual phản ánh ĐỘ TRỄ của bộ lọc chứ không phải bias — học lúc đó là học
// nhầm dynamics thành DC error.
#define ALT_EST_BIAS_ADAPT_MAX_AZ_MS2      0.60f

// Chỉ adapt khi drone gần như KHÔNG NGHIÊNG. Nghiêng nhiều thì phép chiếu
// body->earth khuếch đại sai số attitude vào trục Z, và residual lúc đó chứa
// lỗi Mahony chứ không phải bias accel. cos(15°) ~ 0.966.
#define ALT_EST_BIAS_ADAPT_MIN_TILT_COS    0.966f

// Residual lớn hơn mức này = KHÔNG phải trôi DC (va chạm, đổi bề mặt, spike).
// Bỏ qua để một sự kiện đơn lẻ không kéo bias đi.
#define ALT_EST_BIAS_ADAPT_MAX_RESIDUAL_M  0.30f

// stationary: quyết định ở flight_core.c, CHỈ true khi CHƯA inertial-enabled
// VÀ gyro/accel cho thấy đứng yên NGAY TỨC THÌ.
#define ALT_EST_STATIONARY_GYRO_DPS       2.0f    // |gyro| mọi trục dưới ngưỡng này
#define ALT_EST_STATIONARY_ACCEL_TOL_G    0.05f   // |accel_norm - 1.0| dưới ngưỡng này

// ============================================================================
// Baro pre-filter + correction (alpha/beta) + innovation gate + reacquire.
// TẤT CẢ CHƯA tune trên phần cứng thật — điểm khởi đầu theo khoảng đề xuất.
// ============================================================================
#define ALT_EST_BARO_LPF_HZ             2.0f    // cutoff LPF baro sau median-of-3 (Hz)
#define ALT_EST_BARO_INNOVATION_GATE_M  0.6f    // ngoài ngưỡng -> reject (trừ khi REACQUIRE)
#define ALT_EST_BARO_ALPHA              0.03f   // sửa Z lúc bình thường — RẤT nhỏ
#define ALT_EST_BARO_BETA               0.003f  // sửa Vz lúc bình thường (chia dt_baro) — RẤT nhỏ
#define ALT_EST_BARO_TIMEOUT_MS         500     // mất baro quá lâu -> valid=false

// Bất đồng CÙNG DẤU liên tục bao lâu thì coi là nghi INERTIAL DRIFT (không
// phải spike baro) -> vào REACQUIRE. Đủ dài để loại spike thật (<1s), đủ ngắn
// để estimator không trôi hàng chục giây trước khi được kéo lại.
#define ALT_EST_BARO_REACQUIRE_DISAGREEMENT_MS   3000
// Alpha/beta khi REACQUIRE — lớn hơn hẳn bản thường nhưng VẪN << 1.0 (KHÔNG
// snap z=baro giữa chuyến bay).
#define ALT_EST_BARO_REACQUIRE_ALPHA    0.15f
#define ALT_EST_BARO_REACQUIRE_BETA     0.01f

// TRẦN CỨNG cho phần Vz mà MỘT mẫu baro được phép sửa (m/s).
//
// VÌ SAO CẦN: công thức alpha-beta chuẩn là beta*innovation/dt_baro — số chia
// dt_baro (~0.02s) KHUẾCH ĐẠI innovation lên ~50 lần. Ở chế độ thường
// innovation bị gate chặn ở 0.6m nên phần sửa tối đa ~0.09 m/s, chấp nhận
// được. Nhưng ở REACQUIRE innovation THEO ĐỊNH NGHĨA đã vượt gate và có thể
// tới vài mét: innovation 5m -> 0.01*5/0.02 = 2.5 m/s bị nhồi vào Vz CHỈ TỪ
// MỘT mẫu baro. Đó là đúng thứ mà "baro không được làm Vz rung" cấm.
//
// Về mặt vật lý: sai số VỊ TRÍ tích luỹ lâu ngày KHÔNG hàm ý sai số VẬN TỐC
// lớn tương ứng — kéo Z về (nhánh alpha, không chặn) mới là việc của
// reacquire; nhánh beta chỉ nên gỡ trôi vận tốc từ từ. 0.10 m/s vừa đủ cao để
// KHÔNG bao giờ chạm ở chế độ thường (tối đa 0.09), nên đây thuần tuý là lưới
// an toàn cho các trường hợp bệnh lý.
#define ALT_EST_BARO_VZ_CORRECTION_MAX_MS   0.10f

// ============================================================================
// ToF (VL53L0X hướng xuống) — CORRECTION CÓ ĐIỀU KIỆN THEO BỀ MẶT
// ============================================================================
// ⚠ TẤT CẢ ngưỡng dưới CHƯA đo trên phần cứng thật — điểm khởi đầu.
//
// Dải tin được. VL53L0X ~2m ở điều kiện tốt; sát 0 là vùng chip trả rác.
#define ALT_EST_TOF_MIN_RANGE_M        0.03f
#define ALT_EST_TOF_MAX_RANGE_M        1.80f

// cos(góc nghiêng) tối thiểu để CHẤP NHẬN mẫu. ToF đo theo trục sensor; nghiêng
// nhiều thì tia bắn xiên, range dài ra và điểm chạm lệch sang chỗ khác hẳn —
// bù cos không còn đúng vì nó giả định vẫn cùng một mặt phẳng. 0.87 ≈ 30°.
#define ALT_EST_TOF_TILT_MIN_COS       0.87f

// Gate phân loại bề mặt (so với range DỰ ĐOÁN tới sàn đã khoá).
// FLOOR: |innovation| nhỏ -> đang nhìn đúng mặt sàn.
#define ALT_EST_TOF_FLOOR_MATCH_GATE_M     0.10f
// OTHER: innovation ÂM lớn -> bề mặt CAO HƠN sàn (bàn/ghế/bục).
// Phải lớn hơn FLOOR gate rõ rệt để có vùng chết, tránh nhấp nháy giữa 2 trạng
// thái khi bay qua mép bàn.
#define ALT_EST_TOF_SURFACE_STEP_GATE_M    0.18f
// Số mẫu LIÊN TIẾP phải duy trì mới đổi trạng thái (ToF ~30Hz -> 3 mẫu ~100ms).
#define ALT_EST_TOF_FLOOR_TICKS            3
#define ALT_EST_TOF_OTHER_TICKS            3
// Quay LẠI FLOOR khó hơn vào FLOOR lần đầu (hysteresis): gate CHẶT hơn + cần
// nhiều mẫu hơn. Đây là thứ chặn nhấp nháy ở mép bàn.
#define ALT_EST_TOF_REACQUIRE_GATE_M       0.07f
#define ALT_EST_TOF_REACQUIRE_TICKS        6

// alpha/beta ToF — MẠNH HƠN baro nhiều (ToF chính xác cm, baro nhiễu dm) nhưng
// VẪN << 1.0: KHÔNG BAO GIỜ set thẳng z = tof.
//
// ---- alpha 0.25 -> 0.40: correction NHANH HƠN ----
// ToF publish ở 31.25Hz (SENSOR_TOF_DIVISOR=8 trên vòng 250Hz). Correction
// alpha-beta là bộ lọc bậc 1 với hằng số thời gian ~ 1/(f_tof * alpha):
//     alpha 0.25 -> tau = 1/(31.25*0.25) = 0.128s
//     alpha 0.40 -> tau = 1/(31.25*0.40) = 0.080s
// Trễ ước lượng là MỘT trong ba nguồn gây vọt lố (cùng quán tính và trễ lực
// đẩy): controller điều khiển theo Z nó NHÌN THẤY, nên Z trễ 0.128s nghĩa là nó
// bắt đầu hãm muộn 0.128s. Hạ xuống 0.080s là bớt được 1/3 phần trễ đó.
//
// ⚠ CÁI GIÁ, phải biết trước khi nâng thêm: trong cửa sổ chờ xác nhận đổi bề
// mặt (ALT_EST_TOF_OTHER_TICKS = 3 mẫu), correction VẪN chạy. Mỗi mẫu kéo Z đi
// alpha * innovation, mà innovation bị chặn bởi ALT_EST_TOF_FLOOR_MATCH_GATE_M
// (0.10m). Nên sai lệch tệ nhất trước khi bề mặt lạ bị nhận ra:
//     alpha 0.25 -> 3 * 0.25 * 0.10 = 0.075m
//     alpha 0.40 -> 3 * 0.40 * 0.10 = 0.120m
// 12cm vẫn nằm trong tầm chịu được (và gate 0.10m mới là thứ chặn chính).
// KHÔNG nâng alpha lên nữa nếu chưa siết FLOOR_MATCH_GATE hoặc OTHER_TICKS —
// hai số đó mới là hàng rào an toàn thật, alpha chỉ là tốc độ.
//
// beta 0.02 -> 0.035: phần sửa Vz đi kèm, giữ cùng tỉ lệ với alpha để không
// lệch pha giữa hai thành phần (beta/alpha giữ ~0.08-0.09).
#define ALT_EST_TOF_ALPHA                  0.40f
#define ALT_EST_TOF_BETA                   0.035f
// Trần cứng phần Vz mà MỘT mẫu ToF được sửa — cùng lý do như baro (beta/dt
// khuếch đại innovation ~30 lần ở dt=0.033s).
#define ALT_EST_TOF_VZ_CORRECTION_MAX_MS   0.30f

// Cửa sổ coi một nguồn là "đang hoạt động" (xem alt_source_t).
// Phải RỘNG hơn khoảng cách giữa hai mẫu của nguồn CHẬM nhất, nếu không chỉ báo
// sẽ nhấp nháy giữa TOF và TOF+BARO mỗi vài tick và thành vô dụng.
// ToF ~31Hz (32ms), baro ~50Hz (20ms). 500ms = ~15 mẫu ToF -> im lặng, nhưng
// vẫn đủ nhanh để thấy ToF chết trong vòng nửa giây.
#define ALT_EST_SOURCE_ACTIVE_MS           500

// Mất ToF quá lâu -> hạ tof_correction_enabled (KHÔNG hạ estimator.valid: IMU
// mới là nguồn prediction chính, xem "SENSOR FALLBACK").
#define ALT_EST_TOF_TIMEOUT_MS             300

// ============================================================================
// DEGRADED — bao lâu KHÔNG có correction nào thì hết tin được Z
// ============================================================================
// Sai số dead-reckoning thuần accel tăng BẬC HAI: 0.5*a_bias*T².
// Với bias residual ~0.03 m/s² (đo trên bo này):
//     3s -> 0.14m | 5s -> 0.38m | 10s -> 1.5m | 30s -> 13.5m
// 3000ms chọn ở mức sai số còn ~14cm — vẫn cứu được bằng hạ cánh êm.
//
// ⚠ HỆ QUẢ TRỰC TIẾP nếu TẮT BARO: khi ToF chuyển OTHER (bay qua bàn/ghế) thì
// KHÔNG CÒN nguồn correction nào. Bay trên bàn quá 3s sẽ trip degraded ->
// Commander soft fault -> LANDING. Đó KHÔNG phải bug: bay ToF-only trên một bề
// mặt mà ToF không dùng được thì Z THẬT SỰ đang trôi tự do. Muốn bay lâu qua
// bàn thì PHẢI bật baro (SENSOR_BARO_ENABLED=1).
#define ALT_EST_NO_CORRECTION_DEGRADED_MS   3000

void alt_estimator_reset(alt_estimator_t *e);

// alt_estimator_prepare_takeoff() — EVENT gọi MỘT LẦN khi CMD_TAKEOFF được
// accept, TRƯỚC khi chuỗi cất cánh bắt đầu (xem flight_core.c CMD_TAKEOFF).
//
// Zero Z/Vz + 2 shadow, và — đây mới là phần quan trọng — XOÁ trạng thái bất
// đồng/REACQUIRE baro còn sót từ chuyến trước. Ground lock zero Z/Vz mỗi tick
// nhưng KHÔNG chạm tới mấy field reacquire đó, nên không có hàm này thì một
// chuyến kết thúc giữa lúc REACQUIRE sẽ để chuyến sau khởi động với alpha/beta
// LỚN ngay mẫu baro đầu tiên sau liftoff — đúng lúc sát đất baro nhiễu nhất.
//
// GIỮ NGUYÊN: valid (mốc baro không đổi — calib chạy lúc ARM, không phải ở đây),
// accel_bias_ms2 đã học, filter accel, prefilter/seq/fusion_initialized của baro.
// KHÁC reset() (xoá bias) và reanchor() (đặt valid=false vì mốc vừa đổi).
void alt_estimator_prepare_takeoff(alt_estimator_t *e);

// alt_estimator_set_tof_ground_ref() — chốt range ToF đo được khi UAV NẰM TRÊN
// SÀN. Gọi lúc ARM (drone đứng yên trên sàn), cùng chỗ với calib baro.
//
// VÌ SAO CẦN: sensor lắp cách sàn một khoảng vật lý, nên khi UAV nằm đất ToF
// KHÔNG đọc 0. Thiếu số này thì mọi so sánh "range dự đoán tới sàn" lệch đúng
// bằng chiều cao lắp — đủ để phân loại nhầm sàn thành bàn.
void alt_estimator_set_tof_ground_ref(alt_estimator_t *e, float ground_range_m);

// alt_estimator_lock_floor() — KHOÁ mặt phẳng sàn tại world-Z hiện tại (=0 lúc
// cất cánh). Sau khi khoá, ToF chỉ được sửa world-Z khi nó thật sự đang nhìn
// ĐÚNG mặt phẳng này. Gọi ở CMD_TAKEOFF.
void alt_estimator_lock_floor(alt_estimator_t *e);

// alt_estimator_select_landing_surface() — chọn bề mặt ĐANG nhìn thấy làm đích
// hạ cánh (gọi ở CMD_LAND). KHÔNG đụng floor_plane_z_m: hạ xuống một cái bàn
// không được ghi đè mốc sàn của cả chuyến bay (xem §32 — floor ≠ landing
// surface). Trả false nếu ToF không cho được ước lượng dùng được -> caller hạ
// theo world-Z như cũ.
bool alt_estimator_select_landing_surface(alt_estimator_t *e);

// alt_estimator_height_above_landing_surface() — chiều cao trên bề mặt hạ cánh
// đã chọn. CHỈ dùng cho pha LANDING (flare/touchdown). Trả về alt_m nếu chưa
// chọn được bề mặt nào.
float alt_estimator_height_above_landing_surface(const alt_estimator_t *e);

// alt_estimator_reanchor() — gọi NGAY SAU baro_driver_calibrate_ground()
// THÀNH CÔNG: mốc 0m của baro vừa đổi nên mọi state tính theo mốc cũ đều vô
// nghĩa. KHÁC reset(): GIỮ accel_bias_ms2 (đã học, không liên quan mốc baro)
// và giữ az_raw/az_lpf (filter accel không liên quan mốc baro).
void alt_estimator_reanchor(alt_estimator_t *e);

// alt_estimator_update() — một bước (gọi cùng nhịp Mahony, 250Hz).
//   acc_body_g        : accel body-frame (g), SAU calib 6-face, TRƯỚC LPF.
//   q                 : quaternion body->world (mahony_quaternion()).
//   baro_healthy      : baro còn sống VÀ mẫu chưa stale (caller kiểm).
//   baro_seq          : sensor_health_t.seq của baro — estimator tự so với
//                        seq đã xử lý để chỉ correction MỘT LẦN/mẫu. Caller
//                        KHÔNG cần tự lọc "mẫu mới" nữa.
//   baro_timestamp_us : thời điểm sensor_hub publish mẫu -> dt_baro THẬT.
//   baro_alt_m        : độ cao TƯƠNG ĐỐI RAW từ baro_driver_read() (m).
//   stationary        : đứng yên thật (caller đảm bảo => !inertial_enabled).
//   liftoff_candidate : cho phép inertial chạy ĐỂ TẠO bằng chứng liftoff,
//                        nhưng chưa tuyên bố airborne (xem "BA PHA").
//   airborne          : đã CONFIRMED rời đất -> inertial + baro fusion đầy đủ.
//   now_us, dt        : mốc thời gian + chu kỳ gọi (giây).
//   tof_healthy       : ToF còn sống VÀ mẫu chưa stale (caller kiểm).
//   tof_seq           : sensor_health_t.seq của ToF — estimator tự so với seq
//                        đã xử lý để chỉ correction MỘT LẦN/mẫu (ToF ~30Hz vs
//                        estimator 250Hz). Caller KHÔNG cần tự lọc "mẫu mới".
//   tof_timestamp_us  : thời điểm hub publish -> dt_tof THẬT.
//   tof_range_m       : range THÔ theo trục sensor (m).
//
// BÊN TRONG hàm tách rõ 3 giai đoạn (spec §3): predict_from_accel() ->
// correct_from_tof() -> correct_from_baro(). Giữ MỘT public API vì caller chỉ
// có một điểm gọi mỗi tick và thứ tự predict-trước-correct là bất biến không
// nên để caller tự quyết.
void alt_estimator_update(alt_estimator_t *e,
                           vec3f_t acc_body_g, quat_t q,
                           bool baro_healthy, uint32_t baro_seq,
                           int64_t baro_timestamp_us, float baro_alt_m,
                           bool tof_healthy, uint32_t tof_seq,
                           int64_t tof_timestamp_us, float tof_range_m,
                           bool stationary, bool liftoff_candidate, bool airborne,
                           int64_t now_us, float dt);

// alt_estimator_correct_velocity() — CHỪA INTERFACE cho nguồn correction VẬN
// TỐC tương lai (optical flow). No-op có chủ đích ở iteration này.
void alt_estimator_correct_velocity(alt_estimator_t *e, float vz_measured_ms, bool measurement_valid, float dt);

#ifdef __cplusplus
}
#endif
