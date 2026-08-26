// Takeoff + Landing sequences. PORT trực tiếp từ run_takeoff_sequence() và
// run_landing_sequence() (UAV-Mini flight_control.cpp) — ĐÂY LÀ LOGIC ĐÃ BAY
// THẬT, ĐÃ SỬA 2 BUG NGHIÊM TRỌNG (I-term windup gây lật lúc rời đất; handoff
// theo thời gian thay vì theo độ cao thực). Giữ nguyên mọi mốc quyết định.
//
// Cả takeoff_run() và landing_run() dùng CHUNG một alt_hold_state_t (vz_integral)
// với alt_hold_run() (HOLD) — bumpless khi chuyển giữa các pha: KHÔNG được cấp
// alt_hold_state_t khác nhau cho từng pha, nếu không sẽ giật ga lúc chuyển mode.
#pragma once

#include "flight_core/alt_hold.h"
#include "flight_core/tuning.h"
#include "flight_core/types.h"

#ifdef __cplusplus
extern "C" {
#endif

// ================= TAKEOFF =================
//
// ============================================================================
// KIẾN TRÚC: PID + SLEW-RATE-LIMITED TARGET — ĐỌC TRƯỚC KHI SỬA
// ============================================================================
// Toàn bộ chuỗi cất cánh chỉ là MỘT vòng điều khiển duy nhất chạy suốt:
//
//     target_z  TRƯỢT dần  ->  Z-PID  ->  vz_target  ->  Vz-PID  ->  throttle
//                                                        (hover_ff + P + I)
//
// Không có "ram ga để nhấc", không có "chốt hover lúc bàn giao", không có bước
// đổi nguồn throttle. Ba tính chất quan trọng, mất cái nào cũng hỏng:
//
//   1. target_z KHỞI ĐẦU = ĐỘ CAO HIỆN TẠI (không phải 0, không phải đích).
//      Đây là thứ chống windup. Đặt target = đích ngay khi còn nằm đất tạo
//      error khổng lồ -> I windup -> drone PHÓNG lên khi vừa đủ lực.
//
//   2. PRIME KHÔNG chạy Z/Vz PID. Motor phải quay đều trước, nhưng nếu cho PID
//      chạy trong lúc đó thì I đã tích luỹ xong trước khi drone kịp nhấc —
//      mất sạch tác dụng của điểm 1.
//
//   3. hover_ff (= ALT_HOLD_HOVER_NOMINAL) chỉ là feedforward THÔ. KHÔNG cố đo
//      hay chốt hover chính xác ở bất kỳ đâu — thành phần I của vòng Vz TỰ HỌC
//      hover thật dọc đường, và nó thích ứng liên tục theo pin/tải.
//
// CÁC PHA
// ============================================================================
//   TKO_IDLE    chuỗi chưa chạy.
//
//   TKO_PRIME   throttle = prime_duty (ga sàn cho motor brushed quay đều, KHÔNG
//               đủ nhấc). Z/Vz PID KHÔNG chạy. Altitude dynamics còn TẮT ở tầng
//               ngoài (ground lock) — trong pha này motor rung mạnh nhất, tích
//               phân lúc đó chỉ nạp nhiễu vào Z/Vz trước khi có gì thật để đo.
//               Cạnh PRIME -> CLIMB CÓ một bước nhảy ga (prime_duty -> ~hover_ff)
//               và đó là CHỦ ĐÍCH — xem ghi chú "I KHỞI ĐẦU = 0" trong .c.
//
//   TKO_CLIMB   target_z trượt từ độ cao hiện tại lên final_target với tốc độ
//               chặn ở max_climb_ms. Cascade Z->Vz->throttle chạy. liftoff_flag
//               bật khi est_z vượt ground + delta — CHỈ LÀ THÔNG TIN, không đổi
//               luồng, không gate gì.
//
//   TKO_HOLD    đúng MỘT vòng: bàn giao sang HOLDING. Vào khi target_z đã tới
//               đích VÀ est_z trong dung sai, duy trì đủ lâu.
//
//   TKO_ABORT   đúng MỘT vòng: caller đưa FSM về EMERGENCY (mọi lý do, xem
//               takeoff_abort_reason_t).
typedef enum {
    TKO_IDLE = 0,
    TKO_PRIME,
    TKO_CLIMB,
    TKO_HOLD,
    TKO_ABORT,
} takeoff_phase_t;

// Lý do abort — telemetry + log. MỌI lý do đều dẫn về EMERGENCY ở tầng caller;
// EMERGENCY tự phân giải thành LANDING (đang trên không, còn kiểm soát) hoặc
// DISARMED + kill latch (còn ở mặt đất / mất kiểm soát) — xem
// fsm_on_emergency_resolve(). Nhờ vậy bộ này KHÔNG phải tự chọn policy.
typedef enum {
    TKO_ABORT_NONE = 0,
    TKO_ABORT_TIMEOUT,     // không nhấc nổi trong NO_LIFT_TIMEOUT, hoặc quá TOTAL_TIMEOUT
    // Mất HẾT nguồn correction độ cao (ToF VÀ baro) > TAKEOFF_TOF_LOST_MS.
    // Giữ TÊN + GIÁ TRỊ cũ để GUI/log không vỡ (TKOAB= trong STATUS), dù giờ nó
    // không còn riêng ToF nữa.
    TKO_ABORT_TOF_LOST,
    TKO_ABORT_TILT,        // |tilt| > TAKEOFF_ABORT_TILT_DEG duy trì
    // takeoff_run() bị gọi khi chuỗi CHƯA active (takeoff_begin() chưa chạy).
    // Đây là LỖI LUỒNG ở tầng trên, KHÔNG phải sự cố bay.
    //
    // Trước đây nhánh này mượn tạm TKO_ABORT_TIMEOUT, nên GUI/log báo "TIMEOUT"
    // cho một tình huống hoàn toàn khác — người đọc đi tìm pin yếu/kẹt cánh/quá
    // tải (những thứ TIMEOUT thật sự nghĩa là) trong khi lỗi nằm ở thứ tự gọi.
    // Một mã riêng khiến hai thứ không thể bị nhầm nữa.
    TKO_ABORT_NOT_ACTIVE,
    // KHÔNG NHẤC NỔI — collective nằm KỊCH TRẦN liên tục TAKEOFF_STUCK_MS.
    //
    // Đây là bằng chứng thay thế cho TAKEOFF_NO_LIFT_TIMEOUT_MS đã bỏ, và nó
    // KHÔNG dùng độ cao đo được — điều bắt buộc ở cấu hình baro-only, nơi
    // propwash làm est_z sai 10-30cm nên "est_z chưa vượt ngưỡng" không kết
    // luận được gì.
    //
    // Cơ sở vật lý: nếu drone THẬT SỰ đang bay, vòng Vz tìm được điểm cân bằng
    // quanh hover — thấp hơn trần một khoảng rõ rệt. Nếu nó bị kẹt cánh, quá
    // tải, hay pin không còn kéo nổi, controller đòi thêm lực mãi mà vz không
    // đáp lại, nên I bò tới trần rồi NẰM LÌ. Trạng thái "đòi hết sức mà không
    // có phản hồi" đó đúng nghĩa là "không nhấc nổi", bất kể altimeter nói gì.
    TKO_ABORT_STUCK,
    // KHÔNG GOM ĐỦ BẰNG CHỨNG RỜI ĐẤT trong TAKEOFF_NO_LIFT_TIMEOUT_MS kể từ
    // lúc vào CLIMB. Đây là nhánh `!liftoff_flag && elapsed >= NO_LIFT_TIMEOUT`
    // trong takeoff_run().
    //
    // Trước đây nhánh này dùng CHUNG TKO_ABORT_TIMEOUT với hạn chót toàn chuỗi
    // (deadline_us), và đó là một lỗi chẩn đoán THẬT — hai tình huống khác hẳn
    // nhau bị in ra cùng một câu:
    //   - deadline_us  = "chuỗi bị kẹt, pha không tiến" (tick không chạy, logic
    //                    treo). Rất hiếm, và nếu xảy ra thì là bug firmware.
    //   - NO_LIFT      = chuỗi chạy HOÀN TOÀN BÌNH THƯỜNG, chỉ là detector rời
    //                    đất không đủ bằng chứng. Gần như LUÔN LUÔN là ToF
    //                    không fuse (tof_z_m đứng im ở 0) hoặc drone không nhấc
    //                    nổi thật.
    // Người đọc log đi tìm "chuỗi bị kẹt" trong khi vấn đề nằm ở cảm biến độ
    // cao. Một mã riêng làm hai thứ không thể bị nhầm nữa.
    //
    // ⚠ NỐI Ở CUỐI enum, KHÔNG chèn vào giữa: TKOAB= đi thẳng ra wire STATUS,
    // chèn giữa sẽ đổi ý nghĩa mọi mã cũ trong log/GUI đã lưu.
    TKO_ABORT_NO_LIFT_EVIDENCE,
} takeoff_abort_reason_t;

typedef struct {
    takeoff_phase_t phase;
    bool     active;            // chuỗi đang chạy (mọi pha != IDLE)
    int64_t  since_us;          // mốc bắt đầu TOÀN chuỗi
    int64_t  phase_since_us;    // mốc vào pha hiện tại

    // ---- Target ----
    float    ground_alt_m;      // est_z lúc rời PRIME (mốc đo liftoff)
    float    final_target_m;    // đích TỪ LỆNH CMD_TAKEOFF (đã clamp geofence)
    float    target_z_m;        // target ĐANG TRƯỢT — thứ Z-PID thật sự bám
    float    vz_target_ms;      // output tầng Z (telemetry)

    // liftoff_flag — "đã rời đất", QUYẾT ĐỊNH THEO THỜI GIAN, không theo độ cao
    // đo được: true sau TAKEOFF_LIFTOFF_MS kể từ khi vào CLIMB.
    // Lý do đầy đủ (propwash làm est_z sai 10-30cm ở cấu hình baro-only) nằm ở
    // tuning.h mục 3c. Nó KHÔNG phải chỉ telemetry — gate 3 chỗ: trần |I| vòng
    // Vz, hold_integral_freeze (Ki attitude) bên flight_core.c, và
    // takeoff_airborne() -> pha estimator + policy EMERGENCY.
    bool     liftoff_flag;
    bool     liftoff_active;
    int64_t  liftoff_since_us;

    // ---- Cửa sổ duy trì cho abort + vào HOLD. Dùng cờ + mốc RIÊNG, KHÔNG
    // dùng 0 làm sentinel (now_us=0 là mốc hợp lệ).
    bool     tof_lost_active;
    int64_t  tof_lost_since_us;
    bool     tilt_bad_active;
    int64_t  tilt_bad_since_us;
    // Cửa sổ "target_z đã tới đích và GIỮ như vậy" -> bàn giao. Không còn liên
    // quan tới est_z (xem tuning.h 3d).
    bool     at_target_active;
    int64_t  at_target_since_us;
    // Cửa sổ "collective kịch trần" -> TKO_ABORT_STUCK (không nhấc nổi).
    bool     stuck_active;
    int64_t  stuck_since_us;

    // Hạn CHÓT của toàn chuỗi, tính TẠI takeoff_begin() theo target thật.
    // KHÔNG phải hằng số: tốc độ leo có thể rất chậm (TAKEOFF_MAX_CLIMB_MS),
    // nên một hạn cứng dùng chung cho mọi target sẽ đúng với target thấp và
    // SAI HẲN với target cao. Xem takeoff_begin().
    int64_t  deadline_us;

    int      last_throttle;     // giữ ga khi mất ToF trong cửa sổ chờ

    takeoff_abort_reason_t abort_reason;
} takeoff_state_t;

typedef struct {
    int   prime_duty;        // ga PRIME — PHẢI thấp hơn hover (xem tuning.h 3a)
    int   prime_ms;
    float max_climb_ms;      // tốc độ trượt target + trần |vz_target| (m/s)
    int   liftoff_ms;        // sau bấy nhiêu ms trong CLIMB thì coi là đã rời đất
} takeoff_tune_t;

// Mặc định: xem flight_core/tuning.h (mục 3 — TAKEOFF).
takeoff_tune_t takeoff_default_tune(void);

static inline void takeoff_reset(takeoff_state_t *st) {
    takeoff_state_t z = {0};
    *st = z;
    st->phase = TKO_IDLE;
}

// takeoff_begin() — gọi TỪ CMD_TAKEOFF sau khi mọi precondition đã pass.
// target_alt_m PHẢI là giá trị đã clamp geofence ở tầng gọi.
//
// VÌ SAO TÁCH KHỎI takeoff_run(): target đến từ LỆNH, còn takeoff_run() chạy
// mỗi tick và không được biết gì về lệnh. Nhồi target vào takeoff_run() sẽ buộc
// nó phải phân biệt "tick đầu" bằng heuristic — đúng loại state ngầm gây lỗi.
static inline void takeoff_begin(takeoff_state_t *st, float target_alt_m, int64_t now_us) {
    takeoff_reset(st);
    st->active = true;
    st->phase = TKO_PRIME;
    st->since_us = now_us;
    st->phase_since_us = now_us;
    st->final_target_m = target_alt_m;

    // ---- Hạn chót TỰ CO GIÃN theo target, KHÔNG dùng hằng số cứng ----
    // Chuỗi giờ hoàn toàn xác định theo thời gian, nên thời lượng KỲ VỌNG tính
    // được chính xác:
    //     PRIME_MS + (target / max_climb)*1000 + HOLD_ENTER_MS
    //
    // VÌ SAO KHÔNG để một TAKEOFF_TOTAL_TIMEOUT_MS cứng: nó ghép chặt với
    // max_climb, và ghép sai thì hỏng ÂM THẦM. Ví dụ thật của cấu hình hiện tại
    // (max_climb = 0.1 m/s, timeout cũ 15s):
    //     target 0.3m ->  4.3s  -> lọt
    //     target 1.0m -> 11.3s  -> lọt, còn 3.7s biên
    //     target 3.0m -> 31.3s  -> ABORT MỌI LẦN, dù chẳng có gì sai
    // Người dùng hạ max_climb cho êm hơn, và vô tình làm mọi chuyến bay cao trở
    // nên bất khả thi — với lý do abort ghi là "TIMEOUT", tức là đổ lỗi cho
    // phần cứng. Tính theo target thì cái bẫy đó biến mất.
    //
    // Hệ số 2.0 + 3s: chuỗi chạy chậm hơn kỳ vọng là BÌNH THƯỜNG (hover_ff
    // lệch, pin yếu, gió) — hạn này chỉ để bắt "kẹt hẳn", không phải để ép đúng
    // lịch. Sau khi target_z tới đích thì chỉ còn chờ HOLD_ENTER_MS nên phần
    // biên rộng này không làm chậm chuyến bay bình thường.
    // Dùng thẳng macro (không phải tune->) vì takeoff_begin() cố tình KHÔNG
    // nhận takeoff_tune_t — nó chỉ biết về LỆNH. Hai giá trị luôn khớp nhau vì
    // takeoff_default_tune() cũng lấy từ đúng macro này.
    const float slew_s = (TAKEOFF_MAX_CLIMB_MS > 0.0f)
                          ? (target_alt_m / (float)TAKEOFF_MAX_CLIMB_MS) : 0.0f;
    const int64_t expect_ms = (int64_t)(TAKEOFF_PRIME_MS
                                         + slew_s * 1000.0f
                                         + TAKEOFF_HOLD_ENTER_MS);
    // NỚI BIÊN 2x+3s -> 3x+8s. Đo trên phần cứng thật: target 0.20m cho
    // expect_ms = 700 + 667 + 400 = 1767ms, tức deadline cũ chỉ 6.5s — trong
    // khi drone thật cần tới ~7.7s mới ổn định quanh đích (I của vòng Vz phải
    // học xong phần dư hover_thật - hover_ff, và ground effect làm nó vọt lên
    // rồi mới lắng). Chuyến bay HOÀN TOÀN BÌNH THƯỜNG bị hủy vì hết giờ.
    //
    // Biên rộng KHÔNG làm chậm chuyến bay bình thường: đường bàn giao bình
    // thường thoát ngay khi hold_ready đủ TAKEOFF_HOLD_ENTER_MS, deadline chỉ
    // là lưới cuối bắt "kẹt hẳn". Và nó KHÔNG phải cơ chế an toàn duy nhất —
    // guard tilt, mất nguồn đo, STUCK, và toàn bộ Commander vẫn chạy song song
    // với chu kỳ ngắn hơn nhiều.
    st->deadline_us = now_us + (expect_ms * 3 + 8000) * 1000;
}

// ---- Hai truy vấn DUY NHẤT mà tầng ngoài được dùng để gate estimator ----
// Đặt ở header dạng inline để KHÔNG ai phải tự suy luận lại từ `phase` (mỗi nơi
// tự so sánh enum là cách chắc chắn nhất để hai module lệch định nghĩa nhau).
//
// takeoff_control_active(): altitude dynamics PHẢI chạy (estimator tích phân +
// Z/Vz controller). True từ CLIMB trở đi — KHÔNG bao gồm PRIME (xem điểm 2 đầu
// file).
static inline bool takeoff_control_active(const takeoff_state_t *st) {
    return st->active && (st->phase == TKO_CLIMB || st->phase == TKO_HOLD);
}

// takeoff_airborne(): drone đã rời đất. ĐỊNH NGHĨA DUY NHẤT trong pha cất cánh.
static inline bool takeoff_airborne(const takeoff_state_t *st) {
    return st->active && st->liftoff_flag;
}

typedef struct {
    int  throttle_duty;      // collective CUỐI của chuỗi (trước mixer)

    bool control_active;     // == takeoff_control_active()
    bool liftoff_flag;       // đã rời đất
    bool liftoff_edge;       // true dung tick xac nhan bang ToF

    takeoff_phase_t phase;

    // handoff: ĐÚNG vòng TKO_HOLD. Caller phải:
    //   1) s_alt_target_m = out->final_target_m   (KHÔNG phải Z hiện tại!)
    //   2) FSM TAKING_OFF -> HOLDING
    // alt_hold_state_t đã engaged + vz_integral đã học hover nên handoff
    // BUMPLESS — caller KHÔNG được preload/reset gì thêm.
    bool handoff;

    // abort: ĐÚNG vòng TKO_ABORT. Caller đưa FSM về EMERGENCY, mọi lý do.
    bool abort;
    takeoff_abort_reason_t abort_reason;

    // ---- Telemetry / debug (spec: đủ để soi slew + học-hover) ----
    float final_target_m;
    float target_z_m;        // target ĐANG TRƯỢT
    float ground_alt_m;
    float vz_target_ms;      // output tầng Z -> đầu vào Vz-PID
    float hover_ff;          // feedforward thô đang dùng
    float vz_i_term;         // I của vòng Vz — xem nó CÓ đang học hover không
    float alt_thrust_corr;   // throttle - hover_ff
    float tilt_deg;
    float elapsed_s;
} takeoff_result_t;

// takeoff_run() — một bước. Gọi mỗi vòng khi FSM ở TAKING_OFF.
//
//   hold_st/hold_tune : CHUNG với HOLD/LANDING — vz_integral (phần đã học
//                        hover) đi liền mạch từ takeoff sang HOLDING. Đó là
//                        điều làm bàn giao bumpless. KHÔNG được cấp
//                        alt_hold_state_t riêng cho từng pha.
//   alt_valid/alt_m/vz_ms : từ alt_estimator (est.alt_m/est.vz_ms — ToF đã bù
//                        cos_tilt bên trong estimator, KHÔNG BAO GIỜ range thô).
//   alt_source_ok     : CÒN ÍT NHẤT MỘT nguồn correction độ cao (ToF HOẶC
//                        baro). Mất HẾT > TAKEOFF_TOF_LOST_MS -> abort. Trong
//                        cửa sổ chờ thì GIỮ ga, không chạy cascade trên est_z rác.
//
//                        ⚠ TÊN CŨ LÀ `tof_ok` VÀ ĐÓ LÀ MỘT LỖI THẬT, không chỉ
//                        là tên xấu. Caller truyền vào sức khoẻ RIÊNG của ToF,
//                        nên điều kiện abort thành "KHÔNG có ToF => huỷ cất
//                        cánh". Hai hệ quả, cả hai đều chặn bay:
//                          1. Cấu hình baro-only (ToF chưa init được / tắt
//                             trong app_config) abort sau đúng 300ms mỗi lần
//                             cất cánh — dù CMD_TAKEOFF đã cho qua và mọi thứ
//                             khác báo sẵn sàng.
//                          2. VL53L0X chỉ với tới ~1.8m. Leo cao hơn là ToF tự
//                             hết tầm -> abort GIỮA CHỪNG dù cảm biến hoàn toàn
//                             lành lặn.
//                        Kiến trúc luôn nói "cần ÍT NHẤT MỘT nguồn (baro HOẶC
//                        ToF)" — xem alt_estimator.h NO_CORRECTION. Điều kiện ở
//                        đây giờ khớp đúng câu đó.
//   tilt_deg          : max(|roll|,|pitch|) — guard lật.
void takeoff_run(takeoff_state_t *st, const takeoff_tune_t *tune,
                  alt_hold_state_t *hold_st, const alt_hold_tune_t *hold_tune,
                  bool alt_valid, float alt_m, float vz_ms,
                  bool tof_fusable, float tof_z_m, float tof_vz_ms,
                  float tilt_deg,
                  float dt, int64_t now_us, int safe_max_duty,
                  takeoff_result_t *out);

// ================= LANDING =================

typedef enum {
    LAND_IDLE = 0,
    LAND_DESCEND,
    LAND_FLARE,
    LAND_TOUCHDOWN,
    LAND_BLIND,             // mất Z-est quá lâu -> hạ mù, ga giảm cố định theo thời gian
    LAND_CONTACT_CANDIDATE, // NGHI đã chạm — đang xác nhận, CHƯA cắt máy
} land_phase_t;

typedef struct {
    land_phase_t phase;
    int64_t settle_us;      // mốc bắt đầu "ga thấp + vz~0" (backup touchdown detect)
    int64_t az_spike_us;    // mốc bắt đầu az_earth spike (nhánh touchdown thứ 3)
    uint32_t terr_commit_seen;  // terr_commit_count lần cuối đã xử lý (reset pha)
    bool    terr_seen_init;
    int64_t toflost_us;     // mốc bắt đầu mất Z-est
    int64_t cutoff_us;      // mốc bắt đầu ramp ga về 0 (TOUCHDOWN)
    float   cutoff_thr;     // ga tại thời điểm bắt đầu cutoff
    float   blind_thr;      // ga hiện tại trong pha BLIND
    int     last_throttle;

    // ---- TOUCHDOWN detector đa điều kiện (xem landing_run()) ----
    int     contact_ticks;    // số tick LIÊN TỤC đủ điểm chạm đất
    int     last_contact_score;
    float   alt_at_candidate; // Z lúc vào CONTACT_CANDIDATE — để xem Z còn giảm nữa không
} landing_state_t;

typedef struct {
    float descent_vz;       // m/s, tốc độ hạ pha DESCEND
    float flare_alt_m;      // m, ngưỡng vào FLARE
    float flare_vz;         // m/s, tốc độ hạ lúc gần chạm (chậm hơn descent)
    float touchdown_alt_m;  // m, chạm đất -> cutoff
} landing_tune_t;

// Mặc định: xem flight_core/tuning.h (mục 4 — LANDING). Đổi số ở ĐÓ, không ở
// đây.
landing_tune_t landing_default_tune(void);

static inline void landing_reset(landing_state_t *st) {
    landing_state_t z = {0};
    *st = z;
    st->phase = LAND_IDLE;
}

typedef struct {
    int  throttle_duty;
    bool hold_driving;
    bool touchdown_done;   // true ĐÚNG vòng cutoff hoàn tất -> caller disarm + FSM -> DISARMED
} landing_result_t;

// landing_run() — một bước. Gọi mỗi vòng khi FSM ở LANDING.
//
// ============================================================================
// LANDING CHẠY TRÊN AGL (độ cao trên BỀ MẶT ĐANG Ở DƯỚI). KHÔNG PHẢI DATUM.
// ============================================================================
// Đây là bất biến quan trọng nhất của cả khối landing, và nó KHÔNG được suy
// lại khi refactor: đang bay trên một cái bàn cao 0.75m mà bấm land thì phải
// hạ xuống MẶT BÀN. Cố hạ về cao độ SÀN (alt_m = 0) là đâm thẳng vào bàn.
//   alt_m    : PHẢI là alt_estimator_height_above_landing_surface() (= AGL).
//   tof_z_m  : PHẢI là alt_estimator_tof_agl_m() (AGL thô từ ToF).
// Caller nào truyền datum vào đây là một lỗi giết drone, không phải lỗi style.
//
//   hold_st/hold_tune : CHUNG với HOLD/TAKEOFF (bumpless, warm-start vz_integral).
//   was_hold_engaged  : alt_hold_state_t.engaged TRƯỚC khi vào landing_run() —
//                       nếu true, giữ vz_integral ấm; nếu false (vào từ manual
//                       chưa từng engage), preload từ manual_throttle_duty.
//   terrain_pending   : estimator ĐANG nghi có bậc địa hình, chưa xác nhận.
//                       Chặn vào DESCEND (C1) và giữ nguyên pha đang chạy —
//                       vào land giữa lúc ở mép bàn sẽ flare sai điểm.
//   terrain_commits   : alt_estimator_t.terr_commit_count. Số này ĐỔI giữa lúc
//                       đang hạ = vừa có bậc địa hình (trôi ngang khỏi mép
//                       bàn) -> RESET PHA về DESCEND, tính lại flare theo AGL
//                       mới. TUYỆT ĐỐI không nhảy thẳng TOUCHDOWN/cắt ga (C2).
//   az_earth_ms2      : az trục Z-lên ĐÃ TRỪ trọng lực (alt_estimator_t.
//                       az_after_bias_ms2) — nhánh touchdown thứ 3.
void landing_run(landing_state_t *st, const landing_tune_t *tune,
                  alt_hold_state_t *hold_st, const alt_hold_tune_t *hold_tune,
                  bool was_hold_engaged,
                  bool alt_valid, float alt_m, float vz_ms, int manual_throttle_duty,
                  bool tof_fusable, float tof_z_m, float tof_vz_ms,
                  bool terrain_pending, uint32_t terrain_commits, float az_earth_ms2,
                  float dt, int64_t now_us, int safe_max_duty,
                  landing_result_t *out);

#ifdef __cplusplus
}
#endif
