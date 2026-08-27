#include "flight_core/takeoff_land.h"

#include "flight_core/alt_estimator.h"

#include <math.h>

// ============================================================================
// RÀNG BUỘC GIỮA BA NGƯỠNG SÁT ĐẤT — bắt lúc BIÊN DỊCH
// ============================================================================
// Ba hằng số này nằm ở HAI file khác nhau (tuning.h và alt_estimator.h) nhưng
// mắc nối tiếp nhau trên cùng một đường dữ liệu:
//
//   range thô --[>= ALT_EST_TOF_MIN_RANGE_M]--> tof_vertical
//             --[< ALT_EST_GROUND_ZERO_BAND_M thì ép về 0]--> tof_z_m
//             --[>= TAKEOFF_LIFTOFF_Z_M]--> bằng chứng rời đất
//
// Đặt lệch nhau thì KHÔNG có lỗi nào cả — chỉ là takeoff abort
// NO_LIFT_EVIDENCE mãi mãi, và người debug đi tìm pin/cánh quạt/hover_ff trong
// khi nguyên nhân là hai con số ở hai file cách nhau vài trăm dòng. Đã xảy ra
// thật (0.015 vs 0.01). Ràng ở đây để lần sau là lỗi BUILD.
_Static_assert(ALT_EST_GROUND_ZERO_BAND_M <= TAKEOFF_LIFTOFF_Z_M,
                "ALT_EST_GROUND_ZERO_BAND_M > TAKEOFF_LIFTOFF_Z_M: tof_z_m bi ep ve 0 TRUOC khi "
                "cham nguong roi dat -> lift_evidence KHONG BAO GIO dung -> takeoff abort mai mai");
_Static_assert(ALT_EST_TOF_MIN_RANGE_M <= TAKEOFF_LIFTOFF_Z_M,
                "ALT_EST_TOF_MIN_RANGE_M > TAKEOFF_LIFTOFF_Z_M: gate hinh hoc loai mau TRUOC khi "
                "tof_z_m kip dat nguong roi dat -> lift_evidence KHONG BAO GIO dung");

// ================= TAKEOFF =================

takeoff_tune_t takeoff_default_tune(void) {
    takeoff_tune_t t;
    t.prime_duty = TAKEOFF_PRIME_DUTY;
    t.prime_ms = TAKEOFF_PRIME_MS;
    t.max_climb_ms = TAKEOFF_MAX_CLIMB_MS;
    t.liftoff_ms = TAKEOFF_LIFTOFF_MS;
    return t;
}

static void tko_enter_phase(takeoff_state_t *st, takeoff_phase_t p, int64_t now_us) {
    st->phase = p;
    st->phase_since_us = now_us;
}

// tko_latch_window() — bộ đếm "điều kiện đúng LIÊN TỤC bao lâu", dùng cờ + mốc
// riêng thay vì 0-sentinel (now_us=0 là mốc hợp lệ). Trả true khi đã đủ hold_ms.
static bool tko_latch_window(bool cond, bool *active, int64_t *since_us,
                              int64_t now_us, int hold_ms) {
    if (!cond) {
        *active = false;
        return false;
    }
    if (!*active) {
        *active = true;
        *since_us = now_us;
    }
    return (now_us - *since_us) >= (int64_t)hold_ms * 1000;
}

// tko_abort() — điền out cho ĐÚNG vòng abort. Gom vào một chỗ để mọi nhánh
// abort thoát ra giống hệt nhau (throttle=0, active=false, phase=ABORT).
// tko_handoff() — BÀN GIAO LIỀN MẠCH sang HOLDING. Gom vào một chỗ vì có HAI
// đường tới đây (vào dung sai đủ lâu, và hết TOTAL_TIMEOUT khi target đã tới
// đích) và chúng PHẢI thoát ra giống hệt nhau — lệch một field là bàn giao
// xóc hoặc target sai.
//
// KHÔNG preload, KHÔNG reset, KHÔNG đổi throttle. hold_st đang engaged với
// vz_integral = hover thật đã học. alt_hold_run() ở tick sau tiếp tục từ CHÍNH
// state đó, trên CÙNG cascade, với target đứng yên ở final_target -> ga không
// có bước nhảy nào.
static void tko_handoff(takeoff_state_t *st, int64_t now_us, takeoff_result_t *out) {
    tko_enter_phase(st, TKO_HOLD, now_us);
    st->active = false;
    out->phase = TKO_HOLD;
    out->handoff = true;
    // target giữ nguyên = target CỦA LỆNH, KHÔNG phải Z nhiễu hiện tại.
    out->final_target_m = st->final_target_m;
    // GIỮ NGUYÊN ga của tick trước. BẮT BUỘC, không phải cho đẹp:
    // takeoff_run() mở đầu bằng *out = {0}, nên nhánh bàn giao gọi từ ĐẦU hàm
    // (check C — hết TOTAL_TIMEOUT) sẽ trả throttle_duty = 0 nếu không gán lại
    // -> motor bị cắt trọn một tick giữa lúc đang bay, rồi alt_hold nhận lại ở
    // tick sau. Đo được trong test C4: bước nhảy 1449 duty.
    // st->last_throttle == collective mà cascade vừa xuất, nên nhánh bàn giao
    // "bình thường" (gọi ở CUỐI hàm) gán lại đúng cùng giá trị -> hai đường ra
    // giống hệt nhau, đúng như docstring hứa.
    out->throttle_duty = st->last_throttle;
}

static void tko_abort(takeoff_state_t *st, takeoff_abort_reason_t why,
                       int64_t now_us, takeoff_result_t *out) {
    st->abort_reason = why;
    tko_enter_phase(st, TKO_ABORT, now_us);
    st->active = false;
    out->phase = TKO_ABORT;
    out->abort = true;
    out->abort_reason = why;
    out->liftoff_flag = st->liftoff_flag;   // caller/EMERGENCY dùng để biết còn ở đất hay không
    out->throttle_duty = 0;
}

void takeoff_run(takeoff_state_t *st, const takeoff_tune_t *tune,
                  alt_hold_state_t *hold_st, const alt_hold_tune_t *hold_tune,
                  bool alt_valid, float alt_m, float vz_ms,
                  bool tof_fusable, float tof_z_m, float tof_vz_ms,
                  float tilt_deg,
                  float dt, int64_t now_us, int safe_max_duty,
                  takeoff_result_t *out) {
    *out = (takeoff_result_t){0};

    // takeoff_begin() PHẢI đã chạy (CMD_TAKEOFF). Caller gọi khi chuỗi chưa
    // active là lỗi luồng ở tầng trên — báo abort thay vì tự đoán một target
    // rồi đẩy ga: đẩy ga với target bịa là cách tệ nhất để sai.
    if (!st->active) {
        out->phase = TKO_ABORT;
        out->abort = true;
        // KHÔNG mượn TKO_ABORT_TIMEOUT nữa: đây là lỗi luồng (caller gọi khi
        // chuỗi chưa bắt đầu), không phải "không nhấc nổi trong thời gian cho
        // phép". Báo nhầm mã khiến người debug đi tìm pin/tải trong khi vấn đề
        // nằm ở thứ tự gọi — xem takeoff_land.h.
        out->abort_reason = TKO_ABORT_NOT_ACTIVE;
        return;
    }

    out->final_target_m = st->final_target_m;
    out->ground_alt_m = st->ground_alt_m;
    out->hover_ff = hold_tune->hover;
    out->tilt_deg = tilt_deg;
    out->elapsed_s = (float)(now_us - st->since_us) * 1e-6f;

    // ================= ABORT: kiểm TRƯỚC mọi thứ khác =================
    // Mọi lý do đều dẫn về EMERGENCY ở caller; EMERGENCY tự phân giải thành
    // LANDING hay DISARMED tuỳ còn kiểm soát được không (xem takeoff_land.h).

    // (A) Nghiêng quá lớn = sắp lật. Có cửa sổ duy trì ngắn để một mẫu Mahony
    // lỗi không tự huỷ một chuyến bay đang bình thường.
    if (tko_latch_window(fabsf(tilt_deg) > TAKEOFF_ABORT_TILT_DEG,
                          &st->tilt_bad_active, &st->tilt_bad_since_us,
                          now_us, TAKEOFF_ABORT_TILT_MS)) {
        tko_abort(st, TKO_ABORT_TILT, now_us, out);
        return;
    }

    // (B) ĐÃ BỎ: "không nhấc nổi trong NO_LIFT_TIMEOUT".
    //
    // Nó dựa HOÀN TOÀN vào est_z (`est_z > ground + delta`) để kết luận drone
    // có rời đất hay không. Ở cấu hình baro-only, est_z sai 10-30cm vì propwash
    // (số đo đầy đủ ở alt_estimator.h), nên phép kết luận đó không còn cơ sở.
    // liftoff_flag giờ tính theo THỜI GIAN nên nhánh này cũng không bao giờ
    // đúng được nữa — giữ lại chỉ là code chết gây hiểu nhầm.
    //
    // ⚠ MẤT GÌ: đây từng là cơ chế DUY NHẤT tự bắt "kẹt cánh / quá tải / pin
    // yếu". Giờ không còn. Xem khối cảnh báo ở tuning.h mục 3c.

    // (C) Hạn chót TOÀN chuỗi — tính theo target thật tại takeoff_begin(),
    // KHÔNG phải hằng số cứng (xem takeoff_begin() để biết vì sao).
    //
    // Chuỗi giờ xác định theo thời gian: PRIME xong -> target trượt đủ
    // final_target/max_climb giây -> chờ HOLD_ENTER_MS -> bàn giao. Nếu tới hạn
    // này mà VẪN chưa bàn giao thì có thứ gì đó kẹt thật (pha không tiến, tick
    // không chạy) — đây không còn là chuyện "bay chậm hơn dự kiến" nữa vì hạn đã
    // gấp đôi thời lượng kỳ vọng cộng 3 giây.
    //
    // KHÔNG còn nhánh "bàn giao khi hết giờ": đường bàn giao bình thường giờ
    // cũng chỉ phụ thuộc thời gian, nên nếu nó chưa chạy thì lối thoát kia cũng
    // vô nghĩa. Một đường ra, một ý nghĩa.
    if (now_us >= st->deadline_us) {
        tko_abort(st, TKO_ABORT_TIMEOUT, now_us, out);
        return;
    }

    // (D) Mất nguồn đo độ cao. CHỈ tính từ CLIMB trở đi: trong PRIME altitude
    // dynamics còn tắt nên est_z chưa phải thứ ta đang dựa vào.
    //
    // alt_source_ok = CÒN ÍT NHẤT MỘT nguồn correction (ToF HOẶC baro) — KHÔNG
    // phải sức khoẻ riêng của ToF. Xem khối cảnh báo ở takeoff_land.h: bản
    // trước nhận thẳng tof_healthy vào đây, nên baro-only abort sau 300ms mỗi
    // lần cất cánh, và leo quá tầm ToF (~1.8m) cũng abort giữa chừng.
    bool alt_source_lost = false;
    if (st->phase == TKO_CLIMB || st->phase == TKO_HOLD) {
        alt_source_lost = !tof_fusable || !alt_valid;
        if (tko_latch_window(alt_source_lost, &st->tof_lost_active,
                              &st->tof_lost_since_us, now_us, TAKEOFF_TOF_LOST_MS)) {
            tko_abort(st, TKO_ABORT_TOF_LOST, now_us, out);
            return;
        }
    } else {
        st->tof_lost_active = false;
    }

    // ================= PHASE 1: PRIME =================
    // Ga sàn cố định cho motor brushed quay ĐỀU. KHÔNG chạy Z/Vz PID ở đây —
    // đó chính là điều kiện chống windup thứ hai (xem takeoff_land.h điểm 2).
    if (st->phase == TKO_PRIME) {
        const float prime_frac = clampf((float)(now_us - st->phase_since_us) /
                                        ((float)tune->prime_ms * 1000.0f), 0.0f, 1.0f);
        out->throttle_duty = clampi((int)((float)tune->prime_duty * prime_frac),
                                    0, safe_max_duty);
        out->phase = TKO_PRIME;
        out->target_z_m = 0.0f;
        out->vz_target_ms = 0.0f;
        out->alt_thrust_corr = 0.0f;
        out->vz_i_term = 0.0f;
        st->last_throttle = out->throttle_duty;

        // Vòng Vz chưa cầm lái: ép engaged=false + I=0 để không mang tích phân
        // của chuyến trước vào chuyến này. Preload ĐÚNG CHỖ là cạnh
        // PRIME -> CLIMB bên dưới, nơi ta biết ga thật đang là bao nhiêu.
        hold_st->engaged = false;
        hold_st->vz_integral = 0.0f;

        if ((now_us - st->phase_since_us) < (int64_t)tune->prime_ms * 1000) {
            return;
        }

        // ---- CẠNH PRIME -> CLIMB ----
        // Từ tick SAU, tầng ngoài thấy takeoff_control_active()==true và mở
        // altitude dynamics cho estimator (1 tick trễ, xem flight_core.c bước 2).

        // ĐIỂM QUAN TRỌNG NHẤT CỦA CẢ FILE:
        // target_z khởi đầu = ĐỘ CAO HIỆN TẠI, KHÔNG phải 0 và KHÔNG phải đích.
        // Nhờ vậy error của tầng Z ở tick đầu tiên ~ 0 -> P không sốc, I không
        // có gì để windup. Đặt = đích ở đây là quay lại đúng bản PID ngây thơ
        // làm drone phóng lên.
        st->ground_alt_m = alt_valid ? alt_m : 0.0f;
        st->target_z_m = st->ground_alt_m;

        // ---- I KHỞI ĐẦU = 0, KHÔNG preload từ ga PRIME ----
        // Đây là chỗ dễ sai nhất khi port từ kiến trúc cũ, và test đã bắt được:
        // alt_hold_preload(prime_duty) đặt I = prime_duty - hover_ff, tức
        // 780 - 1300 = -520 (bị kẹp về -300). Vòng Vz khi đó phải mất ~5.6s chỉ
        // để bò NGƯỢC lên tới hover thật — dài hơn cả TAKEOFF_NO_LIFT_TIMEOUT,
        // nên drone bị abort trước khi kịp nhấc.
        //
        // Preload là đúng ở BẢN CŨ vì hồi đó spool_duty == hover nên preload cho
        // I ~ 0. Ở đây prime_duty CỐ Ý thấp hơn hover nhiều, nên preload sẽ VỨT
        // BỎ chính cái feedforward hover_ff mà cả kiến trúc dựa vào.
        //
        // I = 0 nghĩa là ngay tick đầu của CLIMB: throttle = hover_ff + P, tức
        // controller nhận quyền TẠI ĐIỂM NEO hover. I chỉ còn phải học phần DƯ
        // (hover_thật - hover_ff), nhanh hơn nhiều bậc.
        //
        // CÓ một bước nhảy ga tại cạnh PRIME -> CLIMB (prime_duty -> ~hover_ff).
        // Đó là CHỦ ĐÍCH, không phải lỗi: prime theo định nghĩa là mức KHÔNG
        // nhấc nổi, còn climb phải đạt mức nhấc được. Ràng buộc "không giật ga"
        // áp cho cạnh CLIMB -> HOLDING (nơi cùng một controller chạy tiếp), chứ
        // không áp cho cạnh này.
        hold_st->engaged = true;
        hold_st->vz_integral = 0.0f;

        tko_enter_phase(st, TKO_CLIMB, now_us);
        out->phase = TKO_CLIMB;
        out->control_active = true;
        out->target_z_m = st->target_z_m;
        out->ground_alt_m = st->ground_alt_m;
        return;
    }

    // ================= PHASE 2: GUIDED CLIMB (PID + slew) =================
    out->control_active = true;

    // ---- Mất nguồn đo TRONG cửa sổ chờ: GIỮ ga, KHÔNG chạy cascade ----
    // est_z lúc này là rác; cho nó vào Z-PID sẽ sinh lệnh vz vô nghĩa và làm
    // bẩn luôn I (thứ đang học hover). Giữ ga cuối là hành vi đúng cho một
    // gián đoạn ngắn; kéo dài quá TOF_LOST_MS thì nhánh (D) ở trên đã abort.
    if (alt_source_lost) {
        out->throttle_duty = clampi(st->last_throttle, 0, safe_max_duty);
        out->phase = st->phase;
        out->target_z_m = st->target_z_m;
        out->vz_target_ms = 0.0f;
        out->vz_i_term = hold_st->vz_integral;
        out->alt_thrust_corr = (float)out->throttle_duty - hold_tune->hover;
        out->liftoff_flag = st->liftoff_flag;
        return;
    }

    // ---- SLEW: target TRƯỢT tới đích, tốc độ bị chặn ----
    // Rate limiter THUẦN (không phải hàm mũ): mỗi tick tiến tối đa
    // max_climb*dt. Khác biệt so với quỹ đạo mũ kp*(target - z_sp): rate
    // limiter CHẠM đích ĐÚNG BẰNG trong thời gian hữu hạn, còn hàm mũ chỉ tiệm
    // cận mãi mãi — bản trước phải thêm một dung sai riêng để thoát, và đặt
    // dung sai đó quá nhỏ từng làm một chuyến cất cánh HOÀN TOÀN ĐÚNG bị abort
    // vì timeout. Ở đây điều kiện "đã tới đích" là một phép so bằng thật.
    const float slew_step = tune->max_climb_ms * dt;
    const float remain = st->final_target_m - st->target_z_m;
    if (remain > slew_step)        st->target_z_m += slew_step;
    else if (remain < -slew_step)  st->target_z_m -= slew_step;
    else                            st->target_z_m = st->final_target_m;

    // ---- CASCADE: Z-PID (tầng ngoài) -> vz_target -> Vz-PID -> collective ----
    // Dùng est.alt_m/est.vz_ms (ToF đã bù cos_tilt trong estimator).
    const float alt_err = st->target_z_m - alt_m;
    const float vz_target = clampf(hold_tune->alt_kp * alt_err,
                                    -tune->max_climb_ms, tune->max_climb_ms);
    st->vz_target_ms = vz_target;

    // ---- Trần |I| TRƯỚC liftoff: GIỚI HẠN, KHÔNG ĐÓNG BĂNG ----
    // Slew làm error NHỎ, không làm nó bằng 0. Khi drone bị giữ lại thì target
    // vẫn trượt đều nên error dai dẳng một chiều và I sẽ bò lên tới khi TIMEOUT
    // nổ. Trần này chặn phần bò đó. KHÔNG freeze: freeze cắt luôn thẩm quyền
    // tìm điểm nhấc khi hover thật cao hơn ước lượng (lỗi đã đo bằng test).
    const float i_limit = st->liftoff_flag ? hold_tune->vz_ilimit
                                            : TAKEOFF_PRELIFT_I_LIMIT_DUTY;

    // Trần collective RIÊNG của takeoff, thấp hơn safe_max: chừa headroom cho
    // mixer tạo mô-men. Vượt qua đó là mất thẩm quyền attitude — nguy hiểm hơn
    // hẳn việc leo chậm.
    const int tko_ceiling = (int)((float)safe_max_duty * TAKEOFF_COLLECTIVE_CEILING_FRAC);

    const int collective = alt_hold_vz_cascade(hold_st, hold_tune, vz_target, vz_ms, dt,
                                                false, i_limit,
                                                ALT_HOLD_MIN_THROTTLE_DUTY, tko_ceiling);

    // ---- KHÔNG NHẤC NỔI ----
    // Thay cho TAKEOFF_NO_LIFT_TIMEOUT_MS đã bỏ. Hai bằng chứng, OR với nhau,
    // KHÔNG cái nào dùng ĐỘ CAO TUYỆT ĐỐI:
    //
    //  (a) KHÔNG BÁM ĐƯỢC TỐC ĐỘ LEO ĐÃ LỆNH — tín hiệu CHÍNH.
    //      Ta lệnh vz_target > 0 mà vz đo được gần như bằng 0 kéo dài = drone
    //      không đi lên, chấm hết.
    //
    //      ⚠ VÌ SAO vz VẪN DÙNG ĐƯỢC dù z thì không: propwash tạo ra một lệch
    //      VỊ TRÍ gần như HẰNG SỐ (baro đọc thấp hơn thật 10-30cm suốt thời
    //      gian ga cao). Đạo hàm của một hằng số bằng 0 — nên nó KHÔNG tạo lỗi
    //      vận tốc. Đây chính là lý do vz sống sót qua đúng cái nhiễu đã giết
    //      chết phép đo độ cao tuyệt đối.
    //
    //  (b) GA KỊCH TRẦN kéo dài — lưới an toàn thứ hai, chậm nhưng dứt khoát.
    //      Bắt trường hợp (a) bỏ sót: hover THẬT >= trần collective (quá tải,
    //      pin cạn) thì drone có thể nhích lên chút ít rồi đứng, vz không đủ
    //      nhỏ để (a) trip nhưng controller thì đã đòi hết sức nó có.
    //
    //      Một mình (b) là KHÔNG ĐỦ và đã đo được: I cần ~12s để bò hết dải
    //      500 duty, trong khi bàn giao (thuần thời gian) xảy ra ở ~5.8s — tức
    //      là drone bị chặn sẽ được bàn giao XONG trước khi (b) kịp phát hiện.
    //      (a) trip trong ~3s nên nó mới là cái thực sự bảo vệ.
    //
    // Chỉ tính TỪ SAU liftoff_ms: trước đó drone chưa được kỳ vọng phải đi lên.
    // ĐIỀU KIỆN: controller đã DÙNG HẾT thẩm quyền nó có (collective kịch trần)
    // và vẫn ở đó. Đây là thứ DUY NHẤT không thể sai:
    //
    //   - "vz vẫn bằng 0" KHÔNG dùng được, dù nghe rất hợp lý. Đã thử và ĐO:
    //     với gain hiện tại (ALT_HOLD_VZ_KI=100, TAKEOFF_MAX_CLIMB_MS=0.1) thì
    //     dI/dt = Ki*vz_err = 100*0.1 = 10 duty/s, nên nếu hover_ff lệch 150
    //     duty so với hover THẬT, một drone HOÀN TOÀN KHOẺ vẫn nằm im 15 GIÂY
    //     trong lúc I bò lên. Mọi ngưỡng thời gian ngắn hơn thế sẽ cắt oan;
    //     dài hơn thế thì vô dụng. Test C1 bắt đúng lỗi này khi tôi thử.
    //
    //   - "collective kịch trần" thì khác hẳn: trong lúc spool hợp lệ, I vẫn
    //     đang TĂNG và collective còn xa trần. Chỉ khi drone thật sự không đáp
    //     lại thì I mới bò tới giới hạn rồi NẰM LÌ. Không có dương tính giả nào
    //     phụ thuộc vào tốc độ spool.
    //
    // ---- ABORT "STUCK" (ga kịch trần liên tục): ĐÃ BỎ (yêu cầu người dùng) ----
    // Khối cũ hủy chuyến khi collective chạm trần thẩm quyền vòng Vz (I chạm
    // i_limit, hoặc collective chạm trần mixer) liên tục TAKEOFF_STUCK_MS.
    // Toàn bộ i_limit_now / i_exhausted / at_ceiling đã xoá theo — không còn ai
    // đọc chúng, giữ lại chỉ sinh warning.
    //
    // ⚠ CÁI MẤT: không còn phát hiện "pin yếu / quá tải / cánh sai" bằng dấu
    // hiệu ga kịch trần mà không lên. Cái CÒN: TKO_ABORT_TIMEOUT, "motor
    // saturated too long" của Commander, và sàn pin. Ba cái đó vẫn chặn được
    // trường hợp nguy hiểm, chỉ muộn hơn vài trăm ms.
    (void)tko_ceiling;
    (void)hold_tune;

    st->last_throttle = collective;
    out->throttle_duty = collective;
    out->target_z_m = st->target_z_m;
    out->vz_target_ms = vz_target;
    out->vz_i_term = hold_st->vz_integral;
    out->alt_thrust_corr = (float)collective - hold_tune->hover;

    // ---- LIFTOFF — THEO THỜI GIAN TRONG CLIMB ----
    // Trước đây: `est_z > ground_alt + delta`. Bỏ vì est_z ở cấu hình baro-only
    // sai 10-30cm do propwash (xem tuning.h 3c) nên nó không trả lời được câu
    // "đã rời đất chưa".
    //
    // Đây KHÔNG phải chỉ telemetry — nó nới trần |I| vòng Vz ngay dưới, mở Ki
    // attitude bên flight_core.c, và bật pha airborne của estimator. Nên
    // tune->liftoff_ms là con số PHẢI đo trên khung thật, không phải đoán:
    // ngắn quá -> mở Ki khi còn đè đất -> ground windup; dài quá -> Ki khoá
    // trong lúc đã bay -> drone trôi.
    // ---- BẰNG CHỨNG RỜI ĐẤT: ĐỘ CAO + GA, KHÔNG DÙNG Vz ----
    //
    // ⚠ ĐÃ BỎ VẾ `tof_vz_ms >= TAKEOFF_LIFTOFF_VZ_MS`. Đo được trên phần cứng
    // thật (log TKOEL=6.5..7.1): drone ĐANG LÊN ĐỀU, tof_z_m đi 0.198 -> 0.419,
    // nhưng tof_vz_lpf_ms dao động 0.185/0.105/0.252/0.138/0.225/0.170/-0.021.
    // Vz đạo hàm từ ToF ~30Hz ở tốc độ leo chậm (0.1-0.3 m/s) có tỷ lệ
    // tín-hiệu/nhiễu rất thấp, nên nó CHẠM 0 thường xuyên ngay giữa lúc leo
    // hoàn toàn bình thường.
    //
    // VÌ SAO ĐIỀU ĐÓ GIẾT CẢ CHUỖI: 4 vế này AND với nhau rồi đi qua
    // tko_latch_window() đòi ĐÚNG liên tục TAKEOFF_LIFTOFF_MS (1000ms). Một
    // mẫu Vz rớt là cửa sổ RESET VỀ 0. Với nhiễu như trên, cửa sổ không bao giờ
    // đóng -> liftoff_flag mãi false -> airborne mãi false -> estimator ép
    // alt_m = 0 và KHÔNG BAO GIỜ fuse ToF (TOFA=0 trong suốt chuyến bay) ->
    // abort. Drone bay lên thật mà firmware khẳng định nó chưa rời đất.
    //
    // Độ cao thì KHÔNG có vấn đề đó: nó là số ĐO trực tiếp, không phải đạo hàm,
    // nên không bị khuếch đại nhiễu. Giữ thêm vế ga để loại trường hợp ai đó
    // NHẤC drone lên bằng tay lúc motor chưa đủ lực.
    const bool lift_evidence = tof_fusable &&
        collective >= (int)(hold_tune->hover * TAKEOFF_LIFTOFF_THR_FRAC) &&
        tof_z_m >= TAKEOFF_LIFTOFF_Z_M;
    if (!st->liftoff_flag && tko_latch_window(lift_evidence,
            &st->liftoff_active, &st->liftoff_since_us, now_us, tune->liftoff_ms)) {
        st->liftoff_flag = true;
        out->liftoff_edge = true;
        st->target_z_m = fmaxf(tof_z_m, 0.0f);
    }
    // ---- ABORT "KHÔNG NHẤC NỔI" — CHỈ XÉT ĐỘ CAO ĐO ĐƯỢC ----
    // Điều kiện DUY NHẤT: sau TAKEOFF_NO_LIFT_TIMEOUT_MS trong CLIMB mà ToF vẫn
    // đọc dưới TAKEOFF_NO_LIFT_ALT_M thì drone thật sự chưa đi lên.
    //
    // VÌ SAO KHÔNG DÙNG LẠI `lift_evidence` (bản trước dùng, và nó SAI Ở ĐÂY):
    // lift_evidence là AND của BỐN vế và được thiết kế cho một câu hỏi KHÁC —
    // "đã đủ chắc chắn để nới trần I và mở Ki attitude chưa". Nó CỐ Ý khắt khe,
    // vì mở Ki sớm khi còn đè đất thì ground-windup. Dùng đúng bộ khắt khe đó
    // làm điều kiện HỦY CHUYẾN BAY thì một chuyến hoàn toàn bình thường vẫn bị
    // hủy chỉ vì MỘT vế phụ chưa khớp — điển hình là `tof_vz_ms >= 0.05`, vốn
    // rớt xuống dưới ngưỡng ngay khi drone leo đều rồi chững lại một nhịp.
    //
    // Hai câu hỏi khác nhau thì phải có hai điều kiện khác nhau:
    //   lift_evidence  -> "đã rời đất CHẮC CHẮN chưa" (gate Ki, khắt khe, 4 vế)
    //   khối này       -> "có đi lên được KHÔNG" (hủy bay, thô, 1 vế: độ cao)
    //
    // tof_fusable KHÔNG có mặt ở đây là CÓ CHỦ ĐÍCH: mất ToF đã có nhánh (D)
    // TKO_ABORT_TOF_LOST xử lý ở ĐẦU hàm, chạy TRƯỚC khối này. Thêm nó vào đây
    // chỉ tạo hai mã abort cho cùng một sự cố.
    // ⚠ NGƯỠNG PHẢI CO THEO TARGET, KHÔNG ĐƯỢC DÙNG HẰNG SỐ TRẦN:
    // target thấp hơn TAKEOFF_NO_LIFT_ALT_M (vd fc.takeoff(150) -> 0.15m) sẽ
    // bay ĐÚNG tới đích rồi vẫn bị hủy vì 0.15 < 0.20 — hủy một chuyến bay
    // hoàn hảo, với lý do ghi là "không nhấc nổi". Lấy min() với một phần của
    // target để ngưỡng luôn nằm DƯỚI đích thật sự.
    // ---- ABORT "KHÔNG NHẤC NỔI": ĐÃ BỎ (yêu cầu người dùng) ----
    // Khối cũ hủy chuyến khi sau TAKEOFF_NO_LIFT_TIMEOUT_MS mà ToF vẫn đọc dưới
    // no_lift_alt. Nó đã báo giả nhiều lần trên bo này.
    //
    // ⚠ CÁI CÒN LẠI khi drone thật sự không nhấc nổi:
    //   - TKO_ABORT_TIMEOUT vẫn hủy khi cả pha CLIMB quá hạn -> vẫn không có
    //     chuyện motor quay mãi mãi mà không ai dừng.
    //   - Commander vẫn có "motor saturated too long" và sàn pin.
    // Tức là mất phát hiện SỚM (vài trăm ms), không mất phát hiện HẲN.
    //
    // liftoff_flag GIỮ NGUYÊN: nó là gate mở Ki attitude / nới trần I, không
    // phải điều kiện hủy bay. Bỏ nhầm nó sẽ gây ground-windup.
    out->liftoff_flag = st->liftoff_flag;

    // ---- VÀO HOLDING — RỜI ĐẤT + RATE-LIMITER ĐÃ TRƯỢT HẾT ----
    // Hai điều kiện, giữ liên tục TAKEOFF_HOLD_ENTER_MS. KHÔNG so với est_z,
    // KHÔNG đòi vz về 0 — xem khối lý do ngay trên `hold_ready`.
    //
    // `target_z == final_target` KHÔNG phải phép đo — nó là trạng thái của
    // chính rate-limiter vài dòng phía trên, hoàn toàn xác định theo đồng hồ.
    // Nên toàn bộ chuỗi cất cánh giờ không có cảm biến độ cao nào tham gia vào
    // quyết định "đã xong hay chưa".
    //
    // Dùng tko_latch_window() thay cho bộ đếm tick tự chế: cùng một cơ chế với
    // guard tilt/mất-nguồn ở đầu hàm, và KHÔNG phụ thuộc dt (bộ đếm tick cũ chia
    // cho dt nên một tick dài bất thường sẽ đếm sai).
    //
    // ⚠ KHÔNG có sàn thời gian CLIMB ở đây, và đó là quyết định có ý thức.
    // Tôi đã thử thêm một sàn để bảo đảm bộ phát hiện "kịch trần" luôn kết luận
    // TRƯỚC khi bàn giao, nhưng nó không thắng được cuộc đua: I cần hàng chục
    // giây mới bão hoà (xem tính toán ở khối STUCK trên), nên sàn đó sẽ làm MỌI
    // chuyến cất cánh chờ ngần ấy thời gian — kể cả chuyến hoàn toàn bình thường.
    //
    // Hệ quả phải biết: với target THẤP (chuỗi ngắn), một drone bị chặn có thể
    // được bàn giao sang HOLDING TRƯỚC khi kịp bị phát hiện. Lúc đó bằng chứng
    // vẫn hiện ra ở telemetry (ALTSAT=1 kéo dài, TKOI kịch trần) nhưng KHÔNG có
    // auto-abort — người lái phải tự KILL. Xem tuning.h mục 3c.
    // ---- ĐÃ RÚT GỌN CÒN HAI VẾ (yêu cầu người dùng) ----
    // Trước đây có BỐN vế, và hai vế cuối là nguyên nhân một lần cất cánh thật
    // bị kẹt: drone leo tới 1.45m trong khi target 1.00m (hover_ff latch thiếu
    // ~460 duty so với pin lúc bay, I-term không kéo lại kịp), nên
    //     |alt_m - final_target| <= TAKEOFF_HOLD_Z_TOL_M
    // KHÔNG BAO GIỜ đúng -> cửa sổ không đóng -> TKO_ABORT_TIMEOUT sau 21.6s.
    //
    // Hai vế đó hỏi "đã tới đúng độ cao và đứng yên chưa" — một câu hỏi mà
    // HOLDING sinh ra để trả lời. Bắt CLIMB trả lời trước là bắt vòng hở làm
    // việc của vòng kín: CLIMB chỉ có rate-limiter và I-term đang nạp, còn
    // HOLDING mới có đủ P+I trên sai số độ cao thật.
    //
    // GIỜ: rời đất + rate-limiter đã trượt hết -> bàn giao. Sai số độ cao còn
    // lại (dù 45cm) là việc của alt_hold, và nó xử lý được vì đó đúng là việc
    // của nó.
    //
    // ⚠ CÁI MẤT: không còn bảo đảm "bàn giao ở đúng độ cao đích". Drone có thể
    // vào HOLDING khi còn lệch, rồi alt_hold kéo về — nhìn sẽ thấy nó trôi một
    // đoạn sau khi báo TAKEOFF XONG. Đó là đánh đổi đã chọn: thà bàn giao hơi
    // sớm còn hơn không bao giờ bàn giao.
    // TAKEOFF_HOLD_Z_TOL_M / TAKEOFF_HOLD_VZ_TOL_MS giờ không còn ai đọc.
    const bool hold_ready = st->liftoff_flag &&
        st->target_z_m == st->final_target_m;
    if (tko_latch_window(hold_ready,
                          &st->at_target_active, &st->at_target_since_us,
                          now_us, TAKEOFF_HOLD_ENTER_MS)) {
        tko_handoff(st, now_us, out);
        return;
    }

    out->phase = st->phase;
}

// ================= LANDING =================

landing_tune_t landing_default_tune(void) {
    landing_tune_t t;
    t.descent_vz = LAND_DESCENT_VZ;
    t.flare_alt_m = LAND_FLARE_ALT_M;
    t.flare_vz = LAND_FLARE_VZ;
    t.touchdown_alt_m = LAND_TOUCHDOWN_ALT_M;
    return t;
}

// ============================================================================
// ⚠ alt_m VÀ tof_z_m Ở HÀM NÀY LÀ **AGL** (độ cao trên BỀ MẶT ĐANG Ở DƯỚI),
//   KHÔNG PHẢI độ cao trên sàn cất cánh. Xem khối cảnh báo ở takeoff_land.h.
//   Hạ xuống một cái bàn cao 0.75m: AGL về 0 là đã CHẠM BÀN; datum về 0 là đã
//   đâm xuyên qua bàn xuống sàn.
// ============================================================================
void landing_run(landing_state_t *st, const landing_tune_t *tune,
                  alt_hold_state_t *hold_st, const alt_hold_tune_t *hold_tune,
                  bool was_hold_engaged,
                  bool alt_valid, float alt_m, float vz_ms, int manual_throttle_duty,
                  bool tof_fusable, float tof_z_m, float tof_vz_ms,
                  bool terrain_pending, uint32_t terrain_commits, float az_earth_ms2,
                  float dt, int64_t now_us, int safe_max_duty,
                  landing_result_t *out) {
    out->hold_driving = true;
    out->touchdown_done = false;

    // ---- C1: KHÔNG vào DESCEND khi terrain đang PENDING ----
    // Ở mép bàn range nhảy qua nhảy lại; bắt đầu hạ ngay lúc đó là flare sai
    // điểm. Giữ độ cao (vz_target = 0) cho tới khi estimator xác nhận xong —
    // pending tự hết sau TERR_CONFIRM_N mẫu (~100-150ms), không phải chờ vô
    // hạn. KHÔNG chạm vz_integral: nó vẫn đang giữ hover đã học.
    if (st->phase == LAND_IDLE && terrain_pending) {
        if (!was_hold_engaged && !hold_st->engaged) {
            alt_hold_preload(hold_st, hold_tune, manual_throttle_duty);
        }
        out->throttle_duty = alt_hold_vz_cascade(hold_st, hold_tune, 0.0f, vz_ms, dt,
                                                  true /* FREEZE I: số liệu đang loạn */,
                                                  hold_tune->vz_ilimit,
                                                  ALT_HOLD_MIN_THROTTLE_DUTY, safe_max_duty);
        st->last_throttle = out->throttle_duty;
        return;
    }

    if (st->phase == LAND_IDLE) {
        st->phase = LAND_DESCEND;
        st->settle_us = 0;
        st->toflost_us = 0;
        st->az_spike_us = 0;
        st->terr_commit_seen = terrain_commits;
        st->terr_seen_init = true;
        // Vào từ HOLD/flight: giữ vz_integral warm (bumpless). Vào từ manual
        // (chưa engaged) thì preload = throttle - hover.
        if (!was_hold_engaged) {
            alt_hold_preload(hold_st, hold_tune, manual_throttle_duty);
        }
    }

    // ---- C2: BẬC TERRAIN GIỮA LÚC ĐANG HẠ -> RESET PHA VỀ DESCEND ----
    // Đang hạ mà drone trôi ngang khỏi mép bàn: AGL đột ngột TĂNG ~0.75m. Logic
    // flare/touchdown ăn thẳng số đó sẽ tưởng vừa bay vọt lên — và nguy hiểm
    // hơn: bộ đếm contact/settle đang chạy dở sẽ mang bằng chứng của BỀ MẶT CŨ
    // sang bề mặt mới. Xoá sạch mọi bộ đếm và tính lại flare theo AGL mới.
    // TUYỆT ĐỐI KHÔNG nhảy thẳng TOUCHDOWN hay cắt ga ở đây.
    if (!st->terr_seen_init) {
        st->terr_commit_seen = terrain_commits;
        st->terr_seen_init = true;
    } else if (terrain_commits != st->terr_commit_seen) {
        st->terr_commit_seen = terrain_commits;
        if (st->phase != LAND_TOUCHDOWN) {   // đã cutoff thì không quay lại nữa
            st->phase = LAND_DESCEND;
            st->contact_ticks = 0;
            st->settle_us = 0;
            st->az_spike_us = 0;
            st->cutoff_us = 0;
        }
    }

    // ---- TOUCHDOWN: ramp ga về 0 trong CUTOFF_MS rồi báo caller disarm ----
    if (st->phase == LAND_TOUCHDOWN) {
        if (st->cutoff_us == 0) {
            st->cutoff_us = now_us;
            st->cutoff_thr = (float)manual_throttle_duty;
        }
        const int64_t el = now_us - st->cutoff_us;
        const int64_t dur = (int64_t)LAND_CUTOFF_MS * 1000;
        if (el >= dur) {
            out->touchdown_done = true;
            out->hold_driving = false;
            out->throttle_duty = 0;
            landing_reset(st);
            return;
        }
        const float frac = 1.0f - (float)el / (float)dur;
        out->throttle_duty = clampi((int)(st->cutoff_thr * frac), 0, safe_max_duty);
        return;
    }

    // ---- BLIND: hạ mù ga giảm cố định (mất ToF). ToF lại -> về DESCEND ----
    if (st->phase == LAND_BLIND) {
        if (alt_valid && tof_fusable) {
            st->phase = LAND_DESCEND;   // fall-through xuống cascade
            st->toflost_us = 0;
        } else {
            if (st->blind_thr <= 0.0f) st->blind_thr = (float)manual_throttle_duty;
            st->blind_thr -= LAND_BLIND_DESCENT_RATE * dt;
            if (st->blind_thr < (float)LAND_MIN_THROTTLE) {
                st->phase = LAND_TOUCHDOWN;   // đủ thấp -> cutoff (vòng kế)
                st->cutoff_us = 0;
            }
            out->throttle_duty = clampi((int)st->blind_thr, 0, safe_max_duty);
            return;
        }
    }

    // ---- Mất ToF quá lâu ở DESCEND/FLARE -> chuyển BLIND ----
    if (!alt_valid || !tof_fusable) {
        if (st->toflost_us == 0) st->toflost_us = now_us;
        if ((now_us - st->toflost_us) > (int64_t)LAND_TOF_TIMEOUT_MS * 1000) {
            st->phase = LAND_BLIND;
            st->blind_thr = (float)manual_throttle_duty;
            out->throttle_duty = clampi(manual_throttle_duty, 0, safe_max_duty);
            return;
        }
    } else {
        st->toflost_us = 0;
    }

    // ---- DESCEND -> FLARE khi gần đất ----
    if (st->phase == LAND_DESCEND && alt_valid && alt_m < tune->flare_alt_m) {
        st->phase = LAND_FLARE;
    }

    // ---- vz_target ép (âm = hạ). FLARE nội suy chậm dần theo alt ----
    float vz_target = -tune->descent_vz;
    if (st->phase == LAND_CONTACT_CANDIDATE) {
        // Đang NGHI đã chạm: giữ lệnh hạ CHẬM NHẤT. Không dừng hẳn (nếu nghi
        // sai thì phải tiếp tục hạ), không hạ nhanh (nếu nghi đúng thì đang
        // ấn drone xuống nền).
        vz_target = -tune->flare_vz;
    } else if (st->phase == LAND_FLARE) {
        const float span = tune->flare_alt_m - tune->touchdown_alt_m;
        float f = (span > 1e-3f) ? (alt_m - tune->touchdown_alt_m) / span : 0.0f;
        f = clampf(f, 0.0f, 1.0f);   // 1 ở flare_alt, 0 ở touchdown
        vz_target = -(tune->flare_vz + (tune->descent_vz - tune->flare_vz) * f);
    }

    // ---- Cascade vz (đúng bộ HOLD, vz_integral CHUNG) ----
    // freeze_integral=false: đang hạ nên mặt đất chưa giữ drone; conditional
    // integration bên trong cascade đủ để chặn windup lúc chạm sàn duty.
    const int throttle_cmd = alt_hold_vz_cascade(hold_st, hold_tune, vz_target, vz_ms, dt,
                                                  false, hold_tune->vz_ilimit,
                                                  ALT_HOLD_MIN_THROTTLE_DUTY, safe_max_duty);
    out->throttle_duty = throttle_cmd;

    // ================= TOUCHDOWN DETECT — ĐA ĐIỀU KIỆN, CÓ DUY TRÌ =================
    //
    // VÌ SAO KHÔNG DÙNG "alt < touchdown_alt_m" MỘT MÌNH (như bản cũ):
    // board GIỜ CÓ ToF (VL53L0X), nhưng Z world vẫn đến từ estimator với nhiễu
    // ±0.3-1m sát đất — lớn gấp NHIỀU LẦN touchdown_alt_m (0.05m). Một cú tụt
    // áp do gió hoặc propwash là đủ để "alt < 0.05" thành true GIỮA KHÔNG
    // TRUNG -> cắt máy -> rơi. Đây là kiểu lỗi giết drone chắc chắn nhất trong
    // toàn bộ file này.
    //
    // Bốn bằng chứng, mỗi cái 1 điểm (cùng nguyên tắc với liftoff detector):
    //   1. throttle đã tụt về sát sàn hạ cánh (motor không còn đỡ nổi -> đang
    //      có gì đó khác đỡ, tức mặt đất)
    //   2. |Vz| nhỏ dù đang LỆNH hạ (còn trên không thì phải đang đi xuống)
    //   3. Z thấp (bằng chứng PHỤ — yếu nhất vì baro, nên không tự quyết được)
    //   4. Z KHÔNG còn giảm nữa dù vẫn lệnh hạ (dấu hiệu mạnh: bị chặn cơ học)
    // Đủ điểm -> vào CONTACT_CANDIDATE (VẪN GIỮ ĐIỀU KHIỂN, chưa cắt gì), phải
    // duy trì LAND_CONTACT_TICKS tick liên tục mới sang TOUCHDOWN.
    {
        const bool thr_reducing = st->last_throttle == 0 || throttle_cmd <= st->last_throttle + 5;
        const bool vz_still     = fabsf(tof_vz_ms) < LAND_CONTACT_VZ_MS;
        const bool alt_low      = tof_fusable && tof_z_m <= tune->touchdown_alt_m;
        // Z ngừng giảm: so với Z lúc bắt đầu nghi ngờ. Chỉ có nghĩa khi ĐANG
        // ở CONTACT_CANDIDATE (đã có mốc để so).
        const bool z_stopped = alt_low && vz_still;

        int score = 0;
        if (thr_reducing) score++;
        if (vz_still)     score++;
        if (alt_low)      score++;
        if (z_stopped)    score++;
        st->last_contact_score = score;

        // ---- NHÁNH 1 (CHÍNH): AGL thấp + vz lặng + ga đang giảm, DUY TRÌ ----
        if (tof_fusable && alt_low && vz_still && thr_reducing) {
            if (st->phase != LAND_CONTACT_CANDIDATE) {
                st->phase = LAND_CONTACT_CANDIDATE;
                st->alt_at_candidate = alt_valid ? alt_m : 0.0f;
                st->contact_ticks = 0;
            }
            st->contact_ticks++;
            if (st->contact_ticks >= LAND_CONTACT_TICKS) {
                st->phase = LAND_TOUCHDOWN;
                st->cutoff_us = 0;
            }
        } else {
            // Mất điều kiện -> QUAY LẠI hạ bình thường. Một accel spike hay
            // một mẫu ToF xấu không được để lại dấu vết gì.
            if (st->phase == LAND_CONTACT_CANDIDATE) {
                st->phase = (alt_valid && alt_m < tune->flare_alt_m) ? LAND_FLARE : LAND_DESCEND;
            }
            st->contact_ticks = 0;
        }

        // ---- NHÁNH 2 (BACKUP): GA SÁT SÀN + Vz LẶNG, KHÔNG DÙNG ĐỘ CAO ----
        // ĐỔI THEO SPEC C3: điều kiện giờ là `ga <= LAND_MIN_THROTTLE` VÀ
        // `|vz| < LAND_SETTLE_VZ_MS`, giữ liên tục LAND_SETTLE_MS.
        //
        // VÌ SAO BỎ ĐIỀU KIỆN ĐỘ CAO Ở NHÁNH NÀY (bản trước là
        // `alt_low && vz_still`, tức vẫn phụ thuộc ToF): VL53L0X ở cự ly rất
        // gần (<3-5cm) đọc kém tin cậy và thường rơi hẳn khỏi gate hình học
        // (ALT_EST_TOF_MIN_RANGE_M = 3cm) -> alt_low KHÔNG BAO GIỜ true đúng
        // lúc drone đã nằm trên nền. Nhánh backup mà lại phụ thuộc chính cái
        // phép đo đang hỏng thì nó không phải backup. Hai bằng chứng còn lại
        // (ga đã tụt kịch sàn hạ cánh mà drone vẫn không đi xuống nữa) là
        // bằng chứng CƠ HỌC, không cần cảm biến độ cao.
        if (throttle_cmd <= LAND_MIN_THROTTLE && fabsf(vz_ms) < LAND_SETTLE_VZ_MS) {
            if (st->settle_us == 0) st->settle_us = now_us;
            if ((now_us - st->settle_us) >= (int64_t)LAND_SETTLE_MS * 1000) {
                st->phase = LAND_TOUCHDOWN;
                st->cutoff_us = 0;
            }
        } else {
            st->settle_us = 0;
        }

        // ---- NHÁNH 3: SPIKE az_earth DUY TRÌ ----
        // Xem cảnh báo DẤU ở tuning.h (LAND_TOUCHDOWN_AZ_MS2): ngưỡng đang là
        // giá trị ÂM đúng theo spec, nên nhánh này bắt "gia tốc hướng xuống
        // lớn", không phải "va chạm đẩy lên". Duy trì bắt buộc — một mẫu accel
        // đơn lẻ TUYỆT ĐỐI không được cắt máy.
        if (az_earth_ms2 < LAND_TOUCHDOWN_AZ_MS2) {
            if (st->az_spike_us == 0) st->az_spike_us = now_us;
            if ((now_us - st->az_spike_us) >= (int64_t)LAND_TOUCHDOWN_AZ_HOLD_MS * 1000) {
                st->phase = LAND_TOUCHDOWN;
                st->cutoff_us = 0;
            }
        } else {
            st->az_spike_us = 0;
        }

        st->last_throttle = throttle_cmd;
    }
}
