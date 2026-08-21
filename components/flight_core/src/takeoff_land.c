#include "flight_core/takeoff_land.h"

#include <math.h>

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
                  bool alt_source_ok, float tilt_deg,
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
        alt_source_lost = !alt_source_ok || !alt_valid;
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
        out->throttle_duty = clampi(tune->prime_duty, 0, safe_max_duty);
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
    // ⚠ SO VỚI TRẦN NÀO: phải là trần THẨM QUYỀN CỦA CHÍNH VÒNG Vz (I chạm
    // i_limit), KHÔNG phải trần collective của mixer. Với cấu hình hiện tại
    // (hover_ff=1000, ALT_HOLD_VZ_ILIMIT=500) collective tối đa chỉ tới
    // 1000+500+P ~ 1500, trong khi trần mixer là 2000*0.85 = 1700 — nên nếu chỉ
    // so với trần mixer thì điều kiện KHÔNG BAO GIỜ đúng và cả bộ phát hiện
    // thành code chết mà không ai biết. Giữ cả hai vế: I-limit bắt trường hợp
    // thường, trần mixer bắt trường hợp hover_ff đặt rất cao.
    const float i_limit_now = st->liftoff_flag ? hold_tune->vz_ilimit
                                                : TAKEOFF_PRELIFT_I_LIMIT_DUTY;
    const bool i_exhausted = (hold_st->vz_integral >= 0.95f * i_limit_now);
    const bool at_ceiling = (collective >= tko_ceiling) || i_exhausted;
    if (tko_latch_window(st->liftoff_flag && at_ceiling,
                          &st->stuck_active, &st->stuck_since_us,
                          now_us, TAKEOFF_STUCK_MS)) {
        st->last_throttle = collective;
        tko_abort(st, TKO_ABORT_STUCK, now_us, out);
        return;
    }

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
    if (!st->liftoff_flag &&
        (now_us - st->phase_since_us) >= (int64_t)tune->liftoff_ms * 1000) {
        st->liftoff_flag = true;
    }
    out->liftoff_flag = st->liftoff_flag;

    // ---- VÀO HOLDING — CŨNG THUẦN THỜI GIAN ----
    // Điều kiện DUY NHẤT: target_z đã trượt tới đích, giữ liên tục
    // TAKEOFF_HOLD_ENTER_MS. KHÔNG còn so với est_z.
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
    if (tko_latch_window((st->target_z_m == st->final_target_m),
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

void landing_run(landing_state_t *st, const landing_tune_t *tune,
                  alt_hold_state_t *hold_st, const alt_hold_tune_t *hold_tune,
                  bool was_hold_engaged,
                  bool alt_valid, float alt_m, float vz_ms, int manual_throttle_duty,
                  float dt, int64_t now_us, int safe_max_duty,
                  landing_result_t *out) {
    if (st->phase == LAND_IDLE) {
        st->phase = LAND_DESCEND;
        st->settle_us = 0;
        st->toflost_us = 0;
        // Vào từ HOLD/flight: giữ vz_integral warm (bumpless). Vào từ manual
        // (chưa engaged) thì preload = throttle - hover.
        if (!was_hold_engaged) {
            alt_hold_preload(hold_st, hold_tune, manual_throttle_duty);
        }
    }

    out->hold_driving = true;
    out->touchdown_done = false;

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
        if (alt_valid) {
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
    if (!alt_valid) {
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
        const bool thr_at_floor = throttle_cmd <= (LAND_MIN_THROTTLE + LAND_CONTACT_THR_MARGIN);
        const bool vz_still     = fabsf(vz_ms) < LAND_CONTACT_VZ_MS;
        const bool alt_low      = alt_valid && alt_m < (tune->touchdown_alt_m + LAND_CONTACT_ALT_MARGIN_M);
        // Z ngừng giảm: so với Z lúc bắt đầu nghi ngờ. Chỉ có nghĩa khi ĐANG
        // ở CONTACT_CANDIDATE (đã có mốc để so).
        const bool z_stopped = (st->phase == LAND_CONTACT_CANDIDATE) && alt_valid &&
                                ((st->alt_at_candidate - alt_m) < LAND_CONTACT_Z_PROGRESS_M);

        int score = 0;
        if (thr_at_floor) score++;
        if (vz_still)     score++;
        if (alt_low)      score++;
        if (z_stopped)    score++;
        st->last_contact_score = score;

        if (score >= LAND_CONTACT_SCORE_MIN) {
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
            // một mẫu baro xấu không được để lại dấu vết gì.
            if (st->phase == LAND_CONTACT_CANDIDATE) {
                st->phase = (alt_valid && alt_m < tune->flare_alt_m) ? LAND_FLARE : LAND_DESCEND;
            }
            st->contact_ticks = 0;
        }

        // settle_us giữ nguyên ý nghĩa cũ (ga sát sàn + vz lặng) nhưng giờ chỉ
        // là ĐƯỜNG DỰ PHÒNG dài hạn: nếu bộ chấm điểm ở trên vì lý do nào đó
        // không bao giờ đủ điểm mà drone rõ ràng đã nằm im rất lâu, vẫn phải
        // kết thúc được thay vì quay motor vô hạn.
        if (thr_at_floor && vz_still) {
            if (st->settle_us == 0) st->settle_us = now_us;
            if ((now_us - st->settle_us) >= (int64_t)LAND_SETTLE_MS * 1000) {
                st->phase = LAND_TOUCHDOWN;
                st->cutoff_us = 0;
            }
        } else {
            st->settle_us = 0;
        }
    }
}
