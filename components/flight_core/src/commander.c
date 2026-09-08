#include "flight_core/commander.h"

#include <math.h>

commander_config_t commander_default_config(void) {
    commander_config_t c;
    c.alt_min_m = COMMANDER_DEFAULT_ALT_MIN_M;
    c.alt_max_m = COMMANDER_DEFAULT_ALT_MAX_M;
    c.battery_floor_v = COMMANDER_DEFAULT_BATTERY_FLOOR_V;
    c.heartbeat_timeout_ms = COMMANDER_DEFAULT_HEARTBEAT_MS;
    c.hard_tilt_deg = COMMANDER_DEFAULT_HARD_TILT_DEG;
    c.motor_sat_hard_ms = COMMANDER_DEFAULT_MOTOR_SAT_MS;
    return c;
}

commander_result_t commander_evaluate(commander_state_t *st, const commander_config_t *cfg,
                                       const commander_inputs_t *in, int64_t now_us) {
    commander_result_t r = {FAULT_NONE, "ok"};

    // Trạng thái CHƯA armed thì không có gì để bảo vệ — mọi ngưỡng dưới đây
    // đều vô nghĩa khi motor không quay (và sẽ báo fault giả liên tục lúc
    // drone nằm trên bàn: heartbeat chưa ai gửi, pin chưa đọc...).
    if (in->state == FSM_DISARMED) {
        st->motor_sat_since_us = 0;
        return r;
    }

    // "Đang bay" = có thể rơi nếu cắt máy. Dùng để chọn giữa "hạ êm" và "cắt
    // luôn": còn ở đất thì cắt là an toàn nhất, đang trên trời thì cắt là tệ nhất.
    const bool airborne = in->airborne ||
                           in->state == FSM_HOLDING || in->state == FSM_FLYING ||
                           in->state == FSM_LANDING;

    // ========================================================================
    // HARD faults — mất kiểm soát thật, không cứu bằng cách bay tiếp được
    // ========================================================================

    // IMU: kiểm tra CẢ hai mặt. imu_ok=false nghĩa là Mahony tự báo không hợp
    // lệ; imu_stale nghĩa là Mahony vẫn "valid" nhưng đang nhai lại mẫu cũ vì
    // sensor_hub ngừng cập nhật (bus treo). Trường hợp thứ hai nguy hiểm hơn
    // vì mọi thứ TRÔNG VẪN BÌNH THƯỜNG — attitude đứng yên một chỗ trong khi
    // drone thật đang nghiêng dần.
    if (!in->imu_ok) {
        r.fault = FAULT_HARD;
        r.reason = "imu invalid";
        return r;
    }
    const float ar = fabsf(in->roll_deg);
    const float ap = fabsf(in->pitch_deg);
    if (ar > cfg->hard_tilt_deg || ap > cfg->hard_tilt_deg) {
        r.fault = FAULT_HARD;
        r.reason = "tilt exceeded hard limit";
        return r;
    }

    // Vòng điều khiển chạy chậm hẳn: PID/estimator đang tính trên dt sai và
    // phản ứng trễ. Không phải lỗi cảm biến nên không có gì "hỏng" để thấy —
    // phải đo trực tiếp bằng số tick trượt hạn liên tiếp.
    if (in->deadline_miss_streak >= COMMANDER_DEADLINE_MISS_HARD) {
        r.fault = FAULT_HARD;
        r.reason = "control loop tre han lien tuc";
        return r;
    }

    // Motor bão hoà kéo dài: đã hết thẩm quyền điều khiển, PID đòi nhiều hơn
    // mức phần cứng cho được. CHỈ tính khi đang bay — lúc còn ở đất
    // (TAKING_OFF spool) duty kịch trần là chuyện bình thường theo thiết kế.
    if (airborne && in->motor_saturated) {
        if (st->motor_sat_since_us == 0) st->motor_sat_since_us = now_us;
        if ((now_us - st->motor_sat_since_us) > (int64_t)cfg->motor_sat_hard_ms * 1000) {
            r.fault = FAULT_HARD;
            r.reason = "motor saturated too long";
            return r;
        }
    } else {
        st->motor_sat_since_us = 0;
    }

    // ========================================================================
    // SOFT faults — còn kiểm soát được, hạ êm
    // ========================================================================

    // Theo policy UAV-Mini: khi hub ngừng cập nhật nhưng attitude cuối vẫn còn
    // hợp lệ, ưu tiên thử LANDING qua Commander. Nếu attitude mất hẳn hoặc góc
    // nghiêng vượt giới hạn, lớp hard-fault/cut trực tiếp vẫn thắng.
    if (in->imu_stale && in->state != FSM_LANDING && in->state != FSM_EMERGENCY) {
        r.fault = FAULT_SOFT;
        r.reason = "imu stale -> thu landing bang attitude cuoi";
        return r;
    }

    // Heartbeat: PHẢN ỨNG THEO STATE.
    //   LANDING  -> BỎ QUA. Đang hạ rồi, báo soft fault nữa chỉ để chuyển sang
    //               ... LANDING, vô nghĩa. Tệ hơn: nếu ground station rớt đúng
    //               lúc hạ cánh thì spam fault che mất fault thật.
    //   EMERGENCY-> BỎ QUA, đã có đường xử lý riêng.
    //   còn lại  -> SOFT (ARMED: về DISARMED; TAKING_OFF: huỷ leo, hạ ngay;
    //               HOLDING/FLYING: hạ êm).
    const int64_t since_hb_us = now_us - st->last_heartbeat_us;
    const bool hb_lost = since_hb_us > (int64_t)cfg->heartbeat_timeout_ms * 1000;
    if (hb_lost && in->state != FSM_LANDING && in->state != FSM_EMERGENCY) {
        r.fault = FAULT_SOFT;
        r.reason = (in->state == FSM_TAKING_OFF)
            ? "heartbeat timeout khi dang cat canh -> huy leo"
            : "heartbeat timeout (python script treo?)";
        return r;
    }

    // Pin dưới sàn: BỎ QUA khi đang LANDING — đang hạ rồi, chuyển sang LANDING
    // lần nữa không giúp gì, mà giữ nguyên pha hạ hiện tại thì tốt hơn là reset
    // nó. Vẫn tính ở TAKING_OFF: cất cánh với pin yếu là cách chắc chắn nhất
    // để rơi giữa chừng.
    // ---- SAN PIN -> EP HA CANH, co DEBOUNCE ----
    // battery_v == 0 nghia la mau KHONG hop le (battery_driver da chan). Do
    // duoc tren log: ADC tra 4.936V cho pin 1S -> BATVALID=0 -> BATV=0.00.
    // Mau nhu vay KHONG duoc xoa dong ho (ADC chap chon ma cu xoa thi fault
    // khong bao gio trip duoc), cung khong duoc tinh la duoi san.
    if (in->state != FSM_LANDING && in->battery_v > 0.0f) {
        if (in->battery_v <= cfg->battery_floor_v) {
            if (st->battery_low_since_us == 0) st->battery_low_since_us = now_us;
            if ((now_us - st->battery_low_since_us) >=
                    (int64_t)COMMANDER_BATTERY_LOW_HOLD_MS * 1000) {
                r.fault = FAULT_SOFT;
                r.reason = "battery below floor";
                return r;
            }
        } else {
            // Mau HOP LE va TREN san -> pin chua yeu that, xoa dong ho.
            st->battery_low_since_us = 0;
        }
    }

    // Mất estimator: chỉ có nghĩa khi đang bay và đang thật sự dùng nó để giữ
    // độ cao. Ở LANDING vẫn theo dõi (landing có nhánh BLIND xử lý) nhưng
    // không báo fault để khỏi reset pha hạ. BENCH_RAMP KHÔNG dùng estimator để
    // giữ gì cả (throttle đi thẳng từ ramp, không qua alt_hold) — báo fault ở
    // đây chỉ tạo false-positive (vd baro chưa init/indoor) làm gián đoạn oan
    // một bài test không liên quan tới độ cao.
    //
    // FSM_ARMED cũng được LOẠI TRỪ, cùng LÝ DO với BENCH_RAMP: ở ARMED
    // throttle_cmd=0 (xem flight_core.c bước 9), motor chưa quay, alt_hold
    // KHÔNG chạy — không có gì đang "giữ độ cao" để mà mất. Bắt buộc phải loại
    // trừ vì ở ARMED motor chưa chạy và floor ToF vẫn đang được kiểm tra bởi
    // prearm/takeoff gate; runtime fault chỉ có nghĩa khi controller đang bay.
    if (in->state != FSM_LANDING && in->state != FSM_BENCH_RAMP &&
        in->state != FSM_ARMED &&
        (in->alt_hold_engage_lost || in->alt_estimator_lost)) {
        r.fault = FAULT_SOFT;
        r.reason = "altitude estimator lost mid-flight";
        return r;
    }

    // Estimator còn HỢP LỆ nhưng ToF đã vào bridge/lost. Z lúc này là
    // dead-reckoning thuần accel và sai số tăng BẬC HAI —
    // xem ALT_EST_NO_CORRECTION_DEGRADED_MS. Thà hạ êm khi còn biết mình ở đâu
    // còn hơn bay tiếp bằng một con số đang trôi tự do.
    //
    // CHỈ tính khi ĐANG BAY: ở đất ground lock giữ Z=0 nên không cần correction
    // nào để tin. TAKING_OFF cũng tính — cất cánh mà mất cả hai nguồn thì huỷ
    // leo là đúng.
    if (airborne && in->alt_estimator_degraded) {
        r.fault = FAULT_SOFT;
        r.reason = "ToF correction mat qua lau -> Z dang troi tu do";
        return r;
    }

    return r;
}
