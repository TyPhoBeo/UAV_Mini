// tuning.h — TẤT CẢ hằng số tune của flight_core gom về MỘT chỗ (tương đương
// tuning.hpp của UAV-Mini gốc). Đây là file DUY NHẤT nên sửa khi tune PID/tốc
// độ/ngưỡng an toàn — KHÔNG sửa các #define rải rác trong attitude_control.h/
// alt_hold.h/takeoff_land.h/commander.h nữa, các file đó chỉ include lại đây.
//
// THANG "DUTY" = 0..MOTOR_SAFE_MAX_DUTY (motor_driver.h, hiện 11-bit -> 2000).
// MỌI hằng số dưới đây có ĐƠN VỊ DUTY (gain PID vòng RATE, ILIMIT/OUTLIM vòng
// RATE, ALT_HOLD_VZ_KP/KI/ILIMIT/HOVER_NOMINAL, TAKEOFF_PRIME_DUTY,
// LAND_MIN_THROTTLE/BLIND_DESCENT_RATE, ATT_MIN_THROTTLE_DUTY) đã được TÍNH
// SẴN theo đúng thang 2000 này — đổi độ phân giải LEDC (motor_driver.h) mà
// KHÔNG nhân/chia lại TOÀN BỘ các hằng số này theo đúng tỷ lệ thì hành vi bay
// sẽ đổi hẳn (PID phản ứng yếu/mạnh sai một nửa/gấp đôi). Hằng số vòng ANGLE
// (KP/KI/KD/ILIMIT/OUTLIM) + mọi thứ đơn vị deg/dps/m/m³/s KHÔNG phải thang
// duty, KHÔNG cần đổi khi đổi độ phân giải PWM.
//
// Sau khi đổi giá trị: build lại (giá trị nạp lúc *_default_*() gọi trong
// flight_core_start(), không đọc lại runtime — muốn chỉnh khi đang chạy thì
// dùng fc.set_param(), xem apply_set_param() trong flight_core.c, hiện chỉ hỗ
// trợ hover_duty).
//
// CẢNH BÁO: các giá trị dưới đây copy y hệt bản UAV-Mini đã bay/debug thật
// (ESP32 WROOM khác). Board S3 mới (motor/prop/pin/battery khác) PHẢI đo lại
// trước khi tin, đặc biệt: ALT_HOLD_HOVER_NOMINAL, TAKEOFF_PRIME_DUTY,
// COMMANDER_DEFAULT_BATTERY_FLOOR_V.
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// 1) ATTITUDE CASCADE (attitude_control.h) — angle PID -> rate PID -> mixer
// ============================================================================

// ---- Angle roll/pitch (outer loop: độ nghiêng -> rate mục tiêu) ----
#define ATT_GAIN_ANGLE_ROLL_KP      4.0f
#define ATT_GAIN_ANGLE_ROLL_KI      2.0f
#define ATT_GAIN_ANGLE_ROLL_KD      0.0f
#define ATT_GAIN_ANGLE_ROLL_ILIMIT  40.0f
#define ATT_GAIN_ANGLE_ROLL_OUTLIM  120.0f

#define ATT_GAIN_ANGLE_PITCH_KP     4.0f
#define ATT_GAIN_ANGLE_PITCH_KI     2.0f
#define ATT_GAIN_ANGLE_PITCH_KD     0.0f
#define ATT_GAIN_ANGLE_PITCH_ILIMIT 40.0f
#define ATT_GAIN_ANGLE_PITCH_OUTLIM 120.0f

// Heading-hold TẮT mặc định (kp=0 -> yaw thuần rate). Xem README phần yaw
// trước khi bật kp>0 — trôi do bias gyro-Z không sửa được nếu không có mag.
#define ATT_GAIN_ANGLE_YAW_KP       0.0f
#define ATT_GAIN_ANGLE_YAW_KI       0.0f
#define ATT_GAIN_ANGLE_YAW_KD       0.0f
#define ATT_GAIN_ANGLE_YAW_ILIMIT   30.0f
#define ATT_GAIN_ANGLE_YAW_OUTLIM   30.0f

// ---- Rate roll/pitch/yaw (inner loop: rate -> duty correction) ----
// KP/KI/KD/ILIMIT/OUTLIM đều map error(dps) -> DUTY -> đã nhân đôi theo thang
// 2000 (xem ghi chú "THANG DUTY" đầu file) — KHÔNG phải chỉ OUTLIM/ILIMIT.
#define ATT_GAIN_RATE_ROLL_KP       4.0f
#define ATT_GAIN_RATE_ROLL_KI       8.0f
#define ATT_GAIN_RATE_ROLL_KD       0.05f
#define ATT_GAIN_RATE_ROLL_ILIMIT   120.0f
#define ATT_GAIN_RATE_ROLL_OUTLIM   300.0f

#define ATT_GAIN_RATE_PITCH_KP      4.0f
#define ATT_GAIN_RATE_PITCH_KI      8.0f
#define ATT_GAIN_RATE_PITCH_KD      0.05f
#define ATT_GAIN_RATE_PITCH_ILIMIT  120.0f
#define ATT_GAIN_RATE_PITCH_OUTLIM  300.0f

#define ATT_GAIN_RATE_YAW_KP        5.0f
#define ATT_GAIN_RATE_YAW_KI        10.0f
#define ATT_GAIN_RATE_YAW_KD        0.05f
#define ATT_GAIN_RATE_YAW_ILIMIT    200.0f
#define ATT_GAIN_RATE_YAW_OUTLIM    300.0f

// ---- Dấu mixer Quad-X — đảo nếu lắp ngược board/motor, KHÔNG sửa gain ----
#define ATT_MIX_ROLL_SIGN           1.0f
#define ATT_MIX_PITCH_SIGN          1.0f
#define ATT_MIX_YAW_SIGN            1.0f

// Ga tối thiểu để PID attitude chạy (dưới ngưỡng: 4 motor quay đều, không PID).
#define ATT_MIN_THROTTLE_DUTY       200

// Nguong ga toi thieu de I-term attitude duoc cong don. Duoi muc nay motor
// chua du luc, cong I chi tao windup.
#define ATT_I_ENABLE_THROTTLE_DUTY  500

// Tran collective cho PID attitude con headroom. Vuot qua thi mixer bao hoa
// va drone mat kha nang giu thang bang o dung luc dang can nhat.
#define ATT_MAX_COLLECTIVE_FRACTION 0.85f

// D-term LPF cutoff (Hz), dùng chung mọi vòng PID: xem PID_D_LPF_HZ trong
// flight_core/pid.h (giữ ở đó vì thuộc về module pid.h thuần, không riêng
// attitude cascade).

// ============================================================================
// 2) ALT HOLD (alt_hold.h) — cascade giữ độ cao: alt -> vz_target -> vz-PI
// ============================================================================

#define ALT_HOLD_ALT_KP             1.0f     // alt_err -> vz_target [1/s]
// Cascade do cao: tang NGOAI (alt_kp) ra vz_target, tang TRONG (vz_kp/ki) ra
// throttle. vz_target bi kep boi ALT_HOLD_VZ_LIMIT_MS nen P chi dong gop toi
// da vz_kp*0.25 = 50 duty -- phan con lai deu den tu I.
#define ALT_HOLD_VZ_KP              200.0f
#define ALT_HOLD_VZ_KI              400.0f
#define ALT_HOLD_VZ_ILIMIT          500.0f
// Ga hover danh nghia. THR = hover + vz_kp*err + I, nen tran thuc te la
// hover + 50 + ALT_HOLD_VZ_ILIMIT = 1550 duty.
#define ALT_HOLD_HOVER_NOMINAL      1000.0f
#define ALT_HOLD_VZ_LIMIT_MS        0.25f   // trần |vz_target| (m/s)

// W/S doi ALT TARGET +/- buoc nay (KHONG phai lenh van toc). Cung buoc cho
// ca HOLDING lan FLYING de hai state hanh xu giong nhau.
#define ALT_HOLD_WS_VZ_MS           0.10f

// ---- FLYING -> HOLDING: PHAI YEN 1s MOI CHOT DO CAO ----
// Truoc day cạnh nay tuc thi: tha can la doi state NGAY, va do cao HOLD duoc
// chot bang tof_z_m cua DUNG tick do. Van de: luc vua tha can drone van dang
// nghieng va dang troi, ToF nhin xuong theo truc than nen so doc chua on —
// chot vao do cho ra target sai, co luc vot len ~2m.
//
// Cho 1s sau lenh dieu khien CUOI CUNG: du de attitude ve gan bang, propwash
// on lai, va ToF cho vai mau lien tiep on dinh. Chi khi do moi chot.
//
// ⚠ TRONG 1s CHO: van o FSM_FLYING, tuc PID do cao VAN KHONG chay (dung theo
// thiet ke da chot truoc day). Drone giu ga hien tai va troi tu nhien.
// 1000 -> 300ms (yeu cau nguoi dung). Danh doi ro rang: 300ms phan hoi nhanh
// hon han khi tha can, nhung it thoi gian cho attitude ve bang va ToF on lai,
// nen do cao chot duoc co the lech hon mot chut so voi 1s.
#define FLYING_TO_HOLD_SETTLE_MS    300

// FLYING chay tang TRONG cua cascade (vz -> throttle) voi vz_target = 0,
// thay vi dong bang throttle. Giu do cao khi nghieng bay ngang.
#define FLYING_VZ_HOLD_ENABLED      1

// I-limit rieng cho FLYING, chat hon HOLDING vi day chi la giu tam thoi.
#define FLYING_VZ_ILIMIT_DUTY       120.0f

// Nguon vz cua FLYING phai la accel-only: vz_ms da duoc ToF sua nen dung no
// se tao vong hoi tiep kin voi chinh cu nhay range.
#define FLYING_VZ_USE_ACCEL_ONLY    1

#define ALT_HOLD_TILT_GATE_DEG      30.0f   // không engage/giữ khi nghiêng quá
#define ALT_HOLD_MIN_ENGAGE_M       0.10f   // cao tối thiểu để engage HOLD (m)
#define ALT_HOLD_MIN_THROTTLE_DUTY  400     // sàn PID khi đang bay

// Bu cos(tilt) cho throttle: nghieng lam thanh phan thang dung cua luc day
// giam theo cos. Feedforward nay bu truoc, I-term lo phan con lai.
#define TILT_COMP_ENABLED           1
#define TILT_COMP_MAX_FACTOR        1.10f

// Duoi nguong nay coi nhu khong nghieng -> khong bu. Tranh rung nhe quanh 0 do
// bi khuech dai thanh nhieu tren duong ga.
#define TILT_COMP_MIN_COS           0.30f

// Expo cho lenh nghieng: phim nhe thi goc nho, de giu vi tri. Chi ap cho
// roll/pitch, KHONG ap cho UP/DOWN/yaw.
#define MOVE_TILT_EXPO              0.40f


// ============================================================================
// 3) TAKEOFF (takeoff_land.h) — PID + slew-rate-limited target:
//    PRIME -> CLIMB -> HOLD. Xem takeoff_land.h để biết semantics từng pha.
// ============================================================================
//
// NGUYÊN LÝ — VÌ SAO SLEW THAY VÌ RAM GA, VÀ VÌ SAO KHÔNG PID NGÂY THƠ:
//
//   (a) Ram ga cứng tới một duty cố định rồi "chốt hover" lúc bàn giao: leo mù,
//       không thích ứng pin/tải, và bàn giao xóc vì đổi nguồn throttle đột ngột.
//
//   (b) PID ngây thơ (target = đích NGAY khi còn nằm đất): error khổng lồ ngay
//       tick đầu -> I windup lúc chưa nhấc -> drone PHÓNG lên. TUYỆT ĐỐI TRÁNH.
//
//   (c) PID + slew (bản này): target TRƯỢT DẦN từ độ cao HIỆN TẠI lên đích với
//       tốc độ bị giới hạn -> error luôn nhỏ -> P không sốc, I không windup ->
//       leo mượt có kiểm soát, tự vào hold. KHÔNG cần bước bàn giao, KHÔNG cần
//       chốt hover: hover THẬT do thành phần I của vòng Vz TỰ HỌC dọc đường.
//
// Lực đẩy cuối = hover_ff (feedforward THÔ) + Vz-PID(P + I). I chính là phần
// học hover thật. hover_ff KHÔNG cần chính xác — sai bao nhiêu thì I bù bấy
// nhiêu, chỉ là hội tụ nhanh hay chậm.

// PRIME_DUTY phai THAP HON HAN hover: no chi de motor quay deu truoc khi vao
// CLIMB. Bang hover thi drone nhac len ngay trong PRIME, mat ca pha chuan bi.
#define TAKEOFF_PRIME_HOVER_FRAC    0.60f

// Giá trị KHỞI TẠO của prime_duty (takeoff_default_tune()). Chỉ có tác dụng
// khi latch theo pin KHÔNG chạy (HOVER_LATCH_ENABLED=0, hoặc trước lần ARM đầu
// tiên) — ARM thành công sẽ GHI ĐÈ s_tko_tune.prime_duty bằng giá trị suy ra
// từ hover thật đo theo pin. Xem hover_model.h.
#define TAKEOFF_PRIME_DUTY          ((int)(ALT_HOLD_HOVER_NOMINAL * TAKEOFF_PRIME_HOVER_FRAC))
#define TAKEOFF_PRIME_MS            1000

// hover_ff (feedforward thô) KHÔNG định nghĩa riêng ở đây — nó LÀ
// ALT_HOLD_HOVER_NOMINAL (mục 2). Một đại lượng vật lý -> một hằng số. Định
// nghĩa thêm "TAKEOFF_HOVER_GUESS" sẽ tạo hai số cho cùng một thứ và bảo đảm
// có ngày chúng lệch nhau -> bước nhảy ga đúng lúc bàn giao.

// Tran toc do leo cua chuoi cat canh. ⚠ Dat THAP hon toc do leo tu nhien se
// lam PID ra lenh HA trong khi drone dang leo -- da do duoc voi gia tri cu
// 0.15 m/s. 0.35 cao hon toc do leo thuc te nen PID khong con chong lai.
#define TAKEOFF_MAX_CLIMB_MS        0.35f

// Roi dat = BA bang chung (Z vuot nguong, vz duong, ga da qua hover) giu
// lien tuc TAKEOFF_LIFTOFF_MS. Mot bang chung don le KHONG duoc quyet dinh.
#define TAKEOFF_LIFTOFF_MS          200
// Độ cao ToF tối thiểu để tính là "đã rời đất". HẠ 0.04 -> 0.01 (yêu cầu người
// dùng, cùng đợt với ALT_EST_TOF_MIN_RANGE_M): 4cm là mức mà drone phải nhấc
// khá cao mới đạt, trong khi ta cần bằng chứng NGAY khi vừa rời sàn.
//
// ⚠ ĐI KÈM RÀNG BUỘC: vế này chỉ có nghĩa khi ALT_EST_TOF_MIN_RANGE_M <= giá
// trị này — gate hình học của estimator loại mẫu dưới MIN_RANGE trước cả khi
// tof_z_m được tính, nên đặt LIFTOFF_Z thấp hơn MIN_RANGE là tạo một vế KHÔNG
// BAO GIỜ đúng. Cả hai giờ đều là 0.01.
#define TAKEOFF_LIFTOFF_Z_M         0.01f
// ⚠ KHÔNG CÒN ĐƯỢC DÙNG trong lift_evidence — xem khối giải thích ở
// takeoff_land.c. Vz đạo hàm từ ToF 30Hz ở tốc độ leo chậm chạm 0 liên tục
// ngay giữa lúc bay bình thường, và vì lift_evidence phải đúng LIÊN TỤC nên
// một mẫu rớt là reset cả cửa sổ -> liftoff không bao giờ xác nhận được.
// Giữ macro để không vỡ build/tham chiếu cũ; ĐỪNG đưa lại vào điều kiện AND.
#define TAKEOFF_LIFTOFF_VZ_MS       0.05f
#define TAKEOFF_LIFTOFF_THR_FRAC    0.75f
#define TAKEOFF_NO_LIFT_TIMEOUT_MS  5000
// Độ cao ToF tối thiểu phải đạt sau TAKEOFF_NO_LIFT_TIMEOUT_MS trong CLIMB.
// Thấp hơn mức này -> TKO_ABORT_NO_LIFT_EVIDENCE (không nhấc nổi).
//
// ⚠ ĐÂY LÀ ĐIỀU KIỆN HỦY BAY, KHÔNG PHẢI ĐIỀU KIỆN "đã rời đất". Nó CỐ Ý THÔ
// (một vế, chỉ độ cao) và CỐ Ý cao hơn nhiều so với TAKEOFF_LIFTOFF_Z_M (0.01):
//   - 0.01 = "có dấu hiệu rời sàn" -> đủ để nới trần I, mở Ki attitude.
//   - 0.20 = "đã đi lên THẬT SỰ"   -> dưới mức này sau 5s là không nhấc nổi.
// Đặt bằng nhau là bỏ mất ý nghĩa của cả hai: 0.01 quá dễ đạt để kết luận
// chuyến bay ổn, còn 0.20 quá khắt khe để mở Ki.
//
// CHỌN SỐ: phải cao hơn hẳn nhiễu ToF sát đất (±2-3cm) và cao hơn mức drone có
// thể "nhích lên rồi đứng" khi quá tải. 0.20 cách nhiễu ~7 lần.
#define TAKEOFF_NO_LIFT_ALT_M       0.20f
// Ngưỡng thực tế = min(TAKEOFF_NO_LIFT_ALT_M, target × FRAC). BẮT BUỘC phải có
// vế thứ hai: target thấp hơn 0.20m (fc.takeoff(150)) sẽ bay ĐÚNG tới đích rồi
// vẫn bị hủy vì chưa chạm ngưỡng cứng — hủy một chuyến bay hoàn hảo với lý do
// ghi là "không nhấc nổi", đúng loại lỗi khiến người dùng đi kiểm cánh quạt và
// pin trong khi phần cứng không có vấn đề gì.
// 0.5 = "đi được nửa đường tới đích sau 5s" — chậm thì chấp nhận, đứng im thì không.
#define TAKEOFF_NO_LIFT_TARGET_FRAC 0.5f
#define TAKEOFF_DEFAULT_TARGET_M    0.5f

// ---- 3d) VÀO HOLDING — CŨNG THEO THỜI GIAN ----
// Điều kiện DUY NHẤT: target_z đã trượt tới đích, VÀ giữ như vậy đủ
// TAKEOFF_HOLD_ENTER_MS. KHÔNG còn kiểm est_z.
//
// `target_z == final_target` KHÔNG phải phép đo — nó là trạng thái của chính
// bộ rate-limiter trong firmware, hoàn toàn xác định theo thời gian
// (final_target / max_climb giây). Nên toàn bộ điều kiện bàn giao giờ là một
// hàm của đồng hồ, không có cảm biến nào tham gia.
//
// Chuyển CLIMB -> HOLDING vẫn LIỀN MẠCH: cùng cascade, cùng alt_hold_state_t,
// KHÔNG reset I, KHÔNG đổi nguồn throttle. Chỉ là target_z ngừng trượt.
//
// Tổng thời gian chuỗi = PRIME_MS + (final_target / MAX_CLIMB_MS) + HOLD_ENTER_MS.
// Với target 1.0m, MAX_CLIMB 0.1 m/s: 500ms + 10.0s + 800ms = 11.3s.
// ⚠ PHẢI nhỏ hơn TAKEOFF_TOTAL_TIMEOUT_MS, nếu không mọi chuyến đều abort.
// Cửa sổ DUY TRÌ của hold_ready. HẠ 1000 -> 400ms: cả 4 vế phải đúng LIÊN TỤC
// bấy nhiêu, và mỗi mẫu rớt là reset về 0. Cửa sổ càng dài thì xác suất "không
// bao giờ đóng được" càng cao khi có nhiễu — đúng cơ chế đã giết lift_evidence.
#define TAKEOFF_HOLD_ENTER_MS       400
// Dung sai độ cao để bàn giao. NỚI 0.03 -> 0.08.
//
// ⚠ ĐO ĐƯỢC TRÊN PHẦN CỨNG THẬT (log t=7.7s): target 0.20m, drone ổn định ở
// Z=0.29m — lệch 9cm, gấp 3 lần dung sai cũ. Với 0.03 thì vế này KHÔNG BAO GIỜ
// đúng, hold_ready mãi false, chuỗi chạy tới hết deadline rồi abort TIMEOUT dù
// drone đã bay lên và giữ độ cao hoàn toàn ổn định.
//
// 9cm sai lệch đó là chuyện RIÊNG của vòng điều khiển (hover_ff lệch, I chưa
// hội tụ, ground effect) và HOLD sẽ tự kéo về sau khi bàn giao — nó KHÔNG phải
// lý do để hủy chuyến bay. Dung sai bàn giao chỉ cần đủ chặt để biết drone
// "đang ở gần đích và không còn lao đi", không phải để ép độ chính xác cuối.
// ⚠ HAI HẰNG SỐ DƯỚI ĐÂY GIỜ KHÔNG CÒN AI ĐỌC (yêu cầu người dùng).
// hold_ready đã rút còn HAI vế: rời đất + rate-limiter trượt hết. Hai dung sai
// này từng là vế thứ ba/tư, và chúng chính là nguyên nhân một lần cất cánh thật
// bị kẹt ở CLIMB 21.6s rồi TKO_ABORT_TIMEOUT: drone leo tới 1.45m trong khi
// target 1.00m nên |Z-tgt| KHÔNG BAO GIỜ <= 0.08.
// GIỮ LẠI định nghĩa (không xoá) để lịch sử tune còn đọc được và để bật lại
// bằng một dòng nếu sau này muốn siết bàn giao. Xem takeoff_land.c `hold_ready`.
#define TAKEOFF_HOLD_Z_TOL_M        0.10f
// Dung sai Vz. NỚI 0.08 -> 0.20: Vz ước lượng dao động ±0.25 m/s ngay cả khi
// drone treo ổn định (ToF 30Hz + propwash), nên 0.08 là dưới mức nhiễu nền —
// cùng loại lỗi với vế tof_vz đã phải bỏ khỏi lift_evidence.
#define TAKEOFF_HOLD_VZ_TOL_MS      0.10f
// Tran thoi gian cho ve "|vz| <= TAKEOFF_HOLD_VZ_TOL_MS" o hold_ready.
// Het han thi ban giao DU vz chua lang.
//
// ⚠ VE NAY BAT BUOC PHAI CO. Dieu kien do cao cu (|alt-target| <= TOL) tung
// lam mot lan cat canh ket 21.6s roi TIMEOUT vi no CO THE khong bao gio dung.
// Ve vz thi ve nguyen ly luon dung sau khi target ngung truot — nhung "ve
// nguyen ly" khong phai bao dam. 1.5s: du cho mot cu vot lo binh thuong lang
// xuong, va ngan hon nhieu so voi TAKEOFF_TOTAL_TIMEOUT_MS.
#define TAKEOFF_VZ_SETTLE_TIMEOUT_MS  1500
// Tran toc do TANG ga cua guard khoang ho. Chan cu nhay +900 duty trong mot
// tick. ⚠ Phai tich luy PHAN LE, ep (int) moi tick se lam ga khong bao gio tang.
#define THROTTLE_MAX_RISE_DUTY_PER_S   500.0f

// Chuoi cat canh KET: da het TAKEOFF_STUCK_MS ma van chua roi dat -> ABORT.
// Khong co cai nay thi drone nam ru ga o san cho toi khi het pin.
#define TAKEOFF_STUCK_MS                 5000


// (3) Mất nguồn đo độ cao (ToF) liên tục bao lâu thì abort. Trong cửa sổ này
//     GIỮ NGUYÊN throttle cuối (không chạy cascade trên est_z rác), quá hạn thì
//     EMERGENCY.
#define TAKEOFF_TOF_LOST_MS              1000       // spec: 300ms
// (4) Nghiêng quá ngưỡng lớn = sắp lật. Có cửa sổ duy trì ngắn để một mẫu
//     Mahony lỗi không tự abort một chuyến bay đang bình thường.
#define TAKEOFF_ABORT_TILT_DEG           45.0f
#define TAKEOFF_ABORT_TILT_MS            60

// ---- 3f) TRẦN COLLECTIVE + trần I trước liftoff ----
// Trần collective mà takeoff được phép đẩy tới. TÁI SỬ DỤNG
// ATT_MAX_COLLECTIVE_FRACTION: vượt qua đó thì mixer không còn dải tạo mô-men
// roll/pitch/yaw -> mất thẩm quyền attitude, nguy hiểm hơn hẳn việc leo chậm.
#define TAKEOFF_COLLECTIVE_CEILING_FRAC  ATT_MAX_COLLECTIVE_FRACTION

// Trần |I| của vòng Vz TRƯỚC khi liftoff.
//
// VÌ SAO VẪN CẦN dù slew đã chống windup: slew làm error NHỎ, không làm nó
// bằng 0. Khi drone bị giữ lại (kẹt cánh/quá tải) target vẫn trượt lên đều nên
// error tích luỹ dai dẳng theo một chiều — I sẽ bò lên mãi tới khi TIMEOUT nổ
// ở 3s. Trần này chặn phần bò đó lại.
//
// GIỚI HẠN, KHÔNG ĐÓNG BĂNG — đây là bản sửa một lỗi thiết kế đã đo bằng test:
// freeze hẳn I trước liftoff làm collective kẹt ở hover + VZ_KP*MAX_CLIMB, nên
// (a) drone không có quyền tìm điểm nhấc nếu hover thật cao hơn ước lượng,
// (b) không bao giờ chạm trần nên fault "không nhấc nổi" bị báo sai loại.
// 300 duty = biên mà bản ram-ga cũ dùng (spool 1600 - hover 1300), tức con số
// đã được xác nhận thực tế là "đủ để nhấc".
#define TAKEOFF_PRELIFT_I_LIMIT_DUTY     300.0f

// ============================================================================
// 4) LANDING (takeoff_land.h) — descend -> flare -> touchdown (+ blind nếu mất ToF)
// ============================================================================

// Chuoi ha canh: DESCEND (vz co dinh) -> FLARE (giam dan duoi flare_alt)
// -> CONTACT_CANDIDATE (4 bang chung) -> TOUCHDOWN (ramp ga ve 0).
#define LAND_DESCENT_VZ             0.25f    // m/s, tốc độ hạ pha DESCEND
#define LAND_FLARE_ALT_M            0.50f    // m, ngưỡng vào FLARE
#define LAND_FLARE_VZ                0.12f   // m/s, tốc độ hạ lúc gần chạm
#define LAND_TOUCHDOWN_ALT_M        0.080f   // m, ToF height tren floor
// NÂNG 0.035 -> 0.080: ở 3.5cm, VL53L1X đã nằm sát cận dưới đáng tin của nó
// (crosstalk cửa sổ kính, min-range-fail) nên số đo bắt đầu nhảy đúng lúc cần
// nó ổn định nhất. 8cm vẫn đủ thấp để cú tiếp đất còn lại là rơi tự do vài cm,
// mà nằm hẳn trong vùng ToF đọc tin cậy.

// ---- TOUCHDOWN DETECTOR ĐA ĐIỀU KIỆN (xem landing_run()) ----
// Board KHÔNG có ToF, Z đến từ baro với nhiễu ±0.3-1m sát đất — lớn gấp nhiều
// lần LAND_TOUCHDOWN_ALT_M. Dùng mình nó để cắt máy là cách chắc chắn nhất để
// rơi giữa không trung khi baro tụt một nhịp. Nên chấm điểm 4 bằng chứng +
// yêu cầu duy trì, và có pha CONTACT_CANDIDATE (VẪN giữ điều khiển) ở giữa.
#define LAND_CONTACT_THR_MARGIN     40       // duty trên LAND_MIN_THROTTLE vẫn coi là "sát sàn"
#define LAND_CONTACT_VZ_MS          0.15f
#define LAND_CONTACT_ALT_MARGIN_M   0.10f    // m, cộng vào touchdown_alt_m cho bằng chứng "Z thấp"
#define LAND_CONTACT_Z_PROGRESS_M   0.03f    // m, Z giảm ÍT HƠN mức này = "đã bị chặn"
#define LAND_CONTACT_SCORE_MIN      3         // /4 điểm mới vào CONTACT_CANDIDATE
#define LAND_CONTACT_TICKS          38        // ~150ms @250Hz
#define LAND_SETTLE_MS              300      // ga thấp + vz~0 giữ liên tục -> touchdown backup
#define LAND_MIN_THROTTLE           400      // thang duty 2000, xem đầu file
#define LAND_TOF_TIMEOUT_MS         300
#define LAND_BLIND_DESCENT_RATE     200.0f   // duty/giây, pha BLIND (thang duty 2000)
#define LAND_CUTOFF_MS              100      // ramp ga về 0 lúc TOUCHDOWN

// Nhanh 2 cua touchdown: ga da tut kich san VA vz lang, giu LAND_SETTLE_MS.
// KHONG dung do cao vi ToF o cu ly rat gan doc khong tin duoc.
#define LAND_SETTLE_VZ_MS           0.05f    // m/s, nhánh 2 (spec: |vz| < 0.05)
#define LAND_TOUCHDOWN_AZ_MS2       (-6.0f)  // m/s², nhánh 3 — xem cảnh báo dấu ở trên
#define LAND_TOUCHDOWN_AZ_HOLD_MS   60       // ms, spike phải DUY TRÌ bấy nhiêu

// ---- Mất ToF khi đang HOLD -> degrade accel-hold rồi mới land (spec D4) ----
// Trong cửa sổ này alt_hold KHÔNG bị coi là engage_lost: cascade chạy tiếp với
// vz_target = 0 trên vz tích phân từ accel, I-term FREEZE (không sạc bằng số
// liệu chết). Hết cửa sổ -> nhả engage_lost -> Commander soft fault -> LANDING
// (nhánh BLIND của landing_run tiếp quản). KHÔNG để HOLD chạy tiếp vô hạn với
// số liệu chết, cũng KHÔNG cắt phăng ngay mẫu đầu tiên bị mất.
#define ALT_HOLD_TOF_DEGRADE_MS     600

// ============================================================================
// 5) COMMANDER (commander.h) — geofence + ngưỡng fault (SOFT -> LANDING,
//    HARD -> EMERGENCY)
// ============================================================================

#define COMMANDER_DEFAULT_ALT_MIN_M          0.0f
#define COMMANDER_DEFAULT_ALT_MAX_M          ALT_EST_MAX_FLIGHT_Z_M
// 1S LiPo (full 4.2V, nominal 3.7V, KHÔNG BAO GIỜ để dưới 3.0V — hại cell
// vĩnh viễn). Floor 3.3V chừa margin cho sụt áp dưới tải trước khi chạm đáy
// tuyệt đối — hạ/land khi chạm ngưỡng này, đừng đợi tới 3.0V. XÁC NHẬN LẠI
// bằng đo thật dưới tải (không chỉ đo hở mạch) trước khi bay.
// San dien ap pin -> SOFT FAULT -> LANDING. Pin 1S duoi tai: 3.10V la vung
// sut sau, duoi do khong con du luc day de giu do cao.
#define COMMANDER_DEFAULT_BATTERY_FLOOR_V    3.10f
// So sanh voi TRUNG VI 5 mau (xem battery_v o flight_core buoc 2), khong phai
// mau tho -- trung vi da phang, debounce chi con de chan mot chum doc hong.
#define COMMANDER_BATTERY_LOW_HOLD_MS        1000
#define COMMANDER_DEFAULT_HEARTBEAT_MS       1000
#define COMMANDER_DEFAULT_HARD_TILT_DEG      60.0f
#define COMMANDER_DEFAULT_MOTOR_SAT_MS       1500

// ============================================================================
// 6) GROUND-STATION SETPOINT (flight_core.c CMD_SET_ATTITUDE/CMD_SET_TRIM +
//    command_parser.c — DÙNG CHUNG để reply "OK <giá trị>" khớp ĐÚNG giá trị
//    flight_core.c sẽ áp dụng, không lệch giữa 2 nơi clamp)
// ============================================================================

#define SP_TILT_MAX_DEG        30.0f    // clamp roll/pitch target SAU khi cộng trim
#define SP_YAW_RATE_MAX_DPS    180.0f
#define TRIM_MAX_DEG            10.0f    // clamp riêng trim (khớp slider GUI UAV-Mini -10..10)

// Trim roll/pitch mac dinh. Gia tri THAT den tu NVS (calibration.c) -- day
// chi la moc khi NVS trong.
#define TRIM_ROLL_DEG_DEFAULT    (0.85f)
#define TRIM_PITCH_DEG_DEFAULT   (1.15f)

// Canh lúc BIÊN DỊCH: mặc định phải nằm trong dải mà runtime chấp nhận. Đường
// CMD_SET_TRIM có clampf(), còn khởi tạo tĩnh thì KHÔNG — thiếu dòng này thì
// một giá trị mặc định ngoài dải sẽ lọt thẳng vào target mà không ai chặn.
_Static_assert(TRIM_ROLL_DEG_DEFAULT >= -TRIM_MAX_DEG &&
               TRIM_ROLL_DEG_DEFAULT <= TRIM_MAX_DEG,   
               "TRIM_ROLL_DEG_DEFAULT vuot TRIM_MAX_DEG");
_Static_assert(TRIM_PITCH_DEG_DEFAULT >= -TRIM_MAX_DEG &&
               TRIM_PITCH_DEG_DEFAULT <= TRIM_MAX_DEG,
               "TRIM_PITCH_DEG_DEFAULT vuot TRIM_MAX_DEG");
// Mất CMD_SET_ATTITUDE mới quá lâu -> flight_core.c tự zero roll/pitch/yaw_rate
// (watchdog RIÊNG cho setpoint bay tay, nhanh hơn Commander heartbeat 1000ms).
#define SP_STALE_TIMEOUT_US     (int64_t)400000   // 400ms

// ============================================================================
// 7) MICROPYTHON fc.control() — joystick angle-mode (rol/pit/yaw/thr, mỗi trục
//    -100..100). DÙNG CHUNG kho lưu s_sp_roll_deg/pitch/yaw_rate_dps + watchdog
//    stale với ground-station SP (mục 6) — CHỈ MỘT nguồn setpoint tay lái tại
//    một thời điểm, xem flight_core.c CMD_CONTROL. 2 hằng số dưới đây CHỈ là
//    default nạp lúc flight_core_start() vào 2 biến runtime s_max_lean_deg/
//    s_max_yawrate_dps — tune lại lúc chạy qua fc.set_param("max_lean_deg"/
//    "max_yawrate_dps", ...), KHÁC SP_TILT_MAX_DEG/SP_YAW_RATE_MAX_DPS (mục 6)
//    là hằng số cứng riêng cho ground-station UDP, không đổi được runtime.
// ============================================================================
#define CONTROL_MAX_LEAN_DEG_DEFAULT      28.0f
#define CONTROL_MAX_YAWRATE_DPS_DEFAULT   180.0f
// thr=+-100% -> +-CONTROL_ALT_SLEW_MPS m/s SLEW target altitude (KHÔNG step) —
// chỉ áp dụng khi HOLDING/FLYING (alt_hold đang chạy), xem flight_core.c step 4b.
#define CONTROL_ALT_SLEW_MPS              0.6f

// ---- Offset throttle của phím GIỮ (W/S ở GUI) khi ĐANG BAY -----------------
// GUI gửi `@THR OFFSET ±100` (WS_THROTTLE_OFFSET_DUTY trong
// tools/uav_udp_console.py). Ở MỌI state dùng được, offset này cộng THẲNG vào
// duty — W/S là cần ga thật, không phải lệnh đổi độ cao.
//
// KHÔNG có hằng số tỉ lệ nào ở đây nữa (trước có MANUAL_THR_OFFSET_FULLSCALE_DUTY
// để quy đổi offset -> tốc độ trượt target độ cao; đã bỏ cùng với cách làm đó).
//
// ⚠ Ở HOLDING/FLYING việc cộng thẳng KHÔNG tự nó đủ, và đây là lý do:
// alt_hold là cascade Z->Vz chạy mỗi 4ms và nó SỞ HỮU throttle. Chỉ cộng +100
// vào output của nó thì drone nhích lên, PID thấy Z vượt target rồi TỰ HẠ GA để
// bù — sau vài trăm ms thành phần I đã trừ đi đúng bằng offset, ga thật quay về
// y như cũ ("giữ W mà không lên"), và nhả phím ra thì I âm còn đọng lại làm
// drone TỤT.
//
// Nên nhánh HOLDING/FLYING trong flight_core.c làm ĐỒNG THỜI ba việc, thiếu một
// việc là hỏng (xem TEST 7..12 trong python/test_ws_throttle_offline.py):
//   1. ĐÓNG BĂNG vz_integral trong lúc còn giữ phím -> I không chống lại người lái
//   2. NEO alt_target theo Z hiện tại -> nhả phím là giữ NGAY tại chỗ đang ở,
//      không giật về độ cao cũ  (vẫn qua commander_clamp_altitude -> geofence
//      KHÔNG bị offset vô hiệu hoá)
//   3. Cộng offset vào duty, kẹp trong [ALT_HOLD_MIN_THROTTLE_DUTY, SAFE_MAX]
//
// Thành phần P thì VẪN chống lại (p = vz_kp * (0 - vz)) và đó là CHỦ ĐÍCH: nó
// là bộ giảm chấn vận tốc, giữ cho "giữ W" không thành gia tốc chạy trốn.
//
// Ở FSM_BENCH_RAMP đơn giản hơn: không có PID độ cao nào sở hữu throttle nên
// chỉ cần cộng thẳng, không cần ba việc trên (drone kẹp trên giá, đo lực nâng).

// ============================================================================
// 8) CALIBRATION (calibration.h/.c + flight_core.c CMD_CALIB_*) — gyro/accel/
//    mag, chỉ chạy khi DISARMED, persist qua NVS. Xem calibration.h.
// ============================================================================
#define CALIB_ACCEL_FACE_SAMPLES     125      // ~0.5s @ 250Hz mỗi mặt (6 mặt)
#define CALIB_ACCEL_FACES_NEEDED     6
#define CALIB_ACCEL_MIN_RANGE_G      1.0f     // mỗi trục PHẢI thấy đổi >= ngưỡng này giữa các mặt
                                                // (kỳ vọng ~2g nếu làm đúng 6-face) - dưới ngưỡng
                                                // = nghi làm sai quy trình, KHÔNG lưu
#define CALIB_MAG_MIN_SAMPLES        200       // ~2s @ ODR 100Hz - sàn chống bấm STOP ngay sau START
// CMD_CALIB_MAG_START giờ TỰ ĐỘNG chạy đúng khoảng này rồi tự tính kết quả
// (không cần gọi STOP) — xem flight_core.c finalize_mag_calibration(). Vẫn
// gọi CMD_CALIB_MAG_STOP được để kết thúc SỚM (tính ngay với mẫu đã có).
#define CALIB_MAG_DURATION_MS        60000     // 60s — XOAY drone liên tục suốt khoảng này
// Chu kỳ in tiến độ ra log trong lúc calib mag (số mẫu + min/max/range từng
// trục) — để biết NGAY đang xoay đủ hay không, không phải chờ hết 60s mới
// biết hỏng. Xem flight_core.c step 1b.
#define CALIB_MAG_PROGRESS_MS        5000      // 5s/lần
// Chất lượng phủ 3 trục: range trục nhỏ nhất / trục lớn nhất. Dưới ngưỡng này
// = có trục xoay thiếu -> soft_iron méo. CẢNH BÁO thôi, VẪN lưu (khác
// CALIB_ACCEL_MIN_RANGE_G là điều kiện CỨNG) — mag còn tuỳ môi trường từ
// trường xung quanh, chưa đủ cơ sở để từ chối cứng trên phần cứng này.
#define CALIB_MAG_MIN_RANGE_RATIO    0.5f
// Gyro/accel: chỉ cộng vào tổng những tick imu.ok==true (đọc I2C thành công)
// — KHÔNG cộng dồn mẫu lỗi (bằng 0 do imu_driver_read() fallback) vào trung
// bình, tránh kéo kết quả lệch về 0 nếu bus/driver lỗi giữa chừng lúc calib.
// Dưới sàn này (tỷ lệ mẫu hợp lệ / tổng tick cửa sổ) -> HỦY, không lưu.
#define CALIB_MIN_VALID_FRACTION     0.5f

// Accel six-face vẫn dùng tỷ lệ mẫu nghi động riêng. Gyro dùng toàn bộ nhóm
// GYRO_CAL_* bên dưới và không còn đường calibration thứ hai.
#define CALIB_MOTION_MAX_BAD_FRACTION   0.05f   // >5% tick "nghi động" trong cửa sổ -> HỦY (CALIB_MOVING)
#define CALIB_ACCEL_MOTION_GYRO_DPS     5.0f    // |gyro| vượt 1 mẫu lúc bắt 1 mặt accel -> tick đó "nghi động"

// ---- Verify sau khi tính bias/scale accel 6-face (xem "13. Accel calibration
// verification") — áp lại công thức correction lên chính 6 mặt vừa đo, mặt
// nào |accel_hiệu_chỉnh| lệch 1.0g quá ngưỡng này -> HỦY toàn bộ, không lưu.
// CHƯA đo trên phần cứng thật, điểm khởi đầu. ----
#define CALIB_ACCEL_MAX_RESIDUAL_G      0.15f

// ============================================================================
// 8b) GYRO CALIBRATION FSM — dùng chung cho startup và CAL GYRO.
//
// VI SAO tu dong moi boot: gyro zero-rate bias cua MPU6050 troi theo NHIET DO
// va theo tung lan cap nguon. Mot gia tri luu tu buoi truoc, o nhiet do khac,
// KHONG con dung — va trieu chung duy nhat la yaw troi cham, thu rat de do
// nham cho PID hoac cho mixer. Do lai moi lan boot loai bo han ca lop loi do.
//
// NVS chỉ lưu history/backup để chẩn đoán. Mỗi boot vẫn bắt buộc đo fresh;
// fresh calibration fail thì gyro_valid=false và ARM bị chặn, tuyệt đối không
// silently bay bằng bias NVS cũ.
// ============================================================================

// Chờ cảm biến ỔN ĐỊNH sau khi cấp nguồn/config xong rồi mới lấy mẫu. MPU6050
// vừa qua DEVICE_RESET + đổi clock source sang PLL X-gyro: những mẫu đầu tiên
// còn mang transient của chính quá trình đó, gom vào trung bình sẽ đẩy bias
// lệch một lượng nhỏ nhưng CỐ ĐỊNH — đúng loại sai số không ai truy ra được.
#define GYRO_CAL_SETTLE_MS                    1500

// ĐÃ BỎ GYRO_CAL_WAIT_STATIONARY_MS / GYRO_CAL_STATIONARY_CONFIRM_MS.
// Cổng "đợi chuỗi mẫu stationary LIÊN TỤC" trước khi được phép thu đã bị gỡ:
// nó test |gyro_raw| mà gyro_raw ĐÃ MANG SẴN BIAS, nên bo có bias lớn thì
// KHÔNG BAO GIỜ qua nổi cổng — tức là càng cần calib thì càng không calib được.
// Chi tiết đầy đủ ở enum gyro_cal_state_t trong flight_core.c.
// Giờ: settle xong là COLLECT thẳng, phán xét bằng thống kê CẢ cửa sổ.

// Sensor hub đã gom 4 mẫu MPU6050 1kHz thành 250Hz; 3s ≈ 750 mẫu điều khiển,
// Welford không cần giữ toàn bộ mẫu trong RAM.
#define GYRO_CAL_DURATION_MS                   3000

// Cửa sổ VALIDATE (mẫu ĐỘC LẬP, thu SAU khi bias đã áp). Ngắn hơn cửa sổ chính
// vì chỉ cần xác nhận trung bình ~0, không cần độ chính xác để ước lượng.
#define GYRO_CAL_VALIDATION_MS                 1500

// ĐÃ BỎ GYRO_CAL_MAX_ATTEMPTS — không còn retry tự động. Nguyên nhân trượt
// (drone bị cầm, rung cơ khí) không tự thay đổi trong vài giây, nên 3 lần thử
// chỉ kéo dài 15s rồi báo cùng một lỗi. Trượt -> nói rõ lý do -> người dùng đặt
// lại drone rồi gõ `calib_gyro`.
#define GYRO_CAL_MIN_VALID_FRACTION             0.80f

// Tỉ lệ mẫu "nghi động" TỐI ĐA cho phép trong một cửa sổ. Thay cho việc cắt
// ngang ngay khi gặp MỘT mẫu xấu — cách cũ biến mọi nhiễu lẻ tẻ (một cú gõ bàn,
// một spike ADC) thành một lần calib hỏng.
// 2% của 750 mẫu = 15 mẫu; đủ rộng cho nhiễu lẻ, đủ hẹp để một bàn tay chạm vào
// drone giữa chừng vẫn bị bắt.
#define GYRO_CAL_MAX_BAD_FRACTION               0.02f

// ---- Ngưỡng phát hiện ĐỨNG YÊN (dùng CẢ gyro LẪN accel, mục 5) ----
// Chặt hơn bộ CALIB_* thủ công ở trên vì boot calib có quyền thử lại nhiều lần
// và không có người đang đứng chờ.

// |accel_norm - 1g| tức thời vượt ngưỡng -> mẫu đó "nghi động".
#define GYRO_CAL_ACCEL_NORM_TOL_G               0.05f

// |gyro| tức thời (độ lớn vector) vượt ngưỡng -> mẫu đó "nghi động". Bias thật
// có thể tới vài dps nên ngưỡng phải CAO HƠN bias kỳ vọng, nếu không một board
// có bias lớn sẽ không bao giờ calib được.
#define GYRO_CAL_RAW_NORM_MAX_DPS                5.0f

// std-dev từng trục trong cửa sổ. Đây là thước đo "có rung/có người cầm không"
// — độc lập với độ lớn bias (bias là DC, std là AC).
#define GYRO_CAL_RAW_STD_MAX_DPS                 0.25f

// accel std-dev từng trục — bắt rung cơ khí mà gyro có thể bỏ sót.
#define GYRO_CAL_ACCEL_STD_MAX_G                 0.02f

// Sanity bound rộng: bias vài dps của MPU6050 là bình thường; chỉ loại giá trị
// rõ ràng phi vật lý/hỏng phần cứng.
#define GYRO_CAL_BIAS_MAX_ABS_DPS                20.0f

// ---- Ngưỡng VALIDATE sau calib (mục 7) ----
// |mean corrected| từng trục phải dưới ngưỡng này. Đây chính là con số quyết
// định "calib có thật sự khử được bias không" — nếu GRAW=[0.08,-2.78,-1.94] và
// GBIAS khớp, thì GCORR phải ≈ 0 và mean phải lọt dưới đây.
#define GYRO_CAL_VALIDATION_MAX_MEAN_DPS          0.15f

// std corrected trong cửa sổ validate — vượt = có chuyển động lúc validate,
// kết quả validate không tin được (khác hẳn "bias sai").
#define GYRO_CAL_VALIDATION_MAX_STD_DPS           0.25f

// ---- Pre-arm gyro health (mục 18) ----
// Kiểm tra LIÊN TỤC khi DISARMED: trung bình trượt của gyro corrected. Vượt
// ngưỡng trong một cửa sổ đủ dài -> chặn ARM. Lỏng hơn ngưỡng validate vì
// drone lúc chờ arm có thể bị chạm nhẹ; đây là lưới an toàn cuối, không phải
// phép đo chính xác.
// NỚI 0.30 -> 1.00 dps theo yêu cầu người dùng.
//
// ⚠ ĐÂY LÀ LƯỚI AN TOÀN DUY NHẤT CÒN LẠI cho gyro bias, sau khi pha VALIDATE
// bị bỏ và bias NVS trở thành authoritative (calib một lần, boot sau chỉ nạp).
// Không còn phép thử tự động nào khác chứng minh "trừ bias xong thì còn ~0" ở
// THỜI ĐIỂM cất cánh — calib PASS chỉ chứng minh điều đó cho quá khứ.
//
// 1.0 dps bắt được cái gì:
//   - Bias chưa từng được áp (Gz = -1.94 dps như đã gặp)  -> BẮT ĐƯỢC
//   - Bias NVS trôi nhiệt độ nặng (>1 dps)                -> BẮT ĐƯỢC
//   - Drone bị cầm/chạm lúc chờ ARM                       -> BẮT ĐƯỢC
// Cái gì LỌT: trôi nhẹ 0.3-1.0 dps. Ở 1.0 dps, yaw trôi ~3.6°/phút — chấp nhận
// được cho chuyến bay ngắn trong nhà, KHÔNG chấp nhận được nếu bay lâu.
// Muốn chặt lại thì hạ về 0.30f; muốn biết mình đang ở đâu thì gõ 'cal_status'
// và đọc GCORR.
#define PREARM_GYRO_MAX_MEAN_DPS        1.00f
// Cửa sổ trung bình trượt — đủ dài để một mẫu nhiễu đơn lẻ không gây từ chối
// (mục 18: "Do not fail based on one noisy sample"). 100ms @250Hz = 25 mẫu.
#define PREARM_GYRO_WINDOW_MS           100

// ---- Cảnh báo lệch nhiệt độ so với lúc calib (mục 9) ----
// KHÔNG chặn ARM, KHÔNG tự sửa bias — chỉ cảnh báo. Bias gyro MPU6050 trôi cỡ
// vài chục mdps/°C; 15°C lệch là đủ để yaw trôi thấy rõ trở lại.
#define CALIB_TEMP_WARN_DELTA_C         15.0f

// ============================================================================
// 9) BARO ALTITUDE SCALE (baro_driver.c) — ToF đã bỏ hẳn (KHÔNG gắn trên
//    board), barometer (BMP280) là NGUỒN DUY NHẤT cho alt_estimator (luôn ở
//    chế độ "PRIMARY", xem alt_estimator.h). alt_m trả về từ baro_driver_read()
//    LUÔN tương đối so với mốc calibrate_ground() (offset độ cao ban đầu —
//    xem baro_driver.h), rồi alt_hold.c điều khiển tương đối quanh `hover`
//    (mục 2) — 2 lớp "tương đối" này đã có sẵn, KHÔNG cần thêm gì.
//
//    BARO_ALT_SCALE — hệ số NHÂN thêm vào alt_m tương đối (giống cách
//    BATTERY_DIVIDER_RATIO nhân vào ADC pin đọc pin, xem battery_driver.h) để
//    hiệu chỉnh sai số hệ thống của công thức khí áp chuẩn (44330*(1-(P/P0)^n))
//    so với độ cao thật đo bằng thước — mặc định 1.0 (chưa hiệu chỉnh). Cách
//    đo: bay/nâng drone lên đúng 1 độ cao THẬT biết trước (vd 1.00m bằng
//    thước), so với alt_m firmware báo lúc đó (BARO_ALT_SCALE=1.0), rồi đặt
//    BARO_ALT_SCALE = <độ cao thật> / <alt_m đọc được>.
#define BARO_ALT_SCALE   1.0f

// ============================================================================
// 10) BATTERY (ADC divider + bù throttle khi pin sụt áp)
// ============================================================================

// ---- 10a) Hệ số chia áp ADC — KHÔNG CÒN Ở ĐÂY ----
// Chuyển sang battery_driver.h (BATTERY_DIVIDER_RATIO, suy ra từ
// BATTERY_R_TOP_OHM=22k / BATTERY_R_BOTTOM_OHM=10k) vì nó là hằng số VẬT LÝ
// của bo mạch, KHÔNG phải tham số tune: để ở tuning.h mời gọi việc "chỉnh cho
// số đẹp" trong khi thứ duy nhất được phép sửa nó là đổi điện trở trên bo.
// Dải hợp lệ pin 1S (BATTERY_MIN/MAX_VALID_V) cũng ở đó, cạnh công thức dùng nó.

// ---- 10b) Bù throttle khi pin sụt áp — ĐÃ BỎ HẲN, KHÔNG CÒN HẰNG SỐ NÀO ----
// TRƯỚC ĐÂY: throttle_cmd *= clamp(NOMINAL_V / battery_v, 1.0, MAX_GAIN), với
// BATTERY_COMPENSATION_NOMINAL_V=4.2f và BATTERY_COMPENSATION_MAX_GAIN=1.25f.
// Cả 2 hằng số đã XOÁ cùng toàn bộ đường nhân (xem flight_core.c bước 9b).
//
// VÌ SAO BỎ: VBAT đo được tụt theo TẢI TỨC THÌ, không chỉ theo mức pin còn
// lại. Lúc UAV nhấc lên, 4 motor rút dòng lớn -> sụt áp trên nội trở pin/dây
// -> VBAT đo giảm mạnh dù pin còn đầy. Nhân throttle theo số đó tạo vòng phản
// hồi DƯƠNG ký sinh nằm ngoài mọi vòng PID đã tune (ga lên -> dòng tăng ->
// VBAT giảm -> comp tăng -> ga lên nữa), và làm duty THỰC ra motor khác duty
// mà alt_hold PID yêu cầu -> mọi gain tune ở bench đều sai khi bay thật.
// Đo được từ log bench thật: BATV dao động 3.51..3.82V trong VÀI GIÂY ở tải
// gần như không đổi -> comp nhảy 1.10..1.20, tức nhiễu áp bị khuếch đại thẳng
// vào throttle.
//
// Điện áp pin VẪN dùng cho: failsafe sàn pin (COMMANDER_DEFAULT_BATTERY_FLOOR_V
// mục 5) và prearm_check(). Đó là so NGƯỠNG — dùng đúng chỗ, không nhân vào
// đường điều khiển. Telemetry BCOMP= giữ lại nhưng LUÔN = 1.000 (giữ format
// dòng STATUS cho GUI cũ).
//
// fc.set_param("battery_comp_nominal_v"/"battery_comp_max_gain", ...) vẫn được
// NHẬN nhưng chỉ log cảnh báo "BO QUA" — xem flight_core.c apply_set_param().
// ============================================================================

// IMU DLPF: cat rung canh quat truoc khi vao Mahony/estimator. Dat qua thap
// thi tre pha lam attitude dao; qua cao thi rung lot vao vong dieu khien.
#define IMU_ACCEL_DLPF_CFG   3

#ifdef __cplusplus
}
#endif
