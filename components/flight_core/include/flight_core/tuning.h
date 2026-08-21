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

#define ATT_GAIN_RATE_YAW_KP        2.5f
#define ATT_GAIN_RATE_YAW_KI        5.0f
#define ATT_GAIN_RATE_YAW_KD        0.0f
#define ATT_GAIN_RATE_YAW_ILIMIT    160.0f
#define ATT_GAIN_RATE_YAW_OUTLIM    180.0f

// ---- Dấu mixer Quad-X — đảo nếu lắp ngược board/motor, KHÔNG sửa gain ----
#define ATT_MIX_ROLL_SIGN           1.0f
#define ATT_MIX_PITCH_SIGN          1.0f
#define ATT_MIX_YAW_SIGN            1.0f

// Ga tối thiểu để PID attitude chạy (dưới ngưỡng: 4 motor quay đều, không PID).
#define ATT_MIN_THROTTLE_DUTY       400

// Ga tối thiểu để I-term (Ki) ĐƯỢC CỘNG DỒN. Dưới ngưỡng này integrator bị
// FREEZE (giữ nguyên, KHÔNG reset — khác ATT_MIN_THROTTLE_DUTY ở trên vốn
// reset SẠCH cả 6 bộ khi bỏ qua PID hoàn toàn).
//
// VÌ SAO TÁCH RIÊNG khỏi ATT_MIN_THROTTLE_DUTY dù mặc định TRÙNG GIÁ TRỊ:
// hai cái trả lời hai câu hỏi khác nhau — "có chạy PID không" (dưới ngưỡng thì
// 4 motor quay đều, không có gì để ổn định) so với "có được TÍCH LŨY sai số
// không" (ga thấp thì lực đẩy chưa đủ tạo mô-men sửa, error tồn tại nhưng
// motor KHÔNG thể khử được -> tích lũy chỉ tạo windup, bung ra lúc ga lên).
// Tách ra để nâng riêng ngưỡng Ki (vd 200 -> 400) khi tune mà KHÔNG đụng tới
// ngưỡng chạy PID.
#define ATT_I_ENABLE_THROTTLE_DUTY  600

// Trần collective (tỷ lệ của MOTOR_SAFE_MAX_DUTY) mà BÙ PIN được phép đẩy tới.
// Phần còn lại (1 - giá trị này) là dải duty chừa cho mixer tạo mô-men roll/
// pitch/yaw. 0.85 = chừa ~15% (~300 duty trên thang 2000).
//
// VÌ SAO: bù pin nhân throttle lên để giữ lực đẩy khi pin sụt. Nếu nó đẩy
// collective lên 95-100%, cả 4 motor gần kịch trần và mixer KHÔNG CÒN dải nào
// để một motor tăng thêm — drone "đủ ga" nhưng MẤT LÁI. Đổi lại là tụt độ cao
// vài cm, sửa được; mất lái thì không.
//
// CHỈ chặn phần TĂNG do bù pin. Lệnh ga chủ động của tầng trên (pha PRIME cần
// đúng TAKEOFF_PRIME_DUTY) KHÔNG bị trần này chặn — xem flight_core.c 9b.
#define ATT_MAX_COLLECTIVE_FRACTION 0.85f

// D-term LPF cutoff (Hz), dùng chung mọi vòng PID: xem PID_D_LPF_HZ trong
// flight_core/pid.h (giữ ở đó vì thuộc về module pid.h thuần, không riêng
// attitude cascade).

// ============================================================================
// 2) ALT HOLD (alt_hold.h) — cascade giữ độ cao: alt -> vz_target -> vz-PI
// ============================================================================

#define ALT_HOLD_ALT_KP             0.5f     // alt_err -> vz_target [1/s] (KHÔNG phải thang duty)
// VZ_KP/KI/ILIMIT map vz_err(m/s) -> DUTY -> đã nhân đôi theo thang 2000.
//
// ---- VÌ SAO KHÔNG CÓ Kd Ở VÒNG NÀY (câu trả lời cho "đề xuất PID cho Vz") ----
// Vòng này là PI có chủ đích, KHÔNG phải PID thiếu sót.
//   - vz ĐÃ LÀ đạo hàm của độ cao. Thêm D nghĩa là đạo hàm bậc hai của một tín
//     hiệu vốn đã nhiễu: ToF lượng tử ~1mm ở 31.25Hz, qua alpha-beta ra vz, rồi
//     đạo hàm ở 250Hz -> khuếch đại nhiễu thẳng vào duty, motor rít, không thêm
//     được thông tin nào.
//   - Thứ mà D lẽ ra làm (giảm vọt lố) ở kiến trúc này do TẦNG NGOÀI làm:
//     vz_target = alt_kp*alt_err bị kẹp bởi TAKEOFF_MAX_CLIMB_MS. Đó mới là
//     chỗ chỉnh vọt lố, không phải Kd.
//   - Đã mô phỏng Kd = 15 và 30 trên plant có trễ: thời gian vào HOLD 6.42s ->
//     6.30s/6.22s, tức là nằm trong sai số của model. Không đáng đổi lấy nhiễu.
// Nếu sau này VẪN vọt sau khi đã hạ MAX_CLIMB: hạ ALT_HOLD_ALT_KP trước, rồi
// mới nghĩ tới D (và phải kèm LPF như PID_D_LPF_HZ bên attitude).
//
// ---- Kp/Ki: vì sao đổi ----
// Ki 200 -> 400. Ki là thứ HỌC ra hover thật (hover_ff chỉ là ước lượng thô),
// nên nó quyết định bao lâu drone mới bám đúng tốc độ leo yêu cầu.
// Bằng chứng, KHÔNG phải cảm tính:
//   - python/test_takeoff_flow_offline.py case C3 ĐANG ĐỎ với Ki=200: leo thật
//     0.22 m/s trong khi lệnh 0.50 m/s.
//   - Mô phỏng (plant có trễ, target 1.0m, hover lệch +150 duty):
//       Ki=200 -> vào HOLD sau 7.54s ; Ki=400 -> 6.27s
//     và khi hover lệch NHIỀU hơn (+300 duty):
//       Ki=200 -> 10.25s ; Ki=400 -> 7.83s
// Kp GIỮ NGUYÊN 100. Tôi đã thử nâng lên 150 và ĐO LẠI cho thấy nâng Kp làm
// TỆ ĐI, không phải tốt lên — tốc độ leo trung bình (test C3, target 0.45m):
//     Kp=100 -> 0.28 m/s   Kp=150 -> 0.24 m/s   Kp=200 -> 0.21 m/s
// trong khi vọt lố gần như không đổi (-5.3% / -4.3% / -4.1%).
// Cơ chế: P dập sai số vz nhanh, nên sai số còn lại cho I nhỏ đi, nên I học
// hover THẬT chậm hơn — mà chính I mới là thứ quyết định drone có bám nổi tốc
// độ leo hay không. Nâng Kp là mua một chút phản ứng tức thời bằng cách làm
// chậm đúng cái vòng đang giới hạn hiệu năng.
//
// ⚠ SỐ NÀY CHƯA ĐO TRÊN PHẦN CỨNG THẬT. Plant mô phỏng không có rung
// động cơ, không có hiệu ứng mặt đất, không có sụt áp pin. Bay thử phải THÁO
// CÁNH/giữ trên giá trước, xem `status` (vz_i, throttle) có mượt không.
#define ALT_HOLD_VZ_KP              50.0f
#define ALT_HOLD_VZ_KI              100.0f
#define ALT_HOLD_VZ_ILIMIT          500.0f
// hover_ff DANH NGHĨA — giá trị KHỞI TẠO của s_hold_tune.hover.
//
// ⚠ KHÔNG còn là hover thật dùng lúc bay. ARM thành công sẽ GHI ĐÈ
// s_hold_tune.hover bằng hover ĐO THEO ĐIỆN ÁP PIN (hover_model.h) — vì hover
// thật của con drone này phụ thuộc pin rất mạnh (900 duty @4.2V so với ~1350
// @3.6V), nên một hằng số biên dịch KHÔNG THỂ đúng cho cả dải pin.
//
// Số này chỉ còn tác dụng ở 3 chỗ: (1) HOVER_LATCH_ENABLED=0, (2) trước lần
// ARM đầu tiên, (3) làm mốc cho TAKEOFF_PRIME_DUTY khởi tạo.
// Đây KHÔNG phải bản sao của HOVER_MODEL_REF_DUTY (=900, hover đo tại ĐÚNG
// 4.2V) — hai đại lượng khác nhau, đừng "đồng bộ" chúng.
#define ALT_HOLD_HOVER_NOMINAL      1000.0f
#define ALT_HOLD_VZ_LIMIT_MS        0.50f   // trần |vz_target| (m/s)

// ---- W/S (GIU phim o GUI) = LỆNH VẬN TỐC LÊN/XUỐNG, không phải cộng duty ----
//
// VÌ SAO Vz CHỨ KHÔNG PHẢI ±duty (bản trước cộng thẳng ±100 duty):
// "+100 duty" là thẩm quyền KHÔNG CÓ ĐƠN VỊ — cùng một phím cho ra tốc độ leo
// khác nhau tuỳ pin đầy/cạn, tuỳ khối lượng, tuỳ mật độ không khí. Đó đúng là
// loại phụ thuộc mà latch hover theo pin (hover_model.h) vừa được thêm vào để
// XOÁ khỏi feedforward — để phím lái mang nó vào lại là không nhất quán.
//
// Lệnh Vz thì có đơn vị: giữ W = leo 0.3 m/s, giống nhau ở pin 4.2V và 3.6V,
// vì thành phần I của vòng Vz nuốt chênh lệch — đúng việc nó sinh ra để làm.
//
// Và trên con drone này Vz là tín hiệu SẠCH HƠN Z: propwash tạo OFFSET VỊ TRÍ
// (sai số DC của Z), không tạo sai số vận tốc — đạo hàm của một hằng số bằng 0.
//
// PHẢI <= ALT_HOLD_VZ_LIMIT_MS (có _Static_assert trong flight_core.c). Đặt
// dưới trần để tầng ngoài Z-PID vẫn còn lề khi người lái nhả phím.
#define ALT_HOLD_WS_VZ_MS           0.30f
#define ALT_HOLD_TILT_GATE_DEG      30.0f   // không engage/giữ khi nghiêng quá
#define ALT_HOLD_MIN_ENGAGE_M       0.10f   // cao tối thiểu để engage HOLD (m)
#define ALT_HOLD_MIN_THROTTLE_DUTY  400     // sàn PID khi đang bay

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

// ---- 3a) PRIME — cho motor quay ĐỀU, KHÔNG chạy Z/Vz PID ----
// Nhiệm vụ DUY NHẤT: đưa 4 motor brushed ra khỏi vùng chết (dead-zone) để chúng
// quay đều nhau trước khi controller cầm lái, tránh cú kick lệch lúc khởi động.
//
// ⚠ PRIME_DUTY PHẢI **THẤP HƠN HẲN** hover — nó KHÔNG được đủ sức nhấc drone.
// Đây là điều kiện then chốt của cả kiến trúc: nếu drone nhấc TRONG lúc PRIME
// thì nó đang bay bằng ga hở không có vòng kín nào, đúng thứ ta vừa bỏ đi.
// Bản trước dùng SPOOL_DUTY = 1350 (>= hover 1300) chính vì hồi đó ram ga là
// thứ DUY NHẤT nhấc drone. Giờ việc nhấc do Z/Vz controller làm, nên số này
// phải TỤT XUỐNG dưới hover.
//
// 70% hover = đủ trên dead-zone của motor brushed, còn xa mức nâng được.
//
// ⚠ ĐÂY LÀ NGUỒN DUY NHẤT của tỷ lệ PRIME/hover. hover_model.c dùng CHÍNH hằng
// số này (hover_model_prime_duty()) để tính ga PRIME từ hover đã latch theo
// pin. Đừng gõ lại 0.70 ở chỗ khác: lúc chưa latch và sau khi latch phải theo
// CÙNG một tỷ lệ, nếu không ga PRIME sẽ đổi giữa hai lần bay mà không ai biết.
#define TAKEOFF_PRIME_HOVER_FRAC    0.70f

// Giá trị KHỞI TẠO của prime_duty (takeoff_default_tune()). Chỉ có tác dụng
// khi latch theo pin KHÔNG chạy (HOVER_LATCH_ENABLED=0, hoặc trước lần ARM đầu
// tiên) — ARM thành công sẽ GHI ĐÈ s_tko_tune.prime_duty bằng giá trị suy ra
// từ hover thật đo theo pin. Xem hover_model.h.
#define TAKEOFF_PRIME_DUTY          ((int)(ALT_HOLD_HOVER_NOMINAL * TAKEOFF_PRIME_HOVER_FRAC))
#define TAKEOFF_PRIME_MS            900       // spec: PRIME_TIME_S = 0.3

// hover_ff (feedforward thô) KHÔNG định nghĩa riêng ở đây — nó LÀ
// ALT_HOLD_HOVER_NOMINAL (mục 2). Một đại lượng vật lý -> một hằng số. Định
// nghĩa thêm "TAKEOFF_HOVER_GUESS" sẽ tạo hai số cho cùng một thứ và bảo đảm
// có ngày chúng lệch nhau -> bước nhảy ga đúng lúc bàn giao.

// ---- 3b) SLEW — tốc độ TRƯỢT của target, và trần vz_target ----
// Đây là thứ quyết định "leo nhanh hay chậm". CÙNG một số dùng cho hai việc:
//   1. giới hạn tốc độ trượt của target_z  (m mỗi giây)
//   2. trần |vz_target| ra khỏi tầng Z-PID (m/s)
// Phải là cùng số: nếu trần vz nhỏ hơn tốc độ trượt, target chạy trước drone và
// error phình dần đúng như PID ngây thơ; nếu lớn hơn, trần vz thành vô nghĩa.
//
// ĐÂY LÀ NÚM CHỈNH VỌT LỐ CHÍNH, không phải Kd của vòng Vz.
// Vọt lố ở đỉnh sinh ra vì drone còn mang vz đi lên ĐÚNG LÚC target ngừng
// trượt: quán tính + trễ lực đẩy + trễ ước lượng cộng lại. Vào đỉnh với vz nhỏ
// hơn thì có ít động lượng phải hãm hơn -> vọt ít hơn. Đó là quan hệ vật lý
// trực tiếp, không phải chuyện tune mò.
//
// 0.5 -> 0.35 m/s. Với target 1.0m: đoạn trượt mất 1.0/0.35 ~ 2.9s (trước là
// 2.0s), tổng thời gian vào HOLD ~6s — còn rất xa TAKEOFF_TOTAL_TIMEOUT_MS
// (15s), nên không có nguy cơ đụng trần thời gian.
//
// Số này dùng cho HAI việc (đọc mục 3b ngay trên): trần tốc độ trượt target VÀ
// trần |vz_target|. Phải giữ là CÙNG một số.
#define TAKEOFF_MAX_CLIMB_MS        0.1f

// ---- 3c) LIFTOFF — THEO THỜI GIAN, KHÔNG THEO ĐỘ CAO ĐO ĐƯỢC ----
//
// ⚠⚠ ĐÂY LÀ MỘT ĐÁNH ĐỔI CÓ Ý THỨC, ĐỌC HẾT TRƯỚC KHI ĐỔI ⚠⚠
//
// Bản trước dùng `est_z > ground_alt + TAKEOFF_LIFTOFF_DELTA_M` làm bằng chứng
// "đã rời đất", và dùng `|est_z - final_target| <= tol` làm bằng chứng "đã tới
// nơi". Cả hai đều dựa vào ĐỘ CAO ĐO ĐƯỢC.
//
// VÌ SAO BỎ: cấu hình hiện tại là BARO-ONLY (SENSOR_TOF_ENABLED=0). Đo được
// trên mô phỏng (bảng đầy đủ ở alt_estimator.h): propwash ánh xạ 1:1 sang sai
// số độ cao — propwash 10cm thì est_z sai 10cm, 30cm thì sai 30cm, và KHÔNG bộ
// lọc nào loại được vì đó là lệch HỆ THỐNG của phép đo. Đòi est_z nằm trong
// ±5cm suốt 800ms từ một nguồn sai lệch 10-30cm là đòi một thứ cảm biến không
// có. Hệ quả thực tế: takeoff hoàn toàn bình thường bị ABORT -> EMERGENCY ->
// LANDING, lặp lại y hệt mỗi lần vì sai lệch là hệ thống chứ không ngẫu nhiên.
//
// GIỜ: chuỗi cất cánh chạy theo THỜI GIAN. Sau TAKEOFF_LIFTOFF_MS kể từ lúc
// vào CLIMB, coi như đã rời đất.
//
// ⚠ CÁI MẤT ĐI, nói thẳng: KHÔNG CÒN cơ chế tự phát hiện "drone không nhấc nổi"
// (kẹt cánh, quá tải, pin yếu). Trước đây một drone bị chặn sẽ ABORT; giờ nó sẽ
// được tuyên bố airborne rồi bàn giao sang HOLDING trong khi vẫn nằm trên đất,
// và alt_hold sẽ đẩy ga lên tới trần để đuổi theo một độ cao không bao giờ tới.
// Dấu hiệu nhìn thấy được: ALTSAT=1 (alt_pid_saturated) kéo dài + TKOI bò tới
// trần. KHÔNG có auto-abort nào cho việc đó nữa — người lái phải tự KILL.
//
// Cơ chế an toàn CÒN LẠI (không phụ thuộc độ cao đo): guard nghiêng
// (TAKEOFF_ABORT_TILT_DEG), mất hết nguồn đo độ cao (TAKEOFF_TOF_LOST_MS),
// tổng thời gian (TAKEOFF_TOTAL_TIMEOUT_MS), và toàn bộ Commander (pin, tilt
// cứng, heartbeat, loop health).
//
// CHỌN SỐ: đây là "bao lâu từ lúc bắt đầu leo cho tới khi bánh rời đất" trên
// KHUNG CỦA BẠN. Đặt quá NGẮN -> mở Ki attitude khi còn đè đất -> ground
// windup. Đặt quá DÀI -> Ki bị khoá trong lúc đã bay thật -> drone trôi.
// 600ms là điểm khởi đầu, PHẢI đo lại trên phần cứng: quay video, đếm từ lúc
// TKOP chuyển 1->2 (PRIME->CLIMB) tới lúc chân rời sàn.
#define TAKEOFF_LIFTOFF_MS          600

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
#define TAKEOFF_HOLD_ENTER_MS       800

// ---- 3e) ABORT — mọi nhánh đều dẫn về EMERGENCY ----
// EMERGENCY tự phân giải thành LANDING (còn kiểm soát + đang trên không) hoặc
// DISARMED + kill latch (mất kiểm soát hoặc còn ở mặt đất) — xem
// fsm_on_emergency_resolve() + bước 7 của stabilize_task. Nhờ vậy takeoff KHÔNG
// cần tự chọn policy trước/sau liftoff nữa.
//
// (1) ĐÃ BỎ — TAKEOFF_NO_LIFT_TIMEOUT_MS ("chưa nhấc nổi trong 3s -> TIMEOUT").
//     Nó kết luận bằng est_z, thứ mà cấu hình baro-only không cung cấp đủ chính
//     xác (propwash sai 10-30cm). Không còn cơ chế tự bắt kẹt cánh/quá tải —
//     xem khối cảnh báo mục 3c.
//
// (2) ĐÃ BỎ — TAKEOFF_TOTAL_TIMEOUT_MS (hằng số cứng 15s).
//     Thay bằng hạn chót TÍNH THEO TARGET trong takeoff_begin(). Hằng số cứng
//     ghép chặt với TAKEOFF_MAX_CLIMB_MS và ghép sai thì hỏng ÂM THẦM: với
//     max_climb 0.1 m/s hiện tại, target 3m cần 31.3s > 15s nên MỌI chuyến bay
//     cao đều abort với lý do "TIMEOUT" — đổ lỗi cho phần cứng trong khi thủ
//     phạm là hai hằng số không khớp nhau.
//
// (2b) THAY THẾ cho (1): "KHÔNG NHẤC NỔI" phát hiện bằng GA KỊCH TRẦN, không
//     bằng độ cao đo được.
//
//     Cơ sở: drone đang bay thật thì vòng Vz cân bằng quanh hover, thấp hơn
//     trần collective một khoảng rõ rệt. Drone kẹt cánh / quá tải / pin yếu thì
//     controller đòi thêm lực mãi mà vz không đáp, nên I bò tới trần rồi nằm
//     lì. Đó là bằng chứng vật lý, không cần altimeter.
//
//     BẰNG CHỨNG CHÍNH thực ra KHÔNG phải ga-kịch-trần mà là (a) "vz KHÔNG đáp
//     lại lệnh leo". Ga-kịch-trần chỉ là lưới thứ hai. Lý do, đo được: I cần
//     ~12s mới bò hết dải 500 duty, trong khi bàn giao (thuần thời gian) xảy ra
//     ở ~5.8s — một mình ga-kịch-trần phát hiện QUÁ MUỘN, drone bị chặn đã được
//     bàn giao xong trước khi nó kịp trip. Xem takeoff_land.c.
//
//     ⚠ VÌ SAO vz DÙNG ĐƯỢC dù z thì không: propwash tạo lệch VỊ TRÍ gần như
//     hằng số (baro đọc thấp hơn thật 10-30cm suốt lúc ga cao). Đạo hàm của
//     hằng số bằng 0 — nên nó KHÔNG tạo lỗi vận tốc. vz sống sót qua đúng cái
//     nhiễu đã giết chết phép đo độ cao tuyệt đối.
//
//     5000ms: phải DÀI HƠN đoạn quá độ HỢP LỆ lúc I đang học hover thật, và số
//     này suy ra từ chính các gain chứ không chọn bừa:
//         t = (hover_thật - hover_ff) / (Ki * vz_err)
//     Với lệch 150 duty, Ki=400, vz_err ~ max_climb 0.1 -> 150/(400*0.1) =
//     3.75s. Suốt 3.75s đó một drone HOÀN TOÀN KHOẺ vẫn nằm im vì lực đẩy chưa
//     đủ. Đặt 3000ms (bản đầu của tôi) làm test C1 ABORT OAN đúng một chuyến
//     cất cánh bình thường — test bắt được.
//     ⚠ HẠ TAKEOFF_MAX_CLIMB_MS hoặc ALT_HOLD_VZ_KI thì PHẢI nâng số này theo.
#define TAKEOFF_STUCK_MS                 5000


// (3) Mất nguồn đo độ cao (ToF) liên tục bao lâu thì abort. Trong cửa sổ này
//     GIỮ NGUYÊN throttle cuối (không chạy cascade trên est_z rác), quá hạn thì
//     EMERGENCY.
#define TAKEOFF_TOF_LOST_MS              300       // spec: 300ms
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

#define LAND_DESCENT_VZ             0.25f    // m/s, tốc độ hạ pha DESCEND
#define LAND_FLARE_ALT_M            0.15f    // m, ngưỡng vào FLARE
#define LAND_FLARE_VZ                0.10f   // m/s, tốc độ hạ lúc gần chạm
#define LAND_TOUCHDOWN_ALT_M        0.05f    // m, chạm đất -> cutoff

// ---- TOUCHDOWN DETECTOR ĐA ĐIỀU KIỆN (xem landing_run()) ----
// Board KHÔNG có ToF, Z đến từ baro với nhiễu ±0.3-1m sát đất — lớn gấp nhiều
// lần LAND_TOUCHDOWN_ALT_M. Dùng mình nó để cắt máy là cách chắc chắn nhất để
// rơi giữa không trung khi baro tụt một nhịp. Nên chấm điểm 4 bằng chứng +
// yêu cầu duy trì, và có pha CONTACT_CANDIDATE (VẪN giữ điều khiển) ở giữa.
#define LAND_CONTACT_THR_MARGIN     40       // duty trên LAND_MIN_THROTTLE vẫn coi là "sát sàn"
#define LAND_CONTACT_VZ_MS          0.06f    // m/s, |vz| dưới mức này dù đang lệnh hạ
#define LAND_CONTACT_ALT_MARGIN_M   0.10f    // m, cộng vào touchdown_alt_m cho bằng chứng "Z thấp"
#define LAND_CONTACT_Z_PROGRESS_M   0.03f    // m, Z giảm ÍT HƠN mức này = "đã bị chặn"
#define LAND_CONTACT_SCORE_MIN      3         // /4 điểm mới vào CONTACT_CANDIDATE
#define LAND_CONTACT_TICKS          75        // ~300ms @250Hz giữ liên tục mới CẮT MÁY
#define LAND_SETTLE_MS              300      // ga thấp + vz~0 giữ liên tục -> touchdown backup
#define LAND_MIN_THROTTLE           400      // thang duty 2000, xem đầu file
#define LAND_TOF_TIMEOUT_MS         500      // mất ToF quá lâu -> chuyển BLIND
#define LAND_BLIND_DESCENT_RATE     200.0f   // duty/giây, pha BLIND (thang duty 2000)
#define LAND_CUTOFF_MS              100      // ramp ga về 0 lúc TOUCHDOWN

// ============================================================================
// 5) COMMANDER (commander.h) — geofence + ngưỡng fault (SOFT -> LANDING,
//    HARD -> EMERGENCY)
// ============================================================================

#define COMMANDER_DEFAULT_ALT_MIN_M          0.0f
#define COMMANDER_DEFAULT_ALT_MAX_M          3.0f    // TODO: chỉnh theo geofence thực tế
// 1S LiPo (full 4.2V, nominal 3.7V, KHÔNG BAO GIỜ để dưới 3.0V — hại cell
// vĩnh viễn). Floor 3.3V chừa margin cho sụt áp dưới tải trước khi chạm đáy
// tuyệt đối — hạ/land khi chạm ngưỡng này, đừng đợi tới 3.0V. XÁC NHẬN LẠI
// bằng đo thật dưới tải (không chỉ đo hở mạch) trước khi bay.
#define COMMANDER_DEFAULT_BATTERY_FLOOR_V    2.0f
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

// ---- TRIM MẶC ĐỊNH (bù lệch cơ khí/CG của khung) ----------------------------
// Trim là HẰNG SỐ BÙ LỆCH của khung, KHÔNG phải setpoint: nó cộng vào
// target_roll/pitch ở CẢ hai nhánh lệnh (timed-command lẫn SP bay tay), nên
// drone bay thẳng khi cần lệnh 0. Xem flight_core.c bước 10.
//
// ⚠ QUY ƯỚC NGOÀI (đúng thứ GUI/@TRIM SET/NVS dùng) — KHÔNG phải trục vật lý:
//     TRIM_ROLL_DEG_DEFAULT  = trái/phải  (dương -> dạt sang PHẢI thì giảm)
//     TRIM_PITCH_DEG_DEFAULT = tiến/lùi   (dương -> dạt về TRƯỚC thì giảm)
// flight_core.c hoán trục MỘT LẦN ở chỗ khởi tạo, y hệt cách nó hoán cho
// CMD_SET_TRIM và cho đường nạp NVS. Nhờ vậy số ở đây copy THẲNG được từ
// slider GUI, không phải đổi trục trong đầu.
//
// ⚠ THỨ TỰ ƯU TIÊN — đọc kỹ trước khi sửa số ở đây:
//     NVS (đã từng bấm @TRIM SET / kéo slider)  >  hai hằng số này
// flight_core_start() nạp trim từ NVS và GHI ĐÈ giá trị mặc định nếu
// trim_valid=1. Nghĩa là: nếu bạn đã lưu trim một lần, sửa số ở đây sẽ KHÔNG
// có tác dụng gì cho tới khi chạy `calib_erase` (xoá toàn bộ calib trong NVS,
// gồm cả gyro/accel/mag — sẽ phải calib lại) hoặc ghi đè bằng chính @TRIM SET.
// Log boot "TRIM nap tu NVS: ..." cho biết bạn đang ở trường hợp nào.
//
// Hai số dưới đây là giá trị đã dò trên khung thật (trước đây nằm chôn trong
// flight_core.c dưới dạng số ma thuật, không ai sửa được từ tuning.h).
#define TRIM_ROLL_DEG_DEFAULT    (-0.68f)
#define TRIM_PITCH_DEG_DEFAULT   (0.8f)

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
#define CALIB_GYRO_DURATION_MS       1500     // đứng yên trong khoảng này để đo bias tĩnh
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

// ---- Motion/stability detection lúc calib gyro + từng mặt accel (xem PX4
// "vehicle phải bất động khi calib gyro") — CHƯA đo trên phần cứng thật, điểm
// khởi đầu hợp lý cho MPU6050 nghỉ trên bàn. ----
#define CALIB_GYRO_MAX_STD_DPS          0.5f    // std-dev gyro trong cửa sổ vượt -> nghi rung/cầm tay
#define CALIB_MOTION_ACCEL_TOL_G        0.15f   // |accel_norm-1.0| vượt 1 mẫu -> tick đó tính "nghi động"
#define CALIB_MOTION_MAX_BAD_FRACTION   0.05f   // >5% tick "nghi động" trong cửa sổ -> HỦY (CALIB_MOVING)
#define CALIB_ACCEL_MOTION_GYRO_DPS     5.0f    // |gyro| vượt 1 mẫu lúc bắt 1 mặt accel -> tick đó "nghi động"

// ---- Verify sau khi tính bias/scale accel 6-face (xem "13. Accel calibration
// verification") — áp lại công thức correction lên chính 6 mặt vừa đo, mặt
// nào |accel_hiệu_chỉnh| lệch 1.0g quá ngưỡng này -> HỦY toàn bộ, không lưu.
// CHƯA đo trên phần cứng thật, điểm khởi đầu. ----
#define CALIB_ACCEL_MAX_RESIDUAL_G      0.15f

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

// ============================================================================
// 11) MPU6050 HARDWARE DLPF (imu_driver.c) — LỚP LỌC RUNG ĐỘNG CƠ CHÍNH, quan
//    trọng hơn LPF phần mềm trong alt_estimator.h vì lọc TRƯỚC KHI số vào MCU
//    (LPF phần mềm chỉ là lớp phòng thủ THỨ 2). Estimator giờ accel-primary
//    (KHÔNG có ToF kéo lại) nên rung motor lọt vào accel = drift Vz TRỰC TIẾP,
//    không có gì sửa nhanh — cắt rung tại nguồn quan trọng hơn bao giờ hết.
//
//    DLPF_CFG (thanh ghi CONFIG 0x1A, bit[2:0]) theo bảng datasheet MPU6050
//    (Register Map, mục 4.3) — accel bandwidth/delay, SAI Ở ĐÂY sẽ SAI CẢ dấu
//    hiệu accel dùng để tích phân Vz VÀ tín hiệu gyro dùng cho attitude:
//      0 = 260Hz (0ms delay)   1 = 184Hz (2.0ms)   2 = 94Hz (3.0ms)
//      3 = 44Hz  (4.9ms)       4 = 21Hz  (8.5ms)   5 = 10Hz (13.8ms)
//      6 = 5Hz   (19.0ms)
//    Cả 6 giá trị 1-6 dùng CHUNG base rate 1kHz cho SMPLRT_DIV (xem
//    IMU_SAMPLE_RATE_HZ, imu_driver.h) — đổi CFG trong khoảng 1-6 KHÔNG cần
//    đổi SMPLRT_DIV. CHỈ CFG=0 (hoặc 7, không dùng) đổi sang base 8kHz.
//
//    Mặc định CŨ (94Hz, CFG=2) gần như không lọc gì so với rung motor thật
//    (thường vài trăm Hz) — đổi sang 44Hz (CFG=3) làm điểm khởi đầu, delay
//    4.9ms chấp nhận được ở loop 250Hz (dt=4ms, tương đương ~1.2 tick trễ).
//    CHƯA đo trên phần cứng thật — xem README test G (rung), hạ xuống 21Hz
//    (CFG=4) nếu Az_world_filtered vẫn nhặt rung tần số motor rõ.
// ============================================================================
#define IMU_ACCEL_DLPF_CFG   3

#ifdef __cplusplus
}
#endif
