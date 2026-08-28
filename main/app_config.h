// app_config.h — BẬT/TẮT cảm biến + tính năng, và bảng xác nhận vị trí vật lý
// động cơ. board_config.h (cùng thư mục) trả lời "chân nào" (pin/địa chỉ I2C);
// file này trả lời "cái nào ĐANG DÙNG". Tách riêng để đổi 1 cờ = 1 chỗ DUY
// NHẤT, không rải if/else cảm biến khắp flight_core.c.
//
// Include CẢ HAI file (board_config.h + app_config.h) ở nơi build
// flight_core_board_config_t (hiện tại: micropython_module/fc/fc_bridge.c và
// src/main.c) — xem cách 2 file đó dùng SENSOR_*_ENABLED bên dưới.
#pragma once

// ================= BẬT/TẮT CẢM BIẾN + TÍNH NĂNG =================
// 1 = board THẬT SỰ có cảm biến này VÀ firmware dùng nó -> flight_core init +
//     đọc bình thường mỗi tick.
// 0 = board KHÔNG hàn / CHƯA muốn dùng -> flight_core BỎ QUA HẲN việc init
//     (không dò I2C, không log warning "not found" gây nhiễu log, không tốn
//     thời gian boot). Driver liên quan coi như "chưa có cảm biến" vĩnh viễn,
//     giống hệt hành vi khi init thật sự thất bại (an toàn mặc định).
//
// Đổi cờ Ở ĐÂY rồi build lại (pio run) là áp dụng — KHÔNG sửa flight_core.c.
#define SENSOR_IMU_ENABLED       1   // MPU6050 — BẮT BUỘC, xem #error bên dưới
#define SENSOR_MAG_ENABLED       0   // QMC5883P — có driver thật, chống trôi yaw (xem README hạn chế #1)
// VL53L0X hướng xuống — nguồn CORRECTION có điều kiện cho alt_estimator (nguồn
// chính là tích phân Az/IMU). ĐÃ BỎ SENSOR_TOF2_ENABLED: con ToF thứ hai chưa
// bao giờ được fuse vào control loop, xem tof_driver.h mục "ĐÃ ĐƠN GIẢN HOÁ".
#define SENSOR_TOF_ENABLED       1
// BMP280 — sửa trôi dài hạn cho alt_estimator. TẮT được (ToF-only bay được),
// NHƯNG khi ToF hết tầm (>~1.8m) hoặc nhìn bề mặt khác (bàn/ghế) thì KHÔNG còn
// nguồn correction nào -> Commander tự LANDING sau
// Flow estimator rút gọn hiện chỉ fusion VL53L0X; cờ baro được giữ để mở rộng
// sau nhưng chưa làm nguồn correction cho Z/Vz.
#define SENSOR_BARO_ENABLED      0   // BMP280 (xem baro_driver.h + ghi chú trên)
#define SENSOR_FLOW_ENABLED      0   // PMW3901 optical flow — mới khai báo pin SPI, CHƯA có driver
#define SENSOR_RGB_LED_ENABLED   0   // WS2812 — mới khai báo pin, CHƯA có driver
#define SENSOR_BATTERY_ENABLED   1   // ADC1 pin sense (IO6)
#define SENSOR_CAMERA_ENABLED    0   // OV2640 — có trên bo, KHÔNG dùng cho firmware bay, không khai báo pin

// ---- Latch ga hover theo dien ap pin (hover_model.h) ----
// 1 = luc ARM, do vbat roi chot hover_ff + ga PRIME theo model hover(V).
// 0 = hover_ff = hang so ALT_HOLD_HOVER_NOMINAL.
// ⚠ TAT (=0) — DA DO DUOC TREN LOG BAY THAT, KHONG PHAI SUY DOAN.
//
// Latch chot ga hover theo vbat DUY NHAT MOT LAN luc ARM, qua model
//     hover(V) = 900 * (4.2/V)^2.6        (hover_model.h)
// So mu 2.6 KHUECH DAI moi sai so cua phep do vbat.
//
// Do ADC vbat trong CUNG mot chuyen bay dao dong 2.93..4.07V (bien do 1.14V
// tren mot vien 1S). Qua model, cung mot con drone cho ra:
//     V=4.07 -> FF =  977 duty
//     V=3.61 -> FF = 1334 duty
//     V=2.93 -> FF = 2295 duty
// Tuc la ket qua phu thuoc vao viec latch TRUNG mau nao luc ARM.
//
// Hau qua da quan sat (2 chuyen bay lien tiep):
//     HOVLV=3.61V -> TKOBASE=1332, ga hover THAT ~993  -> thua 339 duty (34%)
//     vz thuc +1.1 m/s trong khi VZTGT=-0.10 (lenh DI XUONG)
//     TKOI bo toi -127 va VAN chua du (can -339) -> drone vuot target 0.50m
//     len 1.69m va con dang len.
// Vong Vz lam dung viec, nhung no phai chong lai mot feedforward sai 34%.
//
// TAT -> hover_ff = ALT_HOLD_HOVER_NOMINAL (hang so). Do duoc o lan bay on
// dinh: THR=995 = 1000 + VZP 2.80 - VZI 6.97, tuc hover that ~993 duty.
// Hang so 1000 lech ~7 duty (0.7%) — I-term nuot phan do trong vai tram ms.
//
// MUON BAT LAI: phai loc vbat truoc (trung vi cua so dai, tu choi chot khi
// do lech mau qua lon). Bat lai voi ADC nhu hien tai se lap lai dung loi nay.
#define HOVER_LATCH_ENABLED      0


// ---- Terrain offset (bay qua bàn/ghế mà KHÔNG mất tham chiếu ToF) ----
// 1 = alt_estimator theo dõi BẬC TERRAIN dưới bụng drone: phát hiện bằng
//     RESIDUAL (thay đổi range KHÔNG giải thích được bằng vz), xác nhận N mẫu
//     rồi COMMIT một offset giữ alt_datum LIÊN TỤC xuyên qua cú nhảy -> PID
//     không thấy gì bất thường, ToF không bị gate mất vĩnh viễn.
//     Mở thêm frame AGL (giữ khoảng cách so với BỀ MẶT) + guard khoảng hở tối
//     thiểu, và cho landing chạy trên AGL thay vì datum.
// 0 = terrain_off_m LUÔN 0, alt = độ cao trên sàn đã khoá.
//
// ============================================================================
// ĐÃ TẮT (=0) — theo yêu cầu người dùng: "bỏ nhận diện floor hay vật thể"
// ============================================================================
// ⚠ CÂU MÔ TẢ CŨ CỦA NHÁNH "=0" ĐÃ SAI và t sửa luôn: nó nói tắt terrain thì
// "bay qua bàn -> innovation vượt gate -> mất correction". Innovation gate ĐÃ
// BỊ BỎ (alt_estimator.c), nên vế đó không còn đúng nữa.
//
// LÝ DO TẮT — lỗi THẬT, đo được trên bo:
//   TOFF=-0.315  TPEND=1  TCMT=1  TOFST=2  TOFFUSE=0  TOFTRACK=0
//   TOFR 83->95 (tang deu)   TOFA=177..178 (DUNG YEN)
// Bay qua vật thể -> terrain commit offset -0.315m -> sau đó ToF đo mặt sàn
// THẬT nhưng estimator vẫn trừ đi -0.315 -> mọi mẫu lệch 31cm -> bị loại hết.
//
// Và terr_pending KẸT Ở 1 VĨNH VIỄN: đang nghi có bậc thì tof_fusable=false
// (đúng thiết kế), nhưng vì fusable=false nên KHÔNG BAO GIỜ thu đủ mẫu để
// confirm hay huỷ nghi ngờ. Vòng luẩn quẩn — không tự thoát được.
// Kết quả: ALTSRC=4 (TOF_LOST) -> soft-fault -> LANDING giữa chuyến.
//
// ⚠ MẤT GÌ KHI TẮT:
//   - frame AGL (giữ khoảng cách so với BỀ MẶT đang nhìn) không dùng được nữa
//   - guard khoảng hở tối thiểu TERR_MIN_CLEARANCE_M không chạy -> KHÔNG còn
//     tự ép leo khi sắp cắm vào mặt bàn/vật cao
//   - landing chạy trên datum thay vì AGL: hạ xuống một cái bàn cao 0.4m thì
//     firmware vẫn nghĩ còn 0.4m nữa mới chạm
// Ba thứ đó giờ là việc của NGƯỜI LÁI. Đổi lại: bay qua vật thể không còn tự
// hạ cánh giữa chừng.
#define TERRAIN_OFFSET_ENABLED   0

// ================= MỨC CHI TIẾT DÒNG STATUS (telemetry_format.c) =================
// Dòng STATUS có 157 field. Phần lớn là số debug của các giai đoạn đã xong
// (gyro calib, terrain offset, floor gate...) — vẫn được format mỗi 50ms dù
// không ai đọc. Cờ này CẮT BỚT phần đuôi đó, KHÔNG đổi phần đầu.
//
//   0 = OFF     — không gửi STATUS (chỉ reply lệnh trực tiếp)
//   1 = MINIMAL — tới HOVLD. Đây là TOÀN BỘ phần GUI thật sự parse
//                 (tools/uav_udp_console.py::STATUS_RE kết thúc ở HOVLK/HOVLV/
//                 HOVLD, sau đó là r"(?:\s.*)?$" nuốt mọi thứ dư). Nên MINIMAL
//                 KHÔNG làm mất bất kỳ thứ gì GUI đang hiển thị.
//   2 = FULL    — thêm đuôi chẩn đoán: az/PID nội bộ, terrain, gyro calib.
//
// VÌ SAO ranh giới đúng ở HOVLD: mọi field sau nó (TOFZ AZBZ ZREQ VZOUT TOFF
// CLR FRAME GRAWX GCAL...) đã được kiểm tra là KHÔNG xuất hiện trong bất kỳ
// regex nào của GUI — chúng chỉ hiện ở khung log text. Cắt chúng không làm
// hỏng đồ thị hay ô số nào.
//
// KHÔNG ảnh hưởng điều khiển: telemetry_format_status_line() chạy trong
// net_task (prio 5, core 0), tách hẳn stabilize_task (prio 23, core 1).
#ifndef TELEMETRY_LEVEL
#define TELEMETRY_LEVEL          2   // 2=FULL (giữ nguyên hành vi cũ)
#endif

#if (TELEMETRY_LEVEL != 0) && (TELEMETRY_LEVEL != 1) && (TELEMETRY_LEVEL != 2)
#error "TELEMETRY_LEVEL chi duoc la 0 (OFF), 1 (MINIMAL) hoac 2 (FULL)"
#endif

#if SENSOR_IMU_ENABLED == 0
#error "SENSOR_IMU_ENABLED phai = 1 -- khong IMU thi attitude khong bao gio valid, khong the arm, tat co dinh khong ich gi ma con che dau loi neu vo tinh tat nham"
#endif

// ================= WIFI / UDP (ground station qua tools/uav_udp_console.py) =================
// ESP32 kết nối WiFi STA hardcode (giống UAV-Mini). Máy tính cùng LAN gửi 1
// gói UDP bất kỳ tới <ESP32_IP>:WIFI_UDP_PORT để "đăng ký" làm peer; từ đó
// telemetry/ACK gửi ngược lại peer đó, lệnh từ peer (arm/kill/PID tune/setpoint
// bay tay...) được đọc qua net_link — xem src/net_link.c + src/command_parser.c.
//
// !!! SỬA 2 DÒNG DƯỚI trước khi build — KHÔNG có SSID/pass thật của bạn ở đây
// (không thể đoán được) !!!
#define WIFI_STA_SSID              "INNOVISION"
#define WIFI_STA_PASS              "@Innovision68"

#define WIFI_HOSTNAME              "uav-s3"
#define WIFI_UDP_PORT              4210
#define WIFI_CONNECT_TIMEOUT_MS    15000
#define WIFI_UDP_RX_QUEUE_LEN      2048
// Peer coi như mất kết nối nếu không nhận được gói nào trong khoảng này (net_link
// tự "quên" peer, ngừng gửi telemetry — KHÁC HẲN watchdog bay tay SP_STALE_TIMEOUT_US
// hay Commander heartbeat, xem flight_core.c — đây chỉ là mức transport).
#define WIFI_UDP_PEER_TIMEOUT_MS   30000

// ================= XÁC NHẬN VỊ TRÍ VẬT LÝ ĐỘNG CƠ (Quad-X) =================
// board_config.h chỉ có PIN theo NET (CH1..CH4) trên schematic — KHÔNG biết
// CH nào nằm góc nào của khung X hay chiều quay CW/CCW. attitude_control.h
// giả định thứ tự mixer M1=back-right, M2=front-right, M3=front-left,
// M4=back-left. SAI mapping ở đây = lật ngay khi arm, KHÔNG cứu được bằng tune.
//
// VỊ TRÍ và CHIỀU QUAY đều ĐÃ XÁC NHẬN (xem bảng dưới). Quy trình đo lại (khi
// thay motor/ESC hoặc nghi ngờ lắp sai) giữ nguyên ở đây:
//   1. Tháo cánh quạt cả 4 động cơ (BẮT BUỘC, an toàn tuyệt đối, nếu chưa tháo).
//   2. Nạp firmware (src/main.c, console USB) — xem README mục "Bay qua
//      console USB". Đảm bảo state = DISARMED (mặc định lúc boot).
//   3. Gõ lệnh `test_motor 1 15` (motor 1, 15% duty, tự dừng sau ~0.5s) —
//      CHỈ chạy được khi DISARMED, tự chặn nếu đang ARMED/bay (xem
//      flight_core.c::apply_command() case CMD_TEST_MOTOR). Lặp lại cho
//      test_motor 2/3/4, quan sát CHIỀU QUAY (CW nhìn từ trên xuống hay CCW).
//   4. So với chiều mixer Quad-X kỳ vọng: 2 động cơ CHÉO NHAU (front-left +
//      back-right = M3 + M1) quay CÙNG chiều, 2 động cơ CHÉO CÒN LẠI
//      (front-right + back-left = M2 + M4) quay chiều NGƯỢC LẠI.
//      Khớp đúng dấu yaw trong mixer (attitude_control.c): M1/M3 mang -Y,
//      M2/M4 mang +Y. Đây là yêu cầu VẬT LÝ bắt buộc để
//      mixer tạo được mô-men yaw (không liên quan tới đảo dấu ATT_MIX_YAW_SIGN,
//      cái đó CHỈ đổi lệnh xoay bên nào, không tạo ra chiều quay nếu lắp sai
//      từ đầu). Nếu 1 cặp chéo bị lắp NGƯỢC (cùng chiều với cặp kia), PHẢI đảo
//      2 trong 3 dây động cơ đó (đảo thứ tự pha) để đổi chiều quay vật lý —
//      KHÔNG sửa được bằng phần mềm.
//   5. Nếu chiều quay ĐÚNG theo cặp chéo nhưng CẢ HỆ đảo ngược (vd tất cả yaw
//      phản ứng ngược lệnh), đảo ATT_MIX_YAW_SIGN trong tuning.h — KHÔNG đảo
//      dây động cơ trong trường hợp này.
//
// VỊ TRÍ đã xác nhận (sơ đồ người dùng đo trên khung thật) — thứ tự mixer khớp
// THẲNG với số CH nên motor_gpio[] gán trực tiếp CH1..CH4.
//
// CHIỀU QUAY đã xác nhận: M1/M3 = CCW, M2/M4 = CW (nhìn TỪ TRÊN xuống). Đã
// kiểm chứng khớp mixer:
//   - Cặp chéo hợp lệ: M1+M3 (back-right + front-left) cùng CCW, M2+M4
//     (front-right + back-left) cùng CW, hai cặp ngược nhau -> ĐÚNG Quad-X.
//   - Dấu yaw khớp phản lực (định luật 3 Newton — cánh quay CCW đẩy KHUNG
//     theo CW = yaw âm): M1/M3 quay CCW mang -Y, M2/M4 quay CW mang +Y.
//     Cả 4 motor cộng dồn cùng hướng, không con nào triệt tiêu con nào.
//   - Khớp quy ước lệnh: +yaw_rate = CCW = quay trái (xem command.h move_dir_t
//     và apply_command() case CMD_MOVE) -> ATT_MIX_YAW_SIGN giữ = 1.0f.
// Nếu sau này thay motor/ESC làm đổi chiều quay -> KHÔNG sửa vị trí, đảo 2
// trong 3 dây pha của motor đó; chỉ khi CẢ HỆ phản ứng ngược mới đảo dấu mixer
// (ATT_MIX_ROLL_SIGN/PITCH_SIGN/YAW_SIGN, tuning.h).
#define MOTOR_POSITIONS_CONFIRMED   1
#define MOTOR_SPIN_DIRS_CONFIRMED   1

// CHỈ để đọc/log cho người (khớp motor_gpio[] đã gán trong fc_bridge.c/
// src/main.c), KHÔNG ảnh hưởng runtime.
#define MOTOR_CH1_PHYSICAL_POS   "back-right"
#define MOTOR_CH2_PHYSICAL_POS   "front-right"
#define MOTOR_CH3_PHYSICAL_POS   "front-left"
#define MOTOR_CH4_PHYSICAL_POS   "back-left"

// Chiều quay nhìn TỪ TRÊN xuống — chỉ để đọc/log, KHÔNG ảnh hưởng runtime
// (mixer đã mã hoá cứng dấu yaw theo đúng bảng này, xem attitude_control.c).
#define MOTOR_CH1_SPIN_DIR       "CCW"
#define MOTOR_CH2_SPIN_DIR       "CW"
#define MOTOR_CH3_SPIN_DIR       "CCW"
#define MOTOR_CH4_SPIN_DIR       "CW"

// ================= CALIBRATION (xem calibration.h + README.md) =================
// ARM bi tu choi neu accel persistent chua hop le, fresh gyro calibration moi
// boot chua PASS, hoac pre-arm corrected-gyro stationary window khong dat.
// Accel (+ mag neu dung) van la calibration persistent trong NVS; lan flash dau
// can chay `calib_accel_face` x6. Gyro co semantics rieng: boot luon settle ->
// collect RAW 4s -> validate doc lap 1.5s. Bias gyro trong NVS chi la history,
// khong duoc dung thay fresh result va khong cuu ARM khi fresh calibration fail.
// `calib_gyro` khi DISARMED chay lai dung cung FSM startup.
