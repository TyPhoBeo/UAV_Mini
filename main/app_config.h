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
// ALT_EST_NO_CORRECTION_DEGRADED_MS. Muốn bay cao/bay lâu qua bàn thì BẬT.
#define SENSOR_BARO_ENABLED      0   // BMP280 (xem baro_driver.h + ghi chú trên)
#define SENSOR_FLOW_ENABLED      0   // PMW3901 optical flow — mới khai báo pin SPI, CHƯA có driver
#define SENSOR_RGB_LED_ENABLED   0   // WS2812 — mới khai báo pin, CHƯA có driver
#define SENSOR_BATTERY_ENABLED   1   // ADC1 pin sense (IO6)
#define SENSOR_CAMERA_ENABLED    0   // OV2640 — có trên bo, KHÔNG dùng cho firmware bay, không khai báo pin

// ---- Latch ga hover theo điện áp pin (hover_model.h) ----
// 1 = lúc ARM, đo vbat (motor CHƯA quay -> điện áp KHÔNG TẢI) rồi chốt
//     hover_ff + ga PRIME theo model hover(V). Chốt MỘT LẦN, đóng băng suốt
//     chuyến bay — KHÔNG phải bù pin liên tục (cái đó đã bị gỡ có chủ đích, xem
//     flight_core.c bước 9b). Pin tụt dần trong lúc bay do I của vòng Vz tự bù.
// 0 = hover_ff giữ nguyên hằng số ALT_HOLD_HOVER_NOMINAL như trước — hành vi
//     cũ y nguyên, kể cả điều kiện ARM (không có thêm lý do từ chối nào).
//
// TẮT VỀ 0 nếu bay thử thấy ga ban đầu sai hẳn: model chỉ fit từ 2 điểm đo
// bench-ramp của MỘT con drone, đổi motor/cánh/khung là phải đo lại.
#define HOVER_LATCH_ENABLED      1

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
// giả định thứ tự mixer M1=front-left, M2=front-right, M3=back-right,
// M4=back-left. SAI mapping ở đây = lật ngay khi arm, KHÔNG cứu được bằng tune.
//
// VỊ TRÍ (góc nào của khung X) ĐÃ XÁC NHẬN xong (xem bảng dưới) — VẪN CÒN
// THIẾU CHIỀU QUAY (CW/CCW) từng góc, PHẢI làm trước khi lắp cánh quạt/bay:
//   1. Tháo cánh quạt cả 4 động cơ (BẮT BUỘC, an toàn tuyệt đối, nếu chưa tháo).
//   2. Nạp firmware (src/main.c, console USB) — xem README mục "Bay qua
//      console USB". Đảm bảo state = DISARMED (mặc định lúc boot).
//   3. Gõ lệnh `test_motor 1 15` (motor 1, 15% duty, tự dừng sau ~0.5s) —
//      CHỈ chạy được khi DISARMED, tự chặn nếu đang ARMED/bay (xem
//      flight_core.c::apply_command() case CMD_TEST_MOTOR). Lặp lại cho
//      test_motor 2/3/4, quan sát CHIỀU QUAY (CW nhìn từ trên xuống hay CCW).
//   4. So với chiều mixer Quad-X kỳ vọng: 2 động cơ CHÉO NHAU (front-left +
//      back-right) quay CÙNG chiều, 2 động cơ CHÉO CÒN LẠI (front-right +
//      back-left) quay chiều NGƯỢC LẠI — đây là yêu cầu VẬT LÝ bắt buộc để
//      mixer tạo được mô-men yaw (không liên quan tới đảo dấu ATT_MIX_YAW_SIGN,
//      cái đó CHỈ đổi lệnh xoay bên nào, không tạo ra chiều quay nếu lắp sai
//      từ đầu). Nếu 1 cặp chéo bị lắp NGƯỢC (cùng chiều với cặp kia), PHẢI đảo
//      2 trong 3 dây động cơ đó (đảo thứ tự pha) để đổi chiều quay vật lý —
//      KHÔNG sửa được bằng phần mềm.
//   5. Nếu chiều quay ĐÚNG theo cặp chéo nhưng CẢ HỆ đảo ngược (vd tất cả yaw
//      phản ứng ngược lệnh), đảo ATT_MIX_YAW_SIGN trong tuning.h — KHÔNG đảo
//      dây động cơ trong trường hợp này.
//
// VỊ TRÍ đã xác nhận (sơ đồ người dùng đo trên khung thật) — motor_gpio[] ở
// fc_bridge.c/src/main.c đã gán CHÉO đúng theo vị trí này. CHƯA xác nhận
// CHIỀU QUAY (CW/CCW) từng góc — dùng `test_motor <1-4> <pct>` (đã THÁO CÁNH
// QUẠT) quan sát chiều quay thật, so với chiều mixer Quad-X kỳ vọng (diagonal
// front-left/back-right quay CÙNG chiều, front-right/back-left quay chiều
// NGƯỢC lại). Sai chiều quay -> KHÔNG sửa vị trí, đảo dấu mixer
// (ATT_MIX_ROLL_SIGN/PITCH_SIGN/YAW_SIGN, tuning.h) hoặc đảo 2 dây động cơ đó.
#define MOTOR_POSITIONS_CONFIRMED   1

// CHỈ để đọc/log cho người (khớp motor_gpio[] đã gán trong fc_bridge.c/
// src/main.c), KHÔNG ảnh hưởng runtime.
#define MOTOR_CH1_PHYSICAL_POS   "back-right"
#define MOTOR_CH2_PHYSICAL_POS   "front-right"
#define MOTOR_CH3_PHYSICAL_POS   "front-left"
#define MOTOR_CH4_PHYSICAL_POS   "back-left"

// ================= CALIBRATION (xem calibration.h + README.md) =================
// ARM giờ BỊ TỪ CHỐI nếu chưa có accel (+ mag nếu SENSOR_MAG_ENABLED=1) hợp lệ
// trong NVS — xem flight_core.c apply_command() CMD_ARM. Lần flash đầu tiên
// (NVS trống) PHẢI chạy calib 1 lần: console `calib_accel_face` x6 (+
// `calib_mag_start`/`calib_mag_stop` nếu có mag) HOẶC fc.calibrate_accel_face()/
// fc.calibrate_mag_start()/stop() (MicroPython) — xem README mục "Calibration".
//
// KHÔNG có calib nào chạy tự động: boot CHỈ nạp calib đã lưu NVS (gyro/accel/
// mag), KHÔNG tự đo lại bất kỳ cảm biến nào. Cả 3 loại calib (gyro/accel/mag)
// CHỈ chạy khi có lệnh serial tường minh (`calib_gyro`/`calib_accel_face`/
// `calib_mag_start`+`calib_mag_stop`, hoặc `fc.calibrate_*()` phía MicroPython)
// — muốn re-zero gyro chống trôi nhiệt độ giữa các lần bay, tự gọi `calib_gyro`
// (mất ~1.5s, đứng yên) trước khi arm mỗi lần cần, KHÔNG có cờ tự động nào.
