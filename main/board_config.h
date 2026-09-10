// board_config.h — TẤT CẢ pin GPIO + địa chỉ I2C của board miniUav
// (ESP32-S3-WROOM-1) gom về MỘT chỗ DUY NHẤT. KHÔNG rải số GPIO ở nơi khác
// trong repo — mọi module include file này (trực tiếp hoặc qua fc_bridge.c).
//
// Nguồn: Schematic_miniUav1 rev1.0 (EasyEDA). Symbol module ghi "N8R8" nhưng
// bo THẬT là N16R8 — pinout giống hệt, chỉ khác flash=16MB/PSRAM=octal 8MB,
// cấu hình ở sdkconfig (xem mục PSRAM/FLASH cuối file), KHÔNG phải ở đây.
//
// File này được include bởi micropython_module/fc/fc_bridge.c lúc khởi tạo
// module fc (nơi flight_core_start() thực sự được gọi). Nếu layout project
// MicroPython thật cần board_config.h nằm chỗ khác, copy file này sang đó và
// sửa include path DUY NHẤT trong fc_bridge.c — bản thân file này không phụ
// thuộc gì khác.
#pragma once

// ================= I2C bus (DÙNG CHUNG: MPU6050 + QMC5883P + BMP280 + 2x VL53L1X) =================
// Pull-up 2k2 đã có sẵn trên bo (R14/R15); internal pull-up ESP32-S3 (~45kΩ)
// vẫn được bật thêm ở mức bus (i2c_master_bus_config_t.flags.enable_internal_pullup,
// xem init_i2c_bus() trong flight_core.c) — vô hại song song với 2k2 ngoài.
//
// SDA/SCL ĐÃ SỬA theo dây thật trên bo test hiện tại (KHÁC layout ban đầu
// IO15/IO7 của schematic gốc) — xác nhận lại nếu chuyển sang bo khác.
#define BOARD_I2C_SDA_GPIO   44   // IO44
#define BOARD_I2C_SCL_GPIO   43   // IO43
// 400kHz Fast Mode — tốc độ SCL cho TẤT CẢ device trên bus (MPU6050, QMC5883P,
// BMP280, ToF nếu có).
//
// ⚠ ĐÂY LÀ NGUỒN SỰ THẬT DUY NHẤT, và điều đó CHỈ ĐÚNG TỪ BẢN NÀY.
// Trước đó nó là một field CHẾT: main.c/fc_bridge.c gán vào cfg.i2c_freq_hz
// nhưng flight_core KHÔNG ĐỌC, còn tốc độ thật đến từ 4 hằng số 400000 mà mỗi
// driver tự #define riêng. Sửa số ở đây khi đó KHÔNG có tác dụng gì. Giờ
// flight_core_start() truyền nó xuống từng *_driver_init() và 4 bản sao kia đã
// bị xoá.
//
// TẠI SAO tần số ở đây mà không ở bus: trong API I2C master mới của ESP-IDF,
// i2c_master_bus_config_t KHÔNG CÓ field tần số nào — tần số là thuộc tính của
// DEVICE (i2c_device_config_t.scl_speed_hz). Nên nó phải được truyền xuống từng
// driver, và đó là lý do field này từng bị bỏ quên.
//
// Giới hạn: flight_core_start() validate 10kHz..1MHz, ngoài dải thì log lỗi và
// rơi về 400000. Pull-up 2k2 ngoài (R14/R15) đủ dốc cho 400kHz ở độ dài dây bus
// hiện tại; nếu thấy NACK/timeout tăng (soi TERR= trong STATUS, hoặc log
// "*_driver_read that bai") thì hạ về 100000 TRƯỚC khi nghi driver.
#define BOARD_I2C_FREQ_HZ    400000

// IO38 INT_MPU — data-ready interrupt, ĐANG DÙNG làm đồng hồ nhịp của
// stabilize_task (xem imu_driver_enable_data_ready_int()). Đặt < 0 để quay về
// polling bằng đồng hồ FreeRTOS — nhưng ĐỪNG: log thực đo nhịp 250.4 Hz bám
// đúng danh nghĩa 250 Hz chính là nhờ ngắt này.
//
// ✅ IO38 AN TOÀN KHI BẬT PSRAM. Tra bảng pin-mux của chính IDF
// (components/soc/esp32s3/register/soc/io_mux_reg.h):
//     GPIO33 = SPIIO4   GPIO34 = SPIIO5   GPIO35 = SPIIO6
//     GPIO36 = SPIIO7   GPIO37 = SPIDQS
//     GPIO38 = KHÔNG CÓ chức năng MSPI nào  (chỉ FSPIWP/SUBSPIWP, tức SPI2/
//              SPI3 dùng chung — không phải bus flash/PSRAM)
// Bus flash/PSRAM nội (MSPI) không thể chiếm IO38 vì phần cứng không hề route
// tín hiệu nào của nó tới chân đó.
//
// ⚠ Comment CŨ ở đây ghi "IO36 INT_MPU" và cảnh báo bật PSRAM sẽ hỏng ngắt.
// Cả hai đều SAI: macro luôn là 38, và log bay thật cho nhịp đúng 250.4 Hz —
// nếu dây INT nằm ở IO36 thì cấu hình 38 đã không thể chạy. Ghi lại đây để
// không ai đọc bản cũ rồi kết luận nhầm là PSRAM và IMU loại trừ nhau.
#define BOARD_MPU_INT_GPIO   38

#define BOARD_IMU_I2C_ADDR   0x68  // MPU6050, AD0=GND -> 0x68 (xác nhận lại nếu AD0 kéo lên VCC -> 0x69)

// QUAN TRỌNG: chip mag THẬT trên bo là QMC5883P (KHÔNG PHẢI QMC5883L như giả
// định ban đầu) — register map + địa chỉ I2C khác hẳn (xem mag_driver.c).
// Địa chỉ 0x2C đã xác nhận bằng I2C scanner thật trên bo (trùng khớp thiết bị
// "lạ" tìm thấy trước đó — hóa ra chính là mag, không phải chip khác).
#define BOARD_MAG_I2C_ADDR   0x2C  // QMC5883P, địa chỉ cố định 0x2C

// BMP280 — driver thật ở components/flight_core/src/drivers/baro_driver.c,
// fuse vào alt_estimator (sửa trôi dài hạn, xem alt_estimator.h). Địa chỉ phụ
// thuộc chân SDO: SDO=GND -> 0x76, SDO=VCC -> 0x77. ĐÃ XÁC NHẬN bằng I2C
// scanner thật trên bo (thấy thiết bị tại 0x77, KHÔNG phải 0x76 như giả định
// ban đầu) -> SDO đang kéo lên VCC. Bật/tắt qua SENSOR_BARO_ENABLED trong
// app_config.h.
#define BOARD_BARO_I2C_ADDR  0x77

// ================= ToF hướng xuống (MỘT con) =================
// ĐÃ BỎ con ToF thứ hai (forward/dự phòng). Nó chưa bao giờ được fuse vào
// control loop nhưng bắt driver phải mang những ràng buộc chỉ có nghĩa khi có 2
// chip (addr1 không được giữ 0x29, cả 2 XSHUT phải điều khiển được, thứ tự
// bring-up bắt buộc) — và chính những ràng buộc đó đã gây lỗi thật cho cấu hình
// 1 chip. Xem tof_driver.h mục "ĐÃ ĐƠN GIẢN HOÁ TỪ 2 SENSOR VỀ 1".
//
// ---------------------------------------------------------------------------
// BOARD_TOF_CHIP — DÒNG CHIP ĐANG HÀN TRÊN BO
// ---------------------------------------------------------------------------
// Firmware hỗ trợ HAI dòng, mỗi dòng một driver riêng trong
// components/flight_core/src/drivers/ (vl53l0x_driver.c / vl53l1x_driver.c).
// Đổi MỘT dòng dưới đây rồi build lại là xong — không sửa gì khác.
//
//   TOF_CHIP_VL53L0X : thanh ghi 8-bit,  ID 0xC0   = 0xEE
//   TOF_CHIP_VL53L1X : thanh ghi 16-bit, ID 0x010F = 0xEACC (gồm cả VL53L4CD)
//
// ⚠ CẮM NHẦM CHIP KHÔNG IM LẶNG: cả hai đều ACK ở 0x29, nhưng driver đọc ID
// trước khi ghi bất cứ thứ gì. Sai dòng thì log in ra ĐÚNG dòng cần sửa
// (identify_foreign_device() trong tof_driver.c). Không có trường hợp "chạy
// nhưng số sai".
//
// ĐANG DÙNG: VL53L1X. Lý do đổi từ L0X — tầm TIN CẬY.
//   L0X @timing budget 33ms chỉ tin cậy tới ~1.2m, và log chuyến bay trước cho
//   thấy ToF mất mẫu (TOFAGE 28 -> 582ms) đúng khi CLR vượt 1.25m: hết tầm ->
//   BRIDGE -> LOST -> Commander soft-fault -> LAND_BLIND. Trần bay
//   ALT_EST_MAX_FLIGHT_Z_M hiện là 1.20m, tức bay ở trần là bay ở ĐÚNG mép
//   tin cậy của L0X — không còn biên nào.
//   L1X ở SHORT giữ ~1.3m kể cả dưới ánh sáng nền mạnh; LONG tới ~2.6m trong
//   nhà. Đó là biên thật, không phải con số danh nghĩa.
// Mã dòng chip. Định nghĩa Ở ĐÂY (không phải trong driver) để board_config.h
// tự đủ: nó là mô tả PHẦN CỨNG, phải đọc được mà không cần kéo theo header nội
// bộ nào của flight_core. tof_backend.h chỉ #ifndef bọc lại đúng hai số này.
#ifndef TOF_CHIP_VL53L0X
#define TOF_CHIP_VL53L0X      0
#endif
#ifndef TOF_CHIP_VL53L1X
#define TOF_CHIP_VL53L1X      1
#endif
#define BOARD_TOF_CHIP        TOF_CHIP_VL53L1X

// ---- Tham số RIÊNG của VL53L1X (backend L0X bỏ qua hoàn toàn) ----
//
// DISTANCE MODE:
//   1 = SHORT : tới ~1.3m. MIỄN NHIỄM ánh sáng nền tốt nhất, cho phép timing
//               budget ngắn nhất -> nhịp mẫu cao nhất.
//   2 = LONG  : tới ~4m danh nghĩa (~2.6m thực tế trong nhà). Nhạy ánh sáng
//               nền hơn — datasheet ST ghi ngoài nắng tụt về ~73cm.
//
// ĐANG DÙNG LONG (đổi từ SHORT theo yêu cầu người dùng, cùng đợt nâng trần bay
// ALT_EST_MAX_FLIGHT_Z_M lên 3.0m). SHORT chỉ tin cậy ~1.3m nên không còn phủ
// nổi trần mới — bay trên 1.3m với SHORT là mất mẫu ToF giữa chuyến, đúng chuỗi
// đã làm rơi drone lần trước (hết tầm -> stale -> BRIDGE -> LOST -> soft-fault
// -> LAND_BLIND).
//
// ⚠ ĐÁNH ĐỔI KHI DÙNG LONG — phải biết trước khi bay:
//   1. NHẠY ÁNH SÁNG NỀN. Datasheet ST: ngoài nắng tầm tụt về ~73cm. Trong nhà
//      xa cửa sổ thì ~2.6m; gần cửa sổ ban ngày sẽ kém hơn RÕ RỆT. Nếu thấy
//      tof_reject_count tăng dần theo độ cao thì đây là nghi can đầu tiên, và
//      nó KHÔNG phải lỗi phần mềm.
//   2. ~2.6m thực tế VẪN NHỎ HƠN trần 3.0m. Xem cảnh báo ở
//      ALT_EST_MAX_FLIGHT_Z_M — hai con số này hiện CHƯA khớp nhau.
//   3. Nhiễu cao hơn SHORT ở cùng timing budget (VCSEL period dài hơn).
//
// Muốn quay lại SHORT: đổi về 1 VÀ hạ trần bay xuống <= 1.2m. Đổi mỗi một số ở
// đây mà giữ trần 3.0m là quay lại đúng lỗi cũ.
#define BOARD_TOF_L1X_DISTANCE_MODE       2

// TIMING BUDGET (ms): thời gian chip dành cho MỘT lần đo.
// Dài hơn = ít nhiễu hơn + xa hơn, nhưng nhịp mẫu chậm hơn. Nhịp mẫu đi THẲNG
// vào chất lượng vz của alt_estimator (alpha-beta lấy vi phân từ range), nên
// KHÔNG chỉnh tuỳ tiện.
//
// 33ms = cùng thời gian đo mà L0X đang dùng. Chọn có chủ đích: đổi chip mà giữ
// nguyên bậc nhịp mẫu nghĩa là mọi hằng số đã tune theo nhịp (ALT_EST_*,
// alpha-beta, TOF_STALE_TIMEOUT_MS) vẫn đúng — chỉ có TẦM đo thay đổi. Đổi hai
// thứ cùng lúc thì không quy được lỗi cho cái nào.
//
// ⚠ Nhịp mẫu THẬT do INTER_MEASUREMENT quyết định (40ms -> ~25Hz), không phải
// con số này. TB chỉ là thời gian chip dành cho một lần đo.
//
// Bảng của ST chỉ có 15/20/33/50/100/200/500ms (15 chỉ ở SHORT). Giá trị KHÁC
// là LỖI, driver từ chối init — KHÔNG làm tròn im lặng, vì làm vậy sẽ vô hiệu
// hoá đúng cái _Static_assert đang bảo vệ nhịp mẫu.
#define BOARD_TOF_L1X_TIMING_BUDGET_MS    33

// INTER-MEASUREMENT (ms): chu kỳ giữa hai lần BẮT ĐẦU đo.
//
// ⚠ PHẢI LỚN HƠN timing budget, KHÔNG được bằng. ST ULD ghi "IM >= TB", nhưng
// đặt BẰNG NHAU không chừa chỗ cho overhead nội bộ giữa hai lần đo — chip trượt
// nhịp và bỏ mẫu IM LẶNG (triệu chứng: "ToF lúc được lúc không", và người debug
// sẽ đi tìm dây/nguồn). ~20% dư là mức mọi hiện thực ULD dùng.
// Có _Static_assert bắt lúc build trong vl53l1x_driver.c.
//
// 40ms -> ~25Hz. Chậm hơn L0X (~30Hz) một chút, vẫn thừa xa ngưỡng
// TOF_STALE_TIMEOUT_MS=200ms (assert thứ hai kiểm đúng điều này).
// Muốn nhanh hơn: TB=20ms + IM=25ms -> ~40Hz.
#define BOARD_TOF_L1X_INTER_MEASUREMENT_MS 40

// I/O rail của I2C phía sensor. 1 = pull-up ở AVDD (~2.8V), 0 = 1.8V.
// KHÔNG phải cờ vô hại: sai rail thì mức logic không đạt ngưỡng và biểu hiện là
// NACK NGẪU NHIÊN — giống hệt lỗi dây, rất dễ đi sai hướng. Board này 2.8V.
#define BOARD_TOF_L1X_IO_2V8              1

// Địa chỉ: giữ 0x29 MẶC ĐỊNH cho cả hai dòng chip (ST dùng chung cho toàn dòng
// VL53). Chỉ 1 con nên không ai tranh địa chỉ — đổi sang 0x2A chỉ thêm một
// bước có thể thất bại mà không được lợi gì.
#define BOARD_TOF_I2C_ADDR    0x29

// ⚠ TỐC ĐỘ SCL RIÊNG CHO ToF — cố ý THẤP HƠN phần còn lại của bus.
//
// Trong API I2C master mới của ESP-IDF, tốc độ là thuộc tính của DEVICE
// (i2c_device_config_t.scl_speed_hz), KHÔNG phải của bus — nên hoàn toàn hợp lệ
// khi IMU/mag/baro chạy 400kHz còn ToF chạy 100kHz trên CÙNG hai sợi dây. IDF
// tự áp lại timing theo từng transaction.
//
// VÌ SAO ToF cần chậm hơn: BOARD_I2C_FREQ_HZ=400kHz đúng cho các chip nằm SẴN
// trên PCB (pull-up 2k2 R14/R15, dây ngắn). ToF hướng xuống lại thường được nối
// bằng DÂY RỜI ra mép khung để nhìn xuống đất — thêm điện dung dây, thêm pull-up
// 10k của module mắc song song, và chạy cạnh dây motor. Ở 400kHz, thời gian
// sườn lên chỉ còn ~0.3us; đủ điện dung là mức HIGH không kịp đạt ngưỡng và ta
// nhận NACK NGẪU NHIÊN — biểu hiện đúng như "lúc được lúc không", kể cả ở lần
// đọc MODEL_ID lúc boot.
//
// 100kHz cho gấp 4 lần biên thời gian mà KHÔNG làm chậm vòng bay: ToF chỉ đọc
// ~30Hz và nằm ngoài đường nhịp IMU (IMU vẫn 400kHz, xem sensor_hub.c).
// Transaction 12 byte ở 100kHz mất ~1.3ms — vẫn thừa trong ngân sách.
//
// Đặt = 0 để dùng chung BOARD_I2C_FREQ_HZ. Nếu đã đi dây ngắn/che nhiễu tốt và
// muốn lấy lại tốc độ, nâng dần lên 400000 rồi soi `tof_test` + stall_restarts.
#define BOARD_TOF_I2C_FREQ_HZ 400000

// XSHUT — TÙY CHỌN khi chỉ có 1 ToF. Đặt < 0 nếu không nối dây (module có
// pull-up 10k nên chip vẫn chạy). Lý do DUY NHẤT còn lại để nối: reset cứng
// chip lúc boot, xoá state trong RAM nó (kể cả địa chỉ I2C đã đổi lần trước).
//
// ⚠⚠ GPIO0 LÀ CHÂN STRAPPING "BOOT" CỦA ESP32-S3 ⚠⚠
// Nó được LẤY MẪU TẠI THỜI ĐIỂM RESET để chọn boot mode:
//     GPIO0 = HIGH lúc reset -> boot firmware bình thường
//     GPIO0 = LOW  lúc reset -> vào DOWNLOAD MODE (không chạy firmware)
// Driver kéo XSHUT xuống LOW khoảng 10ms trong flight_core_start() để reset
// cứng ToF. NẾU có một cú reset (brownout / task watchdog / nhấn nút RESET)
// RƠI ĐÚNG cửa sổ 10ms đó thì bo sẽ vào download mode và trông như "bo không
// boot nữa" — phải RÚT ĐIỆN cấp lại mới hết. Xác suất thấp nhưng triệu chứng
// rất khó lần ra, nên ghi ở đây.
//
// Ngoài cửa sổ 10ms đó thì AN TOÀN: driver lái open-drain, mức "nhả" là HIGH-Z
// nên pull-up (nội bộ của GPIO0 + 10k trên module ToF) giữ chân ở HIGH — reset
// bất kỳ lúc nào khác đều boot bình thường.
//
// MUỐN BỎ HẲN RỦI RO: đặt BOARD_TOF_XSHUT_GPIO = -1. Với địa chỉ 0x29 mặc định
// thì XSHUT KHÔNG cần thiết cho hoạt động — chỉ mất khả năng reset cứng ToF.
// ================= ĐÃ ĐỔI SANG -1 (KHÔNG DÙNG XSHUT) =================
// Bằng chứng, không phải phòng xa: với XSHUT ở GPIO0, bring-up ToF thất bại
// bằng NACK (ESP_ERR_INVALID_RESPONSE) ở ĐÚNG lệnh GHI đầu tiên của DataInit
// (reg 0x89) — trong khi lệnh ĐỌC MODEL_ID và ĐỌC 0x89 ngay trước đó đều OK.
// Chip trả lời đọc nhưng từ chối ghi = nó đang ở trạng thái reset dở/hardware
// standby, đúng thứ mà một chân XSHUT không lên sạch gây ra.
//
// Đối chứng quyết định: một driver VL53L0X tối giản chạy trên CÙNG board này
// hoạt động hoàn toàn ổn định, và nó KHÔNG hề đụng tới XSHUT — chỉ add device
// ở 0x29 rồi init. Khác biệt cấu trúc duy nhất giữa hai bên là đoạn kéo
// XSHUT xuống LOW 10ms rồi nhả.
//
// Vì sao GPIO0 đặc biệt tệ cho việc này: nó là chân strapping BOOT, có pull-up
// nội, và trên nhiều devkit còn nối thẳng vào nút BOOT. Lái open-drain rồi
// "nhả" = HIGH-Z, nên mức thật phụ thuộc mạch ngoài; nếu nó không lên đủ
// nhanh/đủ cao thì VL53L0X không bao giờ boot xong.
//
// Với MỘT ToF ở địa chỉ mặc định 0x29, XSHUT KHÔNG cần cho hoạt động — lợi ích
// duy nhất là reset cứng chip lúc boot, và cái giá vừa đo được là không init
// nổi. Muốn bật lại: hàn XSHUT sang một GPIO thường (KHÔNG phải strapping pin),
// bảo đảm pull-up lên đúng rail của module, rồi đặt số GPIO đó vào đây.
#define BOARD_TOF_XSHUT_GPIO  (-1)   // KHÔNG nối XSHUT — xem giải thích ở trên

// ================= PMW3901 optical flow (SPI riêng) =================
// LƯU Ý: IO40/41/42 là chân JTAG mặc định trên MỘT SỐ devkit S3 (MTDO/MTDI/
// MTMS) — đây là JTAG NGOÀI (external, qua header debug rời), KHÁC HẲN
// USB-Serial-JTAG controller dùng cho console/nạp code (đi qua IO19/20, xem
// mục "Console / nạp code" bên dưới). ESP-IDF KHÔNG tự bật JTAG ngoài trên
// IO40/41/42 trừ khi cấu hình OpenOCD tường minh (không dùng ở firmware này)
// -> dùng thẳng làm GPIO/SPI thường, KHÔNG cần "release JTAG pin" trong code
// hay sdkconfig. IO19/20 (USB-Serial-JTAG) ĐỘC LẬP với các chân này.
//
// CHƯA wire vào flight_core_board_config_t / chưa init SPI bus ở lần config
// này (đúng theo yêu cầu: "có thể STUB, khai báo pin sẵn"). Sẽ dùng cho
// position-hold khi làm flow_driver.
#define FLOW_MOSI_PIN   40    // IO40  OF_MOSI
#define FLOW_MISO_PIN   42   // IO42 OF_MISO (MTDI)
#define FLOW_SCLK_PIN   41   // IO41 OF_SCLK (MTMS)
#define FLOW_CS_PIN     1   // IO1 OF_CS   (MTDO)

// ================= Motor PWM (LEDC, 4 kênh) =================
// N-MOSFET KAO3400, ACTIVE-HIGH, 10k pulldown gate trên bo -> motor tự OFF
// khi IO nổi lúc boot (an toàn mặc định, không cần code xử lý thêm).
// Tần số PWM (~24kHz) + độ phân giải (10-bit) là THÔNG SỐ LEDC, cấu hình DUY
// NHẤT ở components/flight_core/include/flight_core/drivers/motor_driver.h
// (MOTOR_PWM_FREQ_HZ/MOTOR_PWM_MAX_DUTY) — KHÔNG lặp lại macro ở đây để tránh
// 2 nguồn sự thật; giá trị ở đó đã cập nhật khớp schematic (24000Hz, 1023=10-bit).
//
// QUAN TRỌNG — CHƯA XÁC NHẬN vị trí vật lý: đây là CH1..CH4 theo NET trên
// schematic, KHÔNG chắc CH1=góc nào của khung X hay chiều quay CW/CCW mỗi
// góc. attitude_control.h giả định thứ tự M1=back-right, M2=front-right,
// M3=front-left, M4=back-left (Quad-X). SAI mapping ở đây -> lật ngay khi arm.
// QUY TRÌNH xác nhận (lệnh test_motor qua console, KHÔNG suy đoán) — xem
// app_config.h mục "XÁC NHẬN VỊ TRÍ VẬT LÝ ĐỘNG CƠ".
#define MOTOR_CH1_PIN   4    // IO4  CH1  — TODO: xác nhận vị trí vật lý + chiều quay
#define MOTOR_CH2_PIN   3    // IO8  CH2  — TODO
#define MOTOR_CH3_PIN   48   // IO39 CH3 (MTCK, dùng GPIO OK) — TODO
#define MOTOR_CH4_PIN   2    // IO2  CH4  — TODO

// ================= Battery sense (ADC1, KHÔNG đụng WiFi) =================
// BẮT BUỘC ADC1 — ADC2 xung đột WiFi trên S3. battery_driver dùng ADC1 oneshot
// + calibration (esp_adc/adc_cali.h, curve-fitting scheme) — xem
// components/flight_core/src/drivers/battery_driver.c.
#define VBAT_ADC_PIN         6    // IO6
#define BOARD_BATTERY_ADC1_CHANNEL   5   // ADC1_CH5 <-> GPIO6 trên ESP32-S3 (channel N = GPIO(N+1))
// Hệ số chia áp (VBAT = Vadc * ratio) KHÔNG nằm ở đây — nó là hằng số VẬT LÝ
// suy thẳng từ 2 điện trở trên bo (R4=22k phía trên, R5=10k xuống GND), nên
// định nghĩa cạnh chính công thức dùng nó -> xem BATTERY_R_TOP_OHM /
// BATTERY_R_BOTTOM_OHM / BATTERY_DIVIDER_RATIO trong
// flight_core/drivers/battery_driver.h (kèm dải hợp lệ pin 1S).

// ================= RGB LED WS2812B (1 dây data) =================
// Chỉ khai báo pin ở lần config này — CHƯA có driver (LED trạng thái không
// nằm trong "Ràng buộc khởi tạo peripheral" của lần này). RMT-based WS2812
// driver làm sau nếu cần hiển thị trạng thái FSM.
#define RGB_LED_PIN     5    // IO5

// ================= Console / nạp code =================
// KHÔNG dùng UART0 (không có USB-UART bridge chip trên bo) — nạp firmware +
// debug console + REPL MicroPython + param tuning ĐI QUA USB D+/D- (GPIO19=
// D-, GPIO20=D+, chân USB PHY CỐ ĐỊNH của ESP32-S3, không qua GPIO matrix)
// dùng USB-Serial-JTAG controller có sẵn trong chip — KHÔNG cần thêm linh
// kiện/driver ngoài. KHÔNG gán bất kỳ peripheral nào khác vào IO19/20.
//
// sdkconfig app THẬT (MicroPython port) cần bật primary console qua kênh này
// (xem sdkconfig.esp32s3_flight_core_verify ở gốc repo đã đổi tương ứng):
//   CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y     (thay CONFIG_ESP_CONSOLE_UART_DEFAULT)
//   CONFIG_ESP_CONSOLE_SECONDARY_NONE=y
// Nạp code (esptool download-mode) qua cùng cổng USB này KHÔNG phụ thuộc
// sdkconfig — là tính năng của ROM bootloader, nhưng vì không có chip
// UART-bridge nào tự động toggle DTR/RTS để vào download-mode, PHẢI giữ nút
// BOOT (kéo GPIO0 xuống LOW) rồi nhấn/thả RESET để vào download-mode thủ công
// trước khi flash, nếu board không có mạch auto-reset riêng.
//
// UART0 vật lý (mặc định TXD0/RXD0, KHÔNG wire trên bo) coi như KHÔNG dùng.

// ================= Camera OV2640 (DVP 8-bit) =================
// ⚠ CHUA DIEN. Toan bo 16 chan duoi day dang la -1 = CHUA BIET, KHONG phai
// "khong dung". Doc tu SCHEMATIC/PCB that roi dien vao; TUYET DOI khong chep
// pinout ESP32-CAM AiThinker — day la bo custom, mapping khac han.
//
// Dien xong thi doi BOARD_CAM_PINS_CONFIGURED thanh 1. Bat camera ma co nay
// con 0 se LOI BIEN DICH (#error trong camera_driver.c) — co y: pin sai tren
// mot con drone dang bay khong phai "camera khong len", ma la ghi de chan
// motor hoac chan INT cua IMU.
//
// ---- Chan DANG BI CHIEM (khong duoc trung) ----
//   I2C   43,44   IMU INT 38   Flow SPI 40,41,42,1
//   Motor 2,3,4,48            VBAT ADC 6   RGB LED 5
// ---- Chan KHONG DUNG DUOC tren N16R8 ----
//   26..32  SPI flash
//   33..37  octal PSRAM (chi bi chiem KHI bat CONFIG_SPIRAM_MODE_OCTAL)
//   19,20   USB-Serial-JTAG (console + nap code)
//   0,45,46 strapping — tranh cho tin hieu toc do cao (PCLK/XCLK/data)
// ---- Con lai an toan: 7..18, 21, 39, 47 = 15 chan ----
// OV2640 DVP can 16 (hoac 14 neu PWDN/RESET noi cung ngoai). Tuc la mapping
// BI EP SAT BIEN: rat co the SIOC/SIOD dung chung bus I2C 43/44 voi
// MPU6050+VL53L1X. Neu dung vay thi dien 43/44 vao SIOC/SIOD va doc ky ghi
// chu ve tranh chap bus o cuoi khoi nay.
#define BOARD_CAM_PINS_CONFIGURED   1    // da dien du 14 chan (D0/D1 khong noi)

// ---- BUS DU LIEU: danh so theo SCHEMATIC (D2..D9), khong theo esp32-camera ----
// OV2640 co bus 10 bit. O che do 8 bit ma ta dung, D0/D1 KHONG DAU (dung nhu
// bo nay) va tam duong D2..D9 la bus that. esp32-camera lai danh so 8 duong
// DA DAU do tu 0 (pin_d0..pin_d7), tuc la LECH HAI so voi schematic:
//
//     schematic  D0 D1 | D2  D3  D4  D5  D6  D7  D8  D9
//     esp32-cam  -- -- | d0  d1  d2  d3  d4  d5  d6  d7
//
// Macro o day theo SO CUA SCHEMATIC de dien vao khong phai tru nham trong dau;
// viec doi sang pin_dN nam trong camera_driver.c, mot cho duy nhat.
// KHONG con CAM_D0/CAM_D1: chung khong noi, va de lai mot macro "-1" chi tao
// cho cho nguoi sau tuong la "chua dien".
#define CAM_D2_GPIO      (13)
#define CAM_D3_GPIO      (21)
#define CAM_D4_GPIO      (47)
#define CAM_D5_GPIO      (14)
#define CAM_D6_GPIO      (12)
#define CAM_D7_GPIO      (10)
#define CAM_D8_GPIO      (9)
#define CAM_D9_GPIO      (18)
#define CAM_PCLK_GPIO    (11)
#define CAM_VSYNC_GPIO   (16)
#define CAM_HREF_GPIO    (17)
#define CAM_XCLK_GPIO    (8)
#define CAM_SIOC_GPIO    (15)   // SCCB RIENG, khong phai bus I2C 43/44 cua IMU/ToF
#define CAM_SIOD_GPIO    (7)    // SCCB RIENG -> khong co tranh chap voi sensor_hub
#define CAM_PWDN_GPIO    (-1)   // NOI DAT tren bo -> luon bat, MCU khong tat duoc camera
#define CAM_RESET_GPIO   (-1)   // khong dua ra MCU -> chi reset duoc bang cach cat nguon

// PWDN/RESET: -1 la gia tri HOP LE that su voi esp32-camera (nghia la "khong
// dieu khien duoc, bo tu noi"). Nen camera_driver.c KHONG the dung -1 de biet
// "chua dien" — do chinh la ly do phai co BOARD_CAM_PINS_CONFIGURED rieng.
//
// ⚠ NEU SIOC/SIOD DUNG CHUNG I2C VOI IMU/ToF: esp32-camera tu dieu khien SCCB
// bang driver rieng cua no, khong di qua i2c_master handle ma sensor_hub dang
// giu. Hai master tren cung bus se dam nhau. Luc do PHAI dat
// CAMERA_SCCB_SHARES_I2C 1 trong app_config.h de camera_init() chay TRUOC khi
// sensor_hub start, va sau do KHONG doi thanh ghi camera nao trong luc bay.

// ================= PSRAM + Flash (N16R8: 16MB flash, 8MB octal PSRAM) =================
// KHÔNG cấu hình được qua macro C — bật qua sdkconfig của project MicroPython
// port THẬT (chưa nằm trong repo này, xem README.md phần "Build MicroPython
// port"). Project PlatformIO ở gốc repo (platformio.ini) CHỈ smoke-build
// flight_core (không cần PSRAM, xem comment đầu platformio.ini) nên KHÔNG
// đụng tới các key dưới đây.
// Ghi lại đây để không quên khi build port thật:
//   CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y
//   CONFIG_ESPTOOLPY_FLASHSIZE="16MB"
//   CONFIG_SPIRAM=y
//   CONFIG_SPIRAM_MODE_OCTAL=y
//   CONFIG_SPIRAM_SPEED_80M=y            (hoặc tốc độ khớp datasheet PSRAM thật)
//   CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y
