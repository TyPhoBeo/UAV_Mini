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

// IO36 INT_MPU — data-ready interrupt, ĐANG DÙNG làm đồng hồ nhịp của
// stabilize_task (xem imu_driver_enable_data_ready_int()). Đặt < 0 để quay về
// polling bằng đồng hồ FreeRTOS.
//
// ⚠ GPIO33..37 trên ESP32-S3 là bus SPI cho PSRAM octal. Module N16R8 CÓ chip
// PSRAM nối vào đúng nhóm chân này, nên IO36 chỉ dùng được vì PSRAM đang TẮT
// trong sdkconfig (`CONFIG_SPIRAM is not set`). BẬT PSRAM lên là chân này bị
// chiếm và ngắt IMU sẽ hỏng — lúc đó phải dời INT sang chân khác, đừng đi tìm
// lỗi ở MPU6050.
//
// (Giá trị cũ trong file này là IO16 — số đó là phỏng đoán chưa từng xác nhận;
// IO36 là số đo từ phần cứng thật.)
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

// ================= VL53L0X ToF (MỘT con, hướng xuống) =================
// ⚠ CHIP LÀ VL53L0X, KHÔNG PHẢI VL53L1X (mục này trước đây ghi nhầm L1X).
// Hai họ khác hẳn nhau: register map khác, MODEL_ID khác (L0X=0xEE tại 0xC0),
// tầm tối đa khác (~2m vs ~4m). Driver trong repo là VL53L0X (Pololu-style
// init) — đừng dựa vào cái tên cũ khi tra datasheet.
//
// ĐÃ BỎ con ToF thứ hai (forward/dự phòng). Nó chưa bao giờ được fuse vào
// control loop nhưng bắt driver phải mang những ràng buộc chỉ có nghĩa khi có 2
// chip (addr1 không được giữ 0x29, cả 2 XSHUT phải điều khiển được, thứ tự
// bring-up bắt buộc) — và chính những ràng buộc đó đã gây lỗi thật cho cấu hình
// 1 chip. Xem tof_driver.h mục "ĐÃ ĐƠN GIẢN HOÁ TỪ 2 SENSOR VỀ 1".
//
// Địa chỉ: giữ 0x29 MẶC ĐỊNH. Chỉ 1 con nên không ai tranh địa chỉ — đổi sang
// 0x2A chỉ thêm một bước có thể thất bại mà không được lợi gì.
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
#define BOARD_TOF_I2C_FREQ_HZ 100000

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
// góc. attitude_control.h giả định thứ tự M1=front-left, M2=front-right,
// M3=back-right, M4=back-left (Quad-X). SAI mapping ở đây -> lật ngay khi arm.
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

// ================= Camera OV2640 =================
// Có trên bo nhưng KHÔNG dùng ở firmware bay -> KHÔNG khai báo pin, bỏ qua
// hoàn toàn (đúng yêu cầu).

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
