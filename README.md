# UAV-S3 — Flight controller 2 tầng (C + MicroPython) cho ESP32-S3-WROOM-1 (miniUav)

Kiến trúc 2 tầng + logic điều khiển đã PORT (không phải stub rỗng) từ dự án
UAV-Mini (C++, đã bay/debug thật trên ESP32 WROOM). **Pin GPIO đã điền đúng
theo schematic thật (Schematic_miniUav1 rev1.0)** — không còn placeholder.
Driver IMU/mag/ToF-addressing đã có register map thật (không còn STUB rỗng) —
phần còn thiếu là bảng tuning ST ULD cho ToF (xem mục "Hạn chế đã biết" #5) và
xác nhận vị trí vật lý motor.

## Trạng thái hiện tại (đọc mục này trước)

| Phần | Trạng thái |
|---|---|
| Kiến trúc 2 tầng, command queue / telemetry snapshot | ✅ Xong |
| FlightStateMachine + Commander (safety) | ✅ Xong, logic port thật |
| PID/Mahony/alt_hold/takeoff/landing | ✅ Xong, gain copy từ UAV-Mini (cần tune lại — xem hạn chế) |
| `tuning.h` (1 file gom hết tham số PID/tune) | ✅ Xong |
| `main/board_config.h` — pin thật theo schematic | ✅ Xong |
| `main/app_config.h` — bật/tắt cảm biến + xác nhận vị trí motor | ✅ Xong |
| `motor_driver.c` — LEDC 4 kênh, 24kHz/11-bit | ✅ Xong |
| `battery_driver.c` — ADC1 oneshot + calibration | ✅ Xong |
| `imu_driver.c` — MPU6050 (WHO_AM_I, DLPF, FS range, đọc 6 trục, data-ready INT) | ✅ Xong |
| `mag_driver.c` — QMC5883P (chip-ID, Normal 100Hz, verify DRDY+mẫu thật lúc init, publish seq cho consumer) | ✅ Xong |
| `baro_driver.c` — BMP280 (chip-ID, calib NVM, compensation Bosch, fuse alt_estimator) | ✅ Xong |
| `tof_driver.c` — bring-up XSHUT + đổi địa chỉ I2C (2 con, tof2 tắt được qua app_config.h) | ✅ Xong |
| `tof_driver.c` — giao thức ranging (start/stop/đọc RangeStatus) | ✅ Xong |
| `tof_driver.c` — bảng tuning ST ULD (VHV/crosstalk/offset) | ⚠️ CHƯA — xem hạn chế #5, PHẢI bench-test |
| Remap trục IMU/mag theo hướng lắp thật | ⚠️ CHƯA xác nhận — map thẳng x/y/z tạm |
| Console/nạp code qua USB D+/D- (không qua UART0) | ✅ Đã cấu hình (`sdkconfig` + `platformio.ini`) |
| Flash size 16MB (N16R8) trong `platformio.ini`/`sdkconfig` | ✅ `board = 4d_systems_esp32s3_gen4_r8n16` (khớp N16R8 sẵn, không cần override tay) |
| `src/main.c` — firmware điều khiển THẬT qua console USB (arm/takeoff/land/...) | ✅ Xong, KHÔNG cần MicroPython (xem mục "Bay qua console USB") |
| Vị trí vật lý motor CH1..CH4 trên khung X | ❌ **CHƯA xác nhận — chặn bay an toàn.** Công cụ đã có (`test_motor`, xem app_config.h), còn thiếu bước LÀM trên phần cứng thật |
| `platformio.ini` ở GỐC repo — mở UAV-S3/ là build/nạp được ngay | ✅ SUCCESS (Flash ~26%, RAM ~5%), không cần thư mục con nào khác |
| Build MicroPython port thật | ❌ Chưa làm (ngoài phạm vi repo này — KHÔNG cần thiết để bay, xem console USB) |

## Kiến trúc: ĐÚNG 2 tầng

```
┌──────────────────────────────────────────────────┐
│  TẦNG TRÊN — CORE 0 (mọi task application/net)   │
│  python/fc_api.py  (blocking wrapper)            │
│  micropython_module/fc/  (C bridge, mp_obj)      │
│  hoặc: net / udp_rx / console REPL (src/)        │
└───────────────┬───────────────┬──────────────────┘
                │ command_t      │ telemetry_snapshot_t
                │ (xuống)        │ (lên)
┌───────────────▼───────────────▼──────────────────┐
│  TẦNG DƯỚI — CORE 1, CHỈ vòng bay realtime       │
│  components/flight_core/                         │
│    stabilize (prio 23): Mahony → AltEstimator →  │
│      FSM/Commander → cascade + mixer → motor     │
│    sensor_hub (prio 22): SỞ HỮU DUY NHẤT I2C     │
└──────────────────────────────────────────────────┘
```

**Nguyên tắc bất di**: MicroPython là NGUỒN LỆNH, không phải thành phần điều
khiển. Không một phép toán điều khiển nào (Mahony/PID/mixer/state machine/
safety) nằm trong file `.py` hay `fc_module.c` — toàn bộ nằm trong
`components/flight_core` (C thuần, không include gì từ MicroPython hay `main/`).

**Biên giới CHỈ GỒM 2 KÊNH** (xem `flight_core/flight_core.h`):
- **Xuống**: `flight_core_push_command()` — đẩy `command_t` vào FreeRTOS queue
  (`COMMAND_QUEUE_DEPTH=8`). Đầy queue → trả `false` → `fc_module.c` raise
  `RuntimeError` phía Python (KHÔNG nuốt lệnh âm thầm).
- **Lên**: `flight_core_read_telemetry()` — copy `telemetry_snapshot_t` qua
  mutex nhẹ (chỉ giữ trong lúc `memcpy`, không phải khóa logic).

CHỈ `stabilize_task` (trong `flight_core.c`, pin CỨNG core 1 qua
`xTaskCreatePinnedToCore`) được ghi control state và telemetry — **1 writer,
không race, không cần khóa nặng**.

**Core 1 chỉ có 2 task, cả hai đều là vòng bay.** Mọi task do firmware tạo ở
tầng application/network (`net`, `udp_rx`, console REPL) đều pin CỨNG core 0 —
`xTaskCreate()` trần **không đủ**, vì trên build dual-core nó tương đương
`tskNO_AFFINITY` và scheduler được phép đẩy task lên core 1. Task hệ thống cũng
vậy: lwIP đã được đặt `CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU0=y` trong `sdkconfig`
(mặc định của IDF là NO_AFFINITY). Khi build MicroPython port thật, VM task phải
pin core 0 theo đúng nguyên tắc này. Bảng đầy đủ: `flow.md` §3.1.

Ngoại lệ duy nhất chạm phần cứng từ core 0 là **KILL**
(`flight_core_kill_now()` → `motor_driver_all_off()`). Vì nó ghi PWM song song
với core 1, `motor_driver.c` bọc "đọc armed gate + ghi 4 kênh" trong một
`portMUX_TYPE` — xem `flow.md` §3.7.3.

`fc_bridge_init()` (`micropython_module/fc/fc_bridge.c`) là nơi DUY NHẤT đọc
`main/board_config.h` và build `flight_core_board_config_t` truyền vào
`flight_core_start()` — xem luồng đầy đủ trong `flow.md`.

## Vì sao C (không phải C++) cho tầng dưới

Quyết định của người dùng: lo ngại C++ (RTTI/exception ẩn/vtable/heap runtime
của libstdc++) cộng dồn với MicroPython VM sẽ chật RAM/flash trên cùng một
chip. Toàn bộ `components/flight_core` viết C thuần (struct + hàm, không
class/exception/STL).

## Logic đã PORT (không phải TODO rỗng) — từ UAV-Mini, đã bay/debug thật

| Module | Port từ (UAV-Mini) | File |
|---|---|---|
| Mahony filter | `mahony_filter.cpp` | `mahony_filter.c` |
| PID generic (anti-windup, D-on-measurement, freeze) | `motor_output.cpp::pid_update()` | `pid.c` |
| Cascade angle→rate + Quad-X mixer (desaturation) | `motor_output.cpp::update_balance_quad_x()` | `attitude_control.c` |
| Alt estimator (accel-primary + ToF/baro anchor) | `alt_estimator.cpp` (nguyên lý kiến trúc, KHÔNG copy số) | `alt_estimator.c` |
| Cascade giữ độ cao (alt→vz→PI) | nhánh `ALT_MODE_HOLD` | `alt_hold.c` |
| Takeoff (spool ramp, liftoff-latch, Ki-freeze) | `run_takeoff_sequence()` | `takeoff_land.c` |
| Landing (descend/flare/touchdown/blind) | `run_landing_sequence()` | `takeoff_land.c` |
| Motor LEDC driver (24kHz, 11-bit, khớp KAO3400) | `motor_output.cpp::init()` | `drivers/motor_driver.c` |
| Battery ADC (oneshot + calibration) | — (mới, board S3 chưa có ở UAV-Mini) | `drivers/battery_driver.c` |
| MPU6050 register map (WHO_AM_I, DLPF, FS range) | `MPUDriver` (UAV-Mini dùng chip khác, cùng họ MPU-6xxx) | `drivers/imu_driver.c` |
| QMC5883P register map (continuous mode, đọc 3 trục) | — (mới, board S3 mới có mag) | `drivers/mag_driver.c` |
| ToF x2 bring-up (XSHUT sequencing + đổi địa chỉ I2C) + giao thức ranging | — (mới, UAV-Mini chỉ 1 ToF) | `drivers/tof_driver.c` |

## `tuning.h` — MỘT file duy nhất để chỉnh PID/tham số bay

`components/flight_core/include/flight_core/tuning.h` gom **toàn bộ** hằng số
tune (angle/rate PID roll-pitch-yaw, alt_hold, takeoff, landing, commander
safety) — tương đương `tuning.hpp` của UAV-Mini gốc. Muốn chỉnh gì cũng sửa
Ở ĐÂY, build lại là áp dụng. Gain hiện tại là **copy y hệt UAV-Mini** (board
cũ, motor/prop/pin/khối lượng khác) — PHẢI tune lại trên phần cứng S3 thật,
đặc biệt `ALT_HOLD_HOVER_NOMINAL` và `TAKEOFF_PRIME_DUTY`.

### FlightStateMachine (`flight_state_machine.h`)

```
DISARMED → ARMED          : arm hợp lệ (attitude.valid && |tilt|<10°)
ARMED → TAKING_OFF        : lệnh takeoff
ARMED → DISARMED          : disarm
TAKING_OFF → HOLDING      : chuỗi cất cánh hoàn tất (TKO_HOLD bàn giao tại target)
TAKING_OFF → ARMED        : abort TRƯỚC liftoff (NO_LIFTOFF/TIMEOUT/EST_INVALID)
TAKING_OFF → LANDING      : abort SAU liftoff (đang trên không) HOẶC soft fault
HOLDING ⇄ FLYING          : có/hết move command
HOLDING/FLYING → LANDING  : lệnh land HOẶC soft fault (Commander)
HOLDING/FLYING → EMERGENCY: hard fault (Commander)
LANDING → DISARMED        : đã chạm đất
EMERGENCY → LANDING       : còn kiểm soát được (attitude/alt hợp lý)
EMERGENCY → DISARMED      : mất kiểm soát HOẶC còn ở mặt đất (không có gì để "hạ")
```

`EMERGENCY` là state **transient** — được giải quyết ngay trong CÙNG tick nó
xuất hiện (bước 7 trong `stabilize_task`, `flight_core.c`).

### Commander (`commander.h`)

Trọng tài an toàn, chạy mỗi tick khi `HOLDING`/`FLYING`:
- **Geofence**: `commander_clamp_altitude()` — MỌI đường ghi `alt_target_m` đều
  qua hàm này trước khi vào setpoint.
- **Battery floor** (`COMMANDER_DEFAULT_BATTERY_FLOOR_V` trong `tuning.h`) →
  SOFT fault → LANDING. Đọc từ `battery_driver.c` thật mỗi tick (xem bảng
  trạng thái — đã hết TODO).
- **Heartbeat watchdog** (mặc định 1000ms) — Python script treo là chuyện
  thường, phải hạ êm, KHÔNG cắt máy giữa trời → SOFT. Nguồn nuôi watchdog này
  **CHỈ đến từ bên ngoài chip** (gói UDP, `p` từ GUI, `fc.heartbeat()`, lệnh
  console `heartbeat`) — firmware **không** tự sinh heartbeat, xem `flow.md`
  §4.1 "Heartbeat watchdog".
- **Hard fault** (IMU invalid, `|tilt| > 60°`, motor bão hòa >1.5s liên tục)
  → EMERGENCY.
- **Alt estimator mất giữa chừng khi đang bay** → SOFT fault → LANDING.

**Chỉnh geofence/ngưỡng fault LIVE qua UDP** (`tools/uav_udp_console.py` panel
"Commander / Failsafe", hoặc gõ tay): `@CMDR GET` trả về 6 giá trị hiện tại,
`@CMDR SET <altmin_m> <altmax_m> <battfloor_v> <hbtimeout_ms> <hardtilt_deg>
<motorsat_ms>` áp dụng ngay — KHÁC `@TKO`/`@LAND`, lệnh này chạy được BẤT KỲ
LÚC NÀO kể cả đang bay (geofence/failsafe không có lý do gì phải chờ
DISARMED). Xem `command_parser.c::handle_cmdr()` + `flight_core.c`
`CMD_SET_COMMANDER_CFG`. Console USB KHÔNG có lệnh tương đương (chỉ UDP).

### Khi nào I-term (Ki) của PID attitude được cộng dồn

I-term **chỉ tích lũy khi CẢ HAI gate độc lập cùng mở** — telemetry `KI=` trong
STATUS line (GUI hiện `Ki=DUNG/KHOA`) báo đúng kết quả AND này:

**Gate 1 — trạng thái bay (FSM):**

| FSM state | Ki |
|---|---|
| `DISARMED`, `ARMED` | KHÓA (ga=0, chưa có gì để ổn định) |
| `TAKING_OFF` | KHÓA cho tới khi **`liftoff_flag`** lên |
| `HOLDING`, `FLYING`, `LANDING` | MỞ |

`airborne` = `liftoff_flag` đã lên (`est_z > ground_alt_m +
TAKEOFF_LIFTOFF_DELTA_M`, xem `flow.md` §4.2) HOẶC FSM đã ở state hậu-cất-cánh —
KHÔNG phải "đã hết pha PRIME".

Gate này dùng `liftoff_flag` chứ **không** dùng "đã qua PRIME": giữa lúc rời
`TKO_PRIME` và lúc thật sự nhấc, controller đã đẩy ga lên nhưng drone **vẫn đang
đè mặt đất** — đó chính là cửa sổ dễ **ground windup** nhất. Ki cộng dồn để sửa
một sai số mà **phản lực nền** (chứ không phải motor) đang giữ; I-term phình to
rồi bung ra đúng khoảnh khắc rời đất → drone giật/lật ngay giây đầu.

**Ki ALTITUDE có gate RIÊNG, hoàn toàn độc lập**: nó **giới hạn** `|I|` (không
đóng băng) trước liftoff, vì cascade Vz vẫn cần quyền đẩy ga lên để **tìm** điểm
nhấc — `hover` chỉ là ước lượng. Xem `TAKEOFF_PRELIFT_I_LIMIT_DUTY` trong
`tuning.h`.

**Gate 2 — ngưỡng ga** (`ATT_I_ENABLE_THROTTLE_DUTY`, `tuning.h`): ga thấp thì
motor chưa đủ thẩm quyền tạo mô-men sửa sai số, cộng dồn chỉ sinh windup rồi bung
khi ga lên. So trên duty thực gửi ra motor.

Phân biệt với `ATT_MIN_THROTTLE_DUTY` (cũng =200 mặc định, nhưng **khác vai
trò**): dưới ngưỡng đó PID **không chạy chút nào** và integrator bị **reset
sạch**; còn `ATT_I_ENABLE_THROTTLE_DUTY` chỉ **freeze** I (P/D vẫn chạy ổn định
drone, thả ga rồi kéo lên lại KHÔNG mất trim đã học). Tách 2 hằng số để nâng
riêng ngưỡng Ki khi tune (vd 200 → 400) mà không đụng ngưỡng chạy PID.

Xem `flow.md` (đã tạo sẵn ở gốc repo) cho sơ đồ tuần tự đầy đủ + flowchart
`stabilize_task` nếu cần hình dung sâu hơn code.

## `main/board_config.h` — pin thật theo schematic miniUav

| Nhóm | Pin | Ghi chú |
|---|---|---|
| I2C (IMU+mag+ToF+baro) | SDA=IO44, SCL=IO43 | pull-up 2k2 có sẵn trên bo + internal pull-up ESP32-S3; new I2C master API (`driver/i2c_master.h`), 1 bus dùng chung, xem mục "Kiến trúc I2C" dưới |
| MPU_INT | IO36 | data-ready interrupt — ĐANG DÙNG làm đồng hồ nhịp `stabilize_task` (xem mục "Nhịp vòng điều khiển") |
| ToF1 XSHUT / ToF2 XSHUT | IO17 / IO18 | bring-up tuần tự, đổi địa chỉ ToF1→0x2A |
| Flow PMW3901 (SPI) | MOSI=IO2, MISO=IO41, SCLK=IO42, CS=IO40 | chỉ khai báo pin, CHƯA init bus |
| Motor CH1..CH4 | IO4, IO8, IO39, IO1 | **vị trí vật lý CHƯA xác nhận, xem dưới** |
| Battery ADC | IO6 → ADC1_CH5 | tỷ lệ chia 3.2 |
| RGB LED WS2812 | IO5 | chỉ khai báo pin, CHƯA có driver |
| BMP280 (baro) | I2C, addr 0x77 (đã xác nhận qua scan — SDO=VCC) | driver thật, sửa trôi dài hạn cho alt_estimator |
| QMC5883P (mag) | I2C, addr 0x2C (cố định, đã xác nhận qua scan) | **KHÁC HẲN QMC5883L** giả định ban đầu — register map/chip-ID khác |
| Console/nạp code | USB D+/D- (IO19/IO20, USB-Serial-JTAG) | **KHÔNG phải UART0** — xem mục dưới |
| Camera OV2640 | — | có trên bo, KHÔNG dùng, bỏ qua |

⚠️ **Vị trí vật lý CH1..CH4 CHƯA xác nhận.** `board_config.h`/`fc_bridge.c`
đang gán tạm CH1→M1, CH2→M2, CH3→M3, CH4→M4 theo đúng THỨ TỰ NET trên
schematic — KHÔNG chắc CH1 là góc nào của khung X hay chiều quay CW/CCW.
`attitude_control.h` giả định M1=front-left, M2=front-right, M3=back-right,
M4=back-left (Quad-X). **Sai mapping ở đây = lật ngay khi arm, không cứu được
bằng tune.** Quy trình xác nhận AN TOÀN (tháo cánh quạt + lệnh `test_motor`
qua console, KHÔNG suy đoán) — xem mục "`main/app_config.h`" ngay dưới đây.

## Kiến trúc I2C — 1 bus dùng chung, new I2C master API

Toàn bộ `imu_driver.c`/`mag_driver.c`/`baro_driver.c`/`tof_driver.c` đã
migrate từ legacy `driver/i2c.h` (EOL từ ESP-IDF v6.0, sẽ bị xóa ở v7.0) sang
`driver/i2c_master.h` (new I2C master API). Kiến trúc:

```
ESP32-S3
GPIO43 = SCL, GPIO44 = SDA
        |
        +-- i2c_master_bus_handle_t (s_i2c_bus, flight_core.c, TẠO 1 LẦN DUY NHẤT)
              +-- MPU6050   device handle (imu_driver.c)
              +-- QMC5883P  device handle (mag_driver.c)
              +-- BMP280    device handle (baro_driver.c)
              +-- VL53L1X x2 device handle (tof_driver.c, đổi địa chỉ động qua
                  i2c_master_device_change_address() sau khi bring-up XSHUT)
```

Nguyên tắc (chống 1 sensor lỗi làm treo cả bus — vấn đề thực tế đã gặp trước
khi refactor):
- `flight_core.c::init_i2c_bus()` là nơi DUY NHẤT gọi `i2c_new_master_bus()`.
  Không driver nào tự tạo bus riêng.
- Mỗi driver chỉ `i2c_master_bus_add_device()` để lấy device handle CỦA
  RIÊNG NÓ — init/read lỗi của 1 driver KHÔNG đụng tới handle/bus của driver
  khác (khác object hoàn toàn).
- KHÔNG driver nào gọi `i2c_del_master_bus()` hay tạo lại bus khi gặp lỗi đọc
  — lỗi được trả (`esp_err_t`) lên `flight_core.c`, không tự ý loop-reset bus.
- Timeout truyền trực tiếp bằng ms cho `i2c_master_transmit()`/
  `i2c_master_transmit_receive()` (không dùng `pdMS_TO_TICKS()` — API mới
  nhận ms thẳng).

**Giai đoạn DEBUG**: `BOARD_I2C_FREQ_HZ` (dùng cho bus) + `*_I2C_SCL_SPEED_HZ`
(macro riêng từng driver, hiện đều `100000`) đang set **100kHz** để xác nhận
MPU6050+QMC5883P+BMP280 chạy ổn định trên kiến trúc bus mới trước. Sau khi
bench-test ổn ở 100kHz, nâng lên `400000` (đổi CẢ `BOARD_I2C_FREQ_HZ` trong
`board_config.h` LẪN 4 macro `*_I2C_SCL_SPEED_HZ` trong từng driver — chưa có
1 điểm cấu hình chung, xem TODO nếu muốn gộp lại).

**BMP280 — init tách khỏi lấy mốc 0m**: `baro_driver_init()` chỉ đưa chip vào
trạng thái vận hành được (probe CHIP_ID, chờ NVM copy xong qua `STATUS.im_update`,
validate bảng hiệu chỉnh, ghi CONFIG/CTRL_MEAS + read-back verify) — **KHÔNG**
lấy mốc 0m nữa. `baro_driver_calibrate_ground()` là bước riêng (chờ 300ms ổn
định + lấy trung bình 32 mẫu hợp lệ), `flight_core.c` gọi ngay sau init lúc
boot; `baro_driver_read()` trả `ok=false` cho tới khi bước này thành công.
Gọi lại `baro_driver_calibrate_ground()` (vd ngay trước ARM) để reset mốc nếu
drone bị di chuyển sau boot là cải tiến khả dĩ sau này — **CHƯA làm** ở lần
sửa này (cần hook vào xử lý lệnh ARM trong `flight_core.c`, ngoài phạm vi
thuần túy I2C/driver). `baro_driver_read()` cũng được throttle xuống ~50Hz
(`BARO_READ_TICK_DIVISOR` trong `flight_core.c`) thay vì gọi mỗi tick 250Hz.

CMakeLists.txt của `flight_core` đã đổi `REQUIRES driver` (umbrella, chỉ cần
cho legacy `i2c.h`) thành `REQUIRES esp_driver_i2c` (component chứa
`i2c_master.h`).

## `main/app_config.h` — bật/tắt cảm biến + xác nhận vị trí động cơ

File THỨ HAI (cùng `main/board_config.h`) mà `fc_bridge.c` và `src/main.c`
include khi build `flight_core_board_config_t`. `board_config.h` trả lời
"chân nào" (pin/địa chỉ I2C); `app_config.h` trả lời "cái nào ĐANG DÙNG":

```c
#define SENSOR_IMU_ENABLED       1   // MPU6050 — BẮT BUỘC
#define SENSOR_MAG_ENABLED       0   // QMC5883P — chống trôi yaw (BẬT sau khi calib_mag xong, xem mục Calibration)
#define SENSOR_TOF_ENABLED       1   // VL53L0X hướng xuống — nguồn correction có điều kiện cho alt_estimator
#define SENSOR_BARO_ENABLED      1   // BMP280 — sửa trôi dài hạn cho alt_estimator
#define SENSOR_FLOW_ENABLED      0   // PMW3901 — chưa có driver
#define SENSOR_RGB_LED_ENABLED   0   // WS2812 — chưa có driver
#define SENSOR_BATTERY_ENABLED   1   // ADC1
#define SENSOR_CAMERA_ENABLED    0   // OV2640 — không dùng cho firmware bay
```

(`SENSOR_TOF2_ENABLED` đã bị bỏ hẳn — con ToF thứ hai chưa bao giờ được fuse vào
control loop, xem `tof_driver.h`.)

(Giá trị thật lấy nguyên từ `main/app_config.h` tại thời điểm viết mục này — file đó luôn là nguồn sự thật, đổi cờ ở đó rồi build lại, KHÔNG sửa README.)

Đổi cờ nào xuống `0` là `flight_core_start()` **bỏ qua hẳn** việc init driver
tương ứng (không dò I2C, không log warning nhiễu) — build lại (`pio run`) là
áp dụng, không cần sửa `flight_core.c`.

**Xác nhận vị trí vật lý động cơ** (bắt buộc trước khi bay, KHÔNG suy đoán):

1. **Tháo hết cánh quạt** (bắt buộc, an toàn tuyệt đối).
2. Nạp firmware (xem mục "Bay qua console USB" dưới đây), đảm bảo
   `status` báo `state=DISARMED`.
3. Gõ `test_motor 1 15` (motor 1, 15% duty, tự dừng sau ~0.5s — CHỈ chạy khi
   DISARMED, `flight_core.c::apply_command()` tự chặn nếu không).
4. Quan sát cánh nào quay, ghi lại góc khung tương ứng, lặp lại cho
   `test_motor 2/3/4`.
5. Nếu thứ tự vật lý khác thứ tự NET giả định, sửa lại thứ tự gán
   `motor_gpio[0..3]` trong `fc_bridge.c` và `src/main.c` cho khớp
   M1=front-left/M2=front-right/M3=back-right/M4=back-left.
6. Đổi `MOTOR_POSITIONS_CONFIRMED` trong `app_config.h` xuống `1` — cờ này
   không chặn build/nạp, chỉ khiến `src/main.c` in cảnh báo mỗi lần boot nếu
   còn là `0`.

## ⚠️ Hạn chế đã biết (kế thừa từ UAV-Mini, CHƯA giải quyết ở đây)

0. **Kiến trúc altitude là accel-primary, với HAI nguồn correction: ToF và
   baro.** Vz đến từ tích phân gia tốc thế giới (world-frame, gravity-removed);
   VL53L0X hướng xuống (`SENSOR_TOF_ENABLED=1`) và BMP280 chỉ là **anchor** kéo
   trôi qua correction alpha/beta nhỏ — KHÔNG BAO GIỜ lấy đạo hàm của chúng làm
   Vz, KHÔNG set thẳng estimate = số đo mỗi mẫu. ToF chính xác hơn nhiều (~cm so
   với ~±0.3-1m của baro) nhưng **có tầm giới hạn** (~1.8m) và **đo theo bề mặt
   ngay dưới** (bay qua bàn/ghế là đổi bề mặt) — vì vậy baro vẫn cần cho bay
   cao/bay lâu. Mất **cả hai** nguồn quá `ALT_EST_NO_CORRECTION_DEGRADED_MS` thì
   Commander tự LANDING. `ALT_HOLD_MIN_ENGAGE_M`/`LAND_TOUCHDOWN_ALT_M` phụ
   thuộc Z tuyệt đối, **CÓ THỂ cần nới rộng** nếu bench-test thấy noise gây
   false-trigger. Bench-test kỹ (xem 7 test A-G ở mục altitude estimator) trước
   khi tin takeoff/landing tự động.
1. **Yaw trôi nếu `mag_driver.c` không được BẬT dùng.** `mag_driver.c` đã có
   register map QMC5883P thật, nhưng `flight_core.c` chỉ coi mag hợp lệ khi
   `mag_driver_read()` trả `ok=true` — nếu board không cắm mag hoặc chip lỗi,
   tự động rơi về "không mag" (đúng thiết kế, an toàn). `ATT_GAIN_ANGLE_YAW_KP
   = 0` mặc định (heading-hold tắt, yaw thuần rate) — không mag → bias gyro
   trục Z KHÔNG bao giờ được sửa → drone xoay chậm dù `yaw_rate` lệnh = 0.
2. ~~`takeoff(mm)` không leo thẳng tới `mm`~~ — **ĐÃ SỬA.** `CMD_TAKEOFF` mang
   theo `final_target_m` và firmware chạy trọn `TKO_PRIME → TKO_CLIMB →
   TKO_HOLD` tới đúng target rồi mới bàn giao sang `HOLDING`.
   `python/fc_api.py::takeoff()` giờ **chỉ poll trạng thái**, không gửi lệnh
   `set_altitude()` thứ hai. Xem `flow.md` §4.2.
3. **`set_yaw(deg)` là open-loop theo thời gian**, không phải absolute
   heading (cần mag + heading-hold, xem mục 1).
4. Gain hover/PID trong `tuning.h` là điểm khởi đầu từ board CŨ (WROOM-32) —
   PHẢI đo lại `ALT_HOLD_HOVER_NOMINAL` và tune lại `TAKEOFF_PRIME_DUTY`/
   `prime_ms` trên phần cứng S3 thật trước khi tin cascade độ cao.
5. ~~ToF ranging THIẾU bảng tuning ST ULD~~ — **MÔ TẢ NÀY ĐÃ SAI, đã sửa.**
   `tof_driver.c` hiện có **đủ** bảng "default tuning settings" (80 cặp
   reg/val trong `VL53L0X_TUNING[]`, khớp bản tham chiếu ST ULD/Pololu) và
   chuỗi init đầy đủ: DataInit (2V8 mode + stop_variable) → StaticInit
   (`get_spad_info()` + ref SPAD map + tuning) → cấu hình ngắt new-sample →
   `set_measurement_timing_budget(33ms)` → **ref calibration VHV + phase** →
   continuous back-to-back. Marker `TOF_CONFIG_TODO` **không còn tồn tại**
   trong source. **BẮT BUỘC vẫn phải bench-test** (so thước đo thật ở vài
   khoảng cách, lệnh `tof_test`) trước khi tin alt_hold dùng ToF để bay —
   nhưng lý do là chưa hiệu chuẩn offset/crosstalk cho *tấm che cụ thể* trên
   khung, KHÔNG phải vì thiếu bảng tuning.
6. **Remap trục IMU/mag CHƯA xác nhận.** `imu_driver.c`/`mag_driver.c` map
   thẳng x/y/z sensor→body (chưa biết hướng lắp thật trên khung). Sai remap
   KHÔNG gây lật ngay như sai motor mapping, nhưng Mahony sẽ hội tụ sai chiều
   roll/pitch/yaw — kiểm tra bằng cách nghiêng bo trên bàn và so số đọc.

## Altitude estimator (accel-primary + ToF/baro anchor)

`SENSOR_TOF_ENABLED=1` — board **CÓ** VL53L0X hướng xuống và `alt_estimator.c`
fuse nó thật. Kiến trúc:

```
MPU6050 → accel calib (6-face) → body→world (quaternion Mahony)
        → trừ gravity → LPF (mềm) → trừ accel_bias_ms2 → Az_world
        → tích phân "constant-acceleration": vz_old=vz; vz+=az*dt; z+=vz_old*dt+0.5*az*dt²
                                                              ▲
VL53L0X → surface gating (FLOOR/OBJECT) ──────────────────────┤ correction
BMP280  → median-of-3 → LPF → innovation gate ────────────────┘ alpha/beta
```

`dt` ở đây là **`fusion_dt`** — khoảng giữa hai **mẫu IMU**, không phải giữa hai
tick. Tick không có mẫu mới thì `alt_estimator_update()` **không được gọi** (xem
`flow.md` §3.4b).

**Lớp lọc rung động cơ — phần cứng TRƯỚC, phần mềm SAU** (vì Vz chủ yếu dựa vào
tích phân accel, ToF/baro chỉ kéo lại CHẬM):
1. **DLPF phần cứng MPU6050** (`tuning.h` mục 11, `IMU_ACCEL_DLPF_CFG=3` →
   44Hz, ghi thẳng thanh ghi CONFIG 0x1A) — lọc TRƯỚC KHI số vào MCU, lớp
   phòng thủ CHÍNH.
2. LPF phần mềm trong `alt_estimator.c` (`ALT_EST_VERT_ACC_LPF_HZ=10Hz`) +
   deadband (`ALT_EST_VERT_ACC_DEADBAND_MS2=0.05`) — lớp phòng thủ THỨ 2.

**3 state (alpha-beta-gamma filter làm tay)**: `alt_m` (alpha), `vz_ms`
(beta), `accel_bias_ms2` (gamma — sai số accel còn sót lại SAU calib 6-face,
world-frame, học CHẬM `ALT_EST_ACCEL_BIAS_LEARN_HZ=0.03Hz` CHỈ khi DISARMED
và đứng yên thật, kèm ZUPT — `vz_ms` ép về 0 mỗi tick lúc đó).

**Baro correction** (CHỈ khi có mẫu MỚI — xem "kiến trúc task" dưới):
`alt_m += ALT_EST_BARO_ALPHA * innov` (0.03), `vz_ms += ALT_EST_BARO_BETA *
innov / dt_baro` (0.003) — RẤT nhỏ có chủ đích, baro CHỈ kéo trôi dài hạn,
KHÔNG được làm throttle rung theo từng mẫu baro. Gate `ALT_EST_BARO_
INNOVATION_GATE_M=0.6m` chặn hẳn spike (nghi propwash/airflow). CẢ 3 hằng số
CHƯA đo trên phần cứng thật — tune bằng log thật (xem 7 test A-G dưới).

**🔹 Kiến trúc task**: estimator chạy trong `stabilize_task` (core 1, prio 23);
**mọi** transaction I2C — kể cả baro/ToF — nằm trong `sensor_hub` (core 1, prio
22). Vòng bay chỉ `memcpy` snapshot, không chạm bus. "Không reuse 1 mẫu nhiều
lần" thỏa qua **`seq`** của từng cảm biến: `alt_estimator_update()` nhận `seq` +
`timestamp` THÔ của baro/ToF và tự quyết định mẫu nào đã tiêu thụ (xem
`alt_estimator.h` "CORRECT"). Không còn `BARO_READ_TICK_DIVISOR` — đếm tick chỉ
đúng khi nhịp đọc trùng khít bội số nhịp điều khiển, còn `seq` đúng kể cả khi
hub bị trễ hay bỏ lỡ mẫu.

**Liftoff detect là một ngưỡng Z đơn giản**: `est_z > ground_alt_m +
TAKEOFF_LIFTOFF_DELTA_M` (0.05m) → `liftoff_flag` lên. Nó **không gate gì cả** —
không đổi controller, không đổi phase, không mở/khoá estimator; chỉ dùng cho
telemetry, nới trần `|I|`, và cho EMERGENCY biết drone đang trên không hay còn ở
đất.

> Bản trước dùng bộ chấm điểm **4 bằng chứng** (THR/ACC/VZ/Z, ≥3/4) vì hồi đó cờ
> này gate cả estimator lẫn Ki altitude — báo sai là hỏng cả chuyến bay. Giờ nó
> không gate gì nữa nên một ngưỡng Z là đủ và dễ suy luận hơn nhiều. Trường
> telemetry `LSC` (liftoff score) vì vậy **luôn 0**, giữ lại chỉ để GUI cũ không
> vỡ regex. Chi tiết + sơ đồ: `flow.md` §4.2.

Baro vẫn **không bao giờ** là liftoff detector (sát đất là vùng baro yếu nhất:
propwash + ground effect) và **không bao giờ** tạo Vz bằng đạo hàm.

**Telemetry mới** (`status`, `@CAL STATUS` không liên quan — xem UDP STATUS
line / `tools/uav_udp_console.py` panel Vz + Az world): `vert_accel_raw_ms2`
(Az TRƯỚC LPF), `vert_accel_ms2` (SAU LPF), `accel_bias_ms2`,
`vz_accel_only_ms2`/`z_inertial_m` (shadow tracker THUẦN accel, baro không
BAO GIỜ chạm — so với `vz_ms`/`alt_m` để thấy baro đang kéo bao nhiêu),
`alt_target_vz_ms`, `baro_dt_s`.

**Chưa làm (chừa interface)**: `alt_estimator_correct_velocity()` khai báo
sẵn cho nguồn correction VẬN TỐC tương lai (PMW3901 optical flow — đã có chân
SPI IO40/41/42, `SENSOR_FLOW_ENABLED=0`, CHƯA có driver) — hiện là no-op an
toàn, KHÔNG có logic flow thật.

### Test bắt buộc trước khi bay (chạy trên phần cứng, gửi log lại để review)

A. **Nằm yên, motor OFF, 30-60s**: `Az_world` (VACC) gần 0; `Vz_accel_only`
   (VZAO) có thể trôi nhẹ; `Vz_est` (VZ) PHẢI bị kéo gần 0 (ZUPT); `Z_est`
   không trôi nhanh. Nếu `Vz_est` KHÔNG về gần 0 khi DISARMED+đứng yên → kiểm
   tra `stationary_for_alt` (flight_core.c) có đúng nhận diện đứng yên không.
B. **Nâng bằng tay**: `Az_world` phản ứng trước; `Vz_accel_only` đổi nhanh;
   `Z_inertial` đổi theo; baro (BFILT) theo chậm hơn; `Z_est` mượt.
C. **Giữ cùng độ cao, tilt roll/pitch**: `Z_est`/`Vz_est` KHÔNG đổi lớn chỉ vì
   nghiêng. Nếu có → kiểm tra dấu quaternion body→world trong
   `alt_estimator_update()` (đã khớp quy ước `mahony_filter.c` — accel body
   khi nằm phẳng đọc +1g trên world Z — nhưng PHẢI xác nhận lại trên phần
   cứng thật, xem mục "Gravity convention" trong báo cáo commit này).
D. **Inject baro spike** (thổi/che cảm biến đột ngột): `Z_est`/`Vz_est` KHÔNG
   nhảy mạnh — `baro_reject_count` (BREJ) phải tăng.
E. **So `Vz_est` (VZ) với `Vz_accel_only` (VZAO)**: ngắn hạn phải GẦN NHAU
   (accel phản ứng nhanh); dài hạn `Vz_est` không trôi trong khi
   `Vz_accel_only` có thể trôi chậm (đó chính là baro đang sửa).
F. **Buộc/đè drone cố định, motor quay, đưa ga bậc thang**: `baro_filtered_
   alt_m` (BFILT) KHÔNG được nhảy theo ga. FAIL → prop wash lọt vào baro dù đã
   đặt xa cánh, cần xem lại vị trí lắp BMP280.
G. **Motor quay ở ga hover, drone cố định**: `Az_world` SAU LPF (VACC) KHÔNG
   được nhặt rung tần số motor rõ (xem đồ thị panel "Az world"). Nếu có → hạ
   `IMU_ACCEL_DLPF_CFG` (tuning.h, vd 3→4, tức 44Hz→21Hz) TRƯỚC khi nghi ngờ
   LPF phần mềm.

## Việc còn lại trước khi bay thật (theo thứ tự)

1. **Xác nhận vị trí vật lý CH1..CH4** — dùng lệnh `test_motor` qua console
   USB (xem mục `app_config.h` ở trên), **làm TRƯỚC TIÊN**, sai cái này thì
   mọi thứ khác vô nghĩa.
2. **`tof_driver.c` — bench-test bảng tuning ST ULD** (xem hạn chế #5): giao
   thức ranging đã thật, nhưng bảng "default configuration" ~90 thanh ghi
   CHƯA điền — so ToF với thước đo thật ở vài khoảng cách trước khi tin
   alt_hold dùng ToF.
3. **Remap trục IMU/mag** (xem hạn chế #6) — xác nhận hướng lắp thật trên
   khung (nghiêng bo trên bàn, so số đọc `status` qua console).
4. ~~Calib gyro bias tĩnh~~ — XONG (xem mục "Calibration (gyro/accel/mag)"
   dưới đây): `calib_gyro`/`calib_accel_face`/`calib_mag_start`+`calib_mag_stop`
   (console) hoặc `fc.calibrate_*()` (MicroPython), persist qua NVS. **LƯU Ý
   BREAKING CHANGE**: `arm`/`fc.arm()` giờ bị TỪ CHỐI nếu chưa calib accel
   (+ mag nếu `SENSOR_MAG_ENABLED=1`) — PHẢI calib 1 lần sau khi remap trục xong.
5. Đo `ALT_HOLD_HOVER_NOMINAL` + tune `TAKEOFF_PRIME_DUTY`/`prime_ms` trong
   `tuning.h` trên phần cứng S3 thật (khác board cũ WROOM-32) — xác nhận địa
   chỉ I2C BMP280 (`BOARD_BARO_I2C_ADDR`) đúng SDO thật trong lúc này luôn.
6. Build + nạp qua PlatformIO (xem mục "Build/nạp trực tiếp") sau MỖI lần
   sửa driver — bay thử qua console USB, KHÔNG cần MicroPython port.
7. (Optional, không chặn bay) Build MicroPython port thật (xem mục dưới) —
   chỉ cần nếu muốn viết mission bằng Python thay vì gõ lệnh console trực tiếp.
8. (Không chặn bay) `flow_driver` (PMW3901 SPI) + RGB LED WS2812 driver —
   pin đã khai báo sẵn trong `board_config.h`, bật qua `SENSOR_FLOW_ENABLED`/
   `SENSOR_RGB_LED_ENABLED` trong `app_config.h` sau khi có driver.

## Calibration (gyro/accel/mag) — persist qua NVS

`components/flight_core/src/calibration.c` lưu 3 phần ĐỘC LẬP nhau trong NVS
(namespace `fc_calib`): gyro bias, accel bias+scale (6-face), mag hard/soft-iron
(figure-8). Calib lại 1 cảm biến KHÔNG xóa 2 cảm biến kia.

**BREAKING CHANGE**: `CMD_ARM` (console `arm`, `fc.arm()`, `@ARM` UDP — tất cả
dùng chung `apply_command()` trong `flight_core.c`) giờ bị **TỪ CHỐI** nếu
chưa có accel hợp lệ trong NVS (+ mag nếu `SENSOR_MAG_ENABLED=1`). Lần flash
đầu tiên (NVS trống) PHẢI calib 1 lần trước khi bay được.

Quy trình (chỉ chạy khi `DISARMED`, KHÔNG tháo cánh quạt cũng an toàn vì
không động cơ nào quay trong lúc calib):

| Bước | Console (USB) | MicroPython |
|---|---|---|
| Gyro bias tĩnh (~1.5s, đứng yên) | `calib_gyro` | `fc.calibrate_gyro()` |
| Accel 6-face (gọi 6 lần, đổi hướng đặt drone mỗi lần — nằm ngửa/úp/nghiêng 4 cạnh, miễn mỗi trục thấy cả +g và -g) | `calib_accel_face` x6 | `fc.calibrate_accel_face()` x6 |
| Hủy accel giữa chừng | `calib_accel_reset` | `fc.calibrate_accel_reset()` |
| Mag hard/soft-iron (TỰ ĐỘNG chạy 60s, xoay hình số 8 suốt thời gian này) | `calib_mag_start` ... xoay ... (tự xong, hoặc `calib_mag_stop` để kết thúc sớm) | `fc.calibrate_mag_start()` ... (tự xong, hoặc `fc.calibrate_mag_stop()`) |
| Hủy gyro giữa chừng (không lưu) | `calib_gyro_abort` | `fc.calibrate_gyro_abort()` |
| Hủy mag giữa chừng (không tính/lưu dù đã đủ mẫu) | `calib_mag_abort` | `fc.calibrate_mag_abort()` |
| Xóa toàn bộ (bench/test, sẽ KHÔNG arm được tới khi calib lại) | `calib_erase` | `fc.calibrate_erase()` |
| Lấy lại mốc 0m + std-dev baro NGAY LÚC GỌI (KHÔNG lưu NVS, khuyến nghị ngay trước arm) | `calib_baro_ground` | `fc.calibrate_baro_ground()` |

## Nhịp vòng điều khiển: data-ready interrupt (MPU6050 INT → IO36)

Chuỗi thật (chi tiết + bảng task/priority: `flow.md` §3):

```
MPU6050 INT (IO36) ─ISR→ sensor_hub (core 1, prio 22) thức
                          → đọc IMU qua I2C → publish snapshot (seq++, timestamp)
                          → xTaskNotifyGive(stabilize)
                              stabilize prio 23 > 22 ⇒ PREEMPT NGAY
                              → Mahony → Az/Vz/Z → safety/FSM → PID → mixer → motor
                              → block lại
                          → hub chạy tiếp MAG/BARO/ToF/battery
```

`stabilize_task` **không** tự hẹn giờ bằng `vTaskDelayUntil()` ở đường chính —
nó ngủ chờ notify của hub, mà hub thì ngủ chờ chính MPU6050 báo "có mẫu mới".
Mẫu được đọc ngay sau khi sinh ra và tới tay vòng điều khiển **trước** khi hub
đụng tới cảm biến chậm nào khác, nên độ trễ nhỏ và **ổn định** — không còn
jitter 0–4 ms do lệch pha giữa đồng hồ FreeRTOS và đồng hồ nội của cảm biến. Với
vòng điều khiển, jitter độ trễ chính là nhiễu bơm thẳng vào D-term.

Priority `stabilize` (23) **cao hơn** `sensor_hub` (22) là điều kiện cần cho
bước "PREEMPT NGAY" ở trên, và hai task phải **cùng core 1** (preemption chỉ xảy
ra trong cùng một core). Cả hai được canh bằng `_Static_assert`.

Điều kiện bắt buộc: `IMU_SAMPLE_RATE_HZ` (imu_driver.h) phải bằng
`CONTROL_TASK_HZ` — có `_Static_assert` canh lúc biên dịch. `SMPLRT_DIV` được
tính từ đó, không đặt tay.

**Đường dự phòng luôn có**: task chờ notify *kèm timeout* (2 chu kỳ). Trượt
`IMU_INT_MISS_STREAK_MAX` lần liên tiếp (~100 ms) thì tự rơi về đồng hồ
FreeRTOS và log lỗi. Vòng điều khiển **không bao giờ** được đứng chờ ngắt vô
hạn — nếu dây INT đứt giữa lúc bay, task đứng im nghĩa là motor giữ nguyên duty
cuối cùng và cả Commander/failsafe cũng ngừng chạy.

**Chạy bằng đồng hồ dự phòng KHÔNG có nghĩa là tích phân bằng mẫu cũ.** Tick nào
thức dậy mà snapshot không có mẫu IMU mới (`seq` không đổi, hoặc mẫu quá hạn)
thì Mahony / alt_estimator / I-term của PID **bị bỏ qua** ở đúng tick đó — chỉ
safety/deadline/failsafe/Commander/mixer vẫn chạy. Khi mẫu mới tới, tích phân
dùng `fusion_dt` = khoảng giữa hai **mẫu** (cộng dồn qua các tick bị bỏ), không
phải giữa hai tick. Bảng đầy đủ: `flow.md` §3.4b.

Kiểm chứng bằng `status`:

```
loop_clock: imu_int_active=1 isr=125043 wake=125041 timeout=0 no_new_sample=0
```

`isr` ≈ `wake`, `timeout=0`, `no_new_sample=0` là đúng. `timeout` tăng → kiểm
tra dây INT. `isr` ≫ `wake` → có cạnh giả (dây thả nổi) hoặc task quá tải.
`no_new_sample` tăng đều → vòng bay đang quay nhanh hơn nguồn mẫu.

> ⚠ **GPIO33–37 là bus PSRAM octal của ESP32-S3.** IO36 chỉ dùng được vì PSRAM
> đang tắt (`CONFIG_SPIRAM is not set`). Bật PSRAM lên là chân này bị chiếm và
> ngắt IMU hỏng — khi đó phải dời INT sang chân khác, đừng đi tìm lỗi ở MPU6050.

### `mag_test` — chẩn đoán QMC5883P (KHÔNG phải calibration)

`calib_mag` thu **0 mẫu** thì xoay thêm bao lâu cũng vô ích: đó là chip không
phát mẫu, không phải thao tác sai. Trước khi động vào tầng calibration, phải
chắc chắn phần cứng phát mẫu ổn định đã:

```
mag_test 0        # không ghi REG 0x29
mag_test 1        # có ghi REG 0x29 = 0x06 (datasheet QST yêu cầu)
```

Lệnh chạy lại toàn bộ trình tự init và **log từng bước** (CHIP_ID → soft-reset
→ \[SIGN\] → CTRL2 ghi/đọc → CTRL1 ghi/đọc → chờ DRDY → XYZ đầu → vòng đọc
20ms). Dòng quan trọng nhất là `[mag_test 10 vong doc]`: với ODR=100Hz mà đọc
ở 50Hz thì `ok%` phải cao; `ok=0` nghĩa là chip không đo.

**Chạy cả 2 lần rồi so log** là cách chốt dứt điểm câu hỏi "con chip này có
cần `0x29=0x06` không" — datasheet QST nói CÓ, nhưng phần cứng này đã từng cho
thấy ghi `0x29` làm CTRL2 readback về `0x00`. Firmware chính hiện KHÔNG ghi
`0x29`; đổi quyết định đó phải dựa trên log thật của `mag_test`, không phải suy
đoán. Lệnh tự khôi phục driver khi xong (không cần reboot), chỉ chạy khi
`DISARMED`, block ~1.3s.

Cùng bộ lệnh này còn có qua **UDP ground-station** (`tools/uav_udp_console.py`,
panel "Calibration"): `@CAL STATUS`, `@CAL GYRO START|ABORT`,
`@CAL ACC START|NEXT|ABORT`, `@CAL MAG START|STOP|ABORT`, `@CAL BARO START`,
`@CAL RESET` (xóa toàn bộ) — xem `command_parser.c::handle_cal()`. `@CAL SAVE`/
`LOAD` trả OK kèm giải thích (không cần làm gì — calib tự lưu/nạp, xem dưới).

`python/fc_api.py` có sẵn wrapper BLOCKING poll `fc.calib_status()`:
`calibrate_gyro()`, `calibrate_accel_6face(prompt=...)`, `calibrate_mag(duration_s=...)`.

**Motion/stability detection** (gyro + từng mặt accel): gyro calibration chạy
tự động mỗi boot và lệnh `calib_gyro` dùng cùng FSM: settle 1.5s,
chờ stationary, collect `gyro_raw_dps` liên tục 4s bằng Welford, sau đó
validate trên một cửa sổ mới 1.5s. Chuyển động hủy toàn bộ cửa sổ;
không commit bias dang dở. Các ngưỡng gyro nằm trong nhóm `GYRO_CAL_*`
của `tuning.h`; accel 6-face vẫn dùng `CALIB_ACCEL_MOTION_GYRO_DPS`.

**Verify sau accel 6-face**: áp lại công thức hiệu chỉnh (bias/scale) lên
chính 6 mặt vừa đo, `residual = ||a_hiệu_chỉnh| - 1.0g|` — vượt
`CALIB_ACCEL_MAX_RESIDUAL_G` (0.15g mặc định) thì HỦY toàn bộ, không lưu.

**Persistence**: mỗi field (gyro / accel[bias+scale] / mag[hard+soft]) lưu
kèm CRC32 riêng + 1 "format version" chung (`CALIB_NVS_FORMAT_VERSION` trong
`calibration.c`) — corrupt 1 field chỉ làm field đó invalid, không kéo 2 field
kia; đổi layout struct trong tương lai thì bump version, firmware mới sẽ coi
NVS cũ như UNCALIBRATED thay vì đọc rác.

**Đã CỐ Ý CHƯA làm** (xem lý do trong code, không phải bỏ sót):
- **MPU6050 self-test thật** (đọc `SELF_TEST_X/Y/Z/A`, so factory trim) — cần
  đối chiếu đúng datasheet/register bit layout mà tôi không tự tin nhớ chính
  xác 100%; chép sai sẽ tạo kết quả PASS/FAIL giả, nguy hiểm hơn để trống.
  `imu_self_test()` CHƯA tồn tại — TODO khi có datasheet gốc.
- **Mag soft-iron dạng ellipsoid 3×3 đầy đủ** — hiện vẫn dùng diagonal
  per-axis (đủ dùng cho Mahony normalize, "six-parameter" tương đương accel),
  ellipsoid fit nên chạy OFFLINE trên PC (least-squares) rồi upload matrix,
  KHÔNG chạy trên ESP32 — chưa xây dựng luồng upload này.
- **Thermal compensation** (gyro bias / baro pressure theo nhiệt độ) — CHỈ
  log nhiệt độ (`imu_temp_c` mới, telemetry) để xem có drift đáng kể không,
  CHƯA fit polynomial compensation (cần dataset thật trước).
- **CompassMot** (bù nhiễu từ động cơ/dòng pin vào mag) — KHÔNG tự invent
  (không có current sensor); test bằng tay qua telemetry sẵn có (mag raw +
  throttle) theo quy trình: motor OFF → ghi mag, tăng dần throttle → so sánh.

**KHÔNG có calib nào tự động** — boot CHỈ nạp calib đã lưu NVS (gyro/accel/mag),
KHÔNG tự đo lại bất kỳ cảm biến nào. Cả 3 loại calib CHỈ chạy khi có lệnh
serial tường minh (bảng trên). Muốn chống trôi yaw do gyro bias trôi theo
nhiệt độ giữa các lần bay, tự gọi `calib_gyro` (~1.5s, đứng yên) trước khi
`arm` mỗi lần cần — không có cờ tự động nào chạy ngầm lúc cấp nguồn.

Calib accel/mag là toán **orientation-agnostic** (min/max theo TỪNG TRỤC gộp
qua nhiều lần đo, không gán cứng "mặt nào = trục nào") — không phụ thuộc
hướng lắp IMU/mag thật trên khung (vẫn CHƯA xác nhận, xem hạn chế #6), an
toàn để chạy TRƯỚC khi remap trục xong.

## fc.control() — joystick angle-mode (MicroPython, kiểu pyDrone)

```python
fc.control(rol, pit, yaw, thr)   # mỗi trục -100..100, KHÔNG blocking
roll, pitch, yaw, rol_in, pit_in, yaw_in, battery_v, alt_mm = fc.get_states()
```

`rol`/`pit` -> **GÓC** mục tiêu (thả về 0 -> tự cân bằng phẳng). `yaw` ->
**TỐC ĐỘ** quay (thả về 0 -> giữ nguyên heading, KHÔNG tự quay lại). `thr` ->
**slew** (không step) độ cao mục tiêu khi đang `HOLDING`/`FLYING`. Dùng CHUNG
kho setpoint + watchdog stale với ground-station UDP (`@SP SET`) — chỉ MỘT
nguồn setpoint tay lái thắng tại một thời điểm (lệnh mới nhất). Giới hạn góc
nghiêng/tốc độ quay (`max_lean_deg`/`max_yawrate_dps`, mặc định 28°/180dps)
tune runtime qua `fc.set_param("max_lean_deg", 25)` — xem `tuning.h` mục 7.

`fc.set_flightmode(mode)`: `1`=head (mặc định), `0`=headless — **STUB**, hiện
hành xử NHƯ head (cần world-frame rotation qua mag, trục CHƯA xác nhận trên
phần cứng — xem hạn chế #6).

## PWM 11-bit (bù throttle theo điện áp pin: ĐÃ BỎ)

**Độ phân giải PWM động cơ** (`motor_driver.h`) đã tăng từ 10-bit (0-1023,
`MOTOR_SAFE_MAX_DUTY=1000`) lên **11-bit** (0-2047, `MOTOR_SAFE_MAX_DUTY=2000`)
— vẫn giữ nguyên tần số **24kHz** (80MHz APB / 24kHz đủ chia cho 2^11, KHÔNG
đủ cho 2^12 — xem `motor_driver.h`). **TOÀN BỘ hằng số đơn vị duty trong
`tuning.h`** (gain+ILIMIT+OUTLIM vòng RATE của attitude cascade, `ALT_HOLD_VZ_KP/
KI/ILIMIT/HOVER_NOMINAL`, `TAKEOFF_PRIME_DUTY`, `LAND_MIN_THROTTLE/
BLIND_DESCENT_RATE`, `ATT_MIN_THROTTLE_DUTY`) đã **nhân đôi tương ứng** —
hằng số đơn vị deg/dps/m/m/s (vòng ANGLE, `ALT_HOLD_ALT_KP`, mọi mốc thời
gian/độ cao) giữ nguyên, KHÔNG liên quan thang duty. Nếu tune PID qua GUI
(`@PID SET`/`@ALT SET`...) sau bản này, các số đọc/gửi đều theo thang **0-2000**
mới — slider GUI (`tools/uav_udp_console.py`) đã cập nhật theo (vz_kp/ki/ilim,
hover, spool_duty trần 2000).

**Bù throttle theo điện áp pin — ĐÃ BỎ HẲN** (`tuning.h` mục 10b).

Trước đây firmware nhân `throttle_cmd *= clamp(4.2 / battery_v, 1.0, 1.25)` ở
mọi FSM state. Cả 2 hằng số `BATTERY_COMPENSATION_NOMINAL_V/MAX_GAIN` cùng toàn
bộ đường nhân đã bị xoá, vì cách làm đó **sai về mặt điều khiển**:

- `VBAT` đo được tụt theo **tải tức thì**, không chỉ theo mức pin còn lại. Lúc
  UAV nhấc lên, 4 motor rút dòng lớn → sụt áp trên nội trở pin/dây → `VBAT`
  giảm mạnh dù pin còn đầy.
- Nhân throttle theo con số đó tạo **vòng phản hồi dương ký sinh** nằm ngoài
  mọi vòng PID đã tune: ga lên → dòng tăng → VBAT đo giảm → comp tăng → ga lên
  nữa.
- Nó cũng phá `alt_hold`: duty THỰC ra motor không còn là duty PID yêu cầu, nên
  gain tune ở bench đều sai khi bay thật.
- Đo được từ log bench thật: `BATV` dao động **3.51…3.82V trong vài giây** ở
  tải gần như không đổi → comp nhảy 1.10…1.20, tức nhiễu áp bị khuếch đại
  thẳng vào throttle.

Điện áp pin **vẫn dùng** cho failsafe (sàn `COMMANDER_DEFAULT_BATTERY_FLOOR_V`
= 3.3V → LANDING) và `prearm_check()` — đó là so **ngưỡng**, không nhân vào
đường điều khiển. Telemetry `BCOMP=` / `battery_comp` giữ lại nhưng **luôn
1.000** để dòng STATUS không đổi format;
`fc.set_param("battery_comp_nominal_v"/"battery_comp_max_gain", ...)` vẫn được
nhận nhưng chỉ log cảnh báo "BO QUA".

## Console / nạp code — qua USB D+/D-, KHÔNG qua UART0

miniUav không có chip USB-UART bridge — nạp firmware, xem log, REPL
MicroPython đều đi qua cổng **USB D+/D- (IO19/IO20)**, dùng USB-Serial-JTAG
controller có sẵn trong ESP32-S3 (không cần linh kiện ngoài). Vì không có
chip bridge tự toggle DTR/RTS, PHẢI vào download-mode THỦ CÔNG trước khi flash:

1. Giữ nút **BOOT** (kéo GPIO0 xuống LOW).
2. Nhấn rồi thả **RESET**.
3. Thả **BOOT** — board vào download-mode, `esptool`/`pio run -t upload` mới
   thấy cổng.

`sdkconfig.esp32s3_flight_core_verify` đã đổi console chính sang
`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y` (mặc định ESP-IDF là UART0 — SẼ câm
lặng nếu không đổi, vì UART0 không wire trên bo). Firmware thật (MicroPython
port) cũng cần key này trong sdkconfig của nó (xem mục "Build MicroPython
port thật" bên dưới).

## Build/nạp trực tiếp bằng PlatformIO — firmware CHÍNH, KHÔNG cần MicroPython

`platformio.ini` nằm **NGAY GỐC repo** — mở thẳng thư mục `UAV-S3/` bằng
PlatformIO IDE (VS Code extension) là thấy project, bấm Build/Upload/Monitor
được luôn, KHÔNG cần mở thư mục con nào khác. `flight_core` là ESP-IDF
component ĐỘC LẬP (không include gì từ `main/` hay `micropython_module/`) và
được build **THẲNG từ `components/flight_core/` thật** — không còn bước
copy/đồng bộ nào nữa (đã bỏ thư mục `test_build/` cũ, gộp lên gốc).

`src/main.c` giờ là **firmware điều khiển THẬT** (không còn là smoke-test
"arm 1 lần rồi thoát") — link thẳng `flight_core`, nhận lệnh bay qua console
USB (arm/takeoff/land/move/...), KHÔNG cần build MicroPython port (mục "Build
MicroPython port thật" ở dưới vẫn còn nhưng KHÔNG BẮT BUỘC nữa — chỉ cần nếu
sau này muốn viết mission bằng Python thay vì gõ lệnh console). Xem mục
"Bay qua console USB" ngay dưới đây.

**Dùng PlatformIO IDE (khuyến khích):** mở `UAV-S3/` trong VS Code, extension
PlatformIO tự nhận `platformio.ini` — dùng nút Build/Upload/Monitor ở status
bar hoặc PlatformIO sidebar. Nạp code (Upload) cần vào download-mode thủ công
trước (xem mục "Console / nạp code" ở trên: giữ BOOT, nhấn/thả RESET, thả BOOT).

**Hoặc dùng CLI** (`pio` không có trong PATH mặc định trên máy này — dùng
full path, hoặc thêm `C:\Users\<user>\.platformio\penv\Scripts` vào PATH):

```bash
cd UAV-S3   # gốc repo — KHÔNG cần cd vào thư mục con nào
C:\Users\typho\.platformio\penv\Scripts\pio.exe run -e esp32s3_flight_core_verify              # build
C:\Users\typho\.platformio\penv\Scripts\pio.exe run -e esp32s3_flight_core_verify -t upload    # nạp (sau khi vào download-mode)
C:\Users\typho\.platformio\penv\Scripts\pio.exe device monitor -b 115200                       # xem log qua USB (Ctrl+C thoát)
```

Kết quả build hiện tại: **SUCCESS** (Flash ~26%, RAM ~5%, ESP-IDF 6.0.1 qua
PlatformIO `espressif32@7.0.1`).

## Bay qua console USB (firmware chính, không cần MicroPython)

Sau `pio run -t upload`, mở `pio device monitor -b 115200` (hoặc bất kỳ
terminal serial nào trỏ đúng cổng USB) — thấy prompt `uav>`. Gõ `help` để xem
đầy đủ danh sách lệnh. Các lệnh chính (PORT ngữ nghĩa BLOCKING từ
`python/fc_api.py`, hành vi giống hệt REPL MicroPython nếu sau này build):

| Lệnh | Ý nghĩa |
|---|---|
| `status` | In state/attitude/alt/motor/battery hiện tại |
| `arm` | Arm (bị từ chối nếu attitude invalid hoặc nghiêng > 10°) |
| `disarm` | Disarm ngay lập tức |
| `takeoff <mm> [timeout_s] [climb_timeout_s]` | Cất cánh, blocking tới khi ổn định ở `mm` |
| `land [timeout_s]` | Hạ cánh, blocking tới khi `DISARMED` |
| `hover <sec>` | Giữ nguyên vị trí `sec` giây |
| `move <dir> <pct> <sec>` | `dir=forward/back/left/right/up/down/cw/ccw` |
| `yaw <deg>` | Xoay tương đối (open-loop theo thời gian) |
| `alt <mm>` | Đổi altitude target (qua geofence clamp) |
| `test_motor <1-4> <duty_pct>` | Xác nhận vị trí động cơ — CHỈ khi `DISARMED`, xem `app_config.h` |
| `test_motor_all <duty_pct>` | Cả 4 động cơ quay cùng lúc, đồng duty — sanity-check nhanh, CHỈ khi `DISARMED` |
| `bench_start` | Vào chế độ bench-test tăng ga tay để tune PID — CHỈ từ `ARMED` |
| `+` / `-` | Tăng/giảm throttle bench-test 1 nấc (mặc định ±20 duty, đổi ở `BENCH_THROTTLE_STEP_DUTY` trong `src/main.c`) |
| `bench_stop` | Cắt throttle bench-test NGAY, về `ARMED` (không latch, lặp lại được) |

### Bench-test tăng ga tay để tune PID (`bench_start`/`+`/`-`/`bench_stop`)

Dành riêng cho tune PID mà KHÔNG cần bay thật — **drone PHẢI được giữ chặt/
kẹp trên giá đỡ**, đây KHÔNG phải chế độ bay. Luồng dùng:

```
arm                 # -> ARMED
bench_start          # -> BENCH_RAMP, throttle=0
+                    # +20 duty mỗi lần gõ, lặp lại tới khi throttle đủ cao
status               # xem throttle/attitude hiện tại
```

Ngay khi `throttle >= ATT_MIN_THROTTLE_DUTY` (200, xem `tuning.h`), attitude
PID (roll/pitch/yaw) tự chạy — CHUNG đúng ngưỡng với mọi state khác trong
firmware, không có logic riêng cho bench mode. Dùng `move`/`@SP SET`/`@PID SET`
để quan sát + chỉnh phản ứng trong lúc giữ nguyên hoặc tiếp tục tăng ga bằng
`+`. Gõ `-` để giảm, `bench_stop` để cắt máy ngay (không latch — `bench_start`
lại được luôn) hoặc `disarm`/`kill` để dừng hẳn.

An toàn: `FSM_BENCH_RAMP` được bọc bởi TOÀN BỘ lớp an toàn chung (cắt-ngay-
latch khi nghiêng/attitude-stale, Commander heartbeat/battery/hard-tilt) —
xem GHI CHÚ 3 trong `flight_state_machine.h`.

Firmware **tự động gửi heartbeat mỗi 300ms** ở nền (task riêng, không phụ
thuộc bạn gõ lệnh) — watchdog Commander vì vậy không soft-fault chỉ vì bạn
đang đọc log/suy nghĩ giữa 2 lệnh. Lệnh `heartbeat` thủ công vẫn có, chỉ để
test, không bắt buộc dùng. **Hạn chế đã biết**: nếu chính task console bị
treo (hiếm — vd chờ UART), task heartbeat riêng vẫn chạy đều nên watchdog
không bắt được kiểu treo này — các lớp an toàn khác (cắt motor ngay khi
nghiêng/attitude-stale, battery floor, IMU-invalid) không phụ thuộc heartbeat
nên vẫn hoạt động độc lập.

Lưu ý các điểm đã vướng + đã sửa (không cần lặp lại, để biết nếu build lại từ
máy khác gặp y hệt):
- Repo git chưa có commit nào → `git describe` trong ESP-IDF's `project.cmake`
  fail cứng lúc configure. Fix: file `version.txt` ở gốc repo (ESP-IDF ưu
  tiên đọc version từ đây, bỏ qua git describe).
- ESP-IDF 6.x tách `driver/ledc.h` và `driver/gpio.h` ra component riêng
  (`esp_driver_ledc`, `esp_driver_gpio`) — đã thêm cả 2 vào `REQUIRES` của
  `components/flight_core/CMakeLists.txt` (cùng `esp_adc` cho battery_driver).
  `driver/i2c.h` (legacy) vẫn còn trong `driver` nhưng in cảnh báo EOL — cân
  nhắc migrate sang `driver/i2c_master.h` sau này.
- Nếu sửa CMakeLists.txt (thêm file/REQUIRES) mà build vẫn báo thiếu header:
  xóa `.pio/build/esp32s3_flight_core_verify` rồi build lại (CMake cache
  không luôn tự nhận thay đổi REQUIRES).
- `main/` (chỉ chứa `board_config.h` + `app_config.h`, không có
  `CMakeLists.txt`) nằm cạnh `components/`/`platformio.ini` ở gốc — ESP-IDF's
  component discovery bỏ qua an toàn (không có `CMakeLists.txt` = không phải
  component), không gây lỗi build. `src/main.c` include thẳng
  `../main/board_config.h` + `../main/app_config.h` (đường dẫn tương đối) để
  dùng ĐÚNG pin + cờ bật/tắt thật, không hardcode lại.
- `src/main.c` dùng `esp_console` (REPL qua USB-Serial-JTAG) — `src/CMakeLists.txt`
  đã thêm `REQUIRES console`. Nếu thấy lỗi thiếu `esp_console.h` sau khi pull
  code mới, xóa `.pio/build/esp32s3_flight_core_verify` rồi build lại (CMake
  cache không tự nhận `REQUIRES` mới, giống lưu ý bên trên).

## IntelliSense (VS Code)

`.vscode/c_cpp_properties.json` trỏ vào `compile_commands.json` THẬT do
`pio run` sinh ra ở `.pio/build/esp32s3_flight_core_verify/` (ngay gốc repo)
+ fallback `includePath` cho file KHÔNG nằm trong compile DB (chủ yếu
`micropython_module/fc/*.c`, vì không có checkout MicroPython thật để build
tới).

Regenerate sau khi **thêm/xóa file nguồn** trong `components/flight_core`
(không cần đồng bộ/copy gì nữa — build thẳng từ nguồn):

```bash
C:\Users\typho\.platformio\penv\Scripts\pio.exe run -t compiledb -e esp32s3_flight_core_verify
```

`micropython_module/fc/*.c` sẽ luôn còn lỗi include đỏ (`py/obj.h` v.v.) —
không có checkout MicroPython thật trong repo này để trỏ vào, đây là giới
hạn đã biết (không phải lỗi cấu hình), xem mục "Build MicroPython port thật".

## Build MicroPython port thật (CHƯA làm — bước tiếp theo của bạn)

Ngoài phạm vi repo này (cần clone toàn bộ micropython + pin đúng phiên bản
esp-idf theo port esp32 của nó, dung lượng lớn, build lâu). Các bước dự kiến:

```bash
git clone https://github.com/micropython/micropython.git
cd micropython
git submodule update --init lib/berkeley-db-1.xx   # theo README của micropython/ports/esp32
cd ports/esp32
make submodules
# ESP-IDF version: XEM micropython/ports/esp32/README.md để biết bản esp-idf
# port này pin — cài đúng bản đó (KHÁC ESP-IDF 6.0.1 mà PlatformIO ở gốc repo
# dùng ở trên — hai build KHÔNG liên quan tới nhau).

idf.py -D MICROPY_BOARD=ESP32_GENERIC_S3 \
       -D USER_C_MODULES=<path-to>/UAV-S3/micropython_module/micropython.cmake \
       build
```

`micropython_module/fc/micropython.cmake` link thẳng vào
`components/flight_core` (qua `target_link_libraries(usermod_fc INTERFACE
__idf_flight_core)`) — cần đường dẫn tương đối đúng theo layout repo này; nếu
di chuyển thư mục thì sửa lại 2 file `micropython.cmake`.

**Chưa build-verify được bước này** — pattern `fc_module.c`/`fc_bridge.c`
theo đúng API chuẩn MicroPython user-C-module (`mp_obj_fun_builtin_*`,
`MP_DEFINE_CONST_FUN_OBJ_*`, `MP_REGISTER_MODULE`) nhưng CHƯA compile thử với
mã nguồn MicroPython thật. Nếu lỗi API lệch phiên bản, báo lỗi cụ thể để sửa.

Sau khi có checkout, sdkconfig của port này cần các key sau (xem
`main/board_config.h` cuối file để không quên):

```
CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y
CONFIG_ESPTOOLPY_FLASHSIZE="16MB"
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_OCTAL=y
CONFIG_SPIRAM_SPEED_80M=y            # hoặc tốc độ khớp datasheet PSRAM thật
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y
CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y # console/nạp code qua USB D+/D-, xem mục ở trên
CONFIG_ESP_CONSOLE_SECONDARY_NONE=y
```

## Bay thử từ REPL

```python
import fc_api as fc
fc.arm()
fc.takeoff(800)   # mm, blocking — xem "Hạn chế đã biết" #2
fc.hover(3)
fc.land()          # blocking tới khi DISARMED
```

Hoặc `python/example_mission.py` chạy sẵn cùng luồng + heartbeat nền (nếu
port có `_thread`).

⚠️ KHÔNG bay thật cho tới khi xong mục 1-5 trong "Việc còn lại trước khi bay
thật" (đặc biệt mục 1 — xác nhận vị trí motor). Nếu `MOTOR_POSITIONS_CONFIRMED=0`
trong `app_config.h`, firmware vẫn chạy bình thường (không tự chặn arm) —
đừng dựa vào code để nhắc, cứ hoàn tất checklist trước khi lắp cánh quạt.

## Test offline (không cần phần cứng)

```bash
cd python
python test_fc_api_offline.py   # mock module `fc`, verify logic blocking/timeout/fault
```

8 test: happy-path takeoff/land, timeout tự land() an toàn, fault raise đúng
exception, hover dừng sớm khi fault, move block đúng thời lượng, heartbeat thủ
công. **PASS** cả 8 (CPython, không cần MicroPython thật — chỉ test logic
Python thuần của `fc_api.py`).

## Tài liệu liên quan

- `flow.md` (gốc repo) — sơ đồ luồng end-to-end chi tiết hơn (sequence diagram
  1 lệnh Python, flowchart 11 bước của `stabilize_task`, state diagram FSM).

## Cấu trúc thư mục

```
platformio.ini                 ← project PlatformIO Ở GỐC — mở UAV-S3/ là build/nạp được ngay
sdkconfig.esp32s3_flight_core_verify ← sdkconfig cho env trên (console USB, flash 16MB)
CMakeLists.txt, version.txt    ← ESP-IDF project files (version.txt né git-describe khi chưa có commit)
src/main.c                     ← firmware điều khiển THẬT (console USB, arm/takeoff/land/...) —
                                  include thẳng main/board_config.h + main/app_config.h
main/board_config.h            ← TẤT CẢ pin/địa chỉ I2C — ĐÃ điền theo schematic thật
main/app_config.h              ← BẬT/TẮT cảm biến (SENSOR_*_ENABLED) + xác nhận vị trí motor
components/flight_core/        ← TẦNG DƯỚI (C thuần, build độc lập) — build THẲNG từ đây,
                                  không có bản copy nào khác
  include/flight_core/
    tuning.h                   ← 1 FILE DUY NHẤT để chỉnh PID/tune
    drivers/                   ← header driver (tất cả đã có register map thật, xem bảng trạng thái)
  src/
    drivers/
      motor_driver.c           ← xong (LEDC 24kHz/11-bit)
      battery_driver.c         ← xong (ADC1 oneshot + calibration)
      imu_driver.c             ← xong (MPU6050)
      mag_driver.c              ← xong (QMC5883P)
      baro_driver.c              ← xong (BMP280, fuse alt_estimator)
      tof_driver.c             ← bring-up + giao thức ranging xong; bảng tuning ST ULD CHƯA (xem hạn chế #5)
micropython_module/fc/         ← TẦNG TRÊN bridge (fc_module.c, fc_bridge.c,
                                  micropython.cmake)
python/                        ← fc_api.py (wrapper blocking), example_mission.py,
                                  test_fc_api_offline.py (mock, chạy được ngay)
flow.md                        ← sơ đồ luồng end-to-end chi tiết
```
#   U A V _ M i n i  
 