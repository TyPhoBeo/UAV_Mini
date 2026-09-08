# Graph Report - UAV-S3  (2026-09-08)

## Corpus Check
- 54 files · ~232,394 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 1313 nodes · 2358 edges · 122 communities (61 shown, 61 thin omitted)
- Extraction: 90% EXTRACTED · 10% INFERRED · 0% AMBIGUOUS · INFERRED: 238 edges (avg confidence: 0.85)
- Token cost: 159,894 input · 8,000 output

## Community Hubs (Navigation)
- Altitude Estimator & Takeoff/Land
- Flight Core Public API
- Flight Core API Test Harness
- Telemetry & MicroPython Bridge
- ToF Facade & I2C Discovery
- Flight Doc: Control Sections
- Mahony Attitude Filter
- Flight Doc: Architecture & Tasks
- PID Tuner GUI App
- README: Project Overview
- VL53L0X Driver
- ToF Bring-up Test
- VL53L1X Driver
- Takeoff/Landing Tuning
- MPU6050 IMU Driver
- BMP280 Barometer Driver
- Build & I2C Architecture Notes
- Magnetometer Driver
- W/S Throttle Bench Test
- ARM & Takeoff Gate Concepts
- Cascade, Landing & Tick Concepts
- Calibration NVS Persistence
- Flight State Machine Impl
- Two-Tier Architecture Principles
- GUI Manual Flight Keys
- WiFi & UDP Net Link
- Terrain Offset & Clearance
- Trim Persistence Test
- Feature Flags & Bench Gating
- fc_api Blocking Wrapper
- GUI Status Line Handlers
- Calibration API & Errors
- Calibration & Diagnostics Notes
- Attitude & Calibration Headers
- Hover Model Test
- GUI Layout Builders
- Driver Header Surface
- Attitude Controller
- Altitude Hold Cascade
- Tilt Comp & Throttle Concepts
- FLYING W/S Offset Test
- Hover Model Impl
- FSM Mode Sequence Test
- GUI Gain Slider Row
- GUI ToF Health Display
- GUI Plot Panel
- UDP Console Client
- Commander Impl
- Attitude & PID State
- Commander Header API
- FSM Header API
- Python Heartbeat Thread
- Terrain Accumulate Test
- ToF Alive vs Valid Test
- GUI Altitude Gain Group
- GUI Gain Group Base
- Battery ADC Driver
- Altitude Validity Test
- Terrain Transition Test
- GUI Commander Group
- GUI Landing Group
- GUI Takeoff Group
- PID Core
- FLYING vz Hold Test
- GUI ToF Health Test
- Terrain Deadlock Test
- Tilt Comp & Expo Test
- ToF Fault Tolerance Test
- GUI Timed Command Group
- UDP Console Entry Points
- FLYING Alt PID Test
- Gyro Calibration Test
- Terrain Rebase Test
- GUI Calibration Panel
- GUI Motor Test Panel
- Alt Estimator Static Test
- ToF Surface Gate Test
- STATUS Parse Test
- Takeoff Flow Test
- Takeoff Gate Test
- AGENTS.md Graphify
- Graphify Policy Docs
- CLAUDE.md Graphify
- Example Mission
- Bench Start Command
- Bench Step Command
- Bench Stop Command
- Calib Status Query
- Baro Ground Calibration
- Calibration Erase
- Gyro Calib Abort
- Mag Calibration
- Mag Calib Abort
- Joystick Control API
- State Query API
- Hover Hold Command
- Kill Command
- Move Command
- Motor Test Command
- Flight Mode Setter
- Altitude Wait Helper
- ARM Gate Test
- Bias Residual Test
- Stall Credit Test
- Quaternion Type
- Attitude Gains Type
- Command Type
- Commander Config Type
- Board Config Type
- Mahony Config Type
- Baro Sample Type
- Battery Sample Type
- Mag Sample Type
- Alt Hold State Type
- PID Gains Type
- FSM State Type

## God Nodes (most connected - your core abstractions)
1. `PidTunerApp` - 102 edges
2. `flight_core_push_command()` - 45 edges
3. `MockFc` - 44 edges
4. `stabilize_task()` - 38 edges
5. `apply_command()` - 33 edges
6. `fc_bridge_push()` - 30 edges
7. `raise_if_queue_full()` - 27 edges
8. `UAV-S3 — Flight controller 2 tầng (C + MicroPython) cho ESP32-S3-WROOM-1 (miniUav)` - 25 edges
9. `flight_core_read_telemetry()` - 20 edges
10. `flight_core_start()` - 19 edges

## Surprising Connections (you probably didn't know these)
- `BENCH_MODE - full logic, no motor output` --semantically_similar_to--> `bench_start / +/- / bench_stop — manual throttle ramp for PID tuning`  [INFERRED] [semantically similar]
  flow.md → README.md
- `Graphify Workflow Policy (AGENTS.md)` --semantically_similar_to--> `Graphify Project Rules (CLAUDE.md)`  [INFERRED] [semantically similar]
  AGENTS.md → CLAUDE.md
- `ToF facade + dual backend compiled via #if, not CMake selection` --semantically_similar_to--> `Root-level PUBLIC include of main/ into flight_core`  [INFERRED] [semantically similar]
  components/flight_core/CMakeLists.txt → CMakeLists.txt
- `version.txt — git-describe workaround` --semantically_similar_to--> `flight_core REQUIRES esp_driver_i2c (new I2C master API)`  [INFERRED] [semantically similar]
  version.txt → components/flight_core/CMakeLists.txt
- `Attitude I-term dual gate (FSM gate AND throttle gate)` --semantically_similar_to--> `Pre-liftoff |I| is LIMITED, not frozen`  [INFERRED] [semantically similar]
  README.md → flow.md

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **Realtime determinism guarantees of the core-1 flight loop** — readme_core_affinity_policy, flow_priority_23_rationale, readme_imu_int_loop_clock, readme_int_fallback_clock, flow_tick_without_new_imu_sample, flow_fusion_dt, flow_i2c_timeout_two_values [INFERRED 0.85]
- **Layered motor-cut safety stack (logic latch, driver gate, cross-core spinlock)** — flow_kill_latch, flow_portmux_cross_core_race, readme_kill_path_exception, readme_commander, readme_flight_state_machine, flow_takeoff_abort_policy [INFERRED 0.85]
- **Terrain offset correctness chain (frames, accumulation, timeout, fail-safe, guard)** — flow_three_altitude_quantities, flow_agl_from_raw_measurement, flow_terrain_candidate_accumulation, flow_terrain_timeout_sample_gate, flow_terr_offset_stale, flow_terr_pending_deadlock, flow_terrain_clearance_guard_b8 [INFERRED 0.85]

## Communities (122 total, 61 thin omitted)

### Community 0 - "Altitude Estimator & Takeoff/Land"
Cohesion: 0.05
Nodes (86): alt_estimator_t, sensor_health_t, sensor_hub_age_us(), landing_state_t, takeoff_state_t, landing_reset(), takeoff_airborne(), takeoff_begin() (+78 more)

### Community 1 - "Flight Core Public API"
Cohesion: 0.05
Nodes (77): attitude_gains_t, command_t, commander_config_t, alt_hold_tune_t, telemetry_snapshot_t, flight_core_get_alt_hold_tune(), flight_core_get_attitude_gains(), flight_core_get_commander_cfg() (+69 more)

### Community 2 - "Flight Core API Test Harness"
Cohesion: 0.07
Nodes (21): FakeClock, fresh_import(), MockFc, patch_clock(), Test offline (CPython, KHÔNG cần MicroPython/phần cứng) cho logic BLOCKING của…, Đồng hồ giả: sleep() nhảy thời gian ngay lập tức thay vì chờ thật — test chạy…, Cài mock vào sys.modules['fc'] rồi (re)import fc_api sạch., test_calibrate_accel_6face_happy_path() (+13 more)

### Community 3 - "Telemetry & MicroPython Bridge"
Cohesion: 0.11
Nodes (47): telemetry_snapshot_t, telemetry_snapshot_init(), board_config_fill(), flight_core_board_config_t, command_t, move_dir_t, telemetry_snapshot_t, fc_bridge_get_control_input() (+39 more)

### Community 4 - "ToF Facade & I2C Discovery"
Cohesion: 0.08
Nodes (48): baro_sample_t, battery_sample_t, add_device_at(), esp_err_t, i2c_master_bus_handle_t, i2c_master_dev_handle_t, tof_reading_t, change_i2c_address() (+40 more)

### Community 5 - "Flight Doc: Control Sections"
Cohesion: 0.05
Nodes (37): (1) Timeout đếm bằng ĐỒNG HỒ, commit đếm bằng MẪU, (2) Timeout KHÔNG fail-safe, (3) Ứng viên offset phải CỘNG DỒN, 4.1 Commander chạy ở MỌI state đã armed, 4.1b ARM — cổng vào duy nhất, và vì sao nó không còn im lặng, 4.2 Chuỗi TAKEOFF — PID + slew-rate-limited target, 4.2b `hover_ff` chốt theo điện áp pin — `hover_model.h`, 4.3 Chuỗi LANDING — touchdown đa điều kiện (+29 more)

### Community 6 - "Mahony Attitude Filter"
Cohesion: 0.19
Nodes (34): vec3f_t, vec3f_zero(), apply_feedback_and_integrate(), mahony_config_t, mahony_t, vec3f_t, compute_accel_error(), compute_mag_error() (+26 more)

### Community 7 - "Flight Doc: Architecture & Tasks"
Cohesion: 0.06
Nodes (33): 1. Kiến trúc tổng quan, 2. Luồng một lệnh Python (ví dụ `fc.takeoff(800)`), 3.1 Bảng task — nguồn sự thật, 3.2 Vì sao chia như vậy, 3.2b Tốc độ I2C — một nguồn sự thật, 3.3 Ba kênh giao tiếp giữa task — và chiều dữ liệu, 3.4 Nhịp: một đồng hồ vật lý, hai tầng dự phòng, 3.4b Tick KHÔNG có mẫu IMU mới — cái gì chạy, cái gì không (+25 more)

### Community 9 - "README: Project Overview"
Cohesion: 0.06
Nodes (31): Altitude estimator (accel-primary + ToF/baro anchor), Bay qua console USB (firmware chính, không cần MicroPython), Bay thử từ REPL, Bench-test tăng ga tay để tune PID (`bench_start`/`+`/`-`/`bench_stop`), Build MicroPython port thật (CHƯA làm — bước tiếp theo của bạn), Build/nạp trực tiếp bằng PlatformIO — firmware CHÍNH, KHÔNG cần MicroPython, Calibration (gyro/accel/mag) — persist qua NVS, Commander (`commander.h`) (+23 more)

### Community 10 - "VL53L0X Driver"
Cohesion: 0.20
Nodes (30): esp_err_t, i2c_master_bus_handle_t, i2c_master_dev_handle_t, tof_sensor_state_t, calc_macro_period_ns(), decode_timeout(), decode_vcsel_period(), encode_timeout() (+22 more)

### Community 11 - "ToF Bring-up Test"
Cohesion: 0.08
Nodes (12): Bus, Chip, find_sensor_addr(), find_sensor_addr_OLD(), object, VL53L0X mo phong. addr: dia chi HIEN TAI, nam trong RAM chip. xshut_wired: day…, Cach lai SAI. Ngoai spec 2.8V -> chip khong con tin duoc., Ban chep tof_driver_init() single-sensor. Tra (err, final_addr, log). (+4 more)

### Community 12 - "VL53L1X Driver"
Cohesion: 0.30
Nodes (23): esp_err_t, i2c_master_bus_handle_t, i2c_master_dev_handle_t, tof_sensor_state_t, l1x_check_ready(), l1x_clear_interrupt(), l1x_read(), l1x_read16() (+15 more)

### Community 13 - "Takeoff/Landing Tuning"
Cohesion: 0.15
Nodes (22): alt_hold_state_t, landing_tune_t, takeoff_tune_t, flight_core_get_landing_tune(), flight_core_get_takeoff_tune(), alt_hold_tune_t, landing_state_t, landing_tune_t (+14 more)

### Community 14 - "MPU6050 IMU Driver"
Cohesion: 0.15
Nodes (19): esp_err_t, i2c_master_bus_handle_t, imu_calib_t, imu_sample_t, TaskHandle_t, vec3f_t, imu_driver_enable_data_ready_int(), imu_driver_init() (+11 more)

### Community 15 - "BMP280 Barometer Driver"
Cohesion: 0.22
Nodes (15): bmp280_calib_t, baro_driver_calibrate_ground(), baro_driver_init(), baro_driver_read(), baro_sample_t, esp_err_t, i2c_master_bus_handle_t, compensate_pressure() (+7 more)

### Community 16 - "Build & I2C Architecture Notes"
Cohesion: 0.12
Nodes (19): ESP-IDF project uav_s3_flight_core_verify, flight_core REQUIRES esp_driver_i2c (new I2C master API), CLIMB -> HOLD handover is seamless, Bumpless I load on the FIRST tick only, sensor_hub sub-rate interleaving with phase offsets, cfg.i2c_freq_hz was a dead field (fixed), Two I2C timeouts — 50ms init, 8ms runtime, sensor_hub task — sole owner of the I2C bus (+11 more)

### Community 17 - "Magnetometer Driver"
Cohesion: 0.32
Nodes (17): esp_err_t, i2c_master_bus_handle_t, mag_sample_t, cleanup_device(), invalidate_sample(), mag_driver_deinit(), mag_driver_get_latest(), mag_driver_init() (+9 more)

### Community 18 - "W/S Throttle Bench Test"
Cohesion: 0.15
Nodes (8): Bench, clampf(), clampi(), Fly, Test phim GIU W/S -> throttle offset momentary. Yeu cau: GIU W -> throttle…, TRANSCRIPTION nhanh FSM_HOLDING/FSM_FLYING sau khi W/S doi sang Vz., alt_hold_vz_cascade() rut gon: PI tren sai so vz, co clamp I., s_bench_throttle_duty + s_bench_throttle_offset (flight_core.c).

### Community 19 - "ARM & Takeoff Gate Concepts"
Cohesion: 0.15
Nodes (18): ARM gate — ordered prearm_check() and arm_reject_t reporting, FLYING -> HOLDING handover is already bumpless, GUI layout and setpoint keepalive cadence, hover_model.h — hover_ff latched from battery voltage at ARM, No I preload at the PRIME->CLIMB edge, PRIME_DUTY must be well below hover, R1/R2 source-reading regression tests, STATUS line format and STATUS_RE parsing contract (+10 more)

### Community 20 - "Cascade, Landing & Tick Concepts"
Cohesion: 0.14
Nodes (18): Commander evaluates in every armed state, CONTACT_CANDIDATE - 4-evidence touchdown debounce, Correlated telemetry pairs worth reading together, Disabling BOTH cascade stages was a real hole, FLYING vz source must be accel-only, FSM_FLYING - outer cascade stage off, inner stage kept, fusion_dt — integration uses sample-to-sample time, not tick-to-tick, LANDING sequence - multi-condition touchdown (+10 more)

### Community 21 - "Calibration NVS Persistence"
Cohesion: 0.33
Nodes (15): calibration_params_t, esp_err_t, vec3f_t, calibration_erase_all(), calibration_load(), calibration_nvs_init(), calibration_save_accel(), calibration_save_gyro() (+7 more)

### Community 22 - "Flight State Machine Impl"
Cohesion: 0.23
Nodes (15): fsm_state_t, fsm_on_arm_request(), fsm_on_bench_ramp_start(), fsm_on_bench_ramp_stop(), fsm_on_disarm_request(), fsm_on_emergency_resolve(), fsm_on_hard_fault(), fsm_on_land_request() (+7 more)

### Community 23 - "Two-Tier Architecture Principles"
Cohesion: 0.14
Nodes (16): command_queue (FreeRTOS, depth 8), fc_module.c / fc_bridge.c are marshal-only, Why stabilize is priority 23 and not 24, src/main.c — second command entry point (console USB + UDP), stabilize_task — the entire flight control loop, ESP-IDF task stack is measured in BYTES (documentation correction), Firmware-owned takeoff completion (alt_mm carried in command_t), Telemetry is strictly one-way outbound (+8 more)

### Community 24 - "GUI Manual Flight Keys"
Cohesion: 0.13
Nodes (3): Mode alt gan nhat tu STATUS ('2'=HOLD, '5'=FLYING, ...) hoac None., Bat dau giu W/S. Hanh vi phu thuoc state — xem khoi comment tren., Nha phim (hoac watchdog ket phim) -> BAT BUOC tra offset ve 0.

### Community 25 - "WiFi & UDP Net Link"
Cohesion: 0.17
Nodes (6): esp_event_base_t, esp_err_t, init_nvs_flash_safe(), init_wifi_sta(), net_link_init(), wifi_event_handler()

### Community 26 - "Terrain Offset & Clearance"
Cohesion: 0.23
Nodes (15): agl_m() must come from the raw measurement, not alt_m - terrain_off_m, Innovation gate and floor-measuring machinery - removed, LANDING runs on CLEARANCE, never absolute Z, Terrain telemetry fields (TOFF, TPEND, TCMT, TREJ, TTMO, TRES, TSTALE, TCAND, TCNT, TSEEN, CLR, FRAME), terr_offset_stale - timeout must fail SAFE, terr_pending deadlock - the bug that disabled this feature, Minimum clearance guard (B8) - the last safety net, Terrain offset - flying over furniture without losing reference (+7 more)

### Community 27 - "Trim Persistence Test"
Cohesion: 0.18
Nodes (7): attitude_target_OLD(), boot_load_WRONG(), clampf(), Core, object, Ban CU -- de chung minh no that su danh roi trim (test co y nghia)., Neu ai do quen hoan truc luc nap -- phai thay hau qua ro rang.

### Community 28 - "Feature Flags & Bench Gating"
Cohesion: 0.19
Nodes (13): Root-level PUBLIC include of main/ into flight_core, flight_core as standalone ESP-IDF component, ToF facade + dual backend compiled via #if, not CMake selection, BENCH_MODE - full logic, no motor output, BENCH_MODE blocks at the lowest layer, and still writes 0, fc_features.h compile-time feature flags, Kill latch + armed gate — two independent layers, Cross-core KILL race fixed with portMUX_TYPE (+5 more)

### Community 30 - "GUI Status Line Handlers"
Cohesion: 0.15
Nodes (5): Hiện VÌ SAO ARM bị từ chối, ngay trên thanh nút. Không có cái này thì GUI hoàn…, Hiện VÌ SAO chuỗi cất cánh không khởi động. Dùng CHUNG nhãn với pha cất cánh…, Nhãn pha cất cánh (closed-loop) cạnh nút TAKEOFF. Hiển thị theo pha THẬT của…, Khoa/mo nut TRIM. allow=False khi drone DANG BAY. Chan o GUI la lop DAU, khong…, Nhan nut W/S + dong goi y phai khop MODE HIEN TAI. Goi moi khi STATUS doi mode.…

### Community 31 - "Calibration API & Errors"
Cohesion: 0.18
Nodes (12): Exception, calibrate_accel_6face(), calibrate_gyro(), FcFaultError, FcTimeoutError, land(), Blocking: đo lại gyro bias tĩnh (~1.5s, DỪNG YÊN drone). Raise FcTimeoutError…, Blocking: chạy đủ 6 mặt. `prompt(face_idx)` (idx 0..5) nếu truyền vào sẽ được… (+4 more)

### Community 32 - "Calibration & Diagnostics Notes"
Cohesion: 0.21
Nodes (12): ARM anchors baro ground and ToF ground-ref (moved out of TAKEOFF), Bus lease — sensor_hub_suspend / resume for bench commands, Three-tier hardware diagnostics (i2c_scan, imu_int_test, tof_test), Two trim bugs fixed (timed commands and RAM-only persistence), Trim roll/pitch persisted to NVS, 6-face accel calibration verify (residual gate), BMP280 init separated from ground-zero calibration, Calibration persistence in NVS (gyro / accel / mag) (+4 more)

### Community 34 - "Attitude & Calibration Headers"
Cohesion: 0.18
Nodes (6): alt_hold_reset(), alt_hold_state_t, mahony_t, quat_t, mahony_quaternion(), clampi()

### Community 35 - "Hover Model Test"
Cohesion: 0.20
Nodes (4): dint(), dnum(), Model ga hover theo dien ap pin + latch mot lan luc ARM. TRANSCRIPTION cua…, Ring

### Community 39 - "Attitude Controller"
Cohesion: 0.27
Nodes (9): attitude_input_t, attitude_output_t, attitude_control_update(), attitude_default_gains(), axis_hold(), attitude_gains_t, attitude_state_t, wrap_deg_180() (+1 more)

### Community 41 - "Altitude Hold Cascade"
Cohesion: 0.47
Nodes (8): alt_hold_result_t, clampf(), alt_hold_default_tune(), alt_hold_preload(), alt_hold_run(), alt_hold_vz_cascade(), alt_hold_state_t, alt_hold_tune_t

### Community 42 - "Tilt Comp & Throttle Concepts"
Cohesion: 0.25
Nodes (9): Pre-liftoff |I| is LIMITED, not frozen, Pure slew rate limiter for target_z (not an exponential trajectory), cos(tilt) throttle feedforward compensation, ToF tilt compensation via quaternion rzz, ATT_MIN_THROTTLE_DUTY vs ATT_I_ENABLE_THROTTLE_DUTY, Attitude I-term dual gate (FSM gate AND throttle gate), liftoff_flag — simple Z threshold, gates nothing, 11-bit motor PWM at 24kHz (+1 more)

### Community 43 - "FLYING W/S Offset Test"
Cohesion: 0.22
Nodes (7): W/S trong FLYING = offset TAM THOI, KHONG duoc cong don. VI SAO CAN TEST NAY --…, Cach DUNG: latch la ga nen, offset chi anh huong tick nay., Cach SAI (ban cu): cong don vao latch., Bo comment truoc khi kiem 'code co lam X khong'. Khoi giai thich o…, strip_c_comments(), tick_dung(), tick_sai()

### Community 44 - "Hover Model Impl"
Cohesion: 0.36
Nodes (6): clampf_local(), hover_model_from_voltage(), hover_vbat_median(), hover_vbat_push(), hover_vbat_reset(), hover_vbat_ring_t

### Community 45 - "FSM Mode Sequence Test"
Cohesion: 0.25
Nodes (5): fsm_rule(), Chuoi FSM tu ARM -> TAKEOFF -> HOLDING -> FLYING -> LANDING, va ma MODE= ma GUI…, Bo comment truoc khi kiem 'code co dung X khong'. Khoi giai thich o…, Tra ve list (dieu_kien_state, state_dich) cua mot ham fsm_on_*., strip_c_comments()

### Community 47 - "GUI ToF Health Display"
Cohesion: 0.25
Nodes (4): Vi sao chip TU CHOI mau gan nhat. Tra chuoi ngan, hoac "" neu khong ro. Doc…, Ket luan ToF co dang chay khong. Tra (text, mau). HAI CAU HOI KHAC NHAU, truoc…, Tien to "Z<-NGUON" cho nhan do cao. Day la thong tin ma ADEGR KHONG noi duoc:…, Hien ToF DANG nhin be mat nao va co dang sua world-Z khong. Day la thong tin…

### Community 48 - "GUI Plot Panel"
Cohesion: 0.29
Nodes (3): PlotPanel, Scrolling multi-line strip chart drawn on a Tk Canvas. Mimics the Arduino IDE…, Append one sample. `values` maps channel key -> float.

### Community 50 - "Commander Impl"
Cohesion: 0.33
Nodes (6): commander_inputs_t, commander_result_t, commander_config_t, commander_state_t, commander_default_config(), commander_evaluate()

### Community 51 - "Attitude & PID State"
Cohesion: 0.38
Nodes (5): attitude_state_reset(), attitude_state_t, pid_state_t, pid_reset(), pid_state_init()

### Community 52 - "Commander Header API"
Cohesion: 0.38
Nodes (6): commander_clamp_altitude(), commander_credit_stall(), commander_heartbeat(), commander_init(), commander_config_t, commander_state_t

### Community 53 - "FSM Header API"
Cohesion: 0.38
Nodes (5): fsm_init(), fsm_state_name(), fsm_transition(), fsm_state_t, fsm_t

### Community 55 - "Terrain Accumulate Test"
Cohesion: 0.29
Nodes (3): BAC DIA HINH KHONG DEN TRONG MOT MAU -- ung vien offset phai CONG DON. LOI THAT…, Tra (committed, offset, ms). Mo phong dung logic estimator., sim()

### Community 56 - "ToF Alive vs Valid Test"
Cohesion: 0.29
Nodes (5): decide(), Phan biet "KHONG CO GI DE DO" voi "MAT CAM BIEN". VI SAO CAN TEST NAY: Dat…, Mo phong dung logic update_age() da sua., Bo comment truoc khi kiem tra 'code co ton tai khong'. Khong co buoc nay thi…, strip_comments()

### Community 59 - "Battery ADC Driver"
Cohesion: 0.53
Nodes (5): battery_driver_init(), battery_driver_read(), battery_driver_read_v(), battery_sample_t, esp_err_t

### Community 60 - "Altitude Validity Test"
Cohesion: 0.33
Nodes (3): Do cao con dung duoc khong: CHI hoi "chip con do khong". TRUOC DAY day la mot…, Bo comment truoc khi kiem 'code co ton tai khong'. Khong co buoc nay thi mot…, strip_comments()

### Community 61 - "Terrain Transition Test"
Cohesion: 0.33
Nodes (3): NHOM A -- terrain transition fail-safe, thu tu commit, clearance doc lap. BUG…, Tra (ket_qua, offset). sample_gate=True = ban moi (doi du mau)., sim()

### Community 65 - "PID Core"
Cohesion: 0.50
Nodes (4): pid_gains_t, pid_state_t, clamp_dt(), pid_update()

### Community 67 - "GUI ToF Health Test"
Cohesion: 0.40
Nodes (3): Fake, GUI phai phan biet "khong co gi de do" voi "mat cam bien". VI SAO CAN TEST NAY:…, Chi can TOF_STALE_MS + method that -- khong dung tkinter.

### Community 70 - "ToF Fault Tolerance Test"
Cohesion: 0.40
Nodes (3): Dung sai ToF: MOT mau xau KHONG duoc huy mot lan cat canh. BOI CANH (loi that,…, Comment CO QUYEN mo ta lai loi cu; chi CODE moi phai dung., strip_comments()

### Community 71 - "GUI Timed Command Group"
Cohesion: 0.50
Nodes (3): StringVar, FlightCommandGroup, Timed-command 1 phát (@HOVER/@MOVE/@YAW — xem command_parser.c…

### Community 72 - "UDP Console Entry Points"
Cohesion: 0.70
Nodes (4): main(), run_cli(), run_gui(), translate_command()

## Knowledge Gaps
- **84 isolated node(s):** `Bay thử từ REPL`, `Bench-test tăng ga tay để tune PID (`bench_start`/`+`/`-`/`bench_stop`)`, `Build MicroPython port thật (CHƯA làm — bước tiếp theo của bạn)`, `Build/nạp trực tiếp bằng PlatformIO — firmware CHÍNH, KHÔNG cần MicroPython`, `Calibration (gyro/accel/mag) — persist qua NVS` (+79 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **61 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `vec3f_zero()` connect `Mahony Attitude Filter` to `Magnetometer Driver`, `Attitude & Calibration Headers`?**
  _High betweenness centrality (0.033) - this node is a cross-community bridge._
- **Why does `flight_core_start()` connect `Altitude Estimator & Takeoff/Land` to `Flight Core Public API`, `Telemetry & MicroPython Bridge`, `ToF Facade & I2C Discovery`, `Takeoff/Landing Tuning`, `MPU6050 IMU Driver`, `Calibration NVS Persistence`?**
  _High betweenness centrality (0.028) - this node is a cross-community bridge._
- **Why does `flight_core_push_command()` connect `Flight Core Public API` to `Altitude Estimator & Takeoff/Land`, `Telemetry & MicroPython Bridge`?**
  _High betweenness centrality (0.027) - this node is a cross-community bridge._
- **Are the 42 inferred relationships involving `flight_core_push_command()` (e.g. with `fc_bridge_push()` and `handle_alt()`) actually correct?**
  _`flight_core_push_command()` has 42 INFERRED edges - model-reasoned connections that need verification._
- **Are the 24 inferred relationships involving `stabilize_task()` (e.g. with `alt_source_name()` and `sensor_hub_age_us()`) actually correct?**
  _`stabilize_task()` has 24 INFERRED edges - model-reasoned connections that need verification._
- **What connects `Bay thử từ REPL`, `Bench-test tăng ga tay để tune PID (`bench_start`/`+`/`-`/`bench_stop`)`, `Build MicroPython port thật (CHƯA làm — bước tiếp theo của bạn)` to the rest of the system?**
  _84 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Altitude Estimator & Takeoff/Land` be split into smaller, more focused modules?**
  _Cohesion score 0.053763440860215055 - nodes in this community are weakly interconnected._