# Graph Report - UAV-S3  (2026-09-09)

## Corpus Check
- 114 files · ~248,985 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 1485 nodes · 2714 edges · 138 communities (85 shown, 53 thin omitted)
- Extraction: 86% EXTRACTED · 14% INFERRED · 0% AMBIGUOUS · INFERRED: 372 edges (avg confidence: 0.85)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `401ac99f`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- flight_core.c
- main.c
- MockFc
- fc_module.c
- sensor_hub.c
- 4.2 Chuỗi TAKEOFF — PID + slew-rate-limited target
- mahony_filter.c
- 3. Phân chia task RTOS
- ._log
- UAV-S3 — Flight controller 2 tầng (C + MicroPython) cho ESP32-S3-WROOM-1 (miniUav)
- vl53l0x_driver.c
- test_tof_bringup_offline.py
- VL53L1X Driver
- sensor_hub task — sole owner of the I2C bus
- imu_driver.c
- BMP280 Barometer Driver
- cpu_bench.c
- mag_driver.c
- W/S Throttle Bench Test
- TAKEOFF sequence — PRIME / CLIMB / HOLD with slew-rate-limited target
- Altitude Estimator (accel-primary, ToF/baro anchor)
- calibration.c
- stabilize_task
- Test D — records a known gap, not a protection
- .send_raw_cmd
- net_link.c
- Terrain offset - flying over furniture without losing reference
- Core
- main/app_config.h — sensor enable flags and motor confirmation
- fc_api.py
- ._handle_line
- FcTimeoutError
- Calibration persistence in NVS (gyro / accel / mag)
- flight_core_start
- test_hover_model_offline.py
- ._build_ui
- types.h
- attitude_control_update
- PidTunerApp
- alt_estimator.c
- Pure slew rate limiter for target_z (not an exponential trajectory)
- test_flying_ws_no_accumulate_offline.py
- hover_model.c
- test_flight_mode_sequence_offline.py
- GainRow
- PlotPanel
- CameraPanel
- UavUdpConsole
- commander_evaluate
- takeoff_run
- motor_driver.c
- flight_core_read_telemetry
- Heartbeat
- test_terrain_accumulate_offline.py
- test_tof_alive_vs_valid_offline.py
- FlightAltGroup
- GainGroup
- test_battery_floor_offline.py
- test_alt_validity_offline.py
- test_terrain_transition_offline.py
- CommanderGroup
- LandingGroup
- TakeoffGroup
- UAV-S3 — Luồng code end-to-end
- test_flying_vz_hold_offline.py
- test_gui_tof_health_offline.py
- test_terrain_deadlock_offline.py
- test_tilt_comp_expo_offline.py
- test_tof_fault_tolerance_offline.py
- FlightCommandGroup
- uav_udp_console.py
- test_flying_alt_pid_offline.py
- test_gyro_calibration_offline.py
- test_terrain_rebase_offline.py
- CalibrationGroup
- TestMotorGroup
- test_alt_estimator_offline.py
- test_alt_tof_surface_offline.py
- test_status_parse_offline.py
- test_takeoff_flow_offline.py
- test_takeoff_gate_offline.py
- AGENTS.md
- Graphify Workflow Policy (AGENTS.md)
- CLAUDE.md
- example_mission.py
- bench_start
- bench_step
- bench_stop
- calib_status
- calibrate_baro_ground
- calibrate_erase
- calibrate_gyro_abort
- calibrate_mag
- calibrate_mag_abort
- control
- get_states
- hover
- kill
- move
- test_motor
- set_flightmode
- _wait_altitude
- test_arm_gate_offline.py
- test_bias_residual_offline.py
- test_stall_credit_offline.py
- test_degraded_clock_offline.py
- color_detect.py
- 4.6 Terrain offset — bay qua bàn/ghế mà không mất tham chiếu
- test_ground_stale_tof_offline.py
- test_terrain_anchor_offline.py
- test_alt_frame_target_offline.py
- 4. FlightStateMachine (topology)
- 3.7 Vòng lặp `stabilize_task` (250Hz, core 1, `flight_core.c`)
- 6. Telemetry an toàn/realtime — đọc gì khi soi log
- test_terrain_threshold_offline.py
- command_parser.c
- 4.4 Altitude estimator — accel-primary, ToF CHỈ LÀ correction
- app_config.h
- 4.7 FSM_FLYING — tắt tầng NGOÀI, giữ tầng TRONG
- camera_driver.c
- clampf
- test_mjpeg_parser_offline.py
- sensor_hub_age_us
- test_camera_display_offline.py
- STATUS line format and STATUS_RE parsing contract
- test_telemetry_minimal_offline.py
- tof_driver.c
- CameraStreamWorker
- ColorDetector
- flight_core_tof_reinit
- MjpegParser
- ._update_tof_label
- battery_driver_read
- sensor_hub_start
- flight_core_get_task_stats

## God Nodes (most connected - your core abstractions)
1. `PidTunerApp` - 103 edges
2. `stabilize_task()` - 64 edges
3. `apply_command()` - 56 edges
4. `flight_core_push_command()` - 45 edges
5. `MockFc` - 44 edges
6. `flight_core_start()` - 31 edges
7. `fc_bridge_push()` - 30 edges
8. `raise_if_queue_full()` - 27 edges
9. `UAV-S3 — Flight controller 2 tầng (C + MicroPython) cho ESP32-S3-WROOM-1 (miniUav)` - 25 edges
10. `flight_core_read_telemetry()` - 21 edges

## Surprising Connections (you probably didn't know these)
- `BENCH_MODE - full logic, no motor output` --semantically_similar_to--> `bench_start / +/- / bench_stop — manual throttle ramp for PID tuning`  [INFERRED] [semantically similar]
  flow.md → README.md
- `Timeout counted by CLOCK vs commit counted by SAMPLES` --semantically_similar_to--> `seq-based sample consumption (replaces tick divisors)`  [INFERRED] [semantically similar]
  flow.md → README.md
- `version.txt — git-describe workaround` --semantically_similar_to--> `flight_core REQUIRES esp_driver_i2c (new I2C master API)`  [INFERRED] [semantically similar]
  version.txt → components/flight_core/CMakeLists.txt
- `I2C 100kHz debug-phase speed (later 400kHz)` --semantically_similar_to--> `cfg.i2c_freq_hz was a dead field (fixed)`  [INFERRED] [semantically similar]
  README.md → flow.md
- `ToF facade + dual backend compiled via #if, not CMake selection` --semantically_similar_to--> `Root-level PUBLIC include of main/ into flight_core`  [INFERRED] [semantically similar]
  components/flight_core/CMakeLists.txt → CMakeLists.txt

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **Realtime determinism guarantees of the core-1 flight loop** — readme_core_affinity_policy, flow_priority_23_rationale, readme_imu_int_loop_clock, readme_int_fallback_clock, flow_tick_without_new_imu_sample, flow_fusion_dt, flow_i2c_timeout_two_values [INFERRED 0.85]
- **Layered motor-cut safety stack (logic latch, driver gate, cross-core spinlock)** — flow_kill_latch, flow_portmux_cross_core_race, readme_kill_path_exception, readme_commander, readme_flight_state_machine, flow_takeoff_abort_policy [INFERRED 0.85]
- **Terrain offset correctness chain (frames, accumulation, timeout, fail-safe, guard)** — flow_three_altitude_quantities, flow_agl_from_raw_measurement, flow_terrain_candidate_accumulation, flow_terrain_timeout_sample_gate, flow_terr_offset_stale, flow_terr_pending_deadlock, flow_terrain_clearance_guard_b8 [INFERRED 0.85]

## Communities (138 total, 53 thin omitted)

### Community 0 - "flight_core.c"
Cohesion: 0.14
Nodes (29): vec3f_t, vec3f_zero(), imu_driver_get_config(), imu_sample_t, TickType_t, vec3f_t, enter_kill_latch(), enter_kill_latch_ex() (+21 more)

### Community 1 - "main.c"
Cohesion: 0.13
Nodes (25): command_t, flight_core_push_command(), move_dir_t, cmd_alt(), cmd_arm(), cmd_bench_start(), cmd_bench_stop(), cmd_bench_throttle_down() (+17 more)

### Community 2 - "MockFc"
Cohesion: 0.07
Nodes (21): FakeClock, fresh_import(), MockFc, patch_clock(), Test offline (CPython, KHÔNG cần MicroPython/phần cứng) cho logic BLOCKING của…, Đồng hồ giả: sleep() nhảy thời gian ngay lập tức thay vì chờ thật — test chạy…, Cài mock vào sys.modules['fc'] rồi (re)import fc_api sạch., test_calibrate_accel_6face_happy_path() (+13 more)

### Community 3 - "fc_module.c"
Cohesion: 0.11
Nodes (46): telemetry_snapshot_t, telemetry_snapshot_init(), flight_core_get_control_input(), command_t, move_dir_t, telemetry_snapshot_t, fc_bridge_get_control_input(), fc_bridge_init() (+38 more)

### Community 4 - "sensor_hub.c"
Cohesion: 0.19
Nodes (19): baro_sample_t, battery_sample_t, imu_sample_t, mag_sample_t, sensor_health_t, TickType_t, tof_reading_t, mark_err() (+11 more)

### Community 5 - "4.2 Chuỗi TAKEOFF — PID + slew-rate-limited target"
Cohesion: 0.18
Nodes (11): 4.2 Chuỗi TAKEOFF — PID + slew-rate-limited target, 4.2b `hover_ff` chốt theo điện áp pin — `hover_model.h`, ABORT — mọi lý do đều dẫn về EMERGENCY, không tự chọn policy, Bàn giao CLIMB → HOLD: LIỀN MẠCH, Cạnh PRIME → CLIMB: `I` khởi đầu = 0, KHÔNG preload, Hai điểm review đã khoá thành test hồi quy (R1/R2), `|I|` trước liftoff: GIỚI HẠN, KHÔNG ĐÓNG BĂNG, `liftoff_flag` — CHỈ LÀ THÔNG TIN, không gate gì (+3 more)

### Community 6 - "mahony_filter.c"
Cohesion: 0.19
Nodes (34): mahony_config_t, flight_core_get_mahony_config(), apply_feedback_and_integrate(), mahony_config_t, mahony_t, vec3f_t, compute_accel_error(), compute_mag_error() (+26 more)

### Community 7 - "3. Phân chia task RTOS"
Cohesion: 0.17
Nodes (12): 3.1 Bảng task — nguồn sự thật, 3.2 Vì sao chia như vậy, 3.2b Tốc độ I2C — một nguồn sự thật, 3.3 Ba kênh giao tiếp giữa task — và chiều dữ liệu, 3.4 Nhịp: một đồng hồ vật lý, hai tầng dự phòng, 3.4b Tick KHÔNG có mẫu IMU mới — cái gì chạy, cái gì không, 3.5 Nhịp con của sensor_hub — vì sao xen kẽ, không gộp, 3.5b Timeout I2C — HAI giá trị, hai mục đích (+4 more)

### Community 9 - "UAV-S3 — Flight controller 2 tầng (C + MicroPython) cho ESP32-S3-WROOM-1 (miniUav)"
Cohesion: 0.06
Nodes (31): Altitude estimator (accel-primary + ToF/baro anchor), Bay qua console USB (firmware chính, không cần MicroPython), Bay thử từ REPL, Bench-test tăng ga tay để tune PID (`bench_start`/`+`/`-`/`bench_stop`), Build MicroPython port thật (CHƯA làm — bước tiếp theo của bạn), Build/nạp trực tiếp bằng PlatformIO — firmware CHÍNH, KHÔNG cần MicroPython, Calibration (gyro/accel/mag) — persist qua NVS, Commander (`commander.h`) (+23 more)

### Community 10 - "vl53l0x_driver.c"
Cohesion: 0.20
Nodes (30): esp_err_t, i2c_master_bus_handle_t, i2c_master_dev_handle_t, tof_sensor_state_t, calc_macro_period_ns(), decode_timeout(), decode_vcsel_period(), encode_timeout() (+22 more)

### Community 11 - "test_tof_bringup_offline.py"
Cohesion: 0.08
Nodes (12): Bus, Chip, find_sensor_addr(), find_sensor_addr_OLD(), object, VL53L0X mo phong. addr: dia chi HIEN TAI, nam trong RAM chip. xshut_wired: day…, Cach lai SAI. Ngoai spec 2.8V -> chip khong con tin duoc., Ban chep tof_driver_init() single-sensor. Tra (err, final_addr, log). (+4 more)

### Community 12 - "VL53L1X Driver"
Cohesion: 0.30
Nodes (23): esp_err_t, i2c_master_bus_handle_t, i2c_master_dev_handle_t, tof_sensor_state_t, l1x_check_ready(), l1x_clear_interrupt(), l1x_read(), l1x_read16() (+15 more)

### Community 13 - "sensor_hub task — sole owner of the I2C bus"
Cohesion: 0.20
Nodes (12): sensor_hub sub-rate interleaving with phase offsets, Two I2C timeouts — 50ms init, 8ms runtime, sensor_hub task — sole owner of the I2C bus, sensor_snapshot_t (value + seq + timestamp + valid + healthy), Timeout counted by CLOCK vs commit counted by SAMPLES, main/board_config.h — real pins from the miniUav schematic, Control loop clocked by MPU6050 data-ready interrupt (IO36), Interrupt-loss fallback to the FreeRTOS clock (+4 more)

### Community 14 - "imu_driver.c"
Cohesion: 0.21
Nodes (14): esp_err_t, i2c_master_bus_handle_t, imu_calib_t, imu_sample_t, TaskHandle_t, vec3f_t, imu_driver_enable_data_ready_int(), imu_driver_init() (+6 more)

### Community 15 - "BMP280 Barometer Driver"
Cohesion: 0.22
Nodes (18): bmp280_calib_t, baro_driver_calibrate_ground(), baro_driver_ground_healthy(), baro_driver_ground_noise_std_pa(), baro_driver_ground_ready(), baro_driver_init(), baro_driver_read(), baro_sample_t (+10 more)

### Community 16 - "cpu_bench.c"
Cohesion: 0.22
Nodes (6): cpu_bench_t, cpu_bench_read(), cpu_bench_reset(), tick_window(), cmd_cpu(), cmd_cpu_reset()

### Community 17 - "mag_driver.c"
Cohesion: 0.32
Nodes (18): esp_err_t, i2c_master_bus_handle_t, mag_sample_t, cleanup_device(), invalidate_sample(), mag_driver_deinit(), mag_driver_get_latest(), mag_driver_init() (+10 more)

### Community 18 - "W/S Throttle Bench Test"
Cohesion: 0.15
Nodes (8): Bench, clampf(), clampi(), Fly, Test phim GIU W/S -> throttle offset momentary. Yeu cau: GIU W -> throttle…, TRANSCRIPTION nhanh FSM_HOLDING/FSM_FLYING sau khi W/S doi sang Vz., alt_hold_vz_cascade() rut gon: PI tren sai so vz, co clamp I., s_bench_throttle_duty + s_bench_throttle_offset (flight_core.c).

### Community 19 - "TAKEOFF sequence — PRIME / CLIMB / HOLD with slew-rate-limited target"
Cohesion: 0.17
Nodes (16): CONTACT_CANDIDATE - 4-evidence touchdown debounce, Disabling BOTH cascade stages was a real hole, FLYING -> HOLDING handover is already bumpless, FSM_FLYING - outer cascade stage off, inner stage kept, hover_model.h — hover_ff latched from battery voltage at ARM, LANDING sequence - multi-condition touchdown, No I preload at the PRIME->CLIMB edge, PRIME_DUTY must be well below hover (+8 more)

### Community 20 - "Altitude Estimator (accel-primary, ToF/baro anchor)"
Cohesion: 0.18
Nodes (13): Commander evaluates in every armed state, Correlated telemetry pairs worth reading together, FLYING vz source must be accel-only, fusion_dt — integration uses sample-to-sample time, not tick-to-tick, ulTaskNotifyTake(pdTRUE) — no catch-up on backlog, Terrain residual expected term must use vz_accel_only_ms, Tick with no new IMU sample — what runs, what is skipped, Altitude Estimator (accel-primary, ToF/baro anchor) (+5 more)

### Community 21 - "calibration.c"
Cohesion: 0.33
Nodes (15): calibration_params_t, esp_err_t, vec3f_t, calibration_erase_all(), calibration_load(), calibration_nvs_init(), calibration_save_accel(), calibration_save_gyro() (+7 more)

### Community 22 - "stabilize_task"
Cohesion: 0.14
Nodes (29): commander_clamp_altitude(), commander_config_t, fsm_init(), fsm_transition(), abort_landing_to_hold(), apply_command(), apply_set_param(), finalize_mag_calibration() (+21 more)

### Community 23 - "Test D — records a known gap, not a protection"
Cohesion: 0.22
Nodes (9): ESP-IDF project uav_s3_flight_core_verify, flight_core REQUIRES esp_driver_i2c (new I2C master API), CLIMB -> HOLD handover is seamless, Bumpless I load on the FIRST tick only, cfg.i2c_freq_hz was a dead field (fixed), Test D — records a known gap, not a protection, I2C 100kHz debug-phase speed (later 400kHz), I2C Architecture — one shared bus, new i2c_master API (+1 more)

### Community 24 - ".send_raw_cmd"
Cohesion: 0.14
Nodes (3): Mode alt gan nhat tu STATUS ('2'=HOLD, '5'=FLYING, ...) hoac None., Bat dau giu W/S. Hanh vi phu thuoc state — xem khoi comment tren., Nha phim (hoac watchdog ket phim) -> BAT BUOC tra offset ve 0.

### Community 25 - "net_link.c"
Cohesion: 0.13
Nodes (20): esp_event_base_t, net_link_ap_t, command_parser_telemetry_enabled(), cmd_tasks(), cmd_wifi_test(), net_task(), print_stack_row(), esp_err_t (+12 more)

### Community 26 - "Terrain offset - flying over furniture without losing reference"
Cohesion: 0.23
Nodes (15): agl_m() must come from the raw measurement, not alt_m - terrain_off_m, Innovation gate and floor-measuring machinery - removed, LANDING runs on CLEARANCE, never absolute Z, Terrain telemetry fields (TOFF, TPEND, TCMT, TREJ, TTMO, TRES, TSTALE, TCAND, TCNT, TSEEN, CLR, FRAME), terr_offset_stale - timeout must fail SAFE, terr_pending deadlock - the bug that disabled this feature, Minimum clearance guard (B8) - the last safety net, Terrain offset - flying over furniture without losing reference (+7 more)

### Community 27 - "Core"
Cohesion: 0.18
Nodes (7): attitude_target_OLD(), boot_load_WRONG(), clampf(), Core, object, Ban CU -- de chung minh no that su danh roi trim (test co y nghia)., Neu ai do quen hoan truc luc nap -- phai thay hau qua ro rang.

### Community 28 - "main/app_config.h — sensor enable flags and motor confirmation"
Cohesion: 0.19
Nodes (13): Root-level PUBLIC include of main/ into flight_core, flight_core as standalone ESP-IDF component, ToF facade + dual backend compiled via #if, not CMake selection, BENCH_MODE - full logic, no motor output, BENCH_MODE blocks at the lowest layer, and still writes 0, fc_features.h compile-time feature flags, Kill latch + armed gate — two independent layers, Cross-core KILL race fixed with portMUX_TYPE (+5 more)

### Community 30 - "._handle_line"
Cohesion: 0.13
Nodes (6): Hiện VÌ SAO ARM bị từ chối, ngay trên thanh nút. Không có cái này thì GUI hoàn…, Hiện VÌ SAO chuỗi cất cánh không khởi động. Dùng CHUNG nhãn với pha cất cánh…, Nhãn pha cất cánh (closed-loop) cạnh nút TAKEOFF. Hiển thị theo pha THẬT của…, Khoa/mo nut TRIM. allow=False khi drone DANG BAY. Chan o GUI la lop DAU, khong…, Nhan nut W/S + dong goi y phai khop MODE HIEN TAI. Goi moi khi STATUS doi mode.…, TELEMETRY_LEVEL=1: chi co ARM/THR/R/P/ALTm/TGT. Cap nhat dung nhung o do. Cac…

### Community 31 - "FcTimeoutError"
Cohesion: 0.18
Nodes (12): Exception, calibrate_accel_6face(), calibrate_gyro(), FcFaultError, FcTimeoutError, land(), Blocking: đo lại gyro bias tĩnh (~1.5s, DỪNG YÊN drone). Raise FcTimeoutError…, Blocking: chạy đủ 6 mặt. `prompt(face_idx)` (idx 0..5) nếu truyền vào sẽ được… (+4 more)

### Community 32 - "Calibration persistence in NVS (gyro / accel / mag)"
Cohesion: 0.17
Nodes (15): ARM anchors baro ground and ToF ground-ref (moved out of TAKEOFF), ARM gate — ordered prearm_check() and arm_reject_t reporting, Bus lease — sensor_hub_suspend / resume for bench commands, CMD_TAKEOFF gate — takeoff_reject_t (separate from prearm_check), Three-tier hardware diagnostics (i2c_scan, imu_int_test, tof_test), Two trim bugs fixed (timed commands and RAM-only persistence), Trim roll/pitch persisted to NVS, 6-face accel calibration verify (residual gate) (+7 more)

### Community 34 - "flight_core_start"
Cohesion: 0.20
Nodes (15): alt_hold_reset(), alt_hold_state_t, landing_state_t, takeoff_state_t, landing_reset(), takeoff_airborne(), takeoff_begin(), takeoff_control_active() (+7 more)

### Community 35 - "test_hover_model_offline.py"
Cohesion: 0.20
Nodes (4): dint(), dnum(), Model ga hover theo dien ap pin + latch mot lan luc ARM. TRANSCRIPTION cua…, Ring

### Community 38 - "types.h"
Cohesion: 0.09
Nodes (14): alt_source_t, alt_source_name(), attitude_state_reset(), attitude_state_t, commander_credit_stall(), commander_heartbeat(), commander_init(), commander_state_t (+6 more)

### Community 39 - "attitude_control_update"
Cohesion: 0.15
Nodes (15): attitude_input_t, attitude_output_t, attitude_control_update(), attitude_default_gains(), axis_hold(), attitude_gains_t, attitude_state_t, wrap_deg_180() (+7 more)

### Community 41 - "alt_estimator.c"
Cohesion: 0.18
Nodes (23): alt_estimator_t, alt_estimator_agl_m(), alt_estimator_confirm_liftoff(), alt_estimator_correct_velocity(), alt_estimator_floor_ready(), alt_estimator_height_above_landing_surface(), alt_estimator_lock_floor(), alt_estimator_lock_floor_at_zero() (+15 more)

### Community 42 - "Pure slew rate limiter for target_z (not an exponential trajectory)"
Cohesion: 0.25
Nodes (9): Pre-liftoff |I| is LIMITED, not frozen, Pure slew rate limiter for target_z (not an exponential trajectory), cos(tilt) throttle feedforward compensation, ToF tilt compensation via quaternion rzz, ATT_MIN_THROTTLE_DUTY vs ATT_I_ENABLE_THROTTLE_DUTY, Attitude I-term dual gate (FSM gate AND throttle gate), liftoff_flag — simple Z threshold, gates nothing, 11-bit motor PWM at 24kHz (+1 more)

### Community 43 - "test_flying_ws_no_accumulate_offline.py"
Cohesion: 0.22
Nodes (7): W/S trong FLYING = offset TAM THOI, KHONG duoc cong don. VI SAO CAN TEST NAY --…, Cach DUNG: latch la ga nen, offset chi anh huong tick nay., Cach SAI (ban cu): cong don vao latch., Bo comment truoc khi kiem 'code co lam X khong'. Khoi giai thich o…, strip_c_comments(), tick_dung(), tick_sai()

### Community 44 - "hover_model.c"
Cohesion: 0.36
Nodes (7): clampf_local(), hover_model_from_voltage(), hover_model_prime_duty(), hover_vbat_median(), hover_vbat_push(), hover_vbat_reset(), hover_vbat_ring_t

### Community 45 - "test_flight_mode_sequence_offline.py"
Cohesion: 0.25
Nodes (5): fsm_rule(), Chuoi FSM tu ARM -> TAKEOFF -> HOLDING -> FLYING -> LANDING, va ma MODE= ma GUI…, Bo comment truoc khi kiem 'code co dung X khong'. Khoi giai thich o…, Tra ve list (dieu_kien_state, state_dich) cua mot ham fsm_on_*., strip_c_comments()

### Community 47 - "PlotPanel"
Cohesion: 0.22
Nodes (3): PlotPanel, Scrolling multi-line strip chart drawn on a Tk Canvas. Mimics the Arduino IDE…, Append one sample. `values` maps channel key -> float.

### Community 48 - "CameraPanel"
Cohesion: 0.16
Nodes (6): CameraPanel, Panel video MJPEG. Doc anh tu CameraStreamWorker (thread rieng). NGUYEN TAC:…, Chay TREN THREAD WORKER, ngay sau khi giai ma. Khong cham Tk o day. Moi thu…, Kiem ba thu vien bat buoc. Goi mot lan, nho ket qua., Goi khi dong app -- thread la daemon nhung van nen dung tu te., numpy BGR -> tk.PhotoImage, qua PPM. KHONG dung Pillow. Da DO tren chinh may…

### Community 50 - "commander_evaluate"
Cohesion: 0.25
Nodes (8): commander_inputs_t, commander_result_t, commander_config_t, commander_state_t, commander_default_config(), commander_evaluate(), commander_config_t, flight_core_get_commander_cfg()

### Community 51 - "takeoff_run"
Cohesion: 0.23
Nodes (15): takeoff_tune_t, flight_core_get_takeoff_tune(), alt_hold_state_t, alt_hold_tune_t, takeoff_state_t, takeoff_tune_t, takeoff_default_tune(), takeoff_run() (+7 more)

### Community 52 - "motor_driver.c"
Cohesion: 0.25
Nodes (12): clampi(), esp_err_t, configure_channel(), motor_driver_all_off(), motor_driver_arm(), motor_driver_disarm(), motor_driver_init(), motor_driver_set_armed() (+4 more)

### Community 53 - "flight_core_read_telemetry"
Cohesion: 0.15
Nodes (15): fsm_arm_guard_ok(), fsm_state_name(), fsm_state_t, tof_driver_stall_restarts(), telemetry_snapshot_t, flight_core_read_telemetry(), handle_single_char(), cmd_cal_status() (+7 more)

### Community 55 - "test_terrain_accumulate_offline.py"
Cohesion: 0.29
Nodes (3): BAC DIA HINH KHONG DEN TRONG MOT MAU -- ung vien offset phai CONG DON. LOI THAT…, Tra (committed, offset, ms). Mo phong dung logic estimator., sim()

### Community 56 - "test_tof_alive_vs_valid_offline.py"
Cohesion: 0.29
Nodes (5): decide(), Phan biet "KHONG CO GI DE DO" voi "MAT CAM BIEN". VI SAO CAN TEST NAY: Dat…, Mo phong dung logic update_age() da sua., Bo comment truoc khi kiem tra 'code co ton tai khong'. Khong co buoc nay thi…, strip_comments()

### Community 59 - "test_battery_floor_offline.py"
Cohesion: 0.18
Nodes (7): batv(), first_below(), med(), San pin -> ep LANDING, va vi sao BAT BUOC phai co debounce. Yeu cau: BATV <=…, Chi so mau dau tien bao 'duoi san'; None neu khong bao gio., Bo comment truoc khi quet "code co lam X khong". Khoi giai thich cua chinh test…, strip_c_comments()

### Community 60 - "test_alt_validity_offline.py"
Cohesion: 0.33
Nodes (3): Do cao con dung duoc khong: CHI hoi "chip con do khong". TRUOC DAY day la mot…, Bo comment truoc khi kiem 'code co ton tai khong'. Khong co buoc nay thi mot…, strip_comments()

### Community 61 - "test_terrain_transition_offline.py"
Cohesion: 0.33
Nodes (3): NHOM A -- terrain transition fail-safe, thu tu commit, clearance doc lap. BUG…, Tra (ket_qua, offset). sample_gate=True = ban moi (doi du mau)., sim()

### Community 65 - "UAV-S3 — Luồng code end-to-end"
Cohesion: 0.20
Nodes (9): 1. Kiến trúc tổng quan, 2. Luồng một lệnh Python (ví dụ `fc.takeoff(800)`), 5. Tổng kết file/module theo lớp, 7.1 Giao thức UDP — hai chế độ ký tự, 7.2 Ba chẩn đoán phần cứng — xem mục 3.6b, 7.3 GUI (`tools/uav_udp_console.py`), 7.4 Trim roll/pitch — lưu NVS, 7. Ground station — console USB + UDP (lối vào thứ hai) (+1 more)

### Community 67 - "test_gui_tof_health_offline.py"
Cohesion: 0.40
Nodes (3): Fake, GUI phai phan biet "khong co gi de do" voi "mat cam bien". VI SAO CAN TEST NAY:…, Chi can TOF_STALE_MS + method that -- khong dung tkinter.

### Community 70 - "test_tof_fault_tolerance_offline.py"
Cohesion: 0.40
Nodes (3): Dung sai ToF: MOT mau xau KHONG duoc huy mot lan cat canh. BOI CANH (loi that,…, Comment CO QUYEN mo ta lai loi cu; chi CODE moi phai dung., strip_comments()

### Community 71 - "FlightCommandGroup"
Cohesion: 0.50
Nodes (3): StringVar, FlightCommandGroup, Timed-command 1 phát (@HOVER/@MOVE/@YAW — xem command_parser.c…

### Community 72 - "uav_udp_console.py"
Cohesion: 0.70
Nodes (4): main(), run_cli(), run_gui(), translate_command()

### Community 108 - "test_degraded_clock_offline.py"
Cohesion: 0.20
Nodes (7): NHOM A2 -- dong ho `degraded` phai DOC LAP voi trang thai terrain. LOI THAT (do…, Tra ve (co_fault, ms_fault). ToF KHOE suot: cu TOF_PERIOD_MS lai co mot mau hop…, ToF ngung tra mau han tu t=0., Bo comment truoc khi kiem 'code co lam X khong'. Khoi giai thich o day dai va…, sim(), sim_dead(), strip_c_comments()

### Community 109 - "color_detect.py"
Cohesion: 0.13
Nodes (6): Nhan dien mau: kiem tren anh TU SINH, khong can camera/drone/mang. VI SAO CAN…, ColorProfile, Detection, Nhan dien mau tren khung hinh BGR cua OpenCV. Module nay chi tra loi "thay gi,…, Ten + DANH SACH khoang HSV + nguong dien tich. La danh sach vi mau do vat qua…, Mot vet mau tim duoc. Xem docstring dau file.

### Community 110 - "4.6 Terrain offset — bay qua bàn/ghế mà không mất tham chiếu"
Cohesion: 0.22
Nodes (9): (1) Timeout đếm bằng ĐỒNG HỒ, commit đếm bằng MẪU, (2) Timeout KHÔNG fail-safe, (3) Ứng viên offset phải CỘNG DỒN, 4.6 Terrain offset — bay qua bàn/ghế mà không mất tham chiếu, ⚠ `agl_m()` phải lấy từ SỐ ĐO THÔ, không phải `alt_m − terrain_off_m`, ⚠ Chuyển tiếp terrain — BA lỗi độc lập, đã sửa cả ba, Guard khoảng hở tối thiểu (B8) — lưới an toàn CUỐI, ⚠ Vòng luẩn quẩn `terr_pending` — lỗi đã làm feature này bị TẮT (+1 more)

### Community 111 - "test_ground_stale_tof_offline.py"
Cohesion: 0.22
Nodes (5): expected_bias(), Trang thai ToF DAN XUAT phai bi xoa khi nam dat. LOI THAT, do duoc tren log cat…, Sai so cua `expected = vz_accel_only_ms * dt` do seed sai., Bo comment truoc khi kiem 'code co lam X khong'. Khoi giai thich o day trich…, strip_c_comments()

### Community 112 - "test_terrain_anchor_offline.py"
Cohesion: 0.25
Nodes (3): Offset terrain phai tinh tu MOC NEO, khong phai cong don cac mau vuot nguong.…, Khoi giai thich trich dan chinh cong thuc cu -> phai bo truoc khi kiem., strip_c_comments()

### Community 113 - "test_alt_frame_target_offline.py"
Cohesion: 0.25
Nodes (5): effective_target(), Frame do cao = DATUM, va TARGET HIEU DUNG so voi be mat (TGTS). NGU NGHIA DA…, Bo comment: khoi giai thich o day trich dan chinh cac ten bien dang kiem., Target hieu dung so voi BE MAT dang bay tren., strip_c_comments()

### Community 114 - "4. FlightStateMachine (topology)"
Cohesion: 0.29
Nodes (7): 4.1 Commander chạy ở MỌI state đã armed, 4.1b ARM — cổng vào duy nhất, và vì sao nó không còn im lặng, 4.3 Chuỗi LANDING — touchdown đa điều kiện, 4.5 Bù cos(tilt) cho throttle — feedforward, 4.8 BENCH_MODE — chạy full logic, KHÔNG xuất ra motor, 4. FlightStateMachine (topology), Heartbeat watchdog — bằng chứng phải đến TỪ BÊN NGOÀI

### Community 115 - "3.7 Vòng lặp `stabilize_task` (250Hz, core 1, `flight_core.c`)"
Cohesion: 0.33
Nodes (6): 3.7.1 Hai task, một chiều dữ liệu, 3.7.2 Một tick của stabilize_task, 3.7.3 Kill path — quyền ưu tiên cao nhất, 3.7.4 Bước 9/9c — khi nào I-term (Ki) được cộng dồn, 3.7 Vòng lặp `stabilize_task` (250Hz, core 1, `flight_core.c`), KILL đến từ CORE 0, duty được ghi từ CORE 1 — lớp thứ ba

### Community 116 - "6. Telemetry an toàn/realtime — đọc gì khi soi log"
Cohesion: 0.33
Nodes (6): 6.1 An toàn / nhịp vòng lặp, 6.2 Chuỗi cất cánh (kiến trúc PID + slew, mục 4.2), 6.3 ToF + terrain (mục 4.4 / 4.6), 6.4 Ba cặp đáng soi cùng nhau, 6. Telemetry an toàn/realtime — đọc gì khi soi log, Terrain (chỉ có khi `TERRAIN_OFFSET_ENABLED=1`)

### Community 118 - "command_parser.c"
Cohesion: 0.13
Nodes (26): alt_hold_tune_t, flight_core_get_alt_hold_tune(), flight_core_get_setpoint(), flight_core_get_trim(), append(), move_dir_t, pid_gains_t, command_parser_feed_byte() (+18 more)

### Community 119 - "4.4 Altitude estimator — accel-primary, ToF CHỈ LÀ correction"
Cohesion: 0.40
Nodes (5): 4.4 Altitude estimator — accel-primary, ToF CHỈ LÀ correction, Ba trạng thái track (`update_age`), Bù nghiêng ToF — `range × cos(roll) × cos(pitch)`, Innovation gate và máy đo sàn — ĐÃ BỎ, `valid` vs `degraded` — hai câu hỏi khác nhau

### Community 120 - "app_config.h"
Cohesion: 0.14
Nodes (9): board_config_fill(), flight_core_board_config_t, esp_err_t, cpu_bench_init(), app_main(), register_commands(), fsm_state_t, telemetry_format_alt_mode() (+1 more)

### Community 121 - "4.7 FSM_FLYING — tắt tầng NGOÀI, giữ tầng TRONG"
Cohesion: 0.40
Nodes (5): 4.7 FSM_FLYING — tắt tầng NGOÀI, giữ tầng TRONG, Bàn giao FLYING → HOLDING là bumpless SẴN, Nguồn `vz` PHẢI là accel-only, Nạp I bumpless — tick ĐẦU, không phải mỗi tick, W/S trong FLYING = offset TẠM THỜI

### Community 122 - "camera_driver.c"
Cohesion: 0.13
Nodes (23): camera_fb_t, camera_selftest_t, camera_stats_t, esp_err_t, camera_acquire_frame(), camera_deinit(), camera_get_stats(), camera_init() (+15 more)

### Community 123 - "clampf"
Cohesion: 0.22
Nodes (15): alt_hold_result_t, clampf(), alt_hold_default_tune(), alt_hold_preload(), alt_hold_run(), alt_hold_vz_cascade(), alt_hold_state_t, alt_hold_tune_t (+7 more)

### Community 124 - "test_mjpeg_parser_offline.py"
Cohesion: 0.25
Nodes (6): fake_jpeg(), mjpeg_stream(), part_header(), MjpegParser: tach JPEG tu byte stream MJPEG. VI SAO CAN TEST NAY. Parser nay…, Mot 'anh' JPEG gia: SOI + than nhan dang duoc + EOI. Than KHONG duoc chua…, Dung dong giong het camera_stream.c phat ra.

### Community 127 - "STATUS line format and STATUS_RE parsing contract"
Cohesion: 0.13
Nodes (18): command_queue (FreeRTOS, depth 8), GUI layout and setpoint keepalive cadence, fc_module.c / fc_bridge.c are marshal-only, Why stabilize is priority 23 and not 24, src/main.c — second command entry point (console USB + UDP), stabilize_task — the entire flight control loop, ESP-IDF task stack is measured in BYTES (documentation correction), STATUS line format and STATUS_RE parsing contract (+10 more)

### Community 129 - "tof_driver.c"
Cohesion: 0.27
Nodes (16): add_device_at(), esp_err_t, i2c_master_bus_handle_t, i2c_master_dev_handle_t, tof_reading_t, change_i2c_address(), configure_xshut_gpio(), find_sensor_addr() (+8 more)

### Community 130 - "CameraStreamWorker"
Cohesion: 0.21
Nodes (4): CameraStreamWorker, (frame, seq, age_s). frame=None neu chua co. Goi tu GUI thread., (frame, ket_qua_processor, seq) — luon la CUNG MOT khung hinh. Diem noi cho…, Doc MJPEG tren thread rieng, giu DUY NHAT frame moi nhat. Khong dung Queue: GUI…

### Community 131 - "ColorDetector"
Cohesion: 0.22
Nodes (5): ColorDetector, Tra list[Detection], sap theo dien tich giam dan (to nhat truoc)., Ve khung + tam len BAN SAO — ve de len anh goc se doi mau cua no, va phan dieu…, Tim cac vet mau theo profile. Mot the hien dung lai duoc moi khung., Mat na nhi phan cho MOT profile; tach rieng de GUI xem duoc.

### Community 132 - "flight_core_tof_reinit"
Cohesion: 0.32
Nodes (8): tof_driver_chip_name(), flight_core_i2c_scan(), flight_core_tof_reinit(), sensor_hub_resume(), sensor_hub_set_tof_present(), sensor_hub_suspend(), cmd_i2c_scan(), cmd_tof_reinit()

### Community 133 - "MjpegParser"
Cohesion: 0.22
Nodes (4): MjpegParser, Client MJPEG cho UAV-S3 -- worker doc /stream tren thread RIENG. MjpegParser…, Bytes -> tung anh JPEG, theo marker SOI/EOI chu khong theo boundary. Khong…, Nap chunk, tra ve list cac JPEG hoan chinh (co the rong).

### Community 134 - "._update_tof_label"
Cohesion: 0.25
Nodes (4): Vi sao chip TU CHOI mau gan nhat. Tra chuoi ngan, hoac "" neu khong ro. Doc…, Ket luan ToF co dang chay khong. Tra (text, mau). HAI CAU HOI KHAC NHAU, truoc…, Tien to "Z<-NGUON" cho nhan do cao. Day la thong tin ma ADEGR KHONG noi duoc:…, Hien ToF DANG nhin be mat nao va co dang sua world-Z khong. Day la thong tin…

### Community 135 - "battery_driver_read"
Cohesion: 0.53
Nodes (5): battery_driver_init(), battery_driver_read(), battery_driver_read_v(), battery_sample_t, esp_err_t

### Community 136 - "sensor_hub_start"
Cohesion: 0.40
Nodes (5): esp_err_t, imu_calib_t, TaskHandle_t, sensor_hub_set_imu_calib(), sensor_hub_start()

### Community 137 - "flight_core_get_task_stats"
Cohesion: 0.50
Nodes (4): flight_core_get_task_stats(), sensor_hub_stack_free_bytes(), sensor_hub_stack_total_bytes(), flight_core_task_stats_t

## Knowledge Gaps
- **84 isolated node(s):** `(1) Timeout đếm bằng ĐỒNG HỒ, commit đếm bằng MẪU`, `(2) Timeout KHÔNG fail-safe`, `(3) Ứng viên offset phải CỘNG DỒN`, `⚠ `agl_m()` phải lấy từ SỐ ĐO THÔ, không phải `alt_m − terrain_off_m``, `Guard khoảng hở tối thiểu (B8) — lưới an toàn CUỐI` (+79 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **53 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `flight_core_start()` connect `flight_core_start` to `flight_core.c`, `tof_driver.c`, `fc_module.c`, `mahony_filter.c`, `battery_driver_read`, `sensor_hub_start`, `imu_driver.c`, `BMP280 Barometer Driver`, `mag_driver.c`, `calibration.c`, `stabilize_task`, `types.h`, `attitude_control_update`, `alt_estimator.c`, `commander_evaluate`, `takeoff_run`, `motor_driver.c`, `app_config.h`, `clampf`?**
  _High betweenness centrality (0.045) - this node is a cross-community bridge._
- **Why does `stabilize_task()` connect `stabilize_task` to `flight_core.c`, `flight_core_start`, `sensor_hub.c`, `types.h`, `attitude_control_update`, `mahony_filter.c`, `alt_estimator.c`, `hover_model.c`, `imu_driver.c`, `BMP280 Barometer Driver`, `commander_evaluate`, `takeoff_run`, `motor_driver.c`, `calibration.c`, `clampf`, `sensor_hub_age_us`?**
  _High betweenness centrality (0.044) - this node is a cross-community bridge._
- **Why does `flight_core_push_command()` connect `main.c` to `flight_core.c`, `fc_module.c`, `flight_core_read_telemetry`, `command_parser.c`, `net_link.c`?**
  _High betweenness centrality (0.031) - this node is a cross-community bridge._
- **Are the 50 inferred relationships involving `stabilize_task()` (e.g. with `alt_source_name()` and `attitude_state_reset()`) actually correct?**
  _`stabilize_task()` has 50 INFERRED edges - model-reasoned connections that need verification._
- **Are the 41 inferred relationships involving `apply_command()` (e.g. with `commander_clamp_altitude()` and `commander_heartbeat()`) actually correct?**
  _`apply_command()` has 41 INFERRED edges - model-reasoned connections that need verification._
- **Are the 42 inferred relationships involving `flight_core_push_command()` (e.g. with `fc_bridge_push()` and `handle_alt()`) actually correct?**
  _`flight_core_push_command()` has 42 INFERRED edges - model-reasoned connections that need verification._
- **What connects `(1) Timeout đếm bằng ĐỒNG HỒ, commit đếm bằng MẪU`, `(2) Timeout KHÔNG fail-safe`, `(3) Ứng viên offset phải CỘNG DỒN` to the rest of the system?**
  _84 weakly-connected nodes found - possible documentation gaps or missing edges._