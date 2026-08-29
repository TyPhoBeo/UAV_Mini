# Graph Report - UAV-S3  (2026-08-29)

## Corpus Check
- 92 files · ~220,060 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 1138 nodes · 2242 edges · 101 communities (54 shown, 47 thin omitted)
- Extraction: 85% EXTRACTED · 15% INFERRED · 0% AMBIGUOUS · INFERRED: 326 edges (avg confidence: 0.85)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `308e2980`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- MockFc
- sensor_hub.c
- fc_module.c
- mahony_filter.c
- test_flying_vz_hold_offline.py
- vl53l0x_driver.c
- commander.h
- ._log
- flight_core_push_command
- test_tof_bringup_offline.py
- stabilize_task
- UAV-S3 — Flight controller 2 tầng (C + MicroPython) cho ESP32-S3-WROOM-1 (miniUav)
- vl53l1x_driver.c
- command_parser.c
- apply_command
- baro_driver.c
- mag_driver.c
- net_link.c
- test_ws_throttle_offline.py
- takeoff_run
- ._build_ui
- attitude_control_update
- flight_core_read_telemetry
- calibration.c
- flight_core.c
- motor_driver.c
- ._handle_line
- types.h
- Core
- imu_driver.c
- fc_api.py
- FcTimeoutError
- test_hover_model_offline.py
- PidTunerApp
- ._ws_press
- 3. Phân chia task RTOS
- hover_model.c
- GainRow
- PlotPanel
- UavUdpConsole
- Heartbeat
- FlightAltGroup
- GainGroup
- CommanderGroup
- LandingGroup
- TakeoffGroup
- test_tof_fault_tolerance_offline.py
- app_config.h
- FlightCommandGroup
- uav_udp_console.py
- CalibrationGroup
- TestMotorGroup
- 4.2 Chuỗi TAKEOFF — PID + slew-rate-limited target
- test_alt_estimator_offline.py
- test_alt_tof_surface_offline.py
- test_status_parse_offline.py
- test_takeoff_flow_offline.py
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
- test_alt_validity_offline.py
- test_arm_gate_offline.py
- test_bias_residual_offline.py
- test_stall_credit_offline.py
- test_takeoff_gate_offline.py
- UAV-S3 — Luồng code end-to-end
- 4. FlightStateMachine (topology)
- 3.7 Vòng lặp `stabilize_task` (250Hz, core 1, `flight_core.c`)
- 7. Ground station — console USB + UDP (lối vào thứ hai)
- test_gyro_calibration_offline.py
- AGENTS.md
- CLAUDE.md
- commander_evaluate
- test_flight_mode_sequence_offline.py
- test_flying_ws_no_accumulate_offline.py
- flight_core_start
- test_terrain_deadlock_offline.py
- test_tilt_comp_expo_offline.py
- test_tof_alive_vs_valid_offline.py
- ._update_tof_label
- imu_driver_get_config
- test_gui_tof_health_offline.py
- sensor_hub_age_us

## God Nodes (most connected - your core abstractions)
1. `PidTunerApp` - 101 edges
2. `stabilize_task()` - 63 edges
3. `apply_command()` - 56 edges
4. `flight_core_push_command()` - 45 edges
5. `MockFc` - 44 edges
6. `flight_core_start()` - 31 edges
7. `fc_bridge_push()` - 30 edges
8. `raise_if_queue_full()` - 27 edges
9. `UAV-S3 — Flight controller 2 tầng (C + MicroPython) cho ESP32-S3-WROOM-1 (miniUav)` - 25 edges
10. `flight_core_read_telemetry()` - 20 edges

## Surprising Connections (you probably didn't know these)
- `handle_trim()` --calls--> `flight_core_get_trim()`  [INFERRED]
  src/command_parser.c → components/flight_core/src/flight_core.c
- `handle_sp()` --calls--> `flight_core_get_setpoint()`  [INFERRED]
  src/command_parser.c → components/flight_core/src/flight_core.c
- `handle_single_char()` --calls--> `fsm_arm_guard_ok()`  [INFERRED]
  src/command_parser.c → components/flight_core/include/flight_core/flight_state_machine.h
- `fc_get_state()` --calls--> `fsm_state_name()`  [INFERRED]
  micropython_module/fc/fc_module.c → components/flight_core/include/flight_core/flight_state_machine.h
- `handle_alt()` --calls--> `fsm_state_name()`  [INFERRED]
  src/command_parser.c → components/flight_core/include/flight_core/flight_state_machine.h

## Import Cycles
- None detected.

## Communities (101 total, 47 thin omitted)

### Community 0 - "MockFc"
Cohesion: 0.07
Nodes (21): FakeClock, fresh_import(), MockFc, patch_clock(), Test offline (CPython, KHÔNG cần MicroPython/phần cứng) cho logic BLOCKING của…, Đồng hồ giả: sleep() nhảy thời gian ngay lập tức thay vì chờ thật — test chạy…, Cài mock vào sys.modules['fc'] rồi (re)import fc_api sạch., test_calibrate_accel_6face_happy_path() (+13 more)

### Community 1 - "sensor_hub.c"
Cohesion: 0.07
Nodes (52): battery_driver_init(), battery_driver_read(), battery_driver_read_v(), battery_sample_t, esp_err_t, add_device_at(), esp_err_t, i2c_master_bus_handle_t (+44 more)

### Community 2 - "fc_module.c"
Cohesion: 0.12
Nodes (45): telemetry_snapshot_t, telemetry_snapshot_init(), command_t, move_dir_t, telemetry_snapshot_t, fc_bridge_get_control_input(), fc_bridge_init(), fc_bridge_parse_move_dir() (+37 more)

### Community 3 - "mahony_filter.c"
Cohesion: 0.18
Nodes (36): vec3f_t, vec3f_zero(), mahony_config_t, flight_core_get_mahony_config(), apply_feedback_and_integrate(), mahony_config_t, mahony_t, vec3f_t (+28 more)

### Community 5 - "vl53l0x_driver.c"
Cohesion: 0.20
Nodes (30): esp_err_t, i2c_master_bus_handle_t, i2c_master_dev_handle_t, tof_sensor_state_t, calc_macro_period_ns(), decode_timeout(), decode_vcsel_period(), encode_timeout() (+22 more)

### Community 6 - "commander.h"
Cohesion: 0.38
Nodes (6): commander_clamp_altitude(), commander_credit_stall(), commander_heartbeat(), commander_init(), commander_config_t, commander_state_t

### Community 8 - "flight_core_push_command"
Cohesion: 0.13
Nodes (25): command_t, flight_core_push_command(), move_dir_t, cmd_alt(), cmd_arm(), cmd_bench_start(), cmd_bench_stop(), cmd_bench_throttle_down() (+17 more)

### Community 9 - "test_tof_bringup_offline.py"
Cohesion: 0.08
Nodes (12): Bus, Chip, find_sensor_addr(), find_sensor_addr_OLD(), object, VL53L0X mo phong. addr: dia chi HIEN TAI, nam trong RAM chip. xshut_wired: day…, Cach lai SAI. Ngoai spec 2.8V -> chip khong con tin duoc., Ban chep tof_driver_init() single-sensor. Tra (err, final_addr, log). (+4 more)

### Community 10 - "stabilize_task"
Cohesion: 0.19
Nodes (25): alt_estimator_t, clampf(), alt_estimator_agl_m(), alt_estimator_confirm_liftoff(), alt_estimator_correct_velocity(), alt_estimator_floor_ready(), alt_estimator_height_above_landing_surface(), alt_estimator_lock_floor() (+17 more)

### Community 11 - "UAV-S3 — Flight controller 2 tầng (C + MicroPython) cho ESP32-S3-WROOM-1 (miniUav)"
Cohesion: 0.06
Nodes (31): Altitude estimator (accel-primary + ToF/baro anchor), Bay qua console USB (firmware chính, không cần MicroPython), Bay thử từ REPL, Bench-test tăng ga tay để tune PID (`bench_start`/`+`/`-`/`bench_stop`), Build MicroPython port thật (CHƯA làm — bước tiếp theo của bạn), Build/nạp trực tiếp bằng PlatformIO — firmware CHÍNH, KHÔNG cần MicroPython, Calibration (gyro/accel/mag) — persist qua NVS, Commander (`commander.h`) (+23 more)

### Community 12 - "vl53l1x_driver.c"
Cohesion: 0.30
Nodes (23): esp_err_t, i2c_master_bus_handle_t, i2c_master_dev_handle_t, tof_sensor_state_t, l1x_check_ready(), l1x_clear_interrupt(), l1x_read(), l1x_read16() (+15 more)

### Community 13 - "command_parser.c"
Cohesion: 0.16
Nodes (23): alt_hold_tune_t, flight_core_get_alt_hold_tune(), append(), move_dir_t, pid_gains_t, command_parser_feed_byte(), dump_pid_line(), handle_alt() (+15 more)

### Community 14 - "apply_command"
Cohesion: 0.20
Nodes (20): fsm_transition(), abort_landing_to_hold(), apply_command(), finalize_mag_calibration(), recompute_uncalibrated(), fsm_state_t, fsm_on_arm_request(), fsm_on_bench_ramp_start() (+12 more)

### Community 15 - "baro_driver.c"
Cohesion: 0.22
Nodes (18): bmp280_calib_t, baro_driver_calibrate_ground(), baro_driver_ground_healthy(), baro_driver_ground_noise_std_pa(), baro_driver_ground_ready(), baro_driver_init(), baro_driver_read(), baro_sample_t (+10 more)

### Community 16 - "mag_driver.c"
Cohesion: 0.32
Nodes (18): esp_err_t, i2c_master_bus_handle_t, mag_sample_t, cleanup_device(), invalidate_sample(), mag_driver_deinit(), mag_driver_get_latest(), mag_driver_init() (+10 more)

### Community 17 - "net_link.c"
Cohesion: 0.15
Nodes (15): esp_event_base_t, command_parser_telemetry_enabled(), cmd_tasks(), net_task(), print_stack_row(), esp_err_t, init_nvs_flash_safe(), init_wifi_sta() (+7 more)

### Community 18 - "test_ws_throttle_offline.py"
Cohesion: 0.15
Nodes (8): Bench, clampf(), clampi(), Fly, Test phim GIU W/S -> throttle offset momentary. Yeu cau: GIU W -> throttle…, TRANSCRIPTION nhanh FSM_HOLDING/FSM_FLYING sau khi W/S doi sang Vz., alt_hold_vz_cascade() rut gon: PI tren sai so vz, co clamp I., s_bench_throttle_duty + s_bench_throttle_offset (flight_core.c).

### Community 19 - "takeoff_run"
Cohesion: 0.12
Nodes (29): alt_hold_result_t, alt_hold_default_tune(), alt_hold_preload(), alt_hold_run(), alt_hold_vz_cascade(), alt_hold_state_t, alt_hold_tune_t, landing_tune_t (+21 more)

### Community 21 - "attitude_control_update"
Cohesion: 0.15
Nodes (15): attitude_input_t, attitude_output_t, attitude_control_update(), attitude_default_gains(), axis_hold(), attitude_gains_t, attitude_state_t, wrap_deg_180() (+7 more)

### Community 22 - "flight_core_read_telemetry"
Cohesion: 0.15
Nodes (16): fsm_arm_guard_ok(), fsm_state_name(), fsm_state_t, tof_driver_stall_restarts(), telemetry_snapshot_t, flight_core_read_telemetry(), handle_land(), handle_single_char() (+8 more)

### Community 23 - "calibration.c"
Cohesion: 0.33
Nodes (15): calibration_params_t, esp_err_t, vec3f_t, calibration_erase_all(), calibration_load(), calibration_nvs_init(), calibration_save_accel(), calibration_save_gyro() (+7 more)

### Community 24 - "flight_core.c"
Cohesion: 0.13
Nodes (29): apply_set_param(), imu_sample_t, TickType_t, vec3f_t, enter_kill_latch(), enter_kill_latch_ex(), flight_core_get_control_input(), flight_core_get_setpoint() (+21 more)

### Community 25 - "motor_driver.c"
Cohesion: 0.25
Nodes (12): clampi(), esp_err_t, configure_channel(), motor_driver_all_off(), motor_driver_arm(), motor_driver_disarm(), motor_driver_init(), motor_driver_set_armed() (+4 more)

### Community 26 - "._handle_line"
Cohesion: 0.15
Nodes (5): Hiện VÌ SAO ARM bị từ chối, ngay trên thanh nút. Không có cái này thì GUI hoàn…, Hiện VÌ SAO chuỗi cất cánh không khởi động. Dùng CHUNG nhãn với pha cất cánh…, Nhãn pha cất cánh (closed-loop) cạnh nút TAKEOFF. Hiển thị theo pha THẬT của…, Khoa/mo nut TRIM. allow=False khi drone DANG BAY. Chan o GUI la lop DAU, khong…, Nhan nut W/S + dong goi y phai khop MODE HIEN TAI. Goi moi khi STATUS doi mode.…

### Community 27 - "types.h"
Cohesion: 0.10
Nodes (10): alt_source_t, alt_source_name(), attitude_state_reset(), attitude_state_t, mahony_t, quat_t, mahony_quaternion(), pid_state_t (+2 more)

### Community 28 - "Core"
Cohesion: 0.18
Nodes (7): attitude_target_OLD(), boot_load_WRONG(), clampf(), Core, object, Ban CU -- de chung minh no that su danh roi trim (test co y nghia)., Neu ai do quen hoan truc luc nap -- phai thay hau qua ro rang.

### Community 29 - "imu_driver.c"
Cohesion: 0.15
Nodes (19): esp_err_t, i2c_master_bus_handle_t, imu_calib_t, imu_sample_t, TaskHandle_t, vec3f_t, imu_driver_enable_data_ready_int(), imu_driver_init() (+11 more)

### Community 32 - "FcTimeoutError"
Cohesion: 0.18
Nodes (12): Exception, calibrate_accel_6face(), calibrate_gyro(), FcFaultError, FcTimeoutError, land(), Blocking: đo lại gyro bias tĩnh (~1.5s, DỪNG YÊN drone). Raise FcTimeoutError…, Blocking: chạy đủ 6 mặt. `prompt(face_idx)` (idx 0..5) nếu truyền vào sẽ được… (+4 more)

### Community 33 - "test_hover_model_offline.py"
Cohesion: 0.20
Nodes (4): dint(), dnum(), Model ga hover theo dien ap pin + latch mot lan luc ARM. TRANSCRIPTION cua…, Ring

### Community 36 - "._ws_press"
Cohesion: 0.25
Nodes (3): Mode alt gan nhat tu STATUS ('2'=HOLD, '5'=FLYING, ...) hoac None., Bat dau giu W/S. Hanh vi phu thuoc state — xem khoi comment tren., Nha phim (hoac watchdog ket phim) -> BAT BUOC tra offset ve 0.

### Community 37 - "3. Phân chia task RTOS"
Cohesion: 0.17
Nodes (12): 3.1 Bảng task — nguồn sự thật, 3.2 Vì sao chia như vậy, 3.2b Tốc độ I2C — một nguồn sự thật, 3.3 Ba kênh giao tiếp giữa task — và chiều dữ liệu, 3.4 Nhịp: một đồng hồ vật lý, hai tầng dự phòng, 3.4b Tick KHÔNG có mẫu IMU mới — cái gì chạy, cái gì không, 3.5 Nhịp con của sensor_hub — vì sao xen kẽ, không gộp, 3.5b Timeout I2C — HAI giá trị, hai mục đích (+4 more)

### Community 38 - "hover_model.c"
Cohesion: 0.36
Nodes (7): clampf_local(), hover_model_from_voltage(), hover_model_prime_duty(), hover_vbat_median(), hover_vbat_push(), hover_vbat_reset(), hover_vbat_ring_t

### Community 40 - "PlotPanel"
Cohesion: 0.29
Nodes (3): PlotPanel, Scrolling multi-line strip chart drawn on a Tk Canvas. Mimics the Arduino IDE…, Append one sample. `values` maps channel key -> float.

### Community 48 - "test_tof_fault_tolerance_offline.py"
Cohesion: 0.40
Nodes (3): Dung sai ToF: MOT mau xau KHONG duoc huy mot lan cat canh. BOI CANH (loi that,…, Comment CO QUYEN mo ta lai loi cu; chi CODE moi phai dung., strip_comments()

### Community 49 - "app_config.h"
Cohesion: 0.17
Nodes (7): board_config_fill(), flight_core_board_config_t, app_main(), register_commands(), fsm_state_t, telemetry_format_alt_mode(), telemetry_format_status_line()

### Community 50 - "FlightCommandGroup"
Cohesion: 0.50
Nodes (3): StringVar, FlightCommandGroup, Timed-command 1 phát (@HOVER/@MOVE/@YAW — xem command_parser.c…

### Community 51 - "uav_udp_console.py"
Cohesion: 0.70
Nodes (4): main(), run_cli(), run_gui(), translate_command()

### Community 54 - "4.2 Chuỗi TAKEOFF — PID + slew-rate-limited target"
Cohesion: 0.18
Nodes (11): 4.2 Chuỗi TAKEOFF — PID + slew-rate-limited target, 4.2b `hover_ff` chốt theo điện áp pin — `hover_model.h`, ABORT — mọi lý do đều dẫn về EMERGENCY, không tự chọn policy, Bàn giao CLIMB → HOLD: LIỀN MẠCH, Cạnh PRIME → CLIMB: `I` khởi đầu = 0, KHÔNG preload, Hai điểm review đã khoá thành test hồi quy (R1/R2), `|I|` trước liftoff: GIỚI HẠN, KHÔNG ĐÓNG BĂNG, `liftoff_flag` — CHỈ LÀ THÔNG TIN, không gate gì (+3 more)

### Community 77 - "test_alt_validity_offline.py"
Cohesion: 0.33
Nodes (3): Do cao con dung duoc khong: CHI hoi "chip con do khong". TRUOC DAY day la mot…, Bo comment truoc khi kiem 'code co ton tai khong'. Khong co buoc nay thi mot…, strip_comments()

### Community 83 - "UAV-S3 — Luồng code end-to-end"
Cohesion: 0.20
Nodes (9): 1. Kiến trúc tổng quan, 2. Luồng một lệnh Python (ví dụ `fc.takeoff(800)`), 5. Tổng kết file/module theo lớp, 6.1 An toàn / nhịp vòng lặp, 6.2 Chuỗi cất cánh (kiến trúc PID + slew, mục 4.2), 6.3 ToF surface-gated (mục 4.4), 6.4 Ba cặp đáng soi cùng nhau, 6. Telemetry an toàn/realtime — đọc gì khi soi log (+1 more)

### Community 84 - "4. FlightStateMachine (topology)"
Cohesion: 0.22
Nodes (9): 4.1 Commander chạy ở MỌI state đã armed, 4.1b ARM — cổng vào duy nhất, và vì sao nó không còn im lặng, 4.3 Chuỗi LANDING — touchdown đa điều kiện, 4.4 Altitude estimator — accel-primary, ToF/baro CHỈ LÀ correction, 4. FlightStateMachine (topology), Heartbeat watchdog — bằng chứng phải đến TỪ BÊN NGOÀI, ToF ground-ref — vì sao chốt ở ARM, ToF surface-gated correction — vì sao cần, và bẫy đã tránh (+1 more)

### Community 85 - "3.7 Vòng lặp `stabilize_task` (250Hz, core 1, `flight_core.c`)"
Cohesion: 0.33
Nodes (6): 3.7.1 Hai task, một chiều dữ liệu, 3.7.2 Một tick của stabilize_task, 3.7.3 Kill path — quyền ưu tiên cao nhất, 3.7.4 Bước 9/9c — khi nào I-term (Ki) được cộng dồn, 3.7 Vòng lặp `stabilize_task` (250Hz, core 1, `flight_core.c`), KILL đến từ CORE 0, duty được ghi từ CORE 1 — lớp thứ ba

### Community 86 - "7. Ground station — console USB + UDP (lối vào thứ hai)"
Cohesion: 0.40
Nodes (5): 7.1 Giao thức UDP — hai chế độ ký tự, 7.2 Ba chẩn đoán phần cứng — xem mục 3.6b, 7.3 GUI (`tools/uav_udp_console.py`), 7.4 Trim roll/pitch — lưu NVS, 7. Ground station — console USB + UDP (lối vào thứ hai)

### Community 90 - "commander_evaluate"
Cohesion: 0.25
Nodes (8): commander_inputs_t, commander_result_t, commander_config_t, commander_state_t, commander_default_config(), commander_evaluate(), commander_config_t, flight_core_get_commander_cfg()

### Community 91 - "test_flight_mode_sequence_offline.py"
Cohesion: 0.25
Nodes (5): fsm_rule(), Chuoi FSM tu ARM -> TAKEOFF -> HOLDING -> FLYING -> LANDING, va ma MODE= ma GUI…, Bo comment truoc khi kiem 'code co dung X khong'. Khoi giai thich o…, Tra ve list (dieu_kien_state, state_dich) cua mot ham fsm_on_*., strip_c_comments()

### Community 92 - "test_flying_ws_no_accumulate_offline.py"
Cohesion: 0.22
Nodes (7): W/S trong FLYING = offset TAM THOI, KHONG duoc cong don. VI SAO CAN TEST NAY --…, Cach DUNG: latch la ga nen, offset chi anh huong tick nay., Cach SAI (ban cu): cong don vao latch., Bo comment truoc khi kiem 'code co lam X khong'. Khoi giai thich o…, strip_c_comments(), tick_dung(), tick_sai()

### Community 93 - "flight_core_start"
Cohesion: 0.17
Nodes (17): alt_hold_reset(), alt_hold_state_t, fsm_init(), landing_state_t, takeoff_state_t, landing_reset(), takeoff_airborne(), takeoff_begin() (+9 more)

### Community 96 - "test_tof_alive_vs_valid_offline.py"
Cohesion: 0.29
Nodes (5): decide(), Phan biet "KHONG CO GI DE DO" voi "MAT CAM BIEN". VI SAO CAN TEST NAY: Dat…, Mo phong dung logic update_age() da sua., Bo comment truoc khi kiem tra 'code co ton tai khong'. Khong co buoc nay thi…, strip_comments()

### Community 97 - "._update_tof_label"
Cohesion: 0.33
Nodes (3): Ket luan ToF co dang chay khong. Tra (text, mau). HAI CAU HOI KHAC NHAU, truoc…, Tien to "Z<-NGUON" cho nhan do cao. Day la thong tin ma ADEGR KHONG noi duoc:…, Hien ToF DANG nhin be mat nao va co dang sua world-Z khong. Day la thong tin…

### Community 98 - "imu_driver_get_config"
Cohesion: 0.40
Nodes (5): imu_driver_get_config(), gyro_cal_state_active(), prearm_check(), gyro_cal_state_t, imu_cfg_readback_t

### Community 99 - "test_gui_tof_health_offline.py"
Cohesion: 0.40
Nodes (3): Fake, GUI phai phan biet "khong co gi de do" voi "mat cam bien". VI SAO CAN TEST NAY:…, Chi can TOF_STALE_MS + method that -- khong dung tkinter.

## Knowledge Gaps
- **68 isolated node(s):** `graphify`, `graphify`, `Trạng thái hiện tại (đọc mục này trước)`, `Kiến trúc: ĐÚNG 2 tầng`, `Vì sao C (không phải C++) cho tầng dưới` (+63 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **47 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `flight_core_start()` connect `flight_core_start` to `sensor_hub.c`, `fc_module.c`, `mahony_filter.c`, `commander.h`, `stabilize_task`, `apply_command`, `baro_driver.c`, `mag_driver.c`, `app_config.h`, `takeoff_run`, `attitude_control_update`, `calibration.c`, `flight_core.c`, `motor_driver.c`, `commander_evaluate`, `types.h`, `imu_driver.c`?**
  _High betweenness centrality (0.069) - this node is a cross-community bridge._
- **Why does `stabilize_task()` connect `stabilize_task` to `sensor_hub.c`, `imu_driver_get_config`, `mahony_filter.c`, `sensor_hub_age_us`, `commander.h`, `hover_model.c`, `apply_command`, `baro_driver.c`, `imu_driver.c`, `takeoff_run`, `attitude_control_update`, `calibration.c`, `flight_core.c`, `motor_driver.c`, `commander_evaluate`, `types.h`, `flight_core_start`?**
  _High betweenness centrality (0.067) - this node is a cross-community bridge._
- **Why does `apply_command()` connect `apply_command` to `sensor_hub.c`, `imu_driver_get_config`, `mahony_filter.c`, `commander.h`, `hover_model.c`, `flight_core_push_command`, `stabilize_task`, `baro_driver.c`, `mag_driver.c`, `flight_core_read_telemetry`, `calibration.c`, `flight_core.c`, `motor_driver.c`, `types.h`, `flight_core_start`?**
  _High betweenness centrality (0.046) - this node is a cross-community bridge._
- **Are the 49 inferred relationships involving `stabilize_task()` (e.g. with `alt_source_name()` and `attitude_state_reset()`) actually correct?**
  _`stabilize_task()` has 49 INFERRED edges - model-reasoned connections that need verification._
- **Are the 41 inferred relationships involving `apply_command()` (e.g. with `commander_clamp_altitude()` and `commander_heartbeat()`) actually correct?**
  _`apply_command()` has 41 INFERRED edges - model-reasoned connections that need verification._
- **Are the 42 inferred relationships involving `flight_core_push_command()` (e.g. with `fc_bridge_push()` and `handle_alt()`) actually correct?**
  _`flight_core_push_command()` has 42 INFERRED edges - model-reasoned connections that need verification._
- **What connects `graphify`, `graphify`, `Trạng thái hiện tại (đọc mục này trước)` to the rest of the system?**
  _68 weakly-connected nodes found - possible documentation gaps or missing edges._