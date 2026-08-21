#include "telemetry_format.h"

#include <stdio.h>

#include "flight_core/flight_core.h"

// alt_mode_from_state() — UAV-S3 KHÔNG có "alt_mode" độc lập như UAV-Mini
// (alt_hold LUÔN chạy khi HOLDING/FLYING, do FSM quyết định — không phải 1
// cờ riêng có thể OFF trong lúc bay). Hàm này CHỈ derive 1 số hiển thị tương
// đương cho GUI cũ (0=OFF 1=LOG_ONLY 2=HOLD 3=TAKEOFF 4=LANDING) từ FSM state
// THẬT — xem command_parser.c mục @ALT MODE để biết @ALT MODE SET ánh xạ ra
// sao (KHÔNG đối xứng hoàn toàn, ghi rõ ở đó).
int telemetry_format_alt_mode(fsm_state_t s) {
    switch (s) {
        case FSM_TAKING_OFF: return 3;
        case FSM_HOLDING:
        case FSM_FLYING:     return 2;
        case FSM_LANDING:    return 4;
        default:              return 0;   // DISARMED/ARMED/EMERGENCY
    }
}

void telemetry_format_status_line(char *out, size_t out_size) {
    telemetry_snapshot_t t;
    flight_core_read_telemetry(&t);

    const int tko_field = (t.state == FSM_TAKING_OFF) ? (t.takeoff_prime_done ? 2 : 1) : 0;

    // ---- Format LÕI khớp NGUYÊN VĂN UAV-Mini flight_pipeline_status_line()
    // (xem STATUS_RE trong tools/uav_udp_console.py) — KHÔNG đổi thứ tự/tên
    // field ở đoạn này dù có vẻ dư (vd "M = " có khoảng trắng thừa) để giữ
    // tương thích GUI cũ 100%. Field UAV-S3-specific NỐI THÊM ở cuối.
    snprintf(
        out, out_size,
        "ARM=%d THR=%d | R=%.2f P=%.2f Y=%.2f | G=%.2f %.2f %.2f | A=%.3f %.3f %.3f | "
        "M = %d %d %d %d | Val=%d | ACC=%.3f ACCU=%d | YAWREL=%.2f "
        "| ALTm=%.2f VZ=%.2f TGT=%.2f MODE=%d AV=%d TOF=%.2f TOK=%d TERR=%u TKO=%d KI=%d LAND=%d"
        " MAGOK=%d BAROOK=%d IMUOK=%d BALT=%.2f DT=%.2f CMDAGE=%d HBAGE=%d FAULT=%d"
        " BATV=%.2f BCOMP=%.3f"
        " BFILT=%.2f BINNOV=%.2f BACC=%u BREJ=%u BCAL=%d BHLT=%d BSTD=%.2f VACC=%.2f"
        " BDT=%.3f VACCR=%.3f ABIAS=%.3f VZAO=%.3f ZINE=%.2f VZTGT=%.2f"
        // Đuôi refactor an toàn/realtime (xem telemetry.h):
        // KILL=kill latch; TKOP=pha cất cánh (0=IDLE 1=PRIME 2=CLIMB 3=HOLD);
        // LSC=LUÔN 0 — bộ dò rời đất đã bỏ, giữ field cho GUI cũ khỏi vỡ regex;
        // LDT/LMX=dt vòng điều khiển hiện tại/đỉnh (us); DLM=số tick trượt hạn;
        // IAGE/MAGE/BAGE=tuổi mẫu IMU/mag/baro (ms, -1=chưa có);
        // HDEG=heading_degraded; MSAT=mixer bão hoà; MRPY=trục bị chặn;
        // MHR=headroom duty còn lại; BTHR=collective TRƯỚC bù pin.
        " KILL=%d TKOP=%d LSC=%d LDT=%d LMX=%d DLM=%u"
        " IAGE=%d MAGE=%d BAGE=%d HDEG=%d"
        " MSAT=%d MRPY=%d%d%d MHR=%.0f BTHR=%d"
        // Đuôi estimator GROUND/CANDIDATE/AIRBORNE + reacquire (alt_estimator.h):
        // AIRB=đã confirmed rời đất; CAND=pha LIFTOFF_CANDIDATE (inertial được
        // chạy để TẠO bằng chứng liftoff — AIRB=0 CAND=0 nghĩa là Z/Vz đang bị
        // khoá 0 có chủ đích); AZCORR=az sau trừ bias + deadband (m/s², giá trị
        // THẬT được tích phân); BCREJ=reject baro LIÊN TỤC; BREACQ=đang
        // REACQUIRE; BSEQ=seq mẫu baro (phải tăng ~50/s); BFI=baro fusion đã
        // khởi tạo gốc toạ độ chưa.
        " AIRB=%d CAND=%d AZCORR=%.3f BCREJ=%u BREACQ=%d BSEQ=%u BFI=%d"
        // ALTSRC = NGUON dang thuc su giu Z: 0=NONE 1=TOF 2=BARO 3=TOF+BARO.
        // KHAC ADEGR: degraded chi noi "co correction hay khong", ALTSRC noi
        // correction do CUA AI. ALTSRC=2 khi ban tuong dang bay bang ToF nghia la
        // ToF da ngung dong gop -> do cao so voi san dang troi theo baro.
        " ALTSRC=%u"
        // Battery debug (battery_driver.h) — BATRAW/BATMV/BATRATIO/BATVRAW cho
        // phép truy ngược thang đo: sai ở ADC, ở calibration, hay ở chia áp.
        // BATVALID=0 nghĩa là BATV ở đầu dòng đã bị ép 0 và KHÔNG dùng cho
        // failsafe/compensation. BATCALI=0 nghĩa là ADC calibration không có.
        " BATRAW=%d BATMV=%d BATRATIO=%.2f BATVRAW=%.3f BATVALID=%d BATCALI=%d BATAGE=%d"
        // ---- Đuôi TAKEOFF (PID + slew-rate-limited target, takeoff_land.h) ----
        // TKOACT = takeoff_control_active (cascade đang cầm lái — KHÁC AIRB).
        // TKOTGT = final_target TỪ LỆNH.
        // ZSP    = target_z ĐANG TRƯỢT. Đây là thứ chính để soi slew: nó phải
        //          BÒ ĐỀU từ TKOGND lên TKOTGT, KHÔNG được nhảy bậc. Nhảy bậc
        //          = mất chống windup (xem takeoff_land.h điểm 1).
        // VZTGT  = output tầng Z -> đầu vào Vz-PID.
        // TKOBASE= hover_ff (feedforward THÔ, hằng số).
        // TKOCORR= throttle - hover_ff (tổng P+I của vòng Vz).
        // TKOI   = RIÊNG phần I. Đây là hover THẬT mà vòng Vz tự học: khởi đầu
        //          ÂM (prime_duty < hover_ff) rồi bò lên và HỘI TỤ. Không hội tụ
        //          / bám trần = hover_ff sai quá xa hoặc drone không đủ lực.
        // TKOGND = mốc est_z lúc rời PRIME; TKOLIFT = đã rời đất (THÔNG TIN).
        // TKOTILT/TKOEL = tilt + thời gian đã trôi của chuỗi.
        // TKOAB  = lý do abort (0=NONE 1=TIMEOUT 2=TOF_LOST 3=TILT).
        // ALTSAT = cascade Vz bị kẹp trần/sàn duty.
        " TKOACT=%d TKOTGT=%.2f ZSP=%.2f VZTGT=%.2f TKOBASE=%.0f TKOCORR=%.0f"
        " TKOI=%.0f TKOGND=%.2f TKOLIFT=%d TKOTILT=%.1f TKOEL=%.1f TKOAB=%d ALTSAT=%d"
        // ARMREJ = vì sao lệnh ARM gần nhất bị từ chối (arm_reject_t trong
        // telemetry.h), ARMRSEQ = đếm số lần bị từ chối. BẮT BUỘC phải có trên
        // wire: mọi ESP_LOGW của prearm_check() CHỈ ra console USB, GUI qua UDP
        // không thấy gì — trước đây bấm ARM mà bị từ chối là im lặng hoàn toàn.
        " ARMREJ=%d ARMRSEQ=%u"
        // TKOREJ/TKORSEQ = y het the, cho lenh TAKEOFF (takeoff_reject_t).
        // Can rieng vi takeoff co bo dieu kien RIENG, khong trung ARM: co the
        // ARM thanh cong roi TAKEOFF bi tu choi (vd baro TAT va ToF hong ->
        // khong con nguon correction nao). KHAC TKOAB= la "da cat canh roi
        // moi phai huy"; TKOREJ= la "chua tung bat dau".
        " TKOREJ=%d TKORSEQ=%u"
        // ---- ToF surface-gated correction (alt_estimator.h) ----
        // TOFV=range da bu nghieng; TOFINN=innovation so voi SAN DA KHOA (am
        // lon = co be mat CAO hon san o duoi); TOFSURF=world-Z be mat dang nhin;
        // TOFST=0 UNKNOWN/1 FLOOR/2 OTHER; TOFCOR=ToF co DANG sua world-Z khong;
        // TOFGR=range luc UAV nam san (chot o ARM); TOFA/TOFR=accept/reject;
        // FLOORZ=mat san khoa luc cat canh; LANDZ/LANDV=be mat ha canh.
        // TOFST=2 voi TOFCOR=0 la HOP LE khi bay qua ban -- KHONG phai loi.
        " TOFV=%.3f TOFINN=%.3f TOFSURF=%.3f TOFST=%d TOFCOR=%d TOFGR=%.3f"
        " TOFA=%u TOFR=%u FLOORZ=%.2f LANDZ=%.2f LANDV=%d TOFAGE=%d"
        // TOFEN = ToF CO trong firmware nay khong. Dat o CUOI dong co chu dich:
        // them field vao GIUA se lam DICH moi group index cua STATUS_RE ben GUI
        // (viec do da mot lan suyt lam GUI hong IM LANG). Them o cuoi thi khong dich gi.
        " TOFEN=%d"
        // Luong sua THUC TE + hoc bias tu residual. Them o CUOI dong
        // (khong chen vao giua) de khong dich group index cua STATUS_RE.
        " TOFCORRZ=%.4f TOFCORRVZ=%.4f BAROCORRZ=%.4f BAROCORRVZ=%.4f"
        " BIASRES=%.4f BIASADP=%u"
        // Latch ga hover theo pin (hover_model.h) — chot LUC ARM, dong bang ca
        // chuyen bay. CUNG LY DO nhu TOFEN/BIASRES o tren: them o CUOI dong,
        // KHONG chen vao giua, de khong dich group index cua STATUS_RE ben GUI.
        //   HOVLK  = da chot lan nao chua (0 = dang chay hang so cu)
        //   HOVLV  = vbat trung vi luc chot (V, KHONG TAI)
        //   HOVLD  = hover_ff suy ra tu model (duty)
        " HOVLK=%d HOVLV=%.2f HOVLD=%.0f\n",
        t.armed ? 1 : 0,
        t.throttle_duty,
        t.roll_deg, t.pitch_deg, t.yaw_deg,
        t.gyro_roll_dps, t.gyro_pitch_dps, t.gyro_yaw_dps,
        t.accel_x_g, t.accel_y_g, t.accel_z_g,
        t.m1, t.m2, t.m3, t.m4,
        t.attitude_valid ? 1 : 0,
        t.acc_norm_g, t.accel_used ? 1 : 0,
        t.yaw_rel_deg,
        t.alt_m, t.vz_ms, t.alt_target_m, telemetry_format_alt_mode(t.state),
        t.alt_valid ? 1 : 0, t.tof_range_m, t.tof_valid ? 1 : 0,
        // KI= báo I-term attitude CÓ ĐANG cộng dồn không (att_integral_active)
        // — KHÔNG phải takeoff_prime_done như trước (prime_done thuần thời
        // gian, không phản ánh gate airborne + gate ga, xem telemetry.h).
        (unsigned)t.sensor_err_count, tko_field, t.att_integral_active ? 1 : 0, t.landing_phase,
        t.mag_ok_driver ? 1 : 0, t.baro_ok_driver ? 1 : 0, t.imu_ok_driver ? 1 : 0,
        t.baro_alt_m, t.loop_dt_ms, (int)t.last_cmd_age_ms, (int)t.heartbeat_age_ms, (int)t.last_fault,
        t.battery_v, t.battery_comp,
        t.baro_filtered_alt_m, t.baro_innovation_m,
        (unsigned)t.baro_accept_count, (unsigned)t.baro_reject_count,
        t.baro_calibrated ? 1 : 0, t.baro_healthy ? 1 : 0, t.baro_ground_noise_std_pa,
        t.vert_accel_ms2,
        // Đuôi mới — accel-primary altitude estimator (xem alt_estimator.h):
        // BDT=dt thật giữa 2 mẫu baro (s), VACCR=accel world-frame TRƯỚC LPF
        // (so với VACC=SAU LPF ở trên), ABIAS=bias đã học (state gamma),
        // VZAO/ZINE=Vz/Z tích phân THUẦN accel (baro không chạm — debug/test
        // E README), VZTGT=vz_target tầng ngoài cascade HOLD (0 nếu không
        // engage/không HOLDING-FLYING).
        t.baro_dt_s, t.vert_accel_raw_ms2, t.accel_bias_ms2,
        t.vz_accel_only_ms, t.z_inertial_m, t.alt_target_vz_ms,
        // Đuôi refactor an toàn/realtime
        t.motor_kill_latched ? 1 : 0, (int)t.takeoff_phase, 0,   /* LSC bỏ, xem trên */
        (int)t.loop_dt_us, (int)t.loop_max_us, (unsigned)t.deadline_miss_count,
        (int)t.imu_age_ms, (int)t.mag_age_ms, (int)t.baro_age_ms,
        t.heading_degraded ? 1 : 0,
        t.mixer_saturated ? 1 : 0,
        t.mixer_roll_limited ? 1 : 0, t.mixer_pitch_limited ? 1 : 0, t.mixer_yaw_limited ? 1 : 0,
        (double)t.mixer_headroom_duty, t.base_throttle_duty,
        t.alt_airborne ? 1 : 0, t.alt_liftoff_candidate ? 1 : 0, (double)t.az_corrected_ms2,
        (unsigned)t.baro_reject_consecutive, t.baro_reacquire_active ? 1 : 0,
        (unsigned)t.baro_seq, t.baro_fusion_initialized ? 1 : 0,
        (unsigned)t.alt_source,   /* ALTSRC -- phai dung NGAY SAU BFI, khop format */
        t.bat_adc_raw, t.bat_adc_mv, (double)t.bat_divider_ratio, (double)t.bat_voltage_raw_v,
        t.bat_valid ? 1 : 0, t.bat_calibrated ? 1 : 0, (int)t.bat_age_ms,
        // Đuôi TAKEOFF closed-loop
        t.takeoff_control_active ? 1 : 0,
        (double)t.takeoff_target_alt_m, (double)t.takeoff_z_sp_m, (double)t.takeoff_vz_target_ms,
        (double)t.takeoff_base_thrust, (double)t.takeoff_alt_corr,
        (double)t.takeoff_vz_i_term, (double)t.takeoff_ground_alt_m,
        t.takeoff_liftoff_flag ? 1 : 0,
        (double)t.takeoff_tilt_deg, (double)t.takeoff_elapsed_s,
        (int)t.takeoff_abort_reason, t.alt_pid_saturated ? 1 : 0,
        (int)t.arm_reject, (unsigned)t.arm_reject_seq,
        (int)t.takeoff_reject, (unsigned)t.takeoff_reject_seq,
        (double)t.tof_vertical_m, (double)t.tof_innovation_m, (double)t.tof_surface_z_m,
        (int)t.tof_surface_state, t.tof_correction_enabled ? 1 : 0,
        (double)t.tof_ground_range_m,
        (unsigned)t.tof_accept_count, (unsigned)t.tof_reject_count,
        (double)t.floor_plane_z_m, (double)t.landing_surface_z_m,
        t.landing_surface_valid ? 1 : 0, (int)t.tof_age_ms,
        t.tof_built_in ? 1 : 0,
        (double)t.tof_corr_z_m, (double)t.tof_corr_vz_ms,
        (double)t.baro_corr_z_m, (double)t.baro_corr_vz_ms,
        (double)t.bias_residual_m, (unsigned)t.bias_adapt_count,
        t.hover_latched ? 1 : 0, (double)t.hover_latch_v, (double)t.hover_latch_duty
    );
}
