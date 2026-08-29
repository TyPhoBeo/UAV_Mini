// fc_features.h — cờ CÓ/KHÔNG từng cảm biến, ở mức BIÊN DỊCH.
//
// Khác với `cfg.mag_enabled` (cờ RUNTIME trong flight_core_board_config_t, chỉ
// quyết định "có gọi init không"): cờ ở đây quyết định driver có được BIÊN DỊCH
// VÀO FIRMWARE hay không. Cờ = 0 thì thân driver, khai báo hàm trong header, và
// mọi call site đều nằm sau #if -> không tốn một byte flash nào, và gọi nhầm
// hàm của cảm biến đã tắt là LỖI BIÊN DỊCH chứ không phải lỗi lúc chạy.
//
// ---------------- NGUỒN THẬT: main/app_config.h ----------------
// Ứng dụng (src/main.c, micropython_module/) đặt SENSOR_*_ENABLED ở đó. File
// này CHỈ dịch tên: SENSOR_MAG_ENABLED -> FC_FEATURE_MAG. Đừng sửa số ở đây.
//
// VÌ SAO dùng __has_include chứ không phải #include thẳng: components/
// flight_core/ phải build ĐỘC LẬP được, không có main/ bên cạnh (bất biến ghi ở
// đầu components/flight_core/CMakeLists.txt). __has_include cho phép "dùng nếu
// có, mặc định đầy đủ nếu không có" — thoả cả hai ràng buộc.
//
// VÌ SAO KHÔNG đọc app_config.h từ CMake rồi truyền -D: đã thử và HỎNG. CMake
// chỉ đọc file lúc configure, mà PlatformIO không chạy lại configure khi chỉ có
// header thay đổi -> sửa cờ xong build lại vẫn ra firmware CŨ, im lặng, không
// một cảnh báo nào. Đi qua #include thì trình biên dịch tự sinh depfile, ninja
// buộc phải dịch lại đúng những file bị ảnh hưởng. Cơ chế đúng là cơ chế mà
// build system KHÔNG THỂ bỏ sót.
//
// IMU KHÔNG có cờ: không IMU thì attitude không bao giờ valid, không arm được,
// nên "tắt IMU" không phải một cấu hình có nghĩa (app_config.h đã #error).
#pragma once

#if defined(__has_include)
#  if __has_include("app_config.h")
#    include "app_config.h"
#  endif
#endif

// Mặc định 1 (có đủ cảm biến) khi build ĐỘC LẬP, không thấy app_config.h.
#ifndef FC_FEATURE_MAG
#  ifdef SENSOR_MAG_ENABLED
#    define FC_FEATURE_MAG      SENSOR_MAG_ENABLED
#  else
#    define FC_FEATURE_MAG      1
#  endif
#endif

#ifndef FC_FEATURE_BARO
#  ifdef SENSOR_BARO_ENABLED
#    define FC_FEATURE_BARO     SENSOR_BARO_ENABLED
#  else
#    define FC_FEATURE_BARO     1
#  endif
#endif

#ifndef FC_FEATURE_TOF
#  ifdef SENSOR_TOF_ENABLED
#    define FC_FEATURE_TOF      SENSOR_TOF_ENABLED
#  else
#    define FC_FEATURE_TOF      1
#  endif
#endif

#ifndef FC_FEATURE_BATTERY
#  ifdef SENSOR_BATTERY_ENABLED
#    define FC_FEATURE_BATTERY  SENSOR_BATTERY_ENABLED
#  else
#    define FC_FEATURE_BATTERY  1
#  endif
#endif

// Latch ga hover theo pin (hover_model.h). KHÔNG phải cảm biến, nhưng đi qua
// CÙNG cơ chế vì cùng một lý do: đổi cờ ở app_config.h là đủ, và tắt thì code
// liên quan không được biên dịch vào.
#ifndef FC_FEATURE_HOVER_LATCH
#  ifdef HOVER_LATCH_ENABLED
#    define FC_FEATURE_HOVER_LATCH  HOVER_LATCH_ENABLED
#  else
#    define FC_FEATURE_HOVER_LATCH  1
#  endif
#endif

// Terrain offset (alt_estimator.h mục TERRAIN). Cùng cơ chế, cùng lý do: đổi
// cờ ở app_config.h là đủ, tắt thì code không được biên dịch vào.
// FC_FEATURE_BENCH_MODE — chay full logic, KHONG xuat ra motor.
// Xem app_config.h muc BENCH_MODE_ENABLED.
#ifndef FC_FEATURE_BENCH_MODE
#  ifdef BENCH_MODE_ENABLED
#    define FC_FEATURE_BENCH_MODE  BENCH_MODE_ENABLED
#  else
// Mac dinh 0 -- KHAC voi cac co cam bien o tren (mac dinh 1).
// Ly do: "khong thay app_config.h" phai co nghia la BAY THAT. Mac dinh 1 se
// lam mot build doc lap im lang khong quay dong co nao va rat kho doan ra.
#    define FC_FEATURE_BENCH_MODE  0
#  endif
#endif

#ifndef FC_FEATURE_TERRAIN_OFFSET
#  ifdef TERRAIN_OFFSET_ENABLED
#    define FC_FEATURE_TERRAIN_OFFSET  TERRAIN_OFFSET_ENABLED
#  else
#    define FC_FEATURE_TERRAIN_OFFSET  1
#  endif
#endif

// Terrain offset ĐỌC ToF để phát hiện bậc — không có ToF thì không có gì để
// phát hiện. Bắt tại đây thay vì để nó chạy trên một range luôn bằng 0.
#if FC_FEATURE_TERRAIN_OFFSET && !FC_FEATURE_TOF
#error "TERRAIN_OFFSET_ENABLED=1 can SENSOR_TOF_ENABLED=1 -- terrain offset do bac dia hinh bang range ToF"
#endif

// ============================================================================
// FC_FEATURE_FLOOR_GATE — CỔNG CHẶN theo "đã chốt được mặt sàn ToF"
// ============================================================================
// =1 (mặc định): ARM và TAKEOFF đòi alt_estimator_floor_ready() — tức ToF phải
//                gom đủ ALT_EST_FLOOR_MIN_SAMPLES mẫu ổn định (std nhỏ) khi
//                drone nằm yên, mới cho cất cánh.
// =0 (TẠM TẮT theo yêu cầu người dùng): bỏ hai cổng chặn đó. Việc THU mẫu sàn
//                và khoá sàn VẪN CHẠY bình thường — chỉ không dùng nó để TỪ
//                CHỐI lệnh nữa.
//
// ⚠ ĐÁNH ĐỔI KHI =0 — phải biết trước khi bay:
//   Mốc sàn (tof_ground_range_m) là gốc toạ độ của alt_m. Chưa chốt được mốc
//   mà cất cánh thì alt_m tính từ một gốc CHƯA ỔN ĐỊNH: độ cao báo về có thể
//   lệch, và alt_hold sẽ giữ sai đúng bằng lượng lệch đó. Cổng này sinh ra để
//   chặn đúng chuyện đó.
//   Các lớp an toàn KHÁC vẫn nguyên: estimator vẫn báo valid/degraded, ToF
//   lost vẫn -> Commander soft-fault -> auto-land, terrain guard vẫn chạy.
//
// Bật lại: đổi về 1 ở đây (hoặc định nghĩa FLOOR_GATE_ENABLED trong
// app_config.h) rồi build lại. KHÔNG có code nào bị xoá.
#ifndef FC_FEATURE_FLOOR_GATE
#  ifdef FLOOR_GATE_ENABLED
#    define FC_FEATURE_FLOOR_GATE  FLOOR_GATE_ENABLED
#  else
#    define FC_FEATURE_FLOOR_GATE  0
#  endif
#endif

// Latch đọc điện áp pin -> KHÔNG có ADC pin thì không có gì để latch. Bắt tại
// đây thay vì để nó âm thầm chốt hover từ một giá trị 0.0f không tồn tại.
#if FC_FEATURE_HOVER_LATCH && !FC_FEATURE_BATTERY
#error "HOVER_LATCH_ENABLED=1 can SENSOR_BATTERY_ENABLED=1 -- latch hover doc dien ap pin, khong co ADC pin thi khong the do"
#endif


// Bắt lỗi đánh máy (vd SENSOR_MAG_ENABLED để trống hoặc = 2) ngay lúc biên dịch.
#if (FC_FEATURE_MAG != 0 && FC_FEATURE_MAG != 1) || \
    (FC_FEATURE_BARO != 0 && FC_FEATURE_BARO != 1) || \
    (FC_FEATURE_TOF != 0 && FC_FEATURE_TOF != 1) || \
    (FC_FEATURE_BATTERY != 0 && FC_FEATURE_BATTERY != 1) || \
    (FC_FEATURE_HOVER_LATCH != 0 && FC_FEATURE_HOVER_LATCH != 1) || \
    (FC_FEATURE_FLOOR_GATE != 0 && FC_FEATURE_FLOOR_GATE != 1) || \
    (FC_FEATURE_TERRAIN_OFFSET != 0 && FC_FEATURE_TERRAIN_OFFSET != 1) || \
    (FC_FEATURE_BENCH_MODE != 0 && FC_FEATURE_BENCH_MODE != 1)
#error "FC_FEATURE_* (tu SENSOR_*_ENABLED trong app_config.h) chi duoc la 0 hoac 1"
#endif
