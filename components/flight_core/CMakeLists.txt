# flight_core — ESP-IDF component, build/test ĐỘC LẬP được (không phụ thuộc
# main/ hay micropython_module/). Xem README.md phần "Build-verify flight_core".
idf_component_register(
    SRCS
        "src/mahony_filter.c"
        "src/pid.c"
        "src/attitude_control.c"
        "src/alt_estimator.c"
        "src/alt_hold.c"
        "src/hover_model.c"
        "src/takeoff_land.c"
        "src/flight_state_machine.c"
        "src/commander.c"
        "src/calibration.c"
        "src/sensor_hub.c"
        "src/flight_core.c"
        "src/drivers/imu_driver.c"
        "src/drivers/mag_driver.c"
        # ToF: facade + MỘT backend cho mỗi dòng chip. CẢ HAI backend luôn
        # nằm trong danh sách nguồn, nhưng mỗi file tự #if theo BOARD_TOF_CHIP
        # nên chỉ một cái sinh ra code — cái còn lại biên dịch thành object
        # rỗng. Làm vậy thay vì chọn file ở CMake vì CMake chỉ đọc cấu hình lúc
        # configure, mà PlatformIO KHÔNG chạy lại configure khi chỉ sửa header
        # -> đổi chip xong build lại vẫn ra firmware CŨ, im lặng (cùng cái bẫy
        # đã ghi ở đầu include/flight_core/fc_features.h). Qua #if thì trình
        # biên dịch tự sinh depfile và ninja buộc phải dịch lại.
        "src/drivers/tof_driver.c"
        "src/drivers/vl53l0x_driver.c"
        "src/drivers/vl53l1x_driver.c"
        "src/drivers/baro_driver.c"
        "src/drivers/motor_driver.c"
        "src/drivers/battery_driver.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        esp_driver_i2c
        esp_driver_ledc
        esp_driver_gpio
        esp_adc
        esp_timer
        nvs_flash
        log
        freertos
)
