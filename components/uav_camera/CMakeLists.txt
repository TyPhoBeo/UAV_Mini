# uav_camera — wrapper OV2640 + MJPEG server. KHONG phai driver camera: driver
# that la component chinh thuc espressif/esp32-camera (xem platformio.ini).
#
# Ca hai file .c deu tu boc trong `#if SENSOR_CAMERA_ENABLED`, nen khi co tat
# chung bien dich thanh object rong va component nay KHONG can esp32-camera.
# Do la ly do build mac dinh (camera tat) chay duoc ma khong phai tai gi ve.
#
# esp32_camera KHONG nam trong REQUIRES: neu liet ke, CMake doi no phai ton
# tai NGAY CA khi camera tat -> build gay cho nguoi chi muon bay. Khi bat
# camera, lib_deps trong platformio.ini keo component ve va no tu duoc them
# vao build; #include "esp_camera.h" luc do moi giai duoc.
idf_component_register(
    SRCS
        "src/camera_driver.c"
        "src/camera_stream.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        esp_http_server
        esp_driver_ledc
        esp_driver_gpio
        esp_timer
        log
        freertos
)
