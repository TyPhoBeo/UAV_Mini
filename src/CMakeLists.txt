# main.c dùng esp_console (REPL qua USB-Serial-JTAG) -> REQUIRES "console".
# net_link.c dùng WiFi STA + UDP (esp_wifi/esp_netif/esp_event/lwip) + NVS
# (esp_wifi cần NVS để lưu calib RF) -> REQUIRES tương ứng. Không component
# nào trong đây tự kéo theo cái còn lại.
FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(
    SRCS ${app_sources}
    REQUIRES console esp_wifi esp_netif esp_event lwip nvs_flash uav_camera
)
