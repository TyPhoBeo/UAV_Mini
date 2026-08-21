# micropython.cmake — đăng ký "fc" như MicroPython user C module (USER_C_MODULES
# pattern chuẩn). Build port esp32 của MicroPython với:
#   idf.py -D MICROPY_BOARD=<board> -D USER_C_MODULES=<path-to>/UAV-S3/micropython_module/micropython.cmake build
# (micropython.cmake ở THƯ MỤC CHA sẽ include() file này — xem
# ../micropython.cmake và README.md phần "Build MicroPython port".)
add_library(usermod_fc INTERFACE)

target_sources(usermod_fc INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/fc_module.c
    ${CMAKE_CURRENT_LIST_DIR}/fc_bridge.c
)

target_include_directories(usermod_fc INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

# Link thẳng vào component flight_core (component C thuần, KHÔNG phụ thuộc
# MicroPython — xem components/flight_core/CMakeLists.txt). Đường dẫn tương
# đối giả định layout mặc định của repo này; sửa nếu anh di chuyển thư mục.
target_link_libraries(usermod_fc INTERFACE
    __idf_flight_core
)

target_link_libraries(usermod INTERFACE usermod_fc)
