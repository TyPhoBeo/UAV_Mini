cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(uav_s3_flight_core_verify)

# ===== Cho flight_core THẤY main/app_config.h (chỉ mở đường include) =====
# components/flight_core/ khai báo nó không phụ thuộc main/ — và vẫn đúng: nó
# dùng __has_include("app_config.h") (xem include/flight_core/fc_features.h),
# tức là "dùng nếu có". Việc quyết định CÓ hay không là của ứng dụng, nên câu
# lệnh mở đường nằm ở ĐÂY (gốc repo), không nằm trong CMakeLists của component.
#
# Đây là thay đổi ĐƯỜNG DẪN, không phải thay đổi GIÁ TRỊ cờ — nên nó không cần
# cmake chạy lại mỗi lần sửa cờ. Giá trị cờ đi qua #include bình thường, ninja
# tự theo dõi bằng depfile của trình biên dịch.
#
# PUBLIC: component nào dùng header của flight_core (src/) cũng thấy cùng đường
# dẫn -> #if trong header driver cho ra CÙNG kết quả ở mọi phía. Nếu chỉ PRIVATE
# thì flight_core biên dịch không có hàm còn src/ lại tưởng có -> lỗi link khó hiểu.
idf_component_get_property(_fc_lib flight_core COMPONENT_LIB)
target_include_directories(${_fc_lib} PUBLIC "${CMAKE_CURRENT_LIST_DIR}/main")

# ===== Cho uav_camera THẤY main/app_config.h + main/board_config.h =====
# Cùng lý do và cùng cách như flight_core ở trên: quyết định "có app_config
# hay không" là của ứng dụng, nên câu lệnh mở đường nằm ở gốc repo.
# camera_driver.c đọc CAM_*_GPIO từ board_config.h và SENSOR_CAMERA_ENABLED
# từ app_config.h — thiếu đường này thì #if ra 0 im lặng và camera "tắt" mà
# không ai biết vì sao.
idf_component_get_property(_cam_lib uav_camera COMPONENT_LIB)
target_include_directories(${_cam_lib} PUBLIC "${CMAKE_CURRENT_LIST_DIR}/main")
