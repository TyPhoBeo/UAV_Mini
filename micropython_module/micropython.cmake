# Entry point cho USER_C_MODULES của MicroPython — trỏ idf.py vào FILE NÀY
# (không phải fc/micropython.cmake trực tiếp), vd:
#   idf.py -D USER_C_MODULES=<path>/UAV-S3/micropython_module/micropython.cmake build
# Chỉ có 1 module "fc" hiện tại — thêm module mới thì include() thêm ở đây.
include(${CMAKE_CURRENT_LIST_DIR}/fc/micropython.cmake)
