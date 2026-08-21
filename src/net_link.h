// net_link — WiFi STA + UDP console link, PORT trực tiếp từ UAV-Mini
// udp_link.hpp/.cpp (C++ class -> C thuần, cùng logic/hằng số).
//
// ESP32 kết nối WiFi hardcode (main/app_config.h: WIFI_STA_SSID/PASS), lắng
// nghe UDP tại <ESP32_IP>:WIFI_UDP_PORT. Peer (máy tính) được HỌC từ gói UDP
// ĐẦU TIÊN gửi tới; sau đó ESP32 chỉ gửi ngược dữ liệu cho peer đó, và tự
// "quên" peer nếu không nhận gói nào trong WIFI_UDP_PEER_TIMEOUT_MS.
//
// KHÔNG chạy PID/logic điều khiển ở đây — chỉ transport thô (byte in/out).
// command_parser.c mới là nơi diễn giải nội dung (xem kiến trúc trong README).
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// net_link_init() — init NVS (bắt buộc cho esp_wifi) nếu chưa, kết nối WiFi
// STA (blocking tới khi có IP hoặc WIFI_CONNECT_TIMEOUT_MS), mở UDP socket
// (non-blocking recv), spawn task RX riêng đẩy byte nhận được vào queue nội
// bộ. Gọi MỘT LẦN từ app_main() SAU flight_core_start().
esp_err_t net_link_init(const char *hostname);

// net_link_connected() — true nếu đã học được peer VÀ còn trong hạn
// WIFI_UDP_PEER_TIMEOUT_MS (net_link_write() tự bỏ qua nếu false).
bool net_link_connected(void);

// net_link_write() — gửi thô tới peer đã học (no-op nếu chưa có peer/timeout).
// KHÔNG block lâu (sendto() trên socket UDP không chờ ACK).
void net_link_write(const char *data, size_t len);

// net_link_read() — rút tối đa `len` byte đã nhận từ queue nội bộ (non-blocking,
// trả 0 ngay nếu queue rỗng). Gọi từ 1 task DUY NHẤT (net_task, xem src/main.c).
int net_link_read(char *buf, size_t len);

const char *net_link_ip_string(void);

// net_link_udp_rx_stack_free_bytes() — chỗ trống còn lại ít nhất trên stack của
// task udp_rx, tính bằng BYTE (xem sensor_hub_stack_free_bytes() để biết vì sao
// là byte chứ không phải word). Trả 0 nếu task chưa được tạo.
//
// udp_rx đáng đo riêng vì nó là task DUY NHẤT có kích thước gói do BÊN NGOÀI
// quyết định — một peer gửi burst lớn sẽ đi thẳng vào stack của nó.
uint32_t net_link_udp_rx_stack_free_bytes(void);
uint32_t net_link_udp_rx_stack_total_bytes(void);

#ifdef __cplusplus
}
#endif
