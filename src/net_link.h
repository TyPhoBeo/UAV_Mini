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

// ================= CHAN DOAN WIFI =================
// VI SAO CAN: "khong ket noi duoc WiFi" gom it nhat bon nguyen nhan hoan toan
// khac nhau ma tu ngoai nhin y het nhau -- khong thay SSID, sai mat khau, AP
// tu choi, hoac DHCP khong cap IP. Chi co REASON CODE cua
// WIFI_EVENT_STA_DISCONNECTED phan biet duoc chung, va truoc day no bi VUT DI
// (handler chi log "reconnecting..."). Struct nay giu lai.
typedef struct {
    bool     started;             // esp_wifi_start() da chay
    bool     is_ap;               // true = dang PHAT wifi (WIFI_HOTSPOT=1)
    uint8_t  ap_clients;          // AP: so may dang noi vao. STA: luon 0
    bool     got_ip;              // dang co IP
    char     ssid[33];            // SSID dang nham toi (bien dich san)
    char     ip[16], netmask[16], gateway[16];
    char     mac[18];             // MAC cua STA
    char     bssid[18];           // MAC cua AP dang noi (rong neu chua noi)
    int8_t   rssi;                // dBm, chi co nghia khi got_ip
    uint8_t  channel;
    uint8_t  last_disc_reason;    // ma reason lan ngat gan nhat, 0 = chua tung
    uint32_t disc_count;          // so lan ngat tu luc boot
    bool     peer_known;          // da hoc duoc peer UDP chua
    char     peer[22];            // ip:port cua peer
} net_link_diag_t;

void net_link_get_diag(net_link_diag_t *out);

// Giai ma reason code sang tieng nguoi + goi y sua. Luon tra chuoi hop le.
const char *net_link_disc_reason_str(uint8_t reason);

// Quet AP xung quanh. BLOCKING ~2s va lam gian doan ket noi hien tai trong
// luc quet -> chi goi tu console, KHONG goi khi dang bay.
// Tra so AP tim duoc (<= max_out), -1 neu WiFi chua start.
typedef struct {
    char    ssid[33];
    int8_t  rssi;
    uint8_t channel;
    uint8_t authmode;   // wifi_auth_mode_t
} net_link_ap_t;

int net_link_scan(net_link_ap_t *out, int max_out);

#ifdef __cplusplus
}
#endif
