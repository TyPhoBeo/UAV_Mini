// net_link.c — PORT trực tiếp từ UAV-Mini src/udp_link.cpp (C++ -> C thuần,
// cùng logic/hằng số/tên biến, chỉ đổi cú pháp class->hàm tĩnh). Xem net_link.h.
#include "net_link.h"

#include <string.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"

#include "../main/app_config.h"

static const char *TAG = "net_link";

// Core cho task của tầng network. PHẢI là 0: core 1 dành riêng cho vòng bay
// (stabilize prio 23 + sensor_hub prio 22, xem components/flight_core/include/
// flight_core/sensor_hub.h). Cùng giá trị với APP_TASK_CORE trong src/main.c —
// hai file, hai module, nên mỗi bên tự khai báo thay vì kéo một header chung
// chỉ vì một hằng số.
#define NET_LINK_TASK_CORE   0

// ĐƠN VỊ: BYTE (xTaskCreate* của ESP-IDF nhận byte). Đặt tên hằng thay vì viết
// thẳng 4096 vào lời gọi để lệnh console `tasks` in ra được tổng dung lượng và
// so với high-water-mark — hai số đó phải cùng một nguồn, nếu không thì lần sau
// sửa một chỗ quên chỗ kia và bảng chẩn đoán nói dối.
#define UDP_RX_TASK_STACK_BYTES  4096

static TaskHandle_t s_udp_rx_task = NULL;

// ================= WIFI STATE =================

static EventGroupHandle_t s_wifi_event_group = NULL;
#define WIFI_CONNECTED_BIT   BIT0

static bool s_netif_created = false;
static bool s_wifi_started = false;

static char s_ip_string[16] = "0.0.0.0";

// ================= UDP STATE =================

static int  s_sock = -1;
static bool s_initialized = false;

static QueueHandle_t s_rx_queue = NULL;

static SemaphoreHandle_t s_peer_mtx = NULL;
static struct sockaddr_in s_peer_addr;
static bool    s_peer_known = false;
static uint8_t  s_last_disc_reason = 0;
static uint32_t s_disc_count = 0;
static int64_t s_peer_last_rx_us = 0;

// ================= NVS (bắt buộc cho esp_wifi) =================

static esp_err_t init_nvs_flash_safe(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS can xoa (%s) -> erase + init lai", esp_err_to_name(err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

// Khai bao truoc: wifi_event_handler() goi no de log ngay luc ngat.
const char *net_link_disc_reason_str(uint8_t reason);

// ================= WIFI EVENT HANDLER =================

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data) {
    (void)arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        return;
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        // GIU LAI reason code. Ban truoc chi log "reconnecting..." va nem no
        // di -- ma day la thu DUY NHAT phan biet duoc "khong thay SSID"
        // (reason 201) voi "sai mat khau" (reason 15/205). Thieu no thi moi
        // that bai WiFi deu trong giong nhau va chi con cach doan.
        const wifi_event_sta_disconnected_t *d =
            (const wifi_event_sta_disconnected_t *)event_data;
        if (d != NULL) {
            s_last_disc_reason = d->reason;
            s_disc_count++;
            ESP_LOGW(TAG, "WiFi disconnected: reason=%u (%s) -- thu lai...",
                     (unsigned)d->reason, net_link_disc_reason_str(d->reason));
        } else {
            ESP_LOGW(TAG, "WiFi disconnected (khong co reason) -- thu lai...");
        }

        snprintf(s_ip_string, sizeof(s_ip_string), "0.0.0.0");

        if (s_peer_mtx != NULL) {
            xSemaphoreTake(s_peer_mtx, portMAX_DELAY);
            s_peer_known = false;
            xSemaphoreGive(s_peer_mtx);
        }

        if (s_wifi_event_group != NULL) {
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        }

        esp_wifi_connect();
        return;
    }

    // ---- Su kien che do AP ----
    // AP KHONG co IP_EVENT_STA_GOT_IP: chinh no la ben cap DHCP. IP cua no co
    // dinh ngay tu luc esp_wifi_start(), nen "san sang" duoc dat o init chu
    // khong doi su kien nao.
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        const wifi_event_ap_staconnected_t *e =
            (const wifi_event_ap_staconnected_t *)event_data;
        if (e != NULL) {
            ESP_LOGI(TAG, "AP: may %02X:%02X:%02X:%02X:%02X:%02X da vao (aid=%d)",
                     e->mac[0], e->mac[1], e->mac[2], e->mac[3], e->mac[4], e->mac[5],
                     (int)e->aid);
        }
        return;
    }
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        const wifi_event_ap_stadisconnected_t *e =
            (const wifi_event_ap_stadisconnected_t *)event_data;
        if (e != NULL) {
            ESP_LOGW(TAG, "AP: may %02X:%02X:%02X:%02X:%02X:%02X da ra (aid=%d)",
                     e->mac[0], e->mac[1], e->mac[2], e->mac[3], e->mac[4], e->mac[5],
                     (int)e->aid);
        }
        // KHONG xoa peer o day: may co the roi wifi mot nhip roi vao lai, va
        // xoa peer nghia la ngung telemetry cho toi khi no gui goi moi. Peer tu
        // het han theo WIFI_UDP_PEER_TIMEOUT_MS -- de MOT co che lo viec do.
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;

        snprintf(s_ip_string, sizeof(s_ip_string), IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "WiFi got IP: %s", s_ip_string);

        if (s_wifi_event_group != NULL) {
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        }
        return;
    }
}

// ================= WIFI STA INIT =================

#if WIFI_HOTSPOT
// Mat khau 1..7 ky tu la LOI IM LANG: esp_wifi tu ha xuong mang MO va khong
// bao gi, nen ban tuong minh dang co mat khau trong khi ai cung vao duoc.
// Chan luc bien dich. Chuoi rong "" van hop le = co y de mang mo.
_Static_assert(sizeof(WIFI_AP_PASS) == 1 || sizeof(WIFI_AP_PASS) >= 9,
               "WIFI_AP_PASS phai RONG (mang mo) hoac >= 8 ky tu (WPA2). "
               "1..7 ky tu se bi esp_wifi am tham ha xuong mang mo.");
_Static_assert(sizeof(WIFI_AP_SSID) >= 2, "WIFI_AP_SSID khong duoc rong");

// init_wifi_ap() — ESP TU PHAT wifi. Khac STA o ba diem, va ca ba deu tung la
// cho de sai:
//   1. KHONG cho su kien got-IP. AP la ben cap DHCP; IP cua no co dinh
//      192.168.4.1 ngay khi esp_wifi_start() xong.
//   2. KHONG co timeout ket noi. Khong co gi de "ket noi" ca.
//   3. netif phai la ..._wifi_ap(), khong phai _sta(). Tao nham loai thi
//      esp_wifi_set_mode(AP) van OK nhung khong bao gio co DHCP server.
static esp_err_t init_wifi_ap(const char *hostname) {
    if (s_wifi_started) return ESP_OK;

    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    if (!s_netif_created) {
        esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
        if (ap_netif == NULL) return ESP_FAIL;
        if (hostname != NULL && hostname[0] != '\0') {
            esp_netif_set_hostname(ap_netif, hostname);
        }
        s_netif_created = true;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    err = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                &wifi_event_handler, NULL, NULL);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    wifi_config_t wc;
    memset(&wc, 0, sizeof(wc));
    strncpy((char *)wc.ap.ssid, WIFI_AP_SSID, sizeof(wc.ap.ssid) - 1);
    wc.ap.ssid_len       = (uint8_t)strlen(WIFI_AP_SSID);
    wc.ap.channel        = WIFI_AP_CHANNEL;
    wc.ap.max_connection = WIFI_AP_MAX_CONN;
    if (sizeof(WIFI_AP_PASS) >= 9) {
        strncpy((char *)wc.ap.password, WIFI_AP_PASS, sizeof(wc.ap.password) - 1);
        wc.ap.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        wc.ap.authmode = WIFI_AUTH_OPEN;   // chuoi rong = co y mo
    }

    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err != ESP_OK) return err;
    err = esp_wifi_set_config(WIFI_IF_AP, &wc);
    if (err != ESP_OK) return err;
    err = esp_wifi_start();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    // Cung ly do nhu STA: power-save lam tre lenh dieu khien.
    esp_wifi_set_ps(WIFI_PS_NONE);

    // Lay IP THAT tu netif thay vi hardcode "192.168.4.1": neu ai do doi
    // CONFIG_LWIP_..._IP thi hardcode se noi doi, ma day la con so nguoi dung
    // go vao trinh duyet.
    esp_netif_t *nif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    esp_netif_ip_info_t ipi;
    if (nif != NULL && esp_netif_get_ip_info(nif, &ipi) == ESP_OK) {
        snprintf(s_ip_string, sizeof(s_ip_string), IPSTR, IP2STR(&ipi.ip));
    }

    ESP_LOGI(TAG, "WiFi AP dang PHAT: SSID=\"%s\" kenh=%d %s, IP=%s",
             WIFI_AP_SSID, (int)WIFI_AP_CHANNEL,
             (sizeof(WIFI_AP_PASS) >= 9) ? "WPA2" : "MO (khong mat khau)",
             s_ip_string);

    s_wifi_started = true;
    return ESP_OK;
}
#endif  // WIFI_HOTSPOT

#if !WIFI_HOTSPOT
static esp_err_t init_wifi_sta(const char *hostname) {
    if (s_wifi_started) return ESP_OK;

    if (s_wifi_event_group == NULL) {
        s_wifi_event_group = xEventGroupCreate();
        if (s_wifi_event_group == NULL) return ESP_ERR_NO_MEM;
    }

    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    if (!s_netif_created) {
        esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
        if (sta_netif == NULL) return ESP_FAIL;

        if (hostname != NULL && hostname[0] != '\0') {
            esp_netif_set_hostname(sta_netif, hostname);
        }
        s_netif_created = true;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    err = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                &wifi_event_handler, NULL, NULL);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    err = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                &wifi_event_handler, NULL, NULL);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config));
    strncpy((char *)wifi_config.sta.ssid, WIFI_STA_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, WIFI_STA_PASS, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) return err;

    err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (err != ESP_OK) return err;

    err = esp_wifi_start();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    // Console link: tránh WiFi power-save gây trễ/jitter cho lệnh điều khiển.
    esp_wifi_set_ps(WIFI_PS_NONE);

    ESP_LOGI(TAG, "WiFi STA connecting to SSID=\"%s\"", WIFI_STA_SSID);

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT,
                                             pdFALSE, pdFALSE,
                                             pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));
    if ((bits & WIFI_CONNECTED_BIT) == 0) {
        ESP_LOGW(TAG, "WiFi connect timeout");
        return ESP_ERR_TIMEOUT;
    }

    s_wifi_started = true;
    return ESP_OK;
}

#endif  // !WIFI_HOTSPOT

// ================= RX TASK =================
// Block trên recvfrom(). Mỗi byte nhận được đẩy vào queue mà net_link_read()
// rút ra. Địa chỉ gói MỚI NHẤT trở thành "peer" mà net_link_write() gửi tới.

static void udp_rx_task(void *arg) {
    (void)arg;
    char buf[512];

    while (true) {
        struct sockaddr_in from;
        socklen_t from_len = sizeof(from);

        int n = recvfrom(s_sock, buf, sizeof(buf), 0, (struct sockaddr *)&from, &from_len);

        if (n <= 0) {
            if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                ESP_LOGW(TAG, "recvfrom failed errno=%d", errno);
            }
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        xSemaphoreTake(s_peer_mtx, portMAX_DELAY);

        const bool new_peer = !s_peer_known ||
                               s_peer_addr.sin_addr.s_addr != from.sin_addr.s_addr ||
                               s_peer_addr.sin_port != from.sin_port;

        s_peer_addr = from;
        s_peer_known = true;
        s_peer_last_rx_us = esp_timer_get_time();

        xSemaphoreGive(s_peer_mtx);

        if (new_peer) {
            char ip[16] = {0};
            inet_ntop(AF_INET, &from.sin_addr, ip, sizeof(ip));
            ESP_LOGI(TAG, "UDP peer learned: %s:%u", ip, (unsigned)ntohs(from.sin_port));
        }

        for (int i = 0; i < n; ++i) {
            char c = buf[i];
            if (xQueueSend(s_rx_queue, &c, 0) != pdTRUE) {
                ESP_LOGW(TAG, "UDP RX queue full, dropping byte");
                break;
            }
        }
    }
}

// ================= PUBLIC API =================

esp_err_t net_link_init(const char *hostname) {
    if (s_initialized) return ESP_OK;

    if (hostname == NULL || hostname[0] == '\0') hostname = WIFI_HOSTNAME;

    esp_err_t err = init_nvs_flash_safe();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS init that bai: %s", esp_err_to_name(err));
        return err;
    }

    if (s_peer_mtx == NULL) s_peer_mtx = xSemaphoreCreateMutex();
    if (s_rx_queue == NULL) s_rx_queue = xQueueCreate(WIFI_UDP_RX_QUEUE_LEN, sizeof(char));
    if (s_peer_mtx == NULL || s_rx_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create UDP mutex/queue");
        return ESP_ERR_NO_MEM;
    }

#if WIFI_HOTSPOT
    err = init_wifi_ap(hostname);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi AP init that bai: %s", esp_err_to_name(err));
        return err;
    }
#else
    err = init_wifi_sta(hostname);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi STA init failed: %s", esp_err_to_name(err));
        return err;
    }
#endif

    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s_sock < 0) {
        ESP_LOGE(TAG, "socket() failed errno=%d", errno);
        return ESP_FAIL;
    }

    int reuse = 1;
    setsockopt(s_sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in local_addr;
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(WIFI_UDP_PORT);
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    int rc = bind(s_sock, (struct sockaddr *)&local_addr, sizeof(local_addr));
    if (rc < 0) {
        ESP_LOGE(TAG, "bind UDP port %d failed errno=%d", WIFI_UDP_PORT, errno);
        close(s_sock);
        s_sock = -1;
        return ESP_FAIL;
    }

    // PIN CORE 0 (stack theo BYTE -> 4096 = 4KB). udp_rx block trong
    // recvfrom() phần lớn thời gian, nhưng nó thức theo lưu lượng mạng — tức
    // hoàn toàn do bên ngoài quyết định. Để tskNO_AFFINITY thì một burst UDP
    // được phép chen vào core 1 giữa các tick bay. Core 1 chỉ có vòng bay:
    // stabilize (prio 23) + sensor_hub (prio 22), xem sensor_hub.h.
    BaseType_t task_ok = xTaskCreatePinnedToCore(udp_rx_task, "udp_rx", UDP_RX_TASK_STACK_BYTES, NULL, 3,
                                                  &s_udp_rx_task, NET_LINK_TASK_CORE);
    if (task_ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create udp_rx task");
        close(s_sock);
        s_sock = -1;
        return ESP_ERR_NO_MEM;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "UDP link ready: %s:%u", s_ip_string, (unsigned)WIFI_UDP_PORT);
    return ESP_OK;
}

bool net_link_connected(void) {
    if (s_peer_mtx == NULL) return false;
    xSemaphoreTake(s_peer_mtx, portMAX_DELAY);
    bool known = s_peer_known;
    xSemaphoreGive(s_peer_mtx);
    return known;
}

void net_link_write(const char *data, size_t len) {
    if (!s_initialized || s_sock < 0 || data == NULL || len == 0) return;

    xSemaphoreTake(s_peer_mtx, portMAX_DELAY);

    bool known = s_peer_known;
    struct sockaddr_in peer = s_peer_addr;

    if (known && (esp_timer_get_time() - s_peer_last_rx_us) >
                     (int64_t)WIFI_UDP_PEER_TIMEOUT_MS * 1000) {
        s_peer_known = false;
        known = false;
    }

    xSemaphoreGive(s_peer_mtx);

    if (!known) return;

    sendto(s_sock, data, len, 0, (const struct sockaddr *)&peer, sizeof(peer));
}

int net_link_read(char *buf, size_t len) {
    if (s_rx_queue == NULL || buf == NULL || len == 0) return 0;

    int n = 0;
    while (n < (int)len) {
        char c = 0;
        if (xQueueReceive(s_rx_queue, &c, 0) != pdTRUE) break;
        buf[n++] = c;
    }
    return n;
}

uint32_t net_link_udp_rx_stack_free_bytes(void) {
    if (s_udp_rx_task == NULL) return 0;
    return (uint32_t)uxTaskGetStackHighWaterMark(s_udp_rx_task);
}

uint32_t net_link_udp_rx_stack_total_bytes(void) {
    return (uint32_t)UDP_RX_TASK_STACK_BYTES;
}

const char *net_link_ip_string(void) {
    return s_ip_string;
}

// ================= CHAN DOAN WIFI =================
// Xem net_link.h de biet vi sao khoi nay ton tai.

const char *net_link_disc_reason_str(uint8_t reason) {
    // Chi liet ke nhung ma THUC SU hay gap khi bring-up, kem GOI Y SUA. Danh
    // sach day du nam trong esp_wifi_types.h; chep het vao day chi lam nguoi
    // doc phai loc, ma luc dang do loi thi do dung la thu ho khong co thoi
    // gian lam.
    switch (reason) {
        case 2:   return "AUTH_EXPIRE — AP cho qua lau, thuong la song yeu";
        case 4:   return "ASSOC_EXPIRE — AP tha ra, song yeu hoac AP qua tai";
        case 15:  return "4WAY_HANDSHAKE_TIMEOUT — GAN NHU CHAC CHAN SAI MAT KHAU";
        case 201: return "NO_AP_FOUND — KHONG THAY SSID (sai ten, AP tat, hoac 5GHz)";
        case 202: return "AUTH_FAIL — AP tu choi xac thuc (sai mat khau/authmode)";
        case 203: return "ASSOC_FAIL — AP tu choi ket nap";
        case 204: return "HANDSHAKE_TIMEOUT — bat tay that bai";
        case 205: return "CONNECTION_FAIL — khong ket noi duoc, hay di kem sai mat khau";
        case 0:   return "chua tung ngat";
        default:  return "xem esp_wifi_types.h (wifi_err_reason_t)";
    }
}

void net_link_get_diag(net_link_diag_t *out) {
    if (out == NULL) return;
    memset(out, 0, sizeof(*out));

    out->started          = s_wifi_started;
    out->last_disc_reason = s_last_disc_reason;
#if WIFI_HOTSPOT
    out->is_ap = true;
    // O che do AP, "co IP" khong phu thuoc vao viec ai do da noi vao hay chua:
    // IP cua chinh AP co ngay tu luc start. Dung s_wifi_started lam moc.
    wifi_sta_list_t stal;
    if (esp_wifi_ap_get_sta_list(&stal) == ESP_OK) {
        out->ap_clients = (uint8_t)stal.num;
    }
#endif
    out->disc_count       = s_disc_count;
#if WIFI_HOTSPOT
    snprintf(out->ssid, sizeof(out->ssid), "%s", WIFI_AP_SSID);
#else
    snprintf(out->ssid, sizeof(out->ssid), "%s", WIFI_STA_SSID);
#endif
    snprintf(out->ip, sizeof(out->ip), "%s", s_ip_string);
    out->got_ip = (strcmp(s_ip_string, "0.0.0.0") != 0 && s_ip_string[0] != 0);

    uint8_t mac[6] = {0};
#if WIFI_HOTSPOT
    if (esp_wifi_get_mac(WIFI_IF_AP, mac) == ESP_OK) {
#else
    if (esp_wifi_get_mac(WIFI_IF_STA, mac) == ESP_OK) {
#endif
        snprintf(out->mac, sizeof(out->mac), "%02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }

    // ap_info CHI co nghia o che do STA (thong tin ve AP ma TA dang noi toi).
    // O che do AP ta CHINH LA AP -> khong co RSSI/BSSID nao de doc, va goi ham
    // nay se tra loi. Bo qua han.
#if !WIFI_HOTSPOT
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        out->rssi    = ap.rssi;
        out->channel = ap.primary;
        snprintf(out->bssid, sizeof(out->bssid), "%02X:%02X:%02X:%02X:%02X:%02X",
                 ap.bssid[0], ap.bssid[1], ap.bssid[2],
                 ap.bssid[3], ap.bssid[4], ap.bssid[5]);
    }
#endif

    esp_netif_t *nif = esp_netif_get_handle_from_ifkey(
#if WIFI_HOTSPOT
        "WIFI_AP_DEF");
#else
        "WIFI_STA_DEF");
#endif
    esp_netif_ip_info_t ipi;
    if (nif != NULL && esp_netif_get_ip_info(nif, &ipi) == ESP_OK) {
        snprintf(out->netmask, sizeof(out->netmask), IPSTR, IP2STR(&ipi.netmask));
        snprintf(out->gateway, sizeof(out->gateway), IPSTR, IP2STR(&ipi.gw));
    }

    if (s_peer_mtx != NULL) {
        xSemaphoreTake(s_peer_mtx, portMAX_DELAY);
        out->peer_known = s_peer_known;
        if (s_peer_known) {
            char ipbuf[16];
            inet_ntoa_r(s_peer_addr.sin_addr, ipbuf, sizeof(ipbuf));
            snprintf(out->peer, sizeof(out->peer), "%s:%u",
                     ipbuf, (unsigned)ntohs(s_peer_addr.sin_port));
        }
        xSemaphoreGive(s_peer_mtx);
    }
}

int net_link_scan(net_link_ap_t *out, int max_out) {
    if (out == NULL || max_out <= 0) return 0;
#if WIFI_HOTSPOT
    // esp_wifi_scan_start() doi interface STA. O che do AP thuan no tra loi,
    // nen tu choi o day voi ma rieng thay vi de nguoi dung thay "quet that bai"
    // ma khong biet vi sao.
    return -2;
#else
    if (!s_wifi_started && esp_wifi_start() != ESP_OK) return -1;

    // show_hidden=false: SSID an khong giup gi cho viec do loi nay, ma lam
    // danh sach dai them.
    wifi_scan_config_t sc;
    memset(&sc, 0, sizeof(sc));
    sc.show_hidden = false;

    if (esp_wifi_scan_start(&sc, true) != ESP_OK) return -1;

    uint16_t n = (uint16_t)max_out;
    wifi_ap_record_t recs[24];
    if (n > (uint16_t)(sizeof(recs) / sizeof(recs[0]))) {
        n = (uint16_t)(sizeof(recs) / sizeof(recs[0]));
    }
    if (esp_wifi_scan_get_ap_records(&n, recs) != ESP_OK) {
        esp_wifi_clear_ap_list();
        return -1;
    }

    for (uint16_t i = 0; i < n; ++i) {
        snprintf(out[i].ssid, sizeof(out[i].ssid), "%s", (const char *)recs[i].ssid);
        out[i].rssi     = recs[i].rssi;
        out[i].channel  = recs[i].primary;
        out[i].authmode = (uint8_t)recs[i].authmode;
    }
    return (int)n;
#endif
}
