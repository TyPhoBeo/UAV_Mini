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

// ================= WIFI EVENT HANDLER =================

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data) {
    (void)arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        return;
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected, reconnecting...");

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

    err = init_wifi_sta(hostname);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi STA init failed: %s", esp_err_to_name(err));
        return err;
    }

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
