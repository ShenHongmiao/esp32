#include "comm_udp.h"

#include <errno.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"

#include "app_config.h"

// 事件组位定义：用于标记 WiFi 已连网。
#define WIFI_CONNECTED_BIT  BIT0

static const char *TAG = "comm_udp";

// WiFi 状态相关全局资源。
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
static bool s_wifi_started = false;

// UDP socket 与远端地址缓存。
static int s_udp_sock = -1;
static struct sockaddr_in s_remote_addr;

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    (void)arg;
    (void)event_data;

    // STA 启动后主动发起连接。
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        // 断线自动重连（受最大重试次数限制）。
        if (s_retry_num < APP_WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGW(TAG, "wifi reconnect %d", s_retry_num);
        }
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        // 获取 IP 表示链路就绪，设置连接位。
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "wifi connected");
    }
}

static esp_err_t wifi_start_once(void) {
    // 已启动则直接复用。
    if (s_wifi_started) {
        return ESP_OK;
    }

    // 初始化网络栈与事件循环（重复调用时允许 INVALID_STATE）。
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    esp_netif_create_default_wifi_sta();

    // 初始化 WiFi 驱动。
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        return err;
    }

    if (s_wifi_event_group == NULL) {
        s_wifi_event_group = xEventGroupCreate();
        if (s_wifi_event_group == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    // 注册 WiFi 与 IP 事件回调。
    err = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    if (err != ESP_OK) {
        return err;
    }
    err = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);
    if (err != ESP_OK) {
        return err;
    }

    // 填充 STA 配置。
    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };
    strncpy((char *)wifi_config.sta.ssid, APP_WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, APP_WIFI_PASSWORD, sizeof(wifi_config.sta.password) - 1);

    // 设置模式、参数并启动 STA。
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        return err;
    }
    err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (err != ESP_OK) {
        return err;
    }
    err = esp_wifi_start();
    if (err != ESP_OK) {
        return err;
    }

    s_wifi_started = true;
    return ESP_OK;
}

static esp_err_t udp_socket_start_once(void) {
    // socket 已创建则直接返回。
    if (s_udp_sock >= 0) {
        return ESP_OK;
    }

    // 创建 UDP socket。
    s_udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s_udp_sock < 0) {
        return ESP_FAIL;
    }

    // 绑定本地端口，便于接收上位机命令。
    struct sockaddr_in local_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(APP_UDP_LOCAL_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(s_udp_sock, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        close(s_udp_sock);
        s_udp_sock = -1;
        return ESP_FAIL;
    }

    // 缓存远端上位机地址，用于 sendto。
    memset(&s_remote_addr, 0, sizeof(s_remote_addr));
    s_remote_addr.sin_family = AF_INET;
    s_remote_addr.sin_port = htons(APP_UDP_REMOTE_PORT);
    inet_pton(AF_INET, APP_UDP_REMOTE_IP, &s_remote_addr.sin_addr.s_addr);

    return ESP_OK;
}

esp_err_t comm_udp_start(void) {
    // 先保证 WiFi，再创建 UDP socket。
    esp_err_t err = wifi_start_once();
    if (err != ESP_OK) {
        return err;
    }

    return udp_socket_start_once();
}

bool comm_udp_is_connected(void) {
    // 未创建事件组视为未连接。
    if (s_wifi_event_group == NULL) {
        return false;
    }

    const EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
    return (bits & WIFI_CONNECTED_BIT) != 0;
}

esp_err_t comm_udp_send(const uint8_t *data, size_t len) {
    // 参数和状态检查。
    if (data == NULL || len == 0 || s_udp_sock < 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!comm_udp_is_connected()) {
        return ESP_ERR_INVALID_STATE;
    }

    const int sent = sendto(s_udp_sock, data, len, 0, (const struct sockaddr *)&s_remote_addr, sizeof(s_remote_addr));
    return sent >= 0 ? ESP_OK : ESP_FAIL;
}

int comm_udp_receive_line(char *out_line, size_t out_len, int timeout_ms) {
    // 接收缓冲检查。
    if (out_line == NULL || out_len < 2 || s_udp_sock < 0) {
        return -1;
    }

    // 设置接收超时，避免阻塞太久影响任务调度。
    struct timeval tv = {
        .tv_sec = timeout_ms / 1000,
        .tv_usec = (timeout_ms % 1000) * 1000,
    };
    setsockopt(s_udp_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in src_addr;
    socklen_t socklen = sizeof(src_addr);
    char rx[128] = {0};

    // 接收单个 UDP 报文。
    const int len = recvfrom(s_udp_sock, rx, sizeof(rx) - 1, 0, (struct sockaddr *)&src_addr, &socklen);
    if (len < 0) {
        // 超时返回 0，调用方可继续轮询。
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;
        }
        return -1;
    }

    // 安全拷贝并补字符串结尾。
    size_t copy_len = (size_t)len;
    if (copy_len >= out_len) {
        copy_len = out_len - 1;
    }
    memcpy(out_line, rx, copy_len);
    out_line[copy_len] = '\0';

    // 去掉行尾 CR/LF，统一上层解析输入。
    for (size_t i = 0; i < copy_len; ++i) {
        if (out_line[i] == '\r' || out_line[i] == '\n') {
            out_line[i] = '\0';
            break;
        }
    }

    return (int)strlen(out_line);
}
