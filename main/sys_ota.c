#include "sys_ota.h"

#include <string.h>

#include "esp_https_ota.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"

#include "app_config.h"
#include "periph_pwm.h"

static const char *TAG = "sys_ota";

void sys_ota_mark_app_valid(void) {
    // 通知引导系统：当前运行版本稳定可用，取消回滚标记。
    esp_ota_mark_app_valid_cancel_rollback();
}

esp_err_t sys_ota_perform_if_safe(const char *url, float current_temp_c) {
    // OTA 地址不能为空。
    if (url == NULL || url[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    // 高温时禁止写入，避免热失控风险。
    if (current_temp_c > APP_OTA_SAFE_TEMP_C) {
        ESP_LOGW(TAG, "ota rejected, temperature too high: %.2fC", current_temp_c);
        return ESP_ERR_INVALID_STATE;
    }

    // 进入 OTA 前强制关闭加热输出。
    periph_pwm_force_off();

    // 配置 HTTPS 客户端。
    esp_http_client_config_t http_cfg = {
        .url = url,
        .timeout_ms = 10000,
        .keep_alive_enable = true,
    };

    // 绑定 OTA 参数。
    esp_https_ota_config_t ota_cfg = {
        .http_config = &http_cfg,
    };

    // 开始下载并写入目标 OTA 分区。
    ESP_LOGI(TAG, "starting ota from %s", url);
    esp_err_t err = esp_https_ota(&ota_cfg);
    if (err == ESP_OK) {
        // 更新成功立即重启，切换到新固件。
        ESP_LOGI(TAG, "ota success, restart now");
        esp_restart();
    }

    ESP_LOGE(TAG, "ota failed: %s", esp_err_to_name(err));
    return err;
}
