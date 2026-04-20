#ifndef SYS_OTA_H
#define SYS_OTA_H

#include "esp_err.h"

/*
 * OTA 系统模块
 * - 提供应用有效标记
 * - 提供带温度安全门禁的 OTA 执行接口
 */

// 在新固件正常启动后标记 app 为 valid，取消回滚状态。
void sys_ota_mark_app_valid(void);

// 当当前温度低于安全阈值时执行 OTA；否则返回状态错误。
esp_err_t sys_ota_perform_if_safe(const char *url, float current_temp_c);

#endif  // SYS_OTA_H
