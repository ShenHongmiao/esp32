#include "periph_i2c.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "driver/i2c.h"

#include "app_config.h"

// 记录 I2C 是否已完成初始化，避免重复安装驱动。
static bool s_i2c_ready = false;
// I2C 总线级互斥锁：串行化所有事务，避免跨任务并发访问导致总线冲突。
static SemaphoreHandle_t s_i2c_lock = NULL;

static esp_err_t i2c_lock(void) {
    if (s_i2c_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_i2c_lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static void i2c_unlock(void) {
    if (s_i2c_lock != NULL) {
        xSemaphoreGive(s_i2c_lock);
    }
}

esp_err_t periph_i2c_init(void) {
    if (s_i2c_lock == NULL) {
        s_i2c_lock = xSemaphoreCreateMutex();
        if (s_i2c_lock == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    // 已初始化时直接返回，保证接口幂等。
    if (s_i2c_ready) {
        return ESP_OK;
    }

    // 根据统一配置构造 I2C 主机参数。
    const i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = APP_I2C_SDA_GPIO,
        .scl_io_num = APP_I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = APP_I2C_FREQ_HZ,
        .clk_flags = 0,
    };

    // 第一步：写入控制器参数。
    esp_err_t err = i2c_param_config(APP_I2C_PORT, &conf);
    if (err != ESP_OK) {
        return err;
    }

    // 第二步：安装驱动。若已安装则返回 ESP_ERR_INVALID_STATE，这里视为可接受。
    err = i2c_driver_install(APP_I2C_PORT, conf.mode, 0, 0, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    s_i2c_ready = true;
    return ESP_OK;
}

esp_err_t periph_i2c_write_then_read(uint8_t addr, const uint8_t *wbuf, size_t wlen, uint8_t *rbuf, size_t rlen) {
    // 防御式检查：确保先初始化后使用。
    if (!s_i2c_ready || s_i2c_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = i2c_lock();
    if (err != ESP_OK) {
        return err;
    }

    // 典型时序：START + WRITE + RESTART + READ + STOP。
    err = i2c_master_write_read_device(APP_I2C_PORT, addr, wbuf, wlen, rbuf, rlen, pdMS_TO_TICKS(100));
    i2c_unlock();
    return err;
}

esp_err_t periph_i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t value) {
    // 单寄存器写入格式：[reg][value]
    uint8_t payload[2] = {reg, value};
    if (!s_i2c_ready || s_i2c_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = i2c_lock();
    if (err != ESP_OK) {
        return err;
    }

    err = i2c_master_write_to_device(APP_I2C_PORT, addr, payload, sizeof(payload), pdMS_TO_TICKS(100));
    i2c_unlock();
    return err;
}

esp_err_t periph_i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *value) {
    // 读取结果输出指针不能为空。
    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 先写寄存器地址，再读取 1 字节数据。
    return periph_i2c_write_then_read(addr, &reg, 1, value, 1);
}
