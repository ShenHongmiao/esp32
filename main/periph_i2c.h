#ifndef PERIPH_I2C_H
#define PERIPH_I2C_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/*
 * I2C 外设抽象层
 * 目的：
 * 1) 屏蔽底层 I2C 驱动初始化与读写细节。
 * 2) 为上层传感器驱动提供统一接口，减少重复代码。
 */

// 初始化 I2C 主机控制器（幂等调用，多次调用只初始化一次）。
esp_err_t periph_i2c_init(void);

// 先写后读：常用于“写寄存器地址，再读数据块”。
esp_err_t periph_i2c_write_then_read(uint8_t addr, const uint8_t *wbuf, size_t wlen, uint8_t *rbuf, size_t rlen);

// 写单字节寄存器。
esp_err_t periph_i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t value);

// 读单字节寄存器。
esp_err_t periph_i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *value);

#endif  // PERIPH_I2C_H
