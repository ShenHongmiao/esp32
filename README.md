# ESP32-S3 加热控制系统

本项目基于 ESP32-S3 实现多路温度采样 + PID 控制 + PWM 加热输出 + UDP/串口通讯 + OTA 更新。整体设计将采样、控制、通讯、OTA 解耦成多个任务，关键参数集中在 `main/app_config.h` 统一管理。

## 1. 硬件与分区概览

### 1.1 设备资源

- 模组：ESP32S3-ZERO (ESP32S3-FH4R2)
- Flash：4MB Quad SPI
- PSRAM：2MB Quad SPI
- CPU：240MHz
- FreeRTOS tick：1000Hz

### 1.2 分区表（与 partitions.csv 对齐）

| Name | Type | SubType | Offset | Size | Flags | 描述 |
| --- | --- | --- | --- | --- | --- | --- |
| nvs | data | nvs | 0x9000 | 0x5000 |  | 存储 PID 参数、WiFi 密码、校准值 |
| otadata | data | ota | 0xe000 | 0x2000 |  | OTA 控制位 |
| phy_init | data | phy | 0x10000 | 0x1000 |  | 射频物理层初始化数据 |
| factory | app | factory | 0x20000 | 0x140000 |  | 出厂固件 |
| ota_0 | app | ota_0 | 0x160000 | 0x140000 |  | OTA 运行分区 |
| ota_1 | app | ota_1 | 0x2A0000 | 0x140000 |  | OTA 运行分区 |
| storage | data | fat | 0x3E0000 | 0x20000 |  | 预留自定义数据空间 |

## 2. 代码结构

`main/` 目录为核心业务代码，按功能划分：

- `main.c`：系统入口与任务编排
- `app_config.h`：所有可调参数/引脚/开关集中配置
- `periph_i2c.*`：I2C 总线初始化与读写（含互斥锁）
- `periph_adc.*`：外部 ADC 读数、12bit 拼接、原始值换算
- `periph_wf5803f.*`：WF5803F 温压传感器驱动
- `periph_pwm.*`：LEDC PWM 输出（导通时间或占空比）
- `ctrl_ntc.*`：NTC 电压 -> 温度换算（Beta 模型）
- `ctrl_pid.*`：PID 控制器实现
- `ctrl_failsafe.*`：心跳超时安全降级
- `comm_protocol.*`：帧协议封装、CRC8、载荷打包
- `comm_command.*`：文本命令解析（串口/UDP 共用）
- `comm_udp.*`：WiFi STA + UDP 发送/接收
- `sys_ota.*`：OTA 触发与温度安全门禁

`CMakeLists.txt` 与 `main/CMakeLists.txt` 已注册全部源文件与组件依赖。

## 3. 任务与数据流

### 3.1 任务划分

- `sampling_task`：高速采样 NTC 电压，维护滑动平均
- `control_task`：固定周期 PID 控制，输出 PWM 导通时间
- `telemetry_task`：周期输出日志并（可选）UDP 上报
- `udp_command_task`：接收 UDP 命令并更新控制参数
- `console_command_task`：USB 串口命令解析
- `ota_task`：低频轮询 OTA 请求并执行

### 3.2 数据快照机制

- 传感器采样与控制状态通过互斥锁保护
- `sampling_task` 更新共享状态
- `control_task` 读取快照后计算 PID
- `telemetry_task` 读取快照后组帧输出

## 4. 统一配置（app_config.h）

### 4.1 功能开关

- `FEATURE_NTC_CHx_ENABLE`：NTC 通道使能
- `FEATURE_WF5803F_ENABLE`：温压传感器
- `FEATURE_VOLTAGE_MONITOR_ENABLE`：电压监测
- `FEATURE_PID_OUT_ENABLE`：PID 输出上报
- `FEATURE_UPLOAD_ENABLE`：UDP 上报
- `FEATURE_WIRELESS_ENABLE`：WiFi/UDP 总开关
- `FEATURE_HEARTBEAT_FAILSAFE_ENABLE`：心跳保护

### 4.2 周期参数

- `APP_CONTROL_PERIOD_MS`：控制周期
- `APP_NTC_SAMPLE_PERIOD_MS`：采样周期
- `APP_NTC_FILTER_WINDOW_SIZE`：滑动窗口大小
- `APP_TELEMETRY_PERIOD_MS`：上报节流周期

### 4.3 PID 参数

- 默认上电 `Kp/Ki/Kd` 固定为 0
- `APP_PID_TASK_START_KP/KI/KD` 为控制任务启动后的参数
- 输出范围固定为 0~1000ms

## 5. I2C 与外设地址

### 5.1 I2C 总线

- SDA/SCL：GPIO11 / GPIO12
- 频率：400kHz
- 外部上拉：2kΩ

### 5.2 外部 ADC 地址（A1/A0 选择）

ADC 7 位地址由固定前缀 `10010` + `A1 A0` 组成。

| A1 | A0 | 7-bit 地址 (bin) | 7-bit 地址 (hex) |
| --- | --- | --- | --- |
| GND | GND | 1001000 | 0x48 |
| GND | VDD | 1001001 | 0x49 |
| VDD | GND | 1001010 | 0x4A |
| VDD | VDD | 1001011 | 0x4B |

- 工程当前使用：`0x48`
- 若换地址，请同步修改 `APP_EXT_ADC_ADDR`

### 5.3 WF5803F 地址

- 支持 `0x6C` / `0x6D`（7-bit）
- 工程默认 `APP_WF5803F_USE_ADDR = 0x6C`

## 6. ADC（补充图片内容）

### 6.1 读数格式

外部 ADC 返回 2 字节：

- Byte0：高 4 位固定，低 4 位为 D11~D8
- Byte1：低 8 位为 D7~D0

拼接方式：

```
raw12 = ((Byte0 & 0x0F) << 8) | Byte1
```

### 6.2 单端输入 + 外部基准命令表

命令格式：`SD C2 C1 C0 PD1 PD0 X X`

- 外部基准模式要求 `PD1=0`，`PD0=1`
- 下表对应 CH0~CH7：

| 通道 | 二进制命令 | 十六进制 |
| --- | --- | --- |
| CH0 | 1000 0100 | 0x84 |
| CH1 | 1100 0100 | 0xC4 |
| CH2 | 1001 0100 | 0x94 |
| CH3 | 1101 0100 | 0xD4 |
| CH4 | 1010 0100 | 0xA4 |
| CH5 | 1110 0100 | 0xE4 |
| CH6 | 1011 0100 | 0xB4 |
| CH7 | 1111 0100 | 0xF4 |

### 6.3 工程内通道映射

- `V_DETECT`：CH0（0x84）
- `NTC0`：CH1（0xC4）
- `NTC1`：CH2（0x94）
- `NTC2`：CH3（0xD4）
- `NTC3`：CH4（0xA4）

### 6.4 电压与输入电压换算

- 12bit 满量程：4095
- 参考电压：3.3V

```
V = raw / 4095 * 3.3
```

电源检测分压：上臂 51k，下臂 5.1k

```
Vin = Vdetect * (Rtop + Rbottom) / Rbottom
```

## 7. NTC 温度换算

- NTC：10k 3950，串联电阻 10k
- 分压公式：

$$R_{ntc} = R_s \cdot \frac{V_{out}}{V_{ref} - V_{out}}$$

- Beta 模型：

$$\frac{1}{T} = \frac{1}{T_0} + \frac{1}{B} \ln\left(\frac{R}{R_0}\right)$$

- 温度单位为摄氏度：$T_{c}=T_{k}-273.15$

## 8. WF5803F 温度/压力传感器

### 8.1 关键寄存器

- 控制寄存器：`0x30`
- 状态寄存器：`0x02`（bit0=1 表示转换完成）
- 压力数据：`0x06~0x08`（24bit）
- 温度数据：`0x09~0x0A`（16bit）

### 8.2 启动命令

- 推荐命令：`0x0A`（单次温度+压力联合测量）

### 8.3 数据换算

- 压力（2bar 型号）：

$$P_{kPa} = \frac{180}{0.81} \cdot \left(\frac{raw}{2^{23}} - 0.1\right) + 30$$

- 温度：

$$T_{c} = \frac{raw}{256}$$

## 9. PWM 输出

- LEDC 低速定时器，频率 20kHz
- PID 输出范围：0~1000ms
- 通过 `periph_pwm_set_on_time_ms()` 写入导通时间

## 10. 通讯协议

### 10.1 帧结构

```
HEAD(0xDE) + CMD + LEN + PAYLOAD + CRC8 + TAIL(0xED)
```

- 字节序：小端
- 数据缩放：浮点放大 100 倍转整数
- CRC8：多项式 0x07，覆盖 HEAD 至 PAYLOAD

### 10.2 CMD ID 与载荷

| CMD | 宏 | 描述 | 载荷格式 |
| --- | --- | --- | --- |
| 0x01 | CMD_NTC | NTC 温度 | int16_t×100，通道可选 |
| 0x02 | CMD_WF5803F | 温度+压力 | int16_t + int32_t |
| 0x03 | CMD_VOLTAGE | 电压状态 | int16_t + uint8_t |
| 0x04 | CMD_PID_OUT | PID 输出 | int32_t（ms×100） |
| 0x0F | CMD_TEXT_INFO | 文本信息 | ASCII 字符串 |

## 11. 命令输入（USB/UDP）

支持文本命令（大小写不敏感）：

- `HB` / `HEARTBEAT`
- `OTA`
- `SETPOINT=xx` / `SP=xx`
- `KP=xx`
- `KI=xx`
- `KD=xx`
- `ILIMIT=xx` / `INTEGRAL_LIMIT=xx`

## 12. OTA 策略

- OTA URL 由 `APP_OTA_URL` 提供
- 高温门禁：超过 `APP_OTA_SAFE_TEMP_C` 禁止 OTA
- 进入 OTA 前强制关闭 PWM 输出

## 13. 构建与烧录

- 建议使用 VS Code + ESP-IDF 插件执行 Build/Flash/Monitor
- 目标芯片：ESP32-S3

## 14. 常见问题排查

1. **WF5803F 无输出**
   - 确认 I2C 地址为 7-bit（0x6C/0x6D），而非 8-bit
   - 启动命令需为 0x0A，否则状态寄存器 bit0 可能一直为 0

2. **NTC 温度异常（25℃显示高温）**
   - 优先检查 ADC 原始值与通道命令是否匹配
   - 检查分压电阻与接线方向（NTC 近地端）

3. **无线调试受干扰**
   - 调试阶段可关闭 `FEATURE_WIRELESS_ENABLE` 以避免重连影响日志
