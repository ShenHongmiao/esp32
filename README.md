# ESP32-S3 加热控制系统

本项目基于 ESP32-S3 实现多路 NTC 温度采样 + 双路 PID 控制 + 双路 PWM 加热输出 + UDP/串口双通道通讯 + OTA 固件更新，并支持压力传感器（WF5803F 温压一体传感器 或 外部 DC 电压型压力传感器）。整体设计将采样、控制、通讯、OTA 解耦成多个 FreeRTOS 任务，关键参数集中在 `main/app_config.h` 统一管理，双路 PID 独立计算、独立输出。支持三种加热模式：标准 PID 加热（模式 1）、循环 PID 加热（模式 2，温度档位自动切换）、双通道互锁交替循环（模式 3，专为相变驱动器设计，无保持时间）。

---

## 1. 硬件与分区概览

### 1.1 主控模组

- 模组型号：ESP32S3-ZERO（ESP32S3-FH4R2）
- CPU：Xtensa LX7 双核，240MHz
- Flash：4MB Quad SPI，80MHz QIO 模式
- PSRAM：2MB Quad SPI，80MHz，作为 malloc 堆使用
- FreeRTOS tick：1000Hz（1ms 分辨率）
- I2C 总线：GPIO11(SDA) / GPIO12(SCL)，频率 400kHz，外部 2kΩ 上拉
- PWM 输出：GPIO4(CH0) / GPIO5(CH1)，LEDC 低速定时器，20kHz，10bit 分辨率

### 1.2 外设地址分配

| 外设 | 总线 | 地址 | 备注 |
| --- | --- | --- | --- |
| 外部 ADC (ADS7924 兼容) | I2C | 0x48 | A1=GND, A0=GND |
| WF5803F 温压传感器 | I2C | 0x6C / 0x6D | 工程默认 0x6C |

### 1.3 分区表（partitions.csv）

| Name | Type | SubType | Offset | Size | 描述 |
| --- | --- | --- | --- | --- | --- |
| nvs | data | nvs | 0x9000 | 0x5000 (20KB) | 存储 PID 参数、WiFi 密码、校准值等非易失数据 |
| otadata | data | ota | 0xE000 | 0x2000 (8KB) | OTA 引导控制位，决定启动 factory/ota_0/ota_1 |
| phy_init | data | phy | 0x10000 | 0x1000 (4KB) | 射频物理层初始化校准数据 |
| factory | app | factory | 0x20000 | 0x140000 (1.25MB) | 出厂固件，永不覆盖 |
| ota_0 | app | ota_0 | 0x160000 | 0x140000 (1.25MB) | OTA 运行分区 A |
| ota_1 | app | ota_1 | 0x2A0000 | 0x140000 (1.25MB) | OTA 运行分区 B |
| storage | data | fat | 0x3E0000 | 0x20000 (128KB) | 预留自定义数据/文件系统空间 |

分区设计支持双 OTA 分区 + 出厂固件回滚，总计三个可启动固件。

---

## 2. 代码结构

`main/` 目录为核心业务代码，按功能层次划分为四层：

### 2.1 外设驱动层（periph_*）

| 文件 | 功能描述 |
| --- | --- |
| `periph_i2c.h/.c` | I2C 总线主机驱动：初始化（幂等调用）、先写后读（write_then_read）、单寄存器读写。内部维护 FreeRTOS 互斥锁 `s_i2c_lock` 串行化所有 I2C 事务，避免多任务并发访问导致总线冲突。所有 I2C 事务超时设为 100ms。 |
| `periph_adc.h/.c` | 外部 12 位 ADC 抽象层：通过 I2C 发送通道选择命令后读取 2 字节 12 位原始值（高字节低 4 位为 D11~D8 + 低字节 8 位为 D7~D0），提供原始值转电压（`raw12 / 4095 × Vref`）和电源电压反算（分压比还原）函数。 |
| `periph_pwm.h/.c` | 双通道 LEDC PWM 输出驱动：初始化定时器（20kHz / 10bit），配置 CH0(GPIO4) 和 CH1(GPIO5) 两个通道。支持以导通时间（ms）或占空比（%）设置输出，内部自动换算为 LEDC 计数值。上电默认关断。支持 `force_off` 强制关断和单通道独立关断。CH0/CH1 各有独立使能宏 `APP_PWM_CH0_ENABLE` / `APP_PWM_CH1_ENABLE`。 |
| `periph_wf5803f.h/.c` | WF5803F 温压一体传感器驱动：向命令寄存器(0x30)写入 0x0A 启动单次温压联合测量，轮询状态寄存器(0x02) bit0 等待转换完成（最多 20 次 × 2ms），从 0x06 连续读取 5 字节（压力 24bit 大端 + 温度 16bit 大端），换算为 kPa 和 ℃。 |
| `periph_pressure_dc.h/.c` | 外部 DC 电压型压力传感器驱动：读取指定 ADC 通道的 12 位原始值，经分压比例还原和线性模型 `Vout = 0.0188 × P + 0.2` 换算为 kPa。负压钳位为 0。 |

### 2.2 控制算法层（ctrl_*）

| 文件 | 功能描述 |
| --- | --- |
| `ctrl_ntc.h/.c` | NTC 热敏电阻温度换算：采用 Beta 模型，通过分压电压反推 NTC 阻值（`Rntc = Rs × V / (Vref - V)`），再代入 `1/T = 1/T0 + (1/B) × ln(R/R0)` 计算开尔文温度，最后转摄氏度。对输入电压越界（≤0 或 ≥Vref）做了异常保护。 |
| `ctrl_pid.h/.c` | 离散 PID 控制器：支持独立设置比例/积分/微分系数（`set_gains`）、目标值（`set_setpoint`）、积分限幅（`set_integral_limit`）、复位（`reset`）。`ctrl_pid_update()` 每周期执行：误差计算 → 死区判定 → 可选积分分离 → 微分差分 → 三项叠加 → 输出限幅。输出范围 0~1000ms。 |
| `ctrl_failsafe.h/.c` | 心跳失联安全保护：跟踪上位机最后心跳时间戳，超时（默认 5000ms）自动切换到安全温度（默认 30℃），心跳恢复后退出安全模式。使用无符号时间差计算，兼容 32 位毫秒计数回绕。 |

### 2.3 通讯层（comm_*）

| 文件 | 功能描述 |
| --- | --- |
| `comm_protocol.h/.c` | (收发通用) 帧协议封装：帧结构 `HEAD(0xDE) + CMD + LEN + PAYLOAD + CRC8 + TAIL(0xED)`。CRC8 多项式 0x07，覆盖 HEAD 至 PAYLOAD 全部字节。浮点数据统一放大 100 倍转为小端整数传输。提供标准载荷的动态与静态打包提取实现（CMD 0x01~0x05 等）。 |
| `comm_command.h/.c` | (命令接收) 文本命令解析：解析并响应上位机下发指令以实施控制。支持纯命令（`HB`、`OTA`）和键值对命令（`SETPOINT=xx`、`KP=xx`、`KI=xx`、`KD=xx`、`ILIMIT=xx`）。解析失败返回 `COMM_COMMAND_NONE`。串口和 UDP 共用同一解析器。 |
| `comm_udp.h/.c` | (收发通用) 网络链路层与套接字管理： WiFi 初始化（STA 模式、WPA2/WPA3 认证）、自动重连。 UDP socket 物理通讯层创建与绑定响应收发。提供 `send()` 上报遥测、`receive_line()` 接收指令数据流块。 |

### 2.4 系统服务层（sys_*）

| 文件 | 功能描述 |
| --- | --- |
| `sys_ota.h/.c` | OTA 固件更新系统：`sys_ota_mark_app_valid()` 在启动时标记当前固件有效（取消回滚），`sys_ota_perform_if_safe()` 在温度低于安全阈值（默认 45℃）时通过 HTTPS 下载并写入 OTA 分区。OTA 前强制关闭 PWM 输出，成功后自动重启。 |

### 2.5 配置与编排

| 文件 | 功能描述 |
| --- | --- |
| `app_config.h` | 工程统一配置文件：包含所有功能开关、加热模式、时序参数、PID 参数、I2C/ADC/PWM 引脚定义、NTC 参数、WiFi/UDP 参数、OTA URL 等。业务代码只依赖此处宏定义，便于调参与硬件迁移。 |
| `pressure_config.h` | 压力传感器派生配置：根据 `FEATURE_PRESSURE_ENABLE`、`FEATURE_PRESSURE_SOURCE`、`FEATURE_PRESSURE_DC_CHx` 的组合推导出 `APP_PRESSURE_SOURCE_WF`、`APP_PRESSURE_SOURCE_DC`、`APP_PRESSURE_DC_ADC_CMD` 等宏，并在编译期校验配置合法性（如 WF 来源需要启用 WF5803F、DC 来源需恰好选一个通道）。 |
| `main.c` | 系统入口与任务编排：初始化 NVS → 创建互斥锁 → 初始化运行态 → 初始化 PID/FailSafe → 初始化外设(I2C/PWM) → 启动 WiFi/UDP → 创建 6 个 FreeRTOS 任务并固定到双核。内含三种加热模式的控制逻辑（标准 PID / 循环 PID / 互锁交替循环）。详见第 3 节。 |
| `main/CMakeLists.txt` | 组件注册：列出全部 15 个源文件，依赖 `driver`、`esp_event`、`esp_netif`、`esp_wifi`、`esp_timer`、`nvs_flash`、`app_update`、`esp_https_ota`。 |

---

## 3. 任务与数据流

### 3.1 任务划分与双核分配

系统共 6 个 FreeRTOS 任务，按职责分配到 ESP32-S3 的两个核心：

**核心 1（core_ctrl）：控制核心，运行实时性要求最高的任务**

| 任务名 | 优先级 | 栈大小 | 周期 | 职责 |
| --- | --- | --- | --- | --- |
| `control_task` | 8（最高） | 4096B | 20ms | 双路 PID 控制计算与 PWM 输出 |
| `sampling_task` | 7 | 4096B | 2ms | 高速 NTC 电压采样与滑动平均滤波 |

**核心 0（core_comm）：通讯核心，运行 IO 密集型任务**

| 任务名 | 优先级 | 栈大小 | 周期 | 职责 |
| --- | --- | --- | --- | --- |
| `udp_command_task` | 6 | 4096B | 200ms 超时轮询 | UDP 命令接收与解析 |
| `console_command_task` | 6 | 4096B | 非阻塞轮询 | USB 串口命令接收与解析 |
| `telemetry_task` | 5 | 4096B | 100ms | 遥测数据组帧与 UDP/串口上报 |
| `ota_task` | 4 | 6144B | 500ms | OTA 请求轮询与安全执行 |

核心分配策略：控制核心独占运行 `control_task` 和 `sampling_task`（优先级 7~8），保证 PID 控制响应的实时性和确定性；通讯核心运行所有 IO 和通讯任务（优先级 4~6），避免阻塞控制链路。

### 3.2 系统运行阶段

**阶段 1：上电初始化（`app_main`）**

```
init_nvs() → sys_ota_mark_app_valid() → 创建互斥锁
→ ntc_filter_reset() → runtime_init() → ctrl_pid_init(kp=0,ki=0,kd=0)
→ ctrl_failsafe_init() → periph_i2c_init() → periph_pwm_init()
→ comm_udp_start() → 创建 6 个任务
```

关键设计：上电阶段 PID 参数固定为 0/0/0，确保烧录和启动阶段 PWM 输出为 0，不会误加热。

**阶段 2：控制任务启动加载运行参数**

`control_task` 首次进入循环前，通过互斥锁保护地将两路 PID 参数更新为 `APP_PID_TASK_START_KP/KI/KD` 并复位积分项和历史误差，此后 PID 正常计算。这种两阶段加载避免了上电瞬态的非预期输出。

**阶段 3：稳态运行**

三个核心循环并行运行：
- `sampling_task`（2ms）：ADC 采样 → 滑动窗口更新 → 锁保护写入共享状态
- `control_task`（20ms）：锁保护读取快照 → 选择温度源 → 根据加热模式计算每路设定值（模式 1 使用固定值 / 模式 2 循环切换 / 模式 3 互锁交替）→ PID 更新 → 锁保护写回状态 → PWM 输出
- `telemetry_task`（100ms）：锁保护读取快照 → 按功能开关组帧 → USB 日志 + UDP 上报

### 3.3 数据快照与互斥锁机制

全局共享状态 `s_state`（类型 `app_runtime_t`）通过单一互斥锁 `s_state_lock` 保护：

```
app_runtime_t {
    ntc_temp_c[4], ntc_voltage_v[4], ntc_valid[4]  // NTC 四路数据
    wf_temp_c, wf_pressure_kpa, wf_valid            // WF5803F 数据
    dc_pressure_kpa_ch1, dc_pressure_kpa_ch2        // DC 压力双通道数据
    pressure_mask                                   // 压力数据有效性掩码 (bit0: CH1, bit1: CH2)
    supply_voltage_v, undervoltage                   // 电源监测
    process_temp_c[2], requested_setpoint_c          // 控制输入
    effective_setpoint_c[2], pwm_on_ms[2]            // 控制输出
    last_heartbeat_ms, ota_pending                   // 通讯状态
}
```

各任务在临界区内快速复制所需字段到本地快照后立即释放锁，临界区仅包含内存拷贝操作，无 I/O 或计算，保证低延迟。NTC 滑动滤波器的环形缓冲区也通过同一锁保护。

### 3.4 双路控制映射

系统维护 2 个控制组（`APP_CONTROL_GROUPS = 2`），通过 `control_group_map_t` 定义每组的主/备 NTC 通道和输出 PWM 通道：

| 控制组 | PID 实例 | 主 NTC | 备 NTC | PWM 输出 | GPIO |
| --- | --- | --- | --- | --- | --- |
| Group 0 | `s_pid[0]` | NTC0 (ADC CH1) | NTC1 (ADC CH2) | PWM CH1 | GPIO5 |
| Group 1 | `s_pid[1]` | NTC2 (ADC CH3) | NTC3 (ADC CH4) | PWM CH0 | GPIO4 |

温度源选择规则：主通道有效则使用主通道值，主通道无效则回退到备通道值，两路都无效则该组控制暂停（PWM 强制关断）。控制温度采用平均规则：主备双通道都有效时取平均值作为过程温度，单通道有效取单通道值。

上位机命令的设定值与 PID 参数（Kp/Ki/Kd/ILimit）对两路同步下发，但两路 PID 控制器各自独立维护积分项和历史误差，控制输出分别独立计算。

### 3.5 NTC 滑动窗口滤波器

每个 NTC 通道维护独立的环形缓冲区：

```c
ntc_filter_state_t {
    voltage_ring[4][WINDOW_SIZE]  // 4 通道 × N 样本环形缓冲
    voltage_sum[4]                // 当前窗口电压和（增量维护，避免重复求和）
    filtered_voltage_v[4]        // 当前窗口均值
    ring_head[4]                  // 环形缓冲区写入位置
    sample_count[4]               // 已采集样本数（≤ WINDOW_SIZE）
    filtered_valid[4]             // 滤波器是否已有有效输出
}
```

`sampling_task` 每 2ms 采集一次各使能通道的 ADC 电压，通过 `ntc_filter_push_voltage_locked()` 推入环形缓冲区：新样本覆盖最旧样本，增量更新电压和，重新计算均值。窗口大小为 `APP_NTC_FILTER_WINDOW_SIZE`（默认 10），即 20ms 的滑动平均窗口。滤波器未填满窗口时，均值基于已采集的样本数计算。

### 3.6 慢采样分频

NTC 电压以 2ms 周期高速采样，而非 NTC 外设（电源电压检测、DC 压力传感器、WF5803F）以较慢的控制周期（20ms）采样。通过 `slow_sample_div` 分频器实现：`sampling_task` 内部维护计数器，每 `APP_CONTROL_PERIOD_MS / APP_NTC_SAMPLE_PERIOD_MS` 次（默认 10 次）触发一次慢采样，读取非 NTC 外设并更新到共享状态。这避免了不必要的 I2C 总线占用。

---

## 4. 统一配置（app_config.h）

### 4.1 功能开关

| 宏定义 | 默认值 | 说明 |
| --- | --- | --- |
| `FEATURE_HEATING_MODE` | 2 | 加热模式：1=标准 PID 加热；2=循环 PID 加热（档位 1/2 自动切换）；3=双通道互锁交替循环（相变驱动器，无保持时间） |
| `FEATURE_NTC_CH0_ENABLE` | 1 | NTC 通道 0 使能（ADC CH1，对应控制组 0 主通道） |
| `FEATURE_NTC_CH1_ENABLE` | 0 | NTC 通道 1 使能（ADC CH2，对应控制组 0 备通道） |
| `FEATURE_NTC_CH2_ENABLE` | 0 | NTC 通道 2 使能（ADC CH3，对应控制组 1 主通道） |
| `FEATURE_NTC_CH3_ENABLE` | 0 | NTC 通道 3 使能（ADC CH4，对应控制组 1 备通道） |
| `FEATURE_WF5803F_ENABLE` | 0 | WF5803F 温压传感器功能使能 |
| `FEATURE_PRESSURE_ENABLE` | 1 | 气压检测总开关：1=启用；0=关闭 |
| `FEATURE_PRESSURE_SOURCE` | 0 | 气压来源选择：0=外部 DC 电压型；1=WF5803F |
| `FEATURE_PRESSURE_DC_CH1` | 1 | DC 压力通道 1 选择（ADC CH7, 0xF4） |
| `FEATURE_PRESSURE_DC_CH2` | 0 | DC 压力通道 2 选择（ADC CH0, 0x84） |
| `FEATURE_VOLTAGE_MONITOR_ENABLE` | 1 | 电源电压监测与欠压保护 |
| `FEATURE_PID_OUT_ENABLE` | 1 | PID 输出值上报（CMD_PID_OUT 帧） |
| `FEATURE_UPLOAD_ENABLE` | 1 | UDP 数据上报总开关（0=仅串口日志） |
| `FEATURE_WIRELESS_ENABLE` | 1 | WiFi/UDP 无线总开关（0=完全关闭 WiFi 和 UDP 任务） |
| `FEATURE_HEARTBEAT_FAILSAFE_ENABLE` | 0 | 心跳失联保护开关：1=超时降级到安全温度；0=忽略心跳超时 |
| `APP_PWM_CH0_ENABLE` | 0 | PWM CH0（GPIO4）输出使能 |
| `APP_PWM_CH1_ENABLE` | 1 | PWM CH1（GPIO5）输出使能 |

### 4.2 控制周期与时序参数

| 宏定义 | 默认值 | 说明 |
| --- | --- | --- |
| `APP_CONTROL_PERIOD_MS` | 20 | 控制任务周期（ms），也是其他任务的时序基准 |
| `APP_NTC_SAMPLE_PERIOD_MS` | 2 | NTC 后台采样周期（ms），2ms = 500Hz 采样率 |
| `APP_NTC_FILTER_WINDOW_SIZE` | 10 | NTC 滑动平均窗口样本数（10 × 2ms = 20ms 窗口） |
| `APP_TELEMETRY_PERIOD_MS` | 100 | 遥测上报周期（ms），用于串口日志和 UDP 发送节流 |
| `APP_HEARTBEAT_TIMEOUT_MS` | 5000 | 心跳超时（ms），超过此时间未收到上位机命令进入安全模式 |
| `APP_SAFE_SETPOINT_C` | 30.0 | 失联安全模式目标温度（℃） |

### 4.3 PID 参数

#### 4.3.1 上电默认参数（固定为 0，勿改）

| 宏定义 | 默认值 | 说明 |
| --- | --- | --- |
| `APP_PID_KP_DEFAULT` | 0.0 | 上电阶段比例增益（必须为 0） |
| `APP_PID_KI_DEFAULT` | 0.0 | 上电阶段积分增益（必须为 0） |
| `APP_PID_KD_DEFAULT` | 0.0 | 上电阶段微分增益（必须为 0） |

#### 4.3.2 控制任务启动后加载的运行参数

| 宏定义 | 默认值 | 说明 |
| --- | --- | --- |
| `APP_PID_TASK_START_KP` | 83.5 | 控制任务启动后的比例增益（等幅振荡临界点：Kp=230，周期 9s） |
| `APP_PID_TASK_START_KI` | 13.8 | 控制任务启动后的积分增益 |
| `APP_PID_TASK_START_KD` | 0.0 | 控制任务启动后的微分增益 |
| `APP_PID_ILIMIT_DEFAULT` | 50.0 | 积分限幅（%），防止积分风暴（windup） |
| `APP_PID_DEADBAND_C` | 0.2 | 死区（℃）：误差落入死区内时保持当前输出，不积分不调节 |
| `APP_PID_ENABLE_INTEGRAL_SEPARATION` | 1 | 积分分离使能：1=启用；0=传统积分 |
| `APP_PID_INTEGRAL_SEPARATION_THRESHOLD_C` | 3.0 | 积分分离阈值（℃）：仅在误差绝对值 ≤ 该值时累积积分 |
| `APP_PID_OUTPUT_MIN_MS` | 0.0 | PID 输出下限（ms 导通时间） |
| `APP_PID_OUTPUT_MAX_MS` | 1000.0 | PID 输出上限（ms 导通时间），对应 1s 控制窗口全导通 |

#### 4.3.3 PID 控制器特性

- **积分分离**：当误差较大时暂停积分累积，仅 P+D 作用，避免大偏差下积分饱和导致超调；误差进入阈值范围后恢复积分，消除稳态误差。
- **死区**：误差落入 ±0.2℃ 范围时，PID 输出保持不变（不做计算），避免微小幅值振荡导致的执行器频繁动作。
- **积分限幅**：积分项的绝对值被限制在积分限幅参数（默认 50.0%）范围内，防止长时间偏差累积导致的积分风暴。
- **Ki 变更清积分**：当 Ki 参数被运行时修改时，积分项自动清零，避免历史积分值与新 Ki 参数不匹配。
- **输出限幅**：PID 输出被钳位在 0~1000ms 之间，映射到 1s PWM 周期的导通时长。

### 4.4 加热模式参数

三种加热模式通过 `FEATURE_HEATING_MODE` 宏在编译期互斥选择（彼此完全独立，无嵌套）。

| 宏定义 | 默认值 | 说明 |
| --- | --- | --- |
| `APP_DEFAULT_SETPOINT_C` | 50.0 | 初始目标温度 / 模式 2 档位 1 温度 / 模式 3 低温点 T_low（℃） |
| `APP_CYCLIC_SETPOINT2_C` | 60.0 | 模式 2 档位 2 温度 / 模式 3 高温点 T_high（℃） |
| `APP_CYCLIC_HOLD_THRESHOLD_C` | 0.5 | 到达判定阈值（℃）：过程温度进入目标温度 ± 该阈值视为"已到达" |
| `APP_CYCLIC_HOLD_TIME_MS` | 1000 | 保持时间（ms）：仅模式 2 使用，稳定在目标范围内持续该时间后切换档位 |
| `APP_MODE3_TRIG_TEMP_C` | 35.0 | 模式 3 专有：降温触发阈值（℃），冷却通道温度降至该值以下即刻触发另一路加热 |

#### 4.4.1 模式 2：循环 PID 加热

当 `FEATURE_HEATING_MODE = 2` 时启用。每路控制组独立在档位 1（`APP_DEFAULT_SETPOINT_C`，默认 50℃）和档位 2（`APP_CYCLIC_SETPOINT2_C`，默认 60℃）之间自动切换。当过程温度进入目标温度 ± `APP_CYCLIC_HOLD_THRESHOLD_C` 范围内，开始计时；持续满足该条件达到 `APP_CYCLIC_HOLD_TIME_MS` 后，自动切换到另一档位。档位切换时 PID 控制器自动复位（清积分和历史误差），避免档位跳变导致的控制冲击。

核心函数：[`update_cyclic_setpoint_group()`](main/main.c#L177)，状态变量 `s_cyclic_stage[2]`、`s_cyclic_hold_start_ms[2]`。

示例时序（单路）：初始 50℃ → 加热到 49.5~50.5℃ 并保持 1s → 切换到 60℃ → 加热到 59.5~60.5℃ 并保持 1s → 切换回 50℃ → 循环往复。

#### 4.4.2 模式 3：双通道互锁交替循环

当 `FEATURE_HEATING_MODE = 3` 时启用。专为相变驱动器设计，**无需高温保持时间，达到高温即刻进入冷却**。两路控制通道互锁：同一时刻仅一路加热（目标 T_high），另一路冷却/待命（目标 T_low），交替循环。

**固有参数借用：**

| 角色 | 宏来源 | 默认值 |
| --- | --- | --- |
| 低温点 T_low | `APP_DEFAULT_SETPOINT_C`（即运行时 `requested_sp`，可通过 `SETPOINT=` 命令动态修改） | 50℃ |
| 高温点 T_high | `APP_CYCLIC_SETPOINT2_C`（编译期固定） | 60℃ |
| 到达高温判定 | `APP_CYCLIC_HOLD_THRESHOLD_C`：过程温度与 T_high 偏差 | ≤ 0.5℃ 视为已到达 |
| 降温触发判定 | `APP_MODE3_TRIG_TEMP_C`：冷却通道温度 ≤ 35℃ 时触发 | 35℃ |

> `APP_CYCLIC_HOLD_TIME_MS` **不被模式 3 使用**——模式 3 的核心理念是"达到即刻跳转，不等待"。

**四态状态机：**

核心函数：[`update_mode3_setpoints()`](main/main.c#L440)，状态变量 `s_mode3_state : mode3_state_t`，初始状态 `MODE3_CH0_HEAT`。

```
                        ┌──────────────────────────┐
                        │     MODE3_CH0_HEAT        │
                        │ CH0 → T_high (加热)       │
                        │ CH1 → T_low  (冷却)       │
                        │ 条件: CH0 到达高温 →       │
                        └────────────┬─────────────┘
                                     │ 到达高温即刻跳转
                                     ▼
                        ┌──────────────────────────┐
                        │     MODE3_CH0_COOL        │
                        │ CH0 → T_low  (冷却)       │
                        │ CH1 → T_low  (冷却)       │
                        │ 条件: CH0 冷却到 ≤ 35℃ →  │
                        └────────────┬─────────────┘
                                     │
                                     ▼
                        ┌──────────────────────────┐
                        │     MODE3_CH1_HEAT        │
                        │ CH0 → T_low  (冷却)       │
                        │ CH1 → T_high (加热)       │
                        │ 条件: CH1 到达高温 →       │
                        └────────────┬─────────────┘
                                     │ 到达高温即刻跳转
                                     ▼
                        ┌──────────────────────────┐
                        │     MODE3_CH1_COOL        │
                        │ CH0 → T_low  (冷却)       │
                        │ CH1 → T_low  (冷却)       │
                        │ 条件: CH1 冷却到 ≤ 35℃ →  │
                        └────────────┬─────────────┘
                                     │
                                     ▼
                         回到 MODE3_CH0_HEAT（闭环）
```

**每个状态的输出与跳转条件：**

| 状态 | CH0 设定值 | CH1 设定值 | 跳转条件 | 跳转目标 |
| --- | --- | --- | --- | --- |
| `MODE3_CH0_HEAT` | T_high (60℃) | T_low (50℃) | `\|CH0 - T_high\| ≤ 0.5℃` | `MODE3_CH0_COOL` |
| `MODE3_CH0_COOL` | T_low (50℃) | T_low (50℃) | `CH0 ≤ 35℃` | `MODE3_CH1_HEAT` |
| `MODE3_CH1_HEAT` | T_low (50℃) | T_high (60℃) | `\|CH1 - T_high\| ≤ 0.5℃` | `MODE3_CH1_COOL` |
| `MODE3_CH1_COOL` | T_low (50℃) | T_low (50℃) | `CH1 ≤ 35℃` | `MODE3_CH0_HEAT` |

**完整时序示例（假定两端加热/冷却速率对称）：**

```
t=0s:  状态=MODE3_CH0_HEAT  CH0→60℃(加热中)  CH1→50℃(待命)
t=3s:  CH0 到达 59.5℃ → 进入 MODE3_CH0_COOL
        CH0→50℃(冷却中)  CH1→50℃(待命)
t=8s:  CH0 降至 35.0℃ → 进入 MODE3_CH1_HEAT
        CH0→50℃(待命)  CH1→60℃(加热中)
t=11s: CH1 到达 59.5℃ → 进入 MODE3_CH1_COOL
        CH0→50℃(待命)  CH1→50℃(冷却中)
t=16s: CH1 降至 35.0℃ → 回到 MODE3_CH0_HEAT
        循环往复...
```

**关键特性：**

- **互锁保护**：任意时刻只有一路设定为高温，另一路为低温或冷却，从逻辑上保证不会两路同时全功率加热。
- **无保持时间**：到达高温后**不等待**，立即跳转。这一点与模式 2 根本不同。
- **PID 目标值突变**：模式 3 每次状态切换时两路设定值会突变（如 T_high→T_low），PID 复位逻辑（`s_last_target_sp` 变化自动 `ctrl_pid_reset`）对模式 3 同样生效，避免积分冲击。
- **传感器失效保护**：若某通道温度无效（`valid=false`），该通道对应的跳转条件不会触发，状态机停在当前状态，该通道 PWM 被关断（由 `control_task` 通用逻辑保证）。
- **编译期互斥**：通过 `#if FEATURE_HEATING_MODE == 2` / `#elif FEATURE_HEATING_MODE == 3` 实现，模式 2 和模式 3 的代码完全不重叠，只共用宏定义。
- **上电初始状态**：`s_mode3_state = MODE3_CH0_HEAT`，即 CH0 先加热，CH1 等待。

### 4.5 引脚与总线定义

| 宏定义 | 默认值 | 说明 |
| --- | --- | --- |
| `APP_I2C_PORT` | I2C_NUM_0 | I2C 主机编号 |
| `APP_I2C_SDA_GPIO` | GPIO11 | I2C 数据线 |
| `APP_I2C_SCL_GPIO` | GPIO12 | I2C 时钟线 |
| `APP_I2C_FREQ_HZ` | 400000 | I2C 时钟频率（400kHz 高速模式） |
| `APP_PWM_GPIO_CH0` | GPIO4 | PWM 通道 0 输出引脚 |
| `APP_PWM_GPIO_CH1` | GPIO5 | PWM 通道 1 输出引脚 |
| `APP_PWM_FREQ_HZ` | 20000 | PWM 频率（20kHz，高于人耳可闻范围） |
| `APP_PWM_PERIOD_MS` | 1000.0 | 控制窗口周期（ms）：PID 输出 0~1000ms 映射为占空比 0~100% |

### 4.6 外部 ADC 参数

| 宏定义 | 默认值 | 说明 |
| --- | --- | --- |
| `APP_EXT_ADC_ADDR` | 0x48 | 外部 ADC I2C 地址（A1=GND, A0=GND） |
| `APP_EXT_ADC_CMD_VDETECT` | 0xE4 | 电源检测通道选择命令（ADC CH5） |
| `APP_EXT_ADC_CMD_NTC0` | 0xC4 | NTC0 通道选择命令（ADC CH1） |
| `APP_EXT_ADC_CMD_NTC1` | 0x94 | NTC1 通道选择命令（ADC CH2） |
| `APP_EXT_ADC_CMD_NTC2` | 0xD4 | NTC2 通道选择命令（ADC CH3） |
| `APP_EXT_ADC_CMD_NTC3` | 0xA4 | NTC3 通道选择命令（ADC CH4） |
| `APP_EXT_ADC_CMD_Press1` | 0xF4 | 压力通道 1 选择命令（ADC CH7） |
| `APP_EXT_ADC_CMD_Press2` | 0x84 | 压力通道 2 选择命令（ADC CH0） |
| `APP_ADC_VREF_V` | 3.3 | ADC 参考电压（V） |
| `APP_ADC_MAX_RAW` | 4095.0 | 12 位 ADC 满量程原始值 |

### 4.7 电源检测参数

| 宏定义 | 默认值 | 说明 |
| --- | --- | --- |
| `APP_VDET_R_TOP_OHM` | 51000.0 | 分压上臂电阻（51kΩ） |
| `APP_VDET_R_BOTTOM_OHM` | 5100.0 | 分压下臂电阻（5.1kΩ） |
| `APP_UNDERVOLTAGE_THRESHOLD_V` | 20.0 | 欠压保护阈值（V）：低于此值控制层强制关闭 PWM 输出 |

电源检测分压比为 `(51k + 5.1k) / 5.1k = 11:1`。欠压保护在 `control_task` 中每周期判定，触发时两路 PWM 均强制输出 0。

### 4.8 NTC 传感器参数

| 宏定义 | 默认值 | 说明 |
| --- | --- | --- |
| `APP_NTC_SERIES_RES_OHM` | 10000.0 | NTC 分压串联电阻（10kΩ） |
| `APP_NTC_R0_OHM` | 10000.0 | NTC 在 25℃ 时的标称阻值（10kΩ） |
| `APP_NTC_BETA` | 3950.0 | NTC Beta 系数（B25/85） |
| `APP_NTC_T0_C` | 25.0 | NTC 标定参考温度（℃） |

### 4.9 压力传感器参数

| 宏定义 | 默认值 | 说明 |
| --- | --- | --- |
| `APP_PRESSURE_VOUT_SLOPE_V_PER_KPA` | 0.0188 | DC 压力传感器 Vout-P 斜率（V/kPa） |
| `APP_PRESSURE_VOUT_OFFSET_V` | 0.2 | DC 压力传感器零点偏移电压（V） |
| `APP_PRESSURE_DC_VOUT_SCALE` | 2.0 | DC 传感器输出分压比（ADC 电压 = Vout / 2） |

### 4.10 WiFi 与 UDP 参数

| 宏定义 | 默认值 | 说明 |
| --- | --- | --- |
| `APP_WIFI_SSID` | "ESP32" | WiFi 热点 SSID |
| `APP_WIFI_PASSWORD` | "12345678" | WiFi 热点密码 |
| `APP_WIFI_MAX_RETRY` | 10 | WiFi 断线最大重连次数 |
| `APP_UDP_REMOTE_IP` | "10.92.90.124" | 上位机 UDP 远端 IP 地址 |
| `APP_UDP_REMOTE_PORT` | 6000 | 上位机 UDP 远端端口 |
| `APP_UDP_LOCAL_PORT` | 6001 | ESP32 本地 UDP 监听端口 |

### 4.11 OTA 参数

| 宏定义 | 默认值 | 说明 |
| --- | --- | --- |
| `APP_OTA_URL` | "" | OTA 固件下载 URL（空字符串时不执行 OTA） |
| `APP_OTA_SAFE_TEMP_C` | 45.0 | OTA 安全温度门禁（℃）：当前温度超过该值时拒绝 OTA |

---

## 5. I2C 与外设详解

### 5.1 I2C 总线

- SDA：GPIO11，SCL：GPIO12
- 主机模式，时钟频率 400kHz（快速模式）
- 启用内部上拉电阻
- 总线级互斥锁 `s_i2c_lock`：所有 I2C 读写操作必须持有该锁，串行化多任务并发访问
- 事务超时：100ms（`pdMS_TO_TICKS(100)`）
- 初始化幂等：`periph_i2c_init()` 可被多次调用，内部通过 `s_i2c_ready` 标志保证只初始化一次

### 5.2 外部 ADC 地址选择（ADS7924 兼容）

7 位 I2C 地址由固定前缀 `10010` + `A1 A0` 组成：

| A1 引脚 | A0 引脚 | 7-bit 地址 (二进制) | 7-bit 地址 (十六进制) |
| --- | --- | --- | --- |
| GND | GND | 1001000 | 0x48 |
| GND | VDD | 1001001 | 0x49 |
| VDD | GND | 1001010 | 0x4A |
| VDD | VDD | 1001011 | 0x4B |

工程当前使用 `0x48`（A1=GND, A0=GND）。若更换地址，请同步修改 `APP_EXT_ADC_ADDR`。

### 5.3 WF5803F 传感器地址

- 7-bit 地址：`0x6C`（ADDR0）或 `0x6D`（ADDR1）
- 工程默认：`APP_WF5803F_USE_ADDR = APP_WF5803F_ADDR0 = 0x6C`

---

## 6. 外部 ADC 数据格式

### 6.1 12 位读数格式

外部 ADC 每次返回 2 字节：

- Byte0：高 4 位为固定标识位（需掩码忽略），低 4 位为 D11~D8
- Byte1：低 8 位为 D7~D0

拼接方式：
```
raw12 = ((Byte0 & 0x0F) << 8) | Byte1
```

`periph_adc_read_raw12()` 函数执行：发送 1 字节通道选择命令 → 读取 2 字节响应 → 拼接为 12 位原始值。

### 6.2 单端输入 + 外部基准命令表

命令字节格式：`SD C2 C1 C0 PD1 PD0 X X`

- 外部基准模式要求 `PD1=0`，`PD0=1`
- `C2 C1 C0` 选择通道 0~7

| 通道 | 二进制命令 | 十六进制 | 工程用途 |
| --- | --- | --- | --- |
| CH0 | 1000 0100 | 0x84 | 压力通道 2（Press2） |
| CH1 | 1100 0100 | 0xC4 | NTC0 温度采样 |
| CH2 | 1001 0100 | 0x94 | NTC1 温度采样 |
| CH3 | 1101 0100 | 0xD4 | NTC2 温度采样 |
| CH4 | 1010 0100 | 0xA4 | NTC3 温度采样 |
| CH5 | 1110 0100 | 0xE4 | 电源电压检测（V_DETECT） |
| CH6 | 1011 0100 | 0xB4 | 预留 |
| CH7 | 1111 0100 | 0xF4 | 压力通道 1（Press1） |

### 6.3 电压与输入电压换算

12 位满量程 4095，参考电压 3.3V：

```
V_adc = raw12 / 4095 * 3.3
```

电源检测分压比：上臂 51kΩ / 下臂 5.1kΩ = 11:1

```
Vin = V_detect * (51000 + 5100) / 5100 = V_detect * 11
```

函数 `periph_adc_calc_supply_voltage()` 封装了此计算。

---

## 7. NTC 温度换算

### 7.1 物理参数

- NTC 类型：10kΩ @ 25℃，B 值 3950（B25/85）
- 分压电路：Vref(3.3V) → 串联电阻 Rs(10kΩ) → NTC → GND
- 采样点位于 Rs 与 NTC 之间，MCU 读取的是 NTC 两端电压

### 7.2 分压反推 NTC 阻值

$$R_{ntc} = R_s \cdot \frac{V_{out}}{V_{ref} - V_{out}}$$

其中 `R_s = APP_NTC_SERIES_RES_OHM = 10000Ω`，`V_ref = APP_ADC_VREF_V = 3.3V`。

### 7.3 Beta 模型温度换算

$$\frac{1}{T} = \frac{1}{T_0} + \frac{1}{B} \ln\left(\frac{R_{ntc}}{R_0}\right)$$

其中 `T_0 = 298.15K (25℃)`，`R_0 = 10000Ω`，`B = 3950`。

摄氏温度：$$T_{℃} = T_{K} - 273.15$$

### 7.4 异常保护

`ctrl_ntc_voltage_to_temp_c()` 对以下异常情况返回 `false`：
- 电压 ≤ 0V 或 ≥ Vref（分压公式分母为 0 或负数）
- 计算出的 NTC 阻值 ≤ 0Ω
- 反推出的温度倒数 ≤ 0（数值溢出）

---

## 8. WF5803F 温度/压力传感器

### 8.1 关键寄存器

| 寄存器地址 | 名称 | 位宽 | 描述 |
| --- | --- | --- | --- |
| 0x30 | CMD | 8bit | 命令寄存器，写入启动转换 |
| 0x02 | STATUS | 8bit | 状态寄存器，bit0=1 表示转换完成 |
| 0x06~0x08 | PRESS | 24bit | 压力原始数据（大端序） |
| 0x09~0x0A | TEMP | 16bit | 温度原始数据（大端序） |

### 8.2 启动命令

单次温度+压力联合测量命令：`0x0A`

### 8.3 读取流程

1. 向命令寄存器(0x30)写入 0x0A
2. 轮询状态寄存器(0x02) bit0，最多 20 次 × 2ms（总计 40ms 超时）
3. 从 0x06 连续读取 5 字节（压力 3B + 温度 2B）
4. 按大端拼接并换算为工程单位

### 8.4 数据换算公式

**压力（2bar 量程型号）：**

$$P_{kPa} = \frac{180}{0.81} \cdot \left(\frac{raw_{24}}{2^{23}} - 0.1\right) + 30$$

原始值先归一化到 `[0, 1)` 范围（除以 `2^23 = 8388608`），再代入线性模型。符号位（bit23）为 1 时做符号扩展到 32 位。

**温度：**

$$T_{℃} = \frac{raw_{16}}{256}$$

温度分辨率为 1/256 ℃（约 0.004℃）。

---

## 9. DC 电压型压力传感器

### 9.1 传感器输出模型

$$V_{out} = 0.0188 \cdot P + 0.2$$

其中 `V_out` 为传感器输出电压（V），`P` 为压力（kPa）。

### 9.2 采样链路

传感器输出电压 → 外部分压（1/2） → 外部 ADC 通道 → I2C 读取 12 位值 → 换算为电压 → 乘以分压比还原 → 代入线性模型

### 9.3 压力换算公式

$$P_{kPa} = \frac{V_{out} - 0.2}{0.0188}$$

其中还原后的传感器电压 `V_out = V_adc × APP_PRESSURE_DC_VOUT_SCALE`（默认 scale=2.0）。

`periph_pressure_dc_read()` 函数封装了完整采样链路。负压结果被钳位为 0。

### 9.4 通道选择

通过 `FEATURE_PRESSURE_DC_CH1` / `FEATURE_PRESSURE_DC_CH2` 选择通道，`pressure_config.h` 在编译期检查恰好选一个通道，并根据选择推导 `APP_PRESSURE_DC_ADC_CMD` 宏：

- CH1 → `APP_EXT_ADC_CMD_Press1` (0xF4, ADC CH7)
- CH2 → `APP_EXT_ADC_CMD_Press2` (0x84, ADC CH0)

---

## 10. PWM 输出

### 10.1 硬件配置

- 定时器：LEDC 低速定时器 0，时钟源自动选择（`LEDC_AUTO_CLK`）
- 频率：20kHz（`APP_PWM_FREQ_HZ`）
- 分辨率：10 bit（0~1023 占空比计数）
- 通道 0：GPIO4（`APP_PWM_GPIO_CH0`）
- 通道 1：GPIO5（`APP_PWM_GPIO_CH1`）
- 每通道独立使能开关（`APP_PWM_CH0_ENABLE` / `APP_PWM_CH1_ENABLE`）

### 10.2 控制模型

PID 输出单位为毫秒导通时间（0~1000ms），映射到 1000ms 控制周期：

```
占空比(%) = (on_time_ms / 1000) × 100
LEDC duty = (占空比 / 100) × 1023
```

即导通时间 0ms → 0% 占空比，1000ms → 100% 占空比（全导通）。

### 10.3 API 接口

| 函数 | 功能 |
| --- | --- |
| `periph_pwm_init()` | 初始化定时器和通道，上电默认关断 |
| `periph_pwm_set_on_time_ms_ch(ch, ms)` | 设置单通道导通时间（0~1000ms） |
| `periph_pwm_set_on_time_ms(ms)` | 设置双通道同步导通时间 |
| `periph_pwm_set_percent(%)` | 设置双通道占空比（0~100%） |
| `periph_pwm_force_off_ch(ch)` | 强制关断单通道输出 |
| `periph_pwm_force_off()` | 强制关断双通道输出 |
| `periph_pwm_get_on_time_ms()` | 读取 CH0 最后一次设置的导通时间 |
| `periph_pwm_get_percent()` | 读取 CH0 最后一次设置的占空比 |

---

## 11. 通讯协议

### 11.1 帧结构

```
| HEAD (1B) | CMD (1B) | LEN (1B) | PAYLOAD (0~N B) | CRC8 (1B) | TAIL (1B) |
|   0xDE    | cmd_id   | payload_len |    payload     |   crc8    |   0xED    |
```

- 字节序：小端（Little Endian）
- 浮点缩放：所有浮点数放大 100 倍转为整数传输
- CRC8：多项式 0x07，初始值 0x00，计算范围从 HEAD（含）到 PAYLOAD 末尾（含）
- 最大帧长度：`out_cap` 由调用方指定（当前遥测任务使用 96 字节缓冲）

### 11.2 帧组装

`comm_protocol_build_frame(cmd_id, payload, payload_len, out_frame, out_cap)`：

1. 校验输出缓冲区容量：`1 + 1 + 1 + payload_len + 1 + 1 = payload_len + 5` 字节
2. 按序写入 HEAD → CMD → LEN → PAYLOAD（memcpy）
3. 计算 CRC8（覆盖 HEAD 到 PAYLOAD 末尾）
4. 写入 CRC8 → TAIL
5. 返回完整帧长度；失败返回 0

### 11.3 CMD ID 与载荷格式

| CMD ID | 宏名称 | 描述 | 载荷格式 | 字节数 |
| --- | --- | --- | --- | --- |
| 0x01 | `COMM_CMD_NTC` | NTC 控制反馈温度 | int16 CH0 平均温度(℃×100) + (可选) int16 CH1 平均温度(℃×100) | 0/2/4 |
| 0x02 | `COMM_CMD_WF5803F` | WF5803F 温压数据 | int16 温度(℃×100) + int32 压力(kPa×100) | 6 |
| 0x03 | `COMM_CMD_VOLTAGE` | 电源电压状态 | int16 电压(V×100) + uint8 状态(0xFF=欠压, 0x01=正常) | 3 |
| 0x04 | `COMM_CMD_PID_OUT` | PID 输出值 | int32×2：PWM CH1 输出(ms×100) + PWM CH0 输出(ms×100) | 8 |
| 0x05 | `COMM_CMD_PRESSURE` | 压力数据 | int32 压力(kPa×100) | 4 |
| 0x0F | `COMM_CMD_TEXT_INFO` | 文本信息 | ASCII 字符串 | 变长 |

**CMD_NTC 通道约定**：
- CH0（载荷中第一个 int16）：PWM CH0 对应控制组（Group 1 = NTC2/NTC3）的平均反馈温度
- CH1（载荷中第二个 int16）：PWM CH1 对应控制组（Group 0 = NTC0/NTC1）的平均反馈温度
- 每通道平均规则：双 NTC 都有效取平均，单 NTC 有效取单值，都无效则该通道不写入载荷
- 两通道都无效时整个帧不发送（payload 长度为 0）

**CMD_PID_OUT 顺序约定**：
- 第一个 int32：PWM CH1 输出（ms×100）—— 对应控制组 0
- 第二个 int32：PWM CH0 输出（ms×100）—— 对应控制组 1

### 11.4 数据缩放与类型转换

| 源数据类型 | 缩放 | 传输类型 | 溢出保护 |
| --- | --- | --- | --- |
| 温度（℃） | ×100 | int16 | [-327.68, 327.67] |
| 电压（V） | ×100 | int16 | [-327.68, 327.67] |
| 压力（kPa） | ×100 | int32 | [-21474836.48, 21474836.47] |
| PID 输出（ms） | ×100 | int32 | 同上 |

转换函数 `scale100_to_i16()` 和 `scale100_to_i32()` 在溢出时返回对应类型的极值（饱和截断）。

---

## 12. 命令输入

### 12.1 输入通道

支持双通道命令输入，共用同一解析器 `comm_command_parse_line()`：

| 通道 | 任务 | 传输 | 说明 |
| --- | --- | --- | --- |
| USB 串口 | `console_command_task` | UART（stdin/stdout） | 通过 `fgets()` 非阻塞读取（20ms 轮询间隔） |
| UDP 网络 | `udp_command_task` | WiFi UDP Socket | 通过 `comm_udp_receive_line()` 阻塞接收（200ms 超时） |

### 12.2 命令解析流程

1. 接收一行文本（最大 96 字节）
2. 去除首尾空白字符
3. 转为大写（大小写不敏感匹配）
4. 按优先级匹配关键字

### 12.3 支持的命令列表

| 命令语法 | 别名 | 类型 | 参数 | 说明 |
| --- | --- | --- | --- | --- |
| `HB` | `HEARTBEAT` | 心跳 | 无 | 重置心跳超时计时器，退出安全模式 |
| `OTA` | — | OTA 触发 | 无 | 标记 OTA 请求，由 `ota_task` 异步执行 |
| `SETPOINT=xx` | `SP=xx` | 目标温度 | 浮点数（℃） | 设置控制目标温度 |
| `KP=xx` | — | PID 比例增益 | 浮点数 | 同步修改两路 PID 的 Kp 参数 |
| `KI=xx` | — | PID 积分增益 | 浮点数 | 同步修改两路 PID 的 Ki 参数（修改时清积分项） |
| `KD=xx` | — | PID 微分增益 | 浮点数 | 同步修改两路 PID 的 Kd 参数 |
| `ILIMIT=xx` | `INTEGRAL_LIMIT=xx` | 积分限幅 | 浮点数（%） | 同步修改两路 PID 的积分限幅 |

**注意事项**：
- 任何合法命令（包括心跳）都更新最后心跳时间戳 `last_heartbeat_ms`
- SETPOINT/KP/KI/KD/ILIMIT 命令对两路 PID 同步生效（无法单独控制一路）
- KI 参数修改时，两路 PID 的积分项同时清零
- 命令解析失败（未知命令）时静默忽略，不影响系统运行

---

## 13. OTA 固件更新策略

### 13.1 安全机制

- **URL 有效性检查**：`APP_OTA_URL` 为空字符串时，`sys_ota_perform_if_safe()` 直接返回 `ESP_ERR_INVALID_ARG`，不会发起网络请求
- **温度安全门禁**：当前过程温度（取两路中较高的有效值）超过 `APP_OTA_SAFE_TEMP_C`（默认 45℃）时，拒绝 OTA 以避免高温下 Flash 写入风险
- **PWM 强制关断**：进入 OTA 流程前，调用 `periph_pwm_force_off()` 关闭两路加热输出
- **固件有效标记**：启动时调用 `sys_ota_mark_app_valid()` → `esp_ota_mark_app_valid_cancel_rollback()`，告知 bootloader 当前运行固件稳定可用，取消自动回滚

### 13.2 执行流程

```
ota_task (500ms 周期)
  → 检查 s_state.ota_pending 标志
  → 置为 false 并读取当前过程温度
  → 调用 sys_ota_perform_if_safe(url, temp)
    → 校验 URL 非空
    → 校验温度 < 45℃
    → periph_pwm_force_off()
    → esp_https_ota(&ota_cfg)  // HTTPS 下载 + 写入目标 OTA 分区
    → 成功: esp_restart()  自动重启到新固件
    → 失败: 记录错误日志，继续运行
```

### 13.3 错误处理

- OTA 下载失败：仅记录日志，系统继续正常运行（PWM 恢复由下个控制周期自动接管）
- 温度过高拒绝：记录警告日志，清除 OTA 挂起标志，等待下次命令触发
- URL 为空：不执行任何操作

---

## 14. 安全保护机制汇总

| 保护机制 | 触发条件 | 动作 | 恢复条件 |
| --- | --- | --- | --- |
| 心跳失联保护 | 上位机心跳超时（默认 5s） | 目标温度降级到安全值（默认 30℃） | 收到新心跳命令 |
| 欠压保护 | 电源电压 < 20V | 两路 PWM 强制输出 0 | 电压恢复到 ≥ 20V |
| OTA 温度门禁 | 过程温度 > 45℃ | 拒绝执行 OTA | 温度降至 ≤ 45℃ 后重新触发 OTA 命令 |
| OTA 前 PWM 关断 | OTA 开始执行 | 两路 PWM 强制输出 0 | OTA 完成自动重启 |
| PID 积分限幅 | 积分项超限 | 钳位到 ±ILimit% | 积分项自然衰减到限幅内 |
| PID 输出限幅 | 输出超出 [0, 1000] ms | 钳位到边界值 | 计算输出回到范围内 |
| 上电零输出 | 上电阶段 Kp/Ki/Kd=0 | PWM 输出为 0 | 控制任务启动后加载运行参数 |
| 传感器失效保护 | NTC 测温无效 | 对应控制组 PWM 关断 | NTC 恢复有效读数 |

---

## 15. 遥测日志格式

### 15.1 USB 串口日志（每 100ms 输出一行）

```
T0=<℃> V0=<V> T1=<℃> V1=<V> T2=<℃> V2=<V> T3=<℃> V3=<V>
WF_T=<℃> P=<kPa> Psrc=<WF/DC/OFF> V=<supply_V>
PWM0=<ms> PWM1=<ms> SP0=<℃> SP1=<℃> SAFE=<0/1>
```

- `T0~T3` / `V0~V3`：四路 NTC 温度（℃）和电压（V），无效通道显示 0.00
- `WF_T` / `P`：WF5803F 温度和压力，无效时显示 NaN
- `Psrc`：压力数据来源 — `WF`（WF5803F）、`DC`（DC 电压型）、`OFF`（压力功能关闭）
- `supply_V`：电源输入电压（V）
- `PWM0/PWM1`：两路 PWM 导通时间（ms）
- `SP0/SP1`：两路 PID 当前有效设定值（℃）。模式 1 固定等于 `requested_sp`；模式 2 在 T_low/T_high 之间循环切换；模式 3 根据互锁状态在 T_low/T_high 之间交替
- `SAFE`：是否处于心跳失联安全模式（0=正常，1=安全模式）

### 15.2 UDP 上报帧

根据配置开关，每 100ms 可能发送以下帧（通过 `telemetry_send()` 组帧并调用 `comm_udp_send()`）：

1. **CMD_NTC (0x01)**：PWM 通道平均反馈温度（若任一 NTC 通道使能）
2. **CMD_WF5803F (0x02)**：WF5803F 温度与压力（若 WF5803F 使能且数据有效）
3. **CMD_PRESSURE (0x05)**：DC 压力传感器数据（若 DC 来源使能且数据有效）
4. **CMD_VOLTAGE (0x03)**：电源电压与欠压状态（若电压监测使能）
5. **CMD_PID_OUT (0x04)**：两路 PID 输出导通时间（若 PID 输出上报使能）

UDP 发送失败不阻塞主流程，由下个周期继续尝试发送。

---

## 16. 构建与烧录

### 16.1 开发环境

- 建议使用 VS Code + ESP-IDF 扩展（`espressif.esp-idf-extension`）
- ESP-IDF 版本：v5.4.3
- 目标芯片：ESP32-S3（`idf.py set-target esp32s3`）

### 16.2 构建命令

```bash
idf.py build          # 编译
idf.py flash          # 烧录（端口见 .vscode/settings.json，默认 COM11，UART 模式）
idf.py monitor        # 串口监视器（波特率 115200）
idf.py build flash monitor  # 一键编译+烧录+监视
```

### 16.3 .vscode 配置

- `settings.json`：配置 IDF 路径、目标芯片 esp32s3、烧录端口 COM11、烧录方式 UART
- `c_cpp_properties.json`：配置编译器为 `xtensa-esp32-elf-gcc`
- `launch.json`：配置 GDB 调试（OpenOCD）

### 16.4 开发容器

项目提供 `.devcontainer/` 配置，基于 `espressif/idf` Docker 镜像，可在 VS Code Remote Container 中开发。

---

## 17. 常见问题排查

### 17.1 WF5803F 无输出

- 确认 I2C 地址为 7-bit（0x6C/0x6D），而非 8-bit（左移 1 位后的值）
- 确认 `FEATURE_WF5803F_ENABLE=1` 且 `FEATURE_PRESSURE_SOURCE=1`
- 启动命令必须为 0x0A（单次温压联合测量），否则状态寄存器 bit0 不会置位
- 检查轮询超时（最多等待 40ms），若传感器响应慢可适当增加轮询次数

### 17.2 NTC 温度异常（常温显示高温或异常值）

- 优先检查 ADC 原始值与通道命令是否匹配（如 NTC0 对应命令 0xC4）
- 检查分压电路接线方向：NTC 应近地端（GND 侧），串联电阻近 Vref 侧（3.3V 侧）
- 检查 `APP_NTC_SERIES_RES_OHM` 是否与实际硬件一致（默认 10kΩ）
- 检查 NTC Beta 值是否匹配（默认 3950，B25/85）
- 确认 `FEATURE_NTC_CHx_ENABLE` 对应通道已设为 1

### 17.3 无线调试受干扰（频繁断开/重连）

- 调试阶段可关闭 `FEATURE_WIRELESS_ENABLE=0` 以避免 WiFi 重连日志干扰
- 确认 WiFi SSID 和密码与实际热点一致
- 确认上位机 IP 地址和端口配置正确
- 检查 WiFi 最大重试次数 `APP_WIFI_MAX_RETRY`（默认 10 次）

### 17.4 PID 控制异常（不加热或过热）

- 检查 `APP_PWM_CH0_ENABLE` / `APP_PWM_CH1_ENABLE` 是否正确使能对应通道
- 检查对应控制组的 NTC 通道是否已使能（如控制组 0 需要 NTC0 或 NTC1 使能）
- 确认 `APP_PID_TASK_START_KP/KI/KD` 参数已根据现场调参设置
- 确认 `APP_DEFAULT_SETPOINT_C` 目标温度设置合理
- 检查是否触发了欠压保护（`supply_voltage_v < 20V`）
- 检查是否处于心跳失联安全模式（`SAFE=1`，目标温度降级到 30℃）

### 17.5 OTA 更新失败

- 确认 `APP_OTA_URL` 已设置为有效的 HTTPS 固件下载地址
- 确认当前过程温度低于 `APP_OTA_SAFE_TEMP_C`（默认 45℃）
- 确认 WiFi 已成功连接（可通过串口日志或 UDP 通讯确认）
- 检查 OTA 分区大小是否足够容纳新固件（每个 OTA 分区 1.25MB）
- 确认 HTTPS 服务器证书可被 ESP32 信任（或使用 HTTP 进行测试）

### 17.6 I2C 通讯异常

- 检查 SDA(GPIO11)/SCL(GPIO12) 外部上拉电阻（建议 2~10kΩ）
- 确认 I2C 设备地址与代码中配置一致（7-bit 地址）
- 确认总线上无地址冲突（外部 ADC 和 WF5803F 地址不同）
- 可用逻辑分析仪或示波器检查 I2C 波形

---

## 18. 附录

### 18.1 编译期配置校验

`pressure_config.h` 在编译期对压力传感器配置进行以下校验：

- `FEATURE_PRESSURE_SOURCE` 必须为 0 或 1
- 若来源为 WF5803F（`FEATURE_PRESSURE_SOURCE=1`），则 `FEATURE_WF5803F_ENABLE` 必须为 1
- 若来源为 DC 电压型（`FEATURE_PRESSURE_SOURCE=0`），则 `FEATURE_PRESSURE_DC_CH1` 和 `FEATURE_PRESSURE_DC_CH2` 中有且仅有一个为 1

`main.c` 在编译期校验：
- `APP_NTC_FILTER_WINDOW_SIZE` 必须 ≥ 1
- `APP_NTC_SAMPLE_PERIOD_MS` 必须 ≥ 1

配置不合法时产生 `#error` 编译错误，提前暴露问题。

### 18.2 文件清单

```
esp32/
├── CMakeLists.txt              # 顶级 CMake 项目文件
├── partitions.csv              # 自定义分区表（factory + ota_0 + ota_1 + storage）
├── sdkconfig                   # ESP-IDF v5.4.3 完整 SDK 配置
├── README.md                   # 本文档
├── .devcontainer/              # VS Code 开发容器配置
│   ├── Dockerfile
│   └── devcontainer.json
├── .vscode/                    # VS Code 工作区配置
│   ├── c_cpp_properties.json
│   ├── launch.json
│   └── settings.json
└── main/                       # 应用源代码
    ├── CMakeLists.txt          # 组件注册与依赖声明
    ├── app_config.h            # 统一配置（功能开关/引脚/参数）
    ├── pressure_config.h       # 压力传感器派生配置与编译校验
    ├── main.c                  # 系统入口、任务编排、控制逻辑
    ├── periph_i2c.h/.c         # I2C 总线驱动
    ├── periph_adc.h/.c         # 外部 12 位 ADC 驱动
    ├── periph_pwm.h/.c         # 双通道 LEDC PWM 驱动
    ├── periph_wf5803f.h/.c     # WF5803F 温压传感器驱动
    ├── periph_pressure_dc.h/.c # DC 电压型压力传感器驱动
    ├── ctrl_ntc.h/.c           # NTC 温度换算（Beta 模型）
    ├── ctrl_pid.h/.c           # PID 控制器（积分分离/死区/限幅）
    ├── ctrl_failsafe.h/.c      # 心跳失联安全保护
    ├── comm_protocol.h/.c      # 帧协议封装（CRC8/载荷打包）
    ├── comm_command.h/.c       # 文本命令解析
    ├── comm_udp.h/.c           # WiFi STA + UDP Socket 管理
    ├── sys_ota.h/.c            # HTTPS OTA 固件更新
    └── CMakeLists.txt          # 组件注册与依赖声明
```
