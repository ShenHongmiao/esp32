#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "driver/gpio.h"

/*
 * 工程统一配置文件
 * 说明：
 * 1.本文件仅放“可调参数/引脚定义/功能开关”，不放业务逻辑。
 * 2.业务代码尽量只依赖这里的宏，便于后续调参与迁移硬件。
 */

// ======================== Feature Switches ========================
// 功能总开关：采集与上传使用同一组使能，避免“采了但没发”或“发了无数据”。
#define FEATURE_NTC_CH0_ENABLE            1 // NTC 通道 0 使能，ADC_CH1，下侧分压测温
#define FEATURE_NTC_CH1_ENABLE            0 // NTC 通道 1 使能, ADC_CH2
#define FEATURE_NTC_CH2_ENABLE            0 // NTC 通道 2 使能, ADC_CH3
#define FEATURE_NTC_CH3_ENABLE            0 // NTC 通道 3 使能
#define FEATURE_WF5803F_ENABLE            0 // WF5803F 功能使能
#define FEATURE_VOLTAGE_MONITOR_ENABLE    1 // 监测电压并上报，必要时触发保护
#define FEATURE_PID_OUT_ENABLE            1 // 发送 PID 输出值
#define FEATURE_UPLOAD_ENABLE             0 // 0: 仅串口日志；1: 同时 UDP 上传
// 无线总开关：0 时完全关闭 WiFi/UDP（不启动 WiFi，不创建 UDP 任务，不发 UDP 包）。
#define FEATURE_WIRELESS_ENABLE           0 // 有线调试建议设为 0，避免无线重连导致调试干扰
// 心跳失联保护开关：1=启用超时降级到安全温度；0=忽略心跳超时。
#define FEATURE_HEARTBEAT_FAILSAFE_ENABLE 0

// ======================== Control Loop ============================
// 控制任务周期（毫秒）：越小响应越快，但 CPU 占用和噪声敏感度越高。也是其他任务（如采样、通信）的时间基准。
#define APP_CONTROL_PERIOD_MS             20
// NTC 后台采样周期（毫秒）：默认 2ms=500Hz，可按现场噪声与总线负载调到 2~10ms（约 500~100Hz）。
#define APP_NTC_SAMPLE_PERIOD_MS          2
// NTC 电压滑动窗口长度：后台每来一个样本滚动更新均值。
#define APP_NTC_FILTER_WINDOW_SIZE        10
// 遥测上报周期（毫秒）：用于串口日志和 UDP 上传节流。
#define APP_TELEMETRY_PERIOD_MS           100
// 心跳超时（毫秒）：超过该时间未收到上位机命令则进入安全模式。
#define APP_HEARTBEAT_TIMEOUT_MS          5000
// 安全模式目标温度（摄氏度）：失联时回退到该设定。
#define APP_SAFE_SETPOINT_C               30.0f

// PID 默认参数（上电初始值，可被运行时命令覆盖）。
#define APP_DEFAULT_SETPOINT_C            50.0f // 初始目标温度（℃）
#define APP_PID_KP_DEFAULT                0.0f  // 固定默认值：上电阶段 Kp 必须为 0（勿改）
#define APP_PID_KI_DEFAULT                0.0f  // 固定默认值：上电阶段 Ki 必须为 0（勿改）
#define APP_PID_KD_DEFAULT                0.0f  // 固定默认值：上电阶段 Kd 必须为 0（勿改）
// PID 输出限幅（ms）：控制量映射到 0~1000ms 导通时间，超过该范围会被 clamp 限幅。
#define APP_PID_OUTPUT_MIN_MS             0.0f    // PID 输出下限：0ms 导通
#define APP_PID_OUTPUT_MAX_MS             1000.0f // PID 输出上限：1000ms 导通（1s 全导通）
// 控制任务启动后加载的运行 PID 参数（可根据现场调参修改）比例增益230等幅振荡点，周期9s。
#define APP_PID_TASK_START_KP             85.5f
#define APP_PID_TASK_START_KI             21.25f
#define APP_PID_TASK_START_KD             0.0f

#define APP_PID_ILIMIT_DEFAULT            50.0f // 积分限幅（%），防止积分风暴
#define APP_PID_DEADBAND_C                0.2f  // 死区（℃）：误差落入该范围时保持当前输出
#define APP_PID_ENABLE_INTEGRAL_SEPARATION 1    // 1=启用积分分离；0=传统积分
#define APP_PID_INTEGRAL_SEPARATION_THRESHOLD_C 5.0f // 仅在误差绝对值不大于该值时累积积分

// ======================== I2C and Peripheral Pins ================
// I2C 总线定义：对应硬件图纸 IO11/IO12，速率 400kHz。
#define APP_I2C_PORT                      I2C_NUM_0   // I2C 主机编号，ESP32 只有一个 I2C 控制器，固定为 0。
#define APP_I2C_SDA_GPIO                  GPIO_NUM_11 
#define APP_I2C_SCL_GPIO                  GPIO_NUM_12
#define APP_I2C_FREQ_HZ                   400000      // I2C 时钟频率（Hz），400kHz 属于高速模式，适合大多数传感器。

// PWM 输出定义：双通道同占空比输出到两路 NMOS 栅极。
#define APP_PWM_CH0_ENABLE               0 // PWM CH0 开关：1=使能输出，0=禁用
#define APP_PWM_CH1_ENABLE               1 // PWM CH1 开关：1=使能输出，0=禁用
#define APP_PWM_GPIO_CH0                  GPIO_NUM_4
#define APP_PWM_GPIO_CH1                  GPIO_NUM_5
#define APP_PWM_FREQ_HZ                   20000 // PWM 频率（Hz），20kHz 以上通常不可闻，适合加热控制。
#define APP_PWM_PERIOD_MS                 1000.0f // 控制窗口：PID 输出 0~1000ms 映射为一个 1s 周期导通时长

// ======================== External ADC ============================
// 外部 ADC 地址与各通道命令字（来自需求文档定义）。
#define APP_EXT_ADC_ADDR                  0x48
#define APP_EXT_ADC_CMD_VDETECT           0x84
#define APP_EXT_ADC_CMD_NTC0              0xC4
#define APP_EXT_ADC_CMD_NTC1              0x94
#define APP_EXT_ADC_CMD_NTC2              0xD4
#define APP_EXT_ADC_CMD_NTC3              0xA4

// ADC 基础参数：12bit 满量程 4095，参考电压 3.3V。
#define APP_ADC_VREF_V                    3.3f
#define APP_ADC_MAX_RAW                   4095.0f

// 电源检测分压参数：上臂 51k，下臂 5.1k；用于反推实际输入电压。
#define APP_VDET_R_TOP_OHM                51000.0f
#define APP_VDET_R_BOTTOM_OHM             5100.0f
// 欠压阈值（V）：低于该值时控制层可触发降载/关断保护。
#define APP_UNDERVOLTAGE_THRESHOLD_V      20.0f

// ======================== NTC Parameters ==========================
// NTC 参数：10k 3950，串联电阻 10k，25℃ 标定。
#define APP_NTC_SERIES_RES_OHM            10000.0f // NTC 分压串联电阻，10k 欧姆
#define APP_NTC_R0_OHM                    10000.0f // NTC 25℃ 时阻值，10k 欧姆
#define APP_NTC_BETA                      3950.0f
#define APP_NTC_T0_C                      25.0f

// ======================== WF5803F ================================
// WF5803F 支持两个地址，当前工程默认使用 ADDR0。
#define APP_WF5803F_ADDR0                 0x6C
#define APP_WF5803F_ADDR1                 0x6D
#define APP_WF5803F_USE_ADDR              APP_WF5803F_ADDR0

// ======================== WiFi / UDP =============================
// WiFi 与 UDP 参数：联调时请按现场网络修改。
#define APP_WIFI_SSID                     "ESP32-AP"
#define APP_WIFI_PASSWORD                 "12345678"
#define APP_WIFI_MAX_RETRY                10

#define APP_UDP_REMOTE_IP                 "192.168.137.1"
#define APP_UDP_REMOTE_PORT               6000
#define APP_UDP_LOCAL_PORT                6001

// ======================== OTA ====================================
// OTA URL 为空时不会执行下载；温度门禁用于避免高温写闪存风险。
#define APP_OTA_URL                       ""
#define APP_OTA_SAFE_TEMP_C               45.0f

#endif  // APP_CONFIG_H
