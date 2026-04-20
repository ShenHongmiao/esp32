#ifndef COMM_COMMAND_H
#define COMM_COMMAND_H

#include <stdbool.h>

/*
 * 控制命令解析模块
 * 输入：一行文本命令（串口或 UDP）
 * 输出：结构化命令类型与参数值
 */

typedef enum {
    COMM_COMMAND_NONE = 0,
    COMM_COMMAND_HEARTBEAT,
    COMM_COMMAND_SETPOINT,
    COMM_COMMAND_KP,
    COMM_COMMAND_KI,
    COMM_COMMAND_KD,
    COMM_COMMAND_ILIMIT,
    COMM_COMMAND_OTA,
} comm_command_type_t;

// 统一命令结果结构。
typedef struct {
    comm_command_type_t type;
    float value;
} comm_command_t;

// 解析一行文本命令，成功返回 true。
bool comm_command_parse_line(const char *line, comm_command_t *out_cmd);

#endif  // COMM_COMMAND_H
