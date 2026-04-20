#include "comm_command.h"

#include <ctype.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

static void trim(char *s) {
    // 去掉字符串首尾空白，避免用户输入格式差异影响解析。
    if (s == NULL) {
        return;
    }

    size_t len = strlen(s);
    while (len > 0 && isspace((unsigned char)s[len - 1])) {
        s[len - 1] = '\0';
        --len;
    }

    size_t start = 0;
    while (s[start] != '\0' && isspace((unsigned char)s[start])) {
        ++start;
    }
    if (start > 0) {
        memmove(s, s + start, strlen(s + start) + 1);
    }
}

static void uppercase(char *s) {
    // 转大写以实现大小写无关命令匹配。
    if (s == NULL) {
        return;
    }

    for (size_t i = 0; s[i] != '\0'; ++i) {
        s[i] = (char)toupper((unsigned char)s[i]);
    }
}

static bool parse_key_value(const char *line, const char *key, float *out_value) {
    // 解析格式：KEY=number
    if (line == NULL || key == NULL || out_value == NULL) {
        return false;
    }

    const size_t key_len = strlen(key);
    if (strncmp(line, key, key_len) != 0) {
        return false;
    }
    if (line[key_len] != '=') {
        return false;
    }

    float value = 0.0f;
    if (sscanf(line + key_len + 1, "%f", &value) != 1) {
        return false;
    }

    *out_value = value;
    return true;
}

bool comm_command_parse_line(const char *line, comm_command_t *out_cmd) {
    // 基础参数校验。
    if (line == NULL || out_cmd == NULL) {
        return false;
    }

    // 拷贝到本地缓冲并标准化（去空白+转大写）。
    char buf[96] = {0};
    strncpy(buf, line, sizeof(buf) - 1);
    trim(buf);
    uppercase(buf);

    out_cmd->type = COMM_COMMAND_NONE;
    out_cmd->value = 0.0f;

    // 心跳命令：HB / HEARTBEAT
    if (strcmp(buf, "HB") == 0 || strcmp(buf, "HEARTBEAT") == 0) {
        out_cmd->type = COMM_COMMAND_HEARTBEAT;
        return true;
    }

    // OTA 触发命令
    if (strcmp(buf, "OTA") == 0) {
        out_cmd->type = COMM_COMMAND_OTA;
        return true;
    }

    // 参数类命令解析。
    float value = 0.0f;
    if (parse_key_value(buf, "SETPOINT", &value) || parse_key_value(buf, "SP", &value)) {
        out_cmd->type = COMM_COMMAND_SETPOINT;
        out_cmd->value = value;
        return true;
    }
    if (parse_key_value(buf, "KP", &value)) {
        out_cmd->type = COMM_COMMAND_KP;
        out_cmd->value = value;
        return true;
    }
    if (parse_key_value(buf, "KI", &value)) {
        out_cmd->type = COMM_COMMAND_KI;
        out_cmd->value = value;
        return true;
    }
    if (parse_key_value(buf, "KD", &value)) {
        out_cmd->type = COMM_COMMAND_KD;
        out_cmd->value = value;
        return true;
    }
    if (parse_key_value(buf, "ILIMIT", &value) || parse_key_value(buf, "INTEGRAL_LIMIT", &value)) {
        out_cmd->type = COMM_COMMAND_ILIMIT;
        out_cmd->value = value;
        return true;
    }

    return false;
}
