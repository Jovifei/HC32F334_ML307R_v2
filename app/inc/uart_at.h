#ifndef UART_AT_H
#define UART_AT_H

#include <stdint.h>
#include <stdbool.h>

// AT命令状态（用于非阻塞API）
typedef enum {
    AT_NB_IDLE = 0,     // 空闲
    AT_NB_WAITING = 1,   // 等待响应中
    AT_NB_OK = 2,        // 收到OK
    AT_NB_ERR = -1,      // 收到ERROR或超时
} at_nb_state_t;

/*============================================================================
 UART AT通信接口
 专供ML307R 4G模组通信的AT命令通道
============================================================================*/

/**
 * 初始化UART2接口
 * 波特率115200 8N1，使能RX中断
 */
void uart_at_init(void);

/**
 * 发送AT命令并等待完整响应
 * 阻塞等待直到收到expected_ok字符串或超时
 * @param cmd AT命令字符串
 * @param expected_ok 期望的响应关键字
 * @param timeout_ms 超时时间（毫秒）
 * @param response 响应接收缓冲区（可为NULL）
 * @param resp_len 响应缓冲区长度
 * @return 0=成功, -1=超时, -2=错误
 */
int at_send_command(const char *cmd, const char *expected_ok,
                    uint32_t timeout_ms, char *response, int resp_len);

/**
 * 获取最近一次 AT 命令解析到的错误码（如 +CME ERROR: <err> / +CMS ERROR: <err>）
 * @return >=0: 错误码；-1: 未记录到错误码
 */
int at_get_last_error_code(void);

/**
 * 获取最近一次 AT 命令解析到的错误文本行（如 "+CME ERROR: 50"）
 * @return 指向内部静态缓冲区的指针；若无则返回空字符串
 */
const char *at_get_last_error_line(void);

/**
 * 发送原始数据（不等响应）
 * 适用于XMODEM OTA等场景
 * @param data 数据指针
 * @param len 数据长度
 */
void at_send_raw(const uint8_t *data, uint16_t len);

/**
 * 注册URC（unsolicited result code）回调
 * 收到包含keyword的消息时自动调用callback
 * @param keyword 关键字
 * @param callback 回调函数
 */
typedef void (*urc_callback_t)(const char *line);
void at_register_urc(const char *keyword, urc_callback_t callback);

/**
 * 清空RX缓冲区
 */
void at_flush_rx(void);

/**
 * 处理UART接收到的数据（在主循环中调用）
 * 解析AT行并触发URC回调
 */
void uart_at_process(void);

/**
 * 启动非阻塞AT命令发送
 * @param cmd AT命令字符串
 * @param timeout_ms 超时时间(毫秒)
 * @return 0=成功启动, -1=失败
 */
int at_command_start(const char *cmd, uint32_t timeout_ms);

/**
 * 检查AT命令状态（非阻塞）
 * @return AT_IDLE=空闲, AT_WAITING=等待中, 1=OK, -1=ERROR/超时
 */
int at_command_check(void);

/**
 * 读取原始响应数据（不解析，按字节读取）
 * @param buf 目标缓冲区
 * @param len 缓冲区大小
 * @return 实际读取的字节数
 */
int at_read_response(char *buf, int len);

/**
 * 检查RX缓冲区是否包含指定字符串（用于检测>提示符等）
 * @param str 要查找的字符串
 * @return 1=包含, 0=不包含
 */
int at_rx_contains(const char *str);

/**
 * 检查是否收到 > 提示符（自动清除标志）
 * 由 parse_rx_lines_budget 在检测到 > 行首字符时设置
 * 用于证书写入两步协议中等待 > 的场景
 * @return true=收到, false=未收到
 */
bool at_got_prompt(void);

/**
 * 检查最近一次 AT 命令的 OK/ERROR 结果（自动清除标志）
 * 用于证书写入完成后检测 OK 的场景（不依赖 at_command_start/check 状态机）
 * @return 1=OK, -1=ERROR/CME ERROR, 0=尚未收到
 */
int at_check_last_result(void);

#endif // UART_AT_H
