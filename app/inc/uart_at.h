#ifndef UART_AT_H
#define UART_AT_H

#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 UART AT通信接口
 用于与ML307R 4G模块通信的AT命令通道
============================================================================*/

/**
 * 初始化UART2接口
 * 配置115200 8N1，使能RX中断
 */
void uart_at_init(void);

/**
 * 发送AT命令并等待期望的响应
 * 阻塞等待直到收到expected_ok字符串或超时
 * @param cmd AT命令字符串
 * @param expected_ok 期望的响应关键字
 * @param timeout_ms 超时时间(毫秒)
 * @param response 响应缓冲区，用于输出
 * @param resp_len 响应缓冲区长度
 * @return 0=成功, -1=超时, -2=错误
 */
int at_send_command(const char *cmd, const char *expected_ok,
                    uint32_t timeout_ms, char *response, int resp_len);

/**
 * 发送原始数据(不等响应)
 * 用于XMODEM OTA传输
 * @param data 数据指针
 * @param len 数据长度
 */
void at_send_raw(const uint8_t *data, uint16_t len);

/**
 * 注册URC( unsolicited result code)回调
 * 当收到包含keyword的消息时自动调用callback
 * @param keyword 关键字
 * @param callback 回调函数
 */
typedef void (*urc_callback_t)(const char *line);
void at_register_urc(const char *keyword, urc_callback_t callback);

/**
 * 清除RX缓冲区
 */
void at_flush_rx(void);

#endif // UART_AT_H
