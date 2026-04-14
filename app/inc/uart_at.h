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
 UART ATͨ�Žӿ�
 ������ML307R 4Gģ��ͨ�ŵ�AT����ͨ��
============================================================================*/

/**
 * ��ʼ��UART2�ӿ�
 * ����115200 8N1��ʹ��RX�ж�
 */
void uart_at_init(void);

/**
 * ����AT����ȴ���������Ӧ
 * �����ȴ�ֱ���յ�expected_ok�ַ�����ʱ
 * @param cmd AT�����ַ���
 * @param expected_ok ��������Ӧ�ؼ���
 * @param timeout_ms ��ʱʱ��(����)
 * @param response ��Ӧ���������������
 * @param resp_len ��Ӧ����������
 * @return 0=�ɹ�, -1=��ʱ, -2=����
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
 * ����ԭʼ����(������Ӧ)
 * ����XMODEM OTA����
 * @param data ����ָ��
 * @param len ���ݳ���
 */
void at_send_raw(const uint8_t *data, uint16_t len);

/**
 * ע��URC( unsolicited result code)�ص�
 * ���յ�����keyword����Ϣʱ�Զ�����callback
 * @param keyword �ؼ���
 * @param callback �ص�����
 */
typedef void (*urc_callback_t)(const char *line);
void at_register_urc(const char *keyword, urc_callback_t callback);

/**
 * ���RX������
 */
void at_flush_rx(void);

/**
 * ����UART���������ݣ����������е��ã�
 * ���������յ��е������URC�ص�
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
