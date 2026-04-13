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

#endif // UART_AT_H
